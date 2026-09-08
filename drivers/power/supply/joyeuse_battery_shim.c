// SPDX-License-Identifier: GPL-2.0
/*
 * joyeuse_battery_shim: synthesize POWER_SUPPLY_PROP_STATUS for Xiaomi
 * joyeuse (PM6150) under mainline. qcom_qg exposes no STATUS, and no
 * charger driver is bound to PM6150's SMB5 peripherals, so UPower sees
 * the battery as state=unknown / icon=missing.
 *
 * This shim registers a new "battery" power_supply that forwards every
 * property from qcom_qg and computes STATUS from
 *   /sys/class/typec/port0/power_role (sink = cable plugged in)
 * with qcom_qg/current_now as the unplugged Discharging/Charging signal.
 *
 * v0.2 adds an OCV-based CAPACITY correction layer. qcom_qg's coulomb
 * counter can lock at low values (1-3%) after deep-discharge + boot
 * cycles, and its VOLTAGE_OCV property is a stale SDAM scratchpad (only
 * updated by Qualcomm's proprietary QG userspace daemon, which mainline
 * does not run). Instead we compute a live OCV estimate from
 *   OCV ≈ V_terminal + I_load * R_int
 * using the OEM joyeuse 5020 mAh ATL battery profile's
 * `qcom,pc-temp-z1-lut` for R_int, then map that OCV through the OEM's
 * own `qcom,pc-temp-v1-lut` SoC-OCV-temperature curve. Correction is
 * one-way (bumps up only), heavily gated (quiet current, settled
 * temperature, plug holdoff), and rate-limited so the meter never
 * jumps. Falls through to raw qg whenever any gating condition fails.
 *
 * Capacity pipeline: raw qcom_qg CAPACITY is clamped to [0,100], optionally
 * adjusted by the OCV-based recovery layer (above), then passed through an
 * EMA smoother (default tau 15 s) and a monotonicity clamp that prevents the
 * reported value from decreasing while charging or full. Low rail (corrected
 * <= 15 %) bypasses both smoother and clamp so UPower's low-battery warnings
 * fire on time. High rail (status == Full or raw >= 99) ensures the meter
 * reaches 100 % without delay. All capacity-pipeline state lives under
 * cap_lock. A panic param shim_passthrough forwards raw qcom_qg unchanged
 * for field debugging.
 */

#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>

#define QG_NAME        "qcom_qg"
#define TCPM_PREFIX    "tcpm-source-psy-"
#define TYPEC_ROLE_F   "/sys/class/typec/port0/power_role"
#define CHG_THRESH_UA  50000
#define REFRESH_MS     3000

static int qg_current_sign = -1;
module_param(qg_current_sign, int, 0644);
MODULE_PARM_DESC(qg_current_sign,
	"Multiplier applied to qcom_qg CURRENT_NOW to get kernel-convention "
	"sign (positive=charging). -1 for Qualcomm raw convention (default), "
	"+1 if upstream qcom_qg flips the sign itself.");

/* OCV correction tunables (Expert A design). */
static int ocv_correct_enabled = 1;
module_param(ocv_correct_enabled, int, 0644);
MODULE_PARM_DESC(ocv_correct_enabled,
	"1: enable OCV-based capacity correction (default). 0: forward raw qg.");

static int ocv_i_quiet_ua = 300000;
module_param(ocv_i_quiet_ua, int, 0644);
MODULE_PARM_DESC(ocv_i_quiet_ua,
	"OCV is only trusted when |current_now| < this (default 300000 uA).");

static int ocv_disagree_pp = 15;
module_param(ocv_disagree_pp, int, 0644);
MODULE_PARM_DESC(ocv_disagree_pp,
	"raw vs OCV-derived gap (pp) below which correction is skipped (default 15).");

static int ocv_max_step_pp = 3;
module_param(ocv_max_step_pp, int, 0644);
MODULE_PARM_DESC(ocv_max_step_pp,
	"max percentage points the corrected value can advance per query (default 3).");

static int ocv_temp_min_dc = 50;
module_param(ocv_temp_min_dc, int, 0644);
MODULE_PARM_DESC(ocv_temp_min_dc,
	"battery temp (deci-degC) below which correction is disabled (default 50 = 5.0 C).");

static int ocv_temp_max_dc = 500;
module_param(ocv_temp_max_dc, int, 0644);
MODULE_PARM_DESC(ocv_temp_max_dc,
	"battery temp (deci-degC) above which correction is disabled (default 500 = 50.0 C).");

static int ocv_plug_holdoff_ms = 30000;
module_param(ocv_plug_holdoff_ms, int, 0644);
MODULE_PARM_DESC(ocv_plug_holdoff_ms,
	"window after a plug/unplug edge during which correction is suppressed (default 30000 ms).");

static int cap_smooth_enabled = 1;
module_param(cap_smooth_enabled, int, 0644);
MODULE_PARM_DESC(cap_smooth_enabled,
	"1: smooth POWER_SUPPLY_PROP_CAPACITY via EMA on the OCV-corrected value (default). "
	"0: forward OCV-corrected value unsmoothed.");

static int cap_smooth_tau_ms = 15000;
module_param(cap_smooth_tau_ms, int, 0644);
MODULE_PARM_DESC(cap_smooth_tau_ms,
	"EMA time constant in milliseconds (default 15000). Floored at 100 internally.");

static int cap_mono_enabled = 1;
module_param(cap_mono_enabled, int, 0644);
MODULE_PARM_DESC(cap_mono_enabled,
	"1: while charging or full, reported capacity never decreases (default). 0: no clamp.");

static int cap_mono_release_ms = 10000;
module_param(cap_mono_release_ms, int, 0644);
MODULE_PARM_DESC(cap_mono_release_ms,
	"Hold the monotonicity floor this many ms after charging stops before releasing "
	"(default 10000). Rides out brief BC1.2 / PD renegotiation dips.");

static int cap_debug = 0;
module_param(cap_debug, int, 0644);
MODULE_PARM_DESC(cap_debug,
	"1: pr_info_ratelimited every CAPACITY read with raw/corrected/smoothed/floor/result.");

static int shim_passthrough = 0;
module_param(shim_passthrough, int, 0644);
MODULE_PARM_DESC(shim_passthrough,
	"1: panic switch. Forward qcom_qg CAPACITY untouched (no OCV correction, no smoothing, "
	"no monotonicity clamp). STATUS synthesis is unaffected.");

static struct power_supply *shim_psy;
static bool shim_published;
static DEFINE_MUTEX(publish_lock);

/*
 * ATL 5020 mAh joyeuse OCV-vs-SoC profile, transcribed from the OEM
 * downstream `qcom,pc-temp-v1-lut` in atl-5020.dtsi. 56 SoC rows × 5
 * temperature columns (0, 10, 25, 40, 50 °C). OCV stored in mV
 * (rounded from the dtsi's 0.1 mV units). Higher-fidelity than any
 * generic Li-ion textbook curve — generic curves over-report by up to
 * 25 pp at 3.80 V on this cell.
 */
#define ATL5020_SOC_ROWS   56
#define ATL5020_TEMP_COLS  5

static const u16 atl5020_soc_pct[ATL5020_SOC_ROWS] = {
	100, 98, 96, 94, 92,
	 90, 88, 86, 84, 82,
	 80, 78, 76, 74, 72,
	 70, 68, 66, 64, 62,
	 60, 58, 56, 54, 52,
	 50, 48, 46, 44, 42,
	 40, 38, 36, 34, 32,
	 30, 28, 26, 24, 22,
	 20, 18, 16, 14, 12,
	 10,  9,  8,  7,  6,
	  5,  4,  3,  2,  1,
	  0,
};

static const s16 atl5020_temp_dc[ATL5020_TEMP_COLS] = { 0, 100, 250, 400, 500 };

static const u16 atl5020_ocv_mv[ATL5020_SOC_ROWS * ATL5020_TEMP_COLS] = {
	/* 100% */ 4415, 4427, 4434, 4437, 4438,
	/*  98% */ 4392, 4403, 4410, 4409, 4409,
	/*  96% */ 4369, 4379, 4387, 4387, 4386,
	/*  94% */ 4345, 4357, 4365, 4365, 4365,
	/*  92% */ 4321, 4335, 4342, 4343, 4343,
	/*  90% */ 4298, 4311, 4320, 4321, 4321,
	/*  88% */ 4274, 4288, 4296, 4298, 4298,
	/*  86% */ 4250, 4263, 4272, 4274, 4274,
	/*  84% */ 4226, 4239, 4248, 4250, 4250,
	/*  82% */ 4203, 4215, 4223, 4226, 4226,
	/*  80% */ 4181, 4191, 4199, 4202, 4202,
	/*  78% */ 4159, 4168, 4175, 4178, 4178,
	/*  76% */ 4137, 4145, 4152, 4154, 4154,
	/*  74% */ 4115, 4123, 4130, 4132, 4131,
	/*  72% */ 4094, 4102, 4108, 4109, 4109,
	/*  70% */ 4074, 4081, 4086, 4088, 4088,
	/*  68% */ 4054, 4061, 4066, 4067, 4067,
	/*  66% */ 4035, 4042, 4046, 4047, 4047,
	/*  64% */ 4015, 4024, 4027, 4029, 4028,
	/*  62% */ 3998, 4007, 4011, 4012, 4011,
	/*  60% */ 3983, 3990, 3993, 3994, 3994,
	/*  58% */ 3968, 3973, 3971, 3972, 3972,
	/*  56% */ 3953, 3955, 3945, 3945, 3945,
	/*  54% */ 3937, 3930, 3924, 3923, 3923,
	/*  52% */ 3920, 3901, 3907, 3907, 3908,
	/*  50% */ 3900, 3883, 3893, 3894, 3895,
	/*  48% */ 3879, 3874, 3881, 3881, 3882,
	/*  46% */ 3865, 3867, 3872, 3870, 3871,
	/*  44% */ 3856, 3859, 3862, 3860, 3860,
	/*  42% */ 3849, 3852, 3853, 3851, 3850,
	/*  40% */ 3844, 3845, 3845, 3842, 3841,
	/*  38% */ 3839, 3838, 3838, 3834, 3833,
	/*  36% */ 3835, 3831, 3831, 3827, 3826,
	/*  34% */ 3831, 3826, 3825, 3820, 3819,
	/*  32% */ 3827, 3821, 3820, 3814, 3812,
	/*  30% */ 3824, 3817, 3815, 3809, 3806,
	/*  28% */ 3821, 3814, 3810, 3803, 3799,
	/*  26% */ 3817, 3810, 3806, 3797, 3792,
	/*  24% */ 3812, 3804, 3799, 3789, 3784,
	/*  22% */ 3805, 3796, 3790, 3779, 3773,
	/*  20% */ 3796, 3786, 3780, 3769, 3762,
	/*  18% */ 3783, 3775, 3768, 3757, 3750,
	/*  16% */ 3769, 3761, 3755, 3745, 3737,
	/*  14% */ 3753, 3743, 3738, 3728, 3720,
	/*  12% */ 3736, 3727, 3722, 3714, 3706,
	/*  10% */ 3723, 3715, 3711, 3704, 3698,
	/*   9% */ 3720, 3712, 3708, 3701, 3695,
	/*   8% */ 3716, 3709, 3705, 3699, 3693,
	/*   7% */ 3712, 3707, 3704, 3697, 3691,
	/*   6% */ 3709, 3703, 3699, 3692, 3684,
	/*   5% */ 3702, 3686, 3678, 3667, 3657,
	/*   4% */ 3673, 3642, 3635, 3623, 3613,
	/*   3% */ 3621, 3583, 3577, 3565, 3555,
	/*   2% */ 3552, 3502, 3497, 3486, 3476,
	/*   1% */ 3436, 3363, 3373, 3362, 3351,
	/*   0% */ 3000, 3000, 3000, 3000, 3000,
};

/*
 * DC internal resistance from the OEM `qcom,pc-temp-z1-lut`, taken at
 * the 50% SoC row (mid-table representative). Z1 in the dtsi is in
 * 0.01 mΩ units; here we keep mΩ. Worst-case error from using a single
 * SoC row vs the full polynomial is roughly ±3 mΩ, which translates to
 * ≤2 mV of OCV correction at 1 A load — negligible outside the deep
 * knee.
 */
static const u16 atl5020_rint_mohm[ATL5020_TEMP_COLS] = {
	127, 121, 117, 114, 114,
};

/*
 * Integer linear interpolation. Rounds half-away-from-zero. Operates
 * in s64 so the product never overflows for our worst-case inputs
 * (|y_delta| ≤ 1000, |x_delta| ≤ 5000).
 */
static inline s32 lerp_s32(s32 x, s32 x0, s32 x1, s32 y0, s32 y1)
{
	s64 num = (s64)(y1 - y0) * (s64)(x - x0);
	s32 den = x1 - x0;

	if ((num ^ den) >= 0)
		return y0 + (s32)((num + den / 2) / den);
	return y0 + (s32)((num - den / 2) / den);
}

/*
 * Bilinear OCV→SoC against the ATL5020 v1 table. Clamps inputs to
 * table extents. Returns SoC in whole percent (0..100). Worst-case
 * accuracy: ±3 pp in the plateau region between 60-30% SoC where
 * dOCV/dSoC is ~0.4 mV/%; well under 1 pp at the knees.
 */
static int atl5020_ocv_to_soc(u32 ocv_mv, int temp_dc)
{
	int t_lo_idx, t_hi_idx, t_lo, t_hi;
	s32 soc_at_col[2];
	s32 soc_pcq;
	int c;

	if (ocv_mv > 4438)
		ocv_mv = 4438;
	if (ocv_mv < 3000)
		ocv_mv = 3000;
	if (temp_dc < atl5020_temp_dc[0])
		temp_dc = atl5020_temp_dc[0];
	if (temp_dc > atl5020_temp_dc[ATL5020_TEMP_COLS - 1])
		temp_dc = atl5020_temp_dc[ATL5020_TEMP_COLS - 1];

	t_hi_idx = 1;
	while (t_hi_idx < ATL5020_TEMP_COLS - 1 &&
	       atl5020_temp_dc[t_hi_idx] < temp_dc)
		t_hi_idx++;
	t_lo_idx = t_hi_idx - 1;
	t_lo = atl5020_temp_dc[t_lo_idx];
	t_hi = atl5020_temp_dc[t_hi_idx];

	for (c = 0; c < 2; c++) {
		int col = (c == 0) ? t_lo_idx : t_hi_idx;
		int r;
		s32 v_hi, v_lo;
		s32 soc_hi_cq, soc_lo_cq;

		if ((s32)ocv_mv >=
		    (s32)atl5020_ocv_mv[0 * ATL5020_TEMP_COLS + col]) {
			soc_at_col[c] = 10000;
			continue;
		}
		if ((s32)ocv_mv <=
		    (s32)atl5020_ocv_mv[(ATL5020_SOC_ROWS - 1) *
					ATL5020_TEMP_COLS + col]) {
			soc_at_col[c] = 0;
			continue;
		}

		soc_at_col[c] = 0;
		for (r = 0; r < ATL5020_SOC_ROWS - 1; r++) {
			v_hi = atl5020_ocv_mv[r * ATL5020_TEMP_COLS + col];
			v_lo = atl5020_ocv_mv[(r + 1) * ATL5020_TEMP_COLS + col];

			if (v_lo > v_hi) {
				s32 tmp = v_lo; v_lo = v_hi; v_hi = tmp;
			}

			if ((s32)ocv_mv <= v_hi && (s32)ocv_mv >= v_lo) {
				soc_hi_cq = (s32)atl5020_soc_pct[r] * 100;
				soc_lo_cq = (s32)atl5020_soc_pct[r + 1] * 100;
				if (v_hi == v_lo)
					soc_at_col[c] = soc_hi_cq;
				else
					soc_at_col[c] = lerp_s32(
						(s32)ocv_mv, v_lo, v_hi,
						soc_lo_cq, soc_hi_cq);
				break;
			}
		}
	}

	if (t_hi == t_lo)
		soc_pcq = soc_at_col[0];
	else
		soc_pcq = lerp_s32(temp_dc, t_lo, t_hi,
				   soc_at_col[0], soc_at_col[1]);

	if (soc_pcq < 0)
		soc_pcq = 0;
	if (soc_pcq > 10000)
		soc_pcq = 10000;
	return (int)((soc_pcq + 50) / 100);
}

/*
 * Compute a live OCV estimate from terminal voltage and load current,
 * replacing the stale SDAM voltage_ocv. Discharge convention: positive
 * i_ua_qg_raw means current flowing OUT of the battery; in that case
 * V_terminal < OCV, so OCV = V + I·R_int. Charging negates I and OCV
 * sits below V_terminal — same formula with signed I.
 */
static u32 compute_live_ocv(u32 v_now_uv, s32 i_ua_qg_raw, int temp_dc)
{
	int t_lo_idx, t_hi_idx, t_lo, t_hi;
	s32 r_lo, r_hi, r_mohm;
	s64 drop_uv, ocv_uv;

	if (temp_dc < atl5020_temp_dc[0])
		temp_dc = atl5020_temp_dc[0];
	if (temp_dc > atl5020_temp_dc[ATL5020_TEMP_COLS - 1])
		temp_dc = atl5020_temp_dc[ATL5020_TEMP_COLS - 1];

	t_hi_idx = 1;
	while (t_hi_idx < ATL5020_TEMP_COLS - 1 &&
	       atl5020_temp_dc[t_hi_idx] < temp_dc)
		t_hi_idx++;
	t_lo_idx = t_hi_idx - 1;
	t_lo = atl5020_temp_dc[t_lo_idx];
	t_hi = atl5020_temp_dc[t_hi_idx];

	r_lo = (s32)atl5020_rint_mohm[t_lo_idx];
	r_hi = (s32)atl5020_rint_mohm[t_hi_idx];
	r_mohm = (t_hi == t_lo) ? r_lo
				: lerp_s32(temp_dc, t_lo, t_hi, r_lo, r_hi);

	drop_uv = ((s64)i_ua_qg_raw * (s64)r_mohm) / 1000;
	ocv_uv = (s64)v_now_uv + drop_uv;

	if (ocv_uv < 3000000)
		ocv_uv = 3000000;
	if (ocv_uv > 4438000)
		ocv_uv = 4438000;
	return (u32)((ocv_uv + 500) / 1000);
}

/*
 * Capacity-pipeline state. cap_lock guards every field below. The rate-limit
 * means the corrected value can move toward the true OCV-derived SoC by at
 * most ocv_max_step_pp per query (typical ~3 pp every 3 s refresh tick).
 * INT_MIN sentinels mark "uninitialized" so a legitimate value of 0 or -1
 * is never confused with the unseeded state.
 */
static DEFINE_MUTEX(cap_lock);

/* OCV-correction state. ocv_last_corrected == INT_MIN means "first call,
 * seed from raw". */
static int ocv_last_corrected = INT_MIN;
static bool ocv_correction_active;
static ktime_t ocv_last_plug_edge;
static bool ocv_prev_plugged;

/* EMA smoother state. smoothed_centi_pp holds the smoothed capacity in
 * centi-percentage-points; INT_MIN means "seed on next call". */
static s32 smoothed_centi_pp = INT_MIN;
static ktime_t smooth_last_ktime;

/* Monotonicity-clamp state. clamp_floor == INT_MIN means no floor is in
 * effect; mono_release_deadline == KTIME_MAX means no pending release. */
static int clamp_floor = INT_MIN;
static bool mono_prev_engaged;
static ktime_t mono_release_deadline = KTIME_MAX;

/* shim_passthrough edge tracking, for re-seeding the pipeline on the
 * passthrough -> normal transition. */
static bool prev_passthrough;

/* Last value handed back from POWER_SUPPLY_PROP_CAPACITY (debug only). */
static int last_reported = INT_MIN;

/*
 * Apply OCV-based capacity correction to a raw qg reading. Bumps the
 * value UP only — never decreases — and only when all gates pass:
 *   - Quiet current (|I| < threshold) so the OCV estimate is sound
 *   - Settled temperature (5..50 °C by default, chemistry-stable range)
 *   - Sufficient disagreement (gap >= ocv_disagree_pp) to be worth correcting
 *   - Not within the plug-edge holdoff (qg behavior unstable around plug)
 *   - Not actively charging (let qg's coulomb counter converge from real charge)
 * Returns either the raw capacity or a bumped-up value. Releases the
 * correction when raw catches up within 5 pp.
 */
static int ocv_correct(int raw_cap, bool plugged, int i_ua_kernel,
		       u32 v_now_uv, int temp_dc)
{
	int soc_ocv, gap, target, step, out = raw_cap;
	u32 ocv_mv;
	ktime_t now;
	bool eligible;

	if (!ocv_correct_enabled)
		return raw_cap;

	/* compute_live_ocv expects qg-raw current sign (positive=discharge).
	 * We were handed kernel-convention (positive=charge); negate. */
	ocv_mv = compute_live_ocv(v_now_uv, -i_ua_kernel, temp_dc);
	soc_ocv = atl5020_ocv_to_soc(ocv_mv, temp_dc);
	gap = soc_ocv - raw_cap;

	mutex_lock(&cap_lock);

	now = ktime_get();
	if (plugged != ocv_prev_plugged) {
		ocv_last_plug_edge = now;
		ocv_prev_plugged = plugged;
	}
	if (ocv_last_corrected == INT_MIN)
		ocv_last_corrected = raw_cap;

	eligible =
		ktime_ms_delta(now, ocv_last_plug_edge) >= ocv_plug_holdoff_ms &&
		!(plugged && i_ua_kernel > 0) &&
		abs(i_ua_kernel) < ocv_i_quiet_ua &&
		temp_dc >= ocv_temp_min_dc && temp_dc <= ocv_temp_max_dc &&
		gap >= ocv_disagree_pp;

	if (eligible) {
		target = soc_ocv;
		step = ocv_max_step_pp;
		if (target - ocv_last_corrected < step)
			step = target - ocv_last_corrected;
		if (step < 0)
			step = 0;
		out = ocv_last_corrected + step;
		if (out < raw_cap)
			out = raw_cap;
		if (out > target)
			out = target;
		ocv_correction_active = true;
		pr_info_ratelimited("joyeuse_battery_shim: correct %d->%d "
			"(ocv=%umV soc_ocv=%d i=%dmA T=%d.%dC)\n",
			raw_cap, out, ocv_mv, soc_ocv,
			i_ua_kernel / 1000, temp_dc / 10, abs(temp_dc) % 10);
	} else if (ocv_correction_active) {
		if (raw_cap + 5 >= ocv_last_corrected) {
			ocv_correction_active = false;
			pr_info_ratelimited("joyeuse_battery_shim: release ocv correction, raw=%d\n",
				raw_cap);
		} else {
			out = ocv_last_corrected;
		}
	}

	if (out < 0)
		out = 0;
	if (out > 100)
		out = 100;
	ocv_last_corrected = out;

	mutex_unlock(&cap_lock);
	return out;
}

/*
 * Exponential moving average on the OCV-corrected capacity. Operates in
 * centi-percentage-points (s32) to keep sub-1% resolution invisible to UPower
 * while preventing quantization drift. Time constant tunable at runtime via
 * cap_smooth_tau_ms; the EMA alpha is computed in u64 to safely shift by 16.
 * Snaps (replaces smoothed state with the incoming value) when the input
 * differs from the current smoothed value by 15 pp or more — handles sudden
 * legitimate jumps like first-call seed and post-resume.
 *
 * Caller must hold cap_lock.
 */
static int smooth_capacity(int corrected)
{
	ktime_t now = ktime_get_boottime();
	s32 tau_local = max(100, READ_ONCE(cap_smooth_tau_ms));
	s32 target_cpp = corrected * 100;
	s32 diff, out;
	s64 dt_ms;
	u32 alpha_q16;

	if (!READ_ONCE(cap_smooth_enabled) || smoothed_centi_pp == INT_MIN) {
		/* Disabled OR first call: keep state warm so a flip back to enabled is seamless. */
		smoothed_centi_pp = target_cpp;
		smooth_last_ktime = now;
		return corrected;
	}

	dt_ms = ktime_to_ms(ktime_sub(now, smooth_last_ktime));
	if (dt_ms < 1)
		dt_ms = 1;
	if (dt_ms > (s64)(4 * tau_local))
		dt_ms = 4 * tau_local;

	diff = target_cpp - smoothed_centi_pp;
	if (abs(diff) >= 1500) {                /* 15 pp snap */
		smoothed_centi_pp = target_cpp;
	} else {
		alpha_q16 = (u32)(((u64)dt_ms << 16) / ((u64)dt_ms + (u64)tau_local));
		smoothed_centi_pp += (s32)(((s64)diff * (s64)alpha_q16) >> 16);
	}
	smooth_last_ktime = now;

	out = (smoothed_centi_pp + 50) / 100;
	if (out < 0) out = 0;
	if (out > 100) out = 100;
	return out;
}

/*
 * Monotonicity floor while charging or full. Engages whenever the charger is
 * pushing current INTO the battery or STATUS would report Full; releases after
 * cap_mono_release_ms of continuous non-engagement, so brief BC1.2 / PD
 * renegotiation dips don't drop the meter. On engagement, the floor latches
 * the freshly-computed value (NOT a stale last_reported), so a boot-while-
 * charging seeds the floor correctly on the very first call.
 *
 * Caller must hold cap_lock.
 */
static int monotonic_clamp(int in, bool plugged, int i_ua_kernel, bool status_full)
{
	ktime_t now = ktime_get_boottime();
	bool engaged;
	int out = in;

	if (!READ_ONCE(cap_mono_enabled)) {
		clamp_floor = INT_MIN;
		mono_release_deadline = KTIME_MAX;
		mono_prev_engaged = false;
		return in;
	}

	engaged = (plugged && i_ua_kernel > CHG_THRESH_UA) || status_full;

	if (engaged && !mono_prev_engaged) {
		if (clamp_floor == INT_MIN || in > clamp_floor)
			clamp_floor = in;
	}
	if (!engaged && mono_prev_engaged) {
		s64 hold_ms = max(0, READ_ONCE(cap_mono_release_ms));
		mono_release_deadline = ktime_add_ms(now, hold_ms);
	}
	if (!engaged && mono_release_deadline != KTIME_MAX &&
	    ktime_compare(now, mono_release_deadline) >= 0) {
		clamp_floor = INT_MIN;
		mono_release_deadline = KTIME_MAX;
	}

	if (clamp_floor != INT_MIN && out < clamp_floor)
		out = clamp_floor;

	mono_prev_engaged = engaged;
	return out;
}

/*
 * Read the typec port's power_role sysfs attribute. When plugged in as
 * sink the file contains "source [sink]"; when sourcing OTG it's
 * "[source] sink"; unplugged it's typically empty brackets or just the
 * default role with no brackets. Anything containing "[sink]" means
 * VBUS is being received → charging.
 */
static bool typec_port_is_sink(void)
{
	struct file *f;
	char buf[64];
	loff_t pos = 0;
	ssize_t n;

	f = filp_open(TYPEC_ROLE_F, O_RDONLY, 0);
	if (IS_ERR(f))
		return false;
	n = kernel_read(f, buf, sizeof(buf) - 1, &pos);
	filp_close(f, NULL);
	if (n <= 0)
		return false;
	buf[n] = '\0';
	return strstr(buf, "[sink]") != NULL;
}

static int read_prop(const char *name, enum power_supply_property p,
		     union power_supply_propval *v)
{
	struct power_supply *src = power_supply_get_by_name(name);
	int rc;

	if (!src)
		return -ENODEV;
	rc = power_supply_get_property(src, p, v);
	power_supply_put(src);
	return rc;
}

static int shim_get(struct power_supply *psy,
		    enum power_supply_property prop,
		    union power_supply_propval *val)
{
	if (prop == POWER_SUPPLY_PROP_STATUS) {
		/*
		 * Truth-based STATUS:
		 *   ua = qcom_qg.current_now * qg_current_sign  (kernel convention,
		 *        positive = current INTO battery)
		 *   plugged = /sys/class/typec/port0/power_role contains "[sink]"
		 *
		 *   ua >  +THRESH                          -> Charging
		 *   ua <  -THRESH, plugged                 -> Not charging
		 *                                            (cable in but battery
		 *                                             losing power — weak
		 *                                             charger / data-only)
		 *   ua <  -THRESH, unplugged               -> Discharging
		 *   |ua| <= THRESH, plugged, cap >= 98     -> Full
		 *   |ua| <= THRESH, plugged                -> Not charging
		 *   |ua| <= THRESH, unplugged              -> Not charging
		 */
		union power_supply_propval cur = {0}, cap = {0};
		bool plugged = typec_port_is_sink();
		int rc, ua;

		rc = read_prop(QG_NAME, POWER_SUPPLY_PROP_CURRENT_NOW, &cur);
		if (rc)
			return rc;
		ua = cur.intval * qg_current_sign;

		if (ua > CHG_THRESH_UA) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ua < -CHG_THRESH_UA) {
			/*
			 * Current is flowing OUT of the battery. Always
			 * Discharging — GNOME draws the lightning-bolt icon
			 * for anything else, including NOT_CHARGING, which
			 * would falsely suggest "plugged and charging".
			 */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			if (plugged && !read_prop(QG_NAME, POWER_SUPPLY_PROP_CAPACITY, &cap)
			    && cap.intval >= 98)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
		return 0;
	}

	if (prop == POWER_SUPPLY_PROP_MODEL_NAME) {
		val->strval = "joyeuse-battery";
		return 0;
	}
	if (prop == POWER_SUPPLY_PROP_MANUFACTURER) {
		val->strval = "Xiaomi";
		return 0;
	}

	/*
	 * CURRENT_NOW / CURRENT_AVG come from qcom_qg in Qualcomm-raw form
	 * (positive = discharging on this PMIC); kernel power_supply class
	 * convention is positive = charging. Flip with qg_current_sign so
	 * UPower's own sign-based heuristics agree with our STATUS.
	 */
	if (prop == POWER_SUPPLY_PROP_CURRENT_NOW ||
	    prop == POWER_SUPPLY_PROP_CURRENT_AVG) {
		int rc = read_prop(QG_NAME, prop, val);
		if (!rc)
			val->intval *= qg_current_sign;
		return rc;
	}

	if (prop == POWER_SUPPLY_PROP_CAPACITY) {
		union power_supply_propval raw = {0}, cur = {0}, vol = {0}, t = {0};
		int rc, raw_cap, corrected, i_ua, result;
		bool plugged, status_full, passthrough;

		rc = read_prop(QG_NAME, POWER_SUPPLY_PROP_CAPACITY, &raw);
		if (rc)
			return rc;

		if (read_prop(QG_NAME, POWER_SUPPLY_PROP_CURRENT_NOW, &cur) ||
		    read_prop(QG_NAME, POWER_SUPPLY_PROP_VOLTAGE_NOW, &vol) ||
		    read_prop(QG_NAME, POWER_SUPPLY_PROP_TEMP, &t)) {
			val->intval = raw.intval;
			return 0;
		}

		raw_cap = clamp(raw.intval, 0, 100);
		plugged = typec_port_is_sink();
		i_ua    = cur.intval * qg_current_sign;
		status_full = plugged && abs(i_ua) <= CHG_THRESH_UA && raw_cap >= 98;
		passthrough = READ_ONCE(shim_passthrough) != 0;

		mutex_lock(&cap_lock);

		/* passthrough → reset all pipeline state so a flip back is seamless */
		if (passthrough) {
			prev_passthrough     = true;
			smoothed_centi_pp    = INT_MIN;
			clamp_floor          = INT_MIN;
			mono_release_deadline = KTIME_MAX;
			mono_prev_engaged    = false;
			last_reported        = raw_cap;
			mutex_unlock(&cap_lock);
			val->intval = raw_cap;
			return 0;
		}

		/* leaving passthrough → re-seed so the EMA doesn't snap on a huge dt */
		if (prev_passthrough) {
			smoothed_centi_pp = INT_MIN;        /* triggers first-call seed in smooth_capacity */
			mono_prev_engaged = false;          /* don't synthesize a stale engagement edge */
			prev_passthrough  = false;
		}

		/* ocv_correct still acquires/releases cap_lock internally — we drop the lock
		 * around it to preserve the existing locking discipline. */
		mutex_unlock(&cap_lock);
		corrected = ocv_correct(raw_cap, plugged, i_ua,
					(u32)vol.intval, t.intval);
		mutex_lock(&cap_lock);

		if (corrected <= 15) {
			result = corrected;
		} else {
			result = smooth_capacity(corrected);
			result = monotonic_clamp(result, plugged, i_ua, status_full);
		}

		if (status_full || raw_cap >= 99)
			if (raw_cap > result)
				result = raw_cap;

		result = clamp(result, 0, 100);
		last_reported = result;

		if (READ_ONCE(cap_debug))
			pr_info_ratelimited(
				"joyeuse_battery_shim: cap raw=%d corrected=%d smoothed=%d "
				"floor=%d engaged=%d passthru=%d -> %d (t=%lld)\n",
				raw_cap, corrected,
				smoothed_centi_pp == INT_MIN ? -1 : smoothed_centi_pp / 100,
				clamp_floor == INT_MIN ? -1 : clamp_floor,
				mono_prev_engaged, passthrough, result,
				ktime_to_ns(ktime_get_boottime()));

		mutex_unlock(&cap_lock);

		val->intval = result;
		return 0;
	}

	return read_prop(QG_NAME, prop, val);
}

static enum power_supply_property shim_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct power_supply_desc shim_desc = {
	.name           = "battery",
	.type           = POWER_SUPPLY_TYPE_BATTERY,
	.properties     = shim_props,
	.num_properties = ARRAY_SIZE(shim_props),
	.get_property   = shim_get,
};

static int try_publish(void)
{
	struct power_supply_config cfg = {0};
	struct power_supply *qg;
	int rc = 0;

	mutex_lock(&publish_lock);
	if (shim_published)
		goto out;

	qg = power_supply_get_by_name(QG_NAME);
	if (!qg) {
		rc = -EAGAIN;
		goto out;
	}
	power_supply_put(qg);

	shim_psy = power_supply_register(NULL, &shim_desc, &cfg);
	if (IS_ERR(shim_psy)) {
		rc = PTR_ERR(shim_psy);
		shim_psy = NULL;
		pr_err("joyeuse_battery_shim: register failed (%d)\n", rc);
		goto out;
	}
	shim_published = true;
	pr_info("joyeuse_battery_shim: registered /sys/class/power_supply/battery\n");
out:
	mutex_unlock(&publish_lock);
	return rc;
}

static int shim_notify(struct notifier_block *nb, unsigned long ev, void *v)
{
	struct power_supply *src = v;

	if (!src || !src->desc || !src->desc->name)
		return NOTIFY_OK;
	if (src == shim_psy)
		return NOTIFY_OK;

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (!shim_published && !strcmp(src->desc->name, QG_NAME)) {
		try_publish();
		return NOTIFY_OK;
	}

	if (shim_published &&
	    (!strcmp(src->desc->name, QG_NAME) ||
	     !strncmp(src->desc->name, TCPM_PREFIX, sizeof(TCPM_PREFIX) - 1)))
		power_supply_changed(shim_psy);

	return NOTIFY_OK;
}

static struct notifier_block shim_nb = {
	.notifier_call = shim_notify,
};

/*
 * Periodic refresh — fire power_supply_changed on our shim every few
 * seconds so UPower stays in sync even when TCPM / qcom_qg don't emit
 * change events (e.g. cable still inserted but charger silently stopped).
 */
static struct delayed_work refresh_work;

static void refresh_fn(struct work_struct *w)
{
	if (shim_published)
		power_supply_changed(shim_psy);
	schedule_delayed_work(&refresh_work, msecs_to_jiffies(REFRESH_MS));
}

static int __init shim_init(void)
{
	int rc;

	rc = power_supply_reg_notifier(&shim_nb);
	if (rc)
		return rc;
	try_publish();
	INIT_DELAYED_WORK(&refresh_work, refresh_fn);
	schedule_delayed_work(&refresh_work, msecs_to_jiffies(REFRESH_MS));
	return 0;
}

static void __exit shim_exit(void)
{
	cancel_delayed_work_sync(&refresh_work);
	power_supply_unreg_notifier(&shim_nb);
	if (shim_published) {
		power_supply_unregister(shim_psy);
		shim_psy = NULL;
		shim_published = false;
	}
}

module_init(shim_init);
module_exit(shim_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("alex / joyeuse mainline");
MODULE_DESCRIPTION("PM6150 battery STATUS shim: qcom_qg + tcpm -> battery");
MODULE_VERSION("0.3");
MODULE_SOFTDEP("pre: qcom_qg tcpm");
