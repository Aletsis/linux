// SPDX-License-Identifier: GPL-2.0
/*
 * pm6150_chgr_minimal: minimal SMB5 charger init for PM6150 on
 * Xiaomi joyeuse / miatoll (SM7125). Mainline has no driver bound to
 * PM6150's CHGR/USB_CHGPTH/BAT_IF/MISC peripherals; bootloader leaves
 * ICL at the SDP fallback (~500 mA) and PD never raises it because the
 * mainline TCPM here doesn't complete a power contract.
 *
 * This module walks the SPMI bus, grabs PM6150's regmap, and on USB
 * plug-in fires the same register sequence downstream qpnp-smb5 uses
 * in SW_OVERRIDE_HC_MODE: disable AICL, set ICL/FCC, override APSD,
 * unsuspend input, enable charging. Result: 1.8 A stable ICL instead
 * of 500 mA trickle. Adds termination at 100 % SoC, hot-side USBIN
 * suspend, and a four-band software JEITA (cold/cool/normal/warm) that
 * down-scales FCC outside the safe band. Hardware JEITA is not enabled.
 *
 * Module params: see individual MODULE_PARM_DESC below. The thermal
 * bands and percent scalers are all tunable at runtime via
 * /sys/module/pm6150_chgr_minimal/parameters/<name>.
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#define QG_NAME           "qcom_qg"
#define PM6150_SPMI_NAME  "0-00"

#define USBIN_BASE  0x1300
#define CHGR_BASE   0x1000
#define MISC_BASE   0x1600

#define USBIN_INT_RT_STS_REG        (USBIN_BASE + 0x10) /* 0x1310 */
#define USBIN_INT_LATCHED_CLR_REG   (USBIN_BASE + 0x14) /* 0x1314 */
#define USBIN_CMD_IL_REG            (USBIN_BASE + 0x40) /* 0x1340 */
#define USBIN_CMD_ICL_OVERRIDE_REG  (USBIN_BASE + 0x42) /* 0x1342 */
#define USBIN_ADAPTER_ALLOW_OVERRIDE_REG (USBIN_BASE + 0x44) /* 0x1344 */
#define USBIN_ADAPTER_ALLOW_CFG_REG (USBIN_BASE + 0x60) /* 0x1360 */
#define USBIN_LOAD_CFG_REG          (USBIN_BASE + 0x65) /* 0x1365 */
#define USBIN_ICL_OPTIONS_REG       (USBIN_BASE + 0x66) /* 0x1366 */
#define USBIN_CURRENT_LIMIT_CFG_REG (USBIN_BASE + 0x70) /* 0x1370 */
#define USBIN_AICL_OPTIONS_CFG_REG  (USBIN_BASE + 0x80) /* 0x1380 */
#define USBIN_PLUGIN_RT_STS_BIT     BIT(4)

#define CHARGING_ENABLE_CMD_REG     (CHGR_BASE + 0x42)  /* 0x1042 */
#define FAST_CHARGE_CURRENT_CFG_REG (CHGR_BASE + 0x61)  /* 0x1061 */

#define USBIN_SUSPEND_BIT           BIT(0)
#define USBIN_UV_RT_STS_BIT         BIT(2)  /* in 0x1310: USB input under-voltage */
#define USBIN_OV_RT_STS_BIT         BIT(3)  /* in 0x1310: USB input over-voltage */
#define ICL_OVERRIDE_BIT            BIT(0)
#define ICL_OVERRIDE_AFTER_APSD_BIT BIT(4)
#define USBIN_MODE_CHG_BIT          BIT(0)  /* in 0x1366: high-current charger mode */
#define USB51_MODE_BIT              BIT(1)  /* in 0x1366: USB2 500/100 mA select */
#define CHARGING_ENABLE_BIT         BIT(0)

/* USBIN_AICL_OPTIONS_CFG_REG (0x1380) bits */
#define USBIN_AICL_EN_BIT               BIT(2)
#define USBIN_AICL_ADC_EN_BIT           BIT(3)
#define USBIN_AICL_PERIODIC_RERUN_EN_BIT BIT(4)
#define SUSPEND_ON_COLLAPSE_USBIN_BIT   BIT(7)

/*
 * USBIN_ADAPTER_ALLOW_CFG_REG (0x1360): the static OVP comparator window
 * applied at all times. Value 0x0C (decimal 12) allows 5–12 V; anything
 * narrower trips the OV comparator the moment PD steps to 9 V or PPS
 * walks the rail. This is the Qualcomm shutdown default for a reason.
 */
#define USBIN_ADAPTER_ALLOW_5V_TO_12V   12

/*
 * USBIN_ADAPTER_ALLOW_OVERRIDE_REG (0x1344): runtime override of the
 * adapter-allow window, used while a PD/PPS contract is active. BIT(3)
 * = CONTINUOUS, accepts the full 4–12 V range so PPS voltage walks
 * don't trip the OV comparator. Mirrors smblib_set_usb_pd_allowed_voltage
 * in downstream Qualcomm drivers.
 */
#define USBIN_ADAPTER_ALLOW_OVERRIDE_CONTINUOUS BIT(3)

/* WD_CFG (0x1651) bits — bark/bite watchdog */
#define WD_CFG_REG                  (MISC_BASE + 0x51) /* 0x1651 */
#define WDOG_TIMER_EN_BIT           BIT(0)
#define WDOG_TIMER_EN_ON_PLUGIN_BIT BIT(1)
#define BARK_WDOG_INT_EN_BIT        BIT(6)
#define WATCHDOG_TRIGGER_AFP_EN_BIT BIT(7)

/*
 * SMB5 encodes ICL and FCC in 50 mA steps. Silicon supports up to ~5 A
 * ICL and ~8 A FCC (raw 0xA0). No mask: a `& 0x7F` would silently truncate
 * legitimate values above ~6.3 A into a wrong-but-plausible reading.
 */
#define MA_TO_RAW(ma) ((ma) / 50)

static int icl_ma = 1800;
module_param(icl_ma, int, 0644);
MODULE_PARM_DESC(icl_ma, "USB input current limit in mA (default 1800)");

static int fcc_ma = 2000;
module_param(fcc_ma, int, 0644);
MODULE_PARM_DESC(fcc_ma, "Battery fast-charge current target in mA (default 2000)");

static bool enable_writes = true;
module_param(enable_writes, bool, 0644);
MODULE_PARM_DESC(enable_writes,
	"true: apply the SMB5 init sequence on plug-in; "
	"false: snapshot registers only (read-only safety mode)");

static int reassert_period_ms = 5000;
module_param(reassert_period_ms, int, 0644);
MODULE_PARM_DESC(reassert_period_ms,
	"how often to re-check + re-apply the charger config while plugged in "
	"(0 disables). Defends against silent register reverts from APSD/AICL/TCPM events.");

static int term_capacity = 100;
module_param(term_capacity, int, 0644);
MODULE_PARM_DESC(term_capacity,
	"capacity (percent) at which to suspend USBIN and stop charging "
	"(101 disables). Resumes when capacity drops by term_hysteresis.");

static int term_hysteresis = 3;
module_param(term_hysteresis, int, 0644);
MODULE_PARM_DESC(term_hysteresis,
	"resume threshold below term_capacity, in percent (default 3 -> "
	"resume at 97 when term_capacity is 100).");

/*
 * Software JEITA bands. Below `cold` and between `cold` and `cool` the
 * FCC is reduced (lithium plating risk at low temp). Between `warm` and
 * the hot suspend threshold the FCC is also reduced (cell ageing
 * acceleration). Default thresholds match conservative interpretations
 * of JEITA 1808.
 */
static int temp_cold_dc = 100;        /* 10.0 C */
module_param(temp_cold_dc, int, 0644);
MODULE_PARM_DESC(temp_cold_dc,
	"battery temp (deci-degC) at/below which to apply fcc_cold_pct (default 100 = 10.0 C).");

static int temp_cool_dc = 150;        /* 15.0 C */
module_param(temp_cool_dc, int, 0644);
MODULE_PARM_DESC(temp_cool_dc,
	"battery temp (deci-degC) at/below which to apply fcc_cool_pct (default 150 = 15.0 C).");

static int temp_warm_dc = 450;        /* 45.0 C */
module_param(temp_warm_dc, int, 0644);
MODULE_PARM_DESC(temp_warm_dc,
	"battery temp (deci-degC) at/above which to apply fcc_warm_pct (default 450 = 45.0 C).");

static int temp_hot_dc = 500;         /* 50.0 C */
module_param(temp_hot_dc, int, 0644);
MODULE_PARM_DESC(temp_hot_dc,
	"battery temp (deci-degC) at/above which JZ_HOT engages and charging is "
	"disabled via CHARGING_ENABLE_CMD (default 500 = 50.0 C, 999 disables). "
	"USBIN stays live so VBUS keeps powering the SoC.");

static int fcc_hot_pct;               /* 0 default — charging disabled in hot zone */
module_param(fcc_hot_pct, int, 0644);
MODULE_PARM_DESC(fcc_hot_pct,
	"percent of fcc_ma to use when temp >= temp_hot_dc (default 0 — "
	"charging is disabled and USBIN stays powered).");

static int fcc_cold_pct = 25;
module_param(fcc_cold_pct, int, 0644);
MODULE_PARM_DESC(fcc_cold_pct,
	"percent of fcc_ma to use when temp <= temp_cold_dc (default 25).");

static int fcc_cool_pct = 50;
module_param(fcc_cool_pct, int, 0644);
MODULE_PARM_DESC(fcc_cool_pct,
	"percent of fcc_ma to use when temp_cold_dc < temp <= temp_cool_dc (default 50).");

static int fcc_warm_pct = 50;
module_param(fcc_warm_pct, int, 0644);
MODULE_PARM_DESC(fcc_warm_pct,
	"percent of fcc_ma to use when temp_warm_dc <= temp < temp_hot_dc (default 50).");

static int temp_zone_hyst_dc = 20;    /* 2.0 C */
module_param(temp_zone_hyst_dc, int, 0644);
MODULE_PARM_DESC(temp_zone_hyst_dc,
	"hysteresis (deci-degC) applied when leaving a non-normal JEITA zone (default 20 = 2.0 C).");

static struct device *pm6150_dev;
static struct regmap *pm6150_rm;
static struct notifier_block plug_nb;
static bool was_plugged;
static DEFINE_MUTEX(apply_lock);

struct named_reg {
	const char *name;
	unsigned int addr;
};

/*
 * Diagnostic-only registers, addresses verified against downstream
 * smb5-reg.h (PM6150 / PM8150B subtype). The earlier table held four
 * undefined offsets (0x1009, 0x100A, 0x100C, 0x100E) and pointed at
 * the wrong base for the Type-C status block (real Type-C lives at
 * TYPEC_BASE = 0x1500, not under USBIN); also pointed at 0x1306 for
 * "USBIN_INPUT_STAT" which is reserved space — the real input-path
 * status is at POWER_PATH_STATUS_REG = 0x110B.
 */
static const struct named_reg snapshot_table[] = {
	/* CHGR — charge state machine + faults */
	{ "BAT_CHG_STATUS_1  (0x1006)", 0x1006 },
	{ "BAT_CHG_STATUS_2  (0x1007)", 0x1007 },
	{ "BAT_CHG_STATUS_5  (0x100B)", 0x100B },
	{ "BAT_CHG_STATUS_7  (0x100D)", 0x100D },
	{ "CHGR_INT_RT_STS   (0x1010)", 0x1010 },
	{ "CHARGING_EN_CMD   (0x1042)", CHARGING_ENABLE_CMD_REG },
	{ "FAST_CHRG_CUR_CFG (0x1061)", FAST_CHARGE_CURRENT_CFG_REG },
	/* DCDC — power-path live status (includes USBIN_SUSPEND_STS) */
	{ "POWER_PATH_STATUS (0x110B)", 0x110B },
	/* USBIN — input plug / APSD / ICL config */
	{ "APSD_STATUS       (0x1307)", 0x1307 },
	{ "APSD_RESULT       (0x1308)", 0x1308 },
	{ "USBIN_INT_RT_STS  (0x1310)", USBIN_INT_RT_STS_REG },
	{ "USBIN_CMD_IL      (0x1340)", USBIN_CMD_IL_REG },
	{ "CMD_ICL_OVERRIDE  (0x1342)", USBIN_CMD_ICL_OVERRIDE_REG },
	{ "ADAPT_ALLOW_OVR   (0x1344)", USBIN_ADAPTER_ALLOW_OVERRIDE_REG },
	{ "ADAPT_ALLOW_CFG   (0x1360)", USBIN_ADAPTER_ALLOW_CFG_REG },
	{ "USBIN_OPTIONS_1   (0x1362)", 0x1362 },
	{ "USBIN_OPTIONS_2   (0x1363)", 0x1363 },
	{ "USBIN_LOAD_CFG    (0x1365)", USBIN_LOAD_CFG_REG },
	{ "USBIN_ICL_OPTIONS (0x1366)", USBIN_ICL_OPTIONS_REG },
	{ "USBIN_CURRENT_LIM (0x1370)", USBIN_CURRENT_LIMIT_CFG_REG },
	{ "USBIN_AICL_OPTS   (0x1380)", USBIN_AICL_OPTIONS_CFG_REG },
	/* TYPEC — real Type-C status block (TYPEC_BASE = 0x1500) */
	{ "TYPE_C_SNK_STATUS (0x1506)", 0x1506 },
	{ "TYPE_C_SRC_STATUS (0x1508)", 0x1508 },
	{ "TYPE_C_SM_STATUS  (0x1509)", 0x1509 },
	{ "TYPE_C_MISC_STAT  (0x150B)", 0x150B },
	/* MISC — watchdog */
	{ "BARK_BITE_WD      (0x1643)", 0x1643 },
	{ "WD_CFG            (0x1651)", 0x1651 },
};

/*
 * Reach PM6150's SPMI device by walking up from the qcom_qg PSY:
 *   qcom_qg (power_supply class device)
 *     -> parent = qgauge@4800 (platform_device under PM6150)
 *       -> parent = PM6150 SPMI device (slave "0-00", owns the regmap)
 * We keep a hold on the qg PSY for the lifetime of the module so the
 * device pointer chain stays valid.
 */
static struct power_supply *qg_psy;

static int find_pm6150(void)
{
	struct device *qgauge_dev;
	struct device *pmic_dev;

	qg_psy = power_supply_get_by_name(QG_NAME);
	if (!qg_psy) {
		pr_err("pm6150_chgr: %s power_supply not found\n", QG_NAME);
		return -ENODEV;
	}
	qgauge_dev = qg_psy->dev.parent;
	if (!qgauge_dev) {
		pr_err("pm6150_chgr: qcom_qg has no parent device\n");
		goto out_put_psy;
	}
	pmic_dev = qgauge_dev->parent;
	if (!pmic_dev) {
		pr_err("pm6150_chgr: qgauge has no parent (expected PM6150 SPMI)\n");
		goto out_put_psy;
	}
	if (strcmp(dev_name(pmic_dev), PM6150_SPMI_NAME) != 0) {
		pr_err("pm6150_chgr: unexpected parent '%s' (wanted %s)\n",
		       dev_name(pmic_dev), PM6150_SPMI_NAME);
		goto out_put_psy;
	}
	pm6150_rm = dev_get_regmap(pmic_dev, NULL);
	if (!pm6150_rm) {
		pr_err("pm6150_chgr: no regmap on %s\n", dev_name(pmic_dev));
		goto out_put_psy;
	}
	pm6150_dev = get_device(pmic_dev);
	return 0;

out_put_psy:
	power_supply_put(qg_psy);
	qg_psy = NULL;
	return -ENODEV;
}

static void release_pm6150(void)
{
	if (pm6150_dev) {
		put_device(pm6150_dev);
		pm6150_dev = NULL;
	}
	if (qg_psy) {
		power_supply_put(qg_psy);
		qg_psy = NULL;
	}
}

static void snapshot_regs(const char *tag)
{
	unsigned int v;
	int i, rc;

	pr_info("pm6150_chgr: register snapshot [%s]\n", tag);
	for (i = 0; i < ARRAY_SIZE(snapshot_table); i++) {
		rc = regmap_read(pm6150_rm, snapshot_table[i].addr, &v);
		if (rc) {
			pr_warn("pm6150_chgr:   %s = (read fail %d)\n",
				snapshot_table[i].name, rc);
			continue;
		}
		pr_info("pm6150_chgr:   %s = 0x%02x\n",
			snapshot_table[i].name, v & 0xFF);
	}
}

static bool is_usb_plugged(void)
{
	unsigned int v;

	if (regmap_read(pm6150_rm, USBIN_INT_RT_STS_REG, &v))
		return false;
	return !!(v & USBIN_PLUGIN_RT_STS_BIT);
}

/*
 * True if the PMIC currently flags USBIN over-voltage or under-voltage
 * in USBIN_INT_RT_STS (0x1310). Used to gate USBIN unsuspend so we
 * don't connect the input to a faulted VBUS. Failing the regmap read
 * is treated as "fault present" (fail-safe): don't enable an input we
 * can't observe.
 */
static bool usbin_fault_present(void)
{
	unsigned int v;

	if (regmap_read(pm6150_rm, USBIN_INT_RT_STS_REG, &v))
		return true;
	return !!(v & (USBIN_OV_RT_STS_BIT | USBIN_UV_RT_STS_BIT));
}

/*
 * Software JEITA zone tracking. supervise_jeita() down the file is the
 * sole writer; apply_charge_config() reads current_zone so its FCC write
 * stays consistent with the active derating band across plug-in and
 * drift-reapply paths.
 */
enum jeita_zone { JZ_NORMAL = 0, JZ_COLD, JZ_COOL, JZ_WARM, JZ_HOT };
static enum jeita_zone current_zone = JZ_NORMAL;
static int zone_fcc_ma(enum jeita_zone z);

static int apply_charge_config(void)
{
	u8 icl_raw = MA_TO_RAW(icl_ma);
	int fcc_target_ma = zone_fcc_ma(current_zone);
	u8 fcc_raw;

	/*
	 * zone_fcc_ma returns -1 when the active zone wants charging fully
	 * disabled (currently only JZ_HOT with fcc_hot_pct=0). In that case
	 * write a safe 100 mA floor here — CHARGING_ENABLE is decoupled
	 * into apply_zone_policy_locked() which fires right after this and
	 * will gate charging off, so the floor is never actually delivered
	 * to the cell. We still need a sensible byte value in the FCC
	 * register because MA_TO_RAW(-1) would otherwise be 0 or worse.
	 */
	if (fcc_target_ma < 0)
		fcc_target_ma = 100;
	fcc_raw = MA_TO_RAW(fcc_target_ma);
	int rc;

	pr_info("pm6150_chgr: applying SMB5 init  ICL=%d mA (raw 0x%02x)  FCC=%d mA (raw 0x%02x)\n",
		icl_ma, icl_raw, fcc_target_ma, fcc_raw);

	/*
	 * 1. Disable hardware AICL bits but PRESERVE
	 *    SUSPEND_ON_COLLAPSE_USBIN_BIT. Deliberate design choice: this
	 *    is a software-override driver — we force a static ICL via
	 *    ICL_OVERRIDE_AFTER_APSD (step 7); periodic AICL rerun would
	 *    step our static value back down/up every cycle and oscillate
	 *    against weak chargers. SUSPEND_ON_COLLAPSE stays asserted so
	 *    hardware still latches off a hard supply collapse. Earlier
	 *    revisions wrote 0x00 here, which silently cleared
	 *    SUSPEND_ON_COLLAPSE too — fixed.
	 */
	rc = regmap_update_bits(pm6150_rm, USBIN_AICL_OPTIONS_CFG_REG,
				USBIN_AICL_EN_BIT |
				USBIN_AICL_PERIODIC_RERUN_EN_BIT |
				USBIN_AICL_ADC_EN_BIT |
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				SUSPEND_ON_COLLAPSE_USBIN_BIT);
	if (rc) goto err;

	/* 3. set USB input current limit */
	rc = regmap_write(pm6150_rm, USBIN_CURRENT_LIMIT_CFG_REG, icl_raw);
	if (rc) goto err;

	/* 4. set battery fast-charge current — already zone-derated above */
	rc = regmap_write(pm6150_rm, FAST_CHARGE_CURRENT_CFG_REG, fcc_raw);
	if (rc) goto err;

	/* 5. set USBIN_MODE_CHG (high-current), clear USB51_MODE */
	rc = regmap_update_bits(pm6150_rm, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT | USB51_MODE_BIT,
				USBIN_MODE_CHG_BIT);
	if (rc) goto err;

	/*
	 * 6. CLEAR ICL_OVERRIDE_BIT. Downstream SW_OVERRIDE_HC_MODE writes 0
	 *    here — that bit is for SW_OVERRIDE_USB51_MODE / NO_CC_MODE. In
	 *    HC mode the ICL is governed by ICL_OVERRIDE_AFTER_APSD_BIT in
	 *    the next step. Earlier revisions wrongly set this bit.
	 */
	rc = regmap_update_bits(pm6150_rm, USBIN_CMD_ICL_OVERRIDE_REG,
				ICL_OVERRIDE_BIT, 0);
	if (rc) goto err;

	/* 7. ICL_OVERRIDE_AFTER_APSD — survives APSD reruns */
	rc = regmap_update_bits(pm6150_rm, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT,
				ICL_OVERRIDE_AFTER_APSD_BIT);
	if (rc) goto err;

	/*
	 * 8. widen the OVP comparator window BEFORE sampling INT_RT_STS.
	 * Boot defaults often leave USBIN_ADAPTER_ALLOW_CFG at the 5 V-only
	 * setting; once TCPM walks VBUS up to 9 V via PD/PPS the live OV
	 * comparator (the source of INT_RT_STS bit 3) latches truthy and
	 * the fault gate below would block forever. ALLOW_CFG widens the
	 * always-on window to 5–12 V; OVERRIDE = CONTINUOUS handles the
	 * full PPS range while a PD contract is up.
	 */
	rc = regmap_write(pm6150_rm, USBIN_ADAPTER_ALLOW_CFG_REG,
			  USBIN_ADAPTER_ALLOW_5V_TO_12V);
	if (rc) goto err;
	rc = regmap_write(pm6150_rm, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
			  USBIN_ADAPTER_ALLOW_OVERRIDE_CONTINUOUS);
	if (rc) goto err;
	/*
	 * 9. clear any USBIN latched interrupt bits left over from a
	 * pre-fix transient (W1C; only currently-low bits fall, so a real
	 * ongoing fault stays visible).
	 */
	regmap_write(pm6150_rm, USBIN_INT_LATCHED_CLR_REG, 0xFF);

	/*
	 * 10. clear USBIN_SUSPEND — but only if VBUS is currently healthy.
	 * SMB5's HW OV/UV cutoff is independent of the SUSPEND bit, but
	 * explicit gating here avoids enabling input on a known-faulted
	 * source and gives the dmesg trail of a fault the next reader
	 * needs. With the window widened in step 8, RT_STS bit 3 deasserts
	 * within microseconds for any in-spec adapter voltage.
	 */
	if (usbin_fault_present()) {
		pr_warn("pm6150_chgr: USBIN OV/UV asserted, keeping suspend\n");
	} else {
		rc = regmap_update_bits(pm6150_rm, USBIN_CMD_IL_REG,
					USBIN_SUSPEND_BIT, 0);
		if (rc) goto err;
	}

	/*
	 * Master CHARGING_ENABLE intentionally NOT set here. Decoupled into
	 * apply_zone_policy_locked() which reads the current cell
	 * temperature and either enables charging (with the correct
	 * zone-derated FCC) or leaves it disabled when in JZ_HOT. Earlier
	 * revisions enabled here unconditionally, racing JEITA into one
	 * polling tick of full FCC on a hot cell.
	 */

	pr_info("pm6150_chgr: SMB5 init sequence applied OK\n");
	return 0;
err:
	pr_err("pm6150_chgr: SMB5 init sequence failed at rc=%d\n", rc);
	return rc;
}

/*
 * Periodic re-assert: while a cable is plugged in, repeatedly check the
 * critical bits and re-apply if any have drifted. PMIC events (APSD
 * rerun, AICL rerun, TCPM CC flap, USB type re-detection) can quietly
 * clobber USBIN_MODE_CHG / ICL_OVERRIDE; the snapshot above proves the
 * config holds in steady state but unplug/replug or some plug-events
 * tear it down briefly. Cheap to re-apply if redundant.
 */
static struct delayed_work reassert_work;

/*
 * Decide whether the periodic reassert work should re-fire the SMB5
 * init sequence. We compare the live silicon state against the
 * invariants apply_charge_config sets up:
 *   USBIN_MODE_CHG  must be set        (high-current mode)
 *   USB51_MODE      must be clear      (we are not in USB-2 500/100 mode)
 *   ICL_OVERRIDE_AFTER_APSD must be set (static ICL survives APSD reruns)
 *
 * Note: USBIN_CMD_ICL_OVERRIDE_REG.ICL_OVERRIDE_BIT is deliberately
 * left CLEAR by apply_charge_config (HC mode), so we must NOT check
 * that bit here — earlier revisions did, which caused needs_reapply
 * to return true on every tick and a re-apply storm.
 */
static bool needs_reapply(void)
{
	unsigned int v;

	if (regmap_read(pm6150_rm, USBIN_ICL_OPTIONS_REG, &v))
		return false;
	if ((v & USBIN_MODE_CHG_BIT) == 0)
		return true;
	if (v & USB51_MODE_BIT)
		return true;
	if (regmap_read(pm6150_rm, USBIN_LOAD_CFG_REG, &v))
		return false;
	if ((v & ICL_OVERRIDE_AFTER_APSD_BIT) == 0)
		return true;
	return false;
}

/*
 * Set or clear USBIN_SUSPEND. Setting it stops current draw from VBUS
 * entirely without affecting the rest of the charger config — used for
 * full-charge termination and over-temperature pause.
 */
static int set_usbin_suspended(bool suspended)
{
	return regmap_update_bits(pm6150_rm, USBIN_CMD_IL_REG,
		USBIN_SUSPEND_BIT, suspended ? USBIN_SUSPEND_BIT : 0);
}

/*
 * Write FCC_CFG with a value derived from `ma`, clamped to the SMB5
 * supported range (50–3000 mA in 50 mA steps). Returns the actual
 * milliamps committed (post-rounding) or a negative errno.
 */
static int set_fcc_ma(int ma)
{
	int clamped, raw;

	if (ma < 100)
		clamped = 100;
	else if (ma > 3000)
		clamped = 3000;
	else
		clamped = ma;
	raw = MA_TO_RAW(clamped);
	if (regmap_write(pm6150_rm, FAST_CHARGE_CURRENT_CFG_REG, raw))
		return -EIO;
	return raw * 50;
}

/*
 * Software JEITA zone state. We pick a zone from the current battery
 * temperature with hysteresis on the exit edges, then dial FCC to a
 * percent of the user-configured target. Hot-side suspension stays in
 * supervise_safety() — JEITA only down-scales FCC, never raises VBUS or
 * extends the safe band. The `enum jeita_zone` and `current_zone`
 * definitions live higher up, hoisted so apply_charge_config can read
 * the active zone when reapplying its FCC.
 */

/*
 * Toggle the master charging-enable bit. Independent of USBIN suspend:
 * with this cleared, the cell-side ICHG loop stops but VBUS keeps
 * powering the SoC through the SMB5 system-load path. JZ_HOT uses
 * this instead of suspending USBIN so a hot, drained phone can still
 * boot from the cable.
 */
static int set_charging_enabled(bool en)
{
	return regmap_update_bits(pm6150_rm, CHARGING_ENABLE_CMD_REG,
		CHARGING_ENABLE_BIT, en ? CHARGING_ENABLE_BIT : 0);
}

static const char *zone_name(enum jeita_zone z)
{
	switch (z) {
	case JZ_COLD: return "cold";
	case JZ_COOL: return "cool";
	case JZ_WARM: return "warm";
	case JZ_HOT:  return "hot";
	default:        return "normal";
	}
}

static enum jeita_zone classify_zone(int temp_dc)
{
	int hyst = temp_zone_hyst_dc;

	switch (current_zone) {
	case JZ_COLD:
		if (temp_dc > temp_cold_dc + hyst)
			break;
		return JZ_COLD;
	case JZ_COOL:
		if (temp_dc > temp_cool_dc + hyst)
			break;
		if (temp_dc <= temp_cold_dc)
			return JZ_COLD;
		return JZ_COOL;
	case JZ_WARM:
		if (temp_dc < temp_warm_dc - hyst)
			break;
		if (temp_dc >= temp_hot_dc && temp_hot_dc < 999)
			return JZ_HOT;
		return JZ_WARM;
	case JZ_HOT:
		if (temp_dc < temp_hot_dc - hyst)
			break;
		return JZ_HOT;
	default:
		break;
	}

	if (temp_dc <= temp_cold_dc)
		return JZ_COLD;
	if (temp_dc <= temp_cool_dc)
		return JZ_COOL;
	if (temp_dc >= temp_hot_dc && temp_hot_dc < 999)
		return JZ_HOT;
	if (temp_dc >= temp_warm_dc)
		return JZ_WARM;
	return JZ_NORMAL;
}

/*
 * Returns target FCC mA for a zone, or a negative value to signal
 * "charging fully disabled in this zone". JZ_HOT with fcc_hot_pct=0 is
 * the canonical case: USBIN stays live but the cell does not receive
 * any current.
 */
static int zone_fcc_ma(enum jeita_zone z)
{
	int pct;

	switch (z) {
	case JZ_COLD: pct = fcc_cold_pct; break;
	case JZ_COOL: pct = fcc_cool_pct; break;
	case JZ_WARM: pct = fcc_warm_pct; break;
	case JZ_HOT:  pct = fcc_hot_pct;  break;
	default:        return fcc_ma;
	}
	if (pct <= 0)
		return -1;
	if (pct > 100) pct = 100;
	return (fcc_ma * pct) / 100;
}

/*
 * Takes apply_lock so it cannot race apply_zone_policy_locked() (the
 * other writer of current_zone, called from plug-in / drift-reapply /
 * init paths). Both must be the sole writers under the same lock or
 * the zone state can be torn between concurrent updates.
 */
static void supervise_jeita(int temp_dc)
{
	enum jeita_zone next;
	int target;

	if (!enable_writes)
		return;

	mutex_lock(&apply_lock);

	next = classify_zone(temp_dc);
	if (next == current_zone) {
		mutex_unlock(&apply_lock);
		return;
	}

	target = zone_fcc_ma(next);
	pr_info("pm6150_chgr: JEITA zone %s -> %s (temp=%d.%d C, FCC -> %d mA%s)\n",
		zone_name(current_zone), zone_name(next),
		temp_dc / 10, abs(temp_dc) % 10,
		target < 0 ? 0 : target,
		target < 0 ? "; charging disabled" : "");

	if (target < 0) {
		if (!set_charging_enabled(false))
			current_zone = next;
	} else {
		if (set_charging_enabled(true) == 0 && set_fcc_ma(target) >= 0)
			current_zone = next;
	}

	mutex_unlock(&apply_lock);
}

/*
 * Called at plug-in / drift-reapply / init-with-cable to set FCC and
 * the master charging-enable bit from a *fresh* temperature read.
 * Without this synchronous policy update, apply_charge_config would
 * leave the cell at full FCC for one polling tick — long enough to
 * push real current into a hot cell while supervise_jeita waits for
 * its next reassert tick. Caller must hold apply_lock.
 */
static int read_qg_int(enum power_supply_property prop, int *out);

static void apply_zone_policy_locked(void)
{
	int temp_dc;
	enum jeita_zone next;
	int target;

	if (!enable_writes)
		return;

	if (read_qg_int(POWER_SUPPLY_PROP_TEMP, &temp_dc))
		next = JZ_NORMAL;
	else
		next = classify_zone(temp_dc);

	target = zone_fcc_ma(next);
	if (target < 0) {
		pr_info("pm6150_chgr: post-init policy: zone %s, charging disabled\n",
			zone_name(next));
		set_charging_enabled(false);
	} else {
		set_fcc_ma(target);
		set_charging_enabled(true);
	}
	current_zone = next;
}

/*
 * find_pm6150() already grabbed and held qg_psy for the lifetime of the
 * module. Reuse that reference here instead of doing a name lookup +
 * get/put refcount cycle on every read — supervise_jeita and the safety
 * loop hit this several times per polling tick.
 */
static int read_qg_int(enum power_supply_property prop, int *out)
{
	union power_supply_propval v = {0};
	struct power_supply *src = qg_psy;
	int rc;

	if (!src)
		return -ENODEV;
	rc = power_supply_get_property(src, prop, &v);
	if (rc)
		return rc;
	*out = v.intval;
	return 0;
}

/*
 * Safety supervision: suspend USBIN only on 100 %-SoC termination.
 * Hot battery now lives in the JEITA hot zone (JZ_HOT) which disables
 * charging via CHARGING_ENABLE_CMD while keeping USBIN live — earlier
 * revisions suspended USBIN on overtemp, which gated VBUS from the
 * SoC and turned every hot, drained phone into a brick. JZ_HOT is the
 * canonical path now; the temp branch here is intentionally gone.
 */
static bool safety_suspend_active;

static void supervise_safety(void)
{
	int cap;
	bool want_suspend = false;

	if (!enable_writes)
		return;

	if (read_qg_int(POWER_SUPPLY_PROP_CAPACITY, &cap))
		return;

	if (safety_suspend_active) {
		if (cap >= term_capacity - term_hysteresis)
			want_suspend = true;
	} else if (cap >= term_capacity && term_capacity <= 100) {
		want_suspend = true;
	}

	if (want_suspend && !safety_suspend_active) {
		pr_info("pm6150_chgr: suspending USBIN (termination; cap=%d)\n", cap);
		if (!set_usbin_suspended(true))
			safety_suspend_active = true;
	} else if (!want_suspend && safety_suspend_active) {
		if (usbin_fault_present()) {
			pr_warn("pm6150_chgr: deferring USBIN resume — OV/UV fault present\n");
		} else {
			pr_info("pm6150_chgr: resuming USBIN (cap=%d)\n", cap);
			if (!set_usbin_suspended(false))
				safety_suspend_active = false;
		}
	}
}

static void reassert_fn(struct work_struct *w)
{
	int temp_dc;

	if (is_usb_plugged()) {
		if (enable_writes && needs_reapply()) {
			mutex_lock(&apply_lock);
			pr_info("pm6150_chgr: drift detected, re-applying SMB5 init\n");
			apply_charge_config();
			/* Re-evaluate zone policy in the same critical section
			 * — apply_charge_config no longer enables charging by
			 * itself, so without this call a hot cell could see a
			 * stretch of the previous (stale) policy state. */
			apply_zone_policy_locked();
			mutex_unlock(&apply_lock);
		}
		supervise_safety();
		if (!read_qg_int(POWER_SUPPLY_PROP_TEMP, &temp_dc))
			supervise_jeita(temp_dc);
	} else {
		/*
		 * Cable is out. Defensively clear USBIN_SUSPEND so a bit left
		 * asserted by a debug write, a prior buggy revision, or by an
		 * incomplete termination cycle can't survive into the next
		 * plug-in and silently block charging.
		 */
		if (enable_writes && safety_suspend_active) {
			set_usbin_suspended(false);
			safety_suspend_active = false;
		}
	}
	if (reassert_period_ms > 0)
		schedule_delayed_work(&reassert_work,
			msecs_to_jiffies(reassert_period_ms));
}

static int plug_notify(struct notifier_block *nb, unsigned long ev, void *v)
{
	bool plugged;

	if (ev != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	plugged = is_usb_plugged();
	if (plugged && !was_plugged) {
		mutex_lock(&apply_lock);
		pr_info("pm6150_chgr: plug-in event\n");
		if (enable_writes) {
			apply_charge_config();
			/* Synchronously apply the zone-aware enable + FCC so a
			 * hot cell never sees an interim CHARGING_ENABLE=1
			 * before the next reassert tick. */
			apply_zone_policy_locked();
		}
		snapshot_regs("post-plug-in");
		mutex_unlock(&apply_lock);
	} else if (!plugged && was_plugged) {
		pr_info("pm6150_chgr: unplug event\n");
	}
	was_plugged = plugged;
	return NOTIFY_OK;
}

static int __init pm6150_chgr_init(void)
{
	int rc;

	rc = find_pm6150();
	if (rc)
		return rc;
	pr_info("pm6150_chgr: bound to PM6150 (%s)\n", dev_name(pm6150_dev));

	snapshot_regs("at-insmod");

	/*
	 * Disarm the SMB5 bark/bite watchdog. We don't service its IRQ, so
	 * if the bootloader (or a previous driver) left it armed it can
	 * silently disable charging mid-session. Clearing both the global
	 * enable and the auto-arm-on-plug bit is the same defence the
	 * downstream qpnp-smb5 driver takes when not running its full WD
	 * state machine.
	 */
	if (enable_writes)
		regmap_update_bits(pm6150_rm, WD_CFG_REG,
				   WDOG_TIMER_EN_BIT |
				   WDOG_TIMER_EN_ON_PLUGIN_BIT |
				   BARK_WDOG_INT_EN_BIT |
				   WATCHDOG_TRIGGER_AFP_EN_BIT, 0);

	/*
	 * Unconditionally widen the OVP comparator window at probe, BEFORE
	 * we sample is_usb_plugged(). The plug detector itself rides on the
	 * USBIN comparator and gets wedged off if the boot-default 5 V-only
	 * window leaves OV asserted against a 9 V PD rail still on the
	 * cable. ALLOW_CFG = 5-12 V is the Qualcomm shutdown default and the
	 * safe always-on choice; OVERRIDE = CONTINUOUS covers active PPS.
	 * Latched-clear sweeps any stale one-shots so the post-init snapshot
	 * reflects only live state.
	 */
	if (enable_writes) {
		regmap_write(pm6150_rm, USBIN_ADAPTER_ALLOW_CFG_REG,
			     USBIN_ADAPTER_ALLOW_5V_TO_12V);
		regmap_write(pm6150_rm, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
			     USBIN_ADAPTER_ALLOW_OVERRIDE_CONTINUOUS);
		regmap_write(pm6150_rm, USBIN_INT_LATCHED_CLR_REG, 0xFF);
		pr_info("pm6150_chgr: USBIN OVP window opened to 5-12 V (CONTINUOUS for PPS)\n");
	}

	was_plugged = is_usb_plugged();
	if (was_plugged) {
		mutex_lock(&apply_lock);
		pr_info("pm6150_chgr: cable already plugged in\n");
		if (enable_writes) {
			apply_charge_config();
			/* Same race-fix as plug_notify: classify the zone from
			 * a fresh temp read before letting charging start. */
			apply_zone_policy_locked();
		}
		snapshot_regs("post-init");
		mutex_unlock(&apply_lock);
	}

	plug_nb.notifier_call = plug_notify;
	rc = power_supply_reg_notifier(&plug_nb);
	if (rc) {
		release_pm6150();
		return rc;
	}

	INIT_DELAYED_WORK(&reassert_work, reassert_fn);
	if (reassert_period_ms > 0)
		schedule_delayed_work(&reassert_work,
			msecs_to_jiffies(reassert_period_ms));
	return 0;
}

/*
 * On rmmod restore the PMIC to a state close to its power-on default
 * so a `modprobe -r ; modprobe` cycle (or a permanent removal) doesn't
 * leave hardware in a configuration that needs us to keep running:
 *   - input not suspended (don't gate VBUS)
 *   - charging enabled (don't starve the cell if a cable is in)
 *   - ICL_OVERRIDE bits cleared so APSD-detected ICL takes over
 *   - hardware AICL re-enabled so the PMIC handles weak chargers
 *     adaptively without our supervision
 *
 * cancel_delayed_work_sync runs first so no late work can race the
 * register writes. release_pm6150 runs last so qg_psy stays valid
 * while we touch silicon.
 */
static void __exit pm6150_chgr_exit(void)
{
	cancel_delayed_work_sync(&reassert_work);
	power_supply_unreg_notifier(&plug_nb);

	if (enable_writes && pm6150_rm) {
		regmap_update_bits(pm6150_rm, USBIN_CMD_IL_REG,
				   USBIN_SUSPEND_BIT, 0);
		regmap_update_bits(pm6150_rm, USBIN_CMD_ICL_OVERRIDE_REG,
				   ICL_OVERRIDE_BIT, 0);
		regmap_update_bits(pm6150_rm, USBIN_LOAD_CFG_REG,
				   ICL_OVERRIDE_AFTER_APSD_BIT, 0);
		regmap_update_bits(pm6150_rm, USBIN_AICL_OPTIONS_CFG_REG,
				   USBIN_AICL_EN_BIT |
				   USBIN_AICL_PERIODIC_RERUN_EN_BIT |
				   SUSPEND_ON_COLLAPSE_USBIN_BIT,
				   USBIN_AICL_EN_BIT |
				   USBIN_AICL_PERIODIC_RERUN_EN_BIT |
				   SUSPEND_ON_COLLAPSE_USBIN_BIT);
		/*
		 * Restore CHARGING_ENABLE only if the cell isn't in JZ_HOT.
		 * Otherwise rmmod'ing during overtemp would hand a hot cell
		 * back to the bootloader with charging on — defeats the
		 * whole point of JZ_HOT.
		 */
		if (current_zone != JZ_HOT)
			regmap_update_bits(pm6150_rm, CHARGING_ENABLE_CMD_REG,
					   CHARGING_ENABLE_BIT,
					   CHARGING_ENABLE_BIT);
		else
			pr_warn("pm6150_chgr: leaving charging disabled (hot cell) on exit\n");
		pr_info("pm6150_chgr: restored PMIC defaults on exit\n");
	}

	release_pm6150();
}

module_init(pm6150_chgr_init);
module_exit(pm6150_chgr_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("alex / joyeuse mainline");
MODULE_DESCRIPTION("PM6150 SMB5 minimal charger init for Xiaomi joyeuse");
MODULE_VERSION("0.4");
MODULE_SOFTDEP("pre: qcom_qg");
