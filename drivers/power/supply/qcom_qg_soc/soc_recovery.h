/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOM_QG_SOC_RECOVERY_H
#define QCOM_QG_SOC_RECOVERY_H
#include "baseline/experimental_soc.h"

/* Caller supplies checked identity/configuration, not PMIC register guesses. */
struct qg_soc_basis {
	unsigned char boot_id[16];
	qg_soc_u64 driver_epoch_ms;
	unsigned char controls[10];
	unsigned char scenario_digest[32];
};

struct qg_soc_recovery {
	struct qg_soc_basis basis;
	qg_soc_u64 full_sequence;
	qg_soc_u64 last_sequence;
	qg_soc_u64 gap_begin_ms;
	qg_soc_u64 gap_end_ms;
	int design_uah;
	bool bound;
	bool gap;
	bool gap_closed;
};

/* A scenario is an explicit external hypothesis, never a learned capacity. */
struct qg_soc_reference {
	struct qg_soc_basis basis;
	qg_soc_u64 sequence;
	qg_soc_u64 window_begin_ms;
	qg_soc_u64 window_end_ms;
	qg_soc_u64 now_ms;
	int current_ua;
	qg_soc_s64 relative_uah;
	bool observation_valid;
	bool guard_valid;
	bool temperature_qualified;
	bool phase_qualified;
	bool scenario_valid;
};

enum qg_soc_recovery_result {
	QG_SOC_RECOVERED,
	QG_SOC_NO_BASIS,
	QG_SOC_NO_GAP,
	QG_SOC_GAP_STILL_OPEN,
	QG_SOC_IDENTITY_CHANGED,
	QG_SOC_OLD_REFERENCE,
	QG_SOC_WINDOW_CROSSES_GAP,
	QG_SOC_INVALID_TIME,
	QG_SOC_REFERENCE_TOO_OLD,
	QG_SOC_UNQUALIFIED_REFERENCE,
	QG_SOC_MISSING_TEMPERATURE,
	QG_SOC_MISSING_PHASE,
	QG_SOC_INVALID_SCENARIO,
};

/* Bind only a qualified Full OCV to the existing operative Full=100 anchor. */
bool qg_soc_recovery_bind(struct qg_soc_recovery *recovery,
			  const struct qg_experimental_soc *soc,
			  const struct qg_soc_reference *full_reference);
bool qg_soc_recovery_gap(struct qg_soc_recovery *recovery,
			 struct qg_experimental_soc *soc, qg_soc_u64 now_ms);
/* Mandatory at resume/end-of-error: OCV windows must follow this boundary. */
bool qg_soc_recovery_close_gap(struct qg_soc_recovery *recovery,
			       const struct qg_experimental_soc *soc,
			       qg_soc_u64 now_ms);
enum qg_soc_recovery_result
qg_soc_recover(struct qg_soc_recovery *recovery,
	       struct qg_experimental_soc *soc,
	       const struct qg_soc_reference *reference,
	       qg_soc_u64 maximum_age_ms);
#endif
