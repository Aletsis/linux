/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOM_QG_SOC_INTEGRATION_H
#define QCOM_QG_SOC_INTEGRATION_H
#include "soc_recovery.h"

#define QG_SOC_HISTORY_SIZE 32
#define QG_SOC_HISTORY_GAP_NS 6000000000ULL
#define QG_SOC_REFERENCE_AGE_NS 120000000000ULL

struct qg_soc_environment {
	qg_soc_u64 time_ns;
	int table_index;
	int phase; /* 1 = supplied Full, 2 = unplugged discharge/rest. */
	bool valid;
};

struct qg_soc_observation {
	struct qg_soc_basis basis;
	struct qg_soc_environment environment;
	qg_soc_u64 sequence, begin_ns, end_ns;
	int current_ua, ocv_soc_milli, floor_soc_milli, floor_voltage_uv;
	bool observed, ocv_valid;
};

struct qg_soc_integration {
	struct qg_experimental_soc soc;
	struct qg_soc_recovery recovery;
	struct qg_soc_environment history[QG_SOC_HISTORY_SIZE];
	unsigned int history_count;
	qg_soc_u64 gap_end_ns;
	int nominal_uah, full_soc_milli, floor_soc_milli, floor_voltage_uv;
	int full_table_index;
	/* nominal_uah is a measured Full-to-cutoff capacity, not the design one. */
	bool nominal_measured;
	bool error_open;
	enum qg_soc_recovery_result last_result;
};

void qg_soc_integration_init(struct qg_soc_integration *state, int design_uah);
void qg_soc_integration_gap(struct qg_soc_integration *state, qg_soc_u64 now_ns);
void qg_soc_integration_resume(struct qg_soc_integration *state,
			       qg_soc_u64 now_ns);
void qg_soc_integration_sample(struct qg_soc_integration *state,
			       const struct qg_soc_observation *observation);
#endif
