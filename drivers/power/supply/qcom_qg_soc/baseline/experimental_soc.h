/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef QCOM_QG_EXPERIMENTAL_SOC_H
#define QCOM_QG_EXPERIMENTAL_SOC_H
#ifdef __KERNEL__
#include <linux/types.h>
typedef s64 qg_soc_s64;
typedef u64 qg_soc_u64;
#else
#include <stdbool.h>
#include <stdint.h>
typedef int64_t qg_soc_s64;
typedef uint64_t qg_soc_u64;
#endif
#define QG_SOC_MAX_GAP_MS 60000
#define QG_SOC_TRAPEZOID_DIVISOR 7200000
/* Nominal-capacity, awake-only experimental state. Not learned capacity. */
struct qg_experimental_soc {
	int design_uah;
	bool valid;
	bool have_time;
	qg_soc_u64 last_ms;
	int previous_ua;
	qg_soc_s64 relative_uah;
	qg_soc_s64 remainder;
};

void qg_experimental_soc_init(struct qg_experimental_soc *soc, int design_uah);
void qg_experimental_soc_invalidate(struct qg_experimental_soc *soc);
/* full_zero means verified charger Full, supplied and measured CURRENT_NOW=0. */
bool qg_experimental_soc_sample(struct qg_experimental_soc *soc,
				qg_soc_u64 now_ms, int current_ua, bool full_zero,
				bool observation_valid);
int qg_experimental_soc_capacity(const struct qg_experimental_soc *soc);
#endif
