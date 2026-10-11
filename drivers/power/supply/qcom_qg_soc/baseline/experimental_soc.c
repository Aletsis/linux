// SPDX-License-Identifier: GPL-2.0-only
#ifdef __KERNEL__
#include <linux/math64.h>
#include <linux/string.h>
#else
#include <string.h>
#endif
#include "experimental_soc.h"

void qg_experimental_soc_init(struct qg_experimental_soc *soc, int design_uah)
{
	memset(soc, 0, sizeof(*soc));
	soc->design_uah = design_uah;
}

void qg_experimental_soc_invalidate(struct qg_experimental_soc *soc)
{
	soc->valid = false;
	soc->remainder = 0;
}

bool qg_experimental_soc_sample(struct qg_experimental_soc *soc,
				qg_soc_u64 now_ms, int current_ua, bool full_zero,
				bool observation_valid)
{
	qg_soc_u64 dt_ms = soc->have_time ? now_ms - soc->last_ms : 0;
	qg_soc_s64 numerator, delta;
	bool clock_valid = !soc->have_time || now_ms > soc->last_ms;

	soc->last_ms = now_ms;
	soc->have_time = true;
	if (!observation_valid || !clock_valid || soc->design_uah <= 0) {
		qg_experimental_soc_invalidate(soc);
		return false;
	}
	if (full_zero && current_ua == 0) {
		soc->relative_uah = 0;
		soc->remainder = 0;
		soc->previous_ua = 0;
		soc->valid = true;
		return true;
	}
	if (!soc->valid || dt_ms > QG_SOC_MAX_GAP_MS) {
		qg_experimental_soc_invalidate(soc);
		return false;
	}
	/* Two signed-int currents * <=60000 ms fit signed 64 bits. */
	numerator = ((qg_soc_s64)soc->previous_ua + current_ua) * (qg_soc_s64)dt_ms;
	numerator += soc->remainder;
#ifdef __KERNEL__
	delta = div_s64(numerator, QG_SOC_TRAPEZOID_DIVISOR);
#else
	delta = numerator / QG_SOC_TRAPEZOID_DIVISOR;
#endif
	soc->remainder = numerator - delta * QG_SOC_TRAPEZOID_DIVISOR;
	soc->relative_uah += delta;
	/* Operational coordinate bounded by nominal capacity, not learning. */
	if (soc->relative_uah > 0) {
		soc->relative_uah = 0;
		soc->remainder = 0;
	} else if (soc->relative_uah < -(qg_soc_s64)soc->design_uah) {
		soc->relative_uah = -(qg_soc_s64)soc->design_uah;
		soc->remainder = 0;
	}
	soc->previous_ua = current_ua;
	return true;
}

int qg_experimental_soc_capacity(const struct qg_experimental_soc *soc)
{
	qg_soc_s64 numerator;

	if (!soc->valid)
		return -1;
	/* design_uah is a positive signed int: this product fits signed 64. */
	numerator = ((qg_soc_s64)soc->design_uah + soc->relative_uah) * 100;
	numerator += soc->design_uah / 2;
#ifdef __KERNEL__
	return div_s64(numerator, soc->design_uah);
#else
	return numerator / soc->design_uah;
#endif
}
