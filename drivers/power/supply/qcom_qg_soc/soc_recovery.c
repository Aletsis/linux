// SPDX-License-Identifier: GPL-2.0-only
#ifdef __KERNEL__
#include <linux/string.h>
#else
#include <string.h>
#endif
#include "soc_recovery.h"

static bool nonzero_identity(const unsigned char *bytes, unsigned int count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (bytes[i])
			return true;
	return false;
}

static bool same_basis(const struct qg_soc_basis *a,
		       const struct qg_soc_basis *b)
{
	/* Compare fields: padding bytes are not semantic identity. */
	return a->driver_epoch_ms == b->driver_epoch_ms &&
		!memcmp(a->boot_id, b->boot_id, sizeof(a->boot_id)) &&
		!memcmp(a->controls, b->controls, sizeof(a->controls)) &&
		!memcmp(a->scenario_digest, b->scenario_digest,
			 sizeof(a->scenario_digest));
}

bool qg_soc_recovery_bind(struct qg_soc_recovery *recovery,
			  const struct qg_experimental_soc *soc,
			  const struct qg_soc_reference *full_reference)
{
	memset(recovery, 0, sizeof(*recovery));
	if (!soc->valid || !soc->have_time || soc->design_uah <= 0 ||
	    soc->relative_uah || soc->previous_ua || !full_reference->sequence ||
	    !full_reference->basis.driver_epoch_ms ||
	    full_reference->window_begin_ms < full_reference->basis.driver_epoch_ms ||
	    full_reference->window_end_ms < full_reference->window_begin_ms ||
	    full_reference->window_end_ms > full_reference->now_ms ||
	    full_reference->now_ms != soc->last_ms || full_reference->current_ua ||
	    !full_reference->observation_valid || !full_reference->guard_valid ||
	    !full_reference->temperature_qualified || !full_reference->phase_qualified ||
	    !full_reference->scenario_valid || full_reference->relative_uah ||
	    !nonzero_identity(full_reference->basis.boot_id, 16) ||
	    !nonzero_identity(full_reference->basis.scenario_digest, 32))
		return false;
	recovery->basis = full_reference->basis;
	recovery->full_sequence = full_reference->sequence;
	recovery->last_sequence = full_reference->sequence;
	recovery->design_uah = soc->design_uah;
	recovery->bound = true;
	return true;
}

bool qg_soc_recovery_gap(struct qg_soc_recovery *recovery,
			 struct qg_experimental_soc *soc, qg_soc_u64 now_ms)
{
	/* Even an unbound state must not carry charge across a discontinuity. */
	qg_experimental_soc_invalidate(soc);
	if (!recovery->bound || !soc->have_time)
		return false;
	if (now_ms < soc->last_ms ||
	    (recovery->gap && now_ms < recovery->gap_end_ms)) {
		recovery->bound = false;
		return false;
	}
	/* Repeated PM/errors advance the boundary, never widen eligibility. */
	if (!recovery->gap || now_ms > recovery->gap_begin_ms)
		recovery->gap_begin_ms = now_ms;
	recovery->gap = true;
	recovery->gap_closed = false;
	recovery->gap_end_ms = now_ms;
	return true;
}

bool qg_soc_recovery_close_gap(struct qg_soc_recovery *recovery,
			       const struct qg_experimental_soc *soc,
			       qg_soc_u64 now_ms)
{
	if (!recovery->bound || !recovery->gap || soc->valid)
		return false;
	if (now_ms < recovery->gap_end_ms) {
		recovery->bound = false;
		return false;
	}
	recovery->gap_end_ms = now_ms;
	recovery->gap_closed = true;
	return true;
}

enum qg_soc_recovery_result
qg_soc_recover(struct qg_soc_recovery *recovery,
	       struct qg_experimental_soc *soc,
	       const struct qg_soc_reference *reference,
	       qg_soc_u64 maximum_age_ms)
{
	if (!recovery->bound || soc->design_uah <= 0)
		return QG_SOC_NO_BASIS;
	if (!recovery->gap || soc->valid)
		return QG_SOC_NO_GAP;
	if (!recovery->gap_closed)
		return QG_SOC_GAP_STILL_OPEN;
	if (!same_basis(&recovery->basis, &reference->basis) ||
	    recovery->design_uah != soc->design_uah)
		return QG_SOC_IDENTITY_CHANGED;
	if (reference->sequence <= recovery->last_sequence)
		return QG_SOC_OLD_REFERENCE;
	if (!reference->window_begin_ms ||
	    reference->window_end_ms < reference->window_begin_ms ||
	    reference->now_ms < reference->window_end_ms ||
	    reference->now_ms <= soc->last_ms)
		return QG_SOC_INVALID_TIME;
	if (reference->window_begin_ms <= recovery->gap_end_ms)
		return QG_SOC_WINDOW_CROSSES_GAP;
	/* Age includes the entire uncertain acquisition window. */
	if (!maximum_age_ms ||
	    reference->now_ms - reference->window_begin_ms > maximum_age_ms)
		return QG_SOC_REFERENCE_TOO_OLD;
	if (!reference->guard_valid || !reference->observation_valid)
		return QG_SOC_UNQUALIFIED_REFERENCE;
	if (!reference->temperature_qualified)
		return QG_SOC_MISSING_TEMPERATURE;
	if (!reference->phase_qualified)
		return QG_SOC_MISSING_PHASE;
	if (!reference->scenario_valid || reference->relative_uah > 0 ||
	    reference->relative_uah < -(qg_soc_s64)soc->design_uah)
		return QG_SOC_INVALID_SCENARIO;

	/* Reanchor at now. Never integrate old current across the missing span. */
	soc->relative_uah = reference->relative_uah;
	soc->remainder = 0;
	soc->last_ms = reference->now_ms;
	soc->previous_ua = reference->current_ua;
	soc->have_time = true;
	soc->valid = true;
	recovery->last_sequence = reference->sequence;
	recovery->gap = false;
	return QG_SOC_RECOVERED;
}
