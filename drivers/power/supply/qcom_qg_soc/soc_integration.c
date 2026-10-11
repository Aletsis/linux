// SPDX-License-Identifier: GPL-2.0-only
#ifdef __KERNEL__
#include <linux/math64.h>
#include <linux/string.h>
#else
#include <string.h>
#endif
#include "soc_integration.h"

static qg_soc_u64 qg_soc_ms(qg_soc_u64 ns)
{
#ifdef __KERNEL__
	return div_u64(ns, 1000000);
#else
	return ns / 1000000;
#endif
}

void qg_soc_integration_init(struct qg_soc_integration *state, int design_uah)
{
	memset(state, 0, sizeof(*state));
	state->nominal_uah = design_uah;
	state->last_result = QG_SOC_NO_BASIS;
	qg_experimental_soc_init(&state->soc, design_uah);
}

void qg_soc_integration_gap(struct qg_soc_integration *state, qg_soc_u64 now_ns)
{
	qg_soc_recovery_gap(&state->recovery, &state->soc, qg_soc_ms(now_ns));
	state->history_count = 0;
	state->error_open = true;
	state->last_result = state->recovery.bound ? QG_SOC_GAP_STILL_OPEN : QG_SOC_NO_BASIS;
}

void qg_soc_integration_resume(struct qg_soc_integration *state,
			       qg_soc_u64 now_ns)
{
	qg_soc_recovery_close_gap(&state->recovery, &state->soc, qg_soc_ms(now_ns));
	state->gap_end_ns = now_ns;
	state->error_open = false;
	state->history_count = 0;
	state->last_result = state->recovery.bound ? QG_SOC_UNQUALIFIED_REFERENCE : QG_SOC_NO_BASIS;
}

static bool qg_soc_window_qualified(const struct qg_soc_integration *state,
				    const struct qg_soc_observation *observation)
{
	const struct qg_soc_environment *sample;
	qg_soc_u64 previous_ns = 0;
	unsigned int i;
	bool bracket = false;

	if (!observation->ocv_valid || !observation->sequence ||
	    !observation->begin_ns || observation->end_ns < observation->begin_ns ||
	    observation->end_ns > observation->environment.time_ns ||
	    observation->environment.time_ns - observation->begin_ns >
	    QG_SOC_REFERENCE_AGE_NS)
		return false;
	/* Every sampled segment covering the entire window must be qualified. */
	for (i = 0; i < state->history_count; i++) {
		sample = &state->history[i];
		if (sample->time_ns <= observation->begin_ns) {
			bracket = true;
			previous_ns = sample->time_ns;
			continue;
		}
		if (!bracket)
			return false;
		/* Include the preceding bracket sample as well. */
		if (!state->history[i - 1].valid || !sample->valid ||
		    state->history[i - 1].table_index != observation->environment.table_index ||
		    state->history[i - 1].phase != observation->environment.phase ||
		    sample->table_index != observation->environment.table_index ||
		    sample->phase != observation->environment.phase ||
		    sample->time_ns <= previous_ns ||
		    sample->time_ns - previous_ns > QG_SOC_HISTORY_GAP_NS)
			return false;
		previous_ns = sample->time_ns;
		if (sample->time_ns >= observation->end_ns)
			return true;
	}
	return false;
}

void qg_soc_integration_sample(struct qg_soc_integration *state,
			       const struct qg_soc_observation *observation)
{
	struct qg_soc_reference reference = {};
	qg_soc_u64 now_ns = observation->environment.time_ns;
	qg_soc_u64 now_ms = qg_soc_ms(now_ns);
	qg_soc_s64 relative, window;
	bool qualified;

	if (!observation->observed ||
	    (state->soc.valid && (now_ms <= state->soc.last_ms ||
	     now_ms - state->soc.last_ms > QG_SOC_MAX_GAP_MS))) {
		qg_soc_integration_gap(state, now_ns);
		if (!observation->observed)
			return;
	}
	if (state->error_open)
		qg_soc_integration_resume(state, now_ns);
	if (state->history_count == QG_SOC_HISTORY_SIZE) {
		memmove(state->history, state->history + 1,
			(QG_SOC_HISTORY_SIZE - 1) * sizeof(state->history[0]));
		state->history_count--;
	}
	state->history[state->history_count++] = observation->environment;
	qualified = qg_soc_window_qualified(state, observation);
	if (observation->floor_soc_milli < 0 || observation->floor_soc_milli > 100000 ||
	    observation->ocv_soc_milli < 0 || observation->ocv_soc_milli > 100000 ||
	    observation->floor_voltage_uv <= 0)
		qualified = false;
	reference.basis = observation->basis;
	reference.sequence = observation->sequence;
	reference.window_begin_ms = qg_soc_ms(observation->begin_ns);
	reference.window_end_ms = qg_soc_ms(observation->end_ns);
	reference.now_ms = now_ms;
	reference.current_ua = observation->current_ua;
	reference.observation_valid = observation->ocv_valid;
	reference.guard_valid = observation->observed;
	reference.temperature_qualified = qualified;
	reference.phase_qualified = qualified;
	reference.scenario_valid = qualified;
	if (qualified && observation->environment.phase == 1 &&
	    !observation->current_ua &&
	    (!state->recovery.bound ||
	     observation->sequence > state->recovery.last_sequence)) {
		/* Nominal scaling defines an operational Full-to-design-min window. */
		window = (qg_soc_s64)state->nominal_uah *
			(observation->ocv_soc_milli - observation->floor_soc_milli);
#ifdef __KERNEL__
		window = div_s64(window, 100000);
#else
		window /= 100000;
#endif
		if (window <= 0 || window > state->nominal_uah) {
			qg_soc_integration_gap(state, now_ns);
			state->recovery.bound = false;
			return;
		}
		qg_experimental_soc_init(&state->soc, window);
		qg_experimental_soc_sample(&state->soc, now_ms, 0, true, true);
		if (qg_soc_recovery_bind(&state->recovery, &state->soc, &reference)) {
			state->full_soc_milli = observation->ocv_soc_milli;
			state->floor_soc_milli = observation->floor_soc_milli;
			state->floor_voltage_uv = observation->floor_voltage_uv;
			state->full_table_index = observation->environment.table_index;
			state->gap_end_ns = 0;
			state->last_result = QG_SOC_NO_GAP;
		} else {
			qg_experimental_soc_invalidate(&state->soc);
		}
		return;
	}
	/*
	 * Real supplied Full defines an operative anchor before a new OCV exists.
	 * Without a qualified curve basis, its denominator is explicitly nominal.
	 * A later qualified Full OCV above replaces it only while measured I is zero.
	 */
	if (observation->environment.valid && observation->environment.phase == 1 &&
	    !observation->current_ua) {
		if (!state->recovery.bound && !state->soc.valid)
			qg_experimental_soc_init(&state->soc, state->nominal_uah);
		qg_experimental_soc_sample(&state->soc, now_ms, 0, true, true);
		return;
	}
	if (state->soc.valid) {
		qg_experimental_soc_sample(&state->soc, now_ms,
					   observation->current_ua, false, true);
		return;
	}
	if (!state->recovery.bound) {
		state->last_result = QG_SOC_NO_BASIS;
		return;
	}
	if (!observation->ocv_valid) {
		state->last_result = QG_SOC_UNQUALIFIED_REFERENCE;
		return;
	}
	if (!qualified) {
		/* Combined sampled table/thermal/phase bracket, not continuous proof. */
		state->last_result = QG_SOC_MISSING_TEMPERATURE;
		return;
	}
	if (observation->environment.phase != 2) {
		state->last_result = QG_SOC_MISSING_PHASE;
		return;
	}
	if (observation->environment.table_index != state->full_table_index ||
	    observation->floor_soc_milli != state->floor_soc_milli ||
	    observation->floor_voltage_uv != state->floor_voltage_uv) {
		state->last_result = QG_SOC_INVALID_SCENARIO;
		return;
	}
	if (observation->begin_ns <= state->gap_end_ns) {
		state->last_result = QG_SOC_WINDOW_CROSSES_GAP;
		return;
	}
	/* Relative charge still uses nominal scaling; percentage uses window_uah. */
	relative = (qg_soc_s64)(observation->ocv_soc_milli - state->full_soc_milli) *
		state->nominal_uah;
#ifdef __KERNEL__
	reference.relative_uah = div_s64(relative, 100000);
#else
	reference.relative_uah = relative / 100000;
#endif
	reference.scenario_valid = relative <= 0 &&
		observation->ocv_soc_milli >= state->floor_soc_milli;
	state->last_result = qg_soc_recover(&state->recovery, &state->soc,
					    &reference, QG_SOC_REFERENCE_AGE_NS / 1000000);
}
