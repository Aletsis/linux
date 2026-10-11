/* SPDX-License-Identifier: GPL-2.0-only */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM qcom_qg

#if !defined(_QCOM_QG_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _QCOM_QG_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(qcom_qg_fifo,
	TP_PROTO(const char *device, const struct qcom_qg_fifo_record *record),
	TP_ARGS(device, record),
	TP_STRUCT__entry(
		__string(device, device)
		__field(u64, epoch_ns)
		__field(u64, sequence)
		__field(u64, time_ns)
		__field(u64, elapsed_ns)
		__field(u64, duration_ns)
		__field(u32, flags)
		__field(int, error)
		__field(u16, samples)
		__field(u8, length)
		__field(u8, interval)
		__field(u8, control)
		__array(int, voltage_uv, QG_FIFO_MAX_SAMPLES)
		__array(int, current_ua, QG_FIFO_MAX_SAMPLES)
	),
	TP_fast_assign(
		__assign_str(device);
		__entry->epoch_ns = record->epoch_ns;
		__entry->sequence = record->sequence;
		__entry->time_ns = record->time_ns;
		__entry->elapsed_ns = record->elapsed_ns;
		__entry->duration_ns = record->duration_ns;
		__entry->flags = record->flags;
		__entry->error = record->error;
		__entry->samples = record->samples;
		__entry->length = record->length;
		__entry->interval = record->interval;
		__entry->control = record->control;
		memcpy(__entry->voltage_uv, record->voltage_uv, sizeof(record->voltage_uv));
		memcpy(__entry->current_ua, record->current_ua, sizeof(record->current_ua));
	),
	TP_printk("dev=%s epoch=%llu seq=%llu ns=%llu elapsed=%llu duration=%llu flags=%#x error=%d length=%u samples=%u interval=%u control=%#x voltage=%s current=%s",
		  __get_str(device), __entry->epoch_ns, __entry->sequence,
		  __entry->time_ns, __entry->elapsed_ns, __entry->duration_ns,
		  __entry->flags, __entry->error, __entry->length, __entry->samples,
		  __entry->interval, __entry->control,
		  __print_array(__entry->voltage_uv, QG_FIFO_MAX_SAMPLES, sizeof(int)),
		  __print_array(__entry->current_ua, QG_FIFO_MAX_SAMPLES, sizeof(int)))
);

TRACE_EVENT(qcom_qg_ocv,
	TP_PROTO(const char *device, const struct qcom_qg_ocv_record *record),
	TP_ARGS(device, record),
	TP_STRUCT__entry(
		__string(device, device)
		__field(u64, epoch_ns)
		__field(u64, cycle)
		__field(u64, begin_ns)
		__field(u64, end_ns)
		__field(u64, baseline_begin_ns)
		__field(u64, baseline_end_ns)
		__array(u64, op_begin_ns, QG_OCV_MAX_OPS)
		__array(u64, op_end_ns, QG_OCV_MAX_OPS)
		__array(int, error, QG_OCV_MAX_OPS)
		__field(u32, attempted)
		__field(u32, phase)
		__field(u32, trigger)
		__field(bool, armed)
		__field(bool, valid)
		__field(u64, sample_sequence)
		__field(u64, sample_begin_ns)
		__field(u64, sample_end_ns)
		__field(u64, age_min_ns)
		__field(u64, age_max_ns)
		__field(u32, result)
		__field(u8, status_before)
		__field(u8, status_after)
		__array(u8, raw, QG_OCV_DATA_SIZE)
		__array(u8, check, QG_OCV_DATA_SIZE)
		__field(int, voltage_uv)
		__field(int, current_ua)
	),
	TP_fast_assign(
		__assign_str(device);
		__entry->epoch_ns = record->epoch_ns;
		__entry->cycle = record->cycle;
		__entry->begin_ns = record->begin_ns;
		__entry->end_ns = record->end_ns;
		__entry->baseline_begin_ns = record->baseline_begin_ns;
		__entry->baseline_end_ns = record->baseline_end_ns;
		memcpy(__entry->op_begin_ns, record->op_begin_ns, sizeof(record->op_begin_ns));
		memcpy(__entry->op_end_ns, record->op_end_ns, sizeof(record->op_end_ns));
		memcpy(__entry->error, record->error, sizeof(record->error));
		__entry->attempted = record->attempted;
		__entry->phase = record->phase;
		__entry->trigger = record->trigger;
		__entry->armed = record->armed;
		__entry->valid = record->valid;
		__entry->sample_sequence = record->sample_sequence;
		__entry->sample_begin_ns = record->sample_begin_ns;
		__entry->sample_end_ns = record->sample_end_ns;
		__entry->age_min_ns = record->age_min_ns;
		__entry->age_max_ns = record->age_max_ns;
		__entry->result = record->result;
		__entry->status_before = record->status_before;
		__entry->status_after = record->status_after;
		memcpy(__entry->raw, record->raw, sizeof(record->raw));
		memcpy(__entry->check, record->check, sizeof(record->check));
		__entry->voltage_uv = record->voltage_uv;
		__entry->current_ua = record->current_ua;
	),
	TP_printk("dev=%s epoch=%llu cycle=%llu phase=%u result=%u begin=%llu end=%llu baseline_begin=%llu baseline_end=%llu attempted=%#x op_begin=%s op_end=%s error=%s status_before=%#x status_after=%#x raw=%s check=%s voltage=%d current=%d trigger=%u armed=%u valid=%u sample_seq=%llu sample_begin=%llu sample_end=%llu age_min=%llu age_max=%llu",
		  __get_str(device), __entry->epoch_ns, __entry->cycle,
		  __entry->phase, __entry->result, __entry->begin_ns, __entry->end_ns,
		  __entry->baseline_begin_ns, __entry->baseline_end_ns, __entry->attempted,
		  __print_array(__entry->op_begin_ns, QG_OCV_MAX_OPS, sizeof(u64)),
		  __print_array(__entry->op_end_ns, QG_OCV_MAX_OPS, sizeof(u64)),
		  __print_array(__entry->error, QG_OCV_MAX_OPS, sizeof(int)),
		  __entry->status_before, __entry->status_after,
		  __print_array(__entry->raw, QG_OCV_DATA_SIZE, sizeof(u8)),
		  __print_array(__entry->check, QG_OCV_DATA_SIZE, sizeof(u8)),
		  __entry->voltage_uv, __entry->current_ua,
		  __entry->trigger, __entry->armed, __entry->valid,
		  __entry->sample_sequence, __entry->sample_begin_ns,
		  __entry->sample_end_ns, __entry->age_min_ns, __entry->age_max_ns)
);

TRACE_EVENT(qcom_qg_ocv_guard,

	TP_PROTO(const char *device, const struct qcom_qg_ocv_guard_record *record),

	TP_ARGS(device, record),

	TP_STRUCT__entry(
		__string(device, device)
		__field(u64, epoch_ns)
		__field(u64, begin_ns)
		__field(u64, end_ns)
		__field(u32, trigger)
		__field(u32, reads)
		__field(int, error)
		__field(u8, status_before)
		__field(u8, status_after)
		__array(u8, config, QG_OCV_CONFIG_SIZE)
		__array(u8, reference, QG_OCV_CONFIG_SIZE)
		__field(bool, establish)
		__field(bool, valid_before)
		__field(bool, valid_after)
	),

	TP_fast_assign(
		__assign_str(device);
		__entry->epoch_ns = record->epoch_ns;
		__entry->begin_ns = record->begin_ns;
		__entry->end_ns = record->end_ns;
		__entry->trigger = record->trigger;
		__entry->reads = record->reads;
		__entry->error = record->error;
		__entry->status_before = record->status_before;
		__entry->status_after = record->status_after;
		memcpy(__entry->config, record->config, sizeof(record->config));
		memcpy(__entry->reference, record->reference, sizeof(record->reference));
		__entry->establish = record->establish;
		__entry->valid_before = record->valid_before;
		__entry->valid_after = record->valid_after;
	),

	TP_printk("dev=%s epoch=%llu begin=%llu end=%llu trigger=%u reads=%u error=%d status_before=%#x status_after=%#x config=%s reference=%s establish=%u valid_before=%u valid_after=%u",
		  __get_str(device), __entry->epoch_ns, __entry->begin_ns,
		  __entry->end_ns, __entry->trigger, __entry->reads, __entry->error,
		  __entry->status_before, __entry->status_after,
		  __print_array(__entry->config, QG_OCV_CONFIG_SIZE, sizeof(u8)),
		  __print_array(__entry->reference, QG_OCV_CONFIG_SIZE, sizeof(u8)),
		  __entry->establish, __entry->valid_before, __entry->valid_after)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../drivers/power/supply
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE qcom_qg_trace
#include <trace/define_trace.h>
