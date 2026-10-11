// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Danila Tikhonov <danila@jiaxyga.com>
 */

#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define QG_FIFO_CONTROL_REG		0x51
#define QG_FIFO_DATA_REG			0x90
#define QG_FIFO_MAX_SAMPLES		8
#define QG_FIFO_RESET_VALUE		0x8000

/* Diagnostic flags: matching cadence does not prove lossless acquisition. */
#define QG_FIFO_FIRST			BIT(0)
#define QG_FIFO_EARLY			BIT(1)
#define QG_FIFO_LATE			BIT(2)
#define QG_FIFO_CONFIG_CHANGED		BIT(3)
#define QG_FIFO_READ_ERROR		BIT(4)
#define QG_FIFO_SUSPEND			BIT(5)
#define QG_FIFO_RESUME			BIT(6)

struct qcom_qg_fifo_record {
	u64 epoch_ns;
	u64 sequence;
	u64 time_ns;
	u64 elapsed_ns;
	u64 duration_ns;
	u32 flags;
	int error;
	u16 samples;
	u8 length;
	u8 interval;
	u8 control;
	int voltage_uv[QG_FIFO_MAX_SAMPLES];
	int current_ua[QG_FIFO_MAX_SAMPLES];
};

#define QG_STATUS2_REG			0x09
#define QG_GOOD_OCV_BIT			BIT(1)
#define QG_GOOD_OCV_DATA_REG		0x74
#define QG_OCV_DATA_SIZE			4
#define QG_OCV_MAX_OPS			5
#define QG_STATUS1_REG			0x08
#define QG_BATTERY_PRESENT_BIT		BIT(0)
#define QG_OK_BIT			BIT(7)
#define QG_OCV_CONFIG_SIZE		10

/* Read-only acquisition controls; never read STATUS2 through this path. */
static const u8 qcom_qg_ocv_config_regs[QG_OCV_CONFIG_SIZE] = {
	0x43, 0x51, 0x52, 0x59, 0x5c, 0x5d, 0x5e, 0x5f, 0x60, 0x64,
};

struct qcom_qg_ocv_guard_record {
	u64 epoch_ns;
	u64 begin_ns;
	u64 end_ns;
	u32 trigger;
	u32 reads;
	int error;
	u8 status_before;
	u8 status_after;
	u8 config[QG_OCV_CONFIG_SIZE];
	u8 reference[QG_OCV_CONFIG_SIZE];
	bool establish;
	bool valid_before;
	bool valid_after;
};

enum qcom_qg_ocv_phase {
	QG_OCV_PREPARE = 1,
	QG_OCV_OBSERVE,
	QG_OCV_STATE,
};

enum qcom_qg_ocv_trigger {
	QG_OCV_PROBE = 1,
	QG_OCV_IRQ,
	QG_OCV_SUSPEND,
	QG_OCV_RESUME,
	QG_OCV_REMOVE,
	QG_OCV_FIFO,
	QG_OCV_GETTER,
};

enum qcom_qg_ocv_result {
	QG_OCV_IO_ERROR = 1,
	QG_OCV_BASELINE_HIGH,
	QG_OCV_ARMED,
	QG_OCV_NO_BASELINE,
	QG_OCV_NO_EVENT,
	QG_OCV_STATUS_INCONSISTENT,
	QG_OCV_UNSTABLE_DATA,
	QG_OCV_INVALID_DATA,
	QG_OCV_FRESH_CANDIDATE,
	QG_OCV_PM_BOUNDARY,
	QG_OCV_STOPPED,
	QG_OCV_FIFO_INVALIDATED,
	QG_OCV_ENV_INVALIDATED,
};

struct qcom_qg_ocv_record {
	u64 epoch_ns;
	u64 cycle;
	u64 begin_ns;
	u64 end_ns;
	u64 baseline_begin_ns;
	u64 baseline_end_ns;
	u64 op_begin_ns[QG_OCV_MAX_OPS];
	u64 op_end_ns[QG_OCV_MAX_OPS];
	int error[QG_OCV_MAX_OPS];
	u32 attempted;
	u32 phase;
	u32 trigger;
	u32 result;
	bool armed;
	bool valid;
	u64 sample_sequence;
	u64 sample_begin_ns;
	u64 sample_end_ns;
	u64 age_min_ns;
	u64 age_max_ns;
	u8 status_before;
	u8 status_after;
	u8 raw[QG_OCV_DATA_SIZE];
	u8 check[QG_OCV_DATA_SIZE];
	int voltage_uv;
	int current_ua;
};

#define CREATE_TRACE_POINTS
#include "qcom_qg_trace.h"

/* BATT offsets */
#define QG_PERPH_SUBTYPE_REG		0x05
#define QG_ADC_IBAT_5A			0x03
#define QG_ADC_IBAT_10A			0x04

#define QG_S2_NORMAL_AVG_V_DATA0_REG	0x80 /* 2-byte 0x80-0x81 */
#define QG_S2_NORMAL_AVG_I_DATA0_REG	0x82 /* 2-byte 0x82-0x83 */
#define QG_LAST_ADC_V_DATA0_REG		0xc0 /* 2-byte 0xc0-0xc1 */
#define QG_LAST_ADC_I_DATA0_REG		0xc2 /* 2-byte 0xc2-0xc3 */

/* SRAM offsets */
#define QG_SDAM_LEARNED_CAPACITY_OFFSET	0x68 /* 2-byte 0x68-0x69 */

struct qcom_qg_chip {
	struct device *dev;
	struct regmap *regmap;
	unsigned int base;
	unsigned int current_scale_na;

	struct iio_channel *batt_therm_chan;

	struct nvmem_device *sdam;

	struct power_supply *batt_psy;
	struct power_supply_battery_info *batt_info;
	/* Serializes FIFO records, PM boundaries and OCV acknowledgment. */
	struct mutex fifo_lock;
	u64 fifo_epoch_ns;
	u64 fifo_sequence;
	u64 fifo_previous_ns;
	u8 fifo_control;
	u8 fifo_interval;
	bool fifo_previous_valid;
	u64 ocv_cycle;
	u64 ocv_baseline_begin_ns;
	u64 ocv_baseline_end_ns;
	bool ocv_armed;
	bool ocv_valid;
	u64 ocv_sample_sequence;
	u64 ocv_sample_begin_ns;
	u64 ocv_sample_end_ns;
	int ocv_voltage_uv;
	int ocv_current_ua;
	int ocv_irq;
	bool ocv_irq_enabled;
	bool ocv_suspended;
	bool ocv_stopping;
	u8 ocv_config[QG_OCV_CONFIG_SIZE];
	bool ocv_config_valid;
	bool ocv_notify_pending;
};

static void qcom_qg_ocv_invalidate(struct qcom_qg_chip *chip)
{
	lockdep_assert_held(&chip->fifo_lock);
	if (chip->ocv_valid)
		chip->ocv_notify_pending = true;
	chip->ocv_armed = false;
	chip->ocv_valid = false;
	chip->ocv_config_valid = false;
}

static void qcom_qg_ocv_notify(struct qcom_qg_chip *chip)
{
	bool notify;

	mutex_lock(&chip->fifo_lock);
	notify = chip->ocv_notify_pending && !chip->ocv_stopping;
	chip->ocv_notify_pending = false;
	mutex_unlock(&chip->fifo_lock);
	/* A getter may invalidate once; reading the resulting uevent cannot loop. */
	if (notify && chip->batt_psy)
		power_supply_changed(chip->batt_psy);
}

static int qcom_qg_ocv_guard(struct qcom_qg_chip *chip, unsigned int trigger,
			     bool establish)
{
	struct qcom_qg_ocv_guard_record record = {};
	unsigned int i;
	int ret;

	lockdep_assert_held(&chip->fifo_lock);
	record.epoch_ns = chip->fifo_epoch_ns;
	record.begin_ns = ktime_get_boottime_ns();
	record.trigger = trigger;
	record.establish = establish;
	record.valid_before = chip->ocv_valid;
	memcpy(record.reference, chip->ocv_config, sizeof(record.reference));
	record.reads++;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_STATUS1_REG,
			       &record.status_before, 1);
	if (ret)
		goto out;
	if ((record.status_before & (QG_BATTERY_PRESENT_BIT | QG_OK_BIT)) !=
	    (QG_BATTERY_PRESENT_BIT | QG_OK_BIT)) {
		ret = -ENODATA;
		goto out;
	}
	for (i = 0; i < QG_OCV_CONFIG_SIZE; i++) {
		record.reads++;
		ret = regmap_bulk_read(chip->regmap,
				       chip->base + qcom_qg_ocv_config_regs[i],
				       &record.config[i], 1);
		if (ret)
			goto out;
	}
	record.reads++;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_STATUS1_REG,
			       &record.status_after, 1);
	if (ret)
		goto out;
	if ((record.status_after & (QG_BATTERY_PRESENT_BIT | QG_OK_BIT)) !=
	    (QG_BATTERY_PRESENT_BIT | QG_OK_BIT)) {
		ret = -ENODATA;
		goto out;
	}
	if (chip->ocv_config_valid) {
		if (memcmp(chip->ocv_config, record.config, sizeof(record.config)))
			ret = -ENODATA;
	} else if (establish) {
		memcpy(chip->ocv_config, record.config, sizeof(record.config));
		chip->ocv_config_valid = true;
	} else {
		ret = -ENODATA;
	}
out:
	if (ret)
		qcom_qg_ocv_invalidate(chip);
	record.error = ret;
	record.valid_after = chip->ocv_valid;
	record.end_ns = ktime_get_boottime_ns();
	trace_qcom_qg_ocv_guard(dev_name(chip->dev), &record);
	return ret;
}

static int qcom_qg_get_ocv(struct qcom_qg_chip *chip, int *val)
{
	int ret = -ENODATA;

	mutex_lock(&chip->fifo_lock);
	if (!chip->ocv_valid || chip->ocv_stopping || chip->ocv_suspended)
		goto out;
	ret = qcom_qg_ocv_guard(chip, QG_OCV_GETTER, false);
	if (!ret)
		*val = chip->ocv_voltage_uv;
out:
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_notify(chip);
	return ret;
}

static void qcom_qg_ocv_trace(struct qcom_qg_chip *chip,
			      struct qcom_qg_ocv_record *record)
{
	lockdep_assert_held(&chip->fifo_lock);
	record->end_ns = ktime_get_boottime_ns();
	record->armed = chip->ocv_armed;
	record->valid = chip->ocv_valid;
	record->sample_sequence = chip->ocv_sample_sequence;
	record->sample_begin_ns = chip->ocv_sample_begin_ns;
	record->sample_end_ns = chip->ocv_sample_end_ns;
	if (chip->ocv_sample_sequence) {
		record->age_min_ns = record->end_ns - chip->ocv_sample_end_ns;
		record->age_max_ns = record->end_ns - chip->ocv_sample_begin_ns;
	}
	trace_qcom_qg_ocv(dev_name(chip->dev), record);
}

static void qcom_qg_ocv_state(struct qcom_qg_chip *chip, unsigned int trigger,
			      unsigned int result)
{
	struct qcom_qg_ocv_record record = {
		.phase = QG_OCV_STATE,
		.trigger = trigger,
		.result = result,
	};

	lockdep_assert_held(&chip->fifo_lock);
	record.epoch_ns = chip->fifo_epoch_ns;
	record.cycle = chip->ocv_cycle;
	record.begin_ns = ktime_get_boottime_ns();
	record.baseline_begin_ns = chip->ocv_baseline_begin_ns;
	record.baseline_end_ns = chip->ocv_baseline_end_ns;
	record.voltage_uv = chip->ocv_voltage_uv;
	record.current_ua = chip->ocv_current_ua;
	qcom_qg_ocv_trace(chip, &record);
}

static int qcom_qg_ocv_read(struct qcom_qg_chip *chip,
			    struct qcom_qg_ocv_record *record, unsigned int op,
			    unsigned int offset, void *data, size_t size)
{
	lockdep_assert_held(&chip->fifo_lock);
	record->attempted |= BIT(op);
	record->op_begin_ns[op] = ktime_get_boottime_ns();
	record->error[op] = regmap_bulk_read(chip->regmap, chip->base + offset,
					     data, size);
	record->op_end_ns[op] = ktime_get_boottime_ns();

	return record->error[op];
}

static int qcom_qg_ocv_ack(struct qcom_qg_chip *chip,
			   struct qcom_qg_ocv_record *record)
{
	lockdep_assert_held(&chip->fifo_lock);
	/* Vendor qg_read() explicitly writes zero after reading STATUS2. */
	record->attempted |= BIT(3);
	record->op_begin_ns[3] = ktime_get_boottime_ns();
	record->error[3] = regmap_write(chip->regmap,
					chip->base + QG_STATUS2_REG, 0);
	record->op_end_ns[3] = ktime_get_boottime_ns();

	return record->error[3];
}

static void qcom_qg_ocv_prepare(struct qcom_qg_chip *chip, unsigned int trigger)
{
	struct qcom_qg_ocv_record record = {
		.phase = QG_OCV_PREPARE,
		.trigger = trigger,
		.result = QG_OCV_IO_ERROR,
	};
	int ret;

	lockdep_assert_held(&chip->fifo_lock);
	chip->ocv_armed = false;
	chip->ocv_baseline_begin_ns = 0;
	chip->ocv_baseline_end_ns = 0;
	record.epoch_ns = chip->fifo_epoch_ns;
	record.cycle = ++chip->ocv_cycle;
	record.begin_ns = ktime_get_boottime_ns();
	if (qcom_qg_ocv_guard(chip, trigger, true)) {
		record.result = QG_OCV_ENV_INVALIDATED;
		goto out;
	}
	ret = qcom_qg_ocv_read(chip, &record, 0, QG_STATUS2_REG,
			       &record.status_before, 1);
	if (ret)
		goto out;
	ret = qcom_qg_ocv_read(chip, &record, 1, QG_GOOD_OCV_DATA_REG,
			       record.raw, sizeof(record.raw));
	if (ret)
		goto out;
	ret = qcom_qg_ocv_read(chip, &record, 2, QG_GOOD_OCV_DATA_REG,
			       record.check, sizeof(record.check));
	if (ret)
		goto out;
	ret = qcom_qg_ocv_ack(chip, &record);
	if (ret)
		goto out;
	ret = qcom_qg_ocv_read(chip, &record, 4, QG_STATUS2_REG,
			       &record.status_after, 1);
	if (ret)
		goto out;
	if (record.status_after & QG_GOOD_OCV_BIT) {
		record.result = QG_OCV_BASELINE_HIGH;
		goto out;
	}
	/* The low bit was observed somewhere inside this read, not at its end. */
	record.baseline_begin_ns = record.op_begin_ns[4];
	record.baseline_end_ns = record.op_end_ns[4];
	chip->ocv_baseline_begin_ns = record.baseline_begin_ns;
	chip->ocv_baseline_end_ns = record.baseline_end_ns;
	if (qcom_qg_ocv_guard(chip, trigger, false)) {
		record.result = QG_OCV_ENV_INVALIDATED;
		goto out;
	}
	chip->ocv_armed = true;
	record.result = QG_OCV_ARMED;
out:
	if (record.result != QG_OCV_ARMED)
		qcom_qg_ocv_invalidate(chip);
	qcom_qg_ocv_trace(chip, &record);
}

static unsigned int qcom_qg_ocv_observe(struct qcom_qg_chip *chip,
					unsigned int trigger)
{
	struct qcom_qg_ocv_record record = {
		.phase = QG_OCV_OBSERVE,
		.trigger = trigger,
		.result = QG_OCV_NO_BASELINE,
	};
	unsigned int voltage;
	s16 raw_current;
	bool armed;
	int ret;

	lockdep_assert_held(&chip->fifo_lock);
	armed = chip->ocv_armed;
	/* Keep the one-shot PM behavior when the optional IRQ is absent. */
	if (!chip->ocv_irq)
		chip->ocv_armed = false;
	record.epoch_ns = chip->fifo_epoch_ns;
	record.cycle = chip->ocv_cycle;
	record.begin_ns = ktime_get_boottime_ns();
	record.baseline_begin_ns = chip->ocv_baseline_begin_ns;
	record.baseline_end_ns = chip->ocv_baseline_end_ns;
	if (!armed)
		goto out;
	if (qcom_qg_ocv_guard(chip, trigger, false)) {
		record.result = QG_OCV_ENV_INVALIDATED;
		goto out;
	}
	record.result = QG_OCV_IO_ERROR;
	ret = qcom_qg_ocv_read(chip, &record, 0, QG_STATUS2_REG,
			       &record.status_before, 1);
	if (ret)
		goto out;
	if (!(record.status_before & QG_GOOD_OCV_BIT)) {
		record.result = qcom_qg_ocv_guard(chip, trigger, false) ?
			QG_OCV_ENV_INVALIDATED : QG_OCV_NO_EVENT;
		goto out;
	}
	ret = qcom_qg_ocv_read(chip, &record, 1, QG_GOOD_OCV_DATA_REG,
			       record.raw, sizeof(record.raw));
	if (ret)
		goto out;
	ret = qcom_qg_ocv_read(chip, &record, 2, QG_GOOD_OCV_DATA_REG,
			       record.check, sizeof(record.check));
	if (ret)
		goto out;
	ret = qcom_qg_ocv_read(chip, &record, 3, QG_STATUS2_REG,
			       &record.status_after, 1);
	if (ret)
		goto out;
	if (!(record.status_after & QG_GOOD_OCV_BIT)) {
		record.result = QG_OCV_STATUS_INCONSISTENT;
		goto out;
	}
	if (memcmp(record.raw, record.check, sizeof(record.raw))) {
		record.result = QG_OCV_UNSTABLE_DATA;
		goto out;
	}
	voltage = get_unaligned_le16(record.raw);
	raw_current = get_unaligned_le16(record.raw + 2);
	if (!voltage || voltage == QG_FIFO_RESET_VALUE ||
	    (u16)raw_current == QG_FIFO_RESET_VALUE) {
		record.result = QG_OCV_INVALID_DATA;
		goto out;
	}
	record.voltage_uv = div_u64((u64)voltage * 194637, 1000);
	record.current_ua = -div_s64((s64)raw_current * chip->current_scale_na, 1000);
	if (qcom_qg_ocv_guard(chip, trigger, false)) {
		record.result = QG_OCV_ENV_INVALIDATED;
		goto out;
	}
	record.result = QG_OCV_FRESH_CANDIDATE;
	chip->ocv_armed = false;
	chip->ocv_valid = true;
	chip->ocv_notify_pending = true;
	chip->ocv_sample_sequence++;
	chip->ocv_sample_begin_ns = record.baseline_begin_ns;
	chip->ocv_sample_end_ns = ktime_get_boottime_ns();
	chip->ocv_voltage_uv = record.voltage_uv;
	chip->ocv_current_ua = record.current_ua;
out:
	if (record.result != QG_OCV_NO_EVENT &&
	    record.result != QG_OCV_FRESH_CANDIDATE)
		qcom_qg_ocv_invalidate(chip);
	qcom_qg_ocv_trace(chip, &record);

	return record.result;
}

static void qcom_qg_ocv_service(struct qcom_qg_chip *chip, unsigned int trigger)
{
	unsigned int result;

	lockdep_assert_held(&chip->fifo_lock);
	if (!chip->ocv_armed) {
		/* An unreferenced pending value cannot become a fresh sample. */
		qcom_qg_ocv_prepare(chip, trigger);
		return;
	}
	result = qcom_qg_ocv_observe(chip, trigger);
	if (result == QG_OCV_FRESH_CANDIDATE)
		qcom_qg_ocv_prepare(chip, trigger);
	/* No-event IRQs keep the baseline; errors wait for the next IRQ or PM. */
}

static irqreturn_t qcom_qg_ocv_irq(int irq, void *data)
{
	struct qcom_qg_chip *chip = data;

	mutex_lock(&chip->fifo_lock);
	if (!chip->ocv_suspended && !chip->ocv_stopping)
		qcom_qg_ocv_service(chip, QG_OCV_IRQ);
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_notify(chip);

	return IRQ_HANDLED;
}

static void qcom_qg_ocv_irq_disable(struct qcom_qg_chip *chip)
{
	/* The threaded handler takes fifo_lock: drain it without that lock. */
	if (chip->ocv_irq_enabled) {
		disable_irq(chip->ocv_irq);
		chip->ocv_irq_enabled = false;
	}
}

static void qcom_qg_ocv_irq_enable(struct qcom_qg_chip *chip)
{
	if (chip->ocv_irq && !chip->ocv_irq_enabled && !chip->ocv_stopping) {
		chip->ocv_irq_enabled = true;
		enable_irq(chip->ocv_irq);
	}
}

static void qcom_qg_ocv_stop(void *data)
{
	struct qcom_qg_chip *chip = data;

	qcom_qg_ocv_irq_disable(chip);
	mutex_lock(&chip->fifo_lock);
	chip->ocv_stopping = true;
	qcom_qg_ocv_invalidate(chip);
	chip->ocv_notify_pending = false;
	qcom_qg_ocv_state(chip, QG_OCV_REMOVE, QG_OCV_STOPPED);
	mutex_unlock(&chip->fifo_lock);
}

static int qcom_qg_ocv_init(struct platform_device *pdev, struct qcom_qg_chip *chip)
{
	int ret, irq;

	irq = platform_get_irq_byname_optional(pdev, "good-ocv");
	if (irq == -ENXIO)
		return 0;
	if (irq <= 0)
		return dev_err_probe(chip->dev, irq ?: -EINVAL, "Failed to get OCV IRQ\n");
	chip->ocv_irq = irq;
	ret = devm_request_threaded_irq(chip->dev, irq, NULL, qcom_qg_ocv_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, "qg-good-ocv", chip);
	if (ret)
		return dev_err_probe(chip->dev, ret, "Failed to request OCV IRQ\n");
	/* Runs before devres frees this IRQ or the older device resources. */
	ret = devm_add_action_or_reset(chip->dev, qcom_qg_ocv_stop, chip);
	if (ret)
		return ret;
	mutex_lock(&chip->fifo_lock);
	qcom_qg_ocv_prepare(chip, QG_OCV_PROBE);
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_notify(chip);
	qcom_qg_ocv_irq_enable(chip);

	return 0;
}

static int qcom_qg_decode_fifo(struct qcom_qg_chip *chip,
			       struct qcom_qg_fifo_record *record,
			       const u8 *config, const u8 *raw)
{
	unsigned int voltage;
	s16 raw_current;
	int i;

	if (!config[1])
		return -EINVAL;

	record->length = ((config[0] >> 3) & 7) + 1;
	record->samples = 1U << ((config[0] & 7) + 1);
	record->control = config[0];
	record->interval = config[1];
	/* PM6150 clock correction; retain precision until the full batch. */
	record->duration_ns = div_u64((u64)record->length * record->samples *
				     record->interval * 10000000ULL * 32000, 32764);
	for (i = 0; i < record->length; i++) {
		voltage = get_unaligned_le16(raw + i * 2);
		raw_current = get_unaligned_le16(raw + 16 + i * 2);
		if (voltage == QG_FIFO_RESET_VALUE || (u16)raw_current == QG_FIFO_RESET_VALUE)
			return -ENODATA;
		record->voltage_uv[i] = div_u64((u64)voltage * 194637, 1000);
		record->current_ua[i] = -div_s64((s64)raw_current * chip->current_scale_na, 1000);
	}

	return 0;
}

static int qcom_qg_read_fifo(struct qcom_qg_chip *chip,
			     struct qcom_qg_fifo_record *record)
{
	u8 config[2], config_after[2], raw[32], check[32];
	int ret;

	ret = regmap_bulk_read(chip->regmap, chip->base + QG_FIFO_CONTROL_REG,
			       config, sizeof(config));
	if (ret)
		return ret;
	/*
	 * Completed FIFO banks are read on fifo-done, as in the vendor driver.
	 * Do not toggle MASTER_HOLD: releasing it restarts acquisition.
	 * Repeated reads reject observable changes, not all possible races.
	 */
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_FIFO_DATA_REG,
			       raw, sizeof(raw));
	if (ret)
		return ret;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_FIFO_DATA_REG,
			       check, sizeof(check));
	if (ret)
		return ret;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_FIFO_CONTROL_REG,
			       config_after, sizeof(config_after));
	if (ret)
		return ret;
	if (memcmp(config, config_after, sizeof(config)) || memcmp(raw, check, sizeof(raw)))
		return -EAGAIN;

	return qcom_qg_decode_fifo(chip, record, config, raw);
}

static void qcom_qg_fifo_timing(struct qcom_qg_chip *chip,
				struct qcom_qg_fifo_record *record)
{
	u64 margin = record->duration_ns / 10;

	if (!chip->fifo_previous_valid) {
		record->flags |= QG_FIFO_FIRST;
	} else {
		record->elapsed_ns = record->time_ns - chip->fifo_previous_ns;
		if (record->control != chip->fifo_control ||
		    record->interval != chip->fifo_interval)
			record->flags |= QG_FIFO_CONFIG_CHANGED;
		else if (record->elapsed_ns < record->duration_ns - margin)
			record->flags |= QG_FIFO_EARLY;
		else if (record->elapsed_ns > record->duration_ns + margin)
			record->flags |= QG_FIFO_LATE;
	}
	chip->fifo_previous_ns = record->time_ns;
	chip->fifo_control = record->control;
	chip->fifo_interval = record->interval;
	chip->fifo_previous_valid = true;
}

static irqreturn_t qcom_qg_fifo_irq(int irq, void *data)
{
	struct qcom_qg_chip *chip = data;
	struct qcom_qg_fifo_record record = {};

	mutex_lock(&chip->fifo_lock);
	record.epoch_ns = chip->fifo_epoch_ns;
	record.sequence = ++chip->fifo_sequence;
	record.time_ns = ktime_get_boottime_ns();
	record.error = qcom_qg_read_fifo(chip, &record);
	if (record.error) {
		record.flags = QG_FIFO_READ_ERROR;
		chip->fifo_previous_valid = false;
	} else {
		qcom_qg_fifo_timing(chip, &record);
	}
	trace_qcom_qg_fifo(dev_name(chip->dev), &record);
	if (record.flags & (QG_FIFO_READ_ERROR | QG_FIFO_CONFIG_CHANGED)) {
		qcom_qg_ocv_invalidate(chip);
		qcom_qg_ocv_state(chip, QG_OCV_FIFO, QG_OCV_FIFO_INVALIDATED);
	}
	if (chip->ocv_armed || chip->ocv_valid)
		qcom_qg_ocv_guard(chip, QG_OCV_FIFO, false);
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_notify(chip);

	return IRQ_HANDLED;
}

static void qcom_qg_fifo_boundary(struct qcom_qg_chip *chip, u32 flag)
{
	struct qcom_qg_fifo_record record = {};

	lockdep_assert_held(&chip->fifo_lock);
	chip->fifo_previous_valid = false;
	record.epoch_ns = chip->fifo_epoch_ns;
	record.sequence = ++chip->fifo_sequence;
	record.time_ns = ktime_get_boottime_ns();
	record.flags = flag;
	trace_qcom_qg_fifo(dev_name(chip->dev), &record);
}

static int qcom_qg_suspend(struct device *dev)
{
	struct qcom_qg_chip *chip = dev_get_drvdata(dev);

	qcom_qg_ocv_irq_disable(chip);
	mutex_lock(&chip->fifo_lock);
	chip->ocv_suspended = true;
	qcom_qg_fifo_boundary(chip, QG_FIFO_SUSPEND);
	if (!chip->ocv_irq || !chip->ocv_armed)
		qcom_qg_ocv_prepare(chip, QG_OCV_SUSPEND);
	else if (!qcom_qg_ocv_guard(chip, QG_OCV_SUSPEND, false))
		qcom_qg_ocv_state(chip, QG_OCV_SUSPEND, QG_OCV_PM_BOUNDARY);
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_notify(chip);
	return 0;
}

static int qcom_qg_resume(struct device *dev)
{
	struct qcom_qg_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->fifo_lock);
	qcom_qg_fifo_boundary(chip, QG_FIFO_RESUME);
	if (chip->ocv_irq)
		qcom_qg_ocv_service(chip, QG_OCV_RESUME);
	else
		qcom_qg_ocv_observe(chip, QG_OCV_RESUME);
	chip->ocv_suspended = false;
	/* Re-advertise a cached value hidden from getters during suspension. */
	if (chip->ocv_valid)
		chip->ocv_notify_pending = true;
	mutex_unlock(&chip->fifo_lock);
	qcom_qg_ocv_irq_enable(chip);
	qcom_qg_ocv_notify(chip);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_qg_pm_ops, qcom_qg_suspend, qcom_qg_resume);

static int qcom_qg_get_current(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	s16 temp;
	u8 readval[2];
	int ret;

	ret = regmap_bulk_read(chip->regmap, chip->base + offset, readval, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	temp = (s16)(readval[1] << 8 | readval[0]);
	/* Invert sign: positive = charging, negative = discharging per Linux ABI */
	*val = -div_s64((s64)temp * chip->current_scale_na, 1000);

	return 0;
}

static int qcom_qg_get_voltage(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	int ret, temp;
	u8 readval[2];

	ret = regmap_bulk_read(chip->regmap, chip->base + offset, readval, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read voltage: %d\n", ret);
		return ret;
	}

	temp = readval[1] << 8 | readval[0];
	*val = div_u64((u64)temp * 194637, 1000);

	return 0;
}

static int qcom_qg_get_status(struct qcom_qg_chip *chip, int *val)
{
	union power_supply_propval status;
	int ret;

	ret = power_supply_get_property_from_supplier(chip->batt_psy,
						      POWER_SUPPLY_PROP_STATUS,
						      &status);
	*val = ret ? POWER_SUPPLY_STATUS_UNKNOWN : status.intval;
	return 0;
}

static void qcom_qg_external_power_changed(struct power_supply *psy)
{
	power_supply_changed(psy);
}

static int qcom_qg_get_capacity(struct qcom_qg_chip *chip, int *val)
{
	int ret, voltage_now, temp = 25;
	int voltage_min = chip->batt_info->voltage_min_design_uv;
	int voltage_max = chip->batt_info->voltage_max_design_uv;

	ret = qcom_qg_get_voltage(chip, QG_S2_NORMAL_AVG_V_DATA0_REG,
				  &voltage_now);
	if (ret) {
		dev_err(chip->dev, "Failed to get current voltage: %d\n", ret);
		return ret;
	}

	if (chip->batt_therm_chan) {
		int temp_mc;

		ret = iio_read_channel_processed(chip->batt_therm_chan, &temp_mc);
		if (ret >= 0)
			temp = temp_mc / 1000;
	}

	/* Try OCV lookup table if provided by monitored-battery DT */
	ret = power_supply_batinfo_ocv2cap(chip->batt_info, voltage_now, temp);
	if (ret >= 0) {
		*val = clamp(ret, 0, 100);
		return 0;
	}

	/* Fallback to simple voltage-linear estimation */
	if (voltage_now <= voltage_min)
		*val = 0;
	else if (voltage_now >= voltage_max)
		*val = 100;
	else
		*val = (((voltage_now - voltage_min) * 100) /
						(voltage_max - voltage_min));

	return 0;
}

static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int qcom_qg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct qcom_qg_chip *chip = power_supply_get_drvdata(psy);
	u8 learned_capacity[2];
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return power_supply_get_property_from_supplier(psy, psp, val);
	case POWER_SUPPLY_PROP_STATUS:
		return qcom_qg_get_status(chip, &val->intval);
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = qcom_qg_get_voltage(chip, QG_LAST_ADC_V_DATA0_REG,
					  &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = qcom_qg_get_voltage(chip, QG_S2_NORMAL_AVG_V_DATA0_REG,
					  &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return qcom_qg_get_ocv(chip, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = qcom_qg_get_current(chip, QG_LAST_ADC_I_DATA0_REG,
					  &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = qcom_qg_get_current(chip, QG_S2_NORMAL_AVG_I_DATA0_REG,
					  &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->batt_info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = nvmem_device_read(chip->sdam,
					QG_SDAM_LEARNED_CAPACITY_OFFSET, 2,
					learned_capacity);
		if (ret < 0)
			return ret;
		if (ret != sizeof(learned_capacity))
			return -EIO;
		val->intval = get_unaligned_le16(learned_capacity) * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = qcom_qg_get_capacity(chip, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = iio_read_channel_processed
					(chip->batt_therm_chan, &val->intval);
		if (ret < 0)
			return ret;
		val->intval /= 100; /* 1/1000 °C (millidegC) to 1/10 °C */
		break;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
	return 0;
}

static struct power_supply_desc batt_psy_desc = {
	.name = "qcom_qg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
	.external_power_changed = qcom_qg_external_power_changed,
};

static int qcom_qg_probe(struct platform_device *pdev)
{
	struct qcom_qg_chip *chip;
	struct power_supply_config psy_cfg = {};
	unsigned int subtype;
	int ret, irq;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	mutex_init(&chip->fifo_lock);
	chip->fifo_epoch_ns = ktime_get_boottime_ns();

	/* Regmap */
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "Failed to locate the regmap\n");

	/* Get base address */
	ret = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "Couldn't read base address\n");

	ret = regmap_read(chip->regmap, chip->base + QG_PERPH_SUBTYPE_REG,
			  &subtype);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "Couldn't read QGauge subtype\n");

	/* Current ADC resolution in nA per LSB. */
	switch (subtype) {
	case QG_ADC_IBAT_5A:
		chip->current_scale_na = 152588;
		break;
	case QG_ADC_IBAT_10A:
		chip->current_scale_na = 305176;
		break;
	default:
		return dev_err_probe(chip->dev, -ENODEV,
				     "Unsupported QGauge subtype: %#x\n", subtype);
	}

	/* ADC for thermal channel */
	chip->batt_therm_chan = devm_iio_channel_get(chip->dev, "batt-therm");
	if (IS_ERR(chip->batt_therm_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_therm_chan),
				     "Couldn't get batt-therm IIO channel\n");

	/* NVMEM for SDAM access */
	chip->sdam = devm_nvmem_device_get(chip->dev, NULL);
	if (IS_ERR(chip->sdam))
		return dev_err_probe(chip->dev, PTR_ERR(chip->sdam),
				     "Couldn't get SDAM nvmem device\n");

	psy_cfg.drv_data = chip;
	psy_cfg.of_node = chip->dev->of_node;

	/* Power supply */
	chip->batt_psy =
		devm_power_supply_register(chip->dev, &batt_psy_desc, &psy_cfg);
	if (IS_ERR(chip->batt_psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_psy),
				     "Failed to register power supply\n");

	/* Battery info */
	ret = power_supply_get_battery_info(chip->batt_psy, &chip->batt_info);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "Failed to get battery info\n");

	platform_set_drvdata(pdev, chip);
	irq = platform_get_irq_byname_optional(pdev, "fifo-done");
	if (irq > 0) {
		ret = devm_request_threaded_irq(chip->dev, irq, NULL,
						qcom_qg_fifo_irq, IRQF_ONESHOT,
						"qg-fifo-done", chip);
		if (ret)
			return dev_err_probe(chip->dev, ret, "Failed to request FIFO IRQ\n");
	} else if (irq != -ENXIO) {
		return dev_err_probe(chip->dev, irq ?: -EINVAL, "Failed to get FIFO IRQ\n");
	}

	return qcom_qg_ocv_init(pdev, chip);
}

static const struct of_device_id qcom_qg_of_match[] = {
	{ .compatible = "qcom,pm6150-qg", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, qcom_qg_of_match);

static struct platform_driver qcom_qg_driver = {
	.driver = {
		.name = "qcom,qcom_qg",
		.of_match_table = qcom_qg_of_match,
		.pm = pm_sleep_ptr(&qcom_qg_pm_ops),
	},
	.probe = qcom_qg_probe,
};

module_platform_driver(qcom_qg_driver);

MODULE_AUTHOR("Danila Tikhonov <danila@jiaxyga.com>");
MODULE_DESCRIPTION("Qualcomm PMIC QGauge (QG) driver");
MODULE_LICENSE("GPL");
