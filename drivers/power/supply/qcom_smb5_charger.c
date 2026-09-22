// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PM6150 SMB5 Switch-Mode Battery Charger Driver
 *
 * Copyright (c) 2026 Alexis Barron Luna <alexis.barron.luna@gmail.com>
 *
 * Based on Casey Connolly's qcom_smbx.c and downstream smb-lib / qpnp-smb5.
 * Manages the SMB5 charger and USB input path on Qualcomm PM6150 PMICs.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <soc/qcom/qcom-spmi-pmic.h>

#define PM6150_SUBTYPE			0x28

/* Peripheral offsets relative to CHGR base (0x1000) */
#define SMB5_SUBTYPE_REG		0x005
#define SMB5_CHGR_SUBTYPE		0x80

/* CHGR peripheral (0x1000) */
#define SMB5_BAT_CHG_STATUS_1		0x006
#define SMB5_BAT_CHG_STATUS_2		0x007
#define  SMB5_BAT_OV_BIT		BIT(1)
#define SMB5_BAT_CHG_STATUS_5		0x00b
#define  SMB5_VALID_INPUT_SRC_BIT	BIT(7)
#define  SMB5_CHARGING_ENABLE_STS_BIT	BIT(4)
#define SMB5_CHARGING_ENABLE_CMD	0x042
#define  SMB5_CHARGING_ENABLE_CMD_BIT	BIT(0)
#define SMB5_FAST_CHARGE_CURRENT_CFG	0x061
#define SMB5_FLOAT_VOLTAGE_CFG		0x070

/* DCDC peripheral (0x1100) */
#define SMB5_POWER_PATH			0x10b
#define  SMB5_USE_USB			BIT(4)
#define  SMB5_VALID_INPUT		BIT(0)
#define SMB5_OTG_CMD			0x140

/* BAT_IF peripheral (0x1200) */
#define SMB5_BAT_PRES_STATUS		0x208

/* USB_CHGPTH peripheral (0x1300) */
#define SMB5_APSD_STATUS		0x307
#define  SMB5_APSD_DONE_BIT		BIT(0)
#define SMB5_APSD_RESULT		0x308
#define  SMB5_APSD_SDP_BIT		BIT(0)
#define  SMB5_APSD_OCP_BIT		BIT(1)
#define  SMB5_APSD_CDP_BIT		BIT(2)
#define  SMB5_APSD_DCP_BIT		BIT(3)
#define  SMB5_APSD_FLOAT_BIT		BIT(4)
#define  SMB5_APSD_QC2_BIT		BIT(5)
#define  SMB5_APSD_QC3_BIT		BIT(6)
#define SMB5_USBIN_RT_STS		0x310
#define  SMB5_PLUGIN_RT_STS		BIT(4)
#define  SMB5_USBIN_OV_RT_STS		BIT(3)
#define  SMB5_USBIN_UV_RT_STS		BIT(2)
#define SMB5_USB_CMD			0x340
#define  SMB5_USB_SUSPEND		BIT(0)
#define SMB5_ICL_CMD			0x342
#define  SMB5_ICL_OVERRIDE		BIT(0)
#define SMB5_USB_LOAD			0x365
#define  SMB5_OVERRIDE_APSD		BIT(4)
#define SMB5_USB_OPTIONS		0x366
#define  SMB5_HC_MODE			BIT(0)
#define  SMB5_USB51_MODE		BIT(1)
#define SMB5_USB_ICL			0x370
#define SMB5_AICL_OPTIONS_CFG		0x380

/* MISC peripheral (0x1600) */
#define SMB5_WDOG_PET			0x643
#define  SMB5_WDOG_PET_BIT		BIT(0)
#define SMB5_WD_CFG			0x651
#define SMB5_MISC_AICL_CMD		0x680

#define SMB5_ICL_STEP_UA		50000
#define SMB5_DEFAULT_CEILING_UA		1800000
#define SMB5_DEFAULT_FCC_UA		2000000

static bool apply_limits = true;
module_param(apply_limits, bool, 0644);
MODULE_PARM_DESC(apply_limits, "Apply USB input and charging limits (default: true)");

static const struct {
	unsigned int reg;
	unsigned int mask;
} smb5_owned[] = {
	{ SMB5_USB_ICL, 0xff },
	{ SMB5_USB_OPTIONS, SMB5_HC_MODE },
	{ SMB5_ICL_CMD, SMB5_ICL_OVERRIDE },
	{ SMB5_USB_LOAD, SMB5_OVERRIDE_APSD },
	{ SMB5_USB_CMD, SMB5_USB_SUSPEND },
};

struct smb5_charger {
	struct device *dev;
	struct regmap *regmap;
	unsigned int base;
	unsigned int ceiling_ua;
	unsigned int saved[ARRAY_SIZE(smb5_owned)];

	struct power_supply *source;
	struct power_supply *psy;
	struct power_supply_desc desc;
	struct power_supply_battery_info *batt_info;

	struct iio_channel *usb_i;
	struct iio_channel *usb_v;

	struct mutex lock; /* Protects chip state and register updates */
	struct work_struct work;
	struct notifier_block nb;

	int cable_irq;
	int icl_irq;
	int bat_ov_irq;
	int wdog_irq;

	int requested_ua;
	bool ready;
	bool touched;
};

static int smb5_read(struct smb5_charger *chip, unsigned int reg, unsigned int *val)
{
	return regmap_read(chip->regmap, chip->base + reg, val);
}

static int smb5_update(struct smb5_charger *chip, unsigned int reg,
		       unsigned int mask, unsigned int val)
{
	chip->touched = true;
	return regmap_update_bits(chip->regmap, chip->base + reg, mask, val);
}

static int smb5_suspend_input(struct smb5_charger *chip)
{
	return smb5_update(chip, SMB5_USB_CMD, SMB5_USB_SUSPEND, SMB5_USB_SUSPEND);
}

static int smb5_online(struct smb5_charger *chip, int *online)
{
	union power_supply_propval val;
	unsigned int rt_sts;
	int ret;

	/* Check physical USBIN plug status from PMIC hardware */
	ret = smb5_read(chip, SMB5_USBIN_RT_STS, &rt_sts);
	if (ret)
		return ret;

	if (rt_sts & SMB5_PLUGIN_RT_STS) {
		*online = 1;
		return 0;
	}

	/* Also check TCPM source supply if linked */
	if (chip->source) {
		ret = power_supply_get_property(chip->source,
						POWER_SUPPLY_PROP_ONLINE,
						&val);
		if (!ret && val.intval) {
			*online = 1;
			return 0;
		}
	}

	*online = 0;
	return 0;
}

static int smb5_get_limit(struct smb5_charger *chip, int *ua)
{
	unsigned int cmd, options, load, override, raw;
	int ret;

	ret = smb5_read(chip, SMB5_USB_CMD, &cmd);
	if (ret)
		return ret;
	if (cmd & SMB5_USB_SUSPEND) {
		*ua = 0;
		return 0;
	}
	ret = smb5_read(chip, SMB5_USB_OPTIONS, &options);
	if (ret)
		return ret;
	ret = smb5_read(chip, SMB5_USB_LOAD, &load);
	if (ret)
		return ret;
	ret = smb5_read(chip, SMB5_ICL_CMD, &override);
	if (ret)
		return ret;
	if (!(options & SMB5_HC_MODE) || !(load & SMB5_OVERRIDE_APSD) ||
	    (override & SMB5_ICL_OVERRIDE))
		return -ENODATA;
	ret = smb5_read(chip, SMB5_USB_ICL, &raw);
	if (!ret)
		*ua = raw * SMB5_ICL_STEP_UA;
	return ret;
}

static int smb5_set_limit(struct smb5_charger *chip, unsigned int ua)
{
	int ret, configured_ua;

	ret = smb5_get_limit(chip, &configured_ua);
	if (!ret && configured_ua == ua)
		return 0;

	/* Keep input suspended during configuration */
	ret = smb5_suspend_input(chip);
	if (ret || !ua)
		return ret;

	ret = smb5_update(chip, SMB5_USB_ICL, 0xff, ua / SMB5_ICL_STEP_UA);
	if (ret)
		return ret;
	ret = smb5_update(chip, SMB5_USB_OPTIONS, SMB5_HC_MODE, SMB5_HC_MODE);
	if (ret)
		return ret;
	ret = smb5_update(chip, SMB5_ICL_CMD, SMB5_ICL_OVERRIDE, 0);
	if (ret)
		return ret;
	ret = smb5_update(chip, SMB5_USB_LOAD, SMB5_OVERRIDE_APSD, SMB5_OVERRIDE_APSD);
	if (ret)
		return ret;

	/* Unsuspend USBIN and enable charging */
	ret = smb5_update(chip, SMB5_USB_CMD, SMB5_USB_SUSPEND, 0);
	if (ret)
		return ret;

	return smb5_update(chip, SMB5_CHARGING_ENABLE_CMD,
			   SMB5_CHARGING_ENABLE_CMD_BIT,
			   SMB5_CHARGING_ENABLE_CMD_BIT);
}

static int smb5_compute_budget(struct smb5_charger *chip, unsigned int *budget_ua)
{
	union power_supply_propval val;
	unsigned int target_ua = 0;
	int online, ret;

	ret = smb5_online(chip, &online);
	if (ret)
		return ret;

	if (!online) {
		*budget_ua = 0;
		return 0;
	}

	/* 1. If connected via USB-C PD, check if TCPM established a power contract */
	if (chip->source) {
		ret = power_supply_get_property(chip->source,
						POWER_SUPPLY_PROP_CURRENT_MAX,
						&val);
		if (!ret && val.intval > 0)
			target_ua = val.intval;
	}

	/* 2. Fallback for USB-A / Non-PD / Legacy connections */
	if (target_ua == 0) {
		unsigned int apsd_res = 0;

		ret = smb5_read(chip, SMB5_APSD_RESULT, &apsd_res);
		if (!ret) {
			if (apsd_res & (SMB5_APSD_CDP_BIT | SMB5_APSD_DCP_BIT |
					SMB5_APSD_OCP_BIT | SMB5_APSD_QC2_BIT |
					SMB5_APSD_QC3_BIT))
				target_ua = 1500000;
			else
				target_ua = 500000; /* SDP or standard USB fallback */
		} else {
			target_ua = 500000;
		}
	}

	if (chip->ceiling_ua > 0)
		target_ua = min(target_ua, chip->ceiling_ua);

	*budget_ua = rounddown(target_ua, SMB5_ICL_STEP_UA);
	return 0;
}

static int smb5_refresh(struct smb5_charger *chip)
{
	unsigned int ua;
	int ret, suspend_ret;

	mutex_lock(&chip->lock);
	ret = smb5_compute_budget(chip, &ua);
	if (apply_limits) {
		if (!ret)
			ret = smb5_set_limit(chip, ua);
		if (ret) {
			suspend_ret = smb5_suspend_input(chip);
			if (suspend_ret)
				dev_err(chip->dev, "Unable to suspend input: %d\n", suspend_ret);
		}
	}
	if (ret) {
		dev_err_ratelimited(chip->dev, "Budget update failed: %d\n", ret);
	} else if (chip->requested_ua != ua) {
		chip->requested_ua = ua;
		dev_info(chip->dev, "%s input limit: %u uA\n",
			 apply_limits ? "Applied" : "Read-only", ua);
	}
	mutex_unlock(&chip->lock);
	return ret;
}

static void smb5_work(struct work_struct *work)
{
	struct smb5_charger *chip = container_of(work, struct smb5_charger, work);

	if (!READ_ONCE(chip->ready))
		return;
	smb5_refresh(chip);
	power_supply_changed(chip->psy);
}

static void smb5_external_power_changed(struct power_supply *psy)
{
	struct smb5_charger *chip = power_supply_get_drvdata(psy);

	mutex_lock(&chip->lock);
	if (chip->ready)
		schedule_work(&chip->work);
	mutex_unlock(&chip->lock);
}

static int smb5_notifier_call(struct notifier_block *nb,
			      unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct smb5_charger *chip = container_of(nb, struct smb5_charger, nb);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (chip->source && psy == chip->source) {
		mutex_lock(&chip->lock);
		if (chip->ready)
			schedule_work(&chip->work);
		mutex_unlock(&chip->lock);
	}

	return NOTIFY_OK;
}

static irqreturn_t smb5_cable_isr(int irq, void *data)
{
	struct smb5_charger *chip = data;

	power_supply_changed(chip->psy);
	mutex_lock(&chip->lock);
	if (chip->ready)
		schedule_work(&chip->work);
	mutex_unlock(&chip->lock);

	return IRQ_HANDLED;
}

static irqreturn_t smb5_icl_isr(int irq, void *data)
{
	struct smb5_charger *chip = data;

	power_supply_changed(chip->psy);
	return IRQ_HANDLED;
}

static irqreturn_t smb5_bat_ov_isr(int irq, void *data)
{
	struct smb5_charger *chip = data;
	unsigned int stat;

	smb5_read(chip, SMB5_BAT_CHG_STATUS_2, &stat);
	if (stat & SMB5_BAT_OV_BIT)
		dev_warn(chip->dev, "Battery overvoltage detected\n");

	power_supply_changed(chip->psy);
	return IRQ_HANDLED;
}

static irqreturn_t smb5_wdog_isr(int irq, void *data)
{
	struct smb5_charger *chip = data;

	regmap_update_bits(chip->regmap, chip->base + SMB5_WDOG_PET,
			   SMB5_WDOG_PET_BIT, SMB5_WDOG_PET_BIT);
	return IRQ_HANDLED;
}

static int smb5_get_prop_status(struct smb5_charger *chip, int *val)
{
	unsigned int stat;
	int online, ret;

	ret = smb5_online(chip, &online);
	if (ret)
		return ret;
	if (!online) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = smb5_read(chip, SMB5_BAT_CHG_STATUS_1, &stat);
	if (ret)
		return ret;

	switch (stat & 0x07) {
	case 0: /* TRICKLE */
	case 1: /* PRE */
	case 2: /* FAST */
	case 3: /* FULLON */
	case 4: /* TAPER */
		*val = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 5: /* TERMINATE */
	case 6: /* INHIBIT */
		*val = POWER_SUPPLY_STATUS_FULL;
		break;
	case 7: /* DISABLE */
	default:
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	}
	return 0;
}

static int smb5_get_prop_health(struct smb5_charger *chip, int *val)
{
	unsigned int stat2, rt_sts;

	if (smb5_read(chip, SMB5_BAT_CHG_STATUS_2, &stat2) == 0 &&
	    (stat2 & SMB5_BAT_OV_BIT)) {
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	if (smb5_read(chip, SMB5_USBIN_RT_STS, &rt_sts) == 0) {
		if (rt_sts & SMB5_USBIN_OV_RT_STS) {
			*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			return 0;
		}
	}

	*val = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int smb5_get_usb_type(struct smb5_charger *chip, int *val)
{
	unsigned int apsd_res;
	int ret;

	if (chip->source) {
		ret = power_supply_get_property(chip->source,
						POWER_SUPPLY_PROP_USB_TYPE,
						(union power_supply_propval *)val);
		if (!ret && *val != POWER_SUPPLY_USB_TYPE_UNKNOWN)
			return 0;
	}

	ret = smb5_read(chip, SMB5_APSD_RESULT, &apsd_res);
	if (ret)
		return ret;

	if (apsd_res & (SMB5_APSD_DCP_BIT | SMB5_APSD_OCP_BIT |
			SMB5_APSD_QC2_BIT | SMB5_APSD_QC3_BIT))
		*val = POWER_SUPPLY_USB_TYPE_DCP;
	else if (apsd_res & SMB5_APSD_CDP_BIT)
		*val = POWER_SUPPLY_USB_TYPE_CDP;
	else if (apsd_res & SMB5_APSD_SDP_BIT)
		*val = POWER_SUPPLY_USB_TYPE_SDP;
	else
		*val = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	return 0;
}

static enum power_supply_property smb5_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int smb5_get_property(struct power_supply *psy,
			     enum power_supply_property prop,
			     union power_supply_propval *val)
{
	struct smb5_charger *chip = power_supply_get_drvdata(psy);
	int ret = 0, online;

	mutex_lock(&chip->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Qualcomm";
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "PM6150 SMB5 Charger";
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = smb5_get_prop_status(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = smb5_get_prop_health(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = smb5_online(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		ret = smb5_get_usb_type(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = smb5_get_limit(chip, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = smb5_online(chip, &online);
		if (ret)
			break;
		val->intval = 0;
		if (online && chip->usb_i) {
			ret = iio_read_channel_processed(chip->usb_i, &val->intval);
			if (ret >= 0) {
				val->intval *= 5; /* PM6150 buck: 0.2 V/A -> 5 uA/uV */
				ret = 0;
			}
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = 0;
		if (chip->usb_v) {
			ret = iio_read_channel_processed(chip->usb_v, &val->intval);
			if (ret >= 0)
				ret = 0;
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&chip->lock);
	return ret;
}

static void smb5_stop(void *data)
{
	struct smb5_charger *chip = data;
	int i, ret, error = 0;

	mutex_lock(&chip->lock);
	WRITE_ONCE(chip->ready, false);
	mutex_unlock(&chip->lock);
	cancel_work_sync(&chip->work);

	if (chip->source)
		power_supply_unreg_notifier(&chip->nb);

	mutex_lock(&chip->lock);
	if (!chip->touched)
		goto out;

	ret = smb5_suspend_input(chip);
	if (ret) {
		dev_err(chip->dev, "Cannot suspend for restoration: %d\n", ret);
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(smb5_owned) - 1; i++) {
		ret = regmap_update_bits(chip->regmap, chip->base + smb5_owned[i].reg,
					 smb5_owned[i].mask, chip->saved[i]);
		if (ret)
			error = ret;
	}
	if (!error)
		error = regmap_update_bits(chip->regmap, chip->base + smb5_owned[i].reg,
					   smb5_owned[i].mask, chip->saved[i]);
	if (error)
		dev_err(chip->dev, "Register restore incomplete: %d\n", error);
	else
		dev_info(chip->dev, "Original PM6150 SMB5 settings restored\n");
out:
	mutex_unlock(&chip->lock);
}

static int smb5_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct power_supply_config cfg = {};
	const struct qcom_spmi_pmic *pmic;
	struct smb5_charger *chip;
	unsigned int subtype;
	int i, ret;

	pmic = qcom_pmic_get(dev);
	if (IS_ERR(pmic))
		return dev_err_probe(dev, PTR_ERR(pmic), "Cannot identify PMIC\n");
	if (pmic->subtype != PM6150_SUBTYPE)
		return dev_err_probe(dev, -ENODEV, "Only PM6150 is supported\n");

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->requested_ua = -1;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->work, smb5_work);

	chip->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chip->regmap)
		return -ENODEV;

	ret = of_property_read_u32(dev->of_node, "reg", &chip->base);
	if (ret)
		return ret;
	if (chip->base != 0x1000)
		return dev_err_probe(dev, -EINVAL, "Unexpected CHGR base %#x\n", chip->base);

	ret = smb5_read(chip, SMB5_SUBTYPE_REG, &subtype);
	if (ret || subtype != SMB5_CHGR_SUBTYPE)
		return dev_err_probe(dev, ret ?: -ENODEV,
				     "Unexpected charger subtype: %#x\n", subtype);

	if (of_property_read_u32(dev->of_node, "input-current-limit-microamp", &chip->ceiling_ua))
		chip->ceiling_ua = SMB5_DEFAULT_CEILING_UA;

	chip->source = devm_power_supply_get_by_phandle(dev, "power-supplies");
	if (IS_ERR(chip->source)) {
		if (PTR_ERR(chip->source) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		chip->source = NULL;
	}

	chip->usb_i = devm_iio_channel_get(dev, "usbin_i");
	if (IS_ERR(chip->usb_i))
		chip->usb_i = NULL;

	chip->usb_v = devm_iio_channel_get(dev, "usbin_v");
	if (IS_ERR(chip->usb_v))
		chip->usb_v = NULL;

	for (i = 0; i < ARRAY_SIZE(smb5_owned); i++) {
		ret = smb5_read(chip, smb5_owned[i].reg, &chip->saved[i]);
		if (ret)
			return ret;
		chip->saved[i] &= smb5_owned[i].mask;
	}

	chip->desc.name = devm_kasprintf(dev, GFP_KERNEL, "pm6150-charger");
	if (!chip->desc.name)
		return -ENOMEM;
	chip->desc.type = POWER_SUPPLY_TYPE_USB;
	chip->desc.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
			       BIT(POWER_SUPPLY_USB_TYPE_CDP) |
			       BIT(POWER_SUPPLY_USB_TYPE_DCP) |
			       BIT(POWER_SUPPLY_USB_TYPE_C) |
			       BIT(POWER_SUPPLY_USB_TYPE_PD) |
			       BIT(POWER_SUPPLY_USB_TYPE_PD_DRP) |
			       BIT(POWER_SUPPLY_USB_TYPE_PD_PPS) |
			       BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN);
	chip->desc.properties = smb5_properties;
	chip->desc.num_properties = ARRAY_SIZE(smb5_properties);
	chip->desc.get_property = smb5_get_property;
	chip->desc.external_power_changed = smb5_external_power_changed;

	cfg.drv_data = chip;
	cfg.fwnode = dev_fwnode(dev);

	chip->psy = devm_power_supply_register(dev, &chip->desc, &cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy), "Cannot register charger supply\n");

	power_supply_get_battery_info(chip->psy, &chip->batt_info);
	if (chip->batt_info) {
		unsigned int fcc_val, fv_val;

		if (chip->batt_info->constant_charge_current_max_ua > 0) {
			fcc_val = chip->batt_info->constant_charge_current_max_ua /
				  SMB5_ICL_STEP_UA;
			smb5_update(chip, SMB5_FAST_CHARGE_CURRENT_CFG, 0xff, fcc_val);
		}
		if (chip->batt_info->voltage_max_design_uv >= 3600000) {
			fv_val = (chip->batt_info->voltage_max_design_uv - 3600000) / 10000;
			smb5_update(chip, SMB5_FLOAT_VOLTAGE_CFG, 0xff, fv_val);
		}
	}

	/* Optional SPMI interrupts */
	chip->cable_irq = platform_get_irq_byname_optional(pdev, "usb-plugin");
	if (chip->cable_irq >= 0) {
		ret = devm_request_threaded_irq(dev, chip->cable_irq, NULL,
						smb5_cable_isr,
						IRQF_ONESHOT,
						"usb-plugin", chip);
		if (!ret)
			devm_pm_set_wake_irq(dev, chip->cable_irq);
	}

	chip->icl_irq = platform_get_irq_byname_optional(pdev, "usbin-icl-change");
	if (chip->icl_irq >= 0) {
		ret = devm_request_threaded_irq(dev, chip->icl_irq, NULL,
						smb5_icl_isr,
						IRQF_ONESHOT,
						"usbin-icl-change", chip);
		if (ret)
			dev_warn(dev, "Failed to request icl irq: %d\n", ret);
	}

	chip->bat_ov_irq = platform_get_irq_byname_optional(pdev, "bat-ov");
	if (chip->bat_ov_irq >= 0) {
		ret = devm_request_threaded_irq(dev, chip->bat_ov_irq, NULL,
						smb5_bat_ov_isr,
						IRQF_ONESHOT,
						"bat-ov", chip);
		if (ret)
			dev_warn(dev, "Failed to request bat-ov irq: %d\n", ret);
	}

	chip->wdog_irq = platform_get_irq_byname_optional(pdev, "wdog-bark");
	if (chip->wdog_irq >= 0) {
		ret = devm_request_threaded_irq(dev, chip->wdog_irq, NULL,
						smb5_wdog_isr,
						IRQF_ONESHOT,
						"wdog-bark", chip);
		if (ret)
			dev_warn(dev, "Failed to request wdog irq: %d\n", ret);
	}

	if (chip->source) {
		chip->nb.notifier_call = smb5_notifier_call;
		ret = power_supply_reg_notifier(&chip->nb);
		if (ret)
			return ret;
	}

	ret = devm_add_action_or_reset(dev, smb5_stop, chip);
	if (ret)
		return ret;

	ret = smb5_refresh(chip);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, chip);

	mutex_lock(&chip->lock);
	WRITE_ONCE(chip->ready, true);
	schedule_work(&chip->work);
	mutex_unlock(&chip->lock);

	return 0;
}

static int smb5_resume(struct device *dev)
{
	struct smb5_charger *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	if (chip->ready)
		schedule_work(&chip->work);
	mutex_unlock(&chip->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(smb5_pm_ops, NULL, smb5_resume);

static const struct of_device_id smb5_match[] = {
	{ .compatible = "qcom,pm6150-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, smb5_match);

static struct platform_driver smb5_driver = {
	.probe = smb5_probe,
	.driver = {
		.name = "qcom-smb5-charger",
		.of_match_table = smb5_match,
		.pm = pm_sleep_ptr(&smb5_pm_ops),
	},
};
module_platform_driver(smb5_driver);

MODULE_AUTHOR("Alexis Barron Luna <alexis.barron.luna@gmail.com>");
MODULE_DESCRIPTION("Qualcomm PM6150 SMB5 Switch-Mode Battery Charger Driver");
MODULE_LICENSE("GPL");
