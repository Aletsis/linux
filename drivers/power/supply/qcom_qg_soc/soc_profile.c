// SPDX-License-Identifier: GPL-2.0-only
/* Private QG fixture: standard simple-battery tables, explicit external scenario. */
static int qcom_qg_soc_profile_init(struct qcom_qg_chip *chip)
{
	struct power_supply_battery_info *info = chip->batt_info;
	const struct power_supply_battery_ocv_table *table;
	struct sha256_ctx digest;
	u8 value[4];
	int i, j, count = 0;

	sha256_init(&digest);
	sha256_update(&digest, (const u8 *)"qg-full-to-design-min-v2", 24);
	put_unaligned_le32(info->charge_full_design_uah, value);
	sha256_update(&digest, value, sizeof(value));
	put_unaligned_le32(info->voltage_min_design_uv, value);
	sha256_update(&digest, value, sizeof(value));
	for (i = 0; i < POWER_SUPPLY_OCV_TEMP_MAX; i++) {
		table = info->ocv_table[i];
		if (!table)
			break;
		if (info->ocv_table_size[i] < 2 || info->ocv_temp[i] < -100 ||
		    info->ocv_temp[i] > 100 ||
		    (i && info->ocv_temp[i] <= info->ocv_temp[i - 1]))
			return -EINVAL;
		put_unaligned_le32(info->ocv_temp[i], value);
		sha256_update(&digest, value, sizeof(value));
		put_unaligned_le32(info->ocv_table_size[i], value);
		sha256_update(&digest, value, sizeof(value));
		for (j = 0; j < info->ocv_table_size[i]; j++) {
			if (table[j].ocv <= 0 || table[j].ocv > 10000000 ||
			    table[j].capacity < 0 || table[j].capacity > 100 ||
			    (j && (table[j].ocv >= table[j - 1].ocv ||
			     table[j].capacity >= table[j - 1].capacity)))
				return -EINVAL;
			put_unaligned_le32(table[j].ocv, value);
			sha256_update(&digest, value, sizeof(value));
			put_unaligned_le32(table[j].capacity, value);
			sha256_update(&digest, value, sizeof(value));
		}
		count++;
	}
	if (!count)
		return -EINVAL;
	sha256_final(&digest, chip->soc_basis.scenario_digest);
	return 0;
}

static int qcom_qg_soc_table(struct qcom_qg_chip *chip, int temp_mc)
{
	struct power_supply_battery_info *info = chip->batt_info;
	int i, best = 0, count = 0;

	while (count < POWER_SUPPLY_OCV_TEMP_MAX && info->ocv_table[count])
		count++;
	if (!count || temp_mc < info->ocv_temp[0] * 1000 ||
	    temp_mc > info->ocv_temp[count - 1] * 1000)
		return -EINVAL;
	for (i = 1; i < count; i++)
		if (abs(temp_mc - info->ocv_temp[i] * 1000) <
		    abs(temp_mc - info->ocv_temp[best] * 1000))
			best = i;
	return best;
}

static int qcom_qg_soc_lookup(struct qcom_qg_chip *chip, int index, int uv)
{
	const struct power_supply_battery_ocv_table *table;
	int i, size;

	if (index < 0 || index >= POWER_SUPPLY_OCV_TEMP_MAX)
		return -EINVAL;
	table = chip->batt_info->ocv_table[index];
	size = chip->batt_info->ocv_table_size[index];
	if (!table || size < 2 || uv > table[0].ocv || uv < table[size - 1].ocv)
		return -ERANGE;
	for (i = 1; i < size; i++)
		if (uv >= table[i].ocv)
			return table[i].capacity * 1000 +
				div_s64((s64)(uv - table[i].ocv) *
					(table[i - 1].capacity - table[i].capacity) * 1000,
					table[i - 1].ocv - table[i].ocv);
	return -ERANGE;
}
