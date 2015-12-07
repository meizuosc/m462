#ifndef __M6X_MFD_max77818_H_H_
#define __M6X_MFD_max77818_H_H_

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

static struct regulator_consumer_supply charger_supply[] = {
	REGULATOR_SUPPLY("vinchg1", "max77818-charger"),
};

static struct regulator_consumer_supply reverse_supply[] = {
	REGULATOR_SUPPLY("reverse", NULL),
	REGULATOR_SUPPLY("vbus_reverse", NULL),
};

static struct regulator_consumer_supply battery_supply[] = {
	REGULATOR_SUPPLY("battery", NULL),
};

static struct regulator_init_data safeout1_init_data = {
	.constraints	= {
		.name		= "safeout1 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= true,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout1_supply),
	.consumer_supplies	= safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "safeout2 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct regulator_init_data charger_init_data = {
	.constraints	= {
		.name		= "CHARGER",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_CURRENT,
		.min_uA		= 60000,
		.max_uA		= 2580000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(charger_supply),
	.consumer_supplies	= charger_supply,
};

static struct regulator_init_data reverse_init_data = {
	.constraints	= {
		.name		= "REVERSE",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.boot_on	= false,
		.state_mem	= {
			.disabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(reverse_supply),
	.consumer_supplies	= reverse_supply,
};

static struct regulator_init_data battery_init_data = {
	.constraints	= {
		.name		= "BATTERY",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_CURRENT,
		.boot_on	= true,
		.min_uA		= 0,
		.max_uA		= 2100000,
		.state_mem	= {
			.enabled	= true,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(battery_supply),
	.consumer_supplies	= battery_supply,
};

static struct max77818_regulator_data max77818_regulators[] = {
	{max77818_ESAFEOUT1, &safeout1_init_data,},
	{max77818_ESAFEOUT2, &safeout2_init_data,},
	{max77818_CHARGER, &charger_init_data,},
	{max77818_REVERSE, &reverse_init_data,},
	{max77818_BATTERY, &battery_init_data,},
};
#endif
