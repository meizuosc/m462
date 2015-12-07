/* linux/power/max77818-sfo.h
 *
 * Copyright 2013 Maxim Integrated Products, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_MAX77818_CHARGER_H
#define __LINUX_MAX77818_CHARGER_H

#define MAX77818_SFO1 0
#define MAX77818_SFO2 1

struct max77818_sfo_data
{
	int id;
	bool active_discharge;
	struct regulator_init_data *initdata;
	struct device_node *of_node;
};

struct max77818_sfo_pdata
{
	struct max77818_sfo_data *regulator_data;
	int num_regulators;
};

#endif
