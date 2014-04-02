/*
 * locker.h
 *
 * Copyright 2014 Samsung Electronics Ltd..
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 */

#ifndef __REGULATOR_LOCKER_H
#define __REGULATOR_LOCKER_H

struct regulator_init_data;

/**
 * struct locker_voltage_config - locker_voltage_config structure
 * @supply_name:	Name of the regulator supply
 * @input_supply:	Name of the input regulator supply
 * @init_data:		regulator_init_data
 *
 */
struct locker_voltage_regulator_config {
	const char *supply_name;
	const char *input_supply;
	struct regulator_init_data *init_data;
};

struct locker_voltage_config {
	struct locker_voltage_regulator_config *regulator_configs;
	int num_regs;
	int max_spread;
};
#endif
