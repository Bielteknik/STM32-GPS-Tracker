/*
 * power.h
 *
 *  Created on: 23.01.2012
 *      Author: dimaz
 */

#ifndef POWER_H_
#define POWER_H_

#include "ch.h"
#include "hal.h"

extern void init_power_ctl();

extern void update_power_state();

extern void print_power_state();

#endif /* POWER_H_ */
