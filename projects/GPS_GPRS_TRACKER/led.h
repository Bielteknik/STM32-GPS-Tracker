/*
 * led.h
 *
 *  Created on: 25.01.2012
 *      Author: dimaz
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>

extern void start_led_thread();

extern void set_led_0_prescaler(uint8_t prescaler);

extern void set_led_1_prescaler(uint8_t prescaler);

#endif /* LED_H_ */
