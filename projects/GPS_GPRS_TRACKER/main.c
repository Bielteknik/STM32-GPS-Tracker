/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ch.h"
#include "hal.h"

#include "gps.h"
#include "gprs.h"
#include "power.h"
#include "led.h"

SerialConfig SD1_Config = {
   .sc_speed = 19200,
   .sc_cr2 = USART_CR2_STOP1_BITS
};


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sdStart(&SD1, &SD1_Config);
  palSetPadMode(EXT_USART_PORT, EXT_USART_TX_PIN, PAL_MODE_ALTERNATE(7));
  palSetPadMode(EXT_USART_PORT, EXT_USART_RX_PIN, PAL_MODE_ALTERNATE(7));

  start_led_thread();

  init_gprs();

  init_gps();

  //init_power_ctl();

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched with output on the serial
   * driver 1.
   */
  while (TRUE) {
	  chThdSleepMilliseconds(500);
	  //update_power_state();
    //chThdSleepMilliseconds(500);
    //print_power_state();

  }
}
