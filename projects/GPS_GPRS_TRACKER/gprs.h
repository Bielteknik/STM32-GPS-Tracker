/*
 * gprs.h
 *
 *  Created on: 22.01.2012
 *      Author: dimaz
 */

#ifndef GPRS_H_
#define GPRS_H_

#include "ch.h"
#include "hal.h"

typedef enum GPRS_MODEM_ERRORS {
	E_OK					=	0x00,
	E_NOT_RESPONDING		=	0x01,
	E_INVALID_ANSWER		=	0x02,
	E_NO_NETWORK			=	0x03,
	E_GPRS_CONNECT_ERROR	=	0x04
};

extern void init_gprs();

uint8_t init_modem();

uint8_t gprs_cmd(char * cmd_str, uint16_t cmd_len, char * answer_str, uint16_t answer_len);

uint8_t gprs_cmd_read(char * cmd_str, uint16_t cmd_len, uint16_t *answer_len);

uint8_t gprs_get_signal_level(uint16_t *signalLevel);

extern uint8_t is_gprs_network_ok();

uint8_t send_tcp_message();

#endif /* GPRS_H_ */
