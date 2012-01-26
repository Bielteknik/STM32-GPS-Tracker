/*
 * gps.h
 *
 *  Created on: 22.01.2012
 *      Author: dimaz
 */

#ifndef GPS_H_
#define GPS_H_

#include "ch.h"
#include "hal.h"

#include <stdint.h>

#define GPS_CMD_SEND(A) gps_write_cmd(A, sizeof(A))

typedef enum GPS_MESSAGE {
	GPS_MESSAGE_GPGGA				=	0x00,
	GPS_MESSAGE_GPGLL				=	0x01,
	GPS_MESSAGE_GPGSA				=	0x02,
	GPS_MESSAGE_GPGSV				=	0x03,
	GPS_MESSAGE_GPRMC				=	0x04,
	GPS_MESSAGE_GPVTG				=	0x05,
	GPS_MESSAGE_GPZDA				=	0x06,
	GPS_MESSAGE_UNKNOWN				=	0xFF
} GPS_MESSAGE;

typedef enum GPS_STATE_FLAGS {
	LATITUDE_N				=	1 << 0,
	LATITUDE_S				=	1 << 1,
	LONGITUDE_E				=	1 << 2,
	LONGITUDE_W				=	1 << 3,
	GPS_DATA_INVALID		=	1 << 4
} GPS_STATE_FLAGS;

typedef struct gps_rmc_state {
	uint8_t latitude_degrees;
	uint32_t latitude_seconds;

	uint8_t longitude_degrees;
	uint32_t longitude_seconds;

	uint8_t flags;

	uint16_t speed;
	uint16_t course;

	uint8_t day;
	uint8_t month;
	uint8_t year;

	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} gps_rmc_state_t;

extern void init_gps();

extern void gps_reset();

void gps_write_cmd(uint8_t * cmd_buf, uint8_t len);

uint8_t gps_read_msg(size_t *msg_len);

uint8_t gps_message_type();

uint8_t check_checksum();

uint8_t parse_gps_rmc(gps_rmc_state_t * state);


#endif /* GPS_H_ */
