#include "gps.h"


#include "ch.h"
#include "hal.h"

#include "util.h"

#define GPS_CMD_BUF 256

#define GPS_SERIAL SD3

#define GPS_READ_TIMEOUT_TICS	1000
#define GPS_READ_MSG_TIMEOUT_TICS	100

typedef enum GPS_ERROR {
	E_OK				=	0x00,
	E_READ_TIMEOUT		=	0x01,
	E_INVALID_DATA		=	0x02,
	E_LEN_ERROR			=	0x03,
	E_CHKSUM_ERROR		=	0x04,
	E_MSG_TYPE_ERR		=	0x05

} GPS_ERROR;

SerialConfig SD3_Config = {
   .sc_speed = 9600,
   .sc_cr2 = USART_CR2_STOP1_BITS
};

uint8_t *gps_data = NULL;

static WORKING_AREA(waGPSThread, 256);
static msg_t GPSThread(void *arg) {
  (void)arg;
  chRegSetThreadName("gps_thread");

  if (gps_data != NULL) {
	  chHeapFree(gps_data);
  }

  gps_data = chHeapAlloc(NULL, GPS_CMD_BUF);
  size_t gps_bytes_read;
  uint16_t i;

  if (gps_data == NULL) {
	  while (TRUE) {
		  palTogglePad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);
		  chThdSleepMilliseconds(50);
	  }
  }

  sdStart(&GPS_SERIAL, &SD3_Config);
  palSetPadMode(GPS_USART_PORT, GPS_USART_TX_PIN, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPS_USART_PORT, GPS_USART_RX_PIN, PAL_MODE_ALTERNATE(7));

  gps_reset();

  while (TRUE) {
	  size_t readed_msg_len = 0;
	  uint8_t res = gps_read_msg(&readed_msg_len);

	  if (res == E_OK) {
		  if (check_checksum() != E_OK) {
			  sdWrite(&SD1, "GPS CHK ERR\r\n", 13);
		  } else {

			  if (gps_message_type() == GPS_MESSAGE_GPRMC) {
				  gps_rmc_state_t state;

				  //sdWrite(&SD1, gps_data, readed_msg_len);

				  if (parse_gps_rmc(&state) == E_OK) {
					  sdWrite(&SD1, "GPS PARSE OK\r\n", 14);
				  }

			  } else if (gps_message_type() == GPS_MESSAGE_UNKNOWN) {
				  sdWrite(&SD1, "GPS MSG UNKNOWN\r\n", 17);
			  }

		  }
	  } else {
		  gps_data[0] = '0' + res;
		  sdWrite(&SD1, "GPS ERROR:", 10);
		  sdWrite(&SD1, gps_data, 1);
		  gps_reset();
	  }

	  sdWrite(&SD1, "\r\n", 2);



	//sdWrite(&SD1, "GPS: ", 5);
	//while ((gps_bytes_read = sdReadTimeout(&GPS_SERIAL, gps_data, GPS_CMD_BUF, 100)) > 0)
	//	sdWrite(&SD1, gps_data, gps_bytes_read);
	//sdWrite(&SD1, "|||\r\n", 5);

    //chThdSleepMilliseconds(500);
  }

  chHeapFree(gps_data);
}

void init_gps() {
	// GPS Thread
	chThdCreateStatic(waGPSThread, sizeof(waGPSThread), NORMALPRIO, GPSThread, NULL);
}

void gps_reset() {
	palSetPadMode(GPIO_GPS_PWR_PORT, GPIO_GPS_PWR_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	//! Set GPS power Off
	palClearPad(GPIO_GPS_PWR_PORT, GPIO_GPS_PWR_PIN);

	chThdSleepMilliseconds(500);

	//! Set GPS power On
	palClearPad(GPIO_GPS_PWR_PORT, GPIO_GPS_PWR_PIN);

	sdStop(&GPS_SERIAL);
	SD3_Config.sc_speed = 9600;
	sdStart(&GPS_SERIAL, &SD3_Config);

	// Wait until initialized
	while (sdReadTimeout(&GPS_SERIAL, gps_data, GPS_CMD_BUF, 100) <= 0) {}

	// Set serial config
	GPS_CMD_SEND("PSRF100,1,38400,8,1,0");
	chThdSleepMilliseconds(100);

	sdStop(&GPS_SERIAL);
	SD3_Config.sc_speed = 38400;
	sdStart(&GPS_SERIAL, &SD3_Config);

	// Disable unneeded
	GPS_CMD_SEND("PSRF103,01,00,00,01");
	chThdSleepMilliseconds(100);
	GPS_CMD_SEND("PSRF103,02,00,00,01");
	chThdSleepMilliseconds(100);
	GPS_CMD_SEND("PSRF103,03,00,00,01");
	chThdSleepMilliseconds(100);
	GPS_CMD_SEND("PSRF103,05,00,00,01");

	// Enable needed
	// GGA
	GPS_CMD_SEND("PSRF103,00,00,01,01");
	chThdSleepMilliseconds(100);
	// RMC
	GPS_CMD_SEND("PSRF103,04,00,01,01");
	chThdSleepMilliseconds(100);

	sdAsynchronousRead(&GPS_SERIAL, gps_data, GPS_CMD_BUF);
}

void gps_write_cmd(uint8_t * cmd_buf, uint8_t len) {
	uint8_t chksum = 0, i, num_len = 0;
	char num_buf[4];

	for (i = 0; i < len; i++)
		chksum ^= cmd_buf[i];

	chIOPut(&GPS_SERIAL, '$');

	sdWrite(&GPS_SERIAL, cmd_buf, len);

	chIOPut(&GPS_SERIAL, '*');

	stoh(chksum, num_buf, &num_len);
	sdWrite(&GPS_SERIAL, num_buf, num_len - 1);

	chIOPut(&GPS_SERIAL, '\r');
	chIOPut(&GPS_SERIAL, '\n');
}

uint8_t gps_read_msg(size_t *msg_len) {
	size_t bytes_readed, msg_readed = 0;
	uint8_t correct_byte_count = 0, incorrect_byte_count = 0;
	uint8_t is_reading = 1;
	uint8_t readed_byte;

	// Search for start symbol
	while (is_reading) {
		bytes_readed = sdReadTimeout(&GPS_SERIAL, &readed_byte, 1,
				correct_byte_count == 0 ? GPS_READ_TIMEOUT_TICS : GPS_READ_MSG_TIMEOUT_TICS);

		if (bytes_readed <= 0) {
			return E_READ_TIMEOUT;
		}

		switch (correct_byte_count) {
		case 0:
			if (readed_byte == '$') {
				correct_byte_count++;
			}

			if (incorrect_byte_count++ > 128)
				return E_INVALID_DATA;

			break;
		case 1:
			if (readed_byte != 'G') {
				return E_INVALID_DATA;
			}

			correct_byte_count++;
			break;
		case 2:
			if (readed_byte != 'P') {
				return E_INVALID_DATA;
			}

			correct_byte_count = 0;
			is_reading = 0;
			break;
		}

	}

	// Если мы здесь, значит $GP мы уже получили
	is_reading = 1;

	while (is_reading) {
		bytes_readed = sdReadTimeout(&GPS_SERIAL, &readed_byte, 1, GPS_READ_MSG_TIMEOUT_TICS);

		if (bytes_readed <= 0)
			return E_READ_TIMEOUT;

		gps_data[msg_readed++] = readed_byte;

		// If we read the end of the packet
		if (readed_byte == '*') {
			bytes_readed = sdReadTimeout(&GPS_SERIAL, gps_data + msg_readed, 2, GPS_READ_MSG_TIMEOUT_TICS);

			if (bytes_readed != 2) {
				return E_READ_TIMEOUT;
			}

			msg_readed += 2;
			break;
		}
	}

	*msg_len = msg_readed;

	return E_OK;
}

uint8_t gps_message_type() {
	if (gps_data[0] == 'R' &&
			gps_data[1] == 'M' &&
			gps_data[2] == 'C') {
		return GPS_MESSAGE_GPRMC;
	}

	if (gps_data[0] == 'G' &&
			gps_data[1] == 'G' &&
			gps_data[2] == 'A') {
		return GPS_MESSAGE_GPGGA;
	}

	if (gps_data[0] == 'G' &&
			gps_data[1] == 'L' &&
			gps_data[2] == 'L') {
		return GPS_MESSAGE_GPGLL;
	}

	if (gps_data[0] == 'G' &&
			gps_data[1] == 'S' &&
			gps_data[2] == 'A') {
		return GPS_MESSAGE_GPGSA;
	}

	if (gps_data[0] == 'G' &&
			gps_data[1] == 'S' &&
			gps_data[2] == 'V') {
		return GPS_MESSAGE_GPGSV;
	}

	if (gps_data[0] == 'V' &&
			gps_data[1] == 'T' &&
			gps_data[2] == 'G') {
		return GPS_MESSAGE_GPVTG;
	}

	if (gps_data[0] == 'Z' &&
			gps_data[1] == 'D' &&
			gps_data[2] == 'A') {
		return GPS_MESSAGE_GPZDA;
	}

	return GPS_MESSAGE_UNKNOWN;
}

uint8_t check_checksum() {
	// Checksum for 'GP'
	uint8_t chksum = 'G' ^ 'P';

	uint8_t i = 0, num_len = 0;
	char num_buf[4];

	while (i < (GPS_CMD_BUF - 2)) {
		if (gps_data[i] != '*') {
			chksum ^= gps_data[i];
		} else {
			break;
		}

		i++;
	}

	if (i == (GPS_CMD_BUF - 2))
		return E_LEN_ERROR;

	stoh(chksum, num_buf, &num_len);

	if (num_len == 2) {
		if (gps_data[i + 1] == '0' && gps_data[i + 2] == num_buf[0])
			return E_OK;
	} else if (num_len == 3) {
		if (gps_data[i + 1] == num_buf[0] && gps_data[i + 2] == num_buf[1])
			return E_OK;
	} else {
		return E_LEN_ERROR;
	}

	return E_CHKSUM_ERROR;
}

#define GPS_RMC_HOUR_OFFSET					4
#define GPS_RMC_MINUTE_OFFSET				6
#define GPS_RMC_SECOND_OFFSET				8
#define GPS_RMC_VALID_OFFSET				15
#define GPS_RMC_LATITUDE_DEG_OFFSET			17
#define GPS_RMC_LATITUDE_MIN_VAL_OFFSET		19
#define GPS_RMC_LATITUDE_MIN_FRAC_OFFSET	22
#define GPS_RMC_LATITUDE_NS_OFFSET			27
#define GPS_RMC_LONGITUDE_DEG_OFFSET		29
#define GPS_RMC_LONGITUDE_MIN_VAL_OFFSET	32
#define GPS_RMC_LONGITUDE_MIN_FRAC_OFFSET	35
#define GPS_RMC_LONGITUDE_EW_OFFSET			40
#define GPS_RMC_SPEED_OFFSET				42

uint8_t field_len(uint8_t *buf) {
	uint8_t i;

	for (i = 0; i < 16; i++) {
		if (buf[i] == ',' || buf[i] == '.')
			return i;
	}

	return 0;
}

uint8_t parse_gps_rmc(gps_rmc_state_t * state) {
	if (gps_message_type() != GPS_MESSAGE_GPRMC)
		return E_MSG_TYPE_ERR;

	uint8_t *parse_buffer[16];
	uint8_t i, len;

	state->hour = atos_len(gps_data + GPS_RMC_HOUR_OFFSET, 2);
	state->minute = atos_len(gps_data + GPS_RMC_MINUTE_OFFSET, 2);
	state->second = atos_len(gps_data + GPS_RMC_SECOND_OFFSET, 2);

	if (gps_data[GPS_RMC_VALID_OFFSET] == 'A') {
		state->flags &= ~GPS_DATA_INVALID;
	} else {
		state->flags |= GPS_DATA_INVALID;
	}

	state->latitude_degrees = atos_len(gps_data + GPS_RMC_LATITUDE_DEG_OFFSET, 2);

	state->latitude_seconds = atos_len(gps_data + GPS_RMC_LATITUDE_MIN_VAL_OFFSET, 2) * 10000;
	state->latitude_seconds += atos_len(gps_data + GPS_RMC_LATITUDE_MIN_FRAC_OFFSET, 4);

	state->flags &= ~(LATITUDE_N | LATITUDE_S);

	if (gps_data[GPS_RMC_LATITUDE_NS_OFFSET] == 'N') {
		state->flags |= LATITUDE_N;
	} else if (gps_data[GPS_RMC_LATITUDE_NS_OFFSET] == 'S') {
		state->flags |= LATITUDE_S;
	}

	state->longitude_degrees = atos_len(gps_data + GPS_RMC_LONGITUDE_DEG_OFFSET, 3);
	state->longitude_seconds = atos_len(gps_data + GPS_RMC_LONGITUDE_MIN_VAL_OFFSET, 2) * 10000;
	state->longitude_seconds += atos_len(gps_data + GPS_RMC_LONGITUDE_MIN_FRAC_OFFSET, 4);

	state->flags &= ~(LONGITUDE_E | LONGITUDE_W);

	if (gps_data[GPS_RMC_LONGITUDE_EW_OFFSET] == 'E') {
		state->flags |= LONGITUDE_E;
	} else if (gps_data[GPS_RMC_LONGITUDE_EW_OFFSET] == 'W') {
		state->flags |= LONGITUDE_W;
	}

	i = GPS_RMC_SPEED_OFFSET;

	len = field_len(gps_data + i);

	if (len != 0) {
		state->speed = atos_len(gps_data + i, len) * 100;
		i += len + 1;

		len = field_len(gps_data + i);

		if (len != 0) {
			state->speed += atos_len(gps_data + i, len);
		}
		i += len + 1;
	} else {
		i++;
		state->speed = 0;
	}

	len = field_len(gps_data + i);

	if (len != 0) {
		state->course = atos_len(gps_data + i, len) * 100;
		i += len + 1;

		len = field_len(gps_data + i);

		if (len != 0) {
			state->course += atos_len(gps_data + i, len);
		}
		i += len + 1;
	} else {
		i++;
		state->course = 0;
	}

	// Set data

	state->day = atos_len(gps_data + i, 2);
	i += 2;
	state->month = atos_len(gps_data + i, 2);
	i += 2;
	state->year = atos_len(gps_data + i, 2);

/*
	sdWrite(&SD1, "RMC PARSED:", sizeof("RMC PARSED:") - 1);

	SPRNT(state->hour, "Hour: ", 1);
	SPRNT(state->minute, "Minute: ", 1);
	SPRNT(state->second, "Second: ", 1);

	SPRNT(state->day, "Day: ", 1);
	SPRNT(state->month, "Month: ", 1);
	SPRNT(state->year, "Year: ", 1);

	SPRNT(state->speed, "Speed: ", 1);
	SPRNT(state->course, "Course: ", 1);

	SPRNT(state->flags, "Flags: ", 1);

	SPRNT(state->latitude_degrees, "Lat degrees: ", 1);
	SPRNT(state->latitude_seconds/10000, "Lat seconds: ", 1);
	SPRNT(state->latitude_seconds % 10000, "Lat seconds: ", 1);

	SPRNT(state->longitude_degrees, "Long degrees: ", 1);
	SPRNT(state->longitude_seconds/10000, "Long seconds: ", 1);
	SPRNT(state->longitude_seconds % 10000, "Long seconds: ", 1);
*/


	return E_OK;
}


