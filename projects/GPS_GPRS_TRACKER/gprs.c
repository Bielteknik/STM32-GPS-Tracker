#include "gprs.h"

#include "ch.h"
#include "hal.h"

#include <string.h>

#include "util.h"
#include "led.h"

#define GPRS_CMD_BUF	256
#define ATZ_RETRY		5
#define CMD_WAIT_TIME	250

#define GPRS_SERIAL		SD2

SerialConfig SD2_Config = {
		.sc_speed = 115200,
		.sc_cr2 = USART_CR2_STOP1_BITS,
		.sc_cr3 = USART_CR3_RTSE | USART_CR3_CTSE
};

uint8_t *gprs_data = NULL;

static WORKING_AREA(waGPRSThread, 256);
static msg_t GPRSThread(void *arg) {
	(void)arg;

	if (gprs_data != NULL) {
		chHeapFree(gprs_data);
	}

	gprs_data = chHeapAlloc(NULL, GPRS_CMD_BUF);
	size_t gprs_bytes_read;
	uint16_t signal_level;
	uint16_t i;
	char num_buf[16];
	uint8_t num_len;
	chRegSetThreadName("gprs_thread");

	if (gprs_data == NULL) {
		while (TRUE) {
			//palTogglePad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);
			chThdSleepMilliseconds(50);
		}
	}

	sdStart(&GPRS_SERIAL, &SD2_Config);
	palSetPadMode(GPRS_USART_PORT, GPRS_USART_TX_PIN, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPRS_USART_PORT, GPRS_USART_RX_PIN, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPRS_USART_PORT, GPRS_USART_CTS_PIN, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPRS_USART_PORT, GPRS_USART_RTS_PIN, PAL_MODE_ALTERNATE(7));

	palSetPadMode(GPIO_GPRS_PWR_BAT_PORT, GPIO_GPRS_PWR_BAT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIO_GPRS_RESET_PORT, GPIO_GPRS_RESET_PIN, PAL_MODE_OUTPUT_OPENDRAIN);

	// Turn on GPRS power
	palClearPad(GPIO_GPRS_PWR_BAT_PORT, GPIO_GPRS_PWR_BAT_PIN);

	chThdSleepSeconds(5);

	while (TRUE) {
		//palTogglePad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);

		if (init_modem() == E_OK)
			break;
	}

	set_led_0_prescaler(5);

	uint8_t counter = 0;

	while (TRUE) {
		counter++;
		if (counter == 10) {
			counter = 0;
			send_tcp_message();
		}

		//palTogglePad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);

		if (is_gprs_network_ok() == TRUE) {
			//sdWrite(&SD1, "NETWORK REGISTERED", sizeof("NETWORK REGISTERED") - 1);
		} else {
			//sdWrite(&SD1, "NO NETWORK", sizeof("NO NETWORK") - 1);
		}

		chThdSleepMilliseconds(100);

		if (gprs_get_signal_level(&signal_level) == E_OK) {

		/*	sdWrite(&SD1, "SIGNAL LEVEL: ", sizeof("SIGNAL LEVEL: ") - 1);
			stoa(signal_level, num_buf, &num_len);
			sdWrite(&SD1, num_buf, num_len - 1);
			sdWrite(&SD1, "\r\n", sizeof("\r\n") - 1);*/

		} else {
			//sdWrite(&SD1, "SIGNAL LEVEL: ERR\r\n", sizeof("SIGNAL LEVEL: ERR\r\n") - 1);
		}

		chThdSleepMilliseconds(1000);
	}

	chHeapFree(gprs_data);
}

uint8_t init_modem() {
	uint16_t i;
	uint16_t bytes_read;

	for (i = 0; i < ATZ_RETRY; i++) {
		sdWrite(&GPRS_SERIAL, "ATE0\r\n", sizeof("ATE0\r\n"));
		chThdSleepMilliseconds(200);

		if (gprs_cmd("ATZ\r\n", sizeof("ATZ\r\n") - 1, "\r\nOK\r\n", sizeof("\r\nOK\r\n") - 1) == E_OK)
			break;

		chThdSleepMilliseconds(500);
	}

	if (i >= ATZ_RETRY)
		return E_NOT_RESPONDING;


	return E_OK;
}

uint8_t gprs_cmd(char * cmd_str, uint16_t cmd_len, char * answer_str, uint16_t answer_len) {
	uint16_t bytes_read;

	// Flush buffer
	sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	sdWrite(&GPRS_SERIAL, cmd_str, cmd_len);
	chThdSleepMilliseconds(CMD_WAIT_TIME);

	if (answer_str == NULL) {
		return E_OK;
	}

	bytes_read = sdAsynchronousRead(&GPRS_SERIAL, gprs_data, answer_len);

	if (bytes_read == answer_len) {
		if (strncmp(gprs_data, answer_str, bytes_read) == 0) {
			return E_OK;
		}
	}

	return E_INVALID_ANSWER;
}

uint8_t gprs_cmd_read(char * cmd_str, uint16_t cmd_len, uint16_t *answer_len) {
	uint16_t bytes_read;

	sdWrite(&GPRS_SERIAL, "ATE0\r\n", sizeof("ATE0\r\n"));
	chThdSleepMilliseconds(200);

	// Flush buffer
	sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	sdWrite(&GPRS_SERIAL, cmd_str, cmd_len);
	chThdSleepMilliseconds(1000);
	bytes_read = sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	*answer_len = bytes_read;

	if (bytes_read == 0)
		return E_NOT_RESPONDING;

	return E_OK;
}

uint8_t gprs_get_signal_level(uint16_t *signalLevel) {
	uint16_t bytes_read;
	char signalLevelStr[3] = {0, 0, 0};

	if (gprs_cmd_read("AT+CSQ\r\n", sizeof("AT+CSQ\r\n") - 1, &bytes_read) != E_OK) {
		return E_NOT_RESPONDING;
	}

	if (bytes_read < 13)
		return E_INVALID_ANSWER;

	if (strncmp(gprs_data, "\r\n+CSQ:", sizeof("\r\n+CSQ:") - 1) != 0) {
		return E_INVALID_ANSWER;
	}

	signalLevelStr[0] = gprs_data[8];

	if (gprs_data[9] != ',')
		signalLevelStr[1] = gprs_data[9];

	*signalLevel = atos(signalLevelStr);

	return E_OK;
}

void init_gprs() {
	// GPRS Thread
	chThdCreateStatic(waGPRSThread, sizeof(waGPRSThread), NORMALPRIO, GPRSThread, NULL);
}

uint8_t is_gprs_network_ok() {
	sdWrite(&GPRS_SERIAL, "ATE0\r\n", sizeof("ATE0\r\n"));
	chThdSleepMilliseconds(200);

	if (gprs_cmd("AT+CREG?\r\n", sizeof("AT+CREG?\r\n") - 1, "\r\n+CREG 0,1\r\n", sizeof("\r\n+CREG 0,1\r\n") - 1) == E_OK)
		return TRUE;

	return FALSE;
}

uint8_t send_tcp_message() {
	uint16_t bytes_read;
	uint8_t i;

	if (is_gprs_network_ok() != TRUE) {

		//	return E_NO_NETWORK;
	}

	chThdSleepMilliseconds(100);

	// Enable embedded TCP/IP stack
	gprs_cmd("AT+WIPCFG=1\r\n", sizeof("AT+WIPCFG=1\r\n") - 1, NULL, 0);

	chThdSleepMilliseconds(250);
	sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	// Open GPRS bearer
	gprs_cmd("AT+WIPBR=1,6\r\n", sizeof("AT+WIPBR=1,6\r\n") - 1, NULL, 0);

	chThdSleepMilliseconds(250);
	sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	// Set GPRS AP
	gprs_cmd("AT+WIPBR=2,6,11,\"internet\"\r\n", sizeof("AT+WIPBR=2,6,11,\"internet\"\r\n") - 1, NULL, 0);

	chThdSleepMilliseconds(250);
	sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	// Connect to GPRS
	gprs_cmd("AT+WIPBR=4,6,0\r\n", sizeof("AT+WIPBR=4,6,0\r\n") - 1, NULL, 0);

	chThdSleepSeconds(3);

	bytes_read = sdAsynchronousRead(&GPRS_SERIAL, gprs_data, GPRS_CMD_BUF);

	if (bytes_read > 0) {
		if (strncmp(gprs_data, "\r\nOK", sizeof("\r\nOK") - 1) != 0) {


			//return E_GPRS_CONNECT_ERROR;
		}
	} else {
		//return E_GPRS_CONNECT_ERROR;
	}

	// Establish connection
	gprs_cmd("AT+WIPCREATE=2,1,\"195.209.231.43\",5555\r\n", sizeof("AT+WIPCREATE=2,1,\"195.209.231.43\",5555\r\n") - 1, NULL, 0);

	chThdSleepSeconds(1);

	gprs_cmd("AT+WIPDATA=2,1,1\r\n", sizeof("AT+WIPDATA=2,1,1\r\n") - 1, NULL, 0);

	chThdSleepSeconds(3);

	gprs_cmd("Hello from wismo!\r\n", sizeof("Hello from wismo!\r\n") - 1, NULL, 0);

	chThdSleepMilliseconds(250);
	gprs_cmd("+++", sizeof("+++") - 1, NULL, 0);
	chThdSleepMilliseconds(250);

}
