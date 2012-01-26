#include "led.h"

#include "ch.h"
#include "hal.h"

volatile uint8_t led_0_prescaler = 10, led_1_prescaler = 10;
volatile uint8_t led_0_counter = 0, led_1_counter = 0;

// LED blinker thread
static WORKING_AREA(waLedBlinkThread, 8);
static msg_t ledBlinkThread(void *arg) {
	(void)arg;
	chRegSetThreadName("led_blinker");

	palSetPadMode(GPIO_LED_0_PORT, GPIO_LED_0_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIO_LED_1_PORT, GPIO_LED_1_PIN, PAL_MODE_OUTPUT_PUSHPULL);

	palClearPad(GPIO_LED_0_PORT, GPIO_LED_0_PIN);
	palSetPad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);

	while (TRUE) {
		if (led_0_counter++ == led_0_prescaler) {
			palTogglePad(GPIO_LED_0_PORT, GPIO_LED_0_PIN);
			led_0_counter = 0;
		}

		if (led_1_counter++ == led_1_prescaler) {
			palTogglePad(GPIO_LED_1_PORT, GPIO_LED_1_PIN);
			led_1_counter = 0;
		}

		chThdSleepMilliseconds(50);
	}
}

void start_led_thread() {
	// LED Thread
	chThdCreateStatic(waLedBlinkThread, sizeof(waLedBlinkThread), IDLEPRIO, ledBlinkThread, NULL);
}

void set_led_0_prescaler(uint8_t prescaler) {
	led_0_prescaler = prescaler;
	led_0_counter = 0;
}

void set_led_1_prescaler(uint8_t prescaler) {
	led_1_prescaler = prescaler;
	led_1_counter = 0;
}
