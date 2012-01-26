/*
 * power.c
 *
 *  Created on: 23.01.2012
 *      Author: dimaz
 */

#include "power.h"

#include "util.h"

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   2

/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP1_BUF_DEPTH      4

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

static adcsample_t ext_pwr_level, batt_level;

static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN10   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  NULL,
  /* HW dependent part.*/
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  0,
  ADC_SMPR2_SMP_AN10(ADC_SAMPLE_192) | ADC_SMPR2_SMP_SENSOR(ADC_SAMPLE_192),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  0,
  0,
  ADC_SQR5_SQ2_N(GPIO_12V_SENSE_CHANNEL) | ADC_SQR5_SQ1_N(GPIO_VBAT_SENSE_CHANNEL)
};

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {

    /* Calculates the average values from the ADC samples.*/
	ext_pwr_level = (samples[0] + samples[2] + samples[4] + samples[6]) / 4;
    batt_level = (samples[1] + samples[3] + samples[5] + samples[7]) / 4;

    //chSysLockFromIsr();

    //chSysUnlockFromIsr();
  }
}

void update_power_state() {
	ext_pwr_level = batt_level = 0;
	adcStartConversion(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
}

void print_power_state() {
	char num_buf[10];
	uint8_t num_len;

	sdWrite(&SD1, "POWER LEVEL: ", sizeof("POWER LEVEL: ") - 1);
	stoa(ext_pwr_level, num_buf, &num_len);
	sdWrite(&SD1, num_buf, num_len - 1);
	sdWrite(&SD1, " ", sizeof(" ") - 1);
	stoa(batt_level, num_buf, &num_len);
	sdWrite(&SD1, num_buf, num_len - 1);
	sdWrite(&SD1, "\r\n", sizeof("\r\n") - 1);
}

void init_power_ctl() {

	palSetPadMode(GPIO_12V_SENSE_PORT, GPIO_12V_SENSE_PIN, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIO_VBAT_SENSE_PORT, GPIO_VBAT_SENSE_PIN, PAL_MODE_INPUT_ANALOG);

	adcStart(&ADCD1, NULL);
	adcSTM32EnableTSVREFE();

}
