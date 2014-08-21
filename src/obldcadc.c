/*
 * obldcadc.c
 *
 *  Created on: 31.05.2014
 *      Author: joerg
 */


#include "ch.h"
#include "hal.h"

#include "obldcadc.h"
#include "obldcpwm.h"

#define ADC_GRP1_NUM_CHANNELS   8
#define ADC_GRP1_BUF_DEPTH      1

#define ADC_CATCH_NUM_CHANNELS  3
#define ADC_CATCH_BUF_DEPTH     1

static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
static adcsample_t catchsamples[ADC_CATCH_NUM_CHANNELS * ADC_CATCH_BUF_DEPTH];

uint32_t adccount;
void resetadccount() {
	adccount = 0;
}
/*
 * ADC streaming callback.
 */
static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;

  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;
  adccount++;

}

static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC streaming callback for catching mode
 */
/*static void adccatchcallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;

  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;
  //adccount++;
  float voltage_u = catchsamples[0]/4095.0 * 3 * 13.6/3.6; // convert to voltage: *3 = ADC pin voltage, *13.6/3.6 = phase voltage
  float voltage_v = catchsamples[1]/4095.0 * 3 * 13.6/3.6;
  float voltage_w = catchsamples[2]/4095.0 * 3 * 13.6/3.6;

  float vdiff_1 = voltage_v - voltage_u;
  float vdiff_2 = voltage_w - voltage_v;
  float vdiff_3 = voltage_u - voltage_w;


}*/

static void adccatcherrorcallback(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
}

/*
 * ADC conversion group.
 * Mode:        Continuous, 8 samples of 1 channel, SW triggered.
 * Channels:    IN0.
 */
static const ADCConversionGroup adcgrpcfg1 = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0, ADC_CR2_TSVREFE,           // ADC_CR1, ADC_CR2; ADC_CR2_TSVREFE = Temperature sensor and V REFINT enable
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_239P5) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_239P5), // ADC sample time register 1 (ADC_SMPR1)
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_41P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_41P5) |
  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_41P5) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_41P5) |
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_41P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_41P5),                            // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),  // ADC regular sequence register (ADC_SQRx)
  ADC_SQR2_SQ8_N(ADC_CHANNEL_SENSOR) | ADC_SQR2_SQ7_N(ADC_CHANNEL_VREFINT),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) |
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
};

/* static const ADCConversionGroup adcgrpcfg1 = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0, ADC_CR2_TSVREFE,           // ADC_CR1, ADC_CR2; ADC_CR2_TSVREFE = Temperature sensor and V REFINT enable
  ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_28P5) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_1P5), // ADC sample time register 1 (ADC_SMPR1)
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5) |
  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_1P5) |
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_1P5),                            // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),  // ADC regular sequence register (ADC_SQRx)
  ADC_SQR2_SQ8_N(ADC_CHANNEL_SENSOR) | ADC_SQR2_SQ7_N(ADC_CHANNEL_VREFINT),
  ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) |
  ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) |
  ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
}; */

/* static const ADCConversionGroup adcgrpcfg1 = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerrorcallback,
  0, ADC_CR2_TSVREFE,           // ADC_CR1, ADC_CR2; ADC_CR2_TSVREFE = Temperature sensor and V REFINT enable
  0, // ADC sample time register 1 (ADC_SMPR1)
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5),                            // SMPR2
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),  // ADC regular sequence register (ADC_SQRx)
  0,
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
};*/

static void testcallback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;
  int a = 1;
  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;

}

/*
 * ADC conversion configuration for catching mode
 */
static const ADCConversionGroup adccatchgroup = {
		FALSE, // linear mode
		ADC_CATCH_NUM_CHANNELS,
		testcallback, //adccatchcallback,
		adccatcherrorcallback,
		0, // ADC_CR1
		0, // ADC_CR2
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_CATCH_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) // ADC_SQR3
};
void startmyadc(void) {
	adcStart(&ADCD1, NULL);
	//adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
}

void catchconversion(void) {
	//adcConvert(&ADCD1, &adccatchgroup, catchsamples, ADC_CATCH_BUF_DEPTH);
	adcStartConversion(&ADCD1, &adccatchgroup, catchsamples, ADC_CATCH_BUF_DEPTH);
	//adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
}

int catchmotor(void) {
	// Set disable signal to open all switches
	palClearPad(GPIOB, GPIOB_U_NDTS);
	palClearPad(GPIOB, GPIOB_V_NDTS);
	palClearPad(GPIOB, GPIOB_W_NDTS);
	// start measurements
	startcatchmodePWM();
	return 1; // TODO: Do sth useful
}

adcsample_t* getcatchsamples(void) {
	return catchsamples;
}
