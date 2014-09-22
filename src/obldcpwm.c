/*
 * pwm.c
 *
 *  Created on: 22.05.2014
 *      Author: joerg
 */

#include "ch.h"
#include "hal.h"
#include "uart.h"
#include "uart_scp.h"
#include "pwm.h"

#include "obldc_def.h"
#include "obldcpwm.h"


#define ADC_COMMUTATE_NUM_CHANNELS  1
#define ADC_COMMUTATE_BUF_DEPTH     500
static adcsample_t commutatesamples[ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH];

uint32_t adc_commutate_count;
void reset_adc_commutate_count() {
	adc_commutate_count = 0;
}
/*
 * ADC streaming callback.
 */
static void adc_commutate_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;

  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;
  adc_commutate_count++;

}
static void adc_commutate_err_cb(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
  adc_commutate_count++;
}
/*
 * GPT Callback
 */
/*static void gpt_adc_trigger(GPTDriver *gpt_ptr)
{
	adc_commutate_count++;
}*/
/*
 * Configure a GPT object
 */
/*static GPTConfig gpt_adc_commutate_config =
{
	 2e6,  // timer clock: 1Mhz
	 gpt_adc_trigger //gpt_adc_trigger  // Timer callback function
};*/
static const ADCConversionGroup adc_commutate_group = {
		TRUE, // linear mode
		ADC_COMMUTATE_NUM_CHANNELS,
		adc_commutate_cb,
		adc_commutate_err_cb,
		0, // ADC_CR1
		ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2, // ADC_CR2: use ext event | select Timer3 TRGO event
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_COMMUTATE_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) // ADC_SQR3
};

static uint8_t halldecode[8];





void set_bldc_pwm_adc(int angle, int duty_cycle, int period) {
	// I wanted to use TIM3 timer to get values from ADC on a fixed time rate (100 ms). To do this I setup TIM3 peripheral generate TRGO event:
	// WARNING this is non-portable code!
	adcStopConversion(&ADCD1);
	TIM3->PSC   = (STM32_TIMCLK1/200)-1; // prescaler to get Timer 1 period as clock cycle for one tick
	TIM3->ARR   =  100;  // 100 periods
	TIM3->CNT = 0;
	TIM3->CR2 = TIM_CR2_MMS_1; // TRGO event is timer update event, e.g. overflow
	TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_OPM; // enable the timer / one pulse mode
	adcStartConversion(&ADCD1, &adc_commutate_group, commutatesamples, ADC_COMMUTATE_BUF_DEPTH);//HIER HAENGTS!
	//gptStart(&GPTD3, &gpt_adc_commutate_config);
	//gptStartOneShot(&GPTD3, 100);
	set_bldc_pwm(angle, duty_cycle, period);
}
/*
 * Generic PWM for BLDC motor operation.
 * duty_cycle in percent * 100
 * Period in microseconds
 */
void set_bldc_pwm(int angle, int duty_cycle, int period) {
	static PWMConfig genpwmcfg= {
			2e6, /* 2MHz PWM clock frequency */
			100, /* PWM period 50us */
			NULL,  /* No callback */
			{
					{PWM_OUTPUT_ACTIVE_HIGH, NULL},
					{PWM_OUTPUT_ACTIVE_HIGH, NULL},
					{PWM_OUTPUT_ACTIVE_HIGH, NULL},
					{PWM_OUTPUT_DISABLED, NULL},
			},
			0,//TIM_CR2_MMS_1, // 010: Update - The update event is selected as trigger output (TRGO). //TIM_CR2_MMS_2, // TIM CR2 register initialization data, OC1REF signal is used as trigger output (TRGO)
			0 // TIM DIER register initialization data, "should normally be zero"
	};



	genpwmcfg.period = 2 * period;
    pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    if (angle == 1) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_W_NDTS);
    } else if (angle == 2) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_W_NDTS);
    } else if (angle == 3) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_W_NDTS);
    } else if (angle == 4) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_W_NDTS);
    } else if (angle == 5) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_W_NDTS);
    } else if (angle == 6) {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	palSetPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palSetPad(GPIOB, GPIOB_W_NDTS);
    } else {
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_W_NDTS);
    }
}


static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;
  palSetPad(GPIOB, GPIOB_LEDR);
}

static void pwmc1cb(PWMDriver *pwmp) {

  (void)pwmp;
  palClearPad(GPIOB, GPIOB_LEDR);
}

static PWMConfig pwmcfg = {
  1e6,//2e6, /* 2MHz PWM clock frequency */
  1000,//200, /* PWM period 100us */
  pwmpcb,  /* No callback */
  /* Only channel 1 enabled */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
  },
  0, // TIM CR2 register initialization data, "should normally be zero"
  0 // TIM DIER register initialization data, "should normally be zero"
};

void mystartPWM(void) {
    /* Enables PWM output (of TIM1, channel 1) on "U" phase connected to PA8 */
    //palSetPadMode(GPIOA, GPIOA_U_PWM, PAL_MODE_STM32_ALTERNATE_PUSHPULL); // ist schon in board.h definiert
    pwmStart(&PWMD1, &pwmcfg);
    //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
    //palSetPad(GPIOB, GPIOB_U_NDTS); // activate driver
    palClearPad(GPIOB, GPIOB_U_NDTS); // deactivate driver
    pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
    palSetPad(GPIOB, GPIOB_V_NDTS); // activate driver
    //pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
    //palSetPad(GPIOB, GPIOB_W_NDTS); // activate driver
    palClearPad(GPIOB, GPIOB_W_NDTS); // deactivate driver
}

/* ---------- Catch mode ---------- */
void catchcycle_obsolete(int voltage_u, int voltage_v, int voltage_w, uint8_t init) {
	static int vdiff_1_last;
	static int vdiff_2_last;
	static int vdiff_3_last;
	static int last_zero_crossing;
	static int direction;
	static int stopped_count;
	int crossing_detected;
	int hall_1, hall_2, hall_3;
	int hall_code, hall_decoded, last_hall_decoded;

	if (init == 1) {
		halldecode[0]=0; halldecode[1]=4; halldecode[2]=2; halldecode[3]=3; halldecode[4]=6; halldecode[5]=5; halldecode[6]=1; halldecode[7]=0;
		vdiff_1_last = 0;
		vdiff_2_last = 0;
		vdiff_3_last = 0;
		last_zero_crossing = 0;
		direction = 0;
		stopped_count = 0;
		last_hall_decoded = 0;
	} else {
		// init run variables
		crossing_detected = 0;

		// determine voltage between the phases
		int vdiff_1 = voltage_v - voltage_u;
		int vdiff_2 = voltage_w - voltage_v;
		int vdiff_3 = voltage_u - voltage_w;

		// determine min and max values
		int vdiff_max = MAX(MAX(vdiff_1, vdiff_2), vdiff_3);
		int vdiff_min = MIN(MIN(vdiff_1, vdiff_2), vdiff_3);

		// when difference between min and max values > "MinCatchVoltage" -> Cond 1 fulfilled
		if (ABS(vdiff_max - vdiff_min) > OBLDC_MIN_CATCH_VOLTAGE_OBSOLETE) {
			// Cond 1 fulfilled
			// detect zero crossing of a phase difference voltage
			if (((vdiff_1 < 0) && (vdiff_1_last > 0)) || ((vdiff_1 > 0) && (vdiff_1_last < 0))) {
				// zero crossing on vdiff_1
				if (last_zero_crossing != 1) {
					crossing_detected = 1;
					last_zero_crossing = 1;
				}
			}

			if (((vdiff_2 < 0) && (vdiff_2_last > 0)) || ((vdiff_2 > 0) && (vdiff_2_last < 0))) {
				// zero crossing on vdiff_2
				if (last_zero_crossing != 2) {
					crossing_detected = 1;
					last_zero_crossing = 2;
				}
			}
			if (((vdiff_3 < 0) && (vdiff_3_last > 0)) || ((vdiff_3 > 0) && (vdiff_3_last < 0))) {
				// zero crossing on vdiff_3
				if (last_zero_crossing != 3) {
					crossing_detected = 1;
					last_zero_crossing = 3;
				}
			}

			if (crossing_detected == 1) {
				if (vdiff_1 > 0) {
					hall_1 = 1;
				} else {
					hall_1 = 0;
				}
				if (vdiff_2 > 0) {
					hall_2 = 1;
				} else {
					hall_2 = 0;
				}
				if (vdiff_3 > 0) {
					hall_3 = 1;
				} else {
					hall_3 = 0;
				}
				hall_code = hall_1 + hall_2 * 2 + hall_3 * 4;
				hall_decoded = halldecode[hall_code]; // determine motor angle in [1-6]

				crossing_detected = 0;

			}

			// check if distance to last 'angle' is = 1
			if (ABS(hall_decoded - last_hall_decoded) && (last_hall_decoded != 0)) {
				// determine direction of rotation
				if (hall_decoded > last_hall_decoded) {
					direction = 1;
				} else {
					direction = 2;
				}
			} else {
				last_hall_decoded = hall_decoded;
			}

			vdiff_1_last = vdiff_1;
			vdiff_2_last = vdiff_2;
			vdiff_3_last = vdiff_3;
		} else {
			// count 'elses': if > 10 -> Motor is stopped -> start startup algorithm
			stopped_count = stopped_count + 1;
			if (stopped_count > 10) {
				// start startup algorithm
			}
		}
		// neue adc-messung starten
		catchconversion();
	}
}
static void pwmcatchmodecb(PWMDriver *pwmp) {

  (void)pwmp;
  // evaluate last ADC measurement
  // determine voltage; for efficiency reasons, we calculating with the ADC value and do not convert to a float for [V]
  int voltage_u = getcatchsamples()[0]; // /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
  int voltage_v = getcatchsamples()[1]; // /4095.0 * 3 * 13.6/3.6;
  int voltage_w = getcatchsamples()[2]; // /4095.0 * 3 * 13.6/3.6;

  catchcycle_obsolete(voltage_u, voltage_v, voltage_w, FALSE); // evaluate measurements for 'hall decoding'
}

static PWMConfig pwmcatchmodecfg = {
  2e6, /* 2MHz PWM clock frequency */
  20000, /* PWM period 10ms (orig 100us) => Update-event 100Hz (orig 10kHz) */
  pwmcatchmodecb,  /* No callback */
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
  }, /* We only need the counter; do not generate any PWM output */
  0, // TIM CR2 register initialization data, "should normally be zero"
  0 // TIM DIER register initialization data, "should normally be zero"
};

void startcatchmodePWM(void) {
	/* 1. Trigger first ADC measurement
	 * 2. Start PWM timer for ADC triggering at 10kHz */
	//catchconversion(); // 1.
	//catchcycle_obsolete(0, 0, 0, TRUE); // initialize catch state variables
	//pwmStart(&PWMD3, &pwmcatchmodecfg);
}
