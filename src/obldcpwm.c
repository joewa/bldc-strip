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

motor_s motor;	// Stores all motor data


#define ADC_COMMUTATE_NUM_CHANNELS  5
#define ADC_COMMUTATE_BUF_DEPTH     8
static adcsample_t commutatesamples[ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH];

#define PWM_CLOCK_FREQUENCY			2e6 	// [Hz]
#define PWM_DEFAULT_FREQUENCY		20e3	// [Hz]

uint16_t table_angle2leg[7];
void init_motor_struct(motor_s* motor) {
	motor->state			= OBLDC_STATE_OFF;
	motor->pwm_mode			= PWM_MODE_SINGLEPHASE;
	motor->pwm_duty_cycle	= 0;
	motor->pwm_frequency	= PWM_DEFAULT_FREQUENCY;
	motor->angle			= 0;
	motor->direction		= 0;
	table_angle2leg[0]=0;
	table_angle2leg[1]=0;
	table_angle2leg[2]=2;
	table_angle2leg[3]=2;
	table_angle2leg[4]=1;
	table_angle2leg[5]=1;
	table_angle2leg[6]=0;
}


uint32_t adc_commutate_count;
void reset_adc_commutate_count() {
	adc_commutate_count = 0;
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

static PWMConfig genpwmcfg= {
		PWM_CLOCK_FREQUENCY, /* 2MHz PWM clock frequency */
		PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY, /* PWM period 50us */
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


/*
 * ADC streaming callback.
 */
static void adc_commutate_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void)adcp;

  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;
  uint16_t csamples[ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH];
  int i,a,b;
  chSysLockFromISR();
  /*if (!adc_commutate_count) { // Start PWM so that ADC is in Sync with Timer 1
	  pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
	  chSysLockFromISR();
	  pwmEnableChannelI(&PWMD1, table_angle2leg[motor.angle], PWM_PERCENTAGE_TO_WIDTH(&PWMD1, motor.pwm_duty_cycle));
  } else chSysLockFromISR();*/

  for (i=0; i<ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH; i++ ) csamples[i] = commutatesamples[i];


  /*for (i=0; i<39; i++){
	  if(csamples[i] > 2500){
		  a=csamples[i];
		  b=i;
	  }
  }*/

  if (adc_commutate_count >= 1) {
	  /*pwmEnableChannelI(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
	  pwmEnableChannelI(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
	  pwmEnableChannelI(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
	  adcStopConversionI(&ADCD1); // HERE breakpoint
	  pwmStop(&PWMD1); // PWM signal generation*/
	  adc_commutate_count = 0;
  }
  adc_commutate_count++;
  chSysUnlockFromISR();
}
static void adc_commutate_err_cb(ADCDriver *adcp, adcerror_t err) {

  (void)adcp;
  (void)err;
  //adc_commutate_count++;

}

/**
 * adc_commutate_group is used for back-emf sensing to determine when the motor shall commutate.
 */
static const ADCConversionGroup adc_commutate_group = {
		TRUE, // linear mode
		ADC_COMMUTATE_NUM_CHANNELS,
		adc_commutate_cb,
		adc_commutate_err_cb,
		0, // ADC_CR1
		//0, // ADC_CR2
		//ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2, // ADC_CR2: use ext event | select Timer3 TRGO event
		ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0, // | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0, // ADC_CR2: use ext event | select SWSTART event
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_COMMUTATE_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		// ADC regular sequence register 3 (ADC_SQR3): U_VOLTAGE, V_VOLTAGE, W_VOLTAGE, CURRENT, CURRENTREF (see schematic)
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)// ADC_SQR3
};

static uint8_t halldecode[8];



void set_bldc_pwm_adc(int angle, int duty_cycle, int period) {
	// I wanted to use TIM3 timer to get values from ADC on a fixed time rate (100 ms). To do this I setup TIM3 peripheral generate TRGO event:
	// WARNING this is non-portable code!
	adcStopConversion(&ADCD1);
	//Bit definition for RCC_APB1ENR register
	//RCC_APB1ENR_TIM3EN    //< Timer 3 clock enable
	//RCC->APB1ENR = RCC->APB1ENR | RCC_APB1ENR_TIM3EN;
    rccEnableTIM3(FALSE); // taken from gpt_lld_start in gpt_lld.c
    rccResetTIM3();
    TIM3->PSC   = (STM32_TIMCLK1/1000)-1; // prescaler to get Timer 1 period as clock cycle for one tick
    TIM3->ARR   =  100;  // Time constant;  Here: 100 periods
    //TIM3->EGR   = STM32_TIM_EGR_TG | STM32_TIM_EGR_UG;          // Trigger generation | Update event. // Ist Quatsch: Erzeugt Events "von Hand"
    TIM3->CNT   = 0;                         // Reset counter.
    TIM3->CR1 = TIM_CR1_OPM; // one pulse mode
    //TIM3->CR1 = TIM3->CR1 | STM32_TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt or DMA request if enabled
    TIM3->CR2 = TIM_CR2_MMS_1; // TRGO event is timer update event, e.g. overflow
    TIM3->CR1 = TIM3->CR1 | TIM_CR1_CEN ; //  enable the timer

    adcStartConversion(&ADCD1, &adc_commutate_group, commutatesamples, ADC_COMMUTATE_BUF_DEPTH);
	//gptStart(&GPTD3, &gpt_adc_commutate_config);
	//gptStartOneShot(&GPTD3, 100);
	int i,x;
	for (i=0; i<100000; i++) { // waste some time
		x=2*i;
	}
	ADC1->CR2 = ADC1->CR2 | ADC_CR2_SWSTART;

	for (i=0; i<100000; i++) { // waste some time
		x=2*i;
	}
	//set_bldc_pwm(angle, duty_cycle, period); // set breakpoint here and check TIM3->CNT
}
/*
 * Generic PWM for BLDC motor operation.
 * duty_cycle in percent * 100
 * Period in microseconds
 */
void set_bldc_pwm(motor_s* m) { // Mache neu mit motor_struct (pointer)
	int angle, duty_cycle, frequency;
	angle 		= m->angle;
	duty_cycle 	= m->pwm_duty_cycle;
	frequency	= m->pwm_frequency;
	//pwmStop(&PWMD1);
	if (m->state == OBLDC_STATE_OFF || m->state == OBLDC_STATE_CATCHING) { // PWM OFF!
		angle = 0;
	}
	adcStopConversion(&ADCD1);
	pwmStop(&PWMD1);
	//genpwmcfg.period = PWM_CLOCK_FREQUENCY / frequency
    if (angle < 1 || angle > 6) { // no angle --> all legs to gnd
    	pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_U_NDTS);
    	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_V_NDTS);
    	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	palClearPad(GPIOB, GPIOB_W_NDTS);
    } else {
    	if (angle == 1) {
    		//activeLegID = 0;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		//pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 2) {
    		//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_U_NDTS);
    		//pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		//activeLegID = 2;
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 3) {
    		//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		//pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_V_NDTS);
    		//activeLegID = 2;
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 4) {
    		//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		//activeLegID = 1;
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 5) {
    		//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_U_NDTS);
    		//activeLegID = 1;
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 6) {
    		//activeLegID = 0;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		//pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palClearPad(GPIOB, GPIOB_V_NDTS);
    		//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	}

    	//adcStart(&ADCD1, &adc_commutate_group);
    	int i,x;
    	if (m->state == OBLDC_STATE_RUNNING) {
    		adc_commutate_count = 0;
    		//pwmEnableChannel(&PWMD1, table_angle2leg[angle], PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    		adcStartConversion(&ADCD1, &adc_commutate_group, commutatesamples, ADC_COMMUTATE_BUF_DEPTH);
    		pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    		pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 100));
    		for (i=0; i<100000; i++) { // waste some time
    			x=2*i;
    		}
    		ADC1->CR2 = ADC1->CR2 | ADC_CR2_SWSTART;  // Software trigger ADC conversion (NOT WORKING YET)
    		//Hier einfach PWM und ADC direkt nacheinander starten
    	} else if (m->state == OBLDC_STATE_STARTING){
    		pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    		pwmEnableChannel(&PWMD1, table_angle2leg[angle], PWM_PERCENTAGE_TO_WIDTH(&PWMD1, duty_cycle));
    	} else {
    		//pwmEnableChannel(&PWMD1, table_angle2leg[angle], PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 0));
    	}
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
