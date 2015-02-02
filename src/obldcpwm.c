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
#include "ringbuffer.h"


motor_s motor;	// Stores all motor data

//#define ADC_COMMUTATE_NUM_CHANNELS  5
//#define ADC_COMMUTATE_BUF_DEPTH     8
//f_single =    2.0000e+05
//T_cb_ADC1 =    2.0000e-05
//f_cb_ADC1 =    5.0000e+04

#define ADC_COMMUTATE_NUM_CHANNELS 1
#define ADC_COMMUTATE_BUF_DEPTH     50
#define NREG 20 // Number of samples for a valid regression
#define DROPNOISYSAMPLES 1 // "Drop samples with switching noise

typedef struct {
  int16_t size;
  int16_t start;
  int16_t end;
  int16_t elems[NREG+1];
} commutate_Buffer;

commutate_Buffer xbuf, ybuf;


static adcsample_t commutatesamples[ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH];

#define PWM_CLOCK_FREQUENCY			28e6 //2e6 //14e6 	// [Hz]
#define PWM_DEFAULT_FREQUENCY		100e3 //160e3 //100e3	// [Hz]

#define ADC_COMMUTATE_FREQUENCY		1e6		// [Hz]
#define ADC_PWM_DIVIDER				(PWM_CLOCK_FREQUENCY / ADC_COMMUTATE_FREQUENCY)


#define ADC_VBAT_CURRENT_NUM_CHANNELS 3
#define ADC_VBAT_CURRENT_BUF_DEPTH 1

static adcsample_t vbat_current_samples[ADC_VBAT_CURRENT_NUM_CHANNELS * ADC_VBAT_CURRENT_BUF_DEPTH];



void startmyadc(void) {
	adcStart(&ADCD1, NULL);
}


//uint8_t table_angle2leg[7];
//uint8_t table_angle2leg2[7];
motor_s* get_motor_ptr(void) {
	return &motor;
}

void init_motor_struct(motor_s* motor) {
	motor->state			= OBLDC_STATE_OFF;
	motor->pwm_mode			= PWM_MODE_ANTIPHASE; //PWM_MODE_SINGLEPHASE;
	motor->pwm_t_on			= 0;
	motor->pwm_period		= PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY; // in ticks
	motor->angle			= 0;
	motor->direction		= 0;
	motor->sumx=0; motor->sumx2=0; motor->sumxy=0; motor->sumy=0; motor->sumy2=0;
	/*table_angle2leg[0]=0; table_angle2leg2[0]=0; // 0,  0,  0,  0   SenseBridgeSign
	table_angle2leg[1]=0; table_angle2leg2[0]=1; // 1,  1, -1,  0		-1
	table_angle2leg[2]=2; table_angle2leg2[0]=1; // 2,  0, -1,  1		1
	table_angle2leg[3]=2; table_angle2leg2[0]=0; // 3, -1,  0,  1		-1
	table_angle2leg[4]=1; table_angle2leg2[0]=0; // 4, -1,  1,  0		1
	table_angle2leg[5]=1; table_angle2leg2[0]=2; // 5,  0,  1, -1		-1
	table_angle2leg[6]=0; table_angle2leg2[0]=2; // 6,  1,  0, -1		1*/
}

void motor_set_duty_cycle(motor_s* m, int d) {
	if(m->pwm_mode == PWM_MODE_SINGLEPHASE)
		m->pwm_t_on = m->pwm_period * d / 10000;
	else
		m->pwm_t_on = m->pwm_period * (5000 + d / 2) / 10000;
	m->pwm_t_on_ADC = m->pwm_t_on / ADC_PWM_DIVIDER;
	m->pwm_period_ADC = m->pwm_period / ADC_PWM_DIVIDER;
	m->sumx=0; m->sumx2=0; m->sumxy=0; m->sumy=0; m->sumy2=0;
	m->invSenseSign = m->angle % 2;
	//m->u_dc = get_vbat_sample();
	bufferInitStatic(xbuf, NREG); bufferInitStatic(ybuf, NREG);
}

// TODO: void motor_set_period(motor_s* m, int period)
// TODO: void motor_set_pwm_mode(motor_s* m, obldc_pwm_mode pwm_mode)

uint16_t k_cb_commutate; // Counts how often the ADC callback was called
void reset_adc_commutate_count() {
	k_cb_commutate = 0;
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


uint16_t yreg[(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2 + 1]; //[NREG+1]
//uint16_t xreg[(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2 + 1]; //[NREG+1]

/*
 * The rotor (magnets) have a permeability which causes an emf in the open phase.
 * The emf is zero when the rotor angle is where either no torque (e.g. 0°) is produced or the torque is maximum (e.g. at 90°).
 * The current ripple (determined by observing the power consumption of the ESC) is minimal at maximum torque.
 * At a battery voltage of 11.5V the maximal V-pp is about 3.0V
 */

static void adc_commutate_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  //adcsample_t avg_ch1 = (samples1[0] + samples1[1] + samples1[2] + samples1[3] + samples1[4] + samples1[5] + samples1[6] + samples1[7]) / 8;
  //float voltage = avg_ch1/4095.0*3;
  uint16_t* csamples = yreg;
  int i;
  int k_sample; // Sample in the present commutation cycle
  uint16_t k_pwm_period;//Indicates if the pwm sample occurred at pwm-on
  int16_t y_on, y_off, x_old, y_old;
  int64_t m_reg, b_reg, reg_den, k_zc;

  commutate_Buffer* xbuf_ptr;
  commutate_Buffer* ybuf_ptr;

  chSysLockFromISR();
  //u_dc_int = get_vbat_sample();

  xbuf_ptr = &xbuf; ybuf_ptr = &ybuf;
  //k_start = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  //k_end = k_start + (ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2;
  k_sample = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  for (i=0; i<(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2; i++ ) {// halbe puffertiefe
  //for (k_sample = k_start; k_sample < k_end; k_sample++ ) {// halbe puffertiefe
	  k_pwm_period = k_sample % motor.pwm_period_ADC;
	  //if ( k_pwm_period > DROPNOISYSAMPLES && k_pwm_period < motor.pwm_t_on_ADC) { // Samples during t_on!!!
	  if ( k_pwm_period == DROPNOISYSAMPLES + 2) { // Nimm erstmal nur das erste gueltige Sample von t_on
		  // BEGIN; THE NEW SIMPLE STUFF
		  if (motor.invSenseSign)
			  y_on = -(buffer[i] - motor.u_dc);  // Sensebridgesign
		  else
			  y_on = buffer[i] - motor.u_dc;
	  }
	  //else if (k_pwm_period > motor.pwm_t_on_ADC + 1 + DROPNOISYSAMPLES && k_pwm_period < motor.pwm_period_ADC)  {// Samples during t_off
	  else if (k_pwm_period > motor.pwm_t_on_ADC + 2 + DROPNOISYSAMPLES)  {// // Nimm erstmal nur das erste gueltige Sample von t_off
		  if (motor.invSenseSign)
			  y_off = -(buffer[i] - motor.u_dc);  // Sensebridgesign
		  else
			  y_off = buffer[i] - motor.u_dc;

		  if(y_on+500 < y_off) {// Detect zero crossing here
			  adcStopConversionI(&ADCD1);
			  pwmStop(&PWMD1);
			  chSysUnlockFromISR();// HERE breakpoint
			  return;
		  }

	  }

		  // END; THE NEW SIMPLE STUFF

			 //|| (k_pwm_period > motor.pwm_t_on_ADC + 1 + DROPNOISYSAMPLES && k_pwm_period < motor.pwm_period_ADC) ) {// Samples during t_off
		  //BEGIN COMMENT
		  /*
		  if (isBufferFull(ybuf_ptr)) {
			  bufferRead(xbuf_ptr, x_old); bufferRead(ybuf_ptr, y_old);
			  // Decrement obsolete buffer values from sums
		      motor.sumx -= x_old;
		      motor.sumx2 -= (int64_t)x_old * (int64_t)x_old;
		      motor.sumxy -= (int64_t)x_old * (int64_t)y_old;
		      motor.sumy -= y_old;
		      motor.sumy2 -= (int64_t)y_old * (int64_t)y_old;
		  }
		  if (motor.invSenseSign)
			  y = -(buffer[i] - motor.u_dc);  // Sensebridgesign
		  else
			  y = buffer[i] - motor.u_dc;
		  //bufferWrite(ybuf_ptr, buffer[i]);
		  bufferWrite(ybuf_ptr, y);
		  bufferWrite(xbuf_ptr, k_sample);
	      motor.sumx += k_sample;
	      motor.sumx2 += (int64_t)k_sample * (int64_t)k_sample;
	      motor.sumxy += (int64_t)k_sample * (int64_t)y;
	      motor.sumy += y;
	      motor.sumy2 += (int64_t)y * (int64_t)y;*/ // END COMMENT
		  /*
		   * Keep it simple: Chibios is blocked while doing all the stuff above.
		   * Consider simple zero crossing detection instead!
		   * Goenne dir ein paar confirmation-cycles. Nimm den tollen Ringpuffer dafuer
		   * Next step TODO:
		   * Schedule a general purpose timer and that is properly referenced
		   */
	  k_sample++;
	  csamples[i] = buffer[i];
  }
  // Check for timeout
  if(k_sample > 1000000) {  // TIMEOUT
	  k_sample++;
	  adcStopConversionI(&ADCD1); // HERE breakpoint
	  chSysUnlockFromISR();
	  return;
  }

/*
  reg_den = NREG * motor.sumx2  -  motor.sumx * motor.sumx;
  if( isBufferFull(ybuf_ptr) && (reg_den != 0) ) {
	  m_reg = (NREG * motor.sumxy  -  motor.sumx * motor.sumy) / reg_den;
	  b_reg = (motor.sumy * motor.sumx2  -  motor.sumx * motor.sumxy) / reg_den; // TODO: PUT BREAKPOINT HERE and check m_reg (vs motor speed)
	  if( m_reg < 0 ) {
		  k_zc = -b_reg / m_reg;
		  //if(k_zc < k_sample)
	  }
  }
*/

  k_cb_commutate++; // k_cb_ADC++; PUT BREAKPOINT HERE
  chSysUnlockFromISR();
}

static void adc_commutate_err_cb(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  //adc_commutate_count++;
}


static uint8_t halldecode[8];



static const ADCConversionGroup adc_vbat_current_group = {
		FALSE, // linear mode
		ADC_VBAT_CURRENT_NUM_CHANNELS,
		NULL, // no callback and end of conversion
		NULL,
		0, // ADC_CR1
		0, // ADC_CR2
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_239P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_VBAT_CURRENT_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN5)   // ADC_SQR3
};
/**
 * adc_commutate_group is used for back-emf sensing to determine when the motor shall commutate.
 */
/*static const ADCConversionGroup adc_commutate_group = {
		TRUE, // linear mode
		ADC_COMMUTATE_NUM_CHANNELS,
		adc_commutate_cb,
		adc_commutate_err_cb,
		0, // ADC_CR1
		//0, // ADC_CR2
		//ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_2, // ADC_CR2: use ext event | select Timer3 TRGO event
		ADC_CR2_EXTTRIG, // | ADC_CR2_EXTSEL_2 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0, // ADC_CR2: use ext event | select SWSTART event
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_COMMUTATE_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		// ADC regular sequence register 3 (ADC_SQR3): U_VOLTAGE, V_VOLTAGE, W_VOLTAGE, CURRENT, CURRENTREF (see schematic)
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ4_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN5)// ADC_SQR3
};*/

static ADCConversionGroup adc_commutate_group = {
		TRUE, // circular mode
		ADC_COMMUTATE_NUM_CHANNELS,
		adc_commutate_cb,
		adc_commutate_err_cb,
		0, // ADC_CR1
		ADC_CR2_EXTTRIG | ADC_CR2_CONT, // ADC_CR2: use ext event | select TIM1_CC1 event | Cont-mode (start once, always run)
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_COMMUTATE_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		// ADC regular sequence register 3 (ADC_SQR3): U_VOLTAGE, V_VOLTAGE, W_VOLTAGE, CURRENT, CURRENTREF (see schematic)
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)// ADC_SQR3
};


void v_bat_current_conversion(void) {
	adcStartConversion(&ADCD1, &adc_vbat_current_group, vbat_current_samples, ADC_VBAT_CURRENT_BUF_DEPTH);
}

adcsample_t* get_vbat_current_samples(void) {
	return vbat_current_samples;
}

adcsample_t get_vbat_sample(void) { // value scaled to be 50% of phase voltage sample
	// /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
	// the voltage divider at v_bat is 1.5 and 10 kOhm
	// So, the transformation is 115*36 / (15*136)
	int v_scaled;
	v_scaled = (int)vbat_current_samples[0] * 4140 / 2040 / 2;
	motor.u_dc = v_scaled; // UGLY!
	return (adcsample_t)v_scaled;
}



/*
 * Generic PWM for BLDC motor operation.
 * duty_cycle in percent * 100
 * Period in microseconds
 */
void set_bldc_pwm(motor_s* m) { // Mache neu mit motor_struct (pointer)
	int angle, t_on, period, inv_duty_cycle;
	uint8_t legp, legn; // Positive and negative PWM leg
	angle 	= m->angle;
	t_on	= m->pwm_t_on;
	period	= m->pwm_period;

	if (m->state == OBLDC_STATE_OFF || m->state == OBLDC_STATE_CATCHING) { // PWM OFF!
		angle = 0;
	}
	adcStopConversion(&ADCD1);
	palClearPad(GPIOB, GPIOB_U_NDTS); palClearPad(GPIOB, GPIOB_V_NDTS); palClearPad(GPIOB, GPIOB_W_NDTS);
	pwmStop(&PWMD1);
	//genpwmcfg.period = PWM_CLOCK_FREQUENCY / frequency
	//genpwmcfg.CR1 |= TIM_CR1_CMS_0;
    if (angle < 1 || angle > 6) { // no angle --> all legs to gnd
    	//pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    	//pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 10000));
    	legp = 3;
    	legn = 3;
    	palClearPad(GPIOB, GPIOB_U_NDTS);
    	//pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 10000));
    	palClearPad(GPIOB, GPIOB_V_NDTS);
    	//pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 10000));
    	palClearPad(GPIOB, GPIOB_W_NDTS);
    } else {
    	if (angle == 1) { // sample W_VOLTAGE, triggered by U_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; // ADC_CR2: select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2); // W_VOLTAGE
    		legp = 0; legn = 1;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		palClearPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 2) { // sample U_VOLTAGE, triggered by W_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC3 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		legp = 2; legn = 1;
    		palClearPad(GPIOB, GPIOB_U_NDTS);
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 3) { // sample V_VOLTAGE, triggered by W_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC3 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1); // V_VOLTAGE
    		legp = 2; legn = 0;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		palClearPad(GPIOB, GPIOB_V_NDTS);
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 4) { // sample W_VOLTAGE, triggered by V_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC2 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2); // W_VOLTAGE
    		legp = 1; legn = 0;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		palClearPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 5) { // sample U_VOLTAGE, triggered by V_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC2 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		legp = 1; legn = 2;
    		palClearPad(GPIOB, GPIOB_U_NDTS);
    		palSetPad(GPIOB, GPIOB_V_NDTS);
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	} else if (angle == 6) { // sample V_VOLTAGE, triggered by U_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1); // V_VOLTAGE
    		legp = 0; legn = 2;
    		palSetPad(GPIOB, GPIOB_U_NDTS);
    		palClearPad(GPIOB, GPIOB_V_NDTS);
    		palSetPad(GPIOB, GPIOB_W_NDTS);
    	}

    	//adcStart(&ADCD1, &adc_commutate_group);
    	int i,x;
    	genpwmcfg.channels[legp].mode = PWM_OUTPUT_ACTIVE_HIGH;
    	if (m->pwm_mode == PWM_MODE_ANTIPHASE)
    		genpwmcfg.channels[legn].mode = PWM_OUTPUT_ACTIVE_LOW;
    	else
    		genpwmcfg.channels[legn].mode = PWM_OUTPUT_ACTIVE_HIGH;

    	if (m->state == OBLDC_STATE_RUNNING) {
    		k_cb_commutate = 0;
    		//BEGIN TEST
    		/* Test configuration: sample the PWM on the active leg
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; // ADC_CR2: use ext event | select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		genpwmcfg.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;*/
    		//END TEST
    		adcStartConversion(&ADCD1, &adc_commutate_group, commutatesamples, ADC_COMMUTATE_BUF_DEPTH);

    		pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    		//pwmEnableChannel(&PWMD1, 0, t_on); // TEST
    		if (m->pwm_mode == PWM_MODE_ANTIPHASE) {
    			pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    			pwmEnableChannel(&PWMD1, legp, t_on);
    			pwmEnableChannel(&PWMD1, legn, t_on);
    		} // PWM_MODE_SINGLEPHASE not supported in STATE_RUNNING
    		//ADC1->CR2 = ADC1->CR2 | ADC_CR2_SWSTART;  // Software trigger ADC conversion (NOT WORKING YET)
    	} else if (m->state == OBLDC_STATE_STARTING) {
    		//inv_duty_cycle = 10000-duty_cycle;
    		if (m->pwm_mode == PWM_MODE_ANTIPHASE) {
    			pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    			pwmEnableChannel(&PWMD1, legp, t_on);
    			pwmEnableChannel(&PWMD1, legn, t_on);
    		} else if (m->pwm_mode == PWM_MODE_SINGLEPHASE) {
    			pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    			pwmEnableChannel(&PWMD1, legp, t_on);
    		}
    		//pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
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
    //pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
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
