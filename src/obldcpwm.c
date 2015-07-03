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

inline int mod(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

extern uint8_t debugbyte;

motor_s motor;	// Stores all motor data
motor_cmd_s motor_cmd, motor_cmd_temp;

//#define ADC_COMMUTATE_NUM_CHANNELS  5
//#define ADC_COMMUTATE_BUF_DEPTH     8
//f_single =    2.0000e+05
//T_cb_ADC1 =    2.0000e-05
//f_cb_ADC1 =    5.0000e+04

#define ADC_COMMUTATE_NUM_CHANNELS 1
#define ADC_COMMUTATE_BUF_DEPTH     50
#define NREG 8 // Number of samples for a valid regression
#define DROPNOISYSAMPLES 1 // "Drop samples with switching noise

typedef struct {
  int16_t size;
  int16_t start;
  int16_t end;
  int16_t elems[NREG+1];
} commutate_Buffer;

commutate_Buffer ibuf, ubuf, ybuf;


static adcsample_t commutatesamples[ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH];

/*
 * Ganzzahlige Teiler von ADC_COMMUTATE_FREQUENCY=1e6:
 * 25, 20, 16, 10, 8, 5
 */

#define PWM_CLOCK_FREQUENCY			28e6 //2e6 //14e6 	// [Hz]
#define PWM_DEFAULT_FREQUENCY		100000 // [40e3, 50e3, 62500, 100e3]	choose one of these base frequencies [Hz]
#define PWM_MINIMUM_FREQUENCY		40000

#define ADC_COMMUTATE_FREQUENCY		1e6		// [Hz]
#define ADC_PWM_DIVIDER				(PWM_CLOCK_FREQUENCY / ADC_COMMUTATE_FREQUENCY)
#define ADC_PWM_PERIOD 				(ADC_COMMUTATE_FREQUENCY / PWM_DEFAULT_FREQUENCY)
#define PWM_MAXIMUM_PERIOD			(ADC_PWM_DIVIDER * ADC_COMMUTATE_FREQUENCY / PWM_MINIMUM_FREQUENCY)

#define ADC_VBAT_CURRENT_NUM_CHANNELS 3
#define ADC_VBAT_CURRENT_BUF_DEPTH 4
#define ADC_VBAT_CURRENT_EXTTRIG_NUM_CHANNELS 2
#define ADC_VBAT_CURRENT_EXTTRIG_BUF_DEPTH 12


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
	motor->u_dc				= 0;
	motor->i_dc				= 0;
	motor->i_dc_ref			= 0;
	motor->u_dc_filt		= 0;
	motor->i_dc_filt		= 0;
	motor->i_dc_sum			= 0;
	motor->last_angle		= 0;
	motor->angle			= 0;
	motor->angle_sum		= 0;
	motor->positioncontrol	= 0;
	motor->P_position		= 70; // P-gain of position controller
	motor->dir				= 0;
	motor->dirjustchanged	= 0;
	motor->dir_v_range		= OBLDC_DIR_V_RANGE;
	motor->time				= 0;
	motor->time_zc			= 0;
	motor->time_last_zc		= 0;
	motor->time_next_commutate_cb = 0;
	motor->delta_t_zc		= 0xFFFF;
	motor->last_delta_t_zc	= 0xFFFF;
	motor->inject = 0;
	motor->last_angle4 = 0;
	motor_cmd.duty_cycle 	= 0; //1000;
	motor_cmd.dir			= 1;
	motor_cmd.newcmd		= 0;

	bufferInitStatic(ubuf, 6);
	bufferInitStatic(ibuf, 6);
	//motor->sumx=0; motor->sumx2=0; motor->sumxy=0; motor->sumy=0; motor->sumy2=0;
	/*table_angle2leg[0]=0; table_angle2leg2[0]=0; // 0,  0,  0,  0   SenseBridgeSign
	table_angle2leg[1]=0; table_angle2leg2[0]=1; // 1,  1, -1,  0		-1
	table_angle2leg[2]=2; table_angle2leg2[0]=1; // 2,  0, -1,  1		1
	table_angle2leg[3]=2; table_angle2leg2[0]=0; // 3, -1,  0,  1		-1
	table_angle2leg[4]=1; table_angle2leg2[0]=0; // 4, -1,  1,  0		1
	table_angle2leg[5]=1; table_angle2leg2[0]=2; // 5,  0,  1, -1		-1
	table_angle2leg[6]=0; table_angle2leg2[0]=2; // 6,  1,  0, -1		1*/
}

inline void increment_angle(void) {
	//motor.angle = ((motor.angle - 1 + motor.dir) % 6) + 1; WARUM GEHT DAS NICHT!? Weil Modulo nicht Rest der ganzzahligen Division ist...
	if(motor.dir == 1) {
		motor.angle = (motor.angle) % 6 + 1;
	} else if(motor.dir == -1) {
		motor.angle = (motor.angle + 4) % 6 + 1;
	}
}

inline void count_angle4control(void) {
	int16_t r = (motor.angle - motor.last_angle) % 6;
	if(r > 3) { // wird groeszer
		motor.angle_sum += r - 6;
		//motor.angle_sum -= 1;
	} else if(r < -2) { // wird kleiner
		motor.angle_sum += 6 + r;
		//motor.angle_sum += 1;
	} else {
		motor.angle_sum += r; // Quick'n dirty
	}
	motor.last_angle = motor.angle;
	//uint8_t debugbyte=motor.angle; uartStartSendI(&UARTD1, 1, &debugbyte );// TODO Debug!
	// TODO: motor.angle auf serielle schnittstelle zum debuggen schreiben - sieht gut aus.
}

void motor_set_cmd(motor_s* m, motor_cmd_s* cmd) {
	int d_percent;
	if(cmd->dir == m->dir) {
		m->pwm_d = cmd->duty_cycle;
	} else {
		m->pwm_d = -cmd->duty_cycle;
	}
	//m->dir = cmd->dir;
	d_percent = (5000 + m->pwm_d / 2) / 100;
	//TODO: Limitation of duty cycle to prevent over current using motor speed (and resistance)
	if(m->state == OBLDC_STATE_STARTING_SENSE_1) { // Ramp up the motor
		m->pwm_mode = PWM_MODE_SINGLEPHASE;
		m->pwm_period		= PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY; // in ticks
		// No zero crossing occurred yet
		m->time_zc			= 0;
		m->time_last_zc		= 0;
		m->time_next_commutate_cb = 0;
		m->delta_t_zc		= 0xFFFF;
		m->last_delta_t_zc	= 0xFFFF;
	}
	else {
		m->pwm_mode = PWM_MODE_ANTIPHASE;
	}
	if(m->pwm_mode == PWM_MODE_SINGLEPHASE)
		m->pwm_t_on = m->pwm_period * m->pwm_d / 10000;
	else {//PWM_MODE_ANTIPHASE
		eval_vbat_idc(); // Evaluate from last cycle. New external triggered!
		if(d_percent > 0 && d_percent < 100)// Adaptive PWM period. TODO: Test this!
			m->pwm_period = ADC_PWM_DIVIDER * (int)((ADC_PWM_PERIOD * 2500) / ((100 - d_percent) * d_percent));
		else
			m->pwm_period = PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY;
		if(m->pwm_period > PWM_MAXIMUM_PERIOD)
			m->pwm_period = PWM_MAXIMUM_PERIOD;
		m->pwm_t_on = m->pwm_period * (5000 + m->pwm_d / 2) / 10000;
	}
	m->pwm_t_on_ADC = m->pwm_t_on / ADC_PWM_DIVIDER;
	m->pwm_period_ADC = m->pwm_period / ADC_PWM_DIVIDER;

	if(m->state_reluct == 3 && m->state != OBLDC_STATE_SENSE_INJECT) {//
		m->state_reluct = 0; // Unknown
	}
	//m->sumx=0; m->sumx2=0; m->sumxy=0; m->sumy=0; m->sumy2=0;
	m->sumy=0;

	if(m->dir == -1)
		m->invSenseSign = (m->angle + 1) % 2;
	else
		m->invSenseSign = m->angle % 2;

	//get_vbat_sample(); // not external triggered
	bufferInitStatic(ybuf, NREG);
}

// TODO: void motor_set_period(motor_s* m, int period)
// TODO: void motor_set_pwm_mode(motor_s* m, obldc_pwm_mode pwm_mode)

uint32_t k_cb_commutate; // Counts how often the ADC callback was called
inline void reset_adc_commutate_count(void) {
	k_cb_commutate = 0;
}


static const ADCConversionGroup adc_vbat_current_group = {
		FALSE, // linear mode
		ADC_VBAT_CURRENT_NUM_CHANNELS,
		NULL, // no callback and end of conversion
		NULL,
		0, // ADC_CR1
		0, // ADC_CR2
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_VBAT_CURRENT_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN5)   // ADC_SQR3
};

static ADCConversionGroup adc_vbat_current_exttrig_group = {
		FALSE, // linear mode
		ADC_VBAT_CURRENT_EXTTRIG_NUM_CHANNELS,
		NULL, // no callback and end of conversion
		NULL,
		0, // ADC_CR1
		0, // ADC_CR2
		0, // ADC_SMPR1
		ADC_SMPR2_SMP_AN3(ADC_SAMPLE_1P5) | ADC_SMPR2_SMP_AN4(ADC_SAMPLE_1P5), // ADC_SMPR2
		ADC_SQR1_NUM_CH(ADC_VBAT_CURRENT_EXTTRIG_NUM_CHANNELS), // ADC_SQR1
		0, // ADC_SQR2
		ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN4)   // ADC_SQR3
};

static PWMConfig genpwmcfg= {
		PWM_CLOCK_FREQUENCY, /* PWM clock frequency */
		PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY, /* PWM period */
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



inline int64_t motortime_now(void) {
	return (motor.time + gptGetCounterX(&GPTD4));
}

static inline void motortime_zc(int64_t t) {
	motor.time_last_zc = motor.time_zc;
	motor.time_zc = t;//motortime_now();
	motor.last_delta_t_zc = motor.delta_t_zc;
	motor.delta_t_zc = motor.time_zc - motor.time_last_zc; // TODO: state machine dass kein ueberlauf auftreten kann
}


static void commutatetimercb(GPTDriver *gptp) {
  msg_t msg;

  (void)gptp;
  //chSysLockFromISR();
  adcStopConversionI(&ADCD1);
  if(motor.state == OBLDC_STATE_RUNNING_SLOW || motor.state == OBLDC_STATE_RUNNING) {
	  motor_cmd.newcmd = 0;
	  increment_angle();
	  //BEGIN Position controller
	  if(motor.positioncontrol) {
		  count_angle4control();
		  motor_cmd.duty_cycle = motor.P_position * (motor_cmd.angle - motor.angle_sum); // P-position-controller
		  if(motor_cmd.duty_cycle < 0) {
			  motor_cmd.duty_cycle = -motor_cmd.duty_cycle;
			  motor_cmd.dir = -1;
		  } else {
			  motor_cmd.dir = 1;
		  }

		  if(motor_cmd.duty_cycle > 1000) {
			  motor_cmd.duty_cycle = 1000;
		  }
	  }
	  //END Position controller
	  motor_set_cmd(&motor, &motor_cmd);
	  set_bldc_pwm(&motor);
	  //pwmStop(&PWMD1);
	  //palTogglePad(GPIOB, GPIOB_LEDR);
  }
  else if(motor.state == OBLDC_STATE_SENSE_INJECT) {
	  if(motor.state_reluct == 3) {
		  motor.angle = (motor.angle + 1) % 6 + 1;
	  } else {
		  if(motor_cmd.dir == -1) {
			  motor.angle = (motor.angle + 1) % 6 + 1;
		  } else {
			  motor.angle = (motor.angle + 1) % 6 + 1;
		  }
		  //increment_angle(); increment_angle(); increment_angle(); increment_angle();
	  }
	  //increment_angle();increment_angle(); // Beide richtugen
	  motor_cmd_temp.duty_cycle = 0; motor_cmd_temp.dir = motor_cmd.dir;
	  motor_set_cmd(&motor, &motor_cmd_temp);
	  //motor_set_cmd(&motor, &motor_cmd);
	  set_bldc_pwm(&motor);
  }
  //chSysUnlockFromISR();
}


static void gpttimercb(GPTDriver *gptp) {
	  msg_t msg;

	  (void)gptp;
  gptcnt_t gpt_time_now;
  int64_t time2fire;
  chSysLockFromISR();
  //palTogglePad(GPIOB, GPIOB_LEDR);
  motor.time += TIMER_CB_PERIOD;
  time2fire = motor.time_next_commutate_cb - motor.time;
  if(time2fire < 100) time2fire=100;
  if(motor.time < motor.time_next_commutate_cb && (gptcnt_t)time2fire < TIMER_CB_PERIOD) {
	  // Schedule next commutatetimercb
	  //gptStartOneShotI(&GPTD4, (gptcnt_t)time2fire);
  }
  chSysUnlockFromISR();
}

static const GPTConfig gptcommutatecfg = {
  1000000,  /* 1MHz timer clock.*/
  commutatetimercb,   /* Timer callback.*/
  0,
  0
};

static const GPTConfig gptcfg3 = {
  1000000,  /* 1MHz timer clock.*/
  gpttimercb,   /* Timer callback.*/
  0,
  0
};

static inline void schedule_commutate_cb(gptcnt_t time2fire) {
		motor.time_next_commutate_cb = motor.time_zc + time2fire;//motor.time_next_commutate_cb = t;
		//if(time2fire < TIMER_CB_PERIOD) {
			// Schedule next commutatetimercb
			gptStartOneShotI(&GPTD3, time2fire);
			//adcStartConversionI(&ADCD1, &adc_vbat_current_group, vbat_current_samples, ADC_VBAT_CURRENT_BUF_DEPTH); // Now, not triggered
			// TODO: call this only when running whithout injection
			if(motor.state == OBLDC_STATE_RUNNING) {
				adcStartConversionI(&ADCD1, &adc_vbat_current_exttrig_group, commutatesamples, ADC_VBAT_CURRENT_EXTTRIG_BUF_DEPTH); // triggered
			}
		//}
	//} // TODO: Else something went terribly wrong. Stop!
}

void motor_start_timer(void) {
	motor.time				= 0;
	motor.time_zc			= 0;
	motor.time_last_zc		= 0;
	motor.time_next_commutate_cb = 0;
	//gptStart(&GPTD4, &gptcfg3);
	//gptStartContinuous(&GPTD4, TIMER_CB_PERIOD);
	//pwmEnableChannel(&PWMD3, 0, 1000); // to start the counter
	gptStart(&GPTD3, &gptcommutatecfg);
	//gptStartOneShot(&GPTD4, 40000);
}

/*
 * ADC streaming callback.
 */

uint16_t yreg[(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2 + 1]; //[NREG+1]
uint16_t* csamples;
//uint16_t xreg[(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2 + 1]; //[NREG+1]

/*
 * The rotor (magnets) have a permeability which causes an emf in the open phase.
 * The emf is zero when the rotor angle is where either no torque (e.g. 0°) is produced or the torque is maximum (e.g. at 90°).
 * The current ripple (determined by observing the power consumption of the ESC) is minimal at maximum torque.
 * At a battery voltage of 11.5V the maximal V-pp is about 3.0V
 */

void decode_inject_pattern(void) {
	uint8_t u,v,w;// Is NOT u,v,w phases of the motor
	if(motor.dir >= 0) {
		u = motor.sense_inject_pattern[0]; v = motor.sense_inject_pattern[1]; w = motor.sense_inject_pattern[2];
	} else {// Injection sequence was called in reversed order
		//u = motor.sense_inject_pattern[1]; v = motor.sense_inject_pattern[2]; w = motor.sense_inject_pattern[0];
		u = motor.sense_inject_pattern[0]; v = motor.sense_inject_pattern[1]; w = motor.sense_inject_pattern[2];
		//u = motor.sense_inject_pattern[0]; v = motor.sense_inject_pattern[2]; w = motor.sense_inject_pattern[1];
	}
	if(u == 2) {
		if(v == 1) {
			if(w == 3) motor.angle4 = 1; else motor.angle4 = 0; // D
		} else if(v == 3) {
			if(w == 1) motor.angle4 = 7; else motor.angle4 = 0; // Q
		}
	} else if(v == 2) {
		if(u == 1) {
			if(w == 3) motor.angle4 = 3; else motor.angle4 = 0; // Q
		} else if(u == 3) {
			if(w == 1) motor.angle4 = 9; else motor.angle4 = 0;// D
		}
	} else if(w == 2) {
		if(v == 1) {
			if(u == 3) motor.angle4 = 11; else motor.angle4 = 0;// Q
		} else if(v == 3) {
			if(u == 1) motor.angle4 = 5; else motor.angle4 = 0; // D
		}
	} else if(u == 1) {
		if(v == 1) {
			if(w == 3) motor.angle4 = 2; else motor.angle4 = 0;
		} else if(v == 3) {
			if(w == 1) {
				motor.angle4 = 6;
			} else if(w == 3) {
				motor.angle4 = 4;
			} else motor.angle4 = 0;
		}
	} else if(u == 3) {
		if(v == 3) {
			if(w == 1) motor.angle4 = 8; else motor.angle4 = 0;
		} else if(v == 1) {
			if(w == 1) {
				motor.angle4 = 10;
			} else if(w == 3) {
				motor.angle4 = 12;
			} else motor.angle4 = 0;
		}
	}
	/*
	 * When motor.dir == -1 the results of sense_inject_pattern are inverted, i.e. 1 is 3 and 3 is 1.
	 * To get same motor.angle4 in both directions, it must be shifted by 90 deg.
	 */
	if(motor.dir < 0) motor.angle4 = (motor.angle4 + 5) % 12 + 1;
}

int16_t y_on, y_off, sample_cnt_t_on, sample_cnt_t_off, x_old, y_old;
/*
 * adc_commutate_inject_cb implements a 2-per-rev position detection method based on PWM injection and inductance measurement.
 */
static void adc_commutate_inject_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  csamples = yreg;
  int i;
  uint32_t k_sample, k_zc; // Sample in the present commutation cycle
  uint16_t k_pwm_period;//Indicates if the pwm sample occurred at pwm-on

  chSysLockFromISR();
  sample_cnt_t_on = 0; sample_cnt_t_off = 0; y_on = 0; y_off = 0;

  k_sample = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  if(k_cb_commutate > 1) {
	  adcStopConversionI(&ADCD1);
	  //pwmStop(&PWMD1);
	  for (i=0; i<(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2; i++ ) {// halbe puffertiefe
		  k_pwm_period = k_sample % motor.pwm_period_ADC;
		  if ( k_pwm_period > DROPNOISYSAMPLES && k_pwm_period < motor.pwm_t_on_ADC) { // Samples during t_on!!!
			  sample_cnt_t_on++;
			  y_on += buffer[i];
		  }
		  else if (k_pwm_period > motor.pwm_t_on_ADC + 1 + DROPNOISYSAMPLES && k_pwm_period < motor.pwm_period_ADC)  {// Samples during t_off
			  sample_cnt_t_off++;
			  y_off += buffer[i];  // Sensebridgesign
			  }
		  k_sample++;
		  //csamples[i] = buffer[i]; // For debugging
	  }

	  if (motor.invSenseSign) {
		  y_on = -(y_on / sample_cnt_t_on - motor.u_dc);
		  y_off = -(y_off / sample_cnt_t_off - motor.u_dc);
	  }
	  else {
		  y_on = y_on / sample_cnt_t_on - motor.u_dc;
		  y_off = y_off / sample_cnt_t_off - motor.u_dc;
	  }

	  if(y_on + motor.dir_v_range < y_off) {
		  motor.sense_inject_pattern[motor.state_inject] = 3;
	  }
	  else if(y_on > y_off + motor.dir_v_range) {
		  motor.sense_inject_pattern[motor.state_inject] = 1;
	  }
	  else {
		  motor.sense_inject_pattern[motor.state_inject] = 2;
	  }
	  motor.state_inject++;

	  if(motor.state_inject < 3) {
		  if(motor.state_inject == 1) {// reset last angle because the actual angle is unknown
			  motor.last_angle4 = 0;
		  }
		  gptStartOneShotI(&GPTD3, 8);
	  }
	  else {
		  decode_inject_pattern();
		  //increment_angle(); increment_angle();// Beide richtungen?
		  motor.angle = (motor.angle + 1) % 6 + 1; // restore initial rotor position. Nur für motor.dir == 1 !!!
		  //motor.angle=1;
		  motor.angle4 = ((motor.angle4 - 1) + (motor.angle - 1) * 4) % 12 + 1; // correction of result of decode_inject_pattern
		  if(motor.angle4 != 0) {// Winkel ist gültig; 50% chance dass das klappt
			  if(motor.state_ramp == 0) { // Injection was called for the first time and the result may be wrong
				  // The correct equation is motor.angle = ((motor.angle4 + 5) / 4) % 3 + 2; but motor.angle is incremented by 1 in schedule_commutate_cb
				  //motor.angle = ((motor.angle4 + 5) / 4) % 3 + 1; // Nur für motor.dir == 1 !!!
				  //motor.angle = ((motor.angle4 + 5) / 4 - 1) % 3 + 1; increment_angle(); // Worked!
				  if(motor.dir == 1) { // fine tuning...
					  motor.angle = ((motor.angle4 + 6) / 4 - 1) % 3 + 1; increment_angle();
				  } else {
					  motor.angle = ((motor.angle4 + 4) / 4 - 1) % 3 + 1; increment_angle();
				  }
				  motor.state = OBLDC_STATE_RUNNING_SLOW;
				  gptStartOneShotI(&GPTD3, 8);
			  } else if(motor.state_ramp <= 1){// Injection was called for the second time and we can check weather the first guess was good or bad
				  if( (motor.angle4 - 1) % 4 != 0 ) { // good, angle is at Q-axis
					  //motor.state = OBLDC_STATE_RUNNING_SLOW;
					  //schedule_commutate_cb(5);
					  //gptStartOneShotI(&GPTD3, 8);
					  //TODO motor.dir = sign(pwm_duty_Cycle)
					  //motor.dir = 1;
				  } else { // bad, angle is at D-axis
					  //motor.angle = (motor.angle) % 6 + 1; //Nur für motor.dir == 1 !!!
					  increment_angle();//R
				  }
				  motor.state = OBLDC_STATE_RUNNING_SLOW;
				  //schedule_commutate_cb(5);
				  gptStartOneShotI(&GPTD3, 8);
			  } else { // motor.state_ramp > 1 : Injection, do tracking
				  /*
				   * *** Position tracking method ***
				   * Depending on the commanded and actual spinning direction one of the following conditions are triggered
				   * in case the actual direction changes. This is tracked by setting motor.dir to the new spinning direction.
				   * Note: In case the direction is changed while adc_commutate_cb is in the state motor.state_reluct == 1,
				   * the motor probably stops since adc_commutate_inject_cb is only called when motor.state_reluct becomes 2.
				   * This may sometimes occur if motor_cmd.dir != motor.dir
				   */
				  // Calculate true difference between angle4 and angle
				  if(motor.dir == 1) {
					  motor.something = ( (motor.angle4 - 1) - ((motor.angle - 1) % 3) * 4 ) % 12;
					  motor.delta_angle4 = mod(motor.angle4 - motor.last_angle4, 12); // ACHTUNG!
				  } else {
					  motor.something = ( ((motor.angle - 1) % 3) * 4 - (motor.angle4 - 1) ) % 12;
					  motor.delta_angle4 = mod(motor.last_angle4 - motor.angle4, 12); // ACHTUNG!
				  }
				  if(motor.something >= 6) motor.something -= 12;
				  if(motor.delta_angle4 >= 6) motor.delta_angle4 -= 12;

				  //if(motor.delta_angle4 < -8) motor.state = OBLDC_STATE_OFF; // KANN nicht passieren

				  if(motor.state_reluct == 3) {// Injection was called at the end of a commutation cycle
					  // Positive --> negative direction when positive was commanded
					  if(motor_cmd.dir == 1 && motor.something > -4 && motor.dir == 1) {
						  if(motor.something < 0) {
							  motor.dir = -1; motor.dirjustchanged = 1;
						  } else {// May this really occur???
							  //motor.dir = -1; increment_angle();
						  }
					  } // Negative --> positive direction when negative was commanded
					  else if(motor_cmd.dir == -1 && motor.something > -4 && motor.something < 0 && motor.dir == -1) {
						  if(motor.something < 0) {
							  motor.dir = 1; motor.dirjustchanged = 1;
						  }
					  } // Negative --> positive direction when positive was commanded
					  else if(motor_cmd.dir == 1 && motor.something > -4 && motor.something < 0 && motor.dir == -1) {
						  if(motor.something < 0) {
							  increment_angle();increment_angle();
							  motor.dir = 1; motor.dirjustchanged = 1;
						  } else {
							  //increment_angle();increment_angle();increment_angle(); motor.dir = 1;
						  }
					  } // Positive --> negative direction when negative was commanded
					  else if(motor_cmd.dir == -1 && motor.something > -4 && motor.something < 0 && motor.dir == 1) {
						  if(motor.something < 0) {
							  increment_angle();increment_angle();
							  motor.dir = -1; motor.dirjustchanged = 1;
						  } else {
							  //increment_angle();increment_angle();increment_angle();motor.dir = -1;
						  }
					  }
					  else {
						  motor.dirjustchanged = 0;
					  }
				  } else if(motor.state_reluct == 2 && motor.inject == 2) {// Injection was called at a zero crossing during a commutation cycle
					  //motor.inject = 3;
					  if(motor_cmd.dir == 1 && motor.delta_angle4 > 0 && motor.something == 0 && motor.dir == -1 && motor.dirjustchanged == 1) {
						  // Motor is stuck in synchronous position.
						  motor.angle = (motor.angle) % 6 + 1; motor.state_reluct = 3;
						  motor.dirjustchanged = 0;
						  //motor.state = OBLDC_STATE_OFF;
					  } else if(motor_cmd.dir == 1 && motor.delta_angle4 == 0 && motor.something == 0 && motor.dir == 1 && motor.dirjustchanged == 0) {
						  // Motor is stuck in synchronous position.
						  //motor.angle = (motor.angle) % 6 + 1;
						  motor.state_reluct = 3;
						  motor.dirjustchanged = 0;
						  //motor.state = OBLDC_STATE_OFF;
					  } else if(motor_cmd.dir == 1 && motor.delta_angle4 <= 0 && motor.something == 0 && motor.dir == -1) {
						  // Drehrichtng wechseln und kommutierne. motor.something == 0 vielleicht überflüssig!
						  if(1){//motor.dirjustchanged == 0) {
							  increment_angle();
							  motor.dir = 1; motor.state_reluct = 3;
							  motor.dirjustchanged = 1;
						  }
						  //motor.state = OBLDC_STATE_OFF;
					  } else if(motor_cmd.dir == -1 && motor.delta_angle4 > 0 && motor.something == 0 && motor.dir == 1 && motor.dirjustchanged == 1) {
						  // Motor is stuck in synchronous position.
						  motor.angle = (motor.angle + 4) % 6 + 1; motor.state_reluct = 3;
						  motor.dirjustchanged = 0;
						  //motor.state = OBLDC_STATE_OFF;
					  } else if(motor_cmd.dir == -1 && motor.delta_angle4 <= 0 && motor.something == 0 && motor.dir == 1) {
						  // Drehrichtng wechseln und kommutierne. motor.something == 0 vielleicht überflüssig!
						  if(1){//motor.dirjustchanged == 0) {
							  increment_angle();
							  motor.dir = -1; motor.state_reluct = 3;
							  motor.dirjustchanged = 1;
						  }
						  //motor.state = OBLDC_STATE_OFF;
					  } else {
						  motor.dirjustchanged = 0;
					  }
					  // Injection beim 0-durchgang; bei callback winkel nicht erhöhen
					  increment_angle();increment_angle();increment_angle();increment_angle();increment_angle();
				  }

				  /*if( (motor.angle4 - 1) % 4 == 0 ) {
					  motor.angle = (motor.angle + 5) % 6 + 1;
				  }*/
				  motor.last_angle4 = motor.angle4;

				  if(motor.state != OBLDC_STATE_OFF) {
					  motor.state = OBLDC_STATE_RUNNING_SLOW;
					  gptStartOneShotI(&GPTD3, 8);
				  }
			  }
		  } else { // decode_inject_pattern returned motor.angle4 == 0 --> Invalid angle!!!
			  /*
			   * Plenty of invalid angles are read when motor_cmd.dir == 1 while everything is fine when motor_cmd.dir == -1
			   * TODO Investigate whats wrong there...
			   */
			  //motor.state = OBLDC_STATE_STARTING_SENSE_1;
			  //motor.state = OBLDC_STATE_OFF;
			  if(motor.state_reluct == 3) {
			  } else if(motor.state_reluct == 2 && motor.inject == 2) {
				  increment_angle();increment_angle();increment_angle();increment_angle();increment_angle();
			  }
			  /*if(motor.dir == 1) { // guess that the motor keeps spinning
				  motor.last_angle4 = (motor.last_angle4 % 12) + 1;
			  } else {
				  motor.last_angle4 = ((motor.last_angle4 + 10)% 12) + 1;
			  }*/
			  motor.state = OBLDC_STATE_RUNNING_SLOW;
			  gptStartOneShotI(&GPTD3, 8);
		  }
	  }
  }//if(k_cb_commutate > 1)

  k_cb_commutate++; // k_cb_ADC++; PUT BREAKPOINT HERE
  chSysUnlockFromISR();
}


static void adc_commutate_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;
  csamples = yreg;
  int i;
  uint32_t k_sample, k_zc; // Sample in the present commutation cycle
  uint16_t k_pwm_period;//Indicates if the pwm sample occurred at pwm-on

  chSysLockFromISR();
  sample_cnt_t_on = 0; sample_cnt_t_off = 0; y_on = 0; y_off = 0;

  k_sample = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  if(k_cb_commutate > 1) { // evaluate only if k_pwm_period > DROPSTARTCOMMUTATIONSAMPLES to allow current at sensed phase to become zero
  for (i=0; i<(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2; i++ ) {// halbe puffertiefe
	  k_pwm_period = k_sample % motor.pwm_period_ADC;
	  if ( k_pwm_period > DROPNOISYSAMPLES && k_pwm_period < motor.pwm_t_on_ADC) { // Samples during t_on!!!
		  sample_cnt_t_on++;
		  y_on += buffer[i];
	  }
	  else if (k_pwm_period > motor.pwm_t_on_ADC + 1 + DROPNOISYSAMPLES && k_pwm_period < motor.pwm_period_ADC)  {// Samples during t_off
		  sample_cnt_t_off++;
    	  y_off += buffer[i];  // Sensebridgesign
	  }
	  k_sample++;
	  //csamples[i] = buffer[i]; // For debugging
  }

  if (motor.invSenseSign) {
	  //y_on = -(buffer[i] - motor.u_dc);  // Sensebridgesign
	  y_on = -(y_on / sample_cnt_t_on - motor.u_dc);
	  y_off = -(y_off / sample_cnt_t_off - motor.u_dc);
  }
  else {
	  //y_on = buffer[i] - motor.u_dc;
	  y_on = y_on / sample_cnt_t_on - motor.u_dc;
	  y_off = y_off / sample_cnt_t_off - motor.u_dc;
  }
  /*
   * Low-Speed-Sensorless-Commutation-Method:
   * 1. Voltage is applied to the motor; i.e. the motor is in synchronous position
   * 2. The angle is incremented by 2, e.g. from 1 to 3
   * 3. The following states will be detected sequentially:
   * 	0.: y_on > y_off + margin; motor is before maximum torque position --> go to state 1
   * 	1.: y_on and y_off are within margin; Motor is at maximum torque position --> go to state 2
   * 	2.: y_on+margin < y_off; motor has passed the maximum torque position
   * 4. Now immediately increment the angle by 1 and repeat...
   */
  if(y_on + motor.dir_v_range < y_off) {// Detect zero crossing here
	  if(motor.state_reluct == 2) {
		  motor.state_reluct = 3;
		  adcStopConversionI(&ADCD1); // OK, commutate!
		  if(motor.inject > 1) { //if(motor.state_ramp < 2) {
			  if(motor.state_ramp < 2) motor.state_ramp++;
			  //trigger injection sequence at the two other phases to check if angle4 refers to a D- or a Q-Axis position
			  motor.state = OBLDC_STATE_SENSE_INJECT;
			  motor.sense_inject_pattern[0] = 3; // Keep actually measured value // Keep zero crossing at the the actual phase
			  motor.state_inject = 1; // Skip actual phase
		  }
		  schedule_commutate_cb(50);
		  // Problem: delta_t ist zu groß: prüfe mit Oszi!
		  motor.time_next_commutate_cb += k_sample - k_zc;// set correct time: add time from zero crossing to now
	  }
  } else if( (y_on > y_off + motor.dir_v_range + 200) && motor.state_reluct == 2 && motor.inject == 2 && motor.state_ramp >= 2 && motor.dirjustchanged == 0) {
	  /*
	   *  Becomes true in case the direction is reversed when zero crossing already occurred. If the motor continues in the new direction,
	   *  the next zero crossing will occur in a d-axis position and the
	   */
	  motor.state_reluct = 1;
	  motor.persist_in_state_recluct_2_count = 0;
	  adcStopConversionI(&ADCD1); //motor.state = OBLDC_STATE_OFF;
	  if(motor.dir == 1) motor.dir = -1; else motor.dir = 1;
	  increment_angle(); increment_angle(); increment_angle();
	  motor.state_reluct = 3; motor.dirjustchanged = 1;
	  schedule_commutate_cb(50);
	  // TODO: TEST if this actually works!!!!!!!!!!!!
  } else if(y_on > y_off + motor.dir_v_range) {
	  if(motor.state_reluct == 0) {
		  motor.state_reluct = 1;
	  }
  } else if (motor.persist_in_state_recluct_2_count > 1600) { // Too long in state_reluct==2 --> re-trigger injection
	  motor.persist_in_state_recluct_2_count = 0; motor.state_reluct = 1;
	  //adcStopConversionI(&ADCD1); motor.state = OBLDC_STATE_OFF; // Use this line to see if this is actually triggered
  } else {// Found zero crossing
	  if(motor.state_reluct == 1) { // Zero crossing happened HERE!
	  // if(motor.state_reluct == 1 && y_on < y_off) { // Zero crossing happened HERE! --> Threshold removed because it is sometimes not crossed when the motor is at standstill
		  motor.persist_in_state_recluct_2_count = 0;
		  motor.state_reluct = 2;
		  //motor.u_dc2 = (y_on + y_off) / 2;
		  k_zc = k_sample;
		  motortime_zc(motor.time_next_commutate_cb + k_sample); // Write time of zero crossing
		  //if(motor.inject == 2) {
		  //if(motor.state_ramp < 1 && motor.inject == 2) {
		  if(motor.state_ramp < 1 || motor.inject == 2) {
			  if(motor.state_ramp < 2) {motor.state_ramp++;}//motor.state_ramp++; // Call this sequence only once
			  adcStopConversionI(&ADCD1);
			  //pwmStop(&PWMD1);
			  //trigger injection sequence at the two other phases to check if angle4 refers to a D- or a Q-Axis position
			  motor.state = OBLDC_STATE_SENSE_INJECT;
			  motor.sense_inject_pattern[0] = 2;// Keep zero crossing at the the actual phase
			  motor.state_inject = 1; // Skip actual phase
			  gptStartOneShotI(&GPTD3, 8);
			  /*
			  // Jetzt injection-messung machen:
			  // ...
			  if (...) {
				  //   wenn der winkel Richtig ist, dann alles gut und motor.angle um 1 erhöhen
			  } else {
			  	  //   wenn der winkel falsch ist, dann motor.angle = (motor.angle4 + 5) / 4 + 2;
			  	  motor.angle = motor.angle + 1;
			  }
			  */
		  }
	  } else if (motor.state_reluct == 2 && motor.state_ramp > 1 && motor.inject == 2) {
		  // Count up how many cycles state_reluct==2 is active
		  //motor.persist_in_state_recluct_2_count++;
	  }
  }

  }//if(k_cb_commutate > 1)

  k_cb_commutate++; // k_cb_ADC++; PUT BREAKPOINT HERE
  chSysUnlockFromISR();
}


static void adc_commutate_fast_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void)adcp;

  csamples = yreg;
  int i;
  uint32_t k_sample; // Sample in the present commutation cycle
  uint16_t k_pwm_period;//Indicates if the pwm sample occurred at pwm-on

  int64_t m_reg, b_reg, reg_den, k_zc;

  commutate_Buffer* xbuf_ptr;
  commutate_Buffer* ybuf_ptr;

  chSysLockFromISR();
  sample_cnt_t_on = 0; sample_cnt_t_off = 0; y_on = 0; y_off = 0;
  //u_dc_int = get_vbat_sample();

  ybuf_ptr = &ybuf;
  //k_start = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  //k_end = k_start + (ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2;
  k_sample = (ADC_COMMUTATE_BUF_DEPTH / 2) * k_cb_commutate;
  //motor.sumy = 0; // ENTFERNEN!! Nur zum probieren!
  if(k_cb_commutate > 1) // evaluate only if k_pwm_period > DROPSTARTCOMMUTATIONSAMPLES to allow current at sensed phase to become zero
  for (i=0; i<(ADC_COMMUTATE_NUM_CHANNELS * ADC_COMMUTATE_BUF_DEPTH) / 2; i++ ) {// halbe puffertiefe
	  k_pwm_period = k_sample % motor.pwm_period_ADC;
	  if ( k_pwm_period > DROPNOISYSAMPLES && k_pwm_period < motor.pwm_t_on_ADC) { // Samples during t_on!!!
		  if (isBufferFull(ybuf_ptr)) {
			  bufferRead(ybuf_ptr, y_old);
			  // Decrement obsolete buffer values from sums
		      motor.sumy -= y_old;
		  }
		  if (motor.invSenseSign)
			  y_on = -(buffer[i] - motor.u_dc);  // Sensebridgesign
		  else
			  y_on = buffer[i] - motor.u_dc;
		  //bufferWrite(ybuf_ptr, buffer[i]);
		  bufferWrite(ybuf_ptr, y_on);
	      motor.sumy += y_on;
	  }
	  if( motor.sumy < -50 && isBufferFull(ybuf_ptr)) {// Detect zero crossing here
		  //if(motor.state_reluct == 2 || motor.state_reluct == 1) {
			  motor.state_reluct = 3;
			  //motortime_zc();
			  adcStopConversionI(&ADCD1); // OK, commutate!
			  //pwmStop(&PWMD1);
			  motortime_zc(motor.time_next_commutate_cb + k_sample);

			  y_off=0;
			  //schedule_commutate_cb( (motor.delta_t_zc + motor.last_delta_t_zc) * 2 / 5 );
			  schedule_commutate_cb( (motor.delta_t_zc + motor.last_delta_t_zc) / 4 - 10);
			  return;
	  }
	  k_sample++;
	  //csamples[i] = buffer[i];
  }
  // Check for timeout
  if(k_sample > 10000000) {//(k_sample > 1000000) {  // TIMEOUT
	  k_sample++;
	  adcStopConversionI(&ADCD1); // HERE breakpoint
	  chSysUnlockFromISR();
	  return;
  }

  k_cb_commutate++; // k_cb_ADC++; PUT BREAKPOINT HERE
  chSysUnlockFromISR();
}


static void adc_commutate_err_cb(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
  //adc_commutate_count++;
}


static uint8_t halldecode[8];




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
	init_motor_struct(&motor); // v_bat_current_conversion wird nur aus main aufgerufen TODO: Eigene Funktion machen!
	adcStartConversion(&ADCD1, &adc_vbat_current_group, vbat_current_samples, ADC_VBAT_CURRENT_BUF_DEPTH);
}

adcsample_t* get_vbat_current_samples(void) {
	return vbat_current_samples;
}

adcsample_t get_vbat_sample(void) { // value scaled to be 50% of phase voltage sample
	// /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
	// the voltage divider at v_bat is 1.5 and 10 kOhm
	// So, the transformation is 115*36 / (15*136)
	int i,v_scaled;
	int u_raw=0;
	int i_raw=0;
	int i_raw_ref=0;
	for(i=0; i < ADC_VBAT_CURRENT_NUM_CHANNELS*ADC_VBAT_CURRENT_BUF_DEPTH; i+=ADC_VBAT_CURRENT_NUM_CHANNELS) {
		u_raw += vbat_current_samples[i];
		i_raw += vbat_current_samples[i+1] - vbat_current_samples[i+2];
		i_raw_ref += vbat_current_samples[i+2];
	}
	v_scaled = u_raw / ADC_VBAT_CURRENT_BUF_DEPTH * 4140 / 2040 / 2;
	motor.u_dc = v_scaled; // UGLY!
	motor.i_dc = i_raw / ADC_VBAT_CURRENT_BUF_DEPTH;
	motor.i_dc_ref = i_raw_ref / ADC_VBAT_CURRENT_BUF_DEPTH;
	return (adcsample_t)v_scaled;
}


void eval_vbat_idc(void) { // value scaled to be 50% of phase voltage sample TODO MUSS NOCH GETESTET WERDEN
	// /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
	// the voltage divider at v_bat is 1.5 and 10 kOhm
	// So, the transformation is 115*36 / (15*136)
	int i;
	int u_raw_on=0;
	int u_raw_off=0;
	int i_raw_on=0;
	int i_raw_off=0;
	int16_t i_old;
    commutate_Buffer* ibuf_ptr;
	ibuf_ptr = &ibuf;

	sample_cnt_t_on = 0; sample_cnt_t_off = 0;
	for (i=0; i<(ADC_VBAT_CURRENT_EXTTRIG_NUM_CHANNELS*ADC_VBAT_CURRENT_EXTTRIG_BUF_DEPTH); i+=ADC_VBAT_CURRENT_EXTTRIG_NUM_CHANNELS ) {// halbe puffertiefe
		if ( i > DROPNOISYSAMPLES && i < motor.pwm_t_on_ADC) { // Samples during t_on!!!
			sample_cnt_t_on++;
			u_raw_on += commutatesamples[i];
			i_raw_on += commutatesamples[i+1] - motor.i_dc_ref;
		}
		else if (i > motor.pwm_t_on_ADC + 1 + DROPNOISYSAMPLES && i < motor.pwm_period_ADC)  {// Samples during t_off
		    sample_cnt_t_off++;
			u_raw_off += commutatesamples[i];
			i_raw_off += commutatesamples[i+1] - motor.i_dc_ref;
		}
		  //csamples[i] = buffer[i]; // For debugging
	}
	motor.u_dc = u_raw_on / sample_cnt_t_on * 4140 / 2040 / 2;
	motor.i_dc = i_raw_on / sample_cnt_t_on;

	if (isBufferFull(ibuf_ptr)) {
		bufferRead(ibuf_ptr, i_old);
		// Decrement obsolete buffer values from sums
		motor.i_dc_sum -= i_old;
	}
	bufferWrite(ibuf_ptr, motor.i_dc);
	motor.i_dc_sum += motor.i_dc;
	motor.i_dc_filt = motor.i_dc_sum / 6;
}


/*
 * Generic PWM for BLDC motor operation.
 * duty_cycle in percent * 100
 * Period in microseconds
 */
static const uint8_t pin_leg_enable[3] = {
		GPIOB_U_NDTS, GPIOB_V_NDTS, GPIOB_W_NDTS
};
void set_bldc_pwm(motor_s* m) { // Mache neu mit motor_struct (pointer)
	int angle, t_on, period, inv_duty_cycle;
	uint8_t legp, legn, legoff; // Positive and negative PWM leg
	angle 	= m->angle;
	t_on	= m->pwm_t_on;
	period	= m->pwm_period;

	adcStopConversion(&ADCD1);
	//palClearPad(GPIOB, GPIOB_U_NDTS); palClearPad(GPIOB, GPIOB_V_NDTS); palClearPad(GPIOB, GPIOB_W_NDTS);
	//pwmStop(&PWMD1);

	if (m->state == OBLDC_STATE_RUNNING) {
		adc_commutate_group.end_cb = adc_commutate_fast_cb;
	}
	else if(m->state == OBLDC_STATE_RUNNING_SLOW) {
		adc_commutate_group.end_cb = adc_commutate_cb;
	}
	else if(m->state == OBLDC_STATE_SENSE_INJECT) {
		adc_commutate_group.end_cb = adc_commutate_inject_cb;
	}
	else if (m->state == OBLDC_STATE_OFF || m->state == OBLDC_STATE_CATCHING) { // PWM OFF!
		angle = 0;
	}
	else if(m->state == OBLDC_STATE_STARTING_SENSE_1) {
	}
	else {
		angle = 0;
	}
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
    	pwmStop(&PWMD1);
    } else {
    	if (angle == 1) { // sample W_VOLTAGE, triggered by U_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; // ADC_CR2: select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2); // W_VOLTAGE
    		legp = 0; legn = 1; legoff=2;
    	} else if (angle == 2) { // sample U_VOLTAGE, triggered by W_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC3 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		legp = 2; legn = 1; legoff=0;
    	} else if (angle == 3) { // sample V_VOLTAGE, triggered by W_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC3 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1); // V_VOLTAGE
    		legp = 2; legn = 0; legoff=1;
    	} else if (angle == 4) { // sample W_VOLTAGE, triggered by V_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC2 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN2(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2); // W_VOLTAGE
    		legp = 1; legn = 0; legoff=2;
    	} else if (angle == 5) { // sample U_VOLTAGE, triggered by V_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC2 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		legp = 1; legn = 2; legoff=0;
    	} else if (angle == 6) { // sample V_VOLTAGE, triggered by U_PWM
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; //ADC_CR2: select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN1(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1); // V_VOLTAGE
    		legp = 0; legn = 2; legoff=1;
    	}
    	adc_vbat_current_exttrig_group.cr2 = adc_commutate_group.cr2;

    	//adcStart(&ADCD1, &adc_commutate_group);
    	int i,x;
    	genpwmcfg.channels[legp].mode = PWM_OUTPUT_ACTIVE_HIGH;
    	if (m->pwm_mode == PWM_MODE_ANTIPHASE)
    		genpwmcfg.channels[legn].mode = PWM_OUTPUT_ACTIVE_LOW;
    	else
    		genpwmcfg.channels[legn].mode = PWM_OUTPUT_ACTIVE_HIGH;

    	palClearPad(GPIOB, GPIOB_U_NDTS); palClearPad(GPIOB, GPIOB_V_NDTS); palClearPad(GPIOB, GPIOB_W_NDTS);
    	pwmStop(&PWMD1);
    	if (m->state == OBLDC_STATE_RUNNING || m->state == OBLDC_STATE_RUNNING_SLOW || m->state == OBLDC_STATE_SENSE_INJECT) {
    		k_cb_commutate = 0;
    		genpwmcfg.period = period;
    		//BEGIN TEST
    		/* Test configuration: sample the PWM on the active leg
    		adc_commutate_group.cr2 = ADC_CR2_EXTTRIG | ADC_CR2_CONT; // ADC_CR2: use ext event | select TIM1_CC1 event
    		adc_commutate_group.smpr2 = ADC_SMPR2_SMP_AN0(ADC_SAMPLE_1P5);
    		adc_commutate_group.sqr3 = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0); // U_VOLTAGE
    		genpwmcfg.channels[0].mode = PWM_OUTPUT_ACTIVE_HIGH;*/
    		//END TEST
    		adcStartConversion(&ADCD1, &adc_commutate_group, commutatesamples, ADC_COMMUTATE_BUF_DEPTH);
    		//pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    		//pwmEnableChannel(&PWMD1, 0, t_on); // TEST
    		if (m->pwm_mode == PWM_MODE_ANTIPHASE) {
    			pwmStart(&PWMD1, &genpwmcfg); // PWM signal generation
    			pwmEnableChannel(&PWMD1, legp, t_on);
    			pwmEnableChannel(&PWMD1, legn, t_on);
    		} // PWM_MODE_SINGLEPHASE not supported in STATE_RUNNING
    		//ADC1->CR2 = ADC1->CR2 | ADC_CR2_SWSTART;  // Software trigger ADC conversion (NOT WORKING YET)
    	} else if (m->state == OBLDC_STATE_STARTING_SYNC || m->state == OBLDC_STATE_STARTING_SENSE_1) {
    		genpwmcfg.period = PWM_CLOCK_FREQUENCY / PWM_DEFAULT_FREQUENCY;
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
    	palSetPad(GPIOB, pin_leg_enable[legp]);
    	palSetPad(GPIOB, pin_leg_enable[legn]);
    }
}
