/*
 * pwm.h
 *
 *  Created on: 22.05.2014
 *      Author: joerg
 */
#include "obldcadc.h"

// besser aufgehoben im reglerteil -> simulation
typedef enum {
	OBLDC_STATE_OFF = 0,
	OBLDC_STATE_STARTING,
	OBLDC_STATE_CATCHING,
	OBLDC_STATE_RUNNING
} obldc_state;

typedef enum {
	PWM_MODE_SINGLEPHASE,	// one phase pulses, one phase low side on, one phase open: DANGEROUS when braking and zero crossings
	PWM_MODE_ANTIPHASE,		// inverted pulses to two phases, one phase open
	PWM_MODE_ONEPULSE		// One pulse per commutation, appropriate for very high motor speeds
} obldc_pwm_mode;

typedef enum {
	FAULT_NONE = 0,
	FAULT_OVER_VOLTAGE,
	FAULT_UNDER_VOLTAGE,
	FAULT_OVER_CURRENT
} obldc_fault_code;


typedef struct {
	obldc_state state;
	obldc_pwm_mode pwm_mode;
	int pwm_t_on;		// in timer clock ticks
	int pwm_period;		// in timer clock ticks
	int pwm_t_on_ADC;	// in ADC clock ticks
	int pwm_period_ADC;
	int angle;
	int direction;
} motor_s;



#define OBLDC_PWM_SWITCH_FREQUENCY_MIN 10000 // lowest switching frequency [Hz]
#define OBLDC_PWM_SWITCH_FREQUENCY_MAX 40000 // highest switching frequency [Hz]
#define OBLDC_PWM_PWM_MODE PWM_MODE_SINGLEPHASE // Default PWM mode
#define OBLDC_PWM_MIN_DUTY_CYCLE 0.02 // Minimum duty cycle
#define OBLDC_PWM_MAX_DUTY_CYCLE 0.95 // Maximum duty cycle

#define OBLDC_MIN_CATCH_VOLTAGE_OBSOLETE 360 // 1V minimum voltage to evaluate for motor position catching; /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage TODO: Check max ADC voltage 3V or 3.3V?


#ifndef OBLDCPWM_H_
#define OBLDCPWM_H_



void mystartPWM(void);
void set_bldc_pwm(motor_s* m);
void motor_set_duty_cycle(motor_s* m, int d);
void set_bldc_pwm_adc(int angle, int duty_cycle, int period);
void startcatchmodePWM(void);
void init_motor_struct(motor_s* motor);

#endif /* OBLDCPWM_H_ */
