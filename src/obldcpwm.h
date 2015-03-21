/*
 * pwm.h
 *
 *  Created on: 22.05.2014
 *      Author: joerg
 */

#ifndef OBLDCPWM_H_
#define OBLDCPWM_H_

#include "obldcadc.h"

// besser aufgehoben im reglerteil -> simulation


typedef enum {
	OBLDC_STATE_OFF = 0,
	OBLDC_STATE_STARTING_SYNC,
	OBLDC_STATE_STARTING_SENSE_1,
	OBLDC_STATE_CATCHING,
	OBLDC_STATE_SENSE_INJECT,
	//OBLDC_STATE_RUNNING_INJECT,
	OBLDC_STATE_RUNNING_SLOW,
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
	int16_t duty_cycle; // Duty cycle 100%=10000
	int16_t dir;
} motor_cmd_s;

typedef struct {
	obldc_state state;
	obldc_pwm_mode pwm_mode;
	int pwm_d;			// Duty cycle -10000 < d < 10000
	int pwm_t_on;		// in timer clock ticks
	int pwm_period;		// in timer clock ticks
	int pwm_t_on_ADC;	// in ADC clock ticks
	int pwm_period_ADC;
	int angle, angle4;
	int dir;
	uint8_t dirjustchanged;
	int16_t dir_v_range;
	uint8_t sense_inject_pattern[3];
	uint8_t state_inject, state_ramp, noinject;
	int16_t u_dc, u_dc2, u_dc_filt;
	int16_t i_dc, i_dc_ref, i_dc_filt, i_dc_sum;
	uint8_t state_reluct; // 0=unknown
	int64_t time; // Motor time in usec
	int64_t time_zc, time_last_zc;
	int64_t time_next_commutate_cb;
	uint16_t delta_t_zc, last_delta_t_zc;
	int32_t sumy;
	//int64_t sumx, sumx2, sumxy, sumy, sumy2;
	uint8_t invSenseSign;// True when voltage must be inverted
	int16_t something;
} motor_s;

#define OBLDC_DIR_V_RANGE 300		// Range for detection of voltage zero crossing in the inductance measurement
#define TIMER_CB_PERIOD 60000
#define OBLDC_PWM_SWITCH_FREQUENCY_MIN 50000 // lowest switching frequency [Hz]
#define OBLDC_PWM_SWITCH_FREQUENCY_MAX 40000 // highest switching frequency [Hz]
#define OBLDC_PWM_PWM_MODE PWM_MODE_SINGLEPHASE // Default PWM mode
#define OBLDC_PWM_MIN_DUTY_CYCLE 0.02 // Minimum duty cycle
#define OBLDC_PWM_MAX_DUTY_CYCLE 2500 // Maximum duty cycle

#define OBLDC_MIN_CATCH_VOLTAGE_OBSOLETE 360 // 1V minimum voltage to evaluate for motor position catching; /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage TODO: Check max ADC voltage 3V or 3.3V?





void motor_start_timer();
inline int64_t motortime_now();
motor_s* get_motor_ptr(void);
void set_bldc_pwm(motor_s* m);
void motor_set_cmd(motor_s* m, motor_cmd_s* cmd);
void set_bldc_pwm_adc(int angle, int duty_cycle, int period);
void startcatchmodePWM(void);
void init_motor_struct(motor_s* motor);
void decode_inject_pattern(void);
void v_bat_current_conversion(void);
adcsample_t get_vbat_sample(void);
void eval_vbat_idc(void);


#endif /* OBLDCPWM_H_ */
