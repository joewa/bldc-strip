/*
 * obldc_catchmotor.c
 *
 *  Created on: 25.08.2014
 *      Author: joerg
 */


#include "ch.h"
#include "hal.h"

#include "obldc_def.h"
#include "obldc_catchmotor.h"
#include "obldcadc.h"
#include "obldcpwm.h"

extern motor_s motor; // Motor-struct from obldcpwm.c
extern motor_cmd_s motor_cmd, motor_cmd_temp;// Motor-cmd from obldcpwm.c

static uint8_t halldecode[8];

// Winkel-Test
uint8_t letzte_winkel[10];
int winkelcount = 0;

int last_hall_decoded;
int last_halldiff, last_last_halldiff, last_last_last_halldiff, crossing_counter;
int catchcycle(motor_s* m, int voltage_u, int voltage_v, int voltage_w, uint8_t init) {
	static int vdiff_last[3]; // Neu
	int vdiff[3];
	static int vdiff_1_last;
	static int vdiff_2_last;
	static int vdiff_3_last;
	static int last_zero_crossing;
	int direction = 0;
	static int stopped_count;
	int crossing_detected;
	int hall_1, hall_2, hall_3;
	int hall_code, hall_decoded;
	//static int last_hall_decoded;
	int halldiff, abshalldiff;

	if (init == 1) {
		halldecode[0]=0; halldecode[1]=4; halldecode[2]=2; halldecode[3]=3; halldecode[4]=6; halldecode[5]=5; halldecode[6]=1; halldecode[7]=0;
		vdiff_last[0] = 0; vdiff_last[1] = 0; vdiff_last[2] = 0;
		vdiff_1_last = 0;//obsolet
		vdiff_2_last = 0;
		vdiff_3_last = 0;
		last_zero_crossing = 0;
		direction = 0;
		stopped_count = 0;
		last_hall_decoded = 0;
		last_halldiff = 0; last_last_halldiff = 0; crossing_counter = 0;
	} else {
		// init run variables
		crossing_detected = 0;

		// determine voltage between the phases
		int vdiff_1 = voltage_v - voltage_u;
		int vdiff_2 = voltage_w - voltage_v;
		int vdiff_3 = voltage_u - voltage_w;

		// determine min and max values
		int vdiff_max = MAX(MAX(vdiff_1_last, vdiff_2_last), vdiff_3_last);
		int vdiff_min = MIN(MIN(vdiff_1_last, vdiff_2_last), vdiff_3_last);

		// when difference between min and max values > "MinCatchVoltage" -> Cond 1 fulfilled
		if (vdiff_max - vdiff_min > OBLDC_MIN_CATCH_VOLTAGE) {
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
				// check if distance to last 'angle' is = 1

				halldiff = (hall_decoded - last_hall_decoded) % 6;
				//if ( halldiff >= 3 ) { halldiff -= 6; } // correct difference
				if ( halldiff >= 3 ) { halldiff = 6 - halldiff; } // correct difference

				abshalldiff = halldiff;
				if (abshalldiff < 0) abshalldiff=-abshalldiff; //ABS

				// check for four consecutive zero crossings to be rock safe!
				crossing_counter++;
				if (crossing_counter > 4 && last_last_last_halldiff == last_last_halldiff && last_last_halldiff == last_halldiff && halldiff == last_halldiff && abshalldiff == 1 && last_hall_decoded != 0) {
					// determine direction of rotation
					direction = halldiff;
					crossing_counter = 0;
				}
				last_hall_decoded = hall_decoded;
				last_halldiff = halldiff;
				last_last_halldiff = last_halldiff;
				last_last_last_halldiff = last_last_halldiff;
				if (winkelcount<10) {
				  // Winkel-Test
				  letzte_winkel[winkelcount] = hall_decoded;
				  winkelcount++;
				}
			}


		} else { // Voltage too small
			// count 'elses': if > 10 -> Motor is stopped -> start startup algorithm
			stopped_count = stopped_count + 1;
			if (stopped_count > 100) {
				// start startup algorithm
			}
		}
		vdiff_1_last = vdiff_1;
		vdiff_2_last = vdiff_2;
		vdiff_3_last = vdiff_3;
		// neue adc-messung starten
		//catchconversion();
	}
	motor.dir = direction;
	motor.angle = hall_decoded;
	return direction;
}


/*msg_t myThread(void *arg) {
  unsigned i = 10;
  while (i > 0) {
    palTogglePad(GPIOB, GPIOB_LEDR);
    chThdSleepMilliseconds(500);
    i--;
  }
  return (msg_t)i;
}

thread_t *startMyThread() {
	thread_t *tp = chThdCreateFromHeap(NULL, THD_WA_SIZE(256), NORMALPRIO+1, myThread, NULL); // Heap is BAD
	if (tp == NULL)
	  chSysHalt("Memory exausted");    //Memory exausted.
	return tp;
}*/

#define CATCHMOTOR_STACK_SIZE 128
static THD_WORKING_AREA(waCatchMotorThread, CATCHMOTOR_STACK_SIZE);
static THD_FUNCTION(tCatchMotorTread, arg) {
  (void)arg;
  chRegSetThreadName("CatchMotorThread");
  char sbyte = 77;
  int catchstate = 0; int catchresult = 0;

  catchcycle(&motor, 0, 0, 0, TRUE); // initialize catch state variables
  while (TRUE) {
	  catchconversion(); // start ADC Converter
	  chThdSleepMicroseconds(50);
	  // evaluate last ADC measurement
	  // determine voltage; for efficiency reasons, we calculating with the ADC value and do not convert to a float for [V]
	  int voltage_u = getcatchsamples()[0]; // /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
	  int voltage_v = getcatchsamples()[1]; // /4095.0 * 3 * 13.6/3.6;
	  int voltage_w = getcatchsamples()[2]; // /4095.0 * 3 * 13.6/3.6;
	  catchstate = catchcycle(&motor, voltage_u, voltage_v, voltage_w, FALSE);
	  // Write result TODO
	  if (catchstate != 0) catchresult = catchstate;
  }
  return 0;
}

void startCatchMotorThread(void) {
  chThdCreateStatic(waCatchMotorThread, sizeof(waCatchMotorThread),
		  NORMALPRIO, tCatchMotorTread, NULL);
}


#define RAMPMOTOR_STACK_SIZE 256
static THD_WORKING_AREA(waRampMotorThread, RAMPMOTOR_STACK_SIZE);
static THD_FUNCTION(tRampMotorTread, arg) {
  (void)arg;
  chRegSetThreadName("RampMotorThread");

  //int angle = 1;
  int acceleration = 3;
  int speed = 50;  // initial speed
  int delta_t = 50;
  int64_t temp=0;
  int last_angle4, act_angle;

  int catchstate = 0; int catchresult = 0; int catchcount = 0;
  //init_motor_struct(&motor);
  motor.angle = 4;
  //motor.state = OBLDC_STATE_STARTING_SYNC;
  motor.state = OBLDC_STATE_OFF;
  //motor.pwm_duty_cycle = 0;
  //set_bldc_pwm(&motor);
  while (TRUE) {
	  if(motor.state == OBLDC_STATE_OFF) {
		  adcStopConversion(&ADCD1);
		  pwmStop(&PWMD1);
		  palClearPad(GPIOB, GPIOB_U_NDTS);palClearPad(GPIOB, GPIOB_V_NDTS);palClearPad(GPIOB, GPIOB_W_NDTS);
		  motor.dir = motor_cmd.dir;// Motor stopped, forget direction. TODO remove when tracking works
		  //motor.dir = 0;
		  motor.state_inject = 0;
		  motor.angle = 4;
		  motor.state_ramp = 0;
		  motor_cmd_temp.duty_cycle = 0;
		  chThdSleepMilliseconds(1);
	  }
	  if(motor.state == OBLDC_STATE_STARTING_SENSE_1) { // Ramp up the motor
		  // Rotorlageerkennung
		  chThdSleepMilliseconds(1);
		  motor.state = OBLDC_STATE_SENSE_INJECT;
		  motor.pwm_mode = PWM_MODE_ANTIPHASE;
		  motor.state_inject = 0;
		  motor.angle = 4;
		  motor.state_ramp = 0;
		  motor_cmd_temp.duty_cycle = 0; motor_cmd_temp.dir = 0;
		  motor_set_cmd(&motor, &motor_cmd_temp);
		  set_bldc_pwm(&motor); // Start position detection by inductance measurement
		  //motor.state_ramp = 0;
		  chThdSleepMicroseconds(1000);
		  //motor.state = OBLDC_STATE_STARTING_SENSE_1; // DEBUG position sensing

		  /*
		  // Einfach und funzt!
		  motor.noinject = 1;
		  motor.pwm_mode = PWM_MODE_SINGLEPHASE;
		  motor.angle = 3;
		  motor_set_duty_cycle(&motor, 600);// ACHTUNG!!! 1000 geht gerade noch
		  set_bldc_pwm(&motor); // Position the motor to sync angle
		  chThdSleepMilliseconds(1000);
		  catchcount = 0;
		  motor.state = OBLDC_STATE_RUNNING_SLOW;
		  motor.pwm_mode = PWM_MODE_ANTIPHASE;
		  motor.angle = 1;
		  motor_set_duty_cycle(&motor, 600);// ACHTUNG!!! 1000 geht gerade noch
		  set_bldc_pwm(&motor); // Start running with back EMF detection
		  */
	  }
	  if(motor.state == OBLDC_STATE_SENSE_INJECT) {
		  chThdSleepMicroseconds(500);
	  }
	  if(motor.state == OBLDC_STATE_RUNNING_SLOW) { // Ramp up the motor
		  catchcount++;
		  if(motor.time_zc != temp) {
			  catchcount = 0; temp = motor.time_zc;
		  }
		  /*if(catchcount > 15 && motor.state_reluct == 2) {
			  motor.inject = 2;
			  motor.state_reluct = 1;
		  } else */if(catchcount > 7) {
			  motor.inject = 2;//motor.inject = 3;
		  }

		  if(catchcount > 1000) { // Timeout!
			  pwmStop(&PWMD1);
			  motor.dir = motor_cmd.dir;// Motor stopped, forget direction. TODO remove when tracking works
			  //motor.dir = 0;
			  adcStopConversion(&ADCD1);
			  catchcount = 0;
			  motor.delta_t_zc		= 0xFFFF;
			  motor.last_delta_t_zc	= 0xFFFF;
			  motor.state = OBLDC_STATE_STARTING_SENSE_1;
		  }
		  if(motor.delta_t_zc < 3000 && motor.last_delta_t_zc < 3000) { // TODO: Make definitions for these values
			  motor.inject = 0;
		  }
		  if(motor.delta_t_zc < 1100 && motor.last_delta_t_zc < 1100) { // TODO: Make definitions for these values
			  catchcount=0;
			  motor.state = OBLDC_STATE_RUNNING;
		  }
		  chThdSleepMicroseconds(500);
	  }
	  if(motor.state == OBLDC_STATE_RUNNING) { // Motor is fast!
		  //catchcount++;
		  if( (motor.delta_t_zc > 1200 && motor.last_delta_t_zc > 1200) /*|| (motortime_now() - motor.time_zc > 3500)*/) { // motor tooo slow!
			  motor.delta_t_zc		= 0xFFFF;
			  motor.last_delta_t_zc	= 0xFFFF;
			  motor.angle = (motor.angle) % 6 + 1; // Vorwärts immer, rückwärts nimmer!
			  motor.state = OBLDC_STATE_RUNNING_SLOW; // TODO: Make definitions for these values
		  }
		  chThdSleepMicroseconds(500);
	  }
	  if(motor.state == OBLDC_STATE_STARTING_SYNC) { // Ramp up the motor
		  //set_bldc_pwm(angle, 300 + (speed*4)/3, 50); // u/f operation
		  motor_cmd_temp.duty_cycle = 500 + (speed * 6) / 3; motor_cmd_temp.dir = 1;
		  motor_set_cmd( &motor, &motor_cmd_temp );
		  set_bldc_pwm(&motor);
		  speed = speed + acceleration;
		  delta_t = 1000000 / speed;
		  if (speed > 719) { // speed reached --> try to catch motor
			  motor.state = OBLDC_STATE_CATCHING;
			  set_bldc_pwm(&motor);
			  speed = 50;
			  catchcycle(&motor, 0, 0, 0, TRUE); // initialize catch state variables
		  }
		  chThdSleepMicroseconds(delta_t);
		  motor.angle = (motor.angle) % 6 + 1;
	  }
	  if(motor.state == OBLDC_STATE_CATCHING) {
		  //catchcount++;
		  catchconversion(); // start ADC Converter
		  chThdSleepMicroseconds(40);
		  // evaluate last ADC measurement
		  // determine voltage; for efficiency reasons, we calculating with the ADC value and do not convert to a float for [V]
		  int voltage_u = getcatchsamples()[0]; // /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
		  int voltage_v = getcatchsamples()[1]; // /4095.0 * 3 * 13.6/3.6;
		  int voltage_w = getcatchsamples()[2]; // /4095.0 * 3 * 13.6/3.6;
		  catchstate = catchcycle(&motor, voltage_u, voltage_v, voltage_w, FALSE);
		  // Write result TODO
		  if (1) {//catchstate != 0 && (motor.angle == 3 || motor.angle == 5) ) {
			  /*
			   * TODO catchcycle scheint fuer Winkel 3 und 5 ordentlich zu funktionieren.
			   * --> Ueberarbeiten, damit es fuer alle anderen Winkel auch geht!
			   */
			  /*catchresult = catchstate;
			  uartSendACK(); // Oszilloskop-Trigger auf UART
			  palClearPad(GPIOB, GPIOB_LEDR);
			  //adcStopConversion(&ADCD1);
			  motor.state = OBLDC_STATE_RUNNING;
			  motor.angle = 1;
			  motor_set_duty_cycle(&motor, 600);// ACHTUNG!!! 1000 geht gerade noch
			  set_bldc_pwm(&motor); // Start running with back EMF detection
			  chThdSleepMilliseconds(5000); chThdSleepMilliseconds(5000);
			  pwmStop(&PWMD1);
			  adcStopConversion(&ADCD1);
			  motor.angle = 3;
			  motor_set_duty_cycle(&motor, 600);// ACHTUNG!!! 1000 geht gerade noch
			  set_bldc_pwm(&motor); // Start running with back EMF detection
			  chThdSleepMilliseconds(5000); chThdSleepMilliseconds(5000);
			  palSetPad(GPIOB, GPIOB_LEDR);
			  pwmStop(&PWMD1);
			  adcStopConversion(&ADCD1);
			  catchcount = 0;
			  for (winkelcount=0; winkelcount < 10; winkelcount++) letzte_winkel[winkelcount] = 9;//Winkelcount ist Debug-Kram
			  winkelcount = 0;
			  motor.state = OBLDC_STATE_STARTING_SYNC;*/
		  }
		  if(catchcount > 10000) { // Timeout!
			  catchcount = 0;
			  for (winkelcount=0; winkelcount < 10; winkelcount++) letzte_winkel[winkelcount] = 9;
			  winkelcount = 0;
			  motor.state = OBLDC_STATE_STARTING_SYNC;
		  }
	  }
  }
  return 0;
}

void startRampMotorThread(void) {
  chThdCreateStatic(waRampMotorThread, sizeof(waRampMotorThread),
		  NORMALPRIO, tRampMotorTread, NULL);
}


// ----------- Alternativeloesung mit Virtuellen Timern
virtual_timer_t vt;
int angle = 1;
int acceleration = 3;
int speed = 100;  // initial speed
int delta_t = 50;
// Bestimmt die System-Tick-Frequenz: CH_CFG_ST_FREQUENCY. Vielleicht besser den Tickless mode verwenden?
void rampMotorCb(void *p) {
	angle = (angle) % 6 + 1;
	//set_bldc_pwm(angle, 300 + (speed*4)/3, 50); // u/f operation
	  speed = speed + acceleration;
	  delta_t = 1000000 / speed;
	  if (speed > 700) { // speed reached --> try to catch motor
		  //set_bldc_pwm(0,0,50);
		  speed = 100;
		  delta_t = 2000000;
	  }
	  // Restart the timer
	chSysLockFromISR();
	chVTSetI(&vt, US2ST(delta_t), rampMotorCb, p);
	chSysUnlockFromISR();
}

void startRampMotorCb(void) {
	chSysLock();
	chVTSetI(&vt, US2ST(1000), rampMotorCb, NULL);
	chSysUnlock();
}


