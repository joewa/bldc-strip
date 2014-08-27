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


static uint8_t halldecode[8];

int last_hall_decoded;
int catchcycle(int voltage_u, int voltage_v, int voltage_w, uint8_t init) {
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
		int vdiff_max = MAX(MAX(vdiff_1_last, vdiff_2_last), vdiff_3_last);
		int vdiff_min = MIN(MIN(vdiff_1_last, vdiff_2_last), vdiff_3_last);

		// when difference between min and max values > "MinCatchVoltage" -> Cond 1 fulfilled
		if (ABS(vdiff_max - vdiff_min) > OBLDC_MIN_CATCH_VOLTAGE) {
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
				if (ABS(hall_decoded - last_hall_decoded) == 1 && (last_hall_decoded != 0)) {
					// determine direction of rotation
					if (hall_decoded > last_hall_decoded) {
						direction = 1;// lieber -1 oder 1
					} else {
						direction = 2;
					}
				}
				last_hall_decoded = hall_decoded;
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

  catchcycle(0, 0, 0, TRUE); // initialize catch state variables
  while (TRUE) {
	  catchconversion(); // start ADC Converter
	  chThdSleepMicroseconds(200);
	  // evaluate last ADC measurement
	  // determine voltage; for efficiency reasons, we calculating with the ADC value and do not convert to a float for [V]
	  int voltage_u = getcatchsamples()[0]; // /4095.0 * 3 * 13.6/3.6; // convert to voltage: /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage
	  int voltage_v = getcatchsamples()[1]; // /4095.0 * 3 * 13.6/3.6;
	  int voltage_w = getcatchsamples()[2]; // /4095.0 * 3 * 13.6/3.6;
	  catchstate = catchcycle(voltage_u, voltage_v, voltage_w, FALSE);
	  // Write result TODO
	  if (catchstate != 0) catchresult = catchstate;
  }
  return 0;
}

void startCatchMotorThread(void) {
  chThdCreateStatic(waCatchMotorThread, sizeof(waCatchMotorThread),
		  NORMALPRIO, tCatchMotorTread, NULL);
}


