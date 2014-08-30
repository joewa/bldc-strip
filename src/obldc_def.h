/*
 * obldc_def.h
 *
 *  Created on: 25.08.2014
 *      Author: joerg
 */

#ifndef OBLDC_DEF_H_
#define OBLDC_DEF_H_

// TODO: Lieber als inline implementieren!
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
//#define ABS(a) (((a)>0)?(a):-(a)) // TODO: ACHTUNG KAPUTT!!!

typedef struct {
	int KV; // Voltage constant; scaled with respect to ADC resolution and evaluation method, whatever it will be...
} Motor_t;
/*
 * Stores motor parameters and states used in many functions.
 * The unit time is microseconds [us]
 */

#endif /* OBLDC_DEF_H_ */
