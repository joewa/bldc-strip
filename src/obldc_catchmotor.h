/*
 * obldc_catchmotor.h
 *
 *  Created on: 25.08.2014
 *      Author: joerg
 */

#ifndef OBLDC_CATCHMOTOR_H_
#define OBLDC_CATCHMOTOR_H_

#define OBLDC_MIN_CATCH_VOLTAGE 360 // 1V minimum voltage to evaluate for motor position catching; /4095 ADC resolution, *3 = ADC pin voltage, *13.6/3.6 = phase voltage TODO: Check max ADC voltage 3V or 3.3V?


//msg_t myThread(void *arg);
extern void startCatchMotorThread(void);

#endif /* OBLDC_CATCHMOTOR_H_ */
