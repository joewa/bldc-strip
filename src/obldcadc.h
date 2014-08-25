/*
 * obldcadc.h
 *
 *  Created on: 31.05.2014
 *      Author: joerg
 */

#ifndef OBLDCADC_H_
#define OBLDCADC_H_

void startmyadc(void);
void resetadccount(void);
void catchconversion(void);


adcsample_t* getcatchsamples(void);
int catchmotor_setup(void);

#endif /* OBLDCADC_H_ */
