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

adcsample_t* getcatchsamples(void);
int catchmotor(void);

#endif /* OBLDCADC_H_ */
