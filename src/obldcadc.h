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
void v_bat_current_conversion(void);

adcsample_t* getcatchsamples(void);
adcsample_t get_vbat_sample(void);
int catchmotor_setup(void);

#endif /* OBLDCADC_H_ */
