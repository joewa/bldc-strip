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

#include "obldcpwm.h"

static void pwmpcb(PWMDriver *pwmp) {

  (void)pwmp;
  palSetPad(GPIOB, GPIOB_LEDR);
}

static void pwmc1cb(PWMDriver *pwmp) {

  (void)pwmp;
  palClearPad(GPIOB, GPIOB_LEDR);
}

static PWMConfig pwmcfg = {
  1e6,//2e6, /* 2MHz PWM clock frequency */
  1000,//200, /* PWM period 100us */
  pwmpcb,  /* No callback */
  /* Only channel 1 enabled */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, pwmc1cb},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
  },
  0, // TIM CR2 register initialization data, "should normally be zero"
  0 // TIM DIER register initialization data, "should normally be zero"
};

void mystartPWM(void) {
    /* Enables PWM output (of TIM1, channel 1) on "U" phase connected to PA8 */
    //palSetPadMode(GPIOA, GPIOA_U_PWM, PAL_MODE_STM32_ALTERNATE_PUSHPULL); // ist schon in board.h definiert
    pwmStart(&PWMD1, &pwmcfg);
    //pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 5000));
    //palSetPad(GPIOB, GPIOB_U_NDTS); // activate driver
    //pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
    //palSetPad(GPIOB, GPIOB_V_NDTS); // activate driver
    pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, 2500));
    palSetPad(GPIOB, GPIOB_W_NDTS); // activate driver
}
