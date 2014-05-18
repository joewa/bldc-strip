/*
 * blinky.c
 *
 *  Created on: Dec 10, 2013
 *      Author: kjell
 */

#include "ch.h"
#include "hal.h"
#include "blinky.h"
#include "uart_scp.h"

static THD_WORKING_AREA(waBlinkyBlue, BLINKY_STACK_SIZE);
static THD_FUNCTION(tBlinkyBlue, arg) {
  (void)arg;
  chRegSetThreadName("BlinkyBlue");

  while (TRUE) {
    chThdSleepMilliseconds(200);
    // Use this with the strip
    //palTogglePad(GPIOB, GPIOB_LEDG);
    palTogglePad(GPIOB, GPIOB_LEDR);

    // Use this with the Discovery board
    //palTogglePad(GPIOC, GPIOC_LED4);
  }
  return 0;
}

void startBlinkyBlue(void) {
  chThdCreateStatic(waBlinkyBlue, sizeof(waBlinkyBlue),
  NORMALPRIO,
                    tBlinkyBlue, NULL);
}

static THD_WORKING_AREA(waBlinkyGreen, BLINKY_STACK_SIZE);
static THD_FUNCTION(tBlinkyGreen, arg) {
  (void)arg;
  chRegSetThreadName("BlinkyGreen");
  char sbyte = 77;
  while (TRUE) {
    chThdSleepMilliseconds(300);
    // Use this with the strip
    palTogglePad(GPIOB, GPIOB_LEDG);

    // Use this with the Discovery board
    //palTogglePad(GPIOC, GPIOC_LED3);
    uartStartSend(&UARTD1, 1, &sbyte);
  }
  return 0;
}

void startBlinkyGreen(void) {
  chThdCreateStatic(waBlinkyGreen, sizeof(waBlinkyGreen),
  NORMALPRIO,
                    tBlinkyGreen, NULL);
}
