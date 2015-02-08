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

uint8_t debugbyte;

static THD_WORKING_AREA(waBlinkyRed, BLINKY_STACK_SIZE);
static THD_FUNCTION(tBlinkyRed, arg) {
  (void)arg;
  chRegSetThreadName("BlinkyBlue");

  while (TRUE) {
    chThdSleepMilliseconds(250);
    // Use this with the strip
    //palTogglePad(GPIOB, GPIOB_LEDG);
    palTogglePad(GPIOB, GPIOB_LEDR);

    // Use this with the Discovery board
    //palTogglePad(GPIOC, GPIOC_LED4);
  }
  return 0;
}

void startBlinkyRed(void) {
  chThdCreateStatic(waBlinkyRed, sizeof(waBlinkyRed),
  NORMALPRIO,
                    tBlinkyRed, NULL);
}

static THD_WORKING_AREA(waBlinkyGreen, BLINKY_STACK_SIZE);
static THD_FUNCTION(tBlinkyGreen, arg) {
  (void)arg;
  chRegSetThreadName("BlinkyGreen");
  char sbyte = 77;
  uint8_t last_debugbyte=33;
  while (TRUE) {
    chThdSleepMilliseconds(50);
    /*if(debugbyte!=last_debugbyte) {
    	last_debugbyte = debugbyte;
    	uartStartSend(&UARTD1, 1, &debugbyte);
    }*/
    // Use this with the strip
    palTogglePad(GPIOB, GPIOB_LEDG);
    //palTogglePad(GPIOB, GPIOB_U_NDTS);
    //palTogglePad(GPIOB, GPIOB_V_NDTS);
    //palTogglePad(GPIOB, GPIOB_W_NDTS);


    // Use this with the Discovery board
    //palTogglePad(GPIOC, GPIOC_LED3);
    //uartStartSend(&UARTD1, 1, &sbyte);
  }
  return 0;
}

void startBlinkyGreen(void) {
  chThdCreateStatic(waBlinkyGreen, sizeof(waBlinkyGreen),
  NORMALPRIO,
                    tBlinkyGreen, NULL);
}
