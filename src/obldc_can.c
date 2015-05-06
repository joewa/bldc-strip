/*
 * obldc_can.c
 *
 *  Created on: 27.04.2015
 *      Author: joerg
 */


#include "obldc_can.h"
#include "ch.h"
#include "hal.h"

// Threads

// Variable definitions
CANRxFrame rxmsg; //static uint8_t can_rx_buffer[256];
static uint8_t can_rx_buffer_last_id;

/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP, // , automatic wakeup,
  CAN_BTR_LBKM | CAN_BTR_SJW(0) | CAN_BTR_TS2(1) | // Loop back mode
  CAN_BTR_TS1(10) |
  CAN_BTR_BRP(1) // BaudRatePrescaler is 2 --> half frequency of APB1(here 28MHz) see Fig 395 "Bit timing"
};




static THD_WORKING_AREA(waCanComThread, 2048);
static THD_FUNCTION(tCanComThread, arg) {
	(void)arg;
	chRegSetThreadName("CanComThread");

	event_listener_t el;
	CANRxFrame rxmsg;
	int32_t ind = 0;
	int32_t rxbuf_len;
	uint8_t crc_low;
	uint8_t crc_high;
	bool commands_send;

	chEvtRegister(&CAND1.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
	    if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(100)) == 0)
	    	continue;
	    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
	    	/* Process message.*/
	    	//palTogglePad(IOPORT3, GPIOC_LED);
	    }
	}
	chEvtUnregister(&CAND1.rxfull_event, &el);
}


void obldc_can_init(void) {
	canStart(&CAND1, &cancfg);

	chThdCreateStatic(waCanComThread, sizeof(waCanComThread), NORMALPRIO, tCanComThread, NULL);
}
