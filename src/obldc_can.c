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
  CAN_BTR_LBKM | // Loop back mode
  CAN_BTR_SJW(3) | // Synchronization jump width; must be smaller or equal 4 and smaller than TS1 & TS2
  CAN_BTR_TS2(5) | // Time Segment 2;
  CAN_BTR_TS1(6) | // Time Segment 1;
  CAN_BTR_BRP(1) //CAN_BTR_BRP(1) // BaudRatePrescaler is 2 --> half frequency of APB1(here 28MHz) see Fig 395 "Bit timing"
};
// 24.7.4 seite 655 Each filter bank x consists of two 32-bit registers, CAN_FxR0 and CAN_FxR1.



/*
 * Receiver thread.
 */
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
	    	palTogglePad(GPIOB, GPIOB_LEDR);
	    }
	}
	chEvtUnregister(&CAND1.rxfull_event, &el);
}


/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
  CANTxFrame txmsg;

  (void)p;
  chRegSetThreadName("transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.EID = 0x01234567;
  txmsg.RTR = CAN_RTR_DATA;
  txmsg.DLC = 8;
  txmsg.data32[0] = 0x55AA55AA;
  txmsg.data32[1] = 0x00FF00FF;

  while (!chThdShouldTerminateX()) {
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(10));
    chThdSleepMilliseconds(50);
  }
  return 0;
}


void obldc_can_init(void) {
	canStart(&CAND1, &cancfg);

	//chThdCreateStatic(waCanComThread, sizeof(waCanComThread), NORMALPRIO + 7, tCanComThread, NULL);
	// Empfangsthread hängt manchmal wenn was empfangen wird

    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);

}
