/*
 * uart_scp.c
 *
 *  Created on: Jan 4, 2014
 *      Author: kjell
 */

#include "ch.h"
#include "hal.h"
#include "uart.h"
#include "uart_scp.h"

uint8_t rxBuffer[SCP_PACKET_LENGTH];
uint8_t txBuffer[SCP_PACKET_LENGTH];

uint8_t crc8(uint8_t *data_in, uint8_t number_of_bytes_to_read) {
  uint8_t crc;
  uint8_t loop_count;
  uint8_t bit_counter;
  uint8_t data;
  uint8_t feedback_bit;

  crc = CRC8INIT;

  for (loop_count = 0; loop_count != number_of_bytes_to_read; loop_count++) {
    data = data_in[loop_count];

    bit_counter = 8;
    do {
      feedback_bit = (crc ^ data) & 0x01;

      if (feedback_bit == 0x01) {
        crc = crc ^ CRC8POLY;
      }
      crc = (crc >> 1) & 0x7F;
      if (feedback_bit == 0x01) {
        crc = crc | 0x80;
      }

      data = data >> 1;
      bit_counter--;

    } while (bit_counter > 0);
  }

  return crc;
}

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
  (void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
  (void)uartp;
  (void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
  (void)uartp;
  (void)c;
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
  (void)uartp;

  chSysLockFromISR();

  uint8_t crc = crc8(rxBuffer, SCP_PACKET_LENGTH - 1);


  if (crc == rxBuffer[7]) {
    switch (rxBuffer[0]) {

    case SCP_LEDRED:
      if (rxBuffer[1] == 0x00) {
        //palClearPad(GPIOC, GPIOC_LED4);         //STM32F0Discovery
        palClearPad(GPIOB, GPIOB_LEDR);       //Strip v2
        txBuffer[0] = SCP_ACK;
      }
      else if (rxBuffer[1] == 0xFF) {
        //palSetPad(GPIOC, GPIOC_LED4);           //STM32F0Discovery
        palSetPad(GPIOB, GPIOB_LEDR);         //Strip v2
        txBuffer[0] = SCP_ACK;
      }
      else {
        txBuffer[0] = SCP_NACK;
      }
      break;

    case SCP_LEDGREEN:
      if (rxBuffer[1] == 0x00) {
        //palClearPad(GPIOC, GPIOC_LED3);         //STM32F0Discovery
        palClearPad(GPIOB, GPIOB_LEDG);       //Strip v2
        txBuffer[0] = SCP_ACK;
      }
      else if (rxBuffer[1] == 0xFF) {
        //palSetPad(GPIOC, GPIOC_LED3);           //STM32F0Discovery
        palSetPad(GPIOB, GPIOB_LEDG);         //Strip v2
        txBuffer[0] = SCP_ACK;
      }
      else {
        txBuffer[0] = SCP_NACK;
      }
      break;

    }


    txBuffer[7] = crc8(txBuffer, SCP_PACKET_LENGTH - 1);
    uartStartSendI(&UARTD1, SCP_PACKET_LENGTH, txBuffer);
    uartStartReceiveI(&UARTD1, SCP_PACKET_LENGTH, rxBuffer);

    chSysUnlockFromISR();
  }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {txend1, txend2, rxend, rxchar, rxerr, 9600, 0,
                                USART_CR2_LINEN, 0};

void uartSCPInit(void) {
	/*
	 * * Activates the serial driver 1
	 * */
	//palSetPadMode(GPIOB, GPIOB_USART_TX, PAL_MODE_ALTERNATE(0));
	//palSetPadMode(GPIOB, GPIOB_USART_RX, PAL_MODE_ALTERNATE(0));
	uartStart(&UARTD1, &uart_cfg_1);
	/*
	 * * Initiate RX and wait for message
	 * */
	uartStartReceive(&UARTD1, SCP_PACKET_LENGTH, &rxBuffer[0]);
}
