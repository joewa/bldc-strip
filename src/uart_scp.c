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
#include "obldcpwm.h"

uint8_t rxBuffer[SCP_PACKET_LENGTH];
uint8_t txBuffer[SCP_PACKET_LENGTH];

extern motor_s motor; // Motor-struct from obldcpwm.c
extern motor_cmd_s motor_cmd;

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

  //chSysLockFromISR(); // commutation is disturbed when called
  uint16_t temp_uint16;
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
    case SCP_MOTORSTATE:
    	if (rxBuffer[1] == 0x00) {
    		motor.state = OBLDC_STATE_OFF;
    	} else {
    		if(motor.state == OBLDC_STATE_OFF) {
    			motor.state = OBLDC_STATE_STARTING_SENSE_1;
    		}
    	}
    	txBuffer[0] = SCP_ACK;
    	//txBuffer[1] = (uint8_t)motor.state;
    	//txBuffer[1] = (uint8_t)motor.dirjustchanged;
    	txBuffer[1] = (uint8_t)motor.state_reluct;
    	txBuffer[2] = (uint8_t)(motor.delta_t_zc >> 8); // High byte
    	txBuffer[3] = (uint8_t)(motor.delta_t_zc); // Low byte
    	//txBuffer[4] = (uint8_t)(motor.u_dc/10);//((motor.u_dc*100)/1630);
    	txBuffer[4] = (uint8_t)(motor.dir+1);//((motor.u_dc*100)/1630);
    	if (motor.u_dc2<0) motor.u_dc2 = -motor.u_dc2;
    	//txBuffer[5] = (uint8_t)(motor.u_dc2);//((motor.u_dc*100)/1630);
    	//txBuffer[5] = (uint8_t)(motor.i_dc_filt / 4);
    	txBuffer[5] = (uint8_t)(motor.delta_angle4+20);
    	//txBuffer[6] = (uint8_t)(motor.i_dc_ref / 4);
    	txBuffer[6] = (uint8_t)(motor.something+20);
      break;
    case SCP_SETDUTYCYCLE:
    	//if(rxBuffer[1]>7) rxBuffer[1] = 7;
    	//motor.dir = 1;
    	motor.dir_v_range = OBLDC_DIR_V_RANGE;
    	temp_uint16 = (uint16_t)(rxBuffer[1] << 8) + rxBuffer[2];
    	/*if(temp_uint16 >= 0) {
    		motor.dir = 1;// motor.dir = 1; //TODO: bis das python script geht
    	} else {
    		motor.dir = -1;
    		temp_int16 = -temp_int16;
    	}*/
    	if(temp_uint16 > OBLDC_PWM_MAX_DUTY_CYCLE) {
    		motor_cmd.duty_cycle = OBLDC_PWM_MAX_DUTY_CYCLE;
    	}
    	else {
    		motor_cmd.duty_cycle = temp_uint16;
    	}

        if(motor.dir == 0) {
        	motor.dir = motor_cmd.dir;
        }

    	txBuffer[0] = SCP_ACK;
      break;
    case SCP_DIRECTION:
        if (rxBuffer[1] == 0x00) {
        	motor_cmd.dir = -1;
        	motor_cmd.dir_control = -1;
        	txBuffer[0] = SCP_ACK;
        }
        else if (rxBuffer[1] == 0xFF) {
        	motor_cmd.dir = 1;
        	motor_cmd.dir_control = 1;
        	txBuffer[0] = SCP_ACK;
        }
        else {
        	txBuffer[0] = SCP_NACK;
        }
    	break;
    case SCP_POSITIONCONTROL:
        if (rxBuffer[1] == 0x00) {
        	motor.positioncontrol = 0;
        	txBuffer[0] = SCP_ACK;
        }
        else if (rxBuffer[1] == 0xFF) {
        	motor.positioncontrol = 1;
        	txBuffer[0] = SCP_ACK;
        	motor_cmd.angle = 0;
        	motor_cmd.dir_control = motor_cmd.dir;
        	motor.angle_sum = 0;
        	motor_cmd.angle = 0;
        }
        else {
        	txBuffer[0] = SCP_NACK;
        }
    	break;
    case SCP_ANGLE:
    	motor_cmd.newcmd = 1; // Wird in obldc_catchmotor ausgewertet. Funzt noch nicht.
    	temp_uint16 = (uint16_t)(rxBuffer[1] << 8) + rxBuffer[2];
    	if(temp_uint16 > 30000) {
    		motor_cmd.angle = 30000;
    	}
    	else {
    		motor_cmd.angle = temp_uint16;
    	}
        if(motor.dir == 0) {
        	motor.dir = motor_cmd.dir;
        }
    	txBuffer[0] = SCP_ACK;
      break;
    }


    txBuffer[7] = crc8(txBuffer, SCP_PACKET_LENGTH - 1);
    uartStartSendI(&UARTD1, SCP_PACKET_LENGTH, txBuffer);
    uartStartReceiveI(&UARTD1, SCP_PACKET_LENGTH, rxBuffer);

    //chSysUnlockFromISR();
  }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {txend1, txend2, rxend, rxchar, rxerr, 9600, 0,
                                USART_CR2_LINEN, 0};

void uartSendACK(void) {
	txBuffer[0] = SCP_ACK;
	uartStartSend(&UARTD1, 1, txBuffer);
}


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
