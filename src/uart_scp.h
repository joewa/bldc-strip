/*
 * uart_scp.h
 *
 *  Created on: Jan 4, 2014
 *      Author: kjell
 */

/*
 * Packet structure
 *
 * Byte 0:  Command
 * Byte 1:  User0
 * Byte 2:	User1
 * Byte 3:	User2
 * Byte 4:	User3
 * Byte 5:	User4
 * Byte 6:  User5
 * Byte 7:  CRC8
 */

#ifndef UART_SCP_H_
#define UART_SCP_H_

#define CRC8INIT  0x00
#define CRC8POLY  0x07

#define SCP_ACK_TIMEOUT_MS		100
#define	SCP_PACKET_LENGTH		8

#define SCP_STATUS		0x00
#define SCP_ACK			0x01
#define SCP_NACK		0x02

/*
 * User0 is used to indicate Led on or off
 * 	User0 = 0x00 :  OFF
 * 	User0 = 0xFF :  ON
 */
#define SCP_LEDRED		0xF0
#define SCP_LEDGREEN	0xF1

void uartSCPInit(void);

#endif /* UART_SCP_H_ */
