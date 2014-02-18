
#include "ch.h"
#include "hal.h"
#include "blinky.h"
#include "uart_scp.h"

int main(void) {

  //Start System
  halInit();
  chSysInit();

  startBlinkyBlue();
  //startBlinkyGreen();

  uartSCPInit();

  // Just idle on the main loop
  while (TRUE) {
	  uartStartSend(&UARTD1, 13, "Starting...\r\n");
	  chThdSleepMilliseconds(1000);
  }
}
