
#include "ch.h"
#include "hal.h"
#include "blinky.h"
#include "uart_scp.h"
#include "obldcpwm.h"
#include "obldcadc.h"

int main(void) {

  //Start System
  halInit();
  chSysInit();


  //startBlinkyBlue();
  startBlinkyGreen();

  uartSCPInit();
  //mystartPWM();
  startmyadc();
  int temp = catchmotor();
  // Just idle on the main loop
  while (TRUE) {
	 //   palTogglePad(GPIOB, GPIOB_LEDG);
	 //   palTogglePad(GPIOB, GPIOB_LEDR);
	  uartStartSend(&UARTD1, 13, "Starting...\r\n");
	  chThdSleepMilliseconds(1000);
	  resetadccount();
  }
}
