#include "ch.h"
#include "hal.h"
#include "blinky.h"
#include "uart_scp.h"
#include "obldcpwm.h"
#include "obldcadc.h"
#include "obldc_catchmotor.h"

int main(void) {
	//Start System
	halInit();
	chSysInit();
	//startBlinkyBlue();
	startBlinkyGreen();
	//startBlinkyRed();
	uartSCPInit();
	//mystartPWM();
	startmyadc();
	int temp = catchmotor_setup();
	// start measurements
	//startcatchmodePWM();
	//startCatchMotorThread();
	startRampMotorThread();
	//startRampMotorCb(); // works a few cycles then it hangs
	// Just idle on the main loop
	while (TRUE) {
// palTogglePad(GPIOB, GPIOB_LEDG);
// palTogglePad(GPIOB, GPIOB_LEDR);
//uartStartSend(&UARTD1, 13, "Starting...\r\n");
		chThdSleepMilliseconds(1000);
		resetadccount();
	}
}
