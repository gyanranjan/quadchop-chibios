#include "ch.h"
#include "sam3x8e.h"
#include "halconf.h"
#include "hal.h"
#include "pal.h"
#include <serial.h>
#include <chprintf.h>
#include "debug.h"



// void send_char(char a);
// char get_char();

/*
 * This is a periodic thread that does absolutely nothing except increasing
 * the seconds counter.
 */
static WORKING_AREA(waThread1, 128);





static msg_t Thread1(void *arg) {
	(void)arg;
	uint8_t a;
	int i;


	palSetPadMode(IOPORT2, 27, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(IOPORT3, 30, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(IOPORT1, 21, PAL_MODE_OUTPUT_PUSHPULL);
palTogglePad(IOPORT2, 27);

	
	//palClearPad(IOPORT3, 30);
	//palSetPad(IOPORT2, 27);
	//palClearPad(IOPORT1, 21);
	
	while (TRUE) {
        //if (i > 'z') i='a';
        //i++;
       // sdWrite(&SD5, "how are you\n\r", 15);
       quad_debug(DEBUG_INFO,"here we go\n\r");
       
       //chprintf(chp,"Hi I m  here \n")
        palTogglePad(IOPORT1, 21);
		chThdSleepMilliseconds(1000);
	}
	return 0;
}


/*
 * Application entry point.
 */
int main(void) {

    
    halInit();
	chSysInit();
	quad_debug_init();
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	

  while (1);
}


