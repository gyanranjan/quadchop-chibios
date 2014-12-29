#include "ch.h"
#include "hal.h"
#include "debug.h"
#include "command.h"

static WORKING_AREA(waThread1, 256);

static msg_t Thread1(void *arg) {
	(void)arg;
	while (TRUE) {
        quad_cmd_collector();
	}
	return 0;
}

static 
void disable_wdt(void)
{
volatile Wdt *WD = (Wdt *)0x400E1A50U;
WD->WDT_MR = WDT_MR_WDDIS;

}
/*
 * Application entry point.
 */
int main(void) {
    halInit();
	chSysInit();
	quad_debug_init();
    quad_cmd_init();
    disable_wdt();
	chThdCreateStatic(waThread1, 
                      sizeof(waThread1), 
                      NORMALPRIO, 
                      Thread1, NULL);
	

    while (1);
}


