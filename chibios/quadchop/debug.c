#include "ch.h"
#include "sam3x8e.h"
#include "halconf.h"
#include "hal.h"
#include "pal.h"
#include <serial.h>
#include <chprintf.h>
//#include <stdio.h>
#include <stdarg.h>
//#include <strings.h>
#include "debug.h"


char *level_name[DEBUG_MAX] = {
"info",
"warn",
"error",
}; 


static SerialConfig  serial3;
BaseSequentialStream *stream = (BaseSequentialStream *)&SD5;
char printbuff[80];

void
quad_debug_init() 
{
    serial3.rts_pin.port = NULL;
	serial3.cts_pin.port = NULL;
	serial3.tx_pin.port = PIOD;
	serial3.tx_pin.pin = 4;//PIO_PD4B_TXD3;
	serial3.tx_pin.mode = PIO_MODE_B;
	serial3.rx_pin.port = PIOD;
	serial3.rx_pin.pin = 5;//PIO_PD5B_RXD3;
	serial3.rx_pin.mode = PIO_MODE_B;
	serial3.speed = 9600;
	serial3.mr = US_MR_PAR_NO  |US_MR_CHMODE_NORMAL | US_MR_NBSTOP_1_BIT | US_MR_PAR_NO| US_MR_CHRL_8_BIT;
    serial3.rtor= 0;
    serial3.ttgr= 0;
	sdStart(&SD5, &serial3);
}

// uint32_t 
// quad_debug(int level, const char *format, ...)
// {
    // char *level_name;
    // va_list args;
    // //chprintf(chp, "debug");
    // //return 0;
    // //bzero(printbuff,sizeof(printbuff));
    
    // // va_start(args, format);
    // // vsnprintf(printbuff, sizeof(printbuff), format, args);
    // // va_end(args);
    // if( level >= DEBUG_LEVEL ) {
        // if (level == DEBUG_INFO) {
            // level_name = "info:";
        // } else if (level == DEBUG_WARN) {
           // level_name = "warn:";
        // } else {
            // level_name = "err:";
        // }
        // //chprintf(chp, "%s:%ld:",level_name, chTimeNow());
        // //chprintf(chp, "%s\n\r", printbuff);
        // chprintf(chp, "hello\n\r");
    // }
    // return 0;
// }
uint32_t
quad_serial_fetch_cmd() 
{
    return sdGet(&SD5);
}

