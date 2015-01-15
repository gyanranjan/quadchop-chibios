#include "ch.h"
#include "sam3x8e.h"
#include "halconf.h"
#include "hal.h"
#include "pal.h"
#include <serial.h>
#include <chprintf.h>
#include <stdarg.h>
#include "debug.h"


char *level_name[DEBUG_MAX] = {
"info",
"warn",
"error",
}; 


static SerialConfig  serial3;
BaseSequentialStream *stream = (BaseSequentialStream *)&SD5;

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

uint32_t
quad_serial_fetch_cmd() 
{
    return sdGet(&SD5);
}

