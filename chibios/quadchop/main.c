#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "debug.h"
#include "command.h"


PWMConfig      pwm_ch0_cfg;
#define FIXED_FREQUENCY (1000000)
#define MIN_ESC         (700)
#define MAX_ESC         (2400)
#define INC_ESC         (50  )


static WORKING_AREA(waThread1, 256);
int throttle_val = MIN_ESC;


void
esc_configure()
{
    pwm_ch0_cfg.frequency              = FIXED_FREQUENCY ;
    pwm_ch0_cfg.period                 = MAX_ESC    ;
    pwm_ch0_cfg.callback               = NULL            ; 
    pwm_ch0_cfg.channels[0].mode       = PWM_CHANNEL_POLARITY_HIGH;
    pwm_ch0_cfg.channels[0].h_pin.port = PIOC;
    pwm_ch0_cfg.channels[0].h_pin.pin  = 3;
    pwm_ch0_cfg.channels[0].h_pin.mode = PIO_MODE_B;
    pwm_ch0_cfg.channels[0].l_pin.port = PIOC;
    pwm_ch0_cfg.channels[0].l_pin.pin  = 2;
    pwm_ch0_cfg.channels[0].l_pin.mode = PIO_MODE_B;
    
    pwmStart(&PWMD1, &pwm_ch0_cfg);
}

void
esc_set_throttle(uint16_t duty )
{
    pwm_lld_set_dutycycle( &PWMD1, duty);
}

static msg_t Thread1(void *arg) {
    (void)arg;
    quad_command_t cmd;
    while (TRUE) {
        quad_cmd_collector();
        quad_cmd_get(&cmd);
        if (cmd == CMD_INC_THROTTLE) {
        quad_debug(DEBUG_WARN , "CMD_INC_THROTTLE %d  \n\r", throttle_val);
            throttle_val += INC_ESC;
            if (throttle_val > MAX_ESC) {
                throttle_val = MAX_ESC;
                esc_set_throttle(throttle_val);
            }
        } else if (cmd == CMD_DEC_THROTTLE) {
         quad_debug(DEBUG_WARN , "CMD_DEC_THROTTLE %d  \n\r", throttle_val);
            throttle_val -= INC_ESC;
            if (throttle_val < MIN_ESC ) {
                throttle_val = MIN_ESC;
                esc_set_throttle(throttle_val);
            }
        } else if (cmd == CMD_ARM) {
            quad_debug(DEBUG_WARN , "Armed  \n\r");
            //esc_arm_g();
        } else if (cmd == CMD_TOP) {
            throttle_val = MAX_ESC;
            esc_set_throttle(throttle_val);
            quad_debug(DEBUG_WARN , "Top  %d\n\r", throttle_val);
        } else if (cmd == CMD_DOWN) {
            throttle_val = MIN_ESC;
            esc_set_throttle(MIN_ESC);
            quad_debug(DEBUG_WARN , "Down  %d\n\r", throttle_val);
        }
    }
	return 0;
}


static 
void disable_wdt(void)
{
    volatile Wdt *WD = (Wdt *)0x400E1A50U;
    WD->WDT_MR = WDT_MR_WDDIS;
}


int main(void) {

    halInit();
    chSysInit();
    quad_debug_init();
    quad_cmd_init();
    disable_wdt();
    pwmInit();
  esc_configure();
	chThdCreateStatic(waThread1, 
                      sizeof(waThread1), 
                      NORMALPRIO, 
                      Thread1, NULL);
	

    while (1);
}


