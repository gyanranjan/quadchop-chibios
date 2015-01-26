#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "gpio_intr.h"

static gpio_intr_func  port_d_handler[32];
static void            *port_d_ctx[32];

void 
PIOD_Handler(void)
{
    uint32_t pin = 0;
    uint32_t status = PIOD->PIO_ISR;
   
    for(pin = 0; pin < 32; pin++){
        if (status & (1u << pin)){
            if (port_d_handler[pin]) {
                port_d_handler[pin](port_d_ctx[pin]);
            }
        }   
    }
}

CH_IRQ_HANDLER(SAM3XA_PIOD_HANDLER) {
    CH_IRQ_PROLOGUE();
    PIOD_Handler( );
    CH_IRQ_EPILOGUE();
}

void 
init_gpio_intr(gpio_intr_cfg_t *cfg) 
{
    uint32_t  pin = (1u << cfg->pin.pin);
    //enable port
    peripheral_pin_apply(&cfg->pin);
    
    cfg->pin.port->PIO_PER  = pin;
    
    //Set as input
    cfg->pin.port->PIO_ODR = pin;
    
    //disable pullup 
    cfg->pin.port->PIO_PUDR = pin;
    
    //enable clock
    pmc_enable_peripheral_clock(cfg->peripheral_id);
    
    if (cfg->pin.port == PIOA) {
        nvicEnableVector(PIOA_IRQn, CORTEX_PRIORITY_MASK(14));
    } else if (cfg->pin.port == PIOB) {
        nvicEnableVector(PIOB_IRQn, CORTEX_PRIORITY_MASK(15));    
    } else if (cfg->pin.port == PIOC) {
        nvicEnableVector(PIOC_IRQn, CORTEX_PRIORITY_MASK(16));
    } else { /* Port D */
        nvicEnableVector(PIOD_IRQn, CORTEX_PRIORITY_MASK(17));
        port_d_handler[cfg->pin.pin] = cfg->func;
        port_d_ctx[cfg->pin.pin] = cfg->ctx;
    }
    
    //Additional Interrupt Modes Enable Register
    cfg->pin.port->PIO_AIMER = pin;

    //Edge Select Register
    //PIO_LSR
    if (cfg->type  == GPIO_INT_FALLING || cfg->type  == GPIO_INT_RISING) { 
        cfg->pin.port->PIO_ESR = pin;
        //Falling Edge/Low Level Select Register
        if (cfg->type  == GPIO_INT_FALLING)
            cfg->pin.port->PIO_FELLSR = pin;
        else
            cfg->pin.port->PIO_REHLSR = pin;
            
    } else {
        cfg->pin.port->PIO_LSR = pin;
        if (cfg->type  == GPIO_INT_LOW)
            cfg->pin.port->PIO_FELLSR = pin;
        else
            cfg->pin.port->PIO_REHLSR = pin;
    }
     //Finally enable interrupts on PORTC.PIN12
    cfg->pin.port->PIO_IER = pin;
}

