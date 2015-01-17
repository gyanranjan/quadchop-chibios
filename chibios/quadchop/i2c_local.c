#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "debug.h"
#include "command.h"
#include "i2c.h"
#include "i2c_local.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//-----------------------I2C related  - START   --------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------



#define I2C_STAT_DONE 0
#define I2C_STAT_LASTREAD 2
#define I2C_STAT_BUSY 1
#define I2C_STAT_NACK 3

#define wakeup_isr(i2cp, msg) {                                             \
  chSysLockFromIsr();                                                       \
  if ((i2cp)->thread != NULL) {                                             \
    Thread *tp = (i2cp)->thread;                                            \
    (i2cp)->thread = NULL;                                                  \
    tp->p_u.rdymsg = (msg);                                                 \
    chSchReadyI(tp);                                                        \
  }                                                                         \
  chSysUnlockFromIsr();                                                     \
}


static void i2c_handler( I2CDriver *I2CDev );

I2CDriver I2CDev1; 

CH_IRQ_HANDLER(SAM3XA_TWI1_HANDLER) {
    CH_IRQ_PROLOGUE();
    i2c_handler(&I2CDev1);
    CH_IRQ_EPILOGUE();
}

static uint32_t i2cWaitTx(Twi *pTwi)
{
    volatile int32_t sr;
    do {
        sr = pTwi->TWI_SR;
        if ( sr & TWI_SR_NACK) {
            return sr;
        }
    } while (!(sr & TWI_SR_TXRDY));
    return 0;
}
    
static void i2cWaitComplete(Twi *pTwi)
{
    while(!(pTwi->TWI_SR & TWI_SR_TXCOMP)) ;
}


static void i2cPdcDisable(Twi *pTwi)
{
    pTwi->TWI_PTCR = TWI_PTCR_RXTDIS;
    pTwi->TWI_PTCR = TWI_PTCR_TXTDIS;
    pTwi->TWI_RPR = 0;
    pTwi->TWI_RCR = 0;
    pTwi->TWI_TPR = 0;
    pTwi->TWI_TCR = 0;
}


// i2c interrupt handler used only for PDC mode
// to prevent runaway operations caused by not responding
// fast enough and setting the stop bit.
static 
void i2c_handler( I2CDriver *I2CDev )
{
    uint32_t status;
    uint32_t all_stat = I2C_ALL_OK;
    
    Twi *pTwi = I2CDev->pTwi;

    status = pTwi->TWI_SR & pTwi->TWI_IMR; // Masked status


    if (TWI_SR_TXBUFE & status) { // Transmit has completed.
        i2cPdcDisable(pTwi);
        pTwi->TWI_CR = TWI_CR_STOP;
        pTwi->TWI_IDR = TWI_IDR_TXBUFE;
        pTwi->TWI_IER = TWI_IER_TXCOMP;

        
    } else if (TWI_SR_RXBUFF & status) { // Receive has completed
        
        if (I2CDev->status == I2C_STAT_LASTREAD) {
            i2cPdcDisable(pTwi);
            
            pTwi->TWI_IDR = TWI_IDR_RXBUFF;
            pTwi->TWI_IER = TWI_IER_TXCOMP;

        } else {
            I2CDev->status = I2C_STAT_LASTREAD;
            i2cPdcDisable(pTwi);
            pTwi->TWI_CR = TWI_CR_STOP;
            pTwi->TWI_RPR = I2CDev->lastByte;
            pTwi->TWI_RCR = 1;
            pTwi->TWI_PTCR = TWI_PTCR_RXTEN;
        }
 
    } else if (TWI_SR_TXCOMP & status) {
        I2CDev->status = I2C_STAT_DONE;
        pTwi->TWI_IDR = TWI_IDR_TXCOMP;
        i2cPdcDisable(pTwi);
        if (TWI_SR_NACK & status) {
            I2CDev->status = I2C_STAT_NACK;
            all_stat       = I2C_ERR_NAK;
        }
        wakeup_isr(I2CDev, all_stat);
    } else if (TWI_SR_NACK & status) {
        I2CDev->status = I2C_STAT_NACK;
        all_stat       = I2C_ERR_NAK;

//palSetPad(IOPORT2, 27);
//pTwi->TWI_IDR = 0x00F77;

        i2cPdcDisable(pTwi);
        wakeup_isr(I2CDev, all_stat);
    }
}

static void 
set_clock (I2CDriver *I2CDev )
{
    uint32_t  frequency = I2CDev->frequency;
    Twi *pTwi = I2CDev->pTwi;
    
    uint32_t dwCkDiv = 0 ;
    uint32_t dwClDiv ;
    uint32_t dwOk = 0 ;

    /* Configure clock */
    while ( !dwOk ){
        dwClDiv = ((SystemCoreClock / (2 * frequency)) - 4) / (1<<dwCkDiv) ;

        if ( dwClDiv <= 255 ){
            dwOk = 1 ;
        } else {
            dwCkDiv++ ;
        }
    }
    pTwi->TWI_CWGR = 0 ;
    pTwi->TWI_CWGR = (dwCkDiv << 16) | (dwClDiv << 8) | dwClDiv ;
}


void 
i2c_init(I2CDriver *I2CDev )
{
    Twi *pTwi = I2CDev->pTwi;        
    //setup pin
    peripheral_pin_apply(&I2CDev->sclk);
    peripheral_pin_apply(&I2CDev->sda1);
    
    //enable clock
    pmc_enable_peripheral_clock(I2CDev->peripheral_id);
    
    //setup waveform generator
    set_clock(I2CDev);
    
    //clear the RX FIFO
    pTwi->TWI_RHR;
    
    nvicEnableVector(TWI1_IRQn, CORTEX_PRIORITY_MASK(12));
    pTwi->TWI_IDR = 0x00F77;
    pTwi->TWI_CR = TWI_CR_MSEN ;
}


void
i2c_twi1_init ()
{
    I2CDev1.frequency = 100000;
        
    //PIO_PB13A_TWCK1
    //PIO_PB12A_TWD1 
    I2CDev1.sclk.port  = PIOB;
    I2CDev1.sclk.pin   = 13;
    I2CDev1.sclk.mode  = PIO_MODE_A;
    I2CDev1.sda1.port   = PIOB;
    I2CDev1.sda1.pin    = 12;
    I2CDev1.sda1.mode   = PIO_MODE_A;
    
    I2CDev1.peripheral_id = ID_TWI1;
    I2CDev1.pTwi = TWI1;
    
    i2c_init(&I2CDev1);
        
}  



static inline int32_t
i2c_ops (I2CDriver *I2CDev, uint8_t addr, const uint8_t* sendData, uint8_t sendSz, uint8_t* recvData, uint8_t recvSz) 
{
    Twi *pTwi = I2CDev->pTwi;
    int32_t   ret = I2C_ALL_OK;
    
    if (sendSz == 0 && recvSz == 0) { // Quick (for a scan)
        pTwi->TWI_MMR = 0;
        pTwi->TWI_MMR = (addr << 16);
        pTwi->TWI_CR = TWI_CR_QUICK;
        if (TWI_SR_NACK & (i2cWaitTx(pTwi))) {
            i2cWaitComplete(pTwi);
            ret = I2C_ERR_NAK;
            goto out;    
        }
        i2cWaitComplete(pTwi);
        return I2C_ALL_OK;
    } 
    
    //First handle TX case  || FIXME add a case if rx is there do not send stop bit
    if (sendSz > 0) {
        pTwi->TWI_TPR = (uint32_t)sendData;
        pTwi->TWI_TCR = sendSz;
        pTwi->TWI_MMR = 0;
        pTwi->TWI_MMR = (addr << 16);
        pTwi->TWI_IADR = 0;
        pTwi->TWI_IADR = 0;
        pTwi->TWI_PTCR = TWI_PTCR_TXTEN;
        pTwi->TWI_IER = TWI_IER_TXBUFE | TWI_IER_NACK;
        
         /* Waits for the operation completion.*/
        I2CDev->thread = chThdSelf();
        chSchGoSleepS(THD_STATE_SUSPENDED);
        
        ret = chThdSelf()->p_u.rdymsg;
        
    }
    
    if (recvSz == 1) {
        
        pTwi->TWI_RPR = (uint32_t)recvData;
        pTwi->TWI_RCR = 1;
        pTwi->TWI_MMR = 0;
        pTwi->TWI_MMR = TWI_MMR_MREAD | (addr << 16);
        pTwi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        
        I2CDev->status = I2C_STAT_LASTREAD;
        pTwi->TWI_IER = TWI_IER_RXBUFF | TWI_IER_NACK;

        /* Waits for the operation completion.*/
        I2CDev->thread = chThdSelf();
        pTwi->TWI_PTCR = TWI_PTCR_RXTEN;
        chSchGoSleepS(THD_STATE_SUSPENDED);

        ret = chThdSelf()->p_u.rdymsg;
    
    } else if (recvSz > 1) {
        
        pTwi->TWI_RPR = (uint32_t)recvData;
        pTwi->TWI_RCR = recvSz - 1; // Last byte read is handled manually
        pTwi->TWI_MMR = 0;
        pTwi->TWI_MMR = TWI_MMR_MREAD | (addr << 16);
        pTwi->TWI_CR = TWI_CR_START;
        
        I2CDev->lastByte = recvData[recvSz - 1];
        I2CDev->status = I2C_STAT_BUSY;
        pTwi->TWI_IER = TWI_IER_RXBUFF | TWI_IER_NACK;
        
        /* Waits for the operation completion.*/
        I2CDev->thread = chThdSelf();
        
        pTwi->TWI_PTCR = TWI_PTCR_RXTEN;
        chSchGoSleepS(THD_STATE_SUSPENDED);
        ret = chThdSelf()->p_u.rdymsg;
    }
    
out:
    if (ret != I2C_ALL_OK) {
        pTwi->TWI_CR = TWI_CR_STOP;
        quad_debug(DEBUG_WARN , "pTwi->TWI_IMR %x\n\r", pTwi->TWI_IMR);
        quad_debug(DEBUG_WARN , "pTwi->TWI_SR %x\n\r", pTwi->TWI_SR);
        i2cWaitComplete(pTwi);
    }
    return ret;
} 


int32_t
i2c_scan(uint8_t addr){
    return i2c_ops (&I2CDev1, addr, NULL, 0, NULL, 0); 
}

int32_t
i2c_send(uint8_t addr, const uint8_t* sendData, uint8_t sendSz){
    return i2c_ops (&I2CDev1, addr, sendData, sendSz, NULL, 0); 
}

int32_t
i2c_read(uint8_t addr,  uint8_t* recData, uint8_t recSz){
    return i2c_ops (&I2CDev1, addr, NULL, 0, recData, recSz); 
}

inline int32_t
i2c_read_reg(uint8_t addr, uint8_t reg , uint8_t* rec_data, uint8_t rec_sz)
{
    return i2c_ops (&I2CDev1, addr, &reg, 1, rec_data, rec_sz); 
}

int32_t 
i2c_read_reg_bits(uint8_t addr, uint8_t reg, uint8_t bit_start, 
                       uint8_t bit_len, uint8_t *data)
{
    bool status=0;
    uint8_t byte;
    uint8_t mask = ((1 << bit_len) - 1) << (bit_start - bit_len + 1);

    i2c_read_reg(addr, reg, &byte, 1);
 
    byte &= mask;
    byte >>= (bit_start - bit_len + 1);
    *data = byte;
    
    return status;
 }
 
 
int32_t 
i2c_write_byte(uint8_t addr, uint8_t reg, uint8_t data)
{
    //uint8_t byte[2];
    //byte[0] = reg;
    //byte[1] = data;
    //return  i2c_send(addr, byte, 2 ); 
    i2c_send(addr, reg, 1 );
    i2c_send(addr, data, 1 );
    return 0;
    
}

 
int32_t 
i2c_write_bit(uint8_t addr, uint8_t reg,
                    uint8_t bit_num, uint8_t data)
{
    uint8_t byte;
    i2c_read_reg(addr, reg, &byte, 1);
    byte = (data != 0) ? (byte | (1 << bit_num)) : (byte & ~(1 << bit_num));
    return i2c_write_byte(addr, reg, byte);
}
 
int32_t
i2c_write_bits( uint8_t addr, uint8_t reg,
                uint8_t bit_num, uint8_t bit_len, 
                uint8_t data)
{
    int32_t status;
    uint8_t byte;
    uint8_t mask;

    status = i2c_read_reg(addr, reg, &byte, 1);
    
    if (status == I2C_ALL_OK) {
        mask = ((1 << bit_len) - 1) << (bit_num - bit_len + 1);
        data <<= (bit_num - bit_len + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        byte &= ~(mask); // zero all important bits in existing byte
        byte |= data; // combine data with existing byte
        status = i2c_write_byte(addr, reg, byte);
    }
    return status;      
}    
  
