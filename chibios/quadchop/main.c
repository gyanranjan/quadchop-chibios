#include "ch.h"
#include "hal.h"
#include "pal.h"
#include "debug.h"
#include "command.h"
#include "i2c.h"
#include "i2c_local.h"
#include "interface.h"

//PWMConfig      pwm_ch0_cfg;
#define FIXED_FREQUENCY (1000000)
#define MIN_ESC         (700)
#define MAX_ESC         (2400)
#define INC_ESC         (50  )


#define GYRO_RATE 200 
//------------------------------------
//  Heart beat disaplay  blinks an LED
//------------------------------------
static WORKING_AREA(waHB, 64);
static msg_t HBTh(void *arg) {
	(void)arg;
	palSetPadMode(IOPORT3, 30, PAL_MODE_OUTPUT_PUSHPULL);
	while (TRUE) {
		palTogglePad(IOPORT3, 30);
		chThdSleepMilliseconds(1000);
	}
	return 0;
}

//------------------------------------
//  Watchdog disable
//------------------------------------
static 
void disable_wdt(void)
{
    volatile Wdt *WD = (Wdt *)0x400E1A50U;
    WD->WDT_MR = WDT_MR_WDDIS;
}


  int16_t axi16, ayi16, azi16;
  int16_t gxi16, gyi16, gzi16;

static WORKING_AREA(i2Test, 512);
static msg_t i2c(void *arg) {
    int i;
    uint8_t  send[8];
    uint8_t  addr[2] = {0x68, 0x1e};
    i2c_twi1_init();
    int ret;
    //int scrap;
    int device = 0;
    quad_debug(DEBUG_WARN , "twi init done\n\r");
    ms_open(GYRO_RATE);
    
//mpu6050Init();


//if ( mpu6050Test() == TRUE) {
//mpu6050Reset();
//chThdSleepMilliseconds(50);

//mpu6050SetSleepEnabled(FALSE);
//mpu6050SetTempSensorEnabled(TRUE);
//mpu6050SetIntEnabled(FALSE);
//mpu6050SetI2CBypassEnabled(FALSE);
//mpu6050SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
//mpu6050SetFullScaleGyroRange(MPU6050_GYRO_FS_2000);
//mpu6050SetFullScaleAccelRange(MPU6050_ACCEL_FS_8);
//mpu6050SetRate(15);  // 8000 / (1 + 15) = 500Hz
//mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);

        
        //quad_debug(DEBUG_WARN , "Device MPU6050 found \n\r");
        //quad_debug(DEBUG_WARN , "Range %f \n\r",  mpu6050GetFullScaleAccelGPL());
        //quad_debug(DEBUG_WARN , "Range %f \n\r",  mpu6050GetFullScaleGyroDPL());
//        mpu6050SelfTest();
        
    //for (scrap = 0; scrap < 200; scrap++)
    //{
        //mpu6050GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
        //quad_debug(DEBUG_WARN , "ax-%d, ay-%d, az-%d, gx-%d, gy-%d, gz-%d \n\r",axi16, ayi16, azi16, gxi16, gyi16, gzi16 );
        //chThdSleepMilliseconds(500);
    //}
        
        
    //} else {
        //quad_debug(DEBUG_WARN , "Device MPU6050  not found \n\r");
    //}
    
   
    while(1) {
        //mpu6050GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
        ms_update();
        chThdSleepMilliseconds(10);
    }
    
    while(1)  {
        for (i=1; i<127; i++){
            device = addr[i%2];
            ret = i2c_read_reg(device,0x75,send, 1);
            quad_debug(DEBUG_WARN , "Device %x %s \n\r", device , ret == I2C_ALL_OK ? "Found":"Not Found");
            
            if (ret == I2C_ALL_OK) {
                quad_debug(DEBUG_WARN , "%x %x %x %x \n\r", send[0], send[1], send[2], send[3]);
                
            }
            chThdSleepMilliseconds(200);
        }
        i=1;
    }
    return 0;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//-----------------------I2C related  - END  -----------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

int main(void) {

    halInit();
    chSysInit();
    quad_debug_init();
    quad_cmd_init();
    disable_wdt();
    
    palSetPadMode(IOPORT2, 27, PAL_MODE_OUTPUT_PUSHPULL);
    palClearPad(IOPORT2, 27);
	
    chThdCreateStatic(i2Test, 
                  sizeof(i2Test), 
                  NORMALPRIO+2, 
                  i2c, NULL);
                  
                  
	chThdCreateStatic(waHB, 
                      sizeof(waHB), 
                      LOWPRIO , 
                      HBTh, NULL);
    while (1){
    chThdSleepMilliseconds(5000);
    }
    
    return 0;
}





