#ifndef I2C_L_H
#define I2C_L_H

#define I2C_ALL_OK              0       //all OK
#define I2C_ERR_NAK             1       //got a NAK
#define I2C_ERR_INVAL           2       //invalid command given

typedef struct I2CDriver{
  uint8_t                   status;
  uint8_t                   *rxbuf;
  const uint8_t             *txbuf;
  size_t                    rxbytes;
  size_t                    txbytes;
  Mutex                     mutex;
  Thread                    *thread;
  uint8_t                   lastByte;
  
  //Below must be initialised
  uint32_t                  peripheral_id;
  uint32_t                  frequency;
  Twi                       *pTwi;
  
  //pin related 
  PeripheralPinConfig       sclk;
  PeripheralPinConfig       sda1;
}I2CDriver;

void i2c_twi1_init ();
int32_t i2c_scan(uint8_t addr);

int32_t i2c_send(uint8_t addr, const uint8_t* sendData, uint8_t sendSz);

int32_t i2c_read(uint8_t addr, const uint8_t* recData, uint8_t recSz);



#endif
