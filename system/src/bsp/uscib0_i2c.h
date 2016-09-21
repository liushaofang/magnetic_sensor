#ifndef __USCIB0_I2C_H__
#define __USCIB0_I2C_H__

#include "SGR_CONCEPT.h"

#ifdef HW_I2C_DEV

  void init_uscib0_i2c(void);
  void Ucb0I2c_Start(void);
  uint8_t Ucb0I2c_Read1byte(void);
  void I2C_Write1byte(uint8_t datareg,uint8_t data);
  uint8_t I2C_Read1byte(uint8_t datareg);
  void I2C_ReadNbyte(uint8_t *index,uint8_t n,uint8_t datareg);
  void Ucb0I2c_WriteNbyte(uint8_t  *index,uint8_t n);

#else

  #define I2C_TX_DATA_BUFFER_LEN  4
  #define I2C_RX_DATA_BUFFER_LEN  4

  #define DELAY_TIME  2 //经过测量采用的操作模式为250KHZ

  #define SDA_SET_OUTPUT  (P4DIR |= BIT4)
  #define SDA_SET_INPUT   (P4DIR &= ~BIT4)
  #define SDA_OUTPUT_HIGH (P4OUT |= BIT4)
  #define SDA_OUTPUT_LOW  (P4OUT &= ~BIT4)
  #define SCL_SET_OUTPUT  (P4DIR |= BIT5)
  #define SCL_OUTPUT_HIGH (P4OUT |= BIT5)
  #define SCL_OUTPUT_LOW  (P4OUT &= ~BIT5)
  #define SDA_DATA_OUT    (P4OUT & BIT4)
  #define SDA_DATA_IN     (P4IN & BIT4)
  #define SDA_DATA_BIT    4
  
  extern void init_uscib0_i2c(void);
  extern void uscib0_i2c_start(void);
  extern void uscib0_i2c_restart(void);
  extern void uscib0_i2c_stop(void);
  extern uint8_t uscib0_i2c_get_ack(void);
  extern void uscib0_i2c_ack(void);
  extern void uscib0_i2c_nack(void);
  extern uint8_t uscib0_i2c_write_byte(uint8_t btData);
  extern uint8_t uscib0_i2c_read_byte(void);

#endif

#endif
