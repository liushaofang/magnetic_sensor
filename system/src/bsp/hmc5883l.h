#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include "SGR_CONCEPT.h"

#define CONFIG_REGISTER_A    0x00  /*  ¶ÁÐ´  */
#define CONFIG_REGISTER_B    0x01  /*  ¶ÁÐ´  */
#define MODE_REGISTER        0x02  /*  ¶ÁÐ´  */
#define OUT_X_MSB_ADDR	     0x03  /*  Ö»¶Á  */
#define OUT_X_LSB_ADDR	     0x04  /*  Ö»¶Á  */
#define OUT_Y_MSB_ADDR	     0x05  /*  Ö»¶Á  */
#define OUT_Y_LSB_ADDR	     0x06  /*  Ö»¶Á  */
#define OUT_Z_MSB_ADDR	     0x07  /*  Ö»¶Á  */
#define OUT_Z_LSB_ADDR	     0x08  /*  Ö»¶Á  */
#define STATUS_REGISTER      0x09  /*  ¶ÁÐ´  */
#define ID_REGISTER_A        0x0A  /*  ¶ÁÐ´  */
#define ID_REGISTER_B        0x0B  /*  ¶ÁÐ´  */
#define ID_REGISTER_C        0x0C  /*  ¶ÁÐ´  */

#define CONFIG_ODR_0dot75HZ     0x00    /*      0.75HZ  */
#define CONFIG_ODR_1dot5HZ      0x04    /*      1.5HZ   */
#define CONFIG_ODR_3HZ          0x08    /*      3HZ     */
#define CONFIG_ODR_7dot5HZ      0x0C    /*      7.5HZ   */
#define CONFIG_ODR_15HZ         0x10    /*      15HZ    */
#define CONFIG_ODR_30HZ         0x14    /*      30HZ    */
#define CONFIG_ODR_75HZ         0x18    /*      75HZ    */

#define MODE_CONTINOUS_MEASUREMENT     0x0
#define MODE_SINGLE_MEASUREMENT        0x1
#define MODE_IDLE                      0x3

#define SET_HIGH_THRESHOLD      (0x1 << 0)
#define SET_LOW_THRESHOLD       (0x1 << 1)
#define SET_INC_COUNT_LIMIT     (0x1 << 2)
#define SET_DEC_COUNT_LIMIT     (0x1 << 3)

#define FLUCTUATION_NONE        0
#define FLUCTUATION_UPWARD      1
#define FLUCTUATION_DOWNWARD    2

/**
 * Define P2.3 as magnet interrupt pin.
 */
#define P2BIT_MAGNET_INT 3
#define P2BIT_MAGNET_BIT (1 << P2BIT_MAGNET_INT)

#define CLEAR_MAGNET_INT_FLAG() st( P2IFG &= ~BV(P2BIT_MAGNET_INT); )
#define IS_MAGNET_INT_FLAG_SET() (P2IFG & BV(P2BIT_MAGNET_INT) )

typedef void (* EXECUTE_AT_MAG_VALUE_UPDATE)(void);

int init_mag_sensor(void);
void disable_hmc5883l_int(void);
void enable_hmc5883l_int(void);
void reset_background_magnetic(void);
__monitor void suspend_hmc5883l(void);
__monitor uint8_t resume_hmc5883l(void);
__monitor uint8_t set_hmc5883l_odr(uint8_t nOutputDataRate);
void set_threshold_values(uint8_t uSetFlag, int16_t n_high_threshold_new_value, int16_t n_low_threshold_new_value, uint8_t n_inc_fluctuation_limit, uint8_t n_dec_fluctuation_limit);
uint16_t getValue(uint8_t volatile * pIsCarParked, uint8_t *raw_data, uint8_t nRawDataLen);
#ifdef MAG_TEST
uint16_t getMagTestValue(uint8_t *mag_data, uint8_t nMagDataLen, uint8_t * bg_mag_data, uint8_t nBgMagDataLen, int16_t * n16VariantData);
#endif
void executeAtUpdateMagValue();

#endif
