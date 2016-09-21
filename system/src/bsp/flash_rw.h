#ifndef __FLASH_RW_H__
#define __FLASH_RW_H__

#include "BspDef.h"
#include "SGR_CONCEPT.h"

#define INFO_D_ADDR 0x1800
#define INFO_C_ADDR 0x1880
#define INFO_B_ADDR 0x1900
#define INFO_A_ADDR 0x1980

#define INFO_SEGMENT_SELECT_TYPE_A 0x0
#define INFO_SEGMENT_SELECT_TYPE_B 0x1
#define INFO_SEGMENT_SELECT_TYPE_C 0x2
#define INFO_SEGMENT_SELECT_TYPE_D 0x3
#define INFO_SEGMENT_SELECT_COUNT  0x4

#define INFO_SEGMENT_OFFSET_START  0x10

void flash_write_device_id(uint32_t lDeviceID);
uint32_t flash_read_device_id(void);
#if 0
void flash_read_write_address_init(void);
uint8_t flash_write_data(uint8_t u8Data[], uint8_t uDataLen);
uint8_t flash_read_data(uint8_t u8Data[], uint8_t uDataSize);
#endif
uint8_t flash_write_data(uint8_t u8Data[], uint8_t uDataLen);
uint8_t flash_read_data(uint8_t u8Data[], uint8_t uDataSize);

#endif