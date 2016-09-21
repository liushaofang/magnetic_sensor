#ifndef __FLASH_H__
#define __FLASH_H__

#include "BspDef.h"
#include "SGR_CONCEPT.h"

#pragma pack(push, 1)
typedef struct _Device_Param_Data_Type
{
  uint8_t chMacAddr[3];
}Device_Param_Data_Type;

typedef struct _Magnet_Alg_Param_Type
{
  int16_t nHighThresholdVal;
  int16_t nLowThresholdVal;
  uint8_t nIncFluctuationLimit;
  uint8_t nDecFluctuationLimit;
  uint8_t nIsMagnetRunning;
  uint32_t nNormalTrafficCount;
  uint32_t nReverseTrafficCount;
}Magnet_Alg_Param_Type;

typedef struct _Sensor_Status_Type
{
  uint8_t nIsVehicleExist;
  uint8_t nIsMagnetRunning;
  uint8_t nBackgroundMagnetValue[2];
}Sensor_Status_Type;

#pragma pack(pop)

#define INFO_SEGMENT_SIZE                       (64 - 1)

#define INFO_SEGMENT_TYPE_A                     (0x1 << 0)
#define INFO_SEGMENT_TYPE_B                     (0x1 << 1)
#define INFO_SEGMENT_TYPE_C                     (0x1 << 2)
#define INFO_SEGMENT_TYPE_D                     (0x1 << 3)

#define SEGMENT_TYPE_MAC                        INFO_SEGMENT_TYPE_B
#define SEGMENT_TYPE_NETWORK_PARAMS             INFO_SEGMENT_TYPE_C
#define SEGMENT_TYPE_MAGNET_ALG_PARAMS          INFO_SEGMENT_TYPE_D
#define SEGMENT_TYPE_SENSOR_ADDR_LIST           INFO_SEGMENT_TYPE_D
#define SEGMENT_TYPE_TRAFFIC_COUNT              INFO_SEGMENT_TYPE_D

#define PARAM_TYPE_MAC                          (0x1 << 0)
#define PARAM_TYPE_NETWORK_PARAMS               (0x1 << 1)
#define PARAM_TYPE_MAGNET_ALG_PARAMS            (0x1 << 2)
#define PARAM_TYPE_SENSOR_ADDR_LIST             (0x1 << 3)
#define PARAM_TYPE_SENSOR_STATUS                (0x1 << 4)

#define FLASH_NETWORK_PARAMS_MAGIC_STRING        "NJRTGS"
#define FLASH_NETWORK_PARAMS_MAGIC_STRING_LEN    6

uint8_t init_system_flash_rw(void);
uint8_t erase_system_flash(uint8_t nSegmentType);
uint8_t read_system_param(uint16_t nParamType, uint8_t * pParamBuf, uint8_t nParamBufLen);
uint8_t write_system_param(uint16_t nParamType, uint8_t * nParamBuf, uint8_t nParamBufLen);
//uint8_t system_falsh_read_write_test(void);

#endif
