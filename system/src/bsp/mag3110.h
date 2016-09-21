#ifndef __MAG3110_H__
#define __MAG3110_H__

#include "SGR_CONCEPT.h"

typedef void (* EXECUTE_AT_MAG_VALUE_UPDATE)(void);

int init_mag_sensor();
void executeAtUpdateMagValue();
uint16_t getValue(uint32_t *flow_value, uint8_t *raw_data, uint8_t nRawDataLen);

#endif



