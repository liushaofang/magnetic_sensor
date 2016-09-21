/*
 * MCU Voltage Detection driver
 *
 * use the postfix _BSP
 *
 * @author Taurus Ning
 *
 */

#ifndef __VOLTAGE_H__
#define __VOLTAGE_H__

#include "SGR_CONCEPT.h"
void initPower_BSP(void);
uint16_t getVoltage_BSP(void);

#endif
