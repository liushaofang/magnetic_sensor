/*
 * Basic definition and functions in BSP Layer.
 *
 * Notice:
 * 1. The delay series function are CPU blocked, means high currency consumption.
 *    BE CAUTIOUS WITH THEM!
 * 2. The delay series function are not interrupt-blocked.  it's the caller's duty
 *    to decide if the context of usage should enable/disable interrupt.
 *
 * DON'T MODIFY THIS FILE.
 *
 * @author CaoHua
 *
 */

#ifndef __BSP_DEF_H__
#define __BSP_DEF_H__

#include "SGR_CONCEPT.h"

/**
 * Macro function, delay in us depends MCU CPU beats.
 *
 * 使用IAR内置延时函数__delay_cycles()，通过汇编，比C while -- 精准。
 * void __delay_cycles(uint32_t cycles);
 * Inserts assembler instructions that delays execution
 * the number of clock cycles specified.
 *
 * @param x delay的秒数，必须是 uint32_t 或 字面整数.  Best <1000
 */
#define DELAY_US_BSP(x) __delay_cycles((uint32_t)(((double)8000000)*(double)x/1000000.0))


/**
 * initiate BSP layer core driver.
 * should be called at the beginning of all main program.
 */
void initBSP_BSP(void);

/**
 * delay in milliseconds.
 * @param msCount how many milliseconds to delay
 */
void delayInMs_BSP(uint16_t msCount);

/**
 * delay in seconds.
 * @param secondCount how many seconds to delay
 */
void delayInSecond_BSP(uint16_t secondCount);

#endif
