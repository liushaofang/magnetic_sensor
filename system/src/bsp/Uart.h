/*
 * Driver for UART Communication between PC and Root Node
 *
 * @author CaoHua
 *
 */

#ifndef __UART_H__
#define __UART_H__

#include "SGR_CONCEPT.h"
#include "Led.h"

/**
 * 首先将USCI-A0 置为SW RST，
 * 按标准流程、按SGR要求，置各个寄存器后，
 * 置 SWRST=0，让USCI-A0开始工作。并激活RX中断。
 *
 * Tips: 控制全局中断是调用者的责任
 */
__monitor void initUart_BSP(void);

void writeToUart_BSP(const uint8_t *, uint8_t);
void executeAtUartISR_BSP(const uint8_t *);

#endif
