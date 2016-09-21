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
 * ���Ƚ�USCI-A0 ��ΪSW RST��
 * ����׼���̡���SGRҪ���ø����Ĵ�����
 * �� SWRST=0����USCI-A0��ʼ������������RX�жϡ�
 *
 * Tips: ����ȫ���ж��ǵ����ߵ�����
 */
__monitor void initUart_BSP(void);

void writeToUart_BSP(const uint8_t *, uint8_t);
void executeAtUartISR_BSP(const uint8_t *);

#endif
