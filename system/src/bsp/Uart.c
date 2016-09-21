/*
 * Driver for UART Communication between PC and Root Node
 *
 * It contains driver of USCIA0, as the part on PCB of UART communication.
 * USB(to PC) --- PL2302HX --- USCI-A0
 *
 * UART����˵��:
 * 1. ���ջ���������ΪUART_BUFFER_NUM��
 * 2. �Զ��������ݴ��� TODO ���ζ���(����TN)���������û�м�ʱ�������и��ǵ�������ڡ�
 *
 * TODO Ҫ�Ľ��� ����ͨ��ʹ�����жϣ�δʹ�÷��ж�
 * ���յĹ�����δ���κδ�����λ�����ж�
 *
 * @author Hua Cao
 *
 */

#include "Uart.h"

/*Ϊ�����Ʋ�����ӵ�*/
#ifndef NODE_END
  #ifdef CONFIG_UART_TO_CAN_BUF_SIZE
        #define UART_BUF_LENGTH  12
  #elif defined(CONFIG_TRAFFIC_DETECTION_APPLICATION)
        #define UART_BUF_LENGTH     16
  #else
	#define UART_BUF_LENGTH  20
  #endif
	#define UART_HEADER_LENGTH 2
	#define UART_FOOTER_LENGTH 2
	#define HEADER_HIGH_BYTE_INDEX 	0
	#define HEADER_LOW_BYTE_INDEX	1
	#define FOOTER_HIGH_BYTE_INDEX	(UART_BUF_LENGTH - 2)
	#define FOOTER_LOW_BYTE_INDEX		(UART_BUF_LENGTH -1)
#else
	#define UART_BUF_LENGTH  9
	#define UART_HEADER_LENGTH 2
	#define UART_FOOTER_LENGTH 2
	#define HEADER_HIGH_BYTE_INDEX 	0
	#define HEADER_LOW_BYTE_INDEX	1
	#define FOOTER_HIGH_BYTE_INDEX	7
	#define FOOTER_LOW_BYTE_INDEX		8
#endif

static const uint8_t HEADER_PATTERN[UART_HEADER_LENGTH] = { 0xAA, 0x55 };
static const uint8_t FOOTER_PATTERN[UART_FOOTER_LENGTH] = { 0x55, 0xAA };


static uint8_t rxBuf[UART_BUF_LENGTH];

/**
 * ��¼��ǰ���յ����ֽ���
 */
static uint8_t receivedBytesCounter = 0;

static uint8_t checkUartMsg();

/**
 * Initiate the UART.
 * 1. ��ʼ��UART�Ĵ�����ʹ�乤��
 * 2. ͨѶ�������ã�9600
 */
__monitor void initUart_BSP(void) {

	/* ����430�ܽţ�ѡ��USCI-A0��RX, TX�ܽŹ��� */
	P3SEL |= 0x30;

	/* ��׼����Ҫ��set SWRST�����˳���ȸ�����Ĵ�����ֵ */
	UCA0CTL1 |= UCSWRST;

	/**
	 * ����UCA0CTL0��Ĭ��ֵ
	 */
	UCA0CTL0 = 0;

	/**
	 * 7-6: 10 SMCLK(DCO)
	 * 5: characterУ��������Ȼ����
	 * �������ֶ�����
	 * 0: UCSWRST SW reset  0 disable, 1enable. ��ʱӦ��1.
	 */
	UCA0CTL1 = BV(7) | BV(5) | BV(0);

	/* �������ȱ��������ã���Ϊ���ǶԵ� */
        //baudrate:9600baud
	UCA0BR0 = 0x41;
	UCA0BR1 = 0x03;
	UCA0MCTL = UCBRS0;
#ifdef MAG_TEST
        //MCLK = SMCLK = DCOCLKDIV = default DCO/2 = 819200, Baudrate=115200, N = 819200/115200 = 7(0x0007)
        //baudrate:115200
        UCA0BR0 = 0x45;          
        UCA0BR1 = 0x00;
        //UCA0MCTL = 0x06;
#endif

	/* Clear SWRST, let USCI-A0 released for operation */
	UCA0CTL1 &= ~UCSWRST;

	IE2 &= ~UCA0TXIE;
	IE2 |= UCA0RXIE;
	IFG2 &= ~UCA0RXIFG;

	receivedBytesCounter = 0;
}

/**
 * �򴮿�д����
 * @param *buf ����ָ��
 * @param length Ҫд�����ݳ���
 *
 */
void writeToUart_BSP(const uint8_t* buf, uint8_t length) {
        
	/* ��ͷ */
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = HEADER_PATTERN[0];
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = HEADER_PATTERN[1];

	while (length--) {

		/* *
		 * Wait for TX buffer ready to receive new byte
		 * Ӳ�����ڻ�����������Ϻ��Զ��ñ��λ
		 */
		while (!(IFG2 & UCA0TXIFG))
			;

		/* д��һ��Ҫ���͵��� */
		UCA0TXBUF = *buf;

		/* ָ���¸�Ҫ���͵��� */
		buf++;
	}

	/* ��β */
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = FOOTER_PATTERN[0];
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = FOOTER_PATTERN[1];
}

static uint8_t checkUartMsg() {
	/* check header */
	if (rxBuf[HEADER_HIGH_BYTE_INDEX] == HEADER_PATTERN[0] && rxBuf[HEADER_LOW_BYTE_INDEX] == HEADER_PATTERN[1]) {
		/* check tail */
		if (rxBuf[FOOTER_HIGH_BYTE_INDEX] == FOOTER_PATTERN[0] && rxBuf[FOOTER_LOW_BYTE_INDEX] == FOOTER_PATTERN[1]) {
			return SUCCESS;
		}
	}
	return FAILURE;
}

/**
 * UART ���жϣ�����Ҫ��UCA0RXBUF�е�����ȡ�ߣ������µ���������ϵͳ������
 *
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void doInISR_UART(void) {

	/* У�� UCRXERR */
//	if ( UCA0STAT & 0x04 ) {
//		ASSERT_SGR(0,ERR_UART_RX);
//	}

	/* �Ȱ�Ӳ������������ȡ�� */
	rxBuf[receivedBytesCounter] = UCA0RXBUF;

	//TODO TN Apr24 ��δ���������͸�ˡ���ô���£����׷������˭���ĳ���ô���ذ��������κ�˵����
	if (receivedBytesCounter == UART_BUF_LENGTH - 1) {
		receivedBytesCounter = 0;
		if (checkUartMsg() != SUCCESS)
			return;
		executeAtUartISR_BSP(&(rxBuf[2]));
		LPM3_EXIT;
                
	} else {
		receivedBytesCounter++;
	}
}

