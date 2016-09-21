/*
 * Driver for UART Communication between PC and Root Node
 *
 * It contains driver of USCIA0, as the part on PCB of UART communication.
 * USB(to PC) --- PL2302HX --- USCI-A0
 *
 * UART驱动说明:
 * 1. 接收缓冲区长度为UART_BUFFER_NUM。
 * 2. 自动接收数据存入 TODO 环形队列(不信TN)，如果数据没有及时读出，有覆盖的问题存在。
 *
 * TODO 要改进： 串口通信使用收中断，未使用发中断
 * 在收的过程中未对任何错误标记位进行判断
 *
 * @author Hua Cao
 *
 */

#include "Uart.h"

/*为电量计测试添加的*/
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
 * 记录当前接收到的字节数
 */
static uint8_t receivedBytesCounter = 0;

static uint8_t checkUartMsg();

/**
 * Initiate the UART.
 * 1. 初始化UART寄存器，使其工作
 * 2. 通讯速率设置：9600
 */
__monitor void initUart_BSP(void) {

	/* 设置430管脚：选中USCI-A0的RX, TX管脚功能 */
	P3SEL |= 0x30;

	/* 标准流程要先set SWRST，因此顺便先给这个寄存器赋值 */
	UCA0CTL1 |= UCSWRST;

	/**
	 * 采用UCA0CTL0的默认值
	 */
	UCA0CTL0 = 0;

	/**
	 * 7-6: 10 SMCLK(DCO)
	 * 5: character校验出错后仍然接收
	 * 其它各种都不开
	 * 0: UCSWRST SW reset  0 disable, 1enable. 此时应设1.
	 */
	UCA0CTL1 = BV(7) | BV(5) | BV(0);

	/* 这三个先保留旧设置，认为它是对的 */
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
 * 向串口写数据
 * @param *buf 数据指针
 * @param length 要写的数据长度
 *
 */
void writeToUart_BSP(const uint8_t* buf, uint8_t length) {
        
	/* 包头 */
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = HEADER_PATTERN[0];
	while (!(IFG2 & UCA0TXIFG))
		;
	UCA0TXBUF = HEADER_PATTERN[1];

	while (length--) {

		/* *
		 * Wait for TX buffer ready to receive new byte
		 * 硬件会在缓冲区发送完毕后自动置标记位
		 */
		while (!(IFG2 & UCA0TXIFG))
			;

		/* 写入一个要发送的数 */
		UCA0TXBUF = *buf;

		/* 指向下个要发送的数 */
		buf++;
	}

	/* 包尾 */
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
 * UART 收中断，必须要把UCA0RXBUF中的内容取走，否则新的内容来，系统会死机
 *
 */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void doInISR_UART(void) {

	/* 校验 UCRXERR */
//	if ( UCA0STAT & 0x04 ) {
//		ASSERT_SGR(0,ERR_UART_RX);
//	}

	/* 先把硬件缓冲区的数取出 */
	rxBuf[receivedBytesCounter] = UCA0RXBUF;

	//TODO TN Apr24 这段代码质量烂透了。怎么回事，务必追出来。谁给改成这么天昏地暗还不加任何说明的
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

