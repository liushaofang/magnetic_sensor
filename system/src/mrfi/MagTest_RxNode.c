/*
 * MagTest is a tool for test mag3110 driver, MagTest_Tx reads data from sensor and then transmits all raw data to MagTest_RX.
 * This is the main program for MagTest RX.
 *
 * 程序执行后进入Rx模式
 * 接收来自MagTest Tx发送的包， 然后通过串口传出去
 *
 * @author CaoHua
 *
 */

#include "MagTest_Node.h"
#include <string.h>

#define FLAG_PACKET_RECEIVED 1
#define FLAG_TEST_UART 2

static uint16_t sgriFlags = 0;

#define SGRI_IS_SET(x)  (sgriFlags & (0x01 << x))
#define SGRI_CLEAR(x)  (sgriFlags &= ~(0x01 << x))
#define SGRI_SET(x)    (sgriFlags |= (0x01 << x))

/* end */
/////////////////////////////////////////////////////////////

static mrfiPacket_t msg;

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtUpdateMagValue() {

}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	/* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != PACKET_LEN -3) {
		twinkleLed_LED(LED1, 2, 200);
		delayInMs_BSP(2000);
	}

	memcpy(&msg, packet, sizeof(msg));
	SGRI_SET(FLAG_PACKET_RECEIVED);
}

void executeAtUartISR_BSP(const uint8_t *uart_msg){
}

void executeAtTimeUp_TimerB() {
		SGRI_SET(FLAG_TEST_UART);
}

void executeAtRFFinishedISR_MRFI()
{
}

void main(void) {
	initBSP_BSP();
	DISABLE_INTERRUPTS_SGR();

	initLeds_LED();
	initUart_BSP();
	init_MRFI(TXPOWER, CHANNEL, ST_RX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
	showVersion_LED(ST_RX_VERSION);

	turnOnRX_MRFI();
	ENABLE_INTERRUPTS_SGR();

	/* 测试过程会被各种收包叫醒。收包后自然会恢复WAR，不用上层干预 */
//	startTimer_TimerB(10, executeAtTimeUp_TimerB);
	uint8_t cnt = 0;
	while (1) {
		LPM3;

		if (SGRI_IS_SET(FLAG_PACKET_RECEIVED)) {
			SGRI_CLEAR(FLAG_PACKET_RECEIVED);
			cnt++;
			msg.frame[9] = cnt;
			DISABLE_INTERRUPTS_SGR();
			writeToUart_BSP(&msg.frame[3], PACKET_LEN - 4);
			ENABLE_INTERRUPTS_SGR();
		}

		if (SGRI_IS_SET(FLAG_TEST_UART)) {
			SGRI_CLEAR(FLAG_TEST_UART);
			cnt++;
			msg.frame[3] = cnt;
			writeToUart_BSP(&msg.frame[3], PACKET_LEN - 4);
			startTimer_TimerB(10, executeAtTimeUp_TimerB);
		}
	}
}
