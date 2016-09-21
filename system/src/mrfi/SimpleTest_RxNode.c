/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the main program for SimpleTest RX.
 *
 * 多TX 到 1RX 的设计，采用瘦中断方式
 * TXNUM 用于指定 TX的个数，默认为1
 *
 * 测试的结束采用timerB延时判断
 * 通过闪灯显示丢包数，没有丢包LED1循环长亮
 *
 * @author CaoHua
 *
 */

#include "SimpleTest_Node.h"
#include <string.h>

/////////////////////////////////////////////////////////////
/* 不同测试模式的定义区  */

/**
 * 用于测试漏电电流的模式。
 * 激活此模式时，亮版本灯后，430和1101直接进入低功耗模式
 * 0: 关闭
 * 1: 打开
 */
#define LEAK_CURRENT_MODE 0

#define TERMINATE_IDLE_TIME_MS 500

/* END */
/////////////////////////////////////////////////////////////



#define FLAG_PACKET_RECEIVED 1
#define FLAG_TERMINATED 2

static uint16_t sgriFlags = 0;

#define SGRI_IS_SET(x)  (sgriFlags & (0x01 << x))
#define SGRI_CLEAR(x)  (sgriFlags &= ~(0x01 << x))
#define SGRI_SET(x)    (sgriFlags |= (0x01 << x))

/* end */
/////////////////////////////////////////////////////////////


static uint16_t receivedPacketCounter = 0;
static mrfiPacket_t msg;

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	/* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != 2) {
		twinkleLed_LED(LED2, 2, 200);
		delayInMs_BSP(2000);
	}

	memcpy(&msg, packet, sizeof(msg));
	SGRI_SET(FLAG_PACKET_RECEIVED);
}

/*
static void terminateTheTest() {
	SGRI_SET(FLAG_TERMINATED);
}
*/

void executeAtRFFinishedISR_MRFI()
{
}

void main(void) {
	initBSP_BSP();
	DISABLE_INTERRUPTS_SGR();

	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_RX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
	showVersion_LED(ST_RX_VERSION);

	if (LEAK_CURRENT_MODE) {
		turnOffRadio_MRFI();
		DISABLE_INTERRUPTS_SGR();
		LPM3;
	}

	turnOnWAR_MRFI();
	ENABLE_INTERRUPTS_SGR();

	/* 测试过程会被各种收包叫醒。收包后自然会恢复WAR，不用上层干预 */
	while (1) {
		LPM3;

		if (SGRI_IS_SET(FLAG_PACKET_RECEIVED)) {
			SGRI_CLEAR(FLAG_PACKET_RECEIVED);

			twinkleLed_LED(LED3, 1, 100);
//			stopTimer_TimerB();
//			receivedPacketCounter++;
//
//			startTimer_TimerB(TERMINATE_IDLE_TIME_MS, terminateTheTest);
		}

		/* 测试结束，报告结果 */
		if(SGRI_IS_SET(FLAG_TERMINATED)) {
			SGRI_CLEAR(FLAG_TERMINATED);

			turnOffRadio_MRFI();
			uint16_t lostCounter = STAT_TX_TIMES * 10 * TX_NUM - receivedPacketCounter;

			if (lostCounter == 0) {
				while (1) {
					turnOffLeds_LED();
					startTimerAsDelaying_TimerB(2000);
					LPM3;

					turnOnLed_LED(LED1);
					startTimerAsDelaying_TimerB(1000);
					LPM3;
				}
			} else {
				while (1) {
					uint16_t lostCountTemp = lostCounter;

					turnOffLeds_LED();
					startTimerAsDelaying_TimerB(2000);
					LPM3;

					turnOnLed_LED(LED2);
					startTimerAsDelaying_TimerB(200);
					LPM3;

					while(lostCountTemp >9) {
						turnOnLed_LED(LED1);
						startTimerAsDelaying_TimerB(1000);
						LPM3;

						turnOffLed_LED(LED1);
						startTimerAsDelaying_TimerB(200);
						LPM3;

						lostCountTemp -= 10;
					}

					while(lostCountTemp) {
						turnOnLed_LED(LED1);
						startTimerAsDelaying_TimerB(200);
						LPM3;

						turnOffLed_LED(LED1);
						startTimerAsDelaying_TimerB(200);
						LPM3;

						lostCountTemp--;
					}

					startTimerAsDelaying_TimerB(200);
					LPM3;
				}
			}
		}
	}
}
