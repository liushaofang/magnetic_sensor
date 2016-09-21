/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the main program for SimpleTest RX.
 *
 * ��TX �� 1RX ����ƣ��������жϷ�ʽ
 * TXNUM ����ָ�� TX�ĸ�����Ĭ��Ϊ1
 *
 * ���ԵĽ�������timerB��ʱ�ж�
 * ͨ��������ʾ��������û�ж���LED1ѭ������
 *
 * @author CaoHua
 *
 */

#include "SimpleTest_Node.h"
#include <string.h>

/////////////////////////////////////////////////////////////
/* ��ͬ����ģʽ�Ķ�����  */

/**
 * ���ڲ���©�������ģʽ��
 * �����ģʽʱ�����汾�ƺ�430��1101ֱ�ӽ���͹���ģʽ
 * 0: �ر�
 * 1: ��
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

	/* ���Թ��̻ᱻ�����հ����ѡ��հ�����Ȼ��ָ�WAR�������ϲ��Ԥ */
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

		/* ���Խ����������� */
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
