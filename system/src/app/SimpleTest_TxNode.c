/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the main program for SimpleTest TX.
 *
 * PACKET:30BYTES����һ������ʱ49.5ms
 * ͨ��1000�κ�ʱ��1000 * 49.6 + (1000 / 10) * 200 = 69.6ms
 *
 * @author CaoHua
 *
 */

#include "SimpleTest_Node.h"

/**
 * how many times a single TX-with-CCA fail.
 */
static uint16_t ccaRetryTimes = 0;

/**
 * how many packets that be given up by too many CCA fails.
 */
static uint16_t ccaGiveupPacketsNumber = 0;

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	//��SimpleTest�У����Ͷ�û�н��հ��Ŀ����ԣ���˴˷����в���Ҫ�κδ���
}

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtRFFinishedISR_MRFI()
{
}

static void transmit() {
	mrfiPacket_t packet; //assign mem or not?

	packet.highAddr = ST_RX_NODE_HIGH_ADDR;
	packet.lowAddr = ST_NODE_LOW_ADDR;

	packet.frameLength = 2;
	packet.frame[0] = 0x01;
	packet.frame[0] = 0x02;

	uint8_t retryMaxForCCA = 5;
	while(transmitPacket_MRFI(&packet) == FAILURE) {
		retryMaxForCCA--;
		ccaRetryTimes++;

		startTimerAsDelaying_TimerB(10);
		LPM3;

		if (retryMaxForCCA == 0) {
			ccaGiveupPacketsNumber++;

			/*transmit failed and wait 10ms to transmit next packet*/
			startTimerAsDelaying_TimerB(10);
			break;
		}
	}

	/* Wait the transmit finish ISR to wake up */
	LPM3;
}

void showFailedResult(uint16_t result) {
	turnOnLed_LED(LED2);
	startTimerAsDelaying_TimerB(200);
	LPM3;

	while (result > 9) {
		turnOnLed_LED(LED1);
		startTimerAsDelaying_TimerB(1000);
		LPM3;

		turnOffLed_LED(LED1);
		startTimerAsDelaying_TimerB(200);
		LPM3;

		result -= 10;
	}

	while (result) {
		turnOnLed_LED(LED1);
		startTimerAsDelaying_TimerB(200);
		LPM3;

		turnOffLed_LED(LED1);
		startTimerAsDelaying_TimerB(200);
		LPM3;

		result--;
	}

	turnOffLeds_LED();
}

void executeAtUpdateMagValue() 
{
}

void main() {
	initBSP_BSP();
	DISABLE_INTERRUPTS_SGR();

	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_TX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);

	showVersion_LED(3);
	ENABLE_INTERRUPTS_SGR();

	uint16_t counter = 0;

//	while (counter < STAT_TX_TIMES * 10) {
	while(1){
		counter++;
		transmit();
//		if(counter %3 == 0){
//			changeChannelNum_MRFI(1);
//		} else if(counter %3 == 1){
//			changeChannelNum_MRFI(3);
//		} else {
//			changeChannelNum_MRFI(7);
//		}
//		turnOnWAR_MRFI();
//		startTimerAsDelaying_TimerB(50);
//		LPM3;
		twinkleLed_LED(LED2, 1, 100);
	}

	/**
	 *  CCA������棺�ȱ� packets���ٱ� times
	 *  ѭ����
	 *  ���ֱ�����1s��ÿ���ڼ��3s
	 *
	 *  ������Rx��ͬ��ʮλ��λ������ʽ��
	 *
	 */
	while (1) {

		/*to save power*/
		turnOffRadio_MRFI();

		if (ccaGiveupPacketsNumber == 0) {
			twinkleLed_LED(LED1, 1, 1000);
		} else {
			showFailedResult(ccaGiveupPacketsNumber);
		}

		startTimerAsDelaying_TimerB(1000);
		LPM3;

		if (ccaRetryTimes == 0) {
			twinkleLed_LED(LED1, 1, 1000);
		} else {

			showFailedResult(ccaRetryTimes);
		}

		turnOffLeds_LED();
		startTimerAsDelaying_TimerB(3000);
		LPM3;
	}
}
