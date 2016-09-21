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
 * @author liusf
 *
 */

#include "TestStub.h"
#include <string.h>

/*����յ��������ݰ�*/
#define FLAG_RCV_PACKAGE				1
/*������ݰ����ͽ���*/
#define FLAG_SENT_PACKAGE				2
/*�ȴ�ʱ�䳬ʱ*/
#define FLAG_WAIT_TIME_DUE				3

static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static mrfiPacket_t g_receive_msg;

#define UART_MSG_LEN		        12
static uint8_t uartMsg[UART_MSG_LEN];

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	/* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != PACKET_LEN - 3) {
		delayInMs_BSP(2000);
                return;
	}

	memcpy(&g_receive_msg, packet, sizeof(g_receive_msg));
	PMS_SET(FLAG_RCV_PACKAGE);
}

void executeAtRFFinishedISR_MRFI()
{
}

void executeAtUartISR_BSP(const uint8_t *uart_msg){
}

void main(void) {
	initBSP_BSP();
        initUart_BSP();
	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_RX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
	showVersion_LED(ST_RX_VERSION);
	turnOnWAR_MRFI();
	ENABLE_INTERRUPTS_SGR();
        
        pmsFlags = 0;

	/* ���Թ��̻ᱻ�����հ����ѡ��հ�����Ȼ��ָ�WAR�������ϲ��Ԥ */
	while (1) 
        {
		LPM3;
                __no_operation();

		if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
                        twinkleLed_LED(LED2, 1, 100);    //���̵�
                        memset(uartMsg, 0, sizeof(uartMsg));
                        memcpy(uartMsg, &g_receive_msg.highAddr, UART_MSG_LEN - 1);
                        uartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
                        writeToUart_BSP(uartMsg, UART_MSG_LEN);
		}
	}
}
