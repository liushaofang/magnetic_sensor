/*
 * TestStub is a tool for test the mrfi layer.
 * This is the main program for SimpleTest TX.
 *
 * PACKET:30BYTES����һ������ʱ49.5ms
 * ͨ��1000�κ�ʱ��1000 * 49.6 + (1000 / 10) * 200 = 69.6ms
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
/*ѭ�����͵�����ֵ*/   
#define TRANS_DATA_LOOP_COUNT    2
/*���ݷ�������*/   
#define TRANSMIT_TIME_PERIOD     1000
   
static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static mrfiPacket_t g_transmit_msg;
static uint8_t g_btTransData = 0;
static uint32_t g_nTotalVehicleCount = 0;

static void forcedTransPacket(mrfiPacket_t* msg);

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	//��SimpleTest�У����Ͷ�û�н��հ��Ŀ����ԣ���˴˷����в���Ҫ�κδ���
}

void executeAtTransmitPacketISR_MRFI() {
	//no action required for SimpleTest.
        PMS_SET(FLAG_SENT_PACKAGE);
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtRFFinishedISR_MRFI()
{
}

void executeAtTimeUp_TimerA() {
        PMS_SET(FLAG_WAIT_TIME_DUE);
}

void main() {
	initBSP_BSP();

	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_TX_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
        //init_MRFI(TXPOWER, 0x02, 0x03, 0x03);

	showVersion_LED(TEST_STUB_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
        memset(&g_transmit_msg, 0x0, sizeof(mrfiPacket_t));
	g_transmit_msg.frameLength = PACKET_LEN - 3;
        g_transmit_msg.highAddr = ST_RX_NODE_HIGH_ADDR;
        g_transmit_msg.lowAddr = ST_NODE_LOW_ADDR;
        g_transmit_msg.frame[0] = ST_TX_NODE_HIGH_ADDR;
	g_transmit_msg.frame[1] = ST_NODE_LOW_ADDR;
        
        pmsFlags = 0;
        g_nTotalVehicleCount = 0;
        
        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
        
	while(1)
        {
                LPM3;
		__no_operation();
                
                if(PMS_IS_SET(FLAG_WAIT_TIME_DUE))
                {
                      PMS_CLEAR(FLAG_WAIT_TIME_DUE);
                      g_nTotalVehicleCount += 1;
                      g_transmit_msg.frame[2] = TEST_MRFI;
                      g_transmit_msg.frame[3] = ((g_btTransData++) % TRANS_DATA_LOOP_COUNT);
                      /*    ���ӽ�ͨ����������   */
                      g_transmit_msg.frame[5] = (uint8_t)(g_nTotalVehicleCount & 0xFF);
                      g_transmit_msg.frame[6] = (uint8_t)((g_nTotalVehicleCount >> 8) & 0xFF);
                      g_transmit_msg.frame[7] = (uint8_t)((g_nTotalVehicleCount >> 16) & 0xFF);
                      g_transmit_msg.frame[8] = (uint8_t)((g_nTotalVehicleCount >> 24) & 0xFF);
                      //transmitPacket_MRFI(&g_transmit_msg);   //���Ͳ�����Ƶ����
                      forcedTransPacket(&g_transmit_msg);   //���Ͳ�����Ƶ����
                }
                
                if(PMS_IS_SET(FLAG_SENT_PACKAGE)) 
                {
			PMS_CLEAR(FLAG_SENT_PACKAGE);
                        twinkleLed_LED(LED1, 1, 100);    //�����
                        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
                }
	}
}

/**
 * @brief ǿ�Ʒ������ݰ������͹������������ʧ�ܣ�����ʱ50ms�ٷ���ֱ���������Ϊֹ
 * @param msg Ҫ���͵����ݰ�
 */
static void forcedTransPacket(mrfiPacket_t *msg) {
	uint8_t status = 0;
	startTimerAsDelaying_TimerB(5);
	LPM3;

	do{
		status = transmitPacket_MRFI(msg);

		/*if transmit failed, delay 50ms, retry  to transmit*/
		if(status == FAILURE) {
			/*turn off RF in order to save power*/
			turnOffRadio_MRFI();

			/*delay 50ms*/
			startTimerAsDelaying_TimerB(100);
			LPM3;
		}
	} while (status == FAILURE);
}