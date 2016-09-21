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

#include <string.h>
#include "MassProduction.h"

/*����յ��������ݰ�*/
#define FLAG_RCV_PACKAGE								1
/*������ݰ����ͽ���*/
#define FLAG_SENT_PACKAGE							2
/*�ȴ�ACK��ʱ*/
#define FLAG_WAIT_ACK_OVERTIME				3
/*��ǵشŴ������ɼ���������*/
#define FLAG_GEOMAGNETIC_DATA_PREPARED		4
/*���End�ڵ����CMD��ʱ��Ƭ����*/
#define FLAG_WAIT_CMD_TIMER_DUE				5
/*���End�ڵ㴦��Sleep��ʱ��Ƭ����*/
#define FLAG_SLEEP_TIMER_DUE						6
/*�����Ҫ����������     */
#define FLAG_HEARTBEAT_TIMER_DUE					7

#define WAIT_ACK_TIME_DELAY                     4000

#define TRANSMIT_TIME_PERIOD                    1000

#define SINGLE_TRANSMIT_COUNT_MAX               20

static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static mrfiPacket_t g_transmit_msg, g_receive_msg;

static uint8_t g_bIsMagnetSensorTestNeed = MASS_PRODUCTION_NOT_NEED_MAGNET_TEST;
#define SEND_DATA_LOOP_COUNT    10
static uint8_t g_btLoopData = 0;

static void parseReceivedMsg();
#if 1
static void forcedTransPacket(mrfiPacket_t* msg);
#endif

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	//��SimpleTest�У����Ͷ�û�н��հ��Ŀ����ԣ���˴˷����в���Ҫ�κδ���
        /* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != PACKET_LEN - 3) {
          //2013.12.26: liusf comment it off for led twinkle debug
		//twinkleLed_LED(LED2, 2, 200); 
		delayInMs_BSP(2000);
	}

	memcpy(&g_receive_msg, packet, sizeof(g_receive_msg));
	PMS_SET(FLAG_RCV_PACKAGE);
}

void executeAtTransmitPacketISR_MRFI() {
	PMS_SET(FLAG_SENT_PACKAGE);
}

void executeAtReceiveSyncISR_MRFI() {
	//no action required for SimpleTest.
}

void executeAtRFFinishedISR_MRFI()
{
}

void executeAtTimeUp_TimerB() {
        PMS_SET(FLAG_WAIT_ACK_OVERTIME);
}

void showFailedResult(uint16_t result) {
	//turnOnLed_LED(LED2);
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

void show_test_mrfi_fail()
{
        twinkleLed_LED(LED1, 1, 50);    //�����
}

void show_test_magnet_fail()
{
        twinkleLed_LED(LED3, 1, 50);    //������
}

void main() {
	initBSP_BSP();
	DISABLE_INTERRUPTS_SGR();
        
	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_MASTER_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
        
	showVersion_LED(3);
	ENABLE_INTERRUPTS_SGR();
        
        memset(&g_transmit_msg, 0x0, sizeof(mrfiPacket_t));
	g_transmit_msg.frameLength = PACKET_LEN - 3;
        g_transmit_msg.highAddr = ST_SLAVE_NODE_HIGH_ADDR;
        g_transmit_msg.lowAddr = ST_NODE_LOW_ADDR;
        g_transmit_msg.frame[0] = ST_MASTER_NODE_HIGH_ADDR;
	g_transmit_msg.frame[1] = ST_NODE_LOW_ADDR;
        
        pmsFlags = 0;
        
        //g_bIsMagnetSensorTestNeed = MASS_PRODUCTION_NOT_NEED_MAGNET_TEST;
        
        while(1)
        {
                //turnOffLeds_LED();
                
                g_transmit_msg.frame[2] = MASS_PRODUCTION_TEST_MRFI;
                g_transmit_msg.frame[3] = ((g_btLoopData++) % SEND_DATA_LOOP_COUNT);
                //transmitPacket_MRFI(&g_transmit_msg);   //���Ͳ�����Ƶ����
                forcedTransPacket(&g_transmit_msg);   //���Ͳ�����Ƶ����
                
                LPM3;
                __no_operation();
                
                if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
			PMS_CLEAR(FLAG_SENT_PACKAGE);
                        //twinkleLed_LED(LED2, 1, 50);    //���̵�
                        twinkleLed_LED(LED1, 1, 100);    //�����
                        delayInMs_BSP(50);
                        /*DISABLE_INTERRUPTS_SGR();
                        init_MRFI(TXPOWER, CHANNEL, ST_MASTER_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
                        turnOnRX_MRFI();
                        ENABLE_INTERRUPTS_SGR();*/
                        //startTimer_TimerB(WAIT_ACK_TIME_DELAY ,executeAtTimeUp_TimerB);
                        startTimer_TimerB(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerB);
                        LPM3;
                }
                
#if 0
                if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
			stopTimer_TimerB();
			parseReceivedMsg();
                        twinkleLed_LED(LED3, 1, 1000);    //������
		}
                
                if(PMS_IS_SET(FLAG_WAIT_ACK_OVERTIME)){         //��Ƶ����ʧ��
			PMS_CLEAR(FLAG_WAIT_ACK_OVERTIME);
                        show_test_mrfi_fail();
                        continue;
		}
                
                if(g_bIsMagnetSensorTestNeed == MASS_PRODUCTION_NEED_MAGNET_TEST)
                {
                        g_transmit_msg.frame[3] = MASS_PRODUCTION_TEST_MAGNET;
                        transmitPacket_MRFI(&g_transmit_msg);     //���Ͳ�����Ƶ����
                        
                        LPM3;
                        __no_operation();
                        
                        if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
                                PMS_CLEAR(FLAG_SENT_PACKAGE);
                                //twinkleLed_LED(LED2, 1, 50);    //���̵�
                                delayInMs_BSP(50);
                                
                                DISABLE_INTERRUPTS_SGR();
                                init_MRFI(TXPOWER, CHANNEL, ST_MASTER_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
                                turnOnRX_MRFI();
                                ENABLE_INTERRUPTS_SGR();
                                startTimer_TimerB(WAIT_ACK_TIME_DELAY ,executeAtTimeUp_TimerB);
                                LPM3;
                        }
                        
                        if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
                                PMS_CLEAR(FLAG_RCV_PACKAGE);
                                stopTimer_TimerB();
                                parseReceivedMsg();
                                twinkleLed_LED(LED3, 1, 1000);    //������
                        }
                        
                        if(PMS_IS_SET(FLAG_WAIT_ACK_OVERTIME)){
                                PMS_CLEAR(FLAG_WAIT_ACK_OVERTIME);
                                //show_test_magnet_fail();
                                show_test_mrfi_fail();
                                continue;
                        }
                }
#endif
        }
}

#if 1
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
#endif

/**
 * @brief �����յ������ݰ�
 */
static void parseReceivedMsg() {
	uint8_t operation_type = g_receive_msg.frame[2];
	switch(operation_type) {
	case MASS_PRODUCTION_TEST_MRFI_ACK:
              if(g_receive_msg.frame[3] == MASS_PRODUCTION_NEED_MAGNET_TEST)  //��ʾ��Ҫ���ԵشŴ�����
              {
                    g_bIsMagnetSensorTestNeed = MASS_PRODUCTION_NEED_MAGNET_TEST;
              }
              else if(g_receive_msg.frame[3] == MASS_PRODUCTION_NOT_NEED_MAGNET_TEST)  //��ʾ����Ҫ���ԵشŴ�����
              {
                    g_bIsMagnetSensorTestNeed = MASS_PRODUCTION_NOT_NEED_MAGNET_TEST;
              }
              break;
        case MASS_PRODUCTION_TEST_MAGNET_ACK:
              if(g_receive_msg.frame[3] == MASS_PRODUCTION_TEST_FAIL)        //�شŴ���������ʧ��
              {
                    show_test_magnet_fail();
              }
              break;
	default:
              break;
	}
}