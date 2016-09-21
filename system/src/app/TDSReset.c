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

#include <string.h>
#include "PMSUtil.h"
#include "Uart.h"
#include "Led.h"

#define FLAG_RCV_PACKAGE					1
#define FLAG_SENT_PACKAGE					2
#define FLAG_RCV_UART_DATA                                      3
#define FLAG_GOT_ACK_OVERTIME				        4
/*�ȴ�ʱ�䳬ʱ*/
#define FLAG_WAIT_TIME_DUE				        5
/*���ݷ�������*/   
#define TRANSMIT_TIME_PERIOD                                    1000

/*      ����ת�Ӱ嵽�����������������������������С       */
#define UART_MSG_LEN		16

static uint16_t tmsFlags = 0;
#define TMS_IS_SET(x)  (tmsFlags & (0x01 << x))
#define TMS_CLEAR(x)  (tmsFlags &= ~(0x01 << x))
#define TMS_SET(x)    (tmsFlags |= (0x01 << x))

#define RESET_COMM_CHANNEL              0x02

#define ACK_WAITING_MS		50
#define DATA_WAITING_MS		50

#define MAX_RESEND_TIMES                        150

#define FORCE_TRANS_LIMIT               100
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static mrfiPacket_t g_transmit_msg, g_receive_msg;

static uint8_t uartMsg[UART_MSG_LEN];

#define RESET_NODE_HIGH_ADDR            0x09
#define RESET_NODE_LOW_ADDR             0x09
#define SENSOR_NODE_START_HIGH_ADDR     0x00
#define SENSOR_NODE_COUNT               0x03

#define SENSOR_NODE_LOW_ADDR            0x03

static uint8_t g_btHighAddr = 0;

void process_uart_cmd_packet();

/**
 * address of parent node.
 */
static addr_t nodeAddr = {CONFIG_ADDR };
const addr_t broadcastAddr = {BROADCAST_ADDR};
static uint16_t resend_times = 0;
static uint8_t cur_channel = CHANNEL;

static void forcedTransPacket(mrfiPacket_t* msg);

void executeAtTransmitPacketISR_MRFI() {
	TMS_SET(FLAG_SENT_PACKAGE);
}

void executeAtReceiveSyncISR_MRFI() {
}

void executeAtRFFinishedISR_MRFI() {
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	/* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != PACKET_LEN - 3) {
		twinkleLed_LED(LED2, 2, 200);
		delayInMs_BSP(2000);
	}

	memcpy(&g_receive_msg, packet, sizeof(g_receive_msg));
	TMS_SET(FLAG_RCV_PACKAGE);
}

void executeAtTimeUp_TimerB() {
        TMS_SET(FLAG_GOT_ACK_OVERTIME);
}

void executeAtTimeUp_TimerA() {
        TMS_SET(FLAG_WAIT_TIME_DUE);
}

/*void executeAtUartISR_BSP(const uint8_t *uart_msg){
	  memcpy(uartMsg, uart_msg, UART_MSG_LEN);
	  TMS_SET(FLAG_RCV_UART_DATA);
}*/

void main() {
	/* Board Init */
	initBSP_BSP();
	//initUart_BSP();
	initLeds_LED();
	init_MRFI(TXPOWER, RESET_COMM_CHANNEL, nodeAddr.addr[0], nodeAddr.addr[1]);
        
	showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
	g_transmit_msg.frameLength = PACKET_LEN -3;
	turnOnWAR_MRFI();
        
        g_btHighAddr = 0x0;
        g_transmit_msg.lowAddr = SENSOR_NODE_LOW_ADDR;
        g_transmit_msg.frame[0] = RESET_NODE_HIGH_ADDR;
        g_transmit_msg.frame[1] = RESET_NODE_LOW_ADDR;
        g_transmit_msg.frame[2] = OP_RESET_MAG_PARAM;
        
        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
                      
	while(1) {
		LPM3;
		__no_operation();
                
		//���յ������ֽڵĴ������ݣ���Ҫ�Խ��յ��Ĵ������ݽ��д���
		/*if(TMS_IS_SET(FLAG_RCV_UART_DATA)) {
                  TMS_CLEAR(FLAG_RCV_UART_DATA);
                  twinkleLed_LED(LED3, 1, 100);
                  process_uart_cmd_packet();
		}*/
                
                if(TMS_IS_SET(FLAG_WAIT_TIME_DUE))
                {
                      TMS_CLEAR(FLAG_WAIT_TIME_DUE);
                      
                      g_btHighAddr = g_btHighAddr % SENSOR_NODE_COUNT;
                      g_transmit_msg.highAddr = g_btHighAddr + 1;
                      g_btHighAddr += 1;
                      resend_times = MAX_RESEND_TIMES;
                      forcedTransPacket(&g_transmit_msg);
                      startTimerAsDelaying_TimerB(100);
                }

		//����������ͳɹ�����Ҫ�ȴ���������Э�������м�����������������Ӧ��
		if(TMS_IS_SET(FLAG_SENT_PACKAGE)){
				TMS_CLEAR(FLAG_SENT_PACKAGE);
                                
                                //twinkleLed_LED(LED3, 1, 10);    //������
                                stopTimer_TimerA();
                                
				startTimer_TimerB(ACK_WAITING_MS, executeAtTimeUp_TimerB);
				resend_times--;
				turnOnWAR_MRFI();
		}
                else
                {
                                startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
                }
                
                if(TMS_IS_SET(FLAG_RCV_PACKAGE)){
			TMS_CLEAR(FLAG_RCV_PACKAGE);
                        
                        twinkleLed_LED(LED1, 1, 100);    //�����
                        
			stopTimer_TimerB();
			resend_times = 0;
                        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
			//memcpy(uartMsg, &receiveMsg.highAddr, UART_MSG_LEN -1);
                        //2014.03.25:   liusf modified for uploading RSSI and LQI data,Ŀǰ���ϴ�RSSIֵ
                        //uartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(receiveMsg.rxMetrics[0]));
                        //2014.03.25:   liusf modified end
			//writeToUart_BSP(uartMsg, UART_MSG_LEN);
		}

		//����Ӧ�����ݳ�ʱ����Ҫͨ�����ڷ���ʧ����Ϣ
		if(TMS_IS_SET(FLAG_GOT_ACK_OVERTIME)) {
				TMS_CLEAR(FLAG_GOT_ACK_OVERTIME);
				if(resend_times > 0){
					forcedTransPacket(&g_transmit_msg);
				} 
                                else 
                                {
					//uartMsg[0] = 0x0 << 7 | 0x10;   //��ʾ������ƵӦ������ʧ��
					//writeToUart_BSP(uartMsg, UART_MSG_LEN);
                                        stopTimer_TimerB();
                                        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
                                        twinkleLed_LED(LED2, 1, 100);    //���Ƶ�
				}
		}
                
        }
}

/**
 * @brief �����Ӵ����յ����������ݰ�
 */
/*void process_uart_cmd_packet() {
		uint8_t index = 0;

		if(uartMsg[4] == OP_CHANGE_CONFIG_DEV_CHAN){
			cur_channel = uartMsg[5];
			changeChannelNum_MRFI(cur_channel);
			uartMsg[4] = OP_ACK_CHANGE_CONFIG_DEV_CHAN;
			writeToUart_BSP(uartMsg, UART_MSG_LEN);
			return;
		}
                
		g_transmit_msg.highAddr = uartMsg[0];
		g_transmit_msg.lowAddr = uartMsg[1];
		g_transmit_msg.frame[0] = nodeAddr.addr[0];
		g_transmit_msg.frame[1] = nodeAddr.addr[1];
		for(index = 2; index < PACKET_LEN - 3; index++) {
		  g_transmit_msg.frame[index] = uartMsg[index + 2];
		}
		resend_times = ((uint16_t)uartMsg[UART_MSG_LEN - 2] << 8)  + uartMsg[UART_MSG_LEN -1];
                if(resend_times > MAX_RESEND_TIMES)
                  resend_times = MAX_RESEND_TIMES;
		forcedTransPacket(&g_transmit_msg);
}*/

/**
 * @brief ǿ�Ʒ������ݰ������͹������������ʧ�ܣ�����ʱ50ms�ٷ���ֱ���������Ϊֹ
 * @param msg Ҫ���͵����ݰ�
 */
void forcedTransPacket(mrfiPacket_t *msg) {
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
			startTimerAsDelaying_TimerB(50);
			LPM3;
		}
                
                g_nForceTransLimit--;
	} while (g_nForceTransLimit > 0 && status == FAILURE);
        
        g_nForceTransLimit = FORCE_TRANS_LIMIT;
}


#if 0

#include "PMSUtil.h"
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

#define FORCE_TRANS_LIMIT               100
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

#define MAX_TIMES_PER_TRANSMISSION      100              //����ÿ�θ����������͵Ĵ���
   
static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

#define RESET_NODE_HIGH_ADDR            0x09
#define RESET_NODE_LOW_ADDR             0x09

#define RESET_COMM_CHANNEL              0x02

#define SENSOR_NODE_START_HIGH_ADDR     0x00
#define SENSOR_NODE_COUNT               0x03

#define SENSOR_NODE_LOW_ADDR            0x03

static uint8_t g_btHighAddr = 0;

static mrfiPacket_t g_transmit_msg;

static uint8_t g_nTransmitTimes = 0;

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
	init_MRFI(TXPOWER, RESET_COMM_CHANNEL, RESET_NODE_HIGH_ADDR, RESET_NODE_LOW_ADDR);

	showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
        memset(&g_transmit_msg, 0x0, sizeof(mrfiPacket_t));
	g_transmit_msg.frameLength = PACKET_LEN - 3;
        
        pmsFlags = 0;
        g_btHighAddr = SENSOR_NODE_START_HIGH_ADDR;
        g_nForceTransLimit = FORCE_TRANS_LIMIT;
        
        startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
        
	while(1)
        {
                LPM3;
		__no_operation();
                
                if(PMS_IS_SET(FLAG_WAIT_TIME_DUE))
                {
                      PMS_CLEAR(FLAG_WAIT_TIME_DUE);
                      
                      g_btHighAddr = g_btHighAddr % SENSOR_NODE_COUNT;
                      g_transmit_msg.highAddr = g_btHighAddr + 1;
                      g_btHighAddr += 1;
                      g_transmit_msg.lowAddr = SENSOR_NODE_LOW_ADDR;
                      g_transmit_msg.frame[0] = RESET_NODE_HIGH_ADDR;
                      g_transmit_msg.frame[1] = RESET_NODE_LOW_ADDR;
                      g_transmit_msg.frame[2] = OP_RESET_MAG_PARAM;
                      //forcedTransPacket(&g_transmit_msg);   //���͸�λ�ش���Ƶ����
//                          delayInMs_BSP(50);
                      for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                      {
                          forcedTransPacket(&g_transmit_msg);   //���͸�λ�ش���Ƶ����
                          delayInMs_BSP(100);
                      }
                }
                startTimer_TimerA(TRANSMIT_TIME_PERIOD ,executeAtTimeUp_TimerA);
                
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
                //g_nForceTransLimit--;
	} while (status == FAILURE);//(g_nForceTransLimit > 0 && status == FAILURE);
        //g_nForceTransLimit = FORCE_TRANS_LIMIT;
}

#endif