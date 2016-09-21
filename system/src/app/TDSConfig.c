#include <string.h>
#include "PMSUtil.h"
#include "Uart.h"
#include "Led.h"

#define FLAG_RCV_PACKAGE					1
#define FLAG_SENT_PACKAGE					2
#define FLAG_RCV_UART_DATA                                      3
#define FLAG_GOT_ACK_OVERTIME				        4

/*      定义转接板到配置器接收命令长度与接收命令缓冲区大小       */
#define UART_MSG_LEN		16

static uint16_t tmsFlags = 0;
#define TMS_IS_SET(x)  (tmsFlags & (0x01 << x))
#define TMS_CLEAR(x)  (tmsFlags &= ~(0x01 << x))
#define TMS_SET(x)    (tmsFlags |= (0x01 << x))

#define ACK_WAITING_MS		50
#define DATA_WAITING_MS		50

#define MAX_RESEND_TIMES                        150

#define FORCE_TRANS_LIMIT               100
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static mrfiPacket_t transmitMsg, receiveMsg;

static uint8_t uartMsg[UART_MSG_LEN];

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

	memcpy(&receiveMsg, packet, sizeof(receiveMsg));
	TMS_SET(FLAG_RCV_PACKAGE);
}

void executeAtTimeUp_TimerB() {
        TMS_SET(FLAG_GOT_ACK_OVERTIME);
}

void executeAtUartISR_BSP(const uint8_t *uart_msg){
	  memcpy(uartMsg, uart_msg, UART_MSG_LEN);
	  TMS_SET(FLAG_RCV_UART_DATA);
}

void main() {
	/* Board Init */
	initBSP_BSP();
	initUart_BSP();
	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, nodeAddr.addr[0], nodeAddr.addr[1]);
        
	showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
	transmitMsg.frameLength = PACKET_LEN -3;
	turnOnWAR_MRFI();
        
	while(1) {
		LPM3;
		__no_operation();
                
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                
		//接收到单个字节的串口数据，需要对接收到的串口数据进行处理
		if(TMS_IS_SET(FLAG_RCV_UART_DATA)) {
                  TMS_CLEAR(FLAG_RCV_UART_DATA);
                  twinkleLed_LED(LED3, 1, 100);
                  process_uart_cmd_packet();
		}

		if(TMS_IS_SET(FLAG_RCV_PACKAGE)){
			TMS_CLEAR(FLAG_RCV_PACKAGE);
			stopTimer_TimerB();
			resend_times = 0;
			memcpy(uartMsg, &receiveMsg.highAddr, UART_MSG_LEN -1);
                        //2014.03.25:   liusf modified for uploading RSSI and LQI data,目前仅上传RSSI值
                        uartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(receiveMsg.rxMetrics[0]));
                        //2014.03.25:   liusf modified end
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
			writeToUart_BSP(uartMsg, UART_MSG_LEN);
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
		}

		//配置器命令发送成功，需要等待接收器（协调器、中继器、传感器）数据应答
		if(TMS_IS_SET(FLAG_SENT_PACKAGE)){
				TMS_CLEAR(FLAG_SENT_PACKAGE);
                                
                                //startTimerAsDelaying_TimerB(70);       //2014.06.08:   liusf add 70ms delay
                                
                                FEED_WDT;
                                
				startTimer_TimerB(ACK_WAITING_MS, executeAtTimeUp_TimerB);
				resend_times--;
                                
				turnOnWAR_MRFI();
                                
                                FEED_WDT;
		}

		//接收应答数据超时，需要通过串口返回失败信息
		if(TMS_IS_SET(FLAG_GOT_ACK_OVERTIME)) {
				TMS_CLEAR(FLAG_GOT_ACK_OVERTIME);
				if(resend_times > 0){
                                        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
					forcedTransPacket(&transmitMsg);
                                        FEED_WDT;       //2015.02.01:liusf add watchdog function
				} else {
					uartMsg[0] = 0x0 << 7 | 0x10;   //表示接收射频应答数据失败
                                        FEED_WDT;       //2015.02.01:liusf add watchdog function
					writeToUart_BSP(uartMsg, UART_MSG_LEN);
                                        FEED_WDT;       //2015.02.01:liusf add watchdog function
				}
		}
        }
}

/**
 * @brief 解析从串口收到的命令数据包
 */
void process_uart_cmd_packet() {
		uint8_t index = 0;

		if(uartMsg[4] == OP_CHANGE_CONFIG_DEV_CHAN){
			cur_channel = uartMsg[5];
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
			changeChannelNum_MRFI(cur_channel);
			uartMsg[4] = OP_ACK_CHANGE_CONFIG_DEV_CHAN;
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
			writeToUart_BSP(uartMsg, UART_MSG_LEN);
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
			return;
		}
                
		transmitMsg.highAddr = uartMsg[0];
		transmitMsg.lowAddr = uartMsg[1];
		transmitMsg.frame[0] = nodeAddr.addr[0];
		transmitMsg.frame[1] = nodeAddr.addr[1];
		for(index = 2; index < PACKET_LEN - 3; index++) {
		  transmitMsg.frame[index] = uartMsg[index + 2];
		}
		resend_times = ((uint16_t)uartMsg[UART_MSG_LEN - 2] << 8)  + uartMsg[UART_MSG_LEN -1];
                if(resend_times > MAX_RESEND_TIMES)
                  resend_times = MAX_RESEND_TIMES;
                DISABLE_WDT;
		forcedTransPacket(&transmitMsg);
                FEED_WDT;
}

/**
 * @brief 强制发送数据包，发送过程中如果发送失败，则延时50ms再发，直到发送完成为止
 * @param msg 要发送的数据包
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
			startTimerAsDelaying_TimerB(130);       //2014.06.08:   liusf changed delay from 50ms delay to 130ms delay
			LPM3;
		}
                
                g_nForceTransLimit--;
	} while (g_nForceTransLimit > 0 && status == FAILURE);
        
        g_nForceTransLimit = FORCE_TRANS_LIMIT;
}
