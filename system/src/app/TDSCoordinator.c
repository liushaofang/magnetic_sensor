#include <string.h>
#include "PMSUtil.h"
#include "Uart.h"
#include "Led.h"

#define FLAG_RCV_PACKAGE								1
#define FLAG_SENT_PACKAGE							2
#define FLAG_RCV_UART_DATA                                      3
#define FLAG_GOT_ACK_OVERTIME				        4

/*      定义转接板到配置器接收命令长度与接收命令缓冲区大小       */
#ifdef CONFIG_UART_TO_CAN_BUF_SIZE
  #define UART_MSG_LEN		        8 
#elif defined(CONFIG_TRAFFIC_DETECTION_APPLICATION)
  #define UART_MSG_LEN		        16
#endif

#define FORCE_TRANS_LIMIT               100
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

#define FLAG_RESET_COOR_DEV			  0
#define FLAG_CHANGE_CHANNEL			  1
#define FLAG_RESET_MAG_SENSOR			  2

static uint16_t g_opt_after_sending_packet = 0;
#define OPT_AFTER_SENDING_PACKET_IS_SET(x)        (g_opt_after_sending_packet & (0x01 << x))
#define OPT_AFTER_SENDING_PACKET_CLEAR(x)         (g_opt_after_sending_packet &= ~(0x01 << x))
#define OPT_AFTER_SENDING_PACKET_SET(x)           (g_opt_after_sending_packet |= (0x01 << x))

#define RESET_COOR_DEV					0x0001
#define CHANGE_CHANNEL			0x0002
#define RESET_MAG_SENSOR			0x0004

#define ACK_WAITING_MS		100
#define DATA_WAITING_MS		50

#define NODE_NUM_PER_PAGE						3
#define MAX_SON_NUM			50

#define SINGLE_NODE_WAIT_TIMEOUT        50

#define MAX_RESEND_TIMES							120

typedef struct element_of_endlist{
	addr_t end_addr;
	uint8_t rssi0;
	uint8_t rssi1;
        enum HeartbeatState heartbeat_flag;
}element_of_endlist_t;

static mrfiPacket_t g_transmit_msg, g_receive_msg;
static uint8_t g_son_number = 0;
static uint8_t g_total_page_num = 1;
static uint8_t g_cur_page_num = 1;
static uint8_t g_cur_rssi_index = 1;
static element_of_endlist_t g_end_list[MAX_SON_NUM];
static addr_t configDevAddr = {CONFIG_ADDR};
static uint8_t mac_addr[3] = MAC_ADDR;
static uint8_t uartMsg[UART_MSG_LEN];
static uint8_t prevUartMsg[UART_MSG_LEN];
static uint8_t g_chUploadUartMsg[UART_MSG_LEN + 4];     //TODO:merge this message with uartMsg
static int8_t g_current_son_index = 0;
static uint16_t resend_times = 0;

static uint8_t g_nUploadCurrentChannelFlag = 1;

void process_uart_cmd_packet();

/**
 * address of parent node.
 */
static nwkParam_t g_nwk_param;
const addr_t broadcastAddr = {BROADCAST_ADDR};

static void forcedTransPacket(mrfiPacket_t* msg);
static void parseReceivedMsg();
static void composeTransPacket(uint8_t operation_id, addr_t targetAddr);
static void configNwkParam();
static void manageEndList();
static int8_t getCurSonIndex(addr_t son_addr);
static uint8_t addEndToList(uint8_t highAddr, uint8_t lowAddr);
static uint8_t deleteEndFromList(uint8_t highAddr, uint8_t lowAddr);
static void processRssiInfo();
static void resetDev();
static void saveEndList();
static void get_ack_from_can_node();

void executeAtTransmitPacketISR_MRFI() {
	PMS_SET(FLAG_SENT_PACKAGE);
}

void executeAtReceiveSyncISR_MRFI() {
}

void executeAtRFFinishedISR_MRFI() {
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	/* quickly check the packet, if not right, report. */
	/* report will probably produce packet-lost */
	if (packet->frameLength != PACKET_LEN - 3) {
                DISABLE_WDT;    //2015.02.01:liusf add watchdog function
		twinkleLed_LED(LED2, 2, 200);
		delayInMs_BSP(2000);
                FEED_WDT;    //2015.02.01:liusf add watchdog function
	}
        
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
	memcpy(&g_receive_msg, packet, sizeof(g_receive_msg));
        FEED_WDT;    //2015.02.01:liusf add watchdog function
	PMS_SET(FLAG_RCV_PACKAGE);
}

void executeAtTimeUp_TimerA() {
        PMS_SET(FLAG_GOT_ACK_OVERTIME);
}

void executeAtUartISR_BSP(const uint8_t *uart_msg){
        memcpy(uartMsg, uart_msg, UART_MSG_LEN);
        PMS_SET(FLAG_RCV_UART_DATA);
}

void main() {
	/* Board Init */
	initBSP_BSP();
	initUart_BSP();
	initLeds_LED();
	init_system_flash_rw();

//	mac_addr[0] = 0x01;     //0x01表示协调器（原来为0x02)、0x10~0x8F表示传感器，0x90~0xFF表示灯节点(原来为02）
//	mac_addr[1] = 0x01;     
//	mac_addr[2] = 0x01;     //协调器节点不断加1的地址值，烧写协调器节点需要修改的字节地址
//	write_system_param(PARAM_TYPE_MAC, mac_addr, 3);
//
//	g_nwk_param.ip_addr.addr[0] = 0x01;
//	g_nwk_param.ip_addr.addr[1] = 0x01;     //协调器节点IP地址以0x01结尾
//	g_nwk_param.channel = 0x0;
//	g_nwk_param.tx_power = TXPOWER;
//	g_nwk_param.father_addr.addr[0] = 0x01;
//	g_nwk_param.father_addr.addr[1] = 0x01; //协调器节点的父节点IP地址为自己
//	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
//
//	/*uint8_t data[21];
//	data[0] = 21;
//	data[1] = 0x01;
//	data[2] = 0x03;
//	data[3] = 0x02;
//	data[4] = 0x03;
//	data[5] = 0x03;
//	data[6] = 0x03;
//	data[7] = 0x04;
//	data[8] = 0x03;
//	data[9] = 0x05;
//	data[10] = 0x03;
//	data[11] = 0x06;
//	data[12] = 0x03;
//	data[13] = 0x07;
//	data[14] = 0x03;
//	data[15] = 0x08;
//	data[16] = 0x03;
//	data[17] = 0x09;
//	data[18] = 0x03;
//	data[19] = 0x0A;
//	data[20] = 0x03;
//	write_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, data, 21);*/
//        write_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, 0, 0);
//	while(1);
        
	if(read_system_param(PARAM_TYPE_MAC, mac_addr, 3) != SUCCESS){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}

	if(read_system_param(PARAM_TYPE_NETWORK_PARAMS, &(g_nwk_param.ip_addr.addr[0]), sizeof(nwkParam_t)) != SUCCESS){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}
        
        /*      如为魔术字符串"NJRTGS"，则初始化网络参数和地磁参数 */
        if(!memcmp((uint8_t *)(&g_nwk_param), FLASH_NETWORK_PARAMS_MAGIC_STRING, FLASH_NETWORK_PARAMS_MAGIC_STRING_LEN))        //对于从量产得到的Flash需要对该传感器的IP地址和网络参数进行设置
        {
                g_nwk_param.ip_addr.addr[0] = 0x01;
                g_nwk_param.ip_addr.addr[1] = 0x01;     //协调器节点IP地址以0x01结尾
                g_nwk_param.channel = 0x0;
                g_nwk_param.tx_power = TXPOWER;
                g_nwk_param.father_addr.addr[0] = 0x01;
                g_nwk_param.father_addr.addr[1] = 0x01; //协调器节点的父节点IP地址为自己
                write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
        }

	/*read end list from flash*/
	uint8_t data_len = 0;
        if(read_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, &data_len, 1) != SUCCESS) {
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}

	if(data_len != 0xFF) {
		g_son_number = (data_len - 1) / 2;
	}

	if(g_son_number > 0){
		uint8_t data[2 * MAX_SON_NUM + 1];
                read_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, data, data_len);
                for(uint8_t i = 0; i < g_son_number; i++) {
			g_end_list[i].end_addr.addr[0] = data[2 * i + 1];
			g_end_list[i].end_addr.addr[1] = data[2 * (i + 1)];
                        g_end_list[i].heartbeat_flag = DEAD;
		}
	}
        
	init_MRFI(g_nwk_param.tx_power, g_nwk_param.channel, g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
        
        showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
        memset(uartMsg, 0 , UART_MSG_LEN);
        memset(prevUartMsg, 0 , UART_MSG_LEN);
	//向CAN节点发送通道号并等待CAN节点应答
	//get_ack_from_can_node();      //TODO:2013.12.17--liusf comment it off for no can node situation.
        
	g_transmit_msg.frameLength = PACKET_LEN -3;
	turnOnWAR_MRFI();
	while(1) {
          
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		LPM3;
                //DISABLE_WDT;       //2015.04.19:liusf add watchdog function
		__no_operation();

                FEED_WDT;       //2015.02.01:liusf add watchdog function
		//接收到单个字节的串口数据，需要对接收到的串口数据进行处理
		if(PMS_IS_SET(FLAG_RCV_UART_DATA)) {
			PMS_CLEAR(FLAG_RCV_UART_DATA);
                        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
			twinkleLed_LED(LED3, 1, 100);
			g_current_son_index = 0;
                        process_uart_cmd_packet(uartMsg, UART_MSG_LEN);
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
		}

                FEED_WDT;       //2015.02.01:liusf add watchdog function
		//配置器命令发送成功，需要等待接收器（协调器、中继器、传感器）数据应答
		if(PMS_IS_SET(FLAG_SENT_PACKAGE)){
			PMS_CLEAR(FLAG_SENT_PACKAGE);
			if(g_transmit_msg.frame[2] == OP_RETRIVE_SENSOR_STATUS || g_transmit_msg.frame[2] == OP_RESERVE_PARKING || g_transmit_msg.frame[2] == OP_RESET_MAGNET_SENSOR){
                        //if(g_transmit_msg.frame[2] == OP_RETRIVE_SENSOR_STATUS || g_transmit_msg.frame[2] == OP_RESERVE_PARKING){
				startTimer_TimerA(SINGLE_NODE_WAIT_TIMEOUT, executeAtTimeUp_TimerA);
			}

			if(g_opt_after_sending_packet & CHANGE_CHANNEL){
				g_opt_after_sending_packet &= ~CHANGE_CHANNEL;
                                FEED_WDT;       //2015.02.01:liusf add watchdog function
				changeChannelNum_MRFI(g_nwk_param.channel);
                                //2014.02.16: liusf modified,向CAN节点发送CAN 节点编号即网络信道号
                                //get_ack_from_can_node();
                                //2014.02.16: liusf modified end
			}
                        if(OPT_AFTER_SENDING_PACKET_IS_SET(FLAG_CHANGE_CHANNEL)){
                                OPT_AFTER_SENDING_PACKET_CLEAR(FLAG_CHANGE_CHANNEL);
                                FEED_WDT;       //2015.02.01:liusf add watchdog function
				changeChannelNum_MRFI(g_nwk_param.channel);
                                //2014.02.16: liusf modified,向CAN节点发送CAN 节点编号即网络信道号
                                //get_ack_from_can_node();
                                //2014.02.16: liusf modified end
			}
			if(OPT_AFTER_SENDING_PACKET_IS_SET(FLAG_RESET_COOR_DEV)){
                                OPT_AFTER_SENDING_PACKET_CLEAR(FLAG_RESET_COOR_DEV);
                                FEED_WDT;       //2015.02.01:liusf add watchdog function
				resetDev();
			}
			turnOnWAR_MRFI();
		}
                
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		if(PMS_IS_SET(FLAG_RCV_PACKAGE)){
			PMS_CLEAR(FLAG_RCV_PACKAGE);
			parseReceivedMsg();
		}

                FEED_WDT;       //2015.02.01:liusf add watchdog function
		//接收应答数据超时，需要通过串口返回失败信息
		if(PMS_IS_SET(FLAG_GOT_ACK_OVERTIME)) {
			PMS_CLEAR(FLAG_GOT_ACK_OVERTIME);
                        
                        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                        
			if(g_transmit_msg.frame[2] == OP_RETRIVE_SENSOR_STATUS){
				if(resend_times < MAX_RESEND_TIMES) {
					resend_times++;
					forcedTransPacket(&g_transmit_msg);
				} else {
					resend_times = 0;
					uartMsg[0] = g_nwk_param.ip_addr.addr[0];
					uartMsg[1] = g_nwk_param.ip_addr.addr[1];
					uartMsg[2] = g_end_list[g_current_son_index].end_addr.addr[0];
					uartMsg[3] = g_end_list[g_current_son_index].end_addr.addr[1];
					uartMsg[4] = OP_REPORT_ERROR_CODE;
					uartMsg[5] = ERROR_NODE_ACK_TIMEOUT;
					uartMsg[6] = 0x0;
					uartMsg[7] = 0x0;
                                        writeToUart_BSP(uartMsg, UART_MSG_LEN);
				}
			}
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
		}
	}
}

/**
  *@brief 与CAN节点交互发送Channel ID后获取应答
**/
static void get_ack_from_can_node()
{
    //uartMsg[0] = g_nwk_param.ip_addr.addr[0];
    //uartMsg[1] = g_nwk_param.ip_addr.addr[1];
    uartMsg[0] = mac_addr[1];
    uartMsg[1] = mac_addr[2];
    uartMsg[2] = g_end_list[0].end_addr.addr[0];
    uartMsg[3] = g_end_list[0].end_addr.addr[1];
    uartMsg[4] = OP_UPLOAD_COORDINATOR_CHAN;
    uartMsg[5] = g_nwk_param.channel;
    uartMsg[6] = 0x0;
    uartMsg[7] = 0x0;
    do
    {
        g_nUploadCurrentChannelFlag = 1;
        writeToUart_BSP(uartMsg, UART_MSG_LEN);
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
        delayInMs_BSP(100);
        FEED_WDT;    //2015.02.01:liusf add watchdog function
        if(PMS_IS_SET(FLAG_RCV_UART_DATA)) 
        {
            PMS_CLEAR(FLAG_RCV_UART_DATA);
            if(uartMsg[4] == OP_ACK_UPLOAD_COORDINATOR_CHAN)
            {
              g_nUploadCurrentChannelFlag = 0;
            }
        }
    }while(g_nUploadCurrentChannelFlag);
}

/**
 * @brief 解析从串口收到的命令数据包
 */
void process_uart_cmd_packet() {
        addr_t temp_addr;
	uint8_t cmd_type = uartMsg[4];
        uint8_t index = 0;
	switch(cmd_type) {
	case OP_RETRIVE_SENSOR_STATUS:
                //2014.02.16:liusf modified to retrive sensor status
                resend_times = 0;
                temp_addr.addr[0] = uartMsg[0];
		temp_addr.addr[1] = uartMsg[1];
                g_current_son_index = getCurSonIndex(temp_addr);
                if(g_current_son_index >= 0){
                        composeTransPacket(OP_RETRIVE_SENSOR_STATUS, g_end_list[g_current_son_index].end_addr);
                        forcedTransPacket(&g_transmit_msg);
                }
                else
                {
                        //通过串口显示发送传感器在协调器中不存在
                        uartMsg[2] = uartMsg[0];
                        uartMsg[3] = uartMsg[1];
                        uartMsg[0] = g_nwk_param.ip_addr.addr[0];
                        uartMsg[1] = g_nwk_param.ip_addr.addr[1];
                        uartMsg[4] = OP_REPORT_ERROR_CODE;
                        uartMsg[5] = ERROR_NODE_NOT_EXIST;       //TODO:增加错误编号
                        uartMsg[6] = 0x0;
                        uartMsg[7] = 0x0;
                        writeToUart_BSP(uartMsg, UART_MSG_LEN);
                }
                //2014.02.16:liusf modified end
                /*composeTransPacket(OP_RETRIVE_SENSOR_STATUS, g_end_list[g_current_son_index].end_addr);
		forcedTransPacket(&g_transmit_msg);*/
		break;
	case OP_RESERVE_PARKING:
		g_transmit_msg.highAddr = uartMsg[0];
		g_transmit_msg.lowAddr = uartMsg[1];
		g_transmit_msg.frame[0] = g_nwk_param.ip_addr.addr[0];
		g_transmit_msg.frame[1] = g_nwk_param.ip_addr.addr[1];
		g_transmit_msg.frame[2] = OP_RESERVE_PARKING;
		g_transmit_msg.frame[3] = uartMsg[5];
		changeChannelNum_MRFI(g_nwk_param.channel + 1);
		forcedTransPacket(&g_transmit_msg);
		break;
        case OP_RESET_MAGNET_SENSOR:
                g_transmit_msg.highAddr = uartMsg[0];
		g_transmit_msg.lowAddr = uartMsg[1];
		g_transmit_msg.frame[0] = uartMsg[2];
		g_transmit_msg.frame[1] = uartMsg[3];
		for(index = 2; index < PACKET_LEN - 3; index++) {
		  g_transmit_msg.frame[index] = uartMsg[index + 2];
		}
		forcedTransPacket(&g_transmit_msg);
                break;
	default:
		break;
	}
}

/**
 * @brief 强制发送数据包，发送过程中如果发送失败，则延时50ms再发，直到发送完成为止
 * @param msg 要发送的数据包
 */
void forcedTransPacket(mrfiPacket_t *msg) {
	uint8_t status = 0;
	startTimerAsDelaying_TimerB(5);
        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
	LPM3;
        DISABLE_WDT;    //2015.04.19:liusf add for watchdog exception

	do{
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		status = transmitPacket_MRFI(msg);
		/*if transmit failed, delay 50ms, retry  to transmit*/
		if(status == FAILURE) {
			/*turn off RF in order to save power*/
			turnOffRadio_MRFI();

			/*delay 50ms*/
			startTimerAsDelaying_TimerB(100);
                        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
			LPM3;
                        DISABLE_WDT;    //2015.04.19:liusf add for watchdog exception
		}
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                
                g_nForceTransLimit--;
	} while (g_nForceTransLimit > 0 && status == FAILURE);
        
        g_nForceTransLimit = FORCE_TRANS_LIMIT;
}

/**
 * @brief
 */
void parseReceivedMsg(){
	uint8_t operation_type = g_receive_msg.frame[2];

	switch(operation_type) {
	case OP_UPLOAD_STATUS_DATA:
	case OP_REPORT_ERROR_CODE:{
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		addr_t temp_addr;
		temp_addr.addr[0] = g_receive_msg.frame[0];
		temp_addr.addr[1] = g_receive_msg.frame[1];
                
                twinkleLed_LED(LED2, 1, 50);
		if(getCurSonIndex(temp_addr) < 0){
			//TODO caohua 回包，发送STOP命令
			break;
		}

		/*将数据发送给CAN节点*/
		/*for(uint8_t index = 0; index < PACKET_LEN - 3; index++){
			uartMsg[index] = g_receive_msg.frame[index];
		}*/
                
                memcpy(uartMsg, &g_receive_msg.highAddr, UART_MSG_LEN - 1);
                if(memcmp(prevUartMsg, uartMsg, UART_MSG_LEN - 1))
                {
                  memcpy(prevUartMsg, uartMsg, UART_MSG_LEN - 1);
                #ifndef CONFIG_ADD_REVERSE_TRAFFIC_COUNT
                  //2014.01.19:   liusf modified for uploading RSSI and LQI data,目前仅上传RSSI值
                  uartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
                  //2014.01.19:   liusf modified end
                  writeToUart_BSP(uartMsg, UART_MSG_LEN);
                #else
                  memcpy(g_chUploadUartMsg, &g_receive_msg.highAddr, UART_MSG_LEN);
                  g_chUploadUartMsg[5] = g_nwk_param.channel;
                  //2014.01.19:   liusf modified for uploading RSSI and LQI data,目前仅上传RSSI值
                  g_chUploadUartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
                  //2014.01.19:   liusf modified end
                  writeToUart_BSP(g_chUploadUartMsg, UART_MSG_LEN);
                #endif
                  twinkleLed_LED(LED2, 1, 50);
                  //2014.04.20:   liusf comment it off,目前传感器给协调器发送状态信息，协调器不需要应答
                  /*
                  toggleLed_LED(LED2);
                  composeTransPacket(OP_ACK_END, temp_addr);
                  forcedTransPacket(&g_transmit_msg);
                  toggleLed_LED(LED2);
                  */
                  //2014.04.20:   liusf comment end
                }
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                
	}
		break;
	case OP_END_HEARTBEAT:{
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		addr_t temp_addr;
		temp_addr.addr[0] = g_receive_msg.frame[0];
		temp_addr.addr[1] = g_receive_msg.frame[1];
                int8_t nIndex = getCurSonIndex(temp_addr);
		if(nIndex < 0){
			//TODO caohua 回包，发送STOP命令
                        g_transmit_msg.highAddr = g_receive_msg.frame[0];
                        g_transmit_msg.lowAddr = g_receive_msg.frame[1];
                        g_transmit_msg.frame[0] = g_nwk_param.ip_addr.addr[0];
                        g_transmit_msg.frame[1] = g_nwk_param.ip_addr.addr[1];
                        g_transmit_msg.frame[2] = OP_STOP_SENSOR_HEARTBEAT;     //通过控制地磁传感器实现STOP心跳
                        g_transmit_msg.frame[3] = 0x0;
                        forcedTransPacket(&g_transmit_msg);
			break;
		}

		//TODO caohua 记录节点心跳
                g_end_list[nIndex].heartbeat_flag = LIVE;
                //2014.04.20:   liusf comment it off,目前传感器给协调器发送状态信息，协调器不需要应答
                /*
                composeTransPacket(OP_ACK_END, temp_addr);
		forcedTransPacket(&g_transmit_msg);
                */
                //2014.04.20:   liusf comment end
                FEED_WDT;       //2015.02.01:liusf add watchdog function
	}
		break;
	case OP_CONFIG_COOR_NWK_PARAM:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		configNwkParam();
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_MANAGE_END_LIST:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		manageEndList();
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_GET_END_RSSI_LIST:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		processRssiInfo();
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_RESET_COORDINATOR_DEV:
		//g_opt_after_sending_packet |= RESET_COOR_DEV;
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                OPT_AFTER_SENDING_PACKET_SET(FLAG_RESET_COOR_DEV);
		composeTransPacket(OP_COOR_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_ACK_RETRIVE_SENSOR_STATUS:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		stopTimer_TimerA();
                resend_times = 0;
		memcpy(uartMsg, &g_receive_msg.highAddr, UART_MSG_LEN -1);
		writeToUart_BSP(uartMsg, UART_MSG_LEN);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		/*
                **caoh add,liusf comment it off to support one sensor status detection only
                g_current_son_index++;
		if(g_current_son_index < g_son_number) {
			resend_times = 0;
			composeTransPacket(OP_RETRIVE_SENSOR_STATUS, g_end_list[g_current_son_index].end_addr);
			forcedTransPacket(&g_transmit_msg);
		} else {
			g_current_son_index = 0;
		}
                */
		break;
	case OP_ACK_RESERVE_PARKING:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		changeChannelNum_MRFI(g_nwk_param.channel);
		memcpy(uartMsg, &g_receive_msg.highAddr, UART_MSG_LEN - 1);
		writeToUart_BSP(uartMsg, UART_MSG_LEN);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
        case OP_UPLOAD_RAW_MAGNETIC_DATA:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                memcpy(uartMsg, &g_receive_msg.highAddr, 4);
                uartMsg[0] = g_receive_msg.frame[0];
                uartMsg[1] = g_receive_msg.frame[1];
                uartMsg[2] = g_receive_msg.frame[3];
                uartMsg[3] = g_receive_msg.frame[4];
                uartMsg[4] = OP_UPLOAD_RAW_MAGNETIC_DATA_TO_UART;
                memcpy(&uartMsg[5], &g_receive_msg.frame[5], 6);
                uartMsg[11] = rssi_value_2_power(g_receive_msg.rxMetrics[0]);
		writeToUart_BSP(uartMsg, UART_MSG_LEN);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	default:
		break;
	}
}

/**
 * @brief 构成发给Relay的包
 */
static void composeTransPacket(uint8_t operation_id, addr_t targetAddr) {
	g_transmit_msg.highAddr = targetAddr.addr[0];
	g_transmit_msg.lowAddr = targetAddr.addr[1];
	g_transmit_msg.frame[0] = g_nwk_param.ip_addr.addr[0];
	g_transmit_msg.frame[1] = g_nwk_param.ip_addr.addr[1];
	g_transmit_msg.frame[2] = operation_id;

	switch(operation_id) {
	case OP_ACK_END:{
		uint8_t operation_type = g_receive_msg.frame[2];
		switch(operation_type){
		case OP_UPLOAD_STATUS_DATA:
			g_transmit_msg.frame[3] = 0x01;
			break;
		case OP_REPORT_ERROR_CODE:
			g_transmit_msg.frame[3] = 0x02;
			break;
		case OP_END_HEARTBEAT:
			g_transmit_msg.frame[3] = 0x03;
			break;
		default :
			break;
		}
	}
		break;
	case OP_UPLOAD_END_LIST:
		if(g_cur_page_num < g_total_page_num) {
			g_transmit_msg.frame[3] = NODE_NUM_PER_PAGE;
		} else {
                        //2014.01.20: liusf modified for upload sensor list debug
			//g_transmit_msg.frame[3] = g_son_number % NODE_NUM_PER_PAGE;
                        g_transmit_msg.frame[3] = (g_son_number % NODE_NUM_PER_PAGE)?(g_son_number % NODE_NUM_PER_PAGE):NODE_NUM_PER_PAGE;
                        //2014.01.20: liusf modified end
		}
		g_transmit_msg.frame[4] = g_cur_page_num;
		g_transmit_msg.frame[5] = g_total_page_num;
		for(uint8_t i = 0; i < NODE_NUM_PER_PAGE; i++) {
			uint8_t cur_end = (g_cur_page_num - 1) * NODE_NUM_PER_PAGE + i;
			g_transmit_msg.frame[6 + 2*i] = g_end_list[cur_end].end_addr.addr[0];
			g_transmit_msg.frame[7 + 2*i] = g_end_list[cur_end].end_addr.addr[1];
		}
		break;
	case OP_ACK_END_LIST_OPERATION:
		for(int i = 3; i < 6; i++) {
			g_transmit_msg.frame[i] = g_receive_msg.frame[i];
		}
		break;
	case OP_UPLOAD_RSSI_LIST:
		g_transmit_msg.frame[3] = g_cur_rssi_index;
		g_transmit_msg.frame[4] = g_son_number;
		g_transmit_msg.frame[5] = g_end_list[g_cur_rssi_index - 1].end_addr.addr[0];
		g_transmit_msg.frame[6] = g_end_list[g_cur_rssi_index - 1].end_addr.addr[1];
		g_transmit_msg.frame[7] = g_end_list[g_cur_rssi_index - 1].rssi0;
		g_transmit_msg.frame[8] = g_end_list[g_cur_rssi_index - 1].rssi1;
		break;
	case OP_COOR_ACK_CONFIG:{
		uint8_t operation_type = g_receive_msg.frame[2];
		switch(operation_type){
		case OP_RESET_COORDINATOR_DEV:
			g_transmit_msg.frame[3] = 0x01;
			break;
		case OP_CONFIG_COOR_NWK_PARAM:
			g_transmit_msg.frame[3] = 0x02;
			g_transmit_msg.frame[4] = mac_addr[0];
			g_transmit_msg.frame[5] = mac_addr[1];
			g_transmit_msg.frame[6] = mac_addr[2];
			g_transmit_msg.frame[7] = g_nwk_param.ip_addr.addr[0];
			g_transmit_msg.frame[8] = g_nwk_param.ip_addr.addr[1];
			g_transmit_msg.frame[9] = g_nwk_param.channel;
			g_transmit_msg.frame[10] = g_nwk_param.tx_power;
			break;
		default :
			break;
		}
	}
		break;
	case OP_RETRIVE_SENSOR_STATUS:
		break;
	default:
		break;
	}
}

/**
 * @brief 根据配置器发送的包配置Nwk的参数，包括Mac地址，Ip地址，father的ip地址
 */
void configNwkParam() {
	uint8_t cmd_byte = g_receive_msg.frame[INDEX_MASK];

	//TODO caohua 是否加判断，修改MAC地址的时候收到的包必须带IP地址，不能使广播地址
	if(cmd_byte & 0x01){
		/*能收到包说明地址校验通过*/
		for(uint8_t i = 0; i < 3; i++) {
			mac_addr[i] = g_receive_msg.frame[INDEX_MAC_BASE_ADDR + i];
		}
		/*修改MAC地址，则此次操作不允许修改其他信息*/
                write_system_param(PARAM_TYPE_MAC, mac_addr, 3);
                composeTransPacket(OP_COOR_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
		return;
	}

	/*MAC地址不匹配，直接返回*/
	if (mac_addr[0] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR]
			|| mac_addr[1] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 1]
			|| mac_addr[2] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 2])
		return;

	/*配置节点IP地址*/
	if(cmd_byte & 0x02){
		g_nwk_param.ip_addr.addr[0] = g_receive_msg.frame[INDEX_NODE_HIGH_ADDR];
		g_nwk_param.ip_addr.addr[1] = g_receive_msg.frame[INDEX_NODE_LOW_ADDR];
		changeNodeAddr_MRFI(g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
	}

	/*配置节点信道号*/
	if(cmd_byte & 0x04){
		g_nwk_param.channel = g_receive_msg.frame[INDEX_CHANNEL_NUM];
		/*收到切信道的命令后先回ACK，然后再切*/
		//g_opt_after_sending_packet |= CHANGE_CHANNEL;
                OPT_AFTER_SENDING_PACKET_SET(FLAG_CHANGE_CHANNEL);
	}

	/*配置发送强度*/
	if(cmd_byte & 0x08){
		g_nwk_param.tx_power = g_receive_msg.frame[INDEX_TX_POWER];
		changeTxPower_MRFI(g_nwk_param.tx_power);
	}

        /*将修改的网络参数写到Flash中      */
	if(cmd_byte != 0x00) {
                
		write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
        }

	/*发送当前mrfi的参数作为ACK，也可以作为回复cmd_byte = 0x00时的应答包*/
	composeTransPacket(OP_COOR_ACK_CONFIG, configDevAddr);
	forcedTransPacket(&g_transmit_msg);
}

void manageEndList(){
	uint8_t cmd_type = g_receive_msg.frame[INDEX_MASK];
	switch(cmd_type) {
	case 0x01:{
		/*读取传感器节点IP列表*/
		/*获取总页数*/
		g_total_page_num = (g_son_number % NODE_NUM_PER_PAGE == 0)? g_son_number / NODE_NUM_PER_PAGE : (g_son_number / NODE_NUM_PER_PAGE + 1)  ;
		g_cur_page_num = 1;
		composeTransPacket(OP_UPLOAD_END_LIST, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
	}
		break;
	case 0x02:{
		/*增加传感器IP*/
		if(addEndToList(g_receive_msg.frame[4], g_receive_msg.frame[5]) == TRUE) {
			composeTransPacket(OP_ACK_END_LIST_OPERATION, configDevAddr);
			forcedTransPacket(&g_transmit_msg);
		} else {
			//TODO caohua 完善协议，通知配置器操作失败
		}
	}
		break;
	case 0x03:{
		/*删除传感器IP*/
		if(deleteEndFromList(g_receive_msg.frame[4], g_receive_msg.frame[5]) == FALSE) {
			composeTransPacket(OP_ACK_END_LIST_OPERATION, configDevAddr);
			forcedTransPacket(&g_transmit_msg);
		} else {
			//TODO caohua 完善协议，通知配置器操作失败
		}
	}
		break;
	case 0x04:{
		/*删除传感器列表*/
		g_son_number = 0;
		g_total_page_num = 1;
		g_cur_page_num = 1;
		memset(g_end_list, 0x00, sizeof(g_end_list));
		erase_system_flash(SEGMENT_TYPE_SENSOR_ADDR_LIST);
		composeTransPacket(OP_ACK_END_LIST_OPERATION, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
	}
		break;
	case 0x05:{
		/*应答列表上传数据*/
		if(g_receive_msg.frame[6] == g_cur_page_num){
			if(g_cur_page_num == g_total_page_num){
				g_cur_page_num = 0;
				break;
			}
			g_cur_page_num++;
			composeTransPacket(OP_UPLOAD_END_LIST, configDevAddr);
			forcedTransPacket(&g_transmit_msg);
		}
	}
		break;
	default:
		break;
	}
}

/**
 * @brief 添加End节点到End列表中
 * @param highAddr 新添加的End节点IP地址的高地址
 * @param lowAddr 新添加的End节点IP地址的低地址
 * @return TRUE：添加成功
 *  			 FALSE：添加失败
 */
uint8_t addEndToList(uint8_t highAddr, uint8_t lowAddr) {
	addr_t temp_addr;
	temp_addr.addr[0] = highAddr;
	temp_addr.addr[1] =lowAddr;
	/*如果添加的节点已经在列表中或者子节点的个数已经到达最大值，返回FALSE*/
	if((getCurSonIndex(temp_addr) >= 0) || g_son_number == MAX_SON_NUM ) {
		return FALSE;
	} else {
		g_end_list[g_son_number].end_addr = temp_addr;
		g_son_number++;
		saveEndList();
		return TRUE;
	}
}

/**
 * @brief 删除End列表中指定的节点
 * @param highAddr 删除的End节点IP地址的高地址
 * @param lowAddr 删除的End节点IP地址的低地址
 * @return TRUE：删除成功
 *  			 FALSE：删除失败
 */
uint8_t deleteEndFromList(uint8_t highAddr, uint8_t lowAddr) {
	addr_t temp_addr;
	temp_addr.addr[0] = highAddr;
	temp_addr.addr[1] = lowAddr;
	int8_t index = getCurSonIndex(temp_addr);
	if((index < 0) || g_son_number == 0){
		return FALSE;
	} else {
		if(index != g_son_number - 1) {
			g_end_list[index].end_addr = g_end_list[g_son_number - 1].end_addr;
		}
		g_end_list[g_son_number - 1].end_addr.addr[0] = 0x00;
		g_end_list[g_son_number - 1].end_addr.addr[1] = 0x00;
		g_son_number--;
		saveEndList();
		return TRUE;
	}
}

/**
 * @brief 获取给定的子节点在子节点列表中的下标
 * @param son_addr 给定的子节点
 * @return cur_index子节点下标 or -1表示给定的子节点地址不在列表中
 */
int8_t getCurSonIndex(addr_t son_addr) {
	int8_t cur_index;
	for(cur_index = 0; cur_index < g_son_number; cur_index++) {
		if(son_addr.addr[0] == g_end_list[cur_index].end_addr.addr[0] && son_addr.addr[1] == g_end_list[cur_index].end_addr.addr[1]) {
			return cur_index;
		}
	}
	return -1;
}

void processRssiInfo(){
	uint8_t cmd_type = g_receive_msg.frame[INDEX_MASK];
	switch(cmd_type) {
	case 0x01:
		g_cur_rssi_index = 1;
		composeTransPacket(OP_UPLOAD_RSSI_LIST, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
		break;
	case 0x02:
		/*应答列表上传数据*/
		if(g_receive_msg.frame[4] == g_cur_rssi_index){
			if(g_cur_rssi_index == g_son_number){
				g_cur_rssi_index = 0;
				break;
			}
			g_cur_rssi_index++;
			composeTransPacket(OP_UPLOAD_RSSI_LIST, configDevAddr);
			forcedTransPacket(&g_transmit_msg);
		}
		break;
	default:
		break;
	}
}

void resetDev() {
	g_opt_after_sending_packet = 0;
	g_nwk_param.channel = CHANNEL;
	g_nwk_param.tx_power = TXPOWER;
        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
        FEED_WDT;       //2015.02.01:liusf add watchdog function
	changeTxPower_MRFI(g_nwk_param.tx_power);
	changeChannelNum_MRFI(g_nwk_param.channel);

	g_son_number = 0;
	g_cur_page_num = 1;
	g_total_page_num = 1;
	g_cur_rssi_index = 1;
	memset(g_end_list, 0x00, MAX_SON_NUM * sizeof(element_of_endlist_t));
	memset(uartMsg, 0x00, UART_MSG_LEN);
        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
	erase_system_flash(SEGMENT_TYPE_SENSOR_ADDR_LIST);
        FEED_WDT;       //2015.02.01:liusf add watchdog function
}

void saveEndList() {
	uint8_t data[2 * MAX_SON_NUM + 1];
	data[0] = 2 * g_son_number + 1;
	for(uint8_t i = 0; i < g_son_number; i++) {
		data[2 * i + 1]  = g_end_list[i].end_addr.addr[0];
		data[2 * (i + 1)] = g_end_list[i].end_addr.addr[1];
	}
	write_system_param(PARAM_TYPE_SENSOR_ADDR_LIST, data, data[0]);
}
