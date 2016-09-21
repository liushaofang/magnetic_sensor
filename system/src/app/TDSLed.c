#include <string.h>
#include "PMSUtil.h"

/*标记收到网络数据包*/
#define FLAG_RCV_PACKAGE								1
/*标记数据包发送结束*/
#define FLAG_SENT_PACKAGE							2
/*标记串口接收到数据*/
#define FLAG_RCV_UART_DATA                                      3

#define UART_MSG_LEN		        16
#define UART_2_CAN_MSG_LEN              8

#define FORCE_TRANS_LIMIT               100
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static uint8_t uartMsg[UART_MSG_LEN];
static uint8_t prevUartMsg[UART_MSG_LEN];


static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static uint16_t g_opt_after_sending_packet = 0;
#define RESET_LED_DEV					0x0001
#define CHANGE_CHANNEL			0x0002
#define RESET_MAG_SENSOR			0x0004

static mrfiPacket_t g_transmit_msg, g_receive_msg;

/**
 * address of parent node.
 */
static nwkParam_t g_nwk_param;
static addr_t configDevAddr = {CONFIG_ADDR};
static uint8_t mac_addr[3] = MAC_ADDR;

static uint8_t bIsCarParked = 0;
static uint8_t g_cur_state = VACANT;

static void parseReceivedMsg();
static void forcedTransPacket(mrfiPacket_t* msg);
static void composeTransPacket(uint8_t operation_id, addr_t target_addr);
static void configNwkParam();
static void resetDev();

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
		twinkleLed_LED(LED2, 2, 200);
		delayInMs_BSP(2000);
	}

	memcpy(&g_receive_msg, packet, sizeof(g_receive_msg));
	PMS_SET(FLAG_RCV_PACKAGE);
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
        
        memset(uartMsg, 0, UART_MSG_LEN);
        memset(prevUartMsg, 0, UART_MSG_LEN);

//	mac_addr[0] = 0x90;     //0x01表示协调器、0x10~0x8F表示传感器，0x90~0xFF表示灯节点(原来为02）
//	mac_addr[1] = 0x01;
//	mac_addr[2] = 0x34;     //灯节点不断加1的地址值，烧写灯节点需要修改的字节地址
//	write_system_param(PARAM_TYPE_MAC, mac_addr, 3);
//
//	g_nwk_param.ip_addr.addr[0] = 0x34;
//	g_nwk_param.ip_addr.addr[1] = 0x03;     //传感器节点IP地址以0x03结尾，灯节点与传感器节点IP地址相同
//	g_nwk_param.channel = 0x0;
//	g_nwk_param.tx_power = TXPOWER;
//	g_nwk_param.father_addr.addr[0] = 0x01;
//	g_nwk_param.father_addr.addr[1] = 0x01;
//	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
//
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
                g_nwk_param.ip_addr.addr[0] = 0x02;
                g_nwk_param.ip_addr.addr[1] = 0x03;     //传感器节点IP地址以0x03结尾，灯节点与传感器节点IP地址相同
                g_nwk_param.channel = 0x0;
                g_nwk_param.tx_power = TXPOWER;
                g_nwk_param.father_addr.addr[0] = 0x01;
                g_nwk_param.father_addr.addr[1] = 0x01;
                write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
        }

	init_MRFI(g_nwk_param.tx_power, g_nwk_param.channel, g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
	showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();

	g_transmit_msg.frameLength = PACKET_LEN - 3;
	turnOnWAR_MRFI();
	while(1) {
		LPM3;
		__no_operation();
                
		/*收到来自RF的数据包*/
		if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
			parseReceivedMsg();
		}

		/*向Relay发送数据包结束，进入WOR状态等100ms*/
		if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
			PMS_CLEAR(FLAG_SENT_PACKAGE);

			if(g_opt_after_sending_packet & CHANGE_CHANNEL){
				g_opt_after_sending_packet &= ~CHANGE_CHANNEL;
				changeChannelNum_MRFI(g_nwk_param.channel);
			}

			if(g_opt_after_sending_packet & RESET_LED_DEV){
				g_opt_after_sending_packet &= ~RESET_LED_DEV;
				resetDev();
			}

			turnOnWAR_MRFI();
		}
	}
}

/**
 * @brief 分析收到的数据包
 */
void parseReceivedMsg() {
	uint8_t operation_type = g_receive_msg.frame[2];
	switch(operation_type) {
	case OP_CONFIG_LED:
		bIsCarParked = g_receive_msg.frame[3];
		if(g_cur_state != bIsCarParked){
			g_cur_state = bIsCarParked;
                        uartMsg[0] = mac_addr[1];       //g_receive_msg.frame[0];
                        uartMsg[1] = mac_addr[2];       //g_receive_msg.frame[1];
                        uartMsg[2] = g_receive_msg.frame[0];    //g_receive_msg.highAddr;
                        uartMsg[3] = g_receive_msg.frame[1];    //g_receive_msg.lowAddr;
                        uartMsg[4] = OP_UPLOAD_STATUS_DATA;
                        uartMsg[5] = g_cur_state;
                        uartMsg[6] = g_receive_msg.frame[4];
                        uartMsg[7] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
                        if(memcmp(prevUartMsg, uartMsg, UART_MSG_LEN))
                        {
                          memcpy(prevUartMsg, uartMsg, UART_MSG_LEN);
                          writeToUart_BSP(uartMsg, UART_2_CAN_MSG_LEN);
                        }
                        
			if(g_cur_state == VACANT){
				//TODO caohua 亮绿灯
				turnOffLed_LED(LED1);
				turnOnLed_LED(LED2);
				turnOffLed_LED(LED3);
			}
                        else if (g_cur_state == OCCUPIED){
				//TODO caohua 亮红灯
				turnOffLed_LED(LED2);
				turnOnLed_LED(LED1);
				turnOffLed_LED(LED3);
			}
                        else if(g_cur_state == RESERVED){
                                //TODO caohua 亮蓝灯
				turnOffLed_LED(LED1);
				turnOffLed_LED(LED2);
				turnOnLed_LED(LED3);
                        }
		}
                //2014.04.20:   liusf comment it off, 目前传感器给灯节点发送数据，灯节点不需要应答数据包
                /*
		composeTransPacket(OP_ACK_CONFIG_LED, g_nwk_param.ip_addr);
		forcedTransPacket(&g_transmit_msg);
                */
                //2014.04.20:   liusf comment end
		break;
	case OP_RESET_LED_DEV:
		g_opt_after_sending_packet |= RESET_LED_DEV;
		composeTransPacket(OP_LED_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
		break;
	case OP_CONFIG_LED_NWK_PARAM:
		configNwkParam();
		break;
	case OP_RESERVE_PARKING:
                if(g_receive_msg.frame[3] == 0x1) {     //表示预定车位
			//TODO caohua 亮蓝灯
			g_cur_state = RESERVED;
			turnOffLed_LED(LED1);
			turnOffLed_LED(LED2);
			turnOnLed_LED(LED3);
		} else if(g_receive_msg.frame[3] == 0x0){       //表示取消预定车位
			//TODO caohua 亮绿灯
			g_cur_state = VACANT;
			turnOffLed_LED(LED1);
			turnOnLed_LED(LED2);
			turnOffLed_LED(LED3);
		}
                //uartMsg[0] = g_receive_msg.frame[0];
                //uartMsg[1] = g_receive_msg.frame[1];
                //uartMsg[2] = g_receive_msg.highAddr;
                //uartMsg[3] = g_receive_msg.lowAddr;
                uartMsg[0] = mac_addr[1];       //g_receive_msg.frame[0];
                uartMsg[1] = mac_addr[2];       //g_receive_msg.frame[1];
                uartMsg[2] = g_receive_msg.frame[0];    //g_receive_msg.highAddr;
                uartMsg[3] = g_receive_msg.frame[1];    //g_receive_msg.lowAddr;
                uartMsg[4] = OP_UPLOAD_STATUS_DATA;
                uartMsg[5] = g_cur_state;
                uartMsg[6] = g_receive_msg.frame[4];
                uartMsg[7] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
                writeToUart_BSP(uartMsg, UART_2_CAN_MSG_LEN);
                composeTransPacket(OP_ACK_RESERVE_PARKING, configDevAddr);
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
                
                g_nForceTransLimit--;
	} while (g_nForceTransLimit > 0 && status == FAILURE);
        
        g_nForceTransLimit = FORCE_TRANS_LIMIT;
}

/**
 * @brief 构成发送的包
 */
static void composeTransPacket(uint8_t operation_id, addr_t targetAddr) {
	g_transmit_msg.highAddr = targetAddr.addr[0];
	g_transmit_msg.lowAddr = targetAddr.addr[1];
	g_transmit_msg.frame[0] = g_nwk_param.ip_addr.addr[0];
	g_transmit_msg.frame[1] = g_nwk_param.ip_addr.addr[1];
	g_transmit_msg.frame[2] = operation_id;

	switch(operation_id) {
	case OP_ACK_CONFIG_LED:
		break;
	case OP_LED_ACK_CONFIG:{
		uint8_t operation_type = g_receive_msg.frame[2];
		switch(operation_type){
		case OP_RESET_LED_DEV:
			g_transmit_msg.frame[3] = 0x01;
			break;
		case OP_CONFIG_LED_NWK_PARAM:
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
        case OP_ACK_RESERVE_PARKING:
                if(g_cur_state == RESERVED)
                {
                    g_transmit_msg.frame[3] = 0x1;      //预定成功
                }
                else if(g_cur_state == VACANT)
                {
                    g_transmit_msg.frame[3] = 0x0;      //取消预定成功
                }
                else
                {
                    g_transmit_msg.frame[3] = 0xFF;      //操作失败
                }
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
		composeTransPacket(OP_LED_ACK_CONFIG, configDevAddr);
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
		g_opt_after_sending_packet |= CHANGE_CHANNEL;
	}

	/*配置发送强度*/
	if(cmd_byte & 0x08){
		g_nwk_param.tx_power = g_receive_msg.frame[INDEX_TX_POWER];
		changeTxPower_MRFI(g_nwk_param.tx_power);
	}

	if(cmd_byte != 0x00) {
		write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
	}
	/*发送当前mrfi的参数作为ACK，也可以作为回复cmd_byte = 0x00时的应答包*/
	composeTransPacket(OP_LED_ACK_CONFIG, configDevAddr);
	forcedTransPacket(&g_transmit_msg);
}

void resetDev() {
	g_opt_after_sending_packet = 0;
	g_nwk_param.ip_addr.addr[0] = 0x00;
	g_nwk_param.ip_addr.addr[1] = 0x03;
	g_nwk_param.channel = CHANNEL;
	g_nwk_param.tx_power = TXPOWER;
	changeTxPower_MRFI(g_nwk_param.tx_power);
	changeChannelNum_MRFI(g_nwk_param.channel);
	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
}
