#include <string.h>
#include "PMSUtil.h"
#include "hmc5883l.h"
#include <stdlib.h>
#include "led.h"
#include "mrfi_spi.h"

/*标记收到网络数据包*/
#define FLAG_RCV_PACKAGE								1
/*标记数据包发送结束*/
#define FLAG_SENT_PACKAGE							2
/*等待ACK超时*/
#define FLAG_WAIT_ACK_OVERTIME				3
/*标记地磁传感器采集到新数据*/
#define FLAG_GEOMAGNETIC_DATA_PREPARED		4
/*标记End节点接收CMD的时间片结束*/
#define FLAG_WAIT_CMD_TIMER_DUE				5
/*标记End节点处于Sleep的时间片结束*/
#define FLAG_SLEEP_TIMER_DUE						6
/*标记需要发送心跳包     */
#define FLAG_HEARTBEAT_TIMER_DUE					7

#define RAW_DATA_LEN					6
#define WAIT_ACK_TIME			500
#define WAIT_CMD_TIME			500
#define SLEEP_TIME			5000

#define MAX_TRANSMIT_TIMES              5

#define MAX_TIMES_PER_TRANSMISSION      5
#define FORCE_TRANS_LIMIT               100

/*      标记End节点的心跳时间间隔HEARTBEAT_TIME_INTERVAL，设置为1小时=3600000 */
#define HEARTBEAT_TIME_INTERVAL         20000    //3600000       //2014.06.08:liusf modified for dead debug
/*      标记End节点的心跳发送心跳时，系统不处于发送心跳的状态需要延迟发送心跳的时间间隔，设置为5秒钟=5000 */
#define DELAY_HEARTBEAT_TIME_INTERVAL   1000    //5000       //2014.06.08:liusf modified for dead debug

static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static uint16_t g_opt_after_sending_packet = 0;
#define RESET_END_DEV					0x0001
#define CHANGE_CHANNEL			0x0002
#define RESET_MAG_SENSOR			0x0004

enum Timer_state{
	SLEEP = 0,
	WOR_FOR_CMD,
	WOR_FOR_ACK
};

static mrfiPacket_t g_transmit_msg, g_receive_msg;

//static uint32_t flow_value = 0;
static uint8_t geomagnetic_raw_data[RAW_DATA_LEN];
static uint8_t g_current_magnet_odr = CONFIG_ODR_75HZ;
/**
 * address of parent node.
 */
static nwkParam_t g_nwk_param;
static Magnet_Alg_Param_Type g_mag_param;
static addr_t configDevAddr = {CONFIG_ADDR};
static uint8_t mac_addr[3] = MAC_ADDR;

static volatile uint8_t g_is_car_parked = 0;
static volatile uint8_t g_cur_state = 0;

static volatile uint8_t g_cur_fluc_state = 0;
static volatile uint8_t g_prev_fluc_state = 0;

#define ABADONED_MAGNET_DATA_TIMEOUT       1000    //丢弃5000ms内的地磁数据
static volatile uint8_t g_n_abandoned_magnet_value_flag = 0;

void executeAtAbandonedMagnetDataTimeout()
{
    g_n_abandoned_magnet_value_flag = 0;
}


//static uint8_t g_led_channel = CHANNEL + 1;
static uint8_t g_rf_communication_state = FALSE;
static uint8_t g_timer_state = SLEEP;

static uint16_t g_u16BatteryVoltage = 0;
static uint8_t g_nTransmitCount = 0;

static uint8_t g_nTransmitTimes = MAX_TIMES_PER_TRANSMISSION;    //记录每次发送需要发送的次数
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static uint32_t g_nTotalVehicleCount = 0;
static uint32_t g_nNormalVehicleCount = 0;
static uint32_t g_nReverseVehicleCount = 0;

static uint16_t g_nMagnetResetCounter = 0;
#define MAGNET_RESET_COUNT_LIMIT        1600    //80HZ * 20s = 1600, 表示连续20秒有车时对地磁进行复位为无车状态

static void parseReceivedMsg();
static void forcedTransPacket(mrfiPacket_t* msg);
static void composeTransPacket(uint8_t operation_id, addr_t target_addr);
static void configNwkParam();
static void changeDevState(uint8_t state, uint32_t time, uint8_t rf_state);
static void resetDev();

void executeAtTransmitPacketISR_MRFI() {
	PMS_SET(FLAG_SENT_PACKAGE);
}

void executeAtReceiveSyncISR_MRFI() {
	g_rf_communication_state = TRUE;
}

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
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

void executeAtRFFinishedISR_MRFI() {
	g_rf_communication_state = FALSE;
}

void executeAtTimeUp_TimerA()   {
        if(g_timer_state != WOR_FOR_ACK && g_timer_state != WOR_FOR_CMD && g_rf_communication_state == FALSE)
        {
                PMS_SET(FLAG_HEARTBEAT_TIMER_DUE);
        }
        else
        {
                //startTimer_TimerA(DELAY_HEARTBEAT_TIME_INTERVAL, executeAtTimeUp_TimerA);       //2014.06.08:   liusf debug to find heatbeat will lead to system dead,so comment it off, wait to be tested.
        }
}

void executeAtTimeUp_TimerB() {
	if(g_timer_state == WOR_FOR_ACK){
		PMS_SET(FLAG_WAIT_ACK_OVERTIME);
                turnOffLeds_LED();
                turnOnLed_LED(LED1);        //闪红灯
                turnOnLed_LED(LED2);        //闪黄灯
	} else if (g_timer_state == WOR_FOR_CMD) {
		PMS_SET(FLAG_WAIT_CMD_TIMER_DUE);
                turnOffLeds_LED();
                turnOnLed_LED(LED2);        //闪黄灯
	} else if(g_timer_state == SLEEP) {
		PMS_SET(FLAG_SLEEP_TIMER_DUE);
                turnOffLeds_LED();
                turnOnLed_LED(LED3);        //闪蓝灯
	}
}

void executeAtUpdateMagValue() {
	PMS_SET(FLAG_GEOMAGNETIC_DATA_PREPARED);
}

void main() {
        //uint16_t nDebugTest = 0;
        
	/* Board Init */
	initBSP_BSP();
	initLeds_LED();
	//TODO caohua 将这个初始化放到读取电压值的时候，采集完电压要将AD关掉
	initPower_BSP();
        
	init_system_flash_rw();
        
//	mac_addr[0] = 0x10;     //0x01表示协调器、0x10~0x8F表示传感器（原来为01)，0x90~0xFF表示灯节点
//	mac_addr[1] = 0x01;
//	mac_addr[2] = 0x01;     //传感器不断加1的1地址值，烧写传感器需要修改的字节地址
//	write_system_param(PARAM_TYPE_MAC, mac_addr, 3);
//
//	g_nwk_param.ip_addr.addr[0] = 0x01;
//	g_nwk_param.ip_addr.addr[1] = 0x03;     //传感器节点IP地址以0x03结尾，灯节点与传感器节点IP地址相同
//	g_nwk_param.channel = 0x4;
//	g_nwk_param.tx_power = TXPOWER;
//	g_nwk_param.father_addr.addr[0] = 0x01;
//	g_nwk_param.father_addr.addr[1] = 0x01; //协调器节点IP地址以0x01结尾
//	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
//        
//	g_mag_param.nHighThresholdVal = 20;//70;
//	g_mag_param.nLowThresholdVal = -20;//5;
//	g_mag_param.nIncFluctuationLimit = 3;
//	g_mag_param.nDecFluctuationLimit = 3;
//        g_mag_param.nIsMagnetRunning = 1;
//        g_mag_param.nNormalTrafficCount = 0;
//        g_mag_param.nReverseTrafficCount = 0;
//        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
//
//	while(1);
        
	if(read_system_param(PARAM_TYPE_MAC, mac_addr, 3) != SUCCESS){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}

	if(read_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t)) != SUCCESS){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}
        /*      如为魔术字符串"NJRTGS"，则初始化网络参数和地磁参数 */
        if(!memcmp((uint8_t *)(&g_nwk_param), FLASH_NETWORK_PARAMS_MAGIC_STRING, FLASH_NETWORK_PARAMS_MAGIC_STRING_LEN))        //对于从量产得到的Flash需要对该传感器的IP地址和网络参数进行设置
        {
                g_nwk_param.ip_addr.addr[0] = 0x01;
                g_nwk_param.ip_addr.addr[1] = 0x03;     //传感器节点IP地址以0x03结尾，灯节点与传感器节点IP地址相同
                g_nwk_param.channel = 0x0;
                g_nwk_param.tx_power = TXPOWER;
                g_nwk_param.father_addr.addr[0] = 0x01;
                g_nwk_param.father_addr.addr[1] = 0x01; //协调器节点IP地址以0x01结尾
                write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
                
                g_mag_param.nHighThresholdVal = 20;//70;
                g_mag_param.nLowThresholdVal = -20;//5;
                g_mag_param.nIncFluctuationLimit = 3;
                g_mag_param.nDecFluctuationLimit = 3;
                g_mag_param.nIsMagnetRunning = 0;
                g_mag_param.nNormalTrafficCount = 0;
                g_mag_param.nReverseTrafficCount = 0;
                write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
        }
        
        g_u16BatteryVoltage = getVoltage_BSP();
        
	//g_led_channel = g_nwk_param.channel + 1;
	init_MRFI(g_nwk_param.tx_power, g_nwk_param.channel, g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
        
	if(init_mag_sensor() < 0){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}

	if(read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type)) != SUCCESS){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}

	set_threshold_values(0xFF, g_mag_param.nHighThresholdVal, g_mag_param.nLowThresholdVal, g_mag_param.nIncFluctuationLimit, g_mag_param.nDecFluctuationLimit);
	suspend_hmc5883l();

	//TODO caohua 将地磁传感器设置为standby模式，不工作
	showVersion_LED(TMS_VERSION);
	ENABLE_INTERRUPTS_SGR();
        
        if(g_mag_param.nIsMagnetRunning)
        {
                resume_hmc5883l();
                reset_background_magnetic();
        }

	g_transmit_msg.frameLength = PACKET_LEN - 3;
	turnOnWAR_MRFI();
	startTimer_TimerB(WAIT_CMD_TIME ,executeAtTimeUp_TimerB);
	g_timer_state = WOR_FOR_CMD;
        
        g_n_abandoned_magnet_value_flag = 0;
        
        //startTimer_TimerA(HEARTBEAT_TIME_INTERVAL, executeAtTimeUp_TimerA);   //2014.06.08:   liusf debug to find heatbeat will lead to system dead,so comment it off, wait to be tested.
        
	//TODO caohua 执行到这里的时候地磁有中断触发，是个bug，暂时先将tmsFlags给清掉，之后需要彻底解决这个bug
	pmsFlags = 0;
        
        g_nTransmitCount = MAX_TRANSMIT_TIMES;
        
        g_nTotalVehicleCount = g_mag_param.nNormalTrafficCount;
        g_nNormalVehicleCount = g_mag_param.nNormalTrafficCount;
        g_nReverseVehicleCount = g_mag_param.nReverseTrafficCount;
        
	while(1) {
                DISABLE_WDT;    //2015.02.01:liusf add watchdog function
		LPM3;
		__no_operation();
                
                /*srand(g_nGlobalTime);
                nDebugTest = rand()%5000 + 100;
                nDebugTest += 1;*/
                
                
                
                /*收到来自RF的数据包*/
		if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
			stopTimer_TimerB();
			parseReceivedMsg();
		}

		if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
			PMS_CLEAR(FLAG_SENT_PACKAGE);
                        
                        //TODO caohua 发送心跳和报告错误也需要进WOR
			//2014.04.14:   liusf modified, 由于目前发送完数据包后，系统不需要等待协调器和灯节点的应答，因此直接进入睡眠状态就行
                        /*
			uint8_t transmit_cmd = g_transmit_msg.frame[2];
                        
                        if(transmit_cmd == OP_UPLOAD_STATUS_DATA	\
					|| transmit_cmd == OP_CONFIG_LED){
				changeDevState(WOR_FOR_ACK, WAIT_ACK_TIME, 1);
			} else {
				changeDevState(SLEEP, SLEEP_TIME, 0);
			}
                        */
                        changeDevState(SLEEP, SLEEP_TIME, 0);
                        //2014.04.14:   liusf modified end
    
			if(g_opt_after_sending_packet & CHANGE_CHANNEL){
				g_opt_after_sending_packet &= ~CHANGE_CHANNEL;
				changeChannelNum_MRFI(g_nwk_param.channel);
			}

			if(g_opt_after_sending_packet & RESET_MAG_SENSOR) {
				g_opt_after_sending_packet &=  ~RESET_MAG_SENSOR;
				reset_background_magnetic();
				g_cur_state = 0;
			}

			if(g_opt_after_sending_packet & RESET_END_DEV){
				g_opt_after_sending_packet &= ~RESET_END_DEV;
				resetDev();
			}
		}

		if(PMS_IS_SET(FLAG_WAIT_ACK_OVERTIME)){
			PMS_CLEAR(FLAG_WAIT_ACK_OVERTIME);
                        if(g_nTransmitCount > 0)
                        {
                                  forcedTransPacket(&g_transmit_msg);
                                  g_nTransmitCount--;
                        }
                        else
                        {
                                  stopTimer_TimerB();
                                  g_nTransmitCount = MAX_TRANSMIT_TIMES;
                        }
		}

		if(PMS_IS_SET(FLAG_WAIT_CMD_TIMER_DUE)){
			PMS_CLEAR(FLAG_WAIT_CMD_TIMER_DUE);
			changeDevState(SLEEP, SLEEP_TIME, 0);
		}

		if(PMS_IS_SET(FLAG_SLEEP_TIMER_DUE)){
			PMS_CLEAR(FLAG_SLEEP_TIMER_DUE);
			changeDevState(WOR_FOR_CMD, WAIT_CMD_TIME, 1);
		}

		/*地磁传感器有新数据*/
		if(PMS_IS_SET(FLAG_GEOMAGNETIC_DATA_PREPARED)) {
			PMS_CLEAR(FLAG_GEOMAGNETIC_DATA_PREPARED);
                        
                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
                        
                        if(g_n_abandoned_magnet_value_flag == 0)
                        {
                            //getValue(&g_is_car_parked, geomagnetic_raw_data, RAW_DATA_LEN);
                            getValue(&g_cur_fluc_state, geomagnetic_raw_data, RAW_DATA_LEN);
                            
                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                            
                            if(g_cur_fluc_state == FLUCTUATION_NONE)    //当前地磁值无波动
                            {
                              g_cur_fluc_state = FLUCTUATION_NONE;
                              g_prev_fluc_state = FLUCTUATION_NONE;
                              g_n_abandoned_magnet_value_flag = 0;
                            }
                            else if(g_cur_fluc_state == FLUCTUATION_UPWARD)
                            {
                              if(g_prev_fluc_state == FLUCTUATION_NONE)     //无波动到正向波动表示正向车流量通过
                              {
                                g_nMagnetResetCounter = 0;
                                g_n_abandoned_magnet_value_flag = 1;
                                startTimer_TimerA(ABADONED_MAGNET_DATA_TIMEOUT, executeAtAbandonedMagnetDataTimeout);
                                if(g_rf_communication_state != TRUE)
                                {
                                        g_cur_state = 0x01;
                                        stopTimer_TimerB();
                                        
                                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
                                        DISABLE_INTERRUPTS_SGR();
                                        read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        g_nTotalVehicleCount += 1;  //车流量总数增1
                                        g_nNormalVehicleCount += 1; //正向车流量增1
                                        g_mag_param.nNormalTrafficCount = g_nNormalVehicleCount;
                                        g_mag_param.nReverseTrafficCount = g_nReverseVehicleCount;
                                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        ENABLE_INTERRUPTS_SGR();
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        changeChannelNum_MRFI(g_nwk_param.channel);
                                        for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                                        {
                                            //给协调器发送数据包
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                            composeTransPacket(OP_UPLOAD_STATUS_DATA, g_nwk_param.father_addr);
                                            forcedTransPacket(&g_transmit_msg);
                                            twinkleLed_LED(LED3, 1, 100);        //闪灯
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        }
                                        g_prev_fluc_state = FLUCTUATION_UPWARD;
                                        //切换系统状态为SLEEP状态
                                        changeDevState(SLEEP, SLEEP_TIME, 0);
                                }
                                g_prev_fluc_state = FLUCTUATION_UPWARD;
                              }
                              else if(g_prev_fluc_state == FLUCTUATION_UPWARD)  //连续正向波动
                              {
                                g_nMagnetResetCounter += 1;
                                if(g_nMagnetResetCounter >= 400)
                                {
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        g_nMagnetResetCounter = 0;
                                        reset_background_magnetic();
                                        g_cur_state = 0;
                                        twinkleLed_LED(LED1, 1, 100);        //闪红灯
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        g_prev_fluc_state = FLUCTUATION_NONE;
                                        g_cur_fluc_state = FLUCTUATION_NONE;
                                        g_n_abandoned_magnet_value_flag = 0;
                                }
                              }
                              else
                              {
                                    
                              }
                            }
                            else if(g_cur_fluc_state == FLUCTUATION_DOWNWARD)
                            {
                              if(g_prev_fluc_state == FLUCTUATION_NONE)     //无波动到逆向波动表示负向车流量通过
                              {
                                g_nMagnetResetCounter = 0;
                                g_n_abandoned_magnet_value_flag = 1;
                                startTimer_TimerA(ABADONED_MAGNET_DATA_TIMEOUT, executeAtAbandonedMagnetDataTimeout);
                                /*如果1101处于RX或者TX状态时车位状态发生变化，则等RX或TX结束后再发送*/
                                if(g_rf_communication_state != TRUE)
                                {
                                        g_cur_state = 0x02;
                                        stopTimer_TimerB();
                                        
                                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
                                        DISABLE_INTERRUPTS_SGR();
                                        read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        g_nTotalVehicleCount += 1;  //车流量总数增1
                                        g_nReverseVehicleCount += 1;        //逆向车流量增1
                                        g_mag_param.nNormalTrafficCount = g_nNormalVehicleCount;
                                        g_mag_param.nReverseTrafficCount = g_nReverseVehicleCount;
                                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        ENABLE_INTERRUPTS_SGR();
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        changeChannelNum_MRFI(g_nwk_param.channel);
                                        for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                                        {
                                            //给协调器发送数据包
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                            composeTransPacket(OP_UPLOAD_STATUS_DATA, g_nwk_param.father_addr);
                                            forcedTransPacket(&g_transmit_msg);
                                            twinkleLed_LED(LED3, 1, 100);        //闪灯
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        }
                                        g_prev_fluc_state = FLUCTUATION_DOWNWARD;
                                        //切换系统状态为SLEEP状态
                                        changeDevState(SLEEP, SLEEP_TIME, 0);
                                }
                                g_prev_fluc_state = FLUCTUATION_DOWNWARD;
                              }
                              else if(g_cur_fluc_state == FLUCTUATION_DOWNWARD)
                              {
                                g_nMagnetResetCounter += 1;
                                if(g_nMagnetResetCounter >= 400)
                                {
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        g_nMagnetResetCounter = 0;
                                        reset_background_magnetic();
                                        g_cur_state = 0;
                                        twinkleLed_LED(LED1, 1, 100);        //闪红灯
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        g_prev_fluc_state = FLUCTUATION_NONE;
                                        g_cur_fluc_state = FLUCTUATION_NONE;
                                        g_n_abandoned_magnet_value_flag = 0;
                                }
                              }
                              else
                              {
                                
                              }
                            }
                            else
                            {
                            }
                            
                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                        }
                }
                
                /* 发送心跳数据包      */
                if(PMS_IS_SET(FLAG_HEARTBEAT_TIMER_DUE)) {
                        PMS_CLEAR(FLAG_HEARTBEAT_TIMER_DUE);
                        g_u16BatteryVoltage = getVoltage_BSP();
                        for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                        {
                          composeTransPacket(OP_END_HEARTBEAT, g_nwk_param.father_addr);
                          forcedTransPacket(&g_transmit_msg);
                        }
                }
	}
}

/**
 * @brief 分析收到的数据包
 */
void parseReceivedMsg() {
	uint8_t operation_type = g_receive_msg.frame[2];
	switch(operation_type) {
	case OP_ACK_END:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		if(g_receive_msg.frame[3] == 0x01){
                #ifdef CONFIG_LED_STATUS
                        //2014.04.20: liusf comment it off, 现在修改为传感器直接给协调器发送，然后切换信道给灯发送，不用等待协调器应答的机制
                        /*
                        changeChannelNum_MRFI(g_led_channel);
                        composeTransPacket(OP_CONFIG_LED, g_nwk_param.ip_addr);
                        toggleLed_LED(LED3);
                        forcedTransPacket(&g_transmit_msg);
                        toggleLed_LED(LED3);
                        */
                #endif
		}else if(g_receive_msg.frame[3] == 0x03){       //应答心跳
                        startTimer_TimerA(HEARTBEAT_TIME_INTERVAL, executeAtTimeUp_TimerA);     //2014.06.08:   liusf debug to find heatbeat will lead to system dead,so comment it off, wait to be tested.
                }else {
			changeDevState(SLEEP, SLEEP_TIME, 0);
		}
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_ACK_CONFIG_LED:
                #ifdef CONFIG_LED_STATUS
                        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                        changeChannelNum_MRFI(g_nwk_param.channel);
                        changeDevState(SLEEP, SLEEP_TIME, 0);
                        FEED_WDT;       //2015.02.01:liusf add watchdog function
                #endif
		break;
	case OP_RESET_MAG_PARAM:
		/*先回ACK给配置器，因为传感器复位时间可能比较长*/
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		g_opt_after_sending_packet |= RESET_MAG_SENSOR;
		composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_CONFIG_MAG_THR:{
                int16_t nHighThresholdValParam = 0;
                int16_t nLowThresholdValParam = 0;
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                nHighThresholdValParam = ((int16_t)g_receive_msg.frame[4] << 8) + g_receive_msg.frame[5];
                nLowThresholdValParam = ((int16_t)g_receive_msg.frame[6] << 8) + g_receive_msg.frame[7];
		//g_mag_param.nHighThresholdVal = ((int16_t)g_receive_msg.frame[4] << 8) + g_receive_msg.frame[5];
		//g_mag_param.nLowThresholdVal = ((int16_t)g_receive_msg.frame[6] << 8) + g_receive_msg.frame[7];
                //if(g_mag_param.nHighThresholdVal != 0 && g_mag_param.nLowThresholdVal != 0)
                if(nHighThresholdValParam != 0 && nLowThresholdValParam != 0)
                {
                      g_mag_param.nHighThresholdVal = ((int16_t)g_receive_msg.frame[4] << 8) + g_receive_msg.frame[5];
                      g_mag_param.nLowThresholdVal = ((int16_t)g_receive_msg.frame[6] << 8) + g_receive_msg.frame[7];
                      set_threshold_values(g_receive_msg.frame[3], g_mag_param.nHighThresholdVal, g_mag_param.nLowThresholdVal, g_mag_param.nIncFluctuationLimit, g_mag_param.nDecFluctuationLimit);
                      DISABLE_INTERRUPTS_SGR();
                      if(write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type)) != SUCCESS){
                              //TODO caohua report error
                      } else {
                              composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
                      }
                      ENABLE_INTERRUPTS_SGR();
                }
                else
                {
                #if 0
                      if(read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type)) != SUCCESS){
                              //TODO caohua report error
                      } else {
                              g_transmit_msg.frame[4] = (uint8_t)((g_mag_param.nHighThresholdVal & 0xFF00) >> 8);
                              g_transmit_msg.frame[5] = (uint8_t)(g_mag_param.nHighThresholdVal & 0xFF);
                              g_transmit_msg.frame[6] = (uint8_t)((g_mag_param.nLowThresholdVal & 0xFF00) >> 8);
                              g_transmit_msg.frame[7] = (uint8_t)(g_mag_param.nLowThresholdVal & 0xFF);
                              composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
                      }
                #else
                      g_transmit_msg.frame[4] = (uint8_t)((g_mag_param.nHighThresholdVal & 0xFF00) >> 8);
                      g_transmit_msg.frame[5] = (uint8_t)(g_mag_param.nHighThresholdVal & 0xFF);
                      g_transmit_msg.frame[6] = (uint8_t)((g_mag_param.nLowThresholdVal & 0xFF00) >> 8);
                      g_transmit_msg.frame[7] = (uint8_t)(g_mag_param.nLowThresholdVal & 0xFF);
                      composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
                #endif
                }
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
	}
		break;
	case OP_SET_MAG_SENSOR_MODE:{
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		uint8_t mode = g_receive_msg.frame[3];
                //2014.01.22:   liusf modified to accelarate resume speed，通过实际测试表明必须在先对地磁传感器进行操作后再对射频进行控制，否则会出现地磁初始化过程中采集车位信息异常导致系统不能正常运行的问题
                //composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
                //forcedTransPacket(&g_transmit_msg);
                //2014.01.22:   liusf modified end
		if(mode == 0){
			suspend_hmc5883l();
                        
                        read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                        g_mag_param.nIsMagnetRunning = 0;
                        DISABLE_INTERRUPTS_SGR();
                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                        ENABLE_INTERRUPTS_SGR();
                        
			//TODO caohua 停止心跳
		} else if(mode == 1){
			resume_hmc5883l();
                        //2013.12.26:   liusf add for magnet sample debug
                        reset_background_magnetic();
                        //2013.12.26:   liusf add end
                        
                        read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                        g_mag_param.nIsMagnetRunning = 1;
                        DISABLE_INTERRUPTS_SGR();
                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                        ENABLE_INTERRUPTS_SGR();
		}
		composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
	}
		break;
	case OP_RESET_END_DEV:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		g_opt_after_sending_packet |= RESET_END_DEV;
                suspend_hmc5883l();
		composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
	case OP_CONFIG_END_NWK_PARAM:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		configNwkParam();
                FEED_WDT;       //2015.02.01:liusf add watchdog function
		break;
        case OP_RETRIVE_SENSOR_STATUS:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                /*addr_t coordDevAddr;
                coordDevAddr.addr[0] = g_receive_msg.frame[0];
                coordDevAddr.addr[1] = g_receive_msg.frame[1];*/
                composeTransPacket(OP_UPLOAD_STATUS_DATA, *(addr_t *)(&(g_receive_msg.frame)));
                forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                break;
        case OP_CONFIG_MAG_SAMPLE_RATE:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                if(g_receive_msg.frame[3] == CONFIG_ODR_0dot75HZ || 
                   g_receive_msg.frame[3] == CONFIG_ODR_1dot5HZ || 
                   g_receive_msg.frame[3] == CONFIG_ODR_3HZ ||
                   g_receive_msg.frame[3] == CONFIG_ODR_7dot5HZ ||
                   g_receive_msg.frame[3] == CONFIG_ODR_15HZ ||
                   g_receive_msg.frame[3] == CONFIG_ODR_30HZ ||
                   g_receive_msg.frame[3] == CONFIG_ODR_75HZ)
                {
                    g_current_magnet_odr = g_receive_msg.frame[3];
                }
                else
                {
                    g_current_magnet_odr = CONFIG_ODR_75HZ;
                }
                set_hmc5883l_odr(g_current_magnet_odr);
                composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		forcedTransPacket(&g_transmit_msg);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
                break;
        case OP_RESET_TRAFFIC_VALUE:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
                g_nTotalVehicleCount = 0;
                //给协调器发送流量清0后的射频数据包
                composeTransPacket(OP_UPLOAD_STATUS_DATA, g_nwk_param.father_addr);
                disable_hmc5883l_int();
                forcedTransPacket(&g_transmit_msg);
                enable_hmc5883l_int();
                delayInMs_BSP(100);
                FEED_WDT;       //2015.02.01:liusf add watchdog function
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
        uint32_t nDelayTime = 0;
	startTimerAsDelaying_TimerB(5);
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
	LPM3;

	do{
		status = transmitPacket_MRFI(msg);
                
                turnOffLeds_LED();
                turnOnLed_LED(LED1);        //闪红灯
                
                /*if transmit failed, delay 50ms, retry  to transmit*/
		if(status == FAILURE) {
                        //add delay time by compute delay
                        srand(g_nGlobalTime);
			/*delay 50ms*/
			//startTimerAsDelaying_TimerB(100);
                        nDelayTime = rand()%500 + 100;
                        
			/*turn off RF in order to save power*/
			turnOffRadio_MRFI();
                        //startTimerAsDelaying_TimerB((rand()%5000 + 100));
                        startTimerAsDelaying_TimerB(nDelayTime);
                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
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
	case OP_UPLOAD_STATUS_DATA:
		  g_transmit_msg.frame[3] = g_cur_state;
                  g_transmit_msg.frame[4] = (uint8_t)((g_u16BatteryVoltage >> 4) & 0xFF);
                  /*    增加交通流量计数器   */
                  g_transmit_msg.frame[5] = (uint8_t)(g_nNormalVehicleCount & 0xFF);
                  g_transmit_msg.frame[6] = (uint8_t)((g_nNormalVehicleCount >> 8) & 0xFF);
                  g_transmit_msg.frame[7] = (uint8_t)((g_nNormalVehicleCount >> 16) & 0xFF);
                  g_transmit_msg.frame[8] = (uint8_t)((g_nNormalVehicleCount >> 24) & 0xFF);
                  g_transmit_msg.frame[9] = (uint8_t)(g_nReverseVehicleCount & 0xFF);
                  g_transmit_msg.frame[10] = (uint8_t)((g_nReverseVehicleCount >> 8) & 0xFF);
                  g_transmit_msg.frame[11] = (uint8_t)((g_nReverseVehicleCount >> 16) & 0xFF);
                  g_transmit_msg.frame[12] = (uint8_t)((g_nReverseVehicleCount >> 24) & 0xFF);
		break;
	case OP_CONFIG_LED:
		g_transmit_msg.frame[3] = g_cur_state;
		break;
	case OP_END_ACK_CONFIG:{
		uint8_t operation_type = g_receive_msg.frame[2];
		switch(operation_type){
		case OP_RESET_END_DEV:
			g_transmit_msg.frame[3] = 0x01;
			break;
		case OP_CONFIG_END_NWK_PARAM:
			g_transmit_msg.frame[3] = 0x02;
			g_transmit_msg.frame[4] = mac_addr[0];
			g_transmit_msg.frame[5] = mac_addr[1];
			g_transmit_msg.frame[6] = mac_addr[2];
			g_transmit_msg.frame[7] = g_nwk_param.ip_addr.addr[0];
			g_transmit_msg.frame[8] = g_nwk_param.ip_addr.addr[1];
			g_transmit_msg.frame[9] = g_nwk_param.channel;
			g_transmit_msg.frame[10] = g_nwk_param.tx_power;
			g_transmit_msg.frame[11] = g_nwk_param.father_addr.addr[0];
			g_transmit_msg.frame[12] = g_nwk_param.father_addr.addr[1];
			break;
		case OP_RESET_MAG_PARAM:
			g_transmit_msg.frame[3] = 0x03;
			break;
		case OP_CONFIG_MAG_THR:
			g_transmit_msg.frame[3] = 0x04;
			break;
		case OP_SET_MAG_SENSOR_MODE:
			g_transmit_msg.frame[3] = 0x05;
			break;
                case OP_CONFIG_MAG_SAMPLE_RATE:
                        g_transmit_msg.frame[3] = 0x06;
                        g_transmit_msg.frame[4] = g_current_magnet_odr;
                        break;
		default :
			break;
		}
	}
		break;
        case OP_END_HEARTBEAT:  //不需要添加额外是数据
                g_transmit_msg.frame[3] = g_cur_state;
                
                break;
        case OP_RESERVE_PARKING:
                g_transmit_msg.frame[3] = 0x1;  //表示预定车位信息，0x1表示预定车位，0x0表示取消预定车位
                break;
        case OP_ACK_RESERVE_PARKING:
                g_transmit_msg.frame[3] = 0x0;  //表示预定车位或者取消车位预定成功
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
		if(write_system_param(PARAM_TYPE_MAC, mac_addr, 3) != SUCCESS){
			//TODO caohua report error to coordinator
		} else {
			composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		}
		forcedTransPacket(&g_transmit_msg);
		return;
	}

	/*MAC地址不匹配，直接返回*/
	if (mac_addr[0] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR]
			|| mac_addr[1] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 1]
			|| mac_addr[2] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 2]){
		changeDevState(SLEEP, SLEEP_TIME, 0);
		return;
	}

	/*配置节点IP地址*/
	if(cmd_byte & 0x02){
		g_nwk_param.ip_addr.addr[0] = g_receive_msg.frame[INDEX_NODE_HIGH_ADDR];
		g_nwk_param.ip_addr.addr[1] = g_receive_msg.frame[INDEX_NODE_LOW_ADDR];
		changeNodeAddr_MRFI(g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
	}

	/*配置节点信道号*/
	if(cmd_byte & 0x04){
		g_nwk_param.channel = g_receive_msg.frame[INDEX_CHANNEL_NUM];
		//g_led_channel = g_nwk_param.channel + 1;
		/*收到切信道的命令后先回ACK，然后再切*/
		g_opt_after_sending_packet |= CHANGE_CHANNEL;
	}

	/*配置发送强度*/
	if(cmd_byte & 0x08){
		g_nwk_param.tx_power = g_receive_msg.frame[INDEX_TX_POWER];
		changeTxPower_MRFI(g_nwk_param.tx_power);
	}

	if(cmd_byte & 0x10){
		g_nwk_param.father_addr.addr[0] = g_receive_msg.frame[INDEX_FATHER_HIGH_ADDR];
		g_nwk_param.father_addr.addr[1] = g_receive_msg.frame[INDEX_FATHER_LOW_ADDR];
	}

	if(cmd_byte != 0x00){
		if(write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t)) != SUCCESS){
			//TODO caohua report error to coordinator
		}
	}
	/*发送当前mrfi的参数作为ACK，也可以作为回复cmd_byte = 0x00时的应答包*/
	composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
	forcedTransPacket(&g_transmit_msg);
}

/**
 * @brief Enter Sleep state， CC1101 is off, SleepTimer starts, MSP430 will enter LPM3
 */
void changeDevState(uint8_t state, uint32_t time, uint8_t rf_state) {
	g_timer_state = state;
        //2014.06.08:liusf add for dead debug
        //startTimerAsDelaying_TimerB(37);
        //stopTimer_TimerB();
        //2014.06.08:liusf add end
	startTimer_TimerB(time ,executeAtTimeUp_TimerB);
	if(rf_state == 1){
                DISABLE_MRFI_INTR();    //2014.06.08: liusf add for dead debug 
		turnOnWAR_MRFI();
                ENABLE_MRFI_INTR();     //2014.06.08: liusf add for dead debug 
	} else {
		turnOffRadio_MRFI();
                //2014.06.08:liusf add for dead debug, it is verified to be invalid
                //LPM3;
                //2014.06.08:liusf add end
	}
}

/**
 * @brief Reset End device, including tx_power, channel, parking state, etc.
 */
void resetDev(){
	g_cur_state = 0;
	g_nwk_param.channel = CHANNEL;
	//g_led_channel = g_nwk_param.channel + 1;
	g_nwk_param.tx_power = TXPOWER;
	g_opt_after_sending_packet = 0;
	//suspend_hmc5883l();
        DISABLE_WDT;
	write_system_param(PARAM_TYPE_NETWORK_PARAMS, (uint8_t *)(&g_nwk_param), sizeof(nwkParam_t));
        FEED_WDT;
	changeTxPower_MRFI(g_nwk_param.tx_power);
	changeChannelNum_MRFI(g_nwk_param.channel);
	changeDevState(SLEEP, SLEEP_TIME, 0);
}
