#include <string.h>
#include "PMSUtil.h"
#include "hmc5883l.h"
#include <stdlib.h>
#include "led.h"
#include "mrfi_spi.h"

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

#define RAW_DATA_LEN					6
#define WAIT_ACK_TIME			500
#define WAIT_CMD_TIME			500
#define SLEEP_TIME			5000

#define MAX_TRANSMIT_TIMES              5

#define MAX_TIMES_PER_TRANSMISSION      5
#define FORCE_TRANS_LIMIT               100

/*      ���End�ڵ������ʱ����HEARTBEAT_TIME_INTERVAL������Ϊ1Сʱ=3600000 */
#define HEARTBEAT_TIME_INTERVAL         20000    //3600000       //2014.06.08:liusf modified for dead debug
/*      ���End�ڵ��������������ʱ��ϵͳ�����ڷ���������״̬��Ҫ�ӳٷ���������ʱ����������Ϊ5����=5000 */
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

#define ABADONED_MAGNET_DATA_TIMEOUT       1000    //����5000ms�ڵĵش�����
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

static uint8_t g_nTransmitTimes = MAX_TIMES_PER_TRANSMISSION;    //��¼ÿ�η�����Ҫ���͵Ĵ���
static uint8_t g_nForceTransLimit = FORCE_TRANS_LIMIT;

static uint32_t g_nTotalVehicleCount = 0;
static uint32_t g_nNormalVehicleCount = 0;
static uint32_t g_nReverseVehicleCount = 0;

static uint16_t g_nMagnetResetCounter = 0;
#define MAGNET_RESET_COUNT_LIMIT        1600    //80HZ * 20s = 1600, ��ʾ����20���г�ʱ�ԵشŽ��и�λΪ�޳�״̬

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
                turnOnLed_LED(LED1);        //�����
                turnOnLed_LED(LED2);        //���Ƶ�
	} else if (g_timer_state == WOR_FOR_CMD) {
		PMS_SET(FLAG_WAIT_CMD_TIMER_DUE);
                turnOffLeds_LED();
                turnOnLed_LED(LED2);        //���Ƶ�
	} else if(g_timer_state == SLEEP) {
		PMS_SET(FLAG_SLEEP_TIMER_DUE);
                turnOffLeds_LED();
                turnOnLed_LED(LED3);        //������
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
	//TODO caohua �������ʼ���ŵ���ȡ��ѹֵ��ʱ�򣬲ɼ����ѹҪ��AD�ص�
	initPower_BSP();
        
	init_system_flash_rw();
        
//	mac_addr[0] = 0x10;     //0x01��ʾЭ������0x10~0x8F��ʾ��������ԭ��Ϊ01)��0x90~0xFF��ʾ�ƽڵ�
//	mac_addr[1] = 0x01;
//	mac_addr[2] = 0x01;     //���������ϼ�1��1��ֵַ����д��������Ҫ�޸ĵ��ֽڵ�ַ
//	write_system_param(PARAM_TYPE_MAC, mac_addr, 3);
//
//	g_nwk_param.ip_addr.addr[0] = 0x01;
//	g_nwk_param.ip_addr.addr[1] = 0x03;     //�������ڵ�IP��ַ��0x03��β���ƽڵ��봫�����ڵ�IP��ַ��ͬ
//	g_nwk_param.channel = 0x4;
//	g_nwk_param.tx_power = TXPOWER;
//	g_nwk_param.father_addr.addr[0] = 0x01;
//	g_nwk_param.father_addr.addr[1] = 0x01; //Э�����ڵ�IP��ַ��0x01��β
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
        /*      ��Ϊħ���ַ���"NJRTGS"�����ʼ����������͵شŲ��� */
        if(!memcmp((uint8_t *)(&g_nwk_param), FLASH_NETWORK_PARAMS_MAGIC_STRING, FLASH_NETWORK_PARAMS_MAGIC_STRING_LEN))        //���ڴ������õ���Flash��Ҫ�Ըô�������IP��ַ�����������������
        {
                g_nwk_param.ip_addr.addr[0] = 0x01;
                g_nwk_param.ip_addr.addr[1] = 0x03;     //�������ڵ�IP��ַ��0x03��β���ƽڵ��봫�����ڵ�IP��ַ��ͬ
                g_nwk_param.channel = 0x0;
                g_nwk_param.tx_power = TXPOWER;
                g_nwk_param.father_addr.addr[0] = 0x01;
                g_nwk_param.father_addr.addr[1] = 0x01; //Э�����ڵ�IP��ַ��0x01��β
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

	//TODO caohua ���شŴ���������Ϊstandbyģʽ��������
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
        
	//TODO caohua ִ�е������ʱ��ش����жϴ������Ǹ�bug����ʱ�Ƚ�tmsFlags�������֮����Ҫ���׽�����bug
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
                
                
                
                /*�յ�����RF�����ݰ�*/
		if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
			stopTimer_TimerB();
			parseReceivedMsg();
		}

		if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
			PMS_CLEAR(FLAG_SENT_PACKAGE);
                        
                        //TODO caohua ���������ͱ������Ҳ��Ҫ��WOR
			//2014.04.14:   liusf modified, ����Ŀǰ���������ݰ���ϵͳ����Ҫ�ȴ�Э�����͵ƽڵ��Ӧ�����ֱ�ӽ���˯��״̬����
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

		/*�شŴ�������������*/
		if(PMS_IS_SET(FLAG_GEOMAGNETIC_DATA_PREPARED)) {
			PMS_CLEAR(FLAG_GEOMAGNETIC_DATA_PREPARED);
                        
                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
                        
                        if(g_n_abandoned_magnet_value_flag == 0)
                        {
                            //getValue(&g_is_car_parked, geomagnetic_raw_data, RAW_DATA_LEN);
                            getValue(&g_cur_fluc_state, geomagnetic_raw_data, RAW_DATA_LEN);
                            
                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                            
                            if(g_cur_fluc_state == FLUCTUATION_NONE)    //��ǰ�ش�ֵ�޲���
                            {
                              g_cur_fluc_state = FLUCTUATION_NONE;
                              g_prev_fluc_state = FLUCTUATION_NONE;
                              g_n_abandoned_magnet_value_flag = 0;
                            }
                            else if(g_cur_fluc_state == FLUCTUATION_UPWARD)
                            {
                              if(g_prev_fluc_state == FLUCTUATION_NONE)     //�޲��������򲨶���ʾ��������ͨ��
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
                                        g_nTotalVehicleCount += 1;  //������������1
                                        g_nNormalVehicleCount += 1; //����������1
                                        g_mag_param.nNormalTrafficCount = g_nNormalVehicleCount;
                                        g_mag_param.nReverseTrafficCount = g_nReverseVehicleCount;
                                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        ENABLE_INTERRUPTS_SGR();
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        changeChannelNum_MRFI(g_nwk_param.channel);
                                        for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                                        {
                                            //��Э�����������ݰ�
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                            composeTransPacket(OP_UPLOAD_STATUS_DATA, g_nwk_param.father_addr);
                                            forcedTransPacket(&g_transmit_msg);
                                            twinkleLed_LED(LED3, 1, 100);        //����
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        }
                                        g_prev_fluc_state = FLUCTUATION_UPWARD;
                                        //�л�ϵͳ״̬ΪSLEEP״̬
                                        changeDevState(SLEEP, SLEEP_TIME, 0);
                                }
                                g_prev_fluc_state = FLUCTUATION_UPWARD;
                              }
                              else if(g_prev_fluc_state == FLUCTUATION_UPWARD)  //�������򲨶�
                              {
                                g_nMagnetResetCounter += 1;
                                if(g_nMagnetResetCounter >= 400)
                                {
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        g_nMagnetResetCounter = 0;
                                        reset_background_magnetic();
                                        g_cur_state = 0;
                                        twinkleLed_LED(LED1, 1, 100);        //�����
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
                              if(g_prev_fluc_state == FLUCTUATION_NONE)     //�޲��������򲨶���ʾ��������ͨ��
                              {
                                g_nMagnetResetCounter = 0;
                                g_n_abandoned_magnet_value_flag = 1;
                                startTimer_TimerA(ABADONED_MAGNET_DATA_TIMEOUT, executeAtAbandonedMagnetDataTimeout);
                                /*���1101����RX����TX״̬ʱ��λ״̬�����仯�����RX��TX�������ٷ���*/
                                if(g_rf_communication_state != TRUE)
                                {
                                        g_cur_state = 0x02;
                                        stopTimer_TimerB();
                                        
                                        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
                                        DISABLE_INTERRUPTS_SGR();
                                        read_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        g_nTotalVehicleCount += 1;  //������������1
                                        g_nReverseVehicleCount += 1;        //����������1
                                        g_mag_param.nNormalTrafficCount = g_nNormalVehicleCount;
                                        g_mag_param.nReverseTrafficCount = g_nReverseVehicleCount;
                                        write_system_param(PARAM_TYPE_MAGNET_ALG_PARAMS, (uint8_t *)(&g_mag_param), sizeof(Magnet_Alg_Param_Type));
                                        ENABLE_INTERRUPTS_SGR();
                                        FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        
                                        changeChannelNum_MRFI(g_nwk_param.channel);
                                        for(g_nTransmitTimes = 0; g_nTransmitTimes < MAX_TIMES_PER_TRANSMISSION; g_nTransmitTimes++)
                                        {
                                            //��Э�����������ݰ�
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                            composeTransPacket(OP_UPLOAD_STATUS_DATA, g_nwk_param.father_addr);
                                            forcedTransPacket(&g_transmit_msg);
                                            twinkleLed_LED(LED3, 1, 100);        //����
                                            FEED_WDT;    //2015.02.01:liusf add watchdog function
                                        }
                                        g_prev_fluc_state = FLUCTUATION_DOWNWARD;
                                        //�л�ϵͳ״̬ΪSLEEP״̬
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
                                        twinkleLed_LED(LED1, 1, 100);        //�����
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
                
                /* �����������ݰ�      */
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
 * @brief �����յ������ݰ�
 */
void parseReceivedMsg() {
	uint8_t operation_type = g_receive_msg.frame[2];
	switch(operation_type) {
	case OP_ACK_END:
                DISABLE_WDT;       //2015.02.01:liusf add watchdog function
		if(g_receive_msg.frame[3] == 0x01){
                #ifdef CONFIG_LED_STATUS
                        //2014.04.20: liusf comment it off, �����޸�Ϊ������ֱ�Ӹ�Э�������ͣ�Ȼ���л��ŵ����Ʒ��ͣ����õȴ�Э����Ӧ��Ļ���
                        /*
                        changeChannelNum_MRFI(g_led_channel);
                        composeTransPacket(OP_CONFIG_LED, g_nwk_param.ip_addr);
                        toggleLed_LED(LED3);
                        forcedTransPacket(&g_transmit_msg);
                        toggleLed_LED(LED3);
                        */
                #endif
		}else if(g_receive_msg.frame[3] == 0x03){       //Ӧ������
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
		/*�Ȼ�ACK������������Ϊ��������λʱ����ܱȽϳ�*/
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
                //2014.01.22:   liusf modified to accelarate resume speed��ͨ��ʵ�ʲ��Ա����������ȶԵشŴ��������в������ٶ���Ƶ���п��ƣ��������ֵشų�ʼ�������вɼ���λ��Ϣ�쳣����ϵͳ�����������е�����
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
                        
			//TODO caohua ֹͣ����
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
                //��Э��������������0�����Ƶ���ݰ�
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
 * @brief ǿ�Ʒ������ݰ������͹������������ʧ�ܣ�����ʱ50ms�ٷ���ֱ���������Ϊֹ
 * @param msg Ҫ���͵����ݰ�
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
                turnOnLed_LED(LED1);        //�����
                
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
 * @brief ���ɷ��͵İ�
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
                  /*    ���ӽ�ͨ����������   */
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
        case OP_END_HEARTBEAT:  //����Ҫ��Ӷ���������
                g_transmit_msg.frame[3] = g_cur_state;
                
                break;
        case OP_RESERVE_PARKING:
                g_transmit_msg.frame[3] = 0x1;  //��ʾԤ����λ��Ϣ��0x1��ʾԤ����λ��0x0��ʾȡ��Ԥ����λ
                break;
        case OP_ACK_RESERVE_PARKING:
                g_transmit_msg.frame[3] = 0x0;  //��ʾԤ����λ����ȡ����λԤ���ɹ�
                break;
	default:
		break;
	}
}

/**
 * @brief �������������͵İ�����Nwk�Ĳ���������Mac��ַ��Ip��ַ��father��ip��ַ
 */
void configNwkParam() {
	uint8_t cmd_byte = g_receive_msg.frame[INDEX_MASK];

	//TODO caohua �Ƿ���жϣ��޸�MAC��ַ��ʱ���յ��İ������IP��ַ������ʹ�㲥��ַ
	if(cmd_byte & 0x01){
		/*���յ���˵����ַУ��ͨ��*/
		for(uint8_t i = 0; i < 3; i++) {
			mac_addr[i] = g_receive_msg.frame[INDEX_MAC_BASE_ADDR + i];
		}
		/*�޸�MAC��ַ����˴β����������޸�������Ϣ*/
		if(write_system_param(PARAM_TYPE_MAC, mac_addr, 3) != SUCCESS){
			//TODO caohua report error to coordinator
		} else {
			composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
		}
		forcedTransPacket(&g_transmit_msg);
		return;
	}

	/*MAC��ַ��ƥ�䣬ֱ�ӷ���*/
	if (mac_addr[0] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR]
			|| mac_addr[1] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 1]
			|| mac_addr[2] != g_receive_msg.frame[INDEX_MAC_BASE_ADDR + 2]){
		changeDevState(SLEEP, SLEEP_TIME, 0);
		return;
	}

	/*���ýڵ�IP��ַ*/
	if(cmd_byte & 0x02){
		g_nwk_param.ip_addr.addr[0] = g_receive_msg.frame[INDEX_NODE_HIGH_ADDR];
		g_nwk_param.ip_addr.addr[1] = g_receive_msg.frame[INDEX_NODE_LOW_ADDR];
		changeNodeAddr_MRFI(g_nwk_param.ip_addr.addr[0], g_nwk_param.ip_addr.addr[1]);
	}

	/*���ýڵ��ŵ���*/
	if(cmd_byte & 0x04){
		g_nwk_param.channel = g_receive_msg.frame[INDEX_CHANNEL_NUM];
		//g_led_channel = g_nwk_param.channel + 1;
		/*�յ����ŵ���������Ȼ�ACK��Ȼ������*/
		g_opt_after_sending_packet |= CHANGE_CHANNEL;
	}

	/*���÷���ǿ��*/
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
	/*���͵�ǰmrfi�Ĳ�����ΪACK��Ҳ������Ϊ�ظ�cmd_byte = 0x00ʱ��Ӧ���*/
	composeTransPacket(OP_END_ACK_CONFIG, configDevAddr);
	forcedTransPacket(&g_transmit_msg);
}

/**
 * @brief Enter Sleep state�� CC1101 is off, SleepTimer starts, MSP430 will enter LPM3
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
