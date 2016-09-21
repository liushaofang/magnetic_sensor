/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the main program for SimpleTest TX.
 *
 * PACKET:30BYTES，发一个包耗时49.5ms
 * 通信1000次耗时：1000 * 49.6 + (1000 / 10) * 200 = 69.6ms
 *
 * @author CaoHua
 *
 */

#include <string.h>
#include "MassProduction.h"
#include "hmc5883l.h"

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
#define FLAG_RCV_UART_DATA                                      8

#define PACKET_FRAME_LENGTH                              4
#define PACKET_FRAME_LENGTH                               4

#define UART_MSG_LEN		        12
static uint8_t uartMsg[UART_MSG_LEN];

static uint16_t pmsFlags = 0;
#define PMS_IS_SET(x)  (pmsFlags & (0x01 << x))
#define PMS_CLEAR(x)  (pmsFlags &= ~(0x01 << x))
#define PMS_SET(x)    (pmsFlags |= (0x01 << x))

static mrfiPacket_t g_transmit_msg, g_receive_msg;
static volatile uint8_t g_is_car_parked = 0;

static void parseReceivedMsg();
static void forcedTransPacket(mrfiPacket_t* msg);

#ifdef HW_I2C_DEV
static uint8_t last_geomagnetic_raw_data[RAW_DATA_LEN];
static uint8_t current_geomagnetic_raw_data[RAW_DATA_LEN];
static uint8_t check_magnet_sensor_status();
#endif

static uint8_t get_magnet_sensor_test_need();

void executeAtReceivePacketISR_MRFI(const mrfiPacket_t *packet) {
	//在SimpleTest中，发送端没有接收包的可能性，因此此方法中不需要任何处理。
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

void executeAtUartISR_BSP(const uint8_t *uart_msg){
        memcpy(uartMsg, uart_msg, UART_MSG_LEN);
        PMS_SET(FLAG_RCV_UART_DATA);
}

void executeAtUpdateMagValue() {
	PMS_SET(FLAG_GEOMAGNETIC_DATA_PREPARED);
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

void main() {
	initBSP_BSP();
	//initUart_BSP();
	initLeds_LED();
	init_MRFI(TXPOWER, CHANNEL, ST_SLAVE_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
        
#ifdef HW_I2C_DEV
        if(init_mag_sensor() < 0){
		twinkleLed_LED(LED1, 1, 100);
		while(1);
	}
#endif
        
	showVersion_LED(3);
        ENABLE_INTERRUPTS_SGR();
        
	turnOnRX_MRFI();
        //turnOnWAR_MRFI();
        //startTimer_TimerB(WAIT_CMD_TIME ,executeAtTimeUp_TimerB);
        
#ifdef HW_I2C_DEV
        resume_hmc5883l();
        reset_background_magnetic();
#endif
        
        while(1)
        {
                LPM3;
                __no_operation();
                
                if(PMS_IS_SET(FLAG_RCV_PACKAGE)) {
			PMS_CLEAR(FLAG_RCV_PACKAGE);
                        twinkleLed_LED(LED2, 1, 100);    //闪绿灯
                        //DISABLE_INTERRUPTS_SGR();
                        //init_MRFI(TXPOWER, CHANNEL, ST_SLAVE_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
                        //ENABLE_INTERRUPTS_SGR();
			parseReceivedMsg();
		}
                
#if 0
                if(PMS_IS_SET(FLAG_SENT_PACKAGE)){
                        PMS_CLEAR(FLAG_SENT_PACKAGE);
                        twinkleLed_LED(LED3, 1, 50);
                        init_MRFI(TXPOWER, CHANNEL, ST_SLAVE_NODE_HIGH_ADDR, ST_NODE_LOW_ADDR);
                        turnOnRX_MRFI();
                }
#endif
                
           /*#ifdef HW_I2C_DEV
                if(PMS_IS_SET(FLAG_SENT_PACKAGE)) {
			PMS_CLEAR(FLAG_SENT_PACKAGE);
                        turnOffRadio_MRFI();    //关闭射频
                        suspend_hmc5883l();     //关闭地磁传感器
                }
          #endif*/
        }
}


/**
 * @brief 强制发送数据包，发送过程中如果发送失败，则延时50ms再发，直到发送完成为止
 * @param msg 要发送的数据包
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

/**
 * @brief 分析收到的数据包
 */
static void parseReceivedMsg() {
	uint8_t operation_type = g_receive_msg.frame[2];
	switch(operation_type) {
	case MASS_PRODUCTION_TEST_MRFI:
              /*
              memset(uartMsg, 0, sizeof(uartMsg));
              memcpy(uartMsg, &g_receive_msg.highAddr, UART_MSG_LEN - 1);
              uartMsg[UART_MSG_LEN - 1] = (uint8_t)(rssi_value_2_power(g_receive_msg.rxMetrics[0]));
              writeToUart_BSP(uartMsg, UART_MSG_LEN);
              */
#if 0
              memset(&g_transmit_msg, 0, sizeof(mrfiPacket_t));
              g_transmit_msg.frameLength = PACKET_LEN - 3;
              g_transmit_msg.highAddr = ST_MASTER_NODE_HIGH_ADDR;
              g_transmit_msg.lowAddr = ST_NODE_LOW_ADDR;
              g_transmit_msg.frame[0] = ST_SLAVE_NODE_HIGH_ADDR;
              g_transmit_msg.frame[1] = ST_NODE_LOW_ADDR;
              g_transmit_msg.frame[2] = MASS_PRODUCTION_TEST_MRFI_ACK;
              g_transmit_msg.frame[3] = get_magnet_sensor_test_need();
              //transmitPacket_MRFI(&g_transmit_msg);
              delayInMs_BSP(800);       //通过测试发现该延时必须添加，否则Master无法收到应答
              forcedTransPacket(&g_transmit_msg);
#endif
              
              LPM3;
              break;
        case MASS_PRODUCTION_TEST_MAGNET:
          #ifdef HW_I2C_DEV
              memset(&g_transmit_msg, 0, sizeof(mrfiPacket_t));
              g_transmit_msg.frameLength = PACKET_LEN - 3;
              g_transmit_msg.highAddr = ST_MASTER_NODE_HIGH_ADDR;
              g_transmit_msg.lowAddr = ST_NODE_LOW_ADDR;
              g_transmit_msg.frame[0] = ST_SLAVE_NODE_HIGH_ADDR;
              g_transmit_msg.frame[1] = ST_NODE_LOW_ADDR;
              g_transmit_msg.frame[2] = MASS_PRODUCTION_TEST_MAGNET_ACK;
              if(check_magnet_sensor_status() == MASS_PRODUCTION_CHECK_MAGNET_SUCCESS)
                      g_transmit_msg.frame[3] = MASS_PRODUCTION_TEST_PASS;
              else
                      g_transmit_msg.frame[3] = MASS_PRODUCTION_TEST_FAIL;
              //transmitPacket_MRFI(&g_transmit_msg);
              delayInMs_BSP(800);       //通过测试发现该延时必须添加，否则Master无法收到应答
              forcedTransPacket(&g_transmit_msg);
              LPM3;
          #endif
              break;
	default:
              break;
	}
}

#ifdef HW_I2C_DEV
static uint8_t get_magnet_sensor_test_need()
{
        return MASS_PRODUCTION_NEED_MAGNET_TEST;
}

static uint8_t check_magnet_sensor_status()
{
        uint8_t nCount = 0;
        
        int8_t nLastXLData = 0;
        int8_t nLastXHData = 0;
        int8_t nLastYLData = 0;
        int8_t nLastYHData = 0;
        int8_t nLastZLData = 0;
        int8_t nLastZHData = 0;
        
        int8_t nCurrentXLData = 0;
        int8_t nCurrentXHData = 0;
        int8_t nCurrentYLData = 0;
        int8_t nCurrentYHData = 0;
        int8_t nCurrentZLData = 0;
        int8_t nCurrentZHData = 0;
        
        int8_t nDeltaXLData = 0;
        int8_t nDeltaXHData = 0;
        int8_t nDeltaYLData = 0;
        int8_t nDeltaYHData = 0;
        int8_t nDeltaZLData = 0;
        int8_t nDeltaZHData = 0;
        
        int16_t nDeltaSum = 0;
        
        getValue(&g_is_car_parked, last_geomagnetic_raw_data, RAW_DATA_LEN);
        nLastXLData = last_geomagnetic_raw_data[0];
        nLastXHData = last_geomagnetic_raw_data[1];
        nLastYLData = last_geomagnetic_raw_data[2];
        nLastYHData = last_geomagnetic_raw_data[3];
        nLastZLData = last_geomagnetic_raw_data[4];
        nLastZHData = last_geomagnetic_raw_data[5];
        
        
        while(nCount < 5)
        {
              getValue(&g_is_car_parked, current_geomagnetic_raw_data, RAW_DATA_LEN);
              
              nCurrentXLData = current_geomagnetic_raw_data[0];
              nCurrentXHData = current_geomagnetic_raw_data[1];
              nCurrentYLData = current_geomagnetic_raw_data[2];
              nCurrentYHData = current_geomagnetic_raw_data[3];
              nCurrentZLData = current_geomagnetic_raw_data[4];
              nCurrentZHData = current_geomagnetic_raw_data[5];
              
              nDeltaXLData = (nCurrentXLData > nLastXLData)?(nCurrentXLData - nLastXLData):(nLastXLData - nCurrentXLData);
              nDeltaXHData = (nCurrentXHData > nLastXHData)?(nCurrentXHData - nLastXHData):(nLastXHData - nCurrentXHData);
              nDeltaYLData = (nCurrentYLData > nLastYLData)?(nCurrentYLData - nLastYLData):(nLastYLData - nCurrentYLData);
              nDeltaYHData = (nCurrentYHData > nLastYHData)?(nCurrentYHData - nLastYHData):(nLastYHData - nCurrentYHData);
              nDeltaZLData = (nCurrentZLData > nLastZLData)?(nCurrentZLData - nLastZLData):(nLastZLData - nCurrentZLData);
              nDeltaZHData = (nCurrentZHData > nLastZHData)?(nCurrentZHData - nLastZHData):(nLastZHData - nCurrentZHData);
              
              nDeltaSum += (nDeltaXLData + nDeltaXHData + nDeltaYLData + nDeltaYHData + nDeltaZLData + nDeltaZHData);
                
              nLastXLData = nCurrentXLData;
              nLastXHData = nCurrentXHData;
              nLastYLData = nCurrentYLData;
              nLastYHData = nCurrentYHData;
              nLastZLData = nCurrentZLData;
              nLastZHData = nCurrentZHData;
        }
        
        if(nDeltaSum <= 3 || nDeltaSum >= 100)
          return MASS_PRODUCTION_CHECK_MAGNET_FAIL;
        else
          return MASS_PRODUCTION_CHECK_MAGNET_SUCCESS;
}

#else
static uint8_t get_magnet_sensor_test_need()
{
        return MASS_PRODUCTION_NOT_NEED_MAGNET_TEST;
}
#endif