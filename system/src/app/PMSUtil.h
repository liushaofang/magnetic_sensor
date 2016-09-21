#ifndef __TMS_UTILS_H__
#define __TMS_UTILS_H__

//#include "SGR_CONFIG.h"

#include "Led.h"
#include "mrfi.h"
#include "TimerA.h"
#include "TimerB.h"
#include "Uart.h"
#include "Voltage.h"
#include "Flash.h"

#define TMS_VERSION 3

/**
 * -10db:34
 * 0db:60
 * 5db:84
 * 10db:C0
 */
//#define TXPOWER 0x34
//#define TXPOWER 0xC0
#define TXPOWER 0x60

/**
 * 1 - 20, 换算成16进制 0x01 - 0x14
 */
#define CHANNEL 0x00

#ifdef CONFIG_TMS_END

#define FATHER_HIGH_ADDR 0x01
#define FATHER_LOW_ADDR 0x01

#define NODE_HIGH_ADDR 0x01
#define NODE_LOW_ADDR 0x03

#elif defined CONFIG_TMS_COORDINATOR

#define FATHER_HIGH_ADDR 0x01
#define FATHER_LOW_ADDR 0x01

#define NODE_HIGH_ADDR 0x01
#define NODE_LOW_ADDR 0x01

#endif

#define FATHER_ADDR		{FATHER_HIGH_ADDR, FATHER_LOW_ADDR}
#define NODE_ADDR		{NODE_HIGH_ADDR, NODE_LOW_ADDR}
#define MAC_ADDR			{0x01, 0x01, 0x03}
#define CONFIG_ADDR     {0x09, 0x09}
#define COORDINATOR_ADD {0x01, 0x01}

/*Operation Code*/
#define OP_UPLOAD_STATUS_DATA				0x01
#define OP_REPORT_ERROR_CODE				0x02
#define OP_END_HEARTBEAT				0x03
#define OP_UPLOAD_RAW_MAGNETIC_DATA                     0x04

#define OP_ACK_END							0x11
#define OP_RESERVE_PARKING						0x12
#define OP_UPLOAD_RAW_MAGNETIC_DATA_TO_UART                             0x14

#define OP_CONFIG_LED									0x21

#define OP_ACK_CONFIG_LED						0x31
#define OP_ACK_RESERVE_PARKING			0x32

#define OP_RESET_END_DEV							0x41
#define OP_CONFIG_END_NWK_PARAM		0x42
#define OP_RESET_MAG_PARAM					0x43
#define OP_CONFIG_MAG_THR						0x44
#define OP_SET_MAG_SENSOR_MODE			0x45
#define OP_CONFIG_MAG_SAMPLE_RATE               0x46
#define OP_RESET_TRAFFIC_VALUE                  0x47

#define OP_STOP_SENSOR_HEARTBEAT                OP_SET_MAG_SENSOR_MODE

#define OP_END_ACK_CONFIG						0x51

#define OP_RESET_LED_DEV							0x61
#define OP_CONFIG_LED_NWK_PARAM		0x62

#define OP_LED_ACK_CONFIG						0x71

#define OP_MANAGE_END_LIST						0x81
#define OP_GET_END_RSSI_LIST					0x82
#define OP_CONFIG_COOR_NWK_PARAM	0x83
#define OP_RESET_COORDINATOR_DEV		0x84

#define OP_UPLOAD_END_LIST						0x91
#define OP_ACK_END_LIST_OPERATION		0x92
#define OP_UPLOAD_RSSI_LIST						0x93
#define OP_COOR_ACK_CONFIG					0x94

#define OP_CHANGE_CONFIG_DEV_CHAN				0xB1
#define OP_ACK_CHANGE_CONFIG_DEV_CHAN		0xB2

#define OP_UPLOAD_COORDINATOR_CHAN              0xC1
#define OP_ACK_UPLOAD_COORDINATOR_CHAN          0xC2
#define OP_RETRIVE_SENSOR_STATUS                0xC3
#define OP_ACK_RETRIVE_SENSOR_STATUS            0xC4

#define OP_RESET_MAGNET_SENSOR                          0x22

/*定义frame中index含义的宏*/
#define INDEX_MASK										3

/*index for nwk parameters*/
#define INDEX_MAC_BASE_ADDR					4
#define INDEX_NODE_HIGH_ADDR				7
#define INDEX_NODE_LOW_ADDR				8
#define INDEX_CHANNEL_NUM						9
#define INDEX_TX_POWER								10
#define INDEX_FATHER_HIGH_ADDR			11
#define INDEX_FATHER_LOW_ADDR				12

/* parking state*/
enum ParkingState{
        VACANT = 0,
	OCCUPIED,
        RESERVED
};

/* heartbeat state */
enum HeartbeatState{
        DEAD = 0,
        LIVE
};

/*Error Code*/
enum ErrorCode{
	/*停车场协议中的错误*/
	ERROR_NO_HEARTBEAT = 0,

	/*配置器相关的错误*/
	ERROR_NODE_ALREADY_EXIST = 100,
	ERROR_NODE_NOT_EXIST,
        ERROR_NODE_ACK_TIMEOUT,
	ERROR_WRONG_MAC_ADDR,
	ERROR_UART_COMM_FAILED,
	ERROR_RF_COMM_FAILED
};

//2014.07.24:liusf add for RF tx and rx test
#define ST_TX_VERSION   2
#define ST_RX_VERSION   1

/**
 * 前提：不处于漏电模式
 * 丢包统计模式：
 * 0 关闭
 * >0  Rx在接收 设置值*10次包后停止再接收包，并且通过循环亮灯的方式显示丢包数。
 *   每隔2S，长亮绿灯：表示没有丢包
 *   每隔2S，长亮红灯，绿灯闪丢包数 1S表示10位数，100ms表示个位数。
 */
#define STAT_TX_TIMES 100

#define TX_NUM 1

////////////////////////////////////////////////////
/* For MagTest, DON'T MODIFY THEM */
#define ST_TX_NODE_HIGH_ADDR	0xA1
#define ST_RX_NODE_HIGH_ADDR	0xB1
#define ST_NODE_LOW_ADDR	0xAA

//2014.07.24:liusf add end

#endif
