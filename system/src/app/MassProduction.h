/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the public header file for SimpleTest TX and RX program.
 *
 * @author CaoHua
 *
 */
#ifndef __MASS_PRODUCTION_H__
#define __MASS_PRODUCTION_H__

#include "Led.h"
#include "Uart.h"
#include "mrfi.h"
#include "TimerB.h"

/**
 * -10db:34
 * 0db:60
 * 5db:84
 * 10db:C0
 */
//#define TXPOWER 0x60
#define TXPOWER 0xC0

/**
 * 1 - 20, 换算成16进制 0x01 - 0x14
 * 对SimpleTest此设置无效，SimpleTest固定为11.（在mrfi层有特殊处理）
 */
#define CHANNEL 0x0E

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

#define RAW_DATA_LEN					6

////////////////////////////////////////////////////
/* For SimpleTest, DON'T MODIFY THEM */
#define ST_MASTER_NODE_HIGH_ADDR	0xA1
#define ST_SLAVE_NODE_HIGH_ADDR	        0xB1
#define ST_NODE_LOW_ADDR	        0xAB

#define MASS_PRODUCTION_TEST_MRFI               0xE1
#define MASS_PRODUCTION_TEST_MAGNET             0xE2

#define MASS_PRODUCTION_TEST_MRFI_ACK           0xEE
#define MASS_PRODUCTION_TEST_MAGNET_ACK         0xEF
   
#define MASS_PRODUCTION_NEED_MAGNET_TEST        0x1
#define MASS_PRODUCTION_NOT_NEED_MAGNET_TEST    0x0   
   
#define MASS_PRODUCTION_TEST_PASS               0x1
#define MASS_PRODUCTION_TEST_FAIL               0x0

#define MASS_PRODUCTION_CHECK_MAGNET_SUCCESS    0x1   
#define MASS_PRODUCTION_CHECK_MAGNET_FAIL       0x0
 
#define MRFI_MODE_TRANSMIT              1
#define MRFI_MODE_RECEIVE               2   

#endif