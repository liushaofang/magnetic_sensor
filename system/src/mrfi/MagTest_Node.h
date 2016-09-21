/*
 * MagTest is a tool for test mag3110 driver, MagTest_Tx reads data from sensor and then transmits all raw data to MagTest_RX.
 * This is the public header file for MagTest TX and RX program.
 *
 * @author CaoHua
 *
 */

#ifndef MAGTEST_NODE_H_
#define MAGTEST_NODE_H_

#include "Led.h"
#include "Uart.h"
#include "mrfi.h"
#include "TimerB.h"
#include "hmc5883l.h"

/**
 * -10db:34
 * 0db:60
 * 5db:84
 * 10db:C0
 */
#define TXPOWER 0x60

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

////////////////////////////////////////////////////
/* For MagTest, DON'T MODIFY THEM */
#define ST_TX_NODE_HIGH_ADDR	0xA1
#define ST_RX_NODE_HIGH_ADDR	0xB1
#define ST_NODE_LOW_ADDR	0xAA


#endif /* MAGTEST_NODE_H_ */
