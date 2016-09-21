/*
 * Test stub is a tool for test the mrfi layer.
 * This is the public header file for SimpleTest TX and RX program.
 *
 * @author liusf
 *
 */
#ifndef __TEST_STUB_NODE_H__
#define __TEST_STUB_NODE_H__

#include "Led.h"
#include "Uart.h"
#include "mrfi.h"
#include "TimerA.h"
#include "TimerB.h"

#define TEST_STUB_VERSION 3

/**
 * -10db:34
 * 0db:60
 * 5db:84
 * 10db:C0
 */
#define TXPOWER 0xC0

/**
 * 1 - 20, 换算成16进制 0x01 - 0x14
 * 对SimpleTest此设置无效，SimpleTest固定为11.（在mrfi层有特殊处理）
 */
#define CHANNEL 0x10

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
/* For SimpleTest, DON'T MODIFY THEM */
#define ST_TX_NODE_HIGH_ADDR	0xA1
#define ST_RX_NODE_HIGH_ADDR	0xB1
#define ST_NODE_LOW_ADDR	0xAB

#define TEST_MRFI               0x01

#endif
