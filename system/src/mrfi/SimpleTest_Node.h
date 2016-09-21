/*
 * SimpleTest is a tool for test the mrfi layer.
 * This is the public header file for SimpleTest TX and RX program.
 *
 * @author CaoHua
 *
 */
#ifndef __SIMPLE_TEST_NODE_H__
#define __SIMPLE_TEST_NODE_H__

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
#define TXPOWER 0xC0

/**
 * 1 - 20, �����16���� 0x01 - 0x14
 * ��SimpleTest��������Ч��SimpleTest�̶�Ϊ11.����mrfi�������⴦����
 */
#define CHANNEL 0x18

#define ST_TX_VERSION   2
#define ST_RX_VERSION   1

/**
 * ǰ�᣺������©��ģʽ
 * ����ͳ��ģʽ��
 * 0 �ر�
 * >0  Rx�ڽ��� ����ֵ*10�ΰ���ֹͣ�ٽ��հ�������ͨ��ѭ�����Ƶķ�ʽ��ʾ��������
 *   ÿ��2S�������̵ƣ���ʾû�ж���
 *   ÿ��2S��������ƣ��̵��������� 1S��ʾ10λ����100ms��ʾ��λ����
 */
#define STAT_TX_TIMES 100

#define TX_NUM 1

////////////////////////////////////////////////////
/* For SimpleTest, DON'T MODIFY THEM */
#define ST_TX_NODE_HIGH_ADDR	0x01
#define ST_RX_NODE_HIGH_ADDR	0xA1
#define ST_NODE_LOW_ADDR	0xBB


#endif