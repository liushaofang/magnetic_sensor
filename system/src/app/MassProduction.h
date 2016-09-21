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
 * 1 - 20, �����16���� 0x01 - 0x14
 * ��SimpleTest��������Ч��SimpleTest�̶�Ϊ11.����mrfi�������⴦��
 */
#define CHANNEL 0x0E

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