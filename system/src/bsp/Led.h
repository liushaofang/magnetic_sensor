/*
 * driver for LED.  including ASSERT mechanism and Error number definition.
 *
 * @author CaoHua
 *
 */

#ifndef __LED_H__
#define __LED_H__

#include "BspDef.h"

/**
 * ָLED1. ���淶�Ǻ��
 * �����޸ģ�������������߼�����
 */
#define LED1			1

/**
 * ָLED2. ���淶���̵�
 * �����޸ģ�������������߼�����
 */
#define LED2			2

/**
 * ָLED3.���淶������
 * �����޸ģ�������������߼�����
 */
#define LED3			3

#ifdef SGR_ASSERT_ON
#define ASSERT_SGR(term, num) assert_LED(term, num)
#else
#define ASSERT_SGR(term, num)
#endif

///////////////////////////////////////////////////////////
/* Error code definition */

/**
 * Error without special information
 */
#define ERR_COMMON_ERROR 1

/**
 * ���ֲ�Ӧ�ô��ڵ�node type
 */
#define ERR_ILLEGAL_NODE_TYPE 2

/**
 * �ڷ�END�����ִ��get sensor value����
 */
#define ERR_CMDGETSENSORVALUE_AT_NOTEND 3

/**
 * ���ַǷ�actionid
 */
#define ERR_ILLEGAL_ACTIONID 4

/**
 * ��ýڵ�ʱ���ַǷ�action.direction
 */
#define ERR_ILLEGAL_DIRECTION_IN_ACTION 5

/**
 * �ظ��ڵ㶨��
 */
#define ERR_REPEAT_NODE_DEFINE 6

/**
 * ACK����MSGIDΪ0
 */
#define ERR_MSGID_IN_ACK 7

/**
 * ����MSG����MSGIDΪ0
 */
#define ERR_MSGID_IN_UP_MSG 8

/**
 * ����MSG����MSGIDΪ0
 */
#define ERR_MSGID_IN_DOWN_MSG 9

/**
 * ����MSGʱ���ֽڵ����ʹ���
 */
#define ERR_NODE_TYPE_IN_ANALYZEMSG 10

/**
 * �Ƿ��ĺ��ӵ�ַ
 */
#define ERR_ILLEGAL_CHILDREN_ADDR 11

/**
 * �Ƿ���Ŀ���ַ
 */
#define ERR_ILLEGAL_DESTADDR_ADDR 12

/**
 * UART �հ���������Receive error flag����λ
 */
#define ERR_UART_RX			13

#define ERR_ILLEGAL_CMD_MRFI 14

#define ERR_ILLEGAL_LEN_MRFI 15

#define ERR_ILLEGAL_BURSTBIT_MRFI 16

#define ERR_ILLEGAL_ARGUMENT 17

#define ERR_ILLEGAL_DETECT_TYPE_SHT10 	18

/* end */
//////////////////////////////////////////////////////////////





/**
 * �������еƣ�����Ŀǰ״̬
 */
void turnOnLeds_LED();

 /**
  * �ص����еƣ�����Ŀǰ״̬
  */
 void turnOffLeds_LED();

/**
 * ����ָ���ĵƣ�������� ��Ŀǰ״̬
 * @param led �ƺţ���LED1 or LED2
 */
void turnOnLed_LED(const uint8_t led);

/**
 * �ص�ָ���ĵƣ�������� ��Ŀǰ״̬
 * @param led �ƺţ���LED1 or LED2
 */
void turnOffLed_LED(const uint8_t led);

/**
 * ת��ָ���ĵ�
 * @param led �ƺţ���LED1 or LED2
 */
void toggleLed_LED(const uint8_t led);

/**
 * Initialize LED hardware and driver.
 * configure LEDs
 */
void initLeds_LED(void);

/**
 * ������һ����5���ӱ�ʾ�ϵ�������
 * Ȼ��LED2һֱ����LED1���汾����˸version�Ρ�
 * Ȼ����������2����˳�
 *
 * @param version �汾��. Best <10
 */
void showVersion_LED(uint8_t version);

/**
 * ��ָ���ĵ���˸
 * ���������Ŀǰ��״̬������������������
 * �˳�����ʱ��һ�����øõ���
 * @param led �ƺţ���LED1 or LED2
 * @param times ��˸����
 * @param idle ÿ����/��ĳ���������(milliseconds)
 */
void twinkleLed_LED(const uint8_t led, uint8_t times, uint16_t idle);

/**
 * Don't call this function DIRECTLY!  use ASSERT_SGR Macro Function as instead.
 *
 * If the assert term is false(0),
 * put the board to halt status and repeating show error number with LED .
 *
 * @param term an expression to check.  0 means fail.
 * @param errorNumber use ERR_ series macro as error number.
 */
__monitor void assert_LED(uint8_t term, uint8_t errorNumber);

#endif
