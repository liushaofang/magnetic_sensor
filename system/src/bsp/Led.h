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
 * 指LED1. 按规范是红灯
 * 不可修改，否则将引起程序逻辑错误
 */
#define LED1			1

/**
 * 指LED2. 按规范是绿灯
 * 不可修改，否则将引起程序逻辑错误
 */
#define LED2			2

/**
 * 指LED3.按规范是蓝灯
 * 不可修改，否则将引起程序逻辑错误
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
 * 发现不应该存在的node type
 */
#define ERR_ILLEGAL_NODE_TYPE 2

/**
 * 在非END结点中执行get sensor value命令
 */
#define ERR_CMDGETSENSORVALUE_AT_NOTEND 3

/**
 * 发现非法actionid
 */
#define ERR_ILLEGAL_ACTIONID 4

/**
 * 获得节点时发现非法action.direction
 */
#define ERR_ILLEGAL_DIRECTION_IN_ACTION 5

/**
 * 重复节点定义
 */
#define ERR_REPEAT_NODE_DEFINE 6

/**
 * ACK包中MSGID为0
 */
#define ERR_MSGID_IN_ACK 7

/**
 * 上行MSG包中MSGID为0
 */
#define ERR_MSGID_IN_UP_MSG 8

/**
 * 下行MSG包中MSGID为0
 */
#define ERR_MSGID_IN_DOWN_MSG 9

/**
 * 分析MSG时发现节点类型错误
 */
#define ERR_NODE_TYPE_IN_ANALYZEMSG 10

/**
 * 非法的孩子地址
 */
#define ERR_ILLEGAL_CHILDREN_ADDR 11

/**
 * 非法的目标地址
 */
#define ERR_ILLEGAL_DESTADDR_ADDR 12

/**
 * UART 收包发生错误，Receive error flag被置位
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
 * 点亮所有灯，不管目前状态
 */
void turnOnLeds_LED();

 /**
  * 关掉所有灯，不管目前状态
  */
 void turnOffLeds_LED();

/**
 * 点亮指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOnLed_LED(const uint8_t led);

/**
 * 关掉指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOffLed_LED(const uint8_t led);

/**
 * 转换指定的灯
 * @param led 灯号，宏LED1 or LED2
 */
void toggleLed_LED(const uint8_t led);

/**
 * Initialize LED hardware and driver.
 * configure LEDs
 */
void initLeds_LED(void);

/**
 * 先两灯一起亮5秒钟表示上电正常，
 * 然后LED2一直亮，LED1按版本号闪烁version次。
 * 然后两灯齐灭，2秒后退出
 *
 * @param version 版本号. Best <10
 */
void showVersion_LED(uint8_t version);

/**
 * 让指定的灯闪烁
 * 不管这个灯目前的状态，都是马上灭亮灭亮
 * 退出方法时，一定会让该灯灭。
 * @param led 灯号，宏LED1 or LED2
 * @param times 闪烁次数
 * @param idle 每次亮/灭的持续毫秒数(milliseconds)
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
