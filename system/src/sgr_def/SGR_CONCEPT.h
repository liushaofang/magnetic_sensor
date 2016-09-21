/*
 * basic concepts of TMS.
 * It can be used for all source code files of TMS.
 *
 * DON'T MODIFY THIS FILE.
 *
 * @author CaoHua
 *
 */

#ifndef __SGR_CONCEPT_H__
#define __SGR_CONCEPT_H__

#define MSP2232

#include <intrinsics.h>
#ifdef MSP2232
#include <msp430x22x2.h>
#else
#include <msp430x22x4.h>
#endif

/* 形如uint8_t是国际通行的写法，sungari沿用这个写法 */
typedef signed char     int8_t;
typedef signed short    int16_t;
typedef signed long     int32_t;
typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned long   uint32_t;

/* Length of node address */
#define ADDR_LENGTH 2

#define TRUE 1
#define FALSE 0
#define SUCCESS 1
#define FAILURE 0

#define RELAY_LOW_ADDR 0x02
#define ROOT_ADDR {0xAA, 0x00}
#define UART_ADDR {0xAE, 0x00}
#define BROADCAST_ADDR  {0x00, 0x03}
#define BROADCAST_HIGH_ADDR 0x00

#define INVALID_ADDR {0xFF, 0xFF}

/**
 * The Max packet length allowed.
 */
#define PACKET_LEN 16
#define RX_METRICS_LEN 2

typedef struct {
	uint8_t  addr[2];
} addr_t;

typedef struct {
	uint8_t frameLength;
	uint8_t highAddr;
	uint8_t lowAddr;
	uint8_t frame[PACKET_LEN-3];
	uint8_t rxMetrics[RX_METRICS_LEN];
} mrfiPacket_t;

#pragma pack(push, 1)
typedef struct {
	addr_t ip_addr;
	uint8_t channel;
	uint8_t tx_power;
	addr_t father_addr;
} nwkParam_t;
#pragma pack(pop)

/**
 * Macro function to set value in bit.
 */
#define BV(n)      (1 << (n))

#define st(x)      do { x } while (__LINE__ == -1)

#ifndef ENABLE_WDT
#define DISABLE_WDT
#define FEED_WDT
#else
#define DISABLE_WDT st(WDTCTL = WDTPW | WDTHOLD;)
#define FEED_WDT st(WDTCTL = WDT_ARST_1000;)
#endif

#define ENABLE_INTERRUPTS_SGR()         __enable_interrupt()
#define DISABLE_INTERRUPTS_SGR()        __disable_interrupt()
#define NOP_SGR()							__no_operation()
#define ENTER_CRITICAL_SECTION_SGR(x)   st( x = __get_interrupt_state(); __disable_interrupt(); )
#define EXIT_CRITICAL_SECTION_SGR(x)    __set_interrupt_state(x)
#define CRITICAL_STATEMENT_SGR(x)       st( bspIState_t s;                    \
                                        ENTER_CRITICAL_SECTION_SGR(s);    \
                                            x; \
										EXIT_CRITICAL_SECTION_SGR(s); )

/**
 * STATIC ASSERT，用于检查程序严谨性，可引发编译时错误
 * @param expression 可为任何表达式，等于0时引发assert错误
 */
#define STATIC_ASSERT_SGR(expression)   void bspDummyPrototype( char dummy[1/((expression)!=0)] )

#endif
