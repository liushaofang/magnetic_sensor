/*
 * Basic definition and functions in BSP Layer.
 *
 * DON'T MODIFY THIS FILE.
 *
 * @author Hua Cao
 *
 */

#include "BspDef.h"

/**
 * initiate the board.
 * 设置DCO的频率为8MHz
 * LFXT1工作在低频模式
 * MCLK和SMCLK时钟源为DCO
 * 外部晶振频率为32768，内部负载电容选择10pF
 * 关闭看门狗
 */
void initBSP_BSP(void) {
	/* Basic Clock Module Configuration */

	/**
	 * DC0CTL BCSCTL1 寄存器需要的值，预存在 芯片 的 Flash SegmentA中。
	 * IAR提供的宏函数 CALDCO_8MHZ CALBC1_8MHZ 读出这两个值。
	 */
	DCOCTL = CALDCO_8MHZ;
	BCSCTL1 = CALBC1_8MHZ;

	/*LFXT1 works in low frequency mode*/
	BCSCTL1 &= ~XTS;

	/**
	 * BCSCTL2使用默认值，即
	 * MCLK和SMCLK的时钟源选用DCO，
	 * DCO resistor选用internal resistor
	 */

	/**
	 * 7:6  not used
	 * 5:4  00 32768 Hz Crystal on LFXT1
	 * 3:2  10 internal capacitance selects 10pF
	 */
	BCSCTL3 = BV(3) | BV(2);

	/* 初始化时先关狗。喂狗时狗自动启动 */
	WDTCTL = WDTPW | WDTHOLD;
}

/**
 * delay in milliseconds.
 * @param msCount how many milliseconds to delay
 */
void delayInMs_BSP(uint16_t msCount) 
{
        while(msCount--) 
        {
                DELAY_US_BSP(1000);
	}
}

/**
 * delay in seconds.
 * @param secondCount how many seconds to delay
 */
void delayInSecond_BSP(uint16_t secondCount) {
	while(secondCount--) {
		delayInMs_BSP(1000);
	}
}

/* Compile Time Integrity Checks */
STATIC_ASSERT_SGR( sizeof( uint8_t ) == 1 );
STATIC_ASSERT_SGR( sizeof( int8_t ) == 1 );
STATIC_ASSERT_SGR( sizeof( uint16_t ) == 2 );
STATIC_ASSERT_SGR( sizeof( int16_t ) == 2 );
STATIC_ASSERT_SGR( sizeof( uint32_t ) == 4 );
STATIC_ASSERT_SGR( sizeof( long ) == 4 );
