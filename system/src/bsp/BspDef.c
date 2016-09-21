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
 * ����DCO��Ƶ��Ϊ8MHz
 * LFXT1�����ڵ�Ƶģʽ
 * MCLK��SMCLKʱ��ԴΪDCO
 * �ⲿ����Ƶ��Ϊ32768���ڲ����ص���ѡ��10pF
 * �رտ��Ź�
 */
void initBSP_BSP(void) {
	/* Basic Clock Module Configuration */

	/**
	 * DC0CTL BCSCTL1 �Ĵ�����Ҫ��ֵ��Ԥ���� оƬ �� Flash SegmentA�С�
	 * IAR�ṩ�ĺ꺯�� CALDCO_8MHZ CALBC1_8MHZ ����������ֵ��
	 */
	DCOCTL = CALDCO_8MHZ;
	BCSCTL1 = CALBC1_8MHZ;

	/*LFXT1 works in low frequency mode*/
	BCSCTL1 &= ~XTS;

	/**
	 * BCSCTL2ʹ��Ĭ��ֵ����
	 * MCLK��SMCLK��ʱ��Դѡ��DCO��
	 * DCO resistorѡ��internal resistor
	 */

	/**
	 * 7:6  not used
	 * 5:4  00 32768 Hz Crystal on LFXT1
	 * 3:2  10 internal capacitance selects 10pF
	 */
	BCSCTL3 = BV(3) | BV(2);

	/* ��ʼ��ʱ�ȹع���ι��ʱ���Զ����� */
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
