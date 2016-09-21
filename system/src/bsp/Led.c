/*
 * driver for LED.
 *
 * Implementation:
 * 1. 严重依赖 delayInMs_BSP
 *
 * @author CaoHua
 *
 */

#include "Led.h"



////////////////////////////////////////////////////////////////
/* PIN configuration */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #1
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  D1
 *   Color     :  Red
 *   Polarity  :  Active High
 *   GPIO      :  P4.0 or P3.6
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define __bsp_LED1_BIT__            			6
#define __bsp_LED1_PORT__           			P3OUT
#define __bsp_LED1_DDR__             		P3DIR
#define __bsp_LED1_IS_ACTIVE_LOW__ 		 0

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #2
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  D2
 *   Color     :  Green
 *   Polarity  :  Active High
 *   GPIO      :  P4.1 or P3.7
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define __bsp_LED2_BIT__            			7
#define __bsp_LED2_PORT__          	 		P3OUT
#define __bsp_LED2_DDR__            			P3DIR
#define __bsp_LED2_IS_ACTIVE_LOW__  		0

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                 LED #3
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   Schematic :  D3
 *   Color     :  Blue
 *   Polarity  :  Active High
 *   GPIO      :  P4.2 or P4.7
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define __bsp_LED3_BIT__            			7
#define __bsp_LED3_PORT__          	 		P4OUT
#define __bsp_LED3_DDR__            			P4DIR
#define __bsp_LED3_IS_ACTIVE_LOW__  		0

/* end */
////////////////////////////////////////////////////////////////



#define __bsp_LED_TURN_ON__(bit,port,ddr,low)  \
  st( if (low) { port &= ~BV(bit); } else { port |= BV(bit); } )

#define __bsp_LED_TURN_OFF__(bit,port,ddr,low)  \
  st( if (low) { port |= BV(bit); } else { port &= ~BV(bit); } )

#define __bsp_LED_IS_ON__(bit,port,ddr,low)  \
  st( (low) ? (!((port) & BV(bit))) : ((port) & BV(bit)) )

#define __bsp_LED_TOGGLE__(bit,port,ddr,low)     	st( port ^= BV(bit); )
#define __bsp_LED_CONFIG__(bit,port,ddr,low)     	st( ddr |= BV(bit); )

/**
 * 点亮所有灯，不管目前状态
 */
void turnOnLeds_LED() {
	__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_ON__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
}

/**
 * 关掉所有灯，不管目前状态
 */
void turnOffLeds_LED() {
	__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
}

/**
 * 点亮指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOnLed_LED(const uint8_t led) {
	if (led == LED1) {
		__bsp_LED_TURN_ON__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	} else if(led == LED2) {
		__bsp_LED_TURN_ON__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	} else if(led == LED3) {
		__bsp_LED_TURN_ON__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
	}
}

/**
 * 关掉指定的灯，不管这个 灯目前状态
 * @param led 灯号，宏LED1 or LED2
 */
void turnOffLed_LED(const uint8_t led) {
	if (led == LED1) {
		__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	} else if (led == LED2){
		__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	} else if (led == LED3) {
		__bsp_LED_TURN_OFF__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
	}
}

/**
 * 转换指定的灯
 * @param led 灯号，宏LED1 or LED2
 */
void toggleLed_LED(const uint8_t led) {
	if (led == LED1) {
		__bsp_LED_TOGGLE__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	} else if(led == LED2) {
		__bsp_LED_TOGGLE__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	} else if(led == LED3) {
		__bsp_LED_TOGGLE__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
	}
}

/**
 * Initialize LED hardware and driver.
 * configure LEDs
 */
void initLeds_LED(void) {
	__bsp_LED_CONFIG__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_CONFIG__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	__bsp_LED_CONFIG__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);

	//turn the leds off
	__bsp_LED_TURN_OFF__(__bsp_LED1_BIT__,__bsp_LED1_PORT__,__bsp_LED1_DDR__,__bsp_LED1_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED2_BIT__,__bsp_LED2_PORT__,__bsp_LED2_DDR__,__bsp_LED2_IS_ACTIVE_LOW__);
	__bsp_LED_TURN_OFF__(__bsp_LED3_BIT__,__bsp_LED3_PORT__,__bsp_LED3_DDR__,__bsp_LED3_IS_ACTIVE_LOW__);
}

/**
 * 让指定的灯闪烁
 * 不管这个灯目前的状态，都是马上灭亮灭亮
 * 退出方法时，一定会让该灯灭。
 * @param led 灯号，宏LED1 or LED2
 * @param times 闪烁次数
 * @param idle 每次亮/灭的持续毫秒数(milliseconds)
 */
void twinkleLed_LED(const uint8_t led, uint8_t times, uint16_t idle) {
	while (times--) {
		turnOffLed_LED(led);
		delayInMs_BSP(idle);
		turnOnLed_LED(led);
		delayInMs_BSP(idle);
	}
	turnOffLeds_LED();
}

/**
 * 先两灯一起亮5秒钟表示上电正常，
 * 然后LED2一直亮，LED1按版本号闪烁version次。
 * 然后两灯齐灭，2秒后退出
 *
 * @param version 版本号. Best <10
 */
void showVersion_LED(uint8_t version) {
	turnOffLeds_LED();
	toggleLed_LED(LED1);
	toggleLed_LED(LED2);
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
	delayInMs_BSP(1000);
        //FEED_WDT;    //2015.02.01:liusf add watchdog function

	while (version--) {
		toggleLed_LED(LED1);
                DISABLE_WDT;    //2015.02.01:liusf add watchdog function
		delayInMs_BSP(200);
                //FEED_WDT;    //2015.02.01:liusf add watchdog function
		toggleLed_LED(LED1);
                DISABLE_WDT;    //2015.02.01:liusf add watchdog function
		delayInMs_BSP(200);
                //FEED_WDT;    //2015.02.01:liusf add watchdog function
	}

	turnOffLeds_LED();
        DISABLE_WDT;    //2015.02.01:liusf add watchdog function
	delayInMs_BSP(1000);
        //FEED_WDT;    //2015.02.01:liusf add watchdog function
}

/**
 * Don't call this function DIRECTLY!  use ASSERT_SGR Macro Function as instead.
 *
 * If the assert term is false(0),
 * put the board to halt status and repeating show error number with LED .
 *
 * @param term an expression to check.  0 means fail.
 * @param errorNumber use ERR_ series macro as error number.
 */
__monitor void assert_LED(const uint8_t term, uint8_t errorNumber) {
        //2014.03.04:         liusf add for antena performance test
        int8_t nErrorCount = 10;
        //2014.03.04:         liusf add end
	if(term)
		return;
        
        DISABLE_WDT;       //2015.02.01:liusf add watchdog function
	turnOnLeds_LED();
	delayInSecond_BSP(1);
        FEED_WDT;       //2015.02.01:liusf add watchdog function
        
        //2014.03.04:   liusf modified start
	DISABLE_WDT;       //2015.02.01:liusf add watchdog function
        while(nErrorCount -- >= 0) {
          //2014.03.04: liusf modified end
		twinkleLed_LED(LED2, errorNumber, 300);
		delayInSecond_BSP(1);
	}
        FEED_WDT;       //2015.02.01:liusf add watchdog function
}

