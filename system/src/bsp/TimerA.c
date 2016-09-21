/*
 * TimerA �� sungari���ṩ��ʱ����ʱ��
 * TODO ���˯��ʱ��ܳ���Ƶ�������жϻ�Ӱ��׼ȷ�ԣ����Կ��Ƿ�Ƶ������TACCR0��ֵ��ÿ16���һ���жϣ���߾���
 *
 * The driver of Sungari Sleep (using TimerA+ACLK)
 *
 * In Sungari, there's 2 usage scenario of TimerA:
 * For BSP_Delay, choose SMCLK as source.
 *   The driver code is in BspDef.c, not here.
 *
 * For Sleep (Sleep by Cmd, and Self-Controled Sleep & Wakeup),
 * choose ACLK(with 32kHz crystal) as source
 * In this mode, the CPU may be LPM3 during the sleep, wait for TimeA to exit.
 *
 * Here is the code.
 * StartTimer function start the TimerA by givin time.
 * when the time is run up, EXIT LPM3 will be excuted and TimerA will stop.
 *
 * It's the caller's duty to enter LPM3, not here.
 *
 * @author Hua Cao
 *
 */

#include "TimerA.h"

#define CLEAR_AND_STOP_TIMER_A() st(TACTL |= TACLR; TACTL = 0;)

/*
 * ACLK 32KHz
 * TIMER_TBCCR0 = 164
 * count for 0.005s
 */
#define	TIMERA_TACCR0 328

#define TIMERA_INTERVAL 10

/* ��¼��ʱ���ڲ�ʱ�䣬��10����Ϊ��λ */
static uint16_t timerCount;

/* function pointer to be executed when time file. */
static EXECUTE_AT_TIME_UP_TIMERA executeFunc = 0;

uint16_t g_nGlobalTime = 0;


/**
 * Start TimerA with specified minutes and seconds.
 * TimerA will end and call executeAtTimeUp_TimerA function when time is up.
 *
 * @param minute must >= 0
 * @param second must >= 0
 */
void startTimer_TimerA(uint32_t milliSeconds, EXECUTE_AT_TIME_UP_TIMERA executeFuncPointer) {
	//TODO Ϊʲôminute + 1 ��ISR�еļ����йأ����忴ISR�еĴ���. Ӧд���ļ�ע��
	timerCount = milliSeconds / TIMERA_INTERVAL;
	CLEAR_AND_STOP_TIMER_A();

	TACCR0 = TIMERA_TACCR0;
	TACTL = TASSEL_1 ; //ACLK
	TACTL |= TAIE;
	TACTL |= MC_1;

	executeFunc = executeFuncPointer;
}

void stopTimer_TimerA() {
	CLEAR_AND_STOP_TIMER_A();
}

/**
 * Timer_A interrupt service routine
 * 		00h No interrupt pending -
 *		02h Capture/compare1 TACCR1 CCIFG Highest
 *		04h Capture/compare2 TACCR2 CCIFG
 *		06h Reserved -
 *		08h Reserved -
 *		0Ah Timer overflow TAIFG
 *		0Ch Reserved -
 *		0Eh Reserved - Lowest
 */
#pragma vector=TIMERA1_VECTOR
__interrupt void doInISR_TimerA(void) {
	switch (TAIV) {
		case 0x0A: {
			timerCount--;
			if(timerCount == 0) {
				CLEAR_AND_STOP_TIMER_A();
				if(executeFunc != 0)
					executeFunc();
				LPM3_EXIT;
			}
                        g_nGlobalTime ++;
			break;
		}//case2

		default : {
			/* do nothing for other flag */
			break;
		}
	}
}
