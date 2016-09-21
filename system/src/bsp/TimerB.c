/*
 * Driver of Timer B.
 * We use Timer B as timer in main processing flow.
 *
 * @author Hua Cao, Taurus Ning, Chuanzheng Wang
 *
 */

#include "TimerB.h"

/*
 * ACLK 32KHz
 * TIMER_TBCCR0 = 164
 * count for 0.005s
 */
#define	TIMERB_TBCCR0 164

#define TIMERB_INTERVAL 5

#define CLEAR_AND_STOP_TIMER_B() st(TBCTL |= TBCLR; TBCTL = 0;)

/* 记录定时器内部时间，以5毫秒为单位 */
static uint16_t timerCount;

/* function pointer to be executed when time file. */
static EXECUTE_AT_TIME_UP_TIMERB executeFunc = 0;

void startTimerAsDelaying_TimerB(uint32_t milliseconds) {
	startTimer_TimerB(milliseconds, 0);
}

void startTimer_TimerB(uint32_t milliseconds, EXECUTE_AT_TIME_UP_TIMERB executeFuncPointer) {
	timerCount = milliseconds / TIMERB_INTERVAL;
	CLEAR_AND_STOP_TIMER_B();

	TBCTL = TBSSEL_1 | TBIE;
	TBCTL |= MC_1;
	TBCCR0 = TIMERB_TBCCR0;

	executeFunc = executeFuncPointer;
}

void stopTimer_TimerB() {
	CLEAR_AND_STOP_TIMER_B();
}

/**
 * Timer_B Interrupt service routine
 *
 * 00h No interrupt pending -
 * 02h Capture/compare 1 TBCCR1 CCIFG Highest
 * 04h Capture/compare 2 TBCCR2 CCIFG
 * 06h Capture/compare 3 TBCCR3 CCIFG
 * 08h Capture/compare 4 TBCCR4 CCIFG
 * 0Ah Capture/compare 5 TBCCR5 CCIFG
 * 0Ch Capture/compare 6 TBCCR6 CCIFG
 * 0Eh Timer overflow TBIFG Lowest
 */
#pragma vector=TIMERB1_VECTOR
__interrupt void doInISR_TimerB(void) {
	switch (TBIV) { // Efficient switch-implementation
		case 0x0E: {
			timerCount--;
			if(timerCount == 0) {
				CLEAR_AND_STOP_TIMER_B();
				if(executeFunc != 0)
					executeFunc();
				LPM3_EXIT;
			}
			break;
		}
		default: {
			/* just ignore other Flags */
			break;
		} //default
	} //switch
}

