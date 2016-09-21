/*
 * Driver of Timer B.
 * We use Timer B as timer in main processing flow.
 *
 * with the postfix _TimerB
 *
 * @author Hua Cao
 *
 */

#ifndef __TIMER_B_H__
#define __TIMER_B_H__

#include "SGR_CONCEPT.h"

typedef void (* EXECUTE_AT_TIME_UP_TIMERB)(void);

/**
 * Start the timer just for delaying usage.
 * No special thing will do when time is up.
 *
 * It could be used as a power-saving delaying as this:
 * 1: startTimerAsDelaying_TimerB(ms);
 * 2: LPM3;
 * 3: next step after the delay;
 *
 * It's the caller's duty to decide whether disable other Interrupts during the delaying or not.
 *
 * @param milliseconds how many milliseconds to delay
 */
void startTimerAsDelaying_TimerB(uint32_t milliSeconds);

/**
 * Start the timer, and specify a function to be executed when time is up.
 *
 * @param milliSeconds how many ms to run
 * @param executeFuncPointer the function to be executed when time is up.
 */
void startTimer_TimerB(uint32_t milliSeconds, EXECUTE_AT_TIME_UP_TIMERB executeFuncPointer);

/**
 * Stop the timer, no matter if the timer is running or not.
 * If timer is running, it will be stopped, and the specified function to execute when time is up
 * won't be executed.
 */
void stopTimer_TimerB();

#endif
