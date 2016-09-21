/*
 * TimerA 在 sungari中提供长时间延时。
 *
 * @author Hua Cao
 *
 */

#ifndef __TIMER_A_H__
#define __TIMER_A_H__

#include "BspDef.h"
#include "SGR_CONCEPT.h"

extern uint16_t g_nGlobalTime;

typedef void (*EXECUTE_AT_TIME_UP_TIMERA) (void);
void startTimer_TimerA(uint32_t milliSeconds, EXECUTE_AT_TIME_UP_TIMERA executeFuncPointer);

void stopTimer_TimerA();
#endif
