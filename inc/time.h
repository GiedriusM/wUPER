/*
 * time.h
 *
 *  Created on: 2013.09.04
 *      Author: Giedrius
 */

#ifndef TIME_H_
#define TIME_H_

#include "main.h"

#define TIMER_STOP	0xFFFFFFFF
#define TIMER_COUNT	16

typedef uint32_t	Time_t;
typedef void (*TimerCallback)(void*);

void Time_init(void);

Time_t Time_getSystemTime(void);

//void Time_setTimer1(uint32_t timeout, TimerCallback callback);
void Time_addTimer(uint32_t timeout, TimerCallback callback, void *param);

void Time_delay(uint32_t time); // Delay a number of milliseconds

void Time_setCountdown(uint32_t time);
uint8_t Time_isCountdownRunning(void);

#endif /* TIME_H_ */
