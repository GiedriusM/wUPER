/*
 * time.c
 *
 *  Created on: 2013.09.04
 *      Author: Giedrius
 */

#include "time.h"

volatile uint32_t time_systime;

typedef struct {
	uint32_t timeout;
	TimerCallback callback;
	void* callbackParam;
} Timer_t;

volatile Timer_t time_timers[TIMER_COUNT];

volatile uint32_t timer_delay;
volatile uint32_t timer_countdown;

void SysTick_Handler() {
	time_systime++;
	timer_delay--;

	if (timer_countdown != TIMER_STOP) {
		timer_countdown--;
	}

	uint8_t i;
	for (i=0; i<TIMER_COUNT; i++) {
		if (time_timers[i].timeout != TIMER_STOP) {
			if (--time_timers[i].timeout == 0) {
				time_timers[i].callback(time_timers[i].callbackParam);
			}
		}
	}
}

void Time_init() {
	time_systime = 0;

	uint8_t i;
	for (i=0; i<TIMER_COUNT; i++) {
		time_timers[i].timeout = TIMER_STOP;
	}

	SysTick_Config(SystemCoreClock/1000);	// Configure Systick to run at 1kHz (1ms)
	NVIC_SetPriority(SysTick_IRQn, 0);
}

Time_t Time_getSystemTime() {
	return time_systime;
}

void Time_addTimer(uint32_t timeout, TimerCallback callback, void *param) {
	uint8_t i;
	for (i=0; i<TIMER_COUNT; i++) {
		if (time_timers[i].timeout == TIMER_STOP) {
			time_timers[i].timeout = timeout;
			time_timers[i].callback = callback;
			time_timers[i].callbackParam = param;
			return;
		}
	}
}

void Time_delay(uint32_t time) {
	timer_delay = time;
	while (timer_delay);
}

void Time_setCountdown(uint32_t time) {
	timer_countdown = time;
}

uint8_t Time_isCountdownRunning(void) {
	if (timer_countdown == TIMER_STOP) return 0;

	return 1;
}
