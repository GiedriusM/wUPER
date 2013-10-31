/**
 * @file	power.c
 * @author  Giedrius Medzevicius <giedrius@8devices.com>
 *
 * @section LICENSE
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2013 UAB 8devices
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 */

#include "power.h"

void WDT_IRQHandler(void) {
	//LPC_WWDT->FEED = 0xAA; LPC_WWDT->FEED = 0x55; // Reset watchdog;
	LPC_WWDT->MOD &= ~(1<<2); // Clear interrupt flags
	LPC_WWDT->MOD &= ~(1<<3);
}

void Power_startWatchdog(uint32_t timeout) {
	if (timeout > 6710)
		timeout = 6710;

	// Configure watchdog
	LPC_SYSCON->WDTOSCCTRL = (1 << 5) | ((30-1) << 0); // 600kHz/2/30 = 10kHz
	LPC_SYSCON->PDRUNCFG &= ~(1<<6);
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<15);

	volatile uint32_t d = 10000; // Let the clocks to warm up
	while (d--);

	LPC_WWDT->CLKSEL = 1; // Select watchdog clock source
	LPC_WWDT->TC = timeout*2500; // Convert seconds to cycles 10kHz/4=2500
	LPC_WWDT->MOD = 1; // Enable watchdog
	LPC_WWDT->WARNINT = 0; // Interrupt at watchdog reset
	LPC_WWDT->FEED = 0xAA; LPC_WWDT->FEED = 0x55; // Reset watchdog

	NVIC_EnableIRQ(WDT_IRQn); // Enable watchdog interrupt
}

void Power_stopWatchdog(void) {
	NVIC_DisableIRQ(WDT_IRQn); // Disable watchdog IRQ
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<15); // Stop clock to WDT block
	LPC_SYSCON->PDRUNCFG |= (1<<6); // Power down watchdog oscillator
	LPC_SYSCON->WDTOSCCTRL = 0;
}

void Power_enterPowerDown(void) {
	// Power down mode
	LPC_PMU->PCON = (LPC_PMU->PCON & 0x7) | (0x2 << 0);

	// WWDT enabled and BOD disabled in power down
	LPC_SYSCON->PDSLEEPCFG = (LPC_SYSCON->PDSLEEPCFG & ~(1<<6)) | (1<<3);

	// Switch system clock to IRC
	LPC_SYSCON->PDRUNCFG &= ~(1<<1); // Make sure IRC is on
	LPC_SYSCON->MAINCLKSEL = 0x0; // Select IRC
	LPC_SYSCON->MAINCLKUEN = 0; LPC_SYSCON->MAINCLKUEN = 1; // Toggle to update clock source

	// Power config after wake up
	LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;

	// Configure wake up interrupts
	LPC_SYSCON->STARTERP0 = 0xFF;	// All GPIO interrupts can cause wake-up
	LPC_SYSCON->STARTERP1 = (1<<12); // Enable watchdog wake-up

	// Set processors SLEEPDEEP bit
	SCB->SCR |= (1<<2);

	// Go to sleep
	__WFI();
}

void Power_exitPowerDown(void) {
	// Reset the old clock
	LPC_SYSCON->MAINCLKSEL = 0x3; // Select PLL out
	LPC_SYSCON->MAINCLKUEN = 0; LPC_SYSCON->MAINCLKUEN = 1; // Toggle to update clock source
}
