/**
 * @file	LPC_GPIO.c
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

#include "Modules/LPC_GPIO.h"

uint8_t const LPC_PIN_IDS[] = {
		0+20,	0+2,	24+26,	24+27,	24+20,	0+21,	24+23,	24+24,	// 8
		0+7,	24+28,	24+31,	24+21,	0+8,	0+9,	0+10,	24+29,	// 16
		24+19,	24+25,	24+16,	0+19,	0+18,	0+17,	24+15,	0+23,	// 24
		0+22,	0+16,	0+15,	24+22,	24+14,	24+13,	0+14,	0+13,	// 32
		0+12,	0+11,	// 34
};

volatile uint32_t * const LPC_PIN_REGISTERS[] = {
		&LPC_IOCON->PIO0_20, 	&LPC_IOCON->PIO0_2 , 	&LPC_IOCON->PIO1_26, 	&LPC_IOCON->PIO1_27,
		&LPC_IOCON->PIO1_20, 	&LPC_IOCON->PIO0_21, 	&LPC_IOCON->PIO1_23, 	&LPC_IOCON->PIO1_24,		// 8
		&LPC_IOCON->PIO0_7 , 	&LPC_IOCON->PIO1_28, 	&LPC_IOCON->PIO1_31, 	&LPC_IOCON->PIO1_21,
		&LPC_IOCON->PIO0_8 ,	&LPC_IOCON->PIO0_9 , 	&LPC_IOCON->SWCLK_PIO0_10, &LPC_IOCON->PIO1_29,		// 16
		&LPC_IOCON->PIO1_19, 	&LPC_IOCON->PIO1_25,	&LPC_IOCON->PIO1_16, 	&LPC_IOCON->PIO0_19,
		&LPC_IOCON->PIO0_18,	&LPC_IOCON->PIO0_17,	&LPC_IOCON->PIO1_15, 	&LPC_IOCON->PIO0_23, 		// 24
		&LPC_IOCON->PIO0_22,	&LPC_IOCON->PIO0_16,	&LPC_IOCON->SWDIO_PIO0_15,&LPC_IOCON->PIO1_22,
		&LPC_IOCON->PIO1_14, 	&LPC_IOCON->PIO1_13, 	&LPC_IOCON->TRST_PIO0_14,&LPC_IOCON->TDO_PIO0_13,	// 32
		&LPC_IOCON->TMS_PIO0_12,&LPC_IOCON->TDI_PIO0_11,	// 34
};

#define LPC_PIN_FUNCTION_MASK	(BIT7 | 7)		// FUNC bits + AD bit
#define LPC_PIN_MODE_MASK		(3 << 3)

uint8_t const LPC_PIN_PRIMARY_FUNCTION[] = {	// GPIO function, All AD bits = 1
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x80,	0x80,	// 8
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x81,	0x80,	// 16
		0x80,	0x80,	0x80,	0x80,		0x80,	0x80,	0x80,	0x80,	// 24
		0x80,	0x80,	0x81,	0x80,		0x80,	0x80,	0x81,	0x81,	// 32
		0x81,	0x81,	// 34
};

uint8_t const LPC_PIN_SECONDARY_FUNCTION[] = {
		0x80 /* GPIO */,	0x80 /* GPIO */,		0x81 /* CT32B0 MAT2 */,	0x80 /* GPIO */,
		0x82 /* SPI1 SCK */,0x82 /* SPI1 MOSI */,	0x80 /* GPIO */,		0x81 /* CT32B0 MAT0 */,	// 8
		0x80 /* GPIO */,	0x80 /* GPIO */,		0x80 /* GPIO */,		0x82 /* SPI1 MISO */,
		0x81 /* SPI0 MISO*/,0x81 /* SPI0 MOSI */,	0x82 /* SPI0 SCK */,	0x80 /* GPIO */,	// 16
		0x80 /* GPIO */,	0x81 /* CT32B0 MAT1 */,	0x80 /* GPIO */,		0x81 /* UART TX */,
		0x81 /* UART RX */,	0x80 /* GPIO */,		0x82 /* PWM16_2 */,		0x01 /* ADC7 */,	// 24
		0x01 /* ADC6 */,	0x01 /* ADC5 */,		0x02 /* ADC4 */,		0x80 /* GPIO */,
		0x82 /* PWM16_1 */,	0x82 /* PWM16_0 */,		0x02 /* ADC3 */,		0x02 /* ADC2 */,	// 32
		0x02 /* ADC1 */,	0x02 /* ADC0 */,	// 34
};

static volatile uint8_t LPC_INTERRUPT_FLAG = 0;
static volatile uint8_t LPC_INTERRUPT_TYPE[LPC_INTERRUPT_COUNT];
static volatile SFPFunctionType LPC_INTERRUPT_FUNCTION_TYPE[LPC_INTERRUPT_COUNT];
static volatile uint32_t LPC_INTERRUPT_DESTINATION[LPC_INTERRUPT_COUNT];

void lpc_config_gpioInit() {
	uint8_t pin;
	for (pin=0; pin<LPC_PIN_COUNT; pin++)
		*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_PRIMARY_FUNCTION[pin];
}

SFPResult lpc_config_setPrimary(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 2) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);

	if (pin >= LPC_PIN_COUNT) return SFP_ERR_ARG_VALUE;

	*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_PRIMARY_FUNCTION[pin];

	return SFP_OK;
}

SFPResult lpc_config_setSecondary(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 2) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);

	if (pin >= LPC_PIN_COUNT) return SFP_ERR_ARG_VALUE;

	*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_SECONDARY_FUNCTION[pin];

	return SFP_OK;
}


SFPResult lpc_pinMode(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 3) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 2) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);
	uint8_t mode = SFPFunction_getArgument_int32(msg, 2);

	if (pin >= LPC_PIN_COUNT) return SFP_ERR_ARG_VALUE;
	if (mode > 4 || mode == 3) return SFP_ERR_ARG_VALUE;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	*LPC_PIN_REGISTERS[pin] &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors

	if (mode == 1) {
		LPC_GPIO->DIR[port] |= (1 << pinNum);	// Set direction bit (output)
	} else {
		*LPC_PIN_REGISTERS[pin] |= (mode << 2) & LPC_PIN_MODE_MASK;		// Setup resistors
		LPC_GPIO->DIR[port] &= ~(1 << pinNum);	// Clear direction bit (input)
	}

	return SFP_OK;
}

SFPResult lpc_digitalWrite(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 3)
		return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 2) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);
	uint8_t value = SFPFunction_getArgument_int32(msg, 2);

	if (pin >= LPC_PIN_COUNT) return SFP_ERR_ARG_VALUE;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	if (value == 0) {
		LPC_GPIO->CLR[port] = (1 << pinNum);
	} else {
		LPC_GPIO->SET[port] = (1 << pinNum);
	}

	return SFP_OK;
}

SFPResult lpc_digitalRead(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 2)
		return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);

	if (pin >= LPC_PIN_COUNT)
		return SFP_ERR_ARG_VALUE;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	uint8_t val = 0;

	if (LPC_GPIO->PIN[port] & (1 << pinNum))
		val = 1;

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	WUPER_SetDestinationAddress(SFPFunction_getArgument_int32(msg, 0));

	SFPFunction_setType(outFunc, SFPFunction_getType(msg));
	SFPFunction_setID(outFunc, WUPER_RF_FID_DIGITALREAD);
	SFPFunction_setName(outFunc, WUPER_RF_FNAME_DIGITALREAD);
	SFPFunction_addArgument_int32(outFunc, WUPER_GetDeviceAddress());
	SFPFunction_addArgument_int32(outFunc, pin);
	SFPFunction_addArgument_int32(outFunc, val);
	SFPFunction_send(outFunc, &spirit_stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult lpc_pulseIn(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 4) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 2) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 3) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t pin = SFPFunction_getArgument_int32(msg, 1);
	uint8_t levelMask = (SFPFunction_getArgument_int32(msg, 2) == 0 ? 0 : 1);
	uint32_t timeout = SFPFunction_getArgument_int32(msg, 3);

	if (pin >= LPC_PIN_COUNT) return SFP_ERR_ARG_VALUE;

	uint8_t port = 0;
	uint8_t pinNum = LPC_PIN_IDS[pin];
	if (pinNum > 23) {	// if not PIO0_0 to PIO0_23
		port = 1;
		pinNum -= 24;
	}

	levelMask <<= pinNum; // shift BIT0 to pin place

	uint32_t startTimeUs = Time_getSystemTime_us();
	uint32_t passedTimeUs = 0;

	while ((LPC_GPIO->PIN[port] & (1 << pinNum)) == levelMask) {	// Wait while signal is on
		if ((passedTimeUs=Time_getSystemTime_us()-startTimeUs) >= timeout)
			break;
	}

	while ((LPC_GPIO->PIN[port] & (1 << pinNum)) != levelMask) { // Wait while signal is off
		if ((passedTimeUs=Time_getSystemTime_us()-startTimeUs) >= timeout)
			break;
	}

	uint32_t signalStartTime = Time_getSystemTime_us();

	while ((LPC_GPIO->PIN[port] & (1 << pinNum)) == levelMask) {	// Wait while signal is on
		if ((passedTimeUs=Time_getSystemTime_us()-startTimeUs) >= timeout)
			break;
	}
	uint32_t signalDuration = Time_getSystemTime_us()-signalStartTime;

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	WUPER_SetDestinationAddress(SFPFunction_getArgument_int32(msg, 0));

	SFPFunction_setType(outFunc, SFPFunction_getType(msg));
	SFPFunction_setID(outFunc, WUPER_RF_FID_PULSEIN);
	SFPFunction_setName(outFunc, WUPER_RF_FNAME_PULSEIN);
	SFPFunction_addArgument_int32(outFunc, WUPER_GetDeviceAddress());
	SFPFunction_addArgument_int32(outFunc, (passedTimeUs < timeout ? signalDuration : 0));
	SFPFunction_send(outFunc, &spirit_stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult lpc_attachInterrupt(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 4)
		return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 2) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 3) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint8_t p_intID = SFPFunction_getArgument_int32(func, 1);	// interrupt ID
	uint8_t p_pin = SFPFunction_getArgument_int32(func, 2);	// pin ID
	uint8_t p_mode = SFPFunction_getArgument_int32(func, 3);	// interrupt mode

	if (p_pin >= LPC_PIN_COUNT || p_intID >= LPC_INTERRUPT_COUNT || p_mode > 4) return SFP_ERR_ARG_VALUE;

	NVIC_DisableIRQ(p_intID);	// Disable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!

	LPC_SYSCON->PINTSEL[p_intID] = LPC_PIN_IDS[p_pin]; 	// select which pin will cause the interrupts

	// XXX: using SI/CI ENF and ENR registers could probably save few instructions
	switch (p_mode) {
		case 0: {	// LOW level mode
			LPC_GPIO_PIN_INT ->ISEL |= (1 << p_intID);	// Set PMODE=level sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable level interrupt.
			LPC_GPIO_PIN_INT ->IENF &= ~(1 << p_intID);	// Set active level LOW.
			break;
		}
		case 1: {	// HIGH level mode
			LPC_GPIO_PIN_INT ->ISEL |= (1 << p_intID);	// Set PMODE=level sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable level interrupt.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Set active level HIGH.
			break;
		}
		case 2: {	// Edge CHANGE mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable rising edge.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Enable falling edge.
			break;
		}
		case 3: {	// RISING edge mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR |= (1 << p_intID);	// Enable rising edge.
			LPC_GPIO_PIN_INT ->IENF &= ~(1 << p_intID);	// Disable falling edge.
			break;
		}
		case 4: {	// FALLING edge mode
			LPC_GPIO_PIN_INT ->ISEL &= ~(1 << p_intID);	// Set PMODE=edge sensitive
			LPC_GPIO_PIN_INT ->IENR &= ~(1 << p_intID);	// Disable rising edge.
			LPC_GPIO_PIN_INT ->IENF |= (1 << p_intID);	// Enable falling edge.
			break;
		}
	}

	LPC_INTERRUPT_FUNCTION_TYPE[p_intID] = SFPFunction_getType(func);
	LPC_INTERRUPT_DESTINATION[p_intID] = SFPFunction_getArgument_int32(func, 0);

	LPC_GPIO_PIN_INT->RISE = (1 << p_intID);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << p_intID);	// Clear falling edge (sort of) flag
	NVIC_SetPriority(p_intID, 3); // set lowest priority
	NVIC_EnableIRQ(p_intID);	// Enable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!

	return SFP_OK;
}

SFPResult lpc_detachInterrupt(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 2)
		return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(msg, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(msg, 1) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint8_t p_intID = SFPFunction_getArgument_int32(msg, 1);	// interrupt ID

	NVIC_DisableIRQ(p_intID);	// Disable interrupt. XXX: Luckily FLEX_INTx_IRQn == x, so it can be used this way, otherwise BE AWARE!
	LPC_GPIO_PIN_INT->CIENR = (1 << p_intID);	// Disable rising edge or level interrupt
	LPC_GPIO_PIN_INT->CIENF = (1 << p_intID);	// Disable falling edge interrupt
	LPC_GPIO_PIN_INT->RISE = (1 << p_intID);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << p_intID);	// Clear falling edge (sort of) flag

	return SFP_OK;
}

static void GPIO_EnableInterrupt(uint8_t intID) {
	LPC_GPIO_PIN_INT->RISE = (1<<intID);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1<<intID);	// Clear falling edge (sort of) flag
	NVIC_EnableIRQ(intID);	// Enable ISR
}

static void GPIO_EnableInterruptCallback(void* ptr) {
	GPIO_EnableInterrupt((uint8_t)(uint32_t)ptr);
}

void GPIO_handleInterrupts(void) {
	uint8_t i;
	for (i=0; i<LPC_INTERRUPT_COUNT; i++) {
		if (LPC_INTERRUPT_FLAG & (1<<i)) {

			System_sendInterrupt(SYSTEM_INTERRUPT_GPIO, i, LPC_INTERRUPT_TYPE[i]);

			LPC_INTERRUPT_FLAG &= ~(1<<i);
			LPC_INTERRUPT_TYPE[i] = 0xFF;
			// XXX: maybe instead of timer enable interrupt here - this would solve the need of INTERRUPT_TYPE buffer
		}
	}
}

static void GPIO_registerInterrupt(uint8_t interruptID) {
	uint8_t intBit = (1 << interruptID);
	if (LPC_GPIO_PIN_INT->IST & intBit) {
		NVIC_DisableIRQ(interruptID);  // Disable IRQ

		uint8_t interruptEvent = 0xFF;

		if ((LPC_GPIO_PIN_INT->ISEL & intBit)) {	// if LEVEL mode
			if (LPC_GPIO_PIN_INT->IENR & intBit) {	// if LEVEL interrupts are enabled
				if (LPC_GPIO_PIN_INT->IENF & intBit) {	// HIGH mode
					interruptEvent = 1;
				} else {								// LOW mode
					interruptEvent = 0;
				}
			}
		} else {	// EDGE mode
			if ((LPC_GPIO_PIN_INT->RISE & intBit) && (LPC_GPIO_PIN_INT->IENR & intBit)) {	// Rising edge interrupt
				interruptEvent = 3;
			}
			if ((LPC_GPIO_PIN_INT->FALL & intBit) && (LPC_GPIO_PIN_INT->IENF & intBit)) {	// Falling edge interrupt
				if (interruptEvent == 3)
					interruptEvent = 2;				// Edge CHANGE (RISE+FALL)
				else
					interruptEvent = 4;				// Falling edge
			}
		}

		LPC_INTERRUPT_FLAG |= intBit; // Mark pending IRQ
		LPC_INTERRUPT_TYPE[interruptID] = interruptEvent;

		Time_addTimer(50, GPIO_EnableInterruptCallback, (void*)(uint32_t)interruptID); // Schedule enable event
	}
}

void FLEX_INT0_IRQHandler() {
	GPIO_registerInterrupt(0);
}

void FLEX_INT1_IRQHandler() {
	GPIO_registerInterrupt(1);
}

void FLEX_INT2_IRQHandler() {
	GPIO_registerInterrupt(2);
}

void FLEX_INT3_IRQHandler() {
	GPIO_registerInterrupt(3);
}

void FLEX_INT4_IRQHandler() {
	GPIO_registerInterrupt(4);
}

void FLEX_INT5_IRQHandler() {
	GPIO_registerInterrupt(5);
}
