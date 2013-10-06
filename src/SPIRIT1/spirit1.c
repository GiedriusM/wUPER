/**
 * @file	spirit.c
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


#include "SPIRIT1/spirit1.h"

#define LPC_PIN_FUNCTION_MASK	(BIT7 | 7)		// FUNC bits + AD bit
#define LPC_PIN_MODE_MASK		(3 << 3)

void SdkEvalSpiInit(void) {
	SPIRIT1_Init();
}

StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	SPIRIT1_SPISlaveEnable();

	uint16_t status = (SPIRIT1_SPITrans(0x00) << 8) | SPIRIT1_SPITrans(cRegAddress);

	uint8_t i;
	for (i=0; i<cNbBytes; i++) {
		SPIRIT1_SPITrans(*pcBuffer++);
	}

	SPIRIT1_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	SPIRIT1_SPISlaveEnable();
	uint16_t status = (SPIRIT1_SPITrans(0x01) << 8) | SPIRIT1_SPITrans(cRegAddress);

	uint8_t i;
	for (i=0; i<cNbBytes; i++)
		*pcBuffer++ = SPIRIT1_SPITrans(0xFF);

	SPIRIT1_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode) {
	SPIRIT1_SPISlaveEnable();
	uint16_t status = (SPIRIT1_SPITrans(0x80) << 8) | SPIRIT1_SPITrans(cCommandCode);
	SPIRIT1_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	return SdkEvalSpiWriteRegisters(0xFF, cNbBytes, pcBuffer);
}

StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	return SdkEvalSpiReadRegisters(0xFF, cNbBytes, pcBuffer);
}

void SPIRIT1_Init(void) {
	/* GPIO config */
	// Shutdown
	LPC_IOCON->PIO1_29 &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors
	LPC_GPIO->DIR[1] |= (1 << 29);				// Set output
	LPC_GPIO->SET[1] = (1 << 29);				// Turn Shutdown ON
	// GPIO0
	LPC_IOCON->TDI_PIO0_11 = (LPC_IOCON->TDI_PIO0_11 & ~LPC_PIN_MODE_MASK) | 0x01 | (0x2 << 3);
	LPC_GPIO->DIR[0] &= ~(1 << 11);				// Set input
	// GPIO1
	LPC_IOCON->TMS_PIO0_12 = (LPC_IOCON->TMS_PIO0_12 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 12);				// Set input
	// GPIO2
	LPC_IOCON->TDO_PIO0_13 = (LPC_IOCON->TDO_PIO0_13 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 13);				// Set input
	// GPIO3
	LPC_IOCON->TRST_PIO0_14 = (LPC_IOCON->TRST_PIO0_14 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 14);				// Set input
	// SSEL
	LPC_IOCON->PIO0_20 &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors
	LPC_GPIO->DIR[0] |= (1 << 20);				// Set output
	LPC_GPIO->SET[0] = (1 << 20);				// SSEL high

	//LPC_IOCON->PIO0_8 = (LPC_IOCON->PIO0_8 & ~LPC_PIN_FUNCTION_MASK) | 0x01; // MISO0
	//LPC_IOCON->PIO0_9 = (LPC_IOCON->PIO0_9 & ~LPC_PIN_FUNCTION_MASK) | 0x01; // MOSI0
	//LPC_IOCON->PIO0_6 = (LPC_IOCON->PIO0_6 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // SCK0

	LPC_IOCON->PIO1_21 = (LPC_IOCON->PIO1_21 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // MISO0
	LPC_IOCON->PIO0_21 = (LPC_IOCON->PIO0_21 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // MOSI0
	LPC_IOCON->PIO1_20 = (LPC_IOCON->PIO1_20 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // SCK0


	/* SPI config */
	SPIRIT1_SPIConfig(2, 0);

	LPC_GPIO->CLR[1] = (1 << 29);	// Turn Shutdown OFF
	Time_delay(10);

	/*
	 * SPIRIT1 config
	 */
	SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);
	SpiritGeneralSetSpiritVersion(SPIRIT_VERSION_3_0);

	SRadioInit spiritRadio;
	spiritRadio.lFrequencyBase = 868000000;
	spiritRadio.nXtalOffsetPpm = 0;
	spiritRadio.lDatarate = 250;
	spiritRadio.xModulationSelect = FSK;
	spiritRadio.lFreqDev = 15000;
	spiritRadio.lBandwidth = 147000;
	spiritRadio.cChannelNumber = 0;
	spiritRadio.nChannelSpace = 300000;

	PktBasicInit basicPacket = {
		PKT_PREAMBLE_LENGTH_04BYTES,
		PKT_SYNC_LENGTH_4BYTES,
		('C' << 24) | (('N' | 0x80) << 16) | ('Y' << 8) | ('S' | 0x80), //(('S' | 0x80) << 24) | ('Y' << 16) | (('N' | 0x80) << 8) | 'C',
		PKT_LENGTH_VAR,
		8, // Packet length size in bits
		PKT_CRC_MODE_16BITS_1,
		PKT_CONTROL_LENGTH_4BYTES,
		S_ENABLE, // Address field enabled
		S_DISABLE, // FEC disabled
		S_DISABLE // Whitening disabled
	};

	PktBasicAddressesInit basicPacketAddresses = {
			// XXX: there is probably a bug, where
			// device and broadcast address filter masks
			// are interchanged (or their addresses)
			S_DISABLE, 	// Disable broadcast
			3,			// Address (network id)
			S_DISABLE,	// Disable multicast
			0,			// Multicast address
			S_ENABLE,	// Enable address (netword id) filtering
			0			// Broadcast address
	};

	SpiritRadioInit(&spiritRadio); // XXX: WCP value is incorrect (according to datasheet)
	SpiritRadioSetPALeveldBm(0, 11);
	SpiritRadioSetPALevelMaxIndex(0);

	SpiritPktBasicInit(&basicPacket);
	SpiritPktBasicAddressesInit(&basicPacketAddresses);
	SpiritPktCommonSetDestinationAddress(3); // Address (network ID)

	SpiritPktCommonSetTransmittedCtrlField(('0' << 24) | ('P' << 16) | ('F' << 8) | 'S');
	SpiritPktCommonSetCtrlReference(('0' << 24) | ('P' << 16) | ('F' << 8) | 'S');
	SpiritPktCommonSetCtrlMask(0xFFFFFFFF);
	SpiritPktCommonFilterOnControlField(S_ENABLE);

	SpiritTimerSetRxTimeoutStopCondition(NO_TIMEOUT_STOP);
	SpiritRadioPersistenRx(S_ENABLE);

	SpiritAesMode(S_ENABLE);

	// Spirit IRQ
	SpiritIrqs irqInit = {0};
	irqInit.IRQ_RX_DATA_READY = S_SET;
	irqInit.IRQ_RX_DATA_DISC = S_SET;
	irqInit.IRQ_RX_FIFO_ERROR = S_SET;
	irqInit.IRQ_RX_FIFO_ALMOST_FULL = S_SET;
	irqInit.IRQ_AES_END = S_SET;

	SpiritIrqInit(&irqInit);
	SpiritIrqGetStatus(&irqInit); // Reset IRQ flags

	// Spirit GPIO
	SGpioInit gpioInit = {
			SPIRIT_GPIO_0, // TODO: change this to other GPIO leg (use GPIO0 for temperature)
			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
			SPIRIT_GPIO_DIG_OUT_IRQ
	};
	SpiritGpioInit(&gpioInit);

	/* Enable IRQ */
	LPC_SYSCON->PINTSEL[7] = 0+11;

	LPC_GPIO_PIN_INT->ISEL |= (1 << 7);	// Set PMODE=level sensitive
	LPC_GPIO_PIN_INT->IENR |= (1 << 7);	// Enable level interrupt.
	LPC_GPIO_PIN_INT->IENF &= ~(1 << 7);// Set active level LOW.

	LPC_GPIO_PIN_INT->RISE = (1 << 7);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << 7);	// Clear falling edge (sort of) flag
	NVIC_SetPriority(7, 2);
	NVIC_EnableIRQ(FLEX_INT7_IRQn);

	/* Enter RX state */
	SpiritSpiCommandStrobes(COMMAND_LOCKRX); // Lock RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

	SpiritSpiCommandStrobes(COMMAND_RX); // RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_RX);
}

void SPIRIT1_SPIConfig(uint32_t divider, uint8_t mode) {
	LPC_SYSCON->PRESETCTRL |= BIT2; 		// de-assert SPI1
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT18;	// enable SPI1 clock
	LPC_SYSCON->SSP1CLKDIV = 1; //48MHz

	LPC_SSP1->CR1 = 0;		// Master mode, SPI disabled
	LPC_SSP1->CPSR = 24;	// 48MHz/24 = 2MHz
	LPC_SSP1->CR0 = (0x7) | (0 << 4) | ((mode & 0x3) << 6) | ((divider-1) << 8); // 8bits, SPI mode 0
	LPC_SSP1->IMSC = 0;		// Interrupts disabled
	LPC_SSP1->CR1 = BIT1;	// Master mode, SPI enabled

	while (LPC_SSP1->SR & BIT4);	// wait while BUSY (reading or writing)

	while (LPC_SSP1->SR & BIT2) {	// Read while Rx FIFO not empty
		LPC_SSP1->DR;
	}
}

uint8_t inline SPIRIT1_SPITrans(uint8_t data) {
	LPC_SSP1->DR = data;
	while (!(LPC_SSP1->SR & BIT2));
	return LPC_SSP1->DR;
}

void inline SPIRIT1_SPISlaveEnable(void) {
	NVIC_DisableIRQ(FLEX_INT7_IRQn);
	LPC_GPIO->CLR[0] = (1 << 20);	// SSEL low

	// CSs to SCLK delay (should be 2us, but less still works)
	volatile uint16_t i;
	for(i=0;i<50;i++);
}

void inline SPIRIT1_SPISlaveDisable(void) {
	LPC_GPIO->SET[0] = (1 << 20);	// SSEL high
	NVIC_EnableIRQ(FLEX_INT7_IRQn);
}

void SPIRIT1_CalculateDatarateME(uint32_t datarate, uint8_t* dr_m, uint8_t* dr_e) {

	// Find the best exponent
	uint8_t e = 15;
	while (e > 0) {
		if (datarate >= (CLOCK_FREQUENCY >> (20 - e)))
			break;

		e--;
	}
	*dr_e = e;

	// Calculate mantissa
	uint32_t tmp = (datarate * (1 << (23 - e))) / (CLOCK_FREQUENCY >> (5));
	uint8_t calcMan;
	if (tmp < 256)
		calcMan = 0;
	else
		calcMan = tmp - 256;

	// Check if nearby values give a better result
	uint16_t datarateDiff = 0xFFFF;
	uint8_t j;
	for (j = 0; j < 3; j++) {
		uint8_t tmpMan = calcMan + j - 1;
		int16_t tmpDatarateDiff = datarate - (((256 + tmpMan) * (CLOCK_FREQUENCY >> (5))) >> (23 - e));
		if (tmpDatarateDiff < 0)
			tmpDatarateDiff = -tmpDatarateDiff;

		if (tmpDatarateDiff < datarateDiff) {
			datarateDiff = tmpDatarateDiff;
			*dr_m = tmpMan;
		}
	}
}
