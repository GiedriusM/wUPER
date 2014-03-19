/**
 * @file	main.c
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

#include "main.h"

#include "CDC/CDC.h"

#define EEPROM_MAGIC_NUMBER		0xA5E769BD

#define EEPROM_ADDR_MAGIC		0x0
#define EEPROM_ADDR_SYSSETTINGS	0x10
#define EEPROM_ADDR_RFSETTINGS	0x50
#define EEPROM_ADDR_NODES		0x100

static volatile uint32_t System_powerSaveStart;

/* USB power interrupt */
void FLEX_INT6_IRQHandler() {
	if (LPC_GPIO_PIN_INT->IST & (1<<6)) {
		NVIC_SystemReset();
	}
}

int main(void) {
	SystemCoreClockUpdate();

	Time_init();

	IAP_GetSerialNumber(GUID);

	LPC_SYSCON->SYSAHBCLKCTRL |= BIT6 | BIT16 | BIT19; // Enable clock for GPIO, IOConfig and Pin Interrupts

#ifndef DEBUG
	// Disabled for debugging (JTAG)
	lpc_config_gpioInit();
#endif

	// PIO0_4 and PIO0_5 forced to I2C
	LPC_IOCON->PIO0_4 |= 1;	// I2C SCL
	LPC_IOCON->PIO0_5 |= 1;	// I2C SDA

	/* Temporary and test configs */
	// XXX: LED config - leave it for now
	LPC_GPIO->DIR[0] |= BIT7;
	LPC_GPIO->CLR[0] |= BIT7;

	System_mode = SYSTEM_MODE_ACTIVE;

#ifdef WUPER_NODE
	// Configure USB sense pin + interrupt
	LPC_IOCON->PIO0_3 &= ~0x1F;
	LPC_IOCON->PIO0_3 |= (1 << 3)|(1 << 0);	/* Secondary function VBUS */
	LPC_GPIO->DIR[0] &= ~(1 << 3);		// input
	Time_delay(10); // XXX: this solves stabilization issue where interrupt happens immediately
	NVIC_DisableIRQ(FLEX_INT6_IRQn);	// Disable interrupt
	LPC_SYSCON->PINTSEL[6] = 0+3; 	// select which pin will cause the interrupts
	LPC_GPIO_PIN_INT->ISEL &= ~(1 << 6);// Set edge sensitive mode
	LPC_GPIO_PIN_INT->IENR |= (1 << 6);	// Enable rising edge.
	LPC_GPIO_PIN_INT->IENF |= (1 << 6);	// Enable falling edge.
	LPC_GPIO_PIN_INT->RISE = (1 << 6);	// Clear rising edge flag
	LPC_GPIO_PIN_INT->FALL = (1 << 6);	// Clear falling edge flag
	NVIC_EnableIRQ(FLEX_INT6_IRQn);		// Enable interrupt

	// Check current USB state
	if (!(LPC_GPIO->PIN[0] & (1 << 3))) { // If no Vcc on PIO0_3
		System_mode = SYSTEM_MODE_POWER_SAVE;
	}
#endif

	System_powerSaveTimeout = 5000; // 5 seconds
	System_powerDownTimeout = 10; // 10 seconds

	/* Initialize Spirit side */
	WUPER_Init(&spirit_stream, GUID);
	System_loadSettings();

	/* Spirit SFP server initialization*/
	SFPServer *spiritServer = SFPServer_new(&spirit_stream);

#ifdef WUPER_NODE
	/* GPIO/Pin functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SETPRIMARY,	WUPER_RF_FID_SETPRIMARY,	lpc_config_setPrimary);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SETSECONDARY, WUPER_RF_FID_SETSECONDARY,	lpc_config_setSecondary);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PINMODE,      WUPER_RF_FID_PINMODE,		lpc_pinMode);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DIGITALWRITE, WUPER_RF_FID_DIGITALWRITE,	lpc_digitalWrite);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DIGITALREAD,  WUPER_RF_FID_DIGITALREAD,	lpc_digitalRead);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_ATTACHINTERRUPT, WUPER_RF_FID_ATTACHINTERRUPT, lpc_attachInterrupt);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DETACHINTERRUPT, WUPER_RF_FID_DETACHINTERRUPT, lpc_detachInterrupt);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PULSEIN, WUPER_RF_FID_PULSEIN, lpc_pulseIn);

	/* ADC functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_ANALOGREAD, WUPER_RF_FID_ANALOGREAD, lpc_analogRead);

	/* SPI0 functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SPI0BEGIN, WUPER_RF_FID_SPI0BEGIN, 	lpc_spi0_begin);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SPI0TRANS, WUPER_RF_FID_SPI0TRANS, 	lpc_spi0_trans);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SPI0END,   WUPER_RF_FID_SPI0END, 		lpc_spi0_end);

	/* I2C functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_I2CBEGIN, WUPER_RF_FID_I2CBEGIN, lpc_i2c_begin);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_I2CTRANS, WUPER_RF_FID_I2CTRANS, lpc_i2c_trans);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_I2CEND,   WUPER_RF_FID_I2CEND, lpc_i2c_end);

	/* PWM functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM0BEGIN, WUPER_RF_FID_PWM0BEGIN, lpc_pwm0_begin);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM0SET,   WUPER_RF_FID_PWM0SET, lpc_pwm0_set);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM0END,   WUPER_RF_FID_PWM0END, lpc_pwm0_end);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM1BEGIN, WUPER_RF_FID_PWM1BEGIN, lpc_pwm1_begin);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM1SET,   WUPER_RF_FID_PWM1SET, lpc_pwm1_set);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PWM1END,   WUPER_RF_FID_PWM1END, lpc_pwm1_end);
#endif
	/* System functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PING, WUPER_RF_FID_PING, PingReceiveCallback);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PONG, WUPER_RF_FID_PONG, PongReceiveCallback);

	SFPServer_setDefaultFunctionHandler(spiritServer, SpiritDefaultCallback);


	if (System_mode == SYSTEM_MODE_ACTIVE) {
		/* Initialize CDC  */
		CDC_Init(&stream, GUID);

		/* CDC SFP server initialization */
		SFPServer *cdcServer = SFPServer_new(&stream);

		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETDEVADDRESS,	WUPER_CDC_FID_GETDEVADDRESS,	GetDeviceAddrCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETTRAFFICINFO,	WUPER_CDC_FID_GETTRAFFICINFO,	GetTrafficInfoCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_CLEARTRAFFICINFO,	WUPER_CDC_FID_CLEARTRAFFICINFO,	ClearTrafficInfoCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SETRFPARAMS,WUPER_CDC_FID_SETRFPARAMS,	SetRFParamsCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETRFPARAMS,WUPER_CDC_FID_GETRFPARAMS,	GetRFParamsCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_PING,		WUPER_CDC_FID_PING,			PingSendCallback);

		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_ADDNODE,	WUPER_CDC_FID_ADDNODE,		AddNodeCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_DELNODE,	WUPER_CDC_FID_DELNODE,		DelNodeCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_CLRNODES,	WUPER_CDC_FID_CLRNODES,		ClearNodesCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETNODES,	WUPER_CDC_FID_GETNODES,		GetNodesCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETNODEINFO,WUPER_CDC_FID_GETNODEINFO,	GetNodeInfoCallback);

		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SAVESETTINGS, WUPER_CDC_FID_SAVESETTINGS, SaveSettingsCallback);

#ifdef WUPER_NODE
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_ENTERPOWERSAVE, WUPER_CDC_FID_ENTERPOWERSAVE, EnterPowerSaveCallback);

		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SETSYSSETTINGS,	WUPER_CDC_FID_SETSYSSETTINGS, SetSystemSettingsCallback);
		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETSYSSETTINGS,	WUPER_CDC_FID_GETSYSSETTINGS, GetSystemSettingsCallback);
#endif

		SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETDEVINFO,	WUPER_CDC_FID_GETDEVINFO, lpc_system_getDeviceInfo);

		SFPServer_setDefaultFunctionHandler(cdcServer, CDCDefaultCallback);

		while (System_mode == SYSTEM_MODE_ACTIVE) {
#ifdef WUPER_NODE
			GPIO_handleInterrupts();
#endif
			SFPServer_cycle(cdcServer);
			SFPServer_cycle(spiritServer);
		}

		SFPServer_delete(cdcServer);
		CDC_Shutdown();
	}

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SLEEP, WUPER_RF_FID_SLEEP, SleepCallback);
	while (1) {
		System_resetPowerSaveTimeout();
		while (Time_getSystemTime()-System_powerSaveStart < System_powerSaveTimeout) {
			/* Interrupt part */
			if (LPC_SYSCON->SYSRSTSTAT & 0x1F) { // System reset
				System_sendInterrupt(SYSTEM_INTERRUPT_RESET, LPC_SYSCON->SYSRSTSTAT & 0x1F, 0);
				LPC_SYSCON->SYSRSTSTAT = 0x1F; // Clear flags
			}
			GPIO_handleInterrupts();

			/* Processing part */
			SFPServer_cycle(spiritServer);
		}
		System_enterPowerDown(System_powerDownTimeout);
	}

}

uint8_t System_loadSettings(void) {
	uint32_t magicHeader = 0;
	IAP_ReadEEPROM(EEPROM_ADDR_MAGIC, (uint8_t*)&magicHeader, 4);
	if (magicHeader != EEPROM_MAGIC_NUMBER)
		return 0;

	// XXX: it might be worth making settings structure, like WUPERSettings
	IAP_ReadEEPROM(EEPROM_ADDR_SYSSETTINGS, (uint8_t*)&System_powerSaveTimeout, 4);
	IAP_ReadEEPROM(EEPROM_ADDR_SYSSETTINGS+4, (uint8_t*)&System_powerDownTimeout, 4);

	// RF Settings
	WUPERSettings wuperSettings = { 0 };
	IAP_ReadEEPROM(EEPROM_ADDR_RFSETTINGS, (uint8_t*)&wuperSettings, sizeof(WUPERSettings));
	WUPER_SetSettings(&wuperSettings);

	// Nodes
	uint8_t nNodes = 0;
	IAP_ReadEEPROM(EEPROM_ADDR_NODES, &nNodes, 1);

	uint32_t addr = EEPROM_ADDR_NODES+1;
	while (nNodes--) {
		uint32_t tmpNodeAddress = 0;
		IAP_ReadEEPROM(addr, (uint8_t*)&tmpNodeAddress, 4);
		addr += 4;
		WUPER_AddNode(tmpNodeAddress);
	}

	return 1;
}

uint8_t System_saveSettings(void) {
	// "Delete" old config by clearing magic "version" number
	uint32_t magicHeader = 0;
	IAP_WriteEEPROM(EEPROM_ADDR_MAGIC, (uint8_t*)&magicHeader, 4);


	// Save System settings
	IAP_WriteEEPROM(EEPROM_ADDR_SYSSETTINGS, (uint8_t*)&System_powerSaveTimeout, 4);
	IAP_WriteEEPROM(EEPROM_ADDR_SYSSETTINGS+4, (uint8_t*)&System_powerDownTimeout, 4);

	// Save RF settings
	WUPERSettings wuperSettings = { 0 };
	WUPER_GetSettings(&wuperSettings);
	IAP_WriteEEPROM(EEPROM_ADDR_RFSETTINGS, (uint8_t*)&wuperSettings, sizeof(WUPERSettings));

	// Save nodes
	uint8_t nNodes = WUPER_GetNodeCount();
	IAP_WriteEEPROM(EEPROM_ADDR_NODES, &nNodes, 1);

	uint32_t addr = EEPROM_ADDR_NODES+1;
	uint8_t i;
	for (i=0; i<nNodes; i++) {
		uint32_t tmpNodeAddress = WUPER_GetNodeAddress(i);
		IAP_WriteEEPROM(addr, (uint8_t*)&tmpNodeAddress, 4);
		addr += 4;
	}


	// Finally confirm settings by writing magic number
	magicHeader = EEPROM_MAGIC_NUMBER;
	IAP_WriteEEPROM(EEPROM_ADDR_MAGIC, (uint8_t*)&magicHeader, 4);

	return 1;
}

void System_sendInterrupt(uint8_t interruptType, uint32_t param1, uint32_t param2) {
	SFPFunction *func = SFPFunction_new();
	if (func != NULL) {
		WUPER_SetDestinationAddress(WUPER_GetNodeAddress(0));

		SFPFunction_setType(func, SFP_FUNC_TYPE_TEXT);
		SFPFunction_setID(func, WUPER_RF_FID_INTERRUPT);
		SFPFunction_setName(func, WUPER_RF_FNAME_INTERRUPT);

		SFPFunction_addArgument_int32(func, WUPER_GetDeviceAddress());
		SFPFunction_addArgument_int32(func, interruptType);
		SFPFunction_addArgument_int32(func, param1);
		SFPFunction_addArgument_int32(func, param2);

		SFPFunction_send(func, &spirit_stream);
		SFPFunction_delete(func);
	}
}

void System_resetPowerSaveTimeout(void) {
	System_powerSaveStart = Time_getSystemTime();
}

void System_enterPowerDown(uint32_t timeout) {
	if (timeout == 0)
		return;

	WUPER_Shutdown();
	System_mode = SYSTEM_MODE_POWER_DOWN;
	Power_startWatchdog(timeout);
	Power_enterPowerDown();

	Power_exitPowerDown();
	Power_stopWatchdog();
	System_mode = SYSTEM_MODE_POWER_SAVE;
	WUPER_Restart();

	System_sendInterrupt(SYSTEM_INTERRUPT_WAKEUP, 0, 0);
}
