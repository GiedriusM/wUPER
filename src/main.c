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

#include "WUPER/wuper.h"

#include "IAP.h"

SFPResult LedCallback(SFPFunction *msg) {
	LPC_GPIO->NOT[0] |= BIT7;

	return SFP_OK;
}

struct {
	uint32_t address;
	uint32_t startTime;
	uint32_t timeout;
	uint8_t size;
} pingData;

SFPResult PingSendCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	pingData.address = addr;
	pingData.startTime = Time_getSystemTime();
	pingData.timeout = 10000; // 10s timeout
	pingData.size = 0;
	LedCallback(NULL);

	SFPFunction *outFunc = SFPFunction_new();
	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	WUPER_SetDestinationAddress(addr);

	SFPFunction_setType(outFunc, SFP_FUNC_TYPE_BIN);
	SFPFunction_setID(outFunc, WUPER_RF_FID_PING);
	SFPFunction_addArgument_int32(outFunc, WUPER_GetDeviceAddress());
	SFPFunction_addArgument_barray(outFunc, NULL, pingData.size);
	SFPFunction_send(outFunc, &spirit_stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult PingReceiveCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 2) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT &&
			SFPFunction_getArgumentType(func, 0) != SFP_ARG_BYTE_ARRAY) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);
	uint32_t size = 0;
	uint8_t *data = SFPFunction_getArgument_barray(func, 1, &size);

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	WUPER_SetDestinationAddress(addr);

	SFPFunction_setType(outFunc, SFP_FUNC_TYPE_BIN);
	SFPFunction_setID(outFunc, WUPER_RF_FID_PONG);
	SFPFunction_addArgument_int32(outFunc, WUPER_GetDeviceAddress());
	SFPFunction_addArgument_barray(outFunc, data, size);
	SFPFunction_send(outFunc, &spirit_stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult PongReceiveCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 2) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT &&
			SFPFunction_getArgumentType(func, 0) != SFP_ARG_BYTE_ARRAY) return SFP_ERR_ARG_TYPE;

	uint32_t deltaTime = Time_getSystemTime() - pingData.startTime;
	if (deltaTime > pingData.timeout)
		return SFP_OK;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);
	uint32_t size = 0;
	SFPFunction_getArgument_barray(func, 1, &size);

	if (addr != pingData.address || size != pingData.size)
		return SFP_OK;

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	SFPFunction_setType(outFunc, SFP_FUNC_TYPE_TEXT);
	SFPFunction_setName(outFunc, WUPER_CDC_FNAME_PONG);
	SFPFunction_setID(outFunc, WUPER_CDC_FID_PONG);
	SFPFunction_addArgument_int32(outFunc, addr);
	SFPFunction_addArgument_int32(outFunc, deltaTime);
	SFPFunction_send(outFunc, &stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult GetDeviceAddrCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	SFPFunction_setType(outFunc, SFPFunction_getType(func));
	SFPFunction_setName(outFunc, WUPER_CDC_FNAME_GETDEVINFO);
	SFPFunction_setID(outFunc, WUPER_CDC_FID_GETDEVINFO);
	SFPFunction_addArgument_int32(outFunc, WUPER_GetDeviceAddress());
	SFPFunction_send(outFunc, &stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult GetTrafficInfoCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	SFPFunction *outFunc = SFPFunction_new();

	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	WUPERTrafficStatistics stats;
	WUPER_GetTrafficStatistics(&stats);

	SFPFunction_setType(outFunc, SFPFunction_getType(func));
	SFPFunction_setName(outFunc, WUPER_CDC_FNAME_GETTRAFFICINFO);
	SFPFunction_setID(outFunc, WUPER_CDC_FID_GETTRAFFICINFO);
	SFPFunction_addArgument_int32(outFunc, stats.packetSendTotal);
	SFPFunction_addArgument_int32(outFunc, stats.packetSendSuccess);
	SFPFunction_addArgument_int32(outFunc, stats.packetSendRetry);
	SFPFunction_addArgument_int32(outFunc, stats.packetSeqSet);
	SFPFunction_send(outFunc, &stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult SetNetworkCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	WUPER_SetNetworkID(SFPFunction_getArgument_int32(func, 0));

	return SFP_OK;
}

SFPResult SetAESKeyCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_BYTE_ARRAY) return SFP_ERR_ARG_TYPE;

	uint32_t size = 0;
	uint8_t *key = SFPFunction_getArgument_barray(func, 0, &size);

	if (size != 16) return SFP_ERR;

	WUPER_SetAESKey(key);

	return SFP_OK;
}

SFPResult SetRFParamsCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 7) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 2) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 3) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 4) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 5) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 6) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint32_t frequency = SFPFunction_getArgument_int32(func, 0);
	uint32_t datarate = SFPFunction_getArgument_int32(func, 1);
	uint8_t modulationFlags = SFPFunction_getArgument_int32(func, 2);
	uint32_t fdev = SFPFunction_getArgument_int32(func, 3);

	int8_t txPower = SFPFunction_getArgument_int32(func, 4);
	uint32_t retryCount = SFPFunction_getArgument_int32(func, 5);
	uint32_t ackTimeout = SFPFunction_getArgument_int32(func, 6);

	WUPER_SetRFParameters(frequency, datarate, modulationFlags, fdev, txPower, retryCount, ackTimeout);

	return SFP_OK;
}

SFPResult CDCDefaultCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) < 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	WUPER_SetDestinationAddress(SFPFunction_getArgument_int32(func, 0));
	SFPFunction_setArgument_int32(func, 0, WUPER_GetDeviceAddress());
	SFPFunction_send(func, &spirit_stream);
	return SFP_OK;
}

SFPResult SpiritDefaultCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) < 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	if (!WUPER_ContainsNode(addr)) return SFP_ERR;

	SFPFunction_send(func, &stream);
	return SFP_OK;
}

SFPResult AddNodeCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	if (WUPER_AddNode(addr) != WUPER_OK) return SFP_ERR;

	return SFP_OK;
}

SFPResult DelNodeCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	if (WUPER_DeleteNode(addr) != WUPER_OK) return SFP_ERR;

	return SFP_OK;
}

SFPResult ClearNodesCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	if (WUPER_ClearNodes() != WUPER_OK) return SFP_ERR;

	return SFP_OK;
}

SFPResult GetNodesCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	SFPFunction *outFunc = SFPFunction_new();
	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	SFPFunction_setType(outFunc, SFPFunction_getType(func));
	SFPFunction_setName(outFunc, WUPER_CDC_FNAME_GETNODES);
	SFPFunction_setID(outFunc, WUPER_CDC_FID_GETNODES);

	uint8_t i;
	uint32_t nodeCount = WUPER_GetNodeCount();
	for (i=0; i<nodeCount; i++)
		SFPFunction_addArgument_int32(outFunc, WUPER_GetNodeAddress(i));

	SFPFunction_send(outFunc, &stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult GetNodeInfoCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	uint16_t inSignalQuality, outSignalQuality;
	WUPER_GetNodeInfo(addr, &inSignalQuality, &outSignalQuality);

	SFPFunction *outFunc = SFPFunction_new();
	if (outFunc == NULL) return SFP_ERR_ALLOC_FAILED;

	SFPFunction_setType(outFunc, SFPFunction_getType(func));
	SFPFunction_setName(outFunc, WUPER_CDC_FNAME_GETNODEINFO);
	SFPFunction_setID(outFunc, WUPER_CDC_FID_GETNODEINFO);

	SFPFunction_addArgument_int32(outFunc, addr);
	SFPFunction_addArgument_int32(outFunc, inSignalQuality);
	SFPFunction_addArgument_int32(outFunc, outSignalQuality);

	SFPFunction_send(outFunc, &stream);
	SFPFunction_delete(outFunc);

	return SFP_OK;
}

SFPResult lpc_system_getDeviceInfo(SFPFunction *msg) {
	if (SFPFunction_getArgumentCount(msg) != 0) return SFP_ERR_ARG_COUNT;

	SFPFunction *func = SFPFunction_new();

	if (func == NULL) return SFP_ERR_ALLOC_FAILED;

	SFPFunction_setType(func, SFPFunction_getType(msg));
	SFPFunction_setID(func, WUPER_CDC_FID_GETDEVINFO);
	SFPFunction_setName(func, WUPER_CDC_FNAME_GETDEVINFO);
	SFPFunction_addArgument_int32(func, WUPER_FIRMWARE_VERSION);
	SFPFunction_addArgument_barray(func, (uint8_t*)&GUID[0], 16);
	SFPFunction_addArgument_int32(func, IAP_GetPartNumber());
	SFPFunction_addArgument_int32(func, IAP_GetBootCodeVersion());
	SFPFunction_send(func, &stream);
	SFPFunction_delete(func);

	return SFP_OK;
}

SFPResult SleepCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 2) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 1) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	uint32_t sleepTime = SFPFunction_getArgument_int32(func, 1);

	WUPER_Shutdown();
	isInPowerDown = 1;
	Power_startWatchdog(sleepTime);
	Power_enterPowerDown();

	if (isInPowerDown) {
		Power_exitPowerDown();
		Power_stopWatchdog();
		isInPowerDown = 0;
		WUPER_Restart();
	}

	return SFP_OK;
}

int main(void) {
	SystemCoreClockUpdate();

	Time_init();

	IAP_GetSerialNumber(GUID);

	LPC_SYSCON->SYSAHBCLKCTRL |= BIT6 | BIT16 | BIT19; // Enable clock for GPIO, IOConfig and Pin Interrupts

#ifndef DEBUG
	// Disabled for debugging (JTAG)
	uint8_t pin;
	for (pin=0; pin<LPC_GPIO_PIN_COUNT; pin++)
		*LPC_PIN_REGISTERS[pin] = (*LPC_PIN_REGISTERS[pin] & ~LPC_PIN_FUNCTION_MASK) | LPC_PIN_PRIMARY_FUNCTION[pin];
#endif

	// PIO0_4 and PIO0_5 forced to I2C
	LPC_IOCON->PIO0_4 |= 1;	// I2C SCL
	LPC_IOCON->PIO0_5 |= 1;	// I2C SDA

	/* Temporary and test configs */
	// XXX: LED config - leave it for now
	LPC_GPIO->DIR[0] |= BIT7;
	LPC_GPIO->CLR[0] |= BIT7;

	/* Load CDC and Spirit streams */
	CDC_Init(&stream, GUID);
	WUPER_Init(&spirit_stream, GUID);

	/* SFP initialization, configuration and launch */
	SFPServer *cdcServer = SFPServer_new(&stream);
	SFPServer *spiritServer = SFPServer_new(&spirit_stream);

	/* CDC SFP server initialization */
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETDEVADDRESS,	WUPER_CDC_FID_GETDEVADDRESS,	GetDeviceAddrCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETTRAFFICINFO,	WUPER_CDC_FID_GETTRAFFICINFO,	GetTrafficInfoCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SETNETWORK, WUPER_CDC_FID_SETNETWORK,	SetNetworkCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SETAESKEY,	WUPER_CDC_FID_SETAESKEY,	SetAESKeyCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_SETRFPARAMS,WUPER_CDC_FID_SETRFPARAMS,	SetRFParamsCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_PING,		WUPER_CDC_FID_PING,			PingSendCallback);

	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_ADDNODE,	WUPER_CDC_FID_ADDNODE,		AddNodeCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_DELNODE,	WUPER_CDC_FID_DELNODE,		DelNodeCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_CLRNODES,	WUPER_CDC_FID_CLRNODES,		ClearNodesCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETNODES,	WUPER_CDC_FID_GETNODES,		GetNodesCallback);
	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETNODEINFO,WUPER_CDC_FID_GETNODEINFO,	GetNodeInfoCallback);

	SFPServer_addFunctionHandler(cdcServer, WUPER_CDC_FNAME_GETDEVINFO,	WUPER_CDC_FID_GETDEVINFO, lpc_system_getDeviceInfo);

	SFPServer_setDefaultFunctionHandler(cdcServer, CDCDefaultCallback);


	/* Spirit SFP server initialization*/
	/* GPIO/Pin functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SETPRIMARY,	WUPER_RF_FID_SETPRIMARY,	lpc_config_setPrimary);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_SETSECONDARY, WUPER_RF_FID_SETSECONDARY,	lpc_config_setSecondary);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PINMODE,      WUPER_RF_FID_PINMODE,		lpc_pinMode);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DIGITALWRITE, WUPER_RF_FID_DIGITALWRITE,	lpc_digitalWrite);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DIGITALREAD,  WUPER_RF_FID_DIGITALREAD,	lpc_digitalRead);

	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_ATTACHINTERRUPT, WUPER_RF_FID_ATTACHINTERRUPT, lpc_attachInterrupt);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_DETACHINTERRUPT, WUPER_RF_FID_DETACHINTERRUPT, lpc_detachInterrupt);

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

	/* System functions */
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PING, WUPER_RF_FID_PING, PingReceiveCallback);
	SFPServer_addFunctionHandler(spiritServer, WUPER_RF_FNAME_PONG, WUPER_RF_FID_PONG, PongReceiveCallback);
	SFPServer_addFunctionHandler(spiritServer, "sleep", 0xFE, SleepCallback);
	SFPServer_addFunctionHandler(spiritServer, "led", 0x81, LedCallback);

	SFPServer_setDefaultFunctionHandler(spiritServer, SpiritDefaultCallback);


	while (1) {
		SFPServer_cycle(cdcServer);
		SFPServer_cycle(spiritServer);
		GPIO_handleInterrupts();
	}

	SFPServer_delete(cdcServer);
	SFPServer_delete(spiritServer);

	while (1);
}
