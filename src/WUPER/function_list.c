/**
 * @file	function_list.c
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

#include "WUPER/function_list.h"

SFPResult PingSendCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 1) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT) return SFP_ERR_ARG_TYPE;

	uint32_t addr = SFPFunction_getArgument_int32(func, 0);

	pingData.address = addr;
	pingData.startTime = Time_getSystemTime();
	pingData.timeout = 10000; // 10s timeout
	pingData.size = 0;

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
	if (SFPFunction_getArgumentCount(func) != 8) return SFP_ERR_ARG_COUNT;

	if (SFPFunction_getArgumentType(func, 0) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 1) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 2) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 3) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 4) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 5) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 6) != SFP_ARG_INT
			|| SFPFunction_getArgumentType(func, 7) != SFP_ARG_INT)
		return SFP_ERR_ARG_TYPE;

	WUPERSettings settings = {
			.frequency = SFPFunction_getArgument_int32(func, 0),
			.datarate = SFPFunction_getArgument_int32(func, 1),
			.modulation =  { SFPFunction_getArgument_int32(func, 2) },
			.freqDev = SFPFunction_getArgument_int32(func, 3),
			.txPowerdBm = SFPFunction_getArgument_int32(func, 4),
			.networkID = SFPFunction_getArgument_int32(func, 5),
			.sendRetryCount = SFPFunction_getArgument_int32(func, 6),
			.ackWaitTimeout = SFPFunction_getArgument_int32(func, 7)
	};

	/*WUPERSettings settings;
	settings.frequency = SFPFunction_getArgument_int32(func, 0);
	settings.datarate = SFPFunction_getArgument_int32(func, 1);
	uint32_t mod = SFPFunction_getArgument_int32(func, 2);
	settings.modulation.type = mod & 7;
	settings.modulation.BT1 = (mod >> 3) & 1;
	settings.modulation.whitening = (mod >> 6) & 1;
	settings.modulation.fec = (mod >> 7) & 1;
	settings.freqDev = SFPFunction_getArgument_int32(func, 3);

	settings.txPowerdBm = SFPFunction_getArgument_int32(func, 4);

	settings.networkID = SFPFunction_getArgument_int32(func, 5);

	settings.sendRetryCount = SFPFunction_getArgument_int32(func, 6);
	settings.ackWaitTimeout = SFPFunction_getArgument_int32(func, 7);*/

	WUPER_SetRFSettings(&settings);

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

SFPResult SaveSettingsCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	System_saveWuperSettings();

	return SFP_OK;
}

SFPResult EnterPowerSaveCallback(SFPFunction *func) {
	if (SFPFunction_getArgumentCount(func) != 0) return SFP_ERR_ARG_COUNT;

	System_mode = SYSTEM_MODE_POWER_SAVE;

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

	System_enterPowerDown(sleepTime);
	System_resetPowerSaveTimeout();

	return SFP_OK;
}