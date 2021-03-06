/**
 * @file	function_list.h
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

#ifndef FUNCTION_LIST_H_
#define FUNCTION_LIST_H_

#include "main.h"

struct {
	uint32_t address;
	uint32_t startTime;
	uint32_t timeout;
	uint8_t size;
} pingData;

SFPResult PingSendCallback(SFPFunction *func);

SFPResult PingReceiveCallback(SFPFunction *func);

SFPResult PongReceiveCallback(SFPFunction *func);

SFPResult GetDeviceAddrCallback(SFPFunction *func);

SFPResult GetTrafficInfoCallback(SFPFunction *func);

SFPResult ClearTrafficInfoCallback(SFPFunction *func);

SFPResult SetRFParamsCallback(SFPFunction *func);

SFPResult GetRFParamsCallback(SFPFunction *func);

SFPResult CDCDefaultCallback(SFPFunction *func);

SFPResult SpiritDefaultCallback(SFPFunction *func);

SFPResult AddNodeCallback(SFPFunction *func);
SFPResult DelNodeCallback(SFPFunction *func);
SFPResult ClearNodesCallback(SFPFunction *func);
SFPResult GetNodesCallback(SFPFunction *func);
SFPResult GetNodeInfoCallback(SFPFunction *func);

SFPResult SetRouterCallback(SFPFunction *func);
SFPResult GetRouterCallback(SFPFunction *func);
SFPResult GetRouterInfoCallback(SFPFunction *func);

SFPResult SaveSettingsCallback(SFPFunction *func);
SFPResult EnterPowerSaveCallback(SFPFunction *func);

SFPResult SetSystemSettingsCallback(SFPFunction *func);
SFPResult GetSystemSettingsCallback(SFPFunction *func);

SFPResult lpc_system_getDeviceInfo(SFPFunction *msg);

SFPResult SleepCallback(SFPFunction *func);

#endif /* FUNCTION_LIST_H_ */
