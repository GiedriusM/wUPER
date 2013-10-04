/**
 * @file	main.h
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

#ifndef MAIN_H_
#define MAIN_H_

#define WUPER_DEVICE_TYPE		'w'
#define WUPER_FW_VERSION_MAJOR	0
#define WUPER_FW_VERSION_MINOR	0
#define WUPER_FIRMWARE_VERSION		((WUPER_DEVICE_TYPE << 24) | (WUPER_FW_VERSION_MAJOR << 16) | WUPER_FW_VERSION_MINOR)


#include "stdlib.h"

#include "LPC11Uxx.h"

#include "lpc_def.h"

#include <SFP/SFP.h>

#include <MemoryManager/MemoryManager.h>

#include "time.h"

#include "WUPER/function_def.h"

#include "Modules/LPC_ADC.h"
#include "Modules/LPC_GPIO.h"
#include "Modules/LPC_I2C.h"
#include "Modules/LPC_PWM.h"
#include "Modules/LPC_SPI.h"

SFPStream stream;
SFPStream spirit_stream;

uint32_t GUID[4];

#endif /* MAIN_H_ */
