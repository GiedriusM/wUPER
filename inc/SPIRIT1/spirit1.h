/**
 * @file	spirit1.h
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

#ifndef SPIRIT1_H_
#define SPIRIT1_H_

#include "main.h"

#include "SPIRIT_Config.h"

#define XTAL_FREQUENCY	52000000
#define CLOCK_FREQUENCY	26000000

void SPIRIT1_Init(void);

void SPIRIT1_SPIConfig(uint32_t divider, uint8_t mode);
uint8_t SPIRIT1_SPITrans(uint8_t data);
void SPIRIT1_SPISlaveEnable(void);
void SPIRIT1_SPISlaveDisable(void);

void SPIRIT1_CalculateDatarateME(uint32_t datarate, uint8_t* dr_m, uint8_t* dr_e);

#endif /* SPIRIT1_H_ */
