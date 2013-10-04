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
 * WUPER board function name and ID defintions
 *
 */

#ifndef FUNCTION_DEF_H_
#define FUNCTION_DEF_H_

/*
 * Functions IDs
 */
#define WUPER_CDC_FID_GETDEVADDRESS		1
#define WUPER_CDC_FID_GETTRAFFICINFO	7
#define WUPER_CDC_FID_SETNETWORK		2
#define WUPER_CDC_FID_SETAESKEY			3
#define WUPER_CDC_FID_SETRFPARAMS		4
#define WUPER_CDC_FID_PING				5
#define WUPER_CDC_FID_PONG				6

#define WUPER_CDC_FID_ADDNODE			10
#define WUPER_CDC_FID_DELNODE			11
#define WUPER_CDC_FID_CLRNODES			12
#define WUPER_CDC_FID_GETNODES			13

#define WUPER_CDC_FID_GETDEVINFO		255


#define WUPER_RF_FID_SETPRIMARY			1
#define WUPER_RF_FID_SETSECONDARY		2
#define WUPER_RF_FID_PINMODE			3
#define WUPER_RF_FID_DIGITALWRITE		4
#define WUPER_RF_FID_DIGITALREAD		5
#define WUPER_RF_FID_ATTACHINTERRUPT	6
#define WUPER_RF_FID_DETACHINTERRUPT	7
#define WUPER_RF_FID_INTERRUPT			8

#define WUPER_RF_FID_ANALOGREAD			10

#define WUPER_RF_FID_SPI0BEGIN			20
#define WUPER_RF_FID_SPI0TRANS			21
#define WUPER_RF_FID_SPI0END			22

#define WUPER_RF_FID_I2CBEGIN			40
#define WUPER_RF_FID_I2CTRANS			41
#define WUPER_RF_FID_I2CEND				42

#define WUPER_RF_FID_PWM0BEGIN			50
#define WUPER_RF_FID_PWM0SET			51
#define WUPER_RF_FID_PWM0END			52

#define WUPER_RF_FID_PWM1BEGIN			60
#define WUPER_RF_FID_PWM1SET			61
#define WUPER_RF_FID_PWM1END			62

#define WUPER_RF_FID_PING				100
#define WUPER_RF_FID_PONG				101

/*
 * Function names
 */
#define WUPER_CDC_FNAME_GETDEVADDRESS	"GetDeviceAddress"
#define WUPER_CDC_FNAME_GETTRAFFICINFO	"GetTrafficInfo"
#define WUPER_CDC_FNAME_SETNETWORK		"SetNetworkID"
#define WUPER_CDC_FNAME_SETAESKEY		"SetAESKey"
#define WUPER_CDC_FNAME_SETRFPARAMS		"SetRFSettings"
#define WUPER_CDC_FNAME_PING			"Ping"
#define WUPER_CDC_FNAME_PONG			"Pong"

#define WUPER_CDC_FNAME_ADDNODE			"AddNode"
#define WUPER_CDC_FNAME_DELNODE			"DeleteNode"
#define WUPER_CDC_FNAME_CLRNODES		"ClearNodes"
#define WUPER_CDC_FNAME_GETNODES		"GetNodes"

#define WUPER_CDC_FNAME_GETDEVINFO		"GetDeviceInfo"


#define WUPER_RF_FNAME_PING				WUPER_CDC_FNAME_PING
#define WUPER_RF_FNAME_PONG				WUPER_CDC_FNAME_PONG

#define WUPER_RF_FNAME_SETPRIMARY		"setPrimary"
#define WUPER_RF_FNAME_SETSECONDARY		"setSecondary"
#define WUPER_RF_FNAME_PINMODE			"pinMode"
#define WUPER_RF_FNAME_DIGITALWRITE		"digitalWrite"
#define WUPER_RF_FNAME_DIGITALREAD		"digitalRead"
#define WUPER_RF_FNAME_ATTACHINTERRUPT	"attachInterrupt"
#define WUPER_RF_FNAME_DETACHINTERRUPT	"detachInterrupt"
#define WUPER_RF_FNAME_INTERRUPT		"interrupt"

#define WUPER_RF_FNAME_ANALOGREAD		"analogRead"

#define WUPER_RF_FNAME_SPI0BEGIN		"spi0_begin"
#define WUPER_RF_FNAME_SPI0TRANS		"spi0_trans"
#define WUPER_RF_FNAME_SPI0END			"spi0_end"

#define WUPER_RF_FNAME_I2CBEGIN			"i2c_begin"
#define WUPER_RF_FNAME_I2CTRANS			"i2c_trans"
#define WUPER_RF_FNAME_I2CEND			"i2c_end"

#define WUPER_RF_FNAME_PWM0BEGIN		"pwm0_begin"
#define WUPER_RF_FNAME_PWM0SET			"pwm0_set"
#define WUPER_RF_FNAME_PWM0END			"pwm0_end"

#define WUPER_RF_FNAME_PWM1BEGIN		"pwm1_begin"
#define WUPER_RF_FNAME_PWM1SET			"pwm1_set"
#define WUPER_RF_FNAME_PWM1END			"pwm1_end"


#endif /* FUNCTION_DEF_H_ */
