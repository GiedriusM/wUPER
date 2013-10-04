/*
 * spirit1.h
 *
 *  Created on: 2013.08.20
 *      Author: Giedrius
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
