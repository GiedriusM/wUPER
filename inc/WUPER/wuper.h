/**
 * @file	wuper.h
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

#ifndef WUPER_H_
#define WUPER_H_

#include "main.h"

#define WUPER_PROTOCOL_VERSION		'0'
#define WUPER_MAX_NODE_COUNT		16

#define WUPER_PAYLOAD_OVERHEAD_SIZE		5

#define WUPER_FLAG_DATA		0
#define WUPER_FLAG_ACK		1
#define WUPER_FLAG_SETSEQ	2
#define WUPER_FLAG_SEQERR	3

typedef enum {
	WUPER_OK = 0,
	WUPER_ERR,
	WUPER_ERR_SFP,
	WUPER_ERR_SPIRIT
} WUPERResult;

typedef struct {
	uint32_t packetSendTotal;
	uint32_t packetSendSuccess;
	uint32_t packetSendRetry;

	uint32_t packetSeqSet;
} WUPERTrafficStatistics;

typedef struct {
	uint32_t frequency;
	uint32_t datarate;
	struct {
		enum { WUPER_MOD_FSK=0, WUPER_MOD_GFSK=1 } type:3;
		uint8_t BT1:1;
		uint8_t reserved:2;
		uint8_t whitening:1;
		uint8_t fec:1;
	} modulation;
	uint32_t freqDev;

	int8_t txPowerdBm;

	uint8_t networkID;

	uint8_t aesKey[16];

	uint32_t sendRetryCount;
	uint32_t ackWaitTimeout;
} WUPERSettings;


void WUPER_Init(SFPStream *stream, uint32_t guid[4]);
void WUPER_Restart(void);
void WUPER_Shutdown(void);

uint32_t 	WUPER_GetDeviceAddress(void);
void	 	WUPER_GetTrafficStatistics(WUPERTrafficStatistics *stats);
void		WUPER_ClearTrafficStatistics(void);

void 		WUPER_SetDestinationAddress(uint32_t addr);
void		WUPER_SetAESKey(uint8_t key[16]);
void 		WUPER_SetRFSettings(WUPERSettings *settings);
void 		WUPER_GetRFSettings(WUPERSettings *settings);

WUPERResult WUPER_AddNode(uint32_t addr);
WUPERResult WUPER_DeleteNode(uint32_t addr);
WUPERResult WUPER_ClearNodes(void);
uint8_t		WUPER_ContainsNode(uint32_t addr);
uint32_t	WUPER_GetNodeCount(void);
uint32_t	WUPER_GetNodeAddress(uint32_t pos);
void		WUPER_GetNodeInfo(uint32_t addr, uint16_t *inSignalQuality, uint16_t *outSignalQuality);

#endif /* WUPER_H_ */
