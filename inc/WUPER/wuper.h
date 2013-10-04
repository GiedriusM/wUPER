/*
 * wUPER.h
 *
 *  Created on: 2013.09.09
 *      Author: Giedrius
 */

#ifndef WUPER_H_
#define WUPER_H_

#include "main.h"
#include "SPIRIT1/spirit1.h"



#define WUPER_PROTOCOL_VERSION		'0'
#define WUPER_MAX_NODE_COUNT		16
#define WUPER_RETRY_COUNT			1

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


void WUPER_Init(SFPStream *stream, uint32_t guid[4]);

uint32_t 	WUPER_GetDeviceAddress(void);
void	 	WUPER_GetTrafficStatistics(WUPERTrafficStatistics *stats);

void 		WUPER_SetDestinationAddress(uint32_t addr);
void		WUPER_SetNetworkID(uint8_t id);
void		WUPER_SetAESKey(uint8_t key[16]);
void		WUPER_SetRFParameters(uint32_t frequency, uint32_t datarate, uint8_t modulation, uint32_t fdev);

WUPERResult WUPER_AddNode(uint32_t addr);
WUPERResult WUPER_DeleteNode(uint32_t addr);
WUPERResult WUPER_ClearNodes(void);
uint8_t		WUPER_ContainsNode(uint32_t addr);
uint32_t	WUPER_GetNodeCount(void);
uint32_t	WUPER_GetNodeAddress(uint32_t pos);

#endif /* WUPER_H_ */
