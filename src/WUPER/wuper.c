/*
 * wuper.c
 *
 *  Created on: 2013.09.09
 *      Author: Giedrius
 */

#include "WUPER/wuper.h"


/*
 * wUPER engine variables
 */

volatile static enum {
	SPIRIT1_TX_INACTIVE,
	SPIRIT1_TX_SENDING,
	SPIRIT1_TX_WAITING_ACK,
	SPIRIT1_TX_RECEIVED_ACK
} WUPER_txState;

volatile static enum {
	SPIRIT1_AES_INACTIVE,
	SPIRIT1_AES_ENCRYPTING,
	SPIRIT1_AES_DECRYPTING_HEADER,
	SPIRIT1_AES_DECRYPTING_BODY,
	SPIRIT1_AES_ENCRYPTING_RESPONSE
} WUPER_AESState;

volatile static struct {
	uint32_t srcAddress;
	uint32_t dstAddress;
	uint8_t version;
	uint8_t flags;
	uint16_t sequenceID;
	uint16_t reserved;
	uint16_t dataLength;
} decryptedHeader;

static WUPERTrafficStatistics trafficStatistics;

typedef struct {
	uint32_t address;
	uint16_t sequenceID;
	enum {
		WUPER_NODE_STATUS_UNKNOWN = 0,
		WUPER_NODE_STATUS_CONNECTED,
		WUPER_NODE_STATUS_SEQERR,
		WUPER_NODE_STATUS_SETSEQ,
	} status;
} WUPER_Node;

static uint8_t WUPER_AESKey[16];
static uint32_t WUPER_deviceAddress;
static uint32_t WUPER_destinationAddress;


static WUPER_Node WUPER_nodes[WUPER_MAX_NODE_COUNT];
static uint8_t WUPER_nodeCount;

/* SFP buffer variables */
#define WUPER_SFP_RX_BUFFER_SIZE_N	7
#define WUPER_SFP_RX_BUFFER_MASK		((1 << WUPER_SFP_RX_BUFFER_SIZE_N) - 1)
static volatile uint8_t  WUPER_SFP_rxBuffer[1 << WUPER_SFP_RX_BUFFER_SIZE_N];

static volatile uint32_t WUPER_SFP_rxBufferReadPos;
static volatile uint32_t WUPER_SFP_rxBufferDataEndPos;

static volatile uint32_t WUPER_SFP_rxBufferDecryptPos;
static volatile uint32_t WUPER_SFP_rxBufferEncryptedEndPos;
static volatile uint32_t WUPER_SFP_rxBufferWritePos;

static volatile uint8_t WUPER_SFP_rxBufferDiscardNextPacket;

#define WUPER_SFP_rxBufferAvailable()  ((WUPER_SFP_rxBufferDataEndPos-WUPER_SFP_rxBufferReadPos) & WUPER_SFP_RX_BUFFER_MASK)
#define WUPER_SFP_rxBufferTmpFree()    ((WUPER_SFP_rxBufferReadPos-1-WUPER_SFP_rxBufferWritePos) & WUPER_SFP_RX_BUFFER_MASK)
#define WUPER_SFP_rxBufferEncryptedAvailable() ((WUPER_SFP_rxBufferEncryptedEndPos-WUPER_SFP_rxBufferDecryptPos) & WUPER_SFP_RX_BUFFER_MASK)

static void WUPER_rxBufferGetData(uint8_t *data, uint32_t pos, uint32_t len) {
	while (len--) {
		*data++ = WUPER_SFP_rxBuffer[pos++ & WUPER_SFP_RX_BUFFER_MASK];
	}
}

static void WUPER_rxBufferPutData(uint8_t *data, uint32_t pos, uint32_t len) {
	while (len--) {
		WUPER_SFP_rxBuffer[pos++ & WUPER_SFP_RX_BUFFER_MASK] = *data++;
	}
}

/*
 * Private functions
 */
#define WUPER_ADDRESS_MIX(a, b)		(a^b)
#define WUPER_ADDRESS_UNMIX(a, b)	(a^b)

void WUPER_SetResponsePacket(uint8_t data[16], uint32_t srcAddr, uint32_t dstAddr,
		uint8_t version, uint8_t flags, uint16_t sequenceID, uint16_t reserved, uint16_t length);

void SPIRIT1_SendResponse(uint8_t data[16]);

uint32_t WUPER_Stream_available(void);
uint32_t WUPER_Stream_read(uint8_t *buf, uint32_t len);
uint8_t  WUPER_Stream_readByte(void);
void	 WUPER_Stream_write(uint8_t *buf, uint32_t len);


/*
 * Interrupt handler
 */
void FLEX_INT7_IRQHandler() {
	SpiritIrqs interrupts = {0};
	SpiritIrqGetStatus(&interrupts);

	if (WUPER_SFP_rxBufferDecryptPos == WUPER_SFP_rxBufferEncryptedEndPos
			&& WUPER_SFP_rxBufferDecryptPos == WUPER_SFP_rxBufferWritePos) {
		WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferDataEndPos;
		WUPER_SFP_rxBufferEncryptedEndPos = WUPER_SFP_rxBufferDataEndPos;
		WUPER_SFP_rxBufferWritePos = WUPER_SFP_rxBufferDataEndPos;
	}

	if (interrupts.IRQ_RX_DATA_DISC) { // RX packet discarded (filter)
		WUPER_SFP_rxBufferWritePos = WUPER_SFP_rxBufferEncryptedEndPos;
		WUPER_SFP_rxBufferDiscardNextPacket = 0;
		SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO); // Clear RX FIFO
	} else {
		if (interrupts.IRQ_RX_FIFO_ERROR) { // RX FIFO overflow/underflow
			WUPER_SFP_rxBufferDiscardNextPacket = 1;
			SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO); // Clear RX FIFO
		}

		if (interrupts.IRQ_RX_FIFO_ALMOST_FULL) { // RX FIFO almost full
			uint8_t available = SpiritLinearFifoReadNumElementsRxFifo();

			if (available > WUPER_SFP_rxBufferTmpFree())
				available = WUPER_SFP_rxBufferTmpFree();

			if (available > 0) {
				SPIRIT1_SPISlaveEnable();
				SPIRIT1_SPITrans(0x01);	// Read
				SPIRIT1_SPITrans(0xFF); // RX FIFO
				while (available--) {
					WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferWritePos++ & WUPER_SFP_RX_BUFFER_MASK] = SPIRIT1_SPITrans(0xFF);
				}
				SPIRIT1_SPISlaveDisable();
			}
		}

		if (interrupts.IRQ_RX_DATA_READY) { // Packet received
			uint16_t payloadLength = SpiritPktBasicGetReceivedPktLength();
			uint8_t available = SpiritLinearFifoReadNumElementsRxFifo();


			if ((available > WUPER_SFP_rxBufferTmpFree())		// Not enough buffer space
					|| WUPER_SFP_rxBufferDiscardNextPacket	// marked to be discarded
					|| ((payloadLength & 0xF) != 0)	// payload is not a multiple of 16 (AES block size)
					|| (WUPER_AESState != SPIRIT1_AES_INACTIVE)) { // previous packet is being processed

				WUPER_SFP_rxBufferWritePos = WUPER_SFP_rxBufferEncryptedEndPos;
				WUPER_SFP_rxBufferDiscardNextPacket = 0;
				SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO); // Clear RX FIFO
			} else {
				if (available > 0) {
					SPIRIT1_SPISlaveEnable();
					SPIRIT1_SPITrans(0x01);	// Read
					SPIRIT1_SPITrans(0xFF); // RX FIFO
					while (available--) {
						WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferWritePos++ & WUPER_SFP_RX_BUFFER_MASK] = SPIRIT1_SPITrans(0xFF);
					}
					SPIRIT1_SPISlaveDisable();
				}

				WUPER_SFP_rxBufferEncryptedEndPos = WUPER_SFP_rxBufferWritePos;

				if (WUPER_AESState == SPIRIT1_AES_INACTIVE) {
					uint8_t tmpBuf[16];
					WUPER_AESState = SPIRIT1_AES_DECRYPTING_HEADER;
					WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
					SpiritAesWriteDataIn(tmpBuf, 16);
					SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
				}
			}
		}
	}

	// Handle decryption/encryption
	if (interrupts.IRQ_AES_END) {
		if (WUPER_AESState == SPIRIT1_AES_ENCRYPTING) { // Encrypted data for transmission (not ACK)
			WUPER_AESState = SPIRIT1_AES_INACTIVE;
		} else if (WUPER_AESState == SPIRIT1_AES_ENCRYPTING_RESPONSE) {
			uint8_t tmpBuf[16];
			SpiritAesReadDataOut(tmpBuf, 16);

			SPIRIT1_SendResponse(tmpBuf);

			if (WUPER_SFP_rxBufferEncryptedAvailable() > 0) {
				WUPER_AESState = SPIRIT1_AES_DECRYPTING_BODY;
				WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
				SpiritAesWriteDataIn(tmpBuf, 16);
				SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
			} else {
				WUPER_AESState = SPIRIT1_AES_INACTIVE;
			}
		} else if (WUPER_AESState == SPIRIT1_AES_DECRYPTING_HEADER) {
			uint8_t tmpBuf[16];
			SpiritAesReadDataOut(tmpBuf, 16);
			WUPER_SFP_rxBufferDecryptPos += 16;

			decryptedHeader.srcAddress	= (tmpBuf[0] << 24) | (tmpBuf[1] << 16) | (tmpBuf[2] << 8) | tmpBuf[3];
			decryptedHeader.dstAddress	= (tmpBuf[4] << 24) | (tmpBuf[5] << 16) | (tmpBuf[6] << 8) | tmpBuf[7];
			decryptedHeader.version 	= tmpBuf[8];
			decryptedHeader.flags 		= tmpBuf[9];
			decryptedHeader.sequenceID	= (tmpBuf[10] << 8) | tmpBuf[11];
			decryptedHeader.reserved	= (tmpBuf[12] << 8) | tmpBuf[13];
			decryptedHeader.dataLength	= (tmpBuf[14] << 8) | tmpBuf[15];

			WUPER_Node *node = NULL;
			uint8_t i;
			for (i=0; i<WUPER_nodeCount; i++) {
				if (WUPER_nodes[i].address == decryptedHeader.srcAddress) {
					node = &WUPER_nodes[i];
					break;
				}
			}

			if ((decryptedHeader.dstAddress == WUPER_deviceAddress)
					&& (decryptedHeader.version == WUPER_PROTOCOL_VERSION)
					&& (((decryptedHeader.dataLength+15) & 0xFFF0) == WUPER_SFP_rxBufferEncryptedAvailable())
					&& (node != NULL)) {

				if (decryptedHeader.flags == WUPER_FLAG_DATA) { // Data
					if (decryptedHeader.sequenceID == node->sequenceID) {
						node->status = WUPER_NODE_STATUS_CONNECTED;

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
											WUPER_PROTOCOL_VERSION, WUPER_FLAG_ACK, node->sequenceID, 0, 0);

						node->sequenceID++;
					} else {
						node->status = WUPER_NODE_STATUS_SEQERR;
						WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
											WUPER_PROTOCOL_VERSION, WUPER_FLAG_SEQERR,
											decryptedHeader.sequenceID, node->sequenceID, 0);
					}

					SpiritAesWriteDataIn(tmpBuf, 16);
					WUPER_AESState = SPIRIT1_AES_ENCRYPTING_RESPONSE;
					SpiritSpiCommandStrobes(COMMAND_AES_ENC);

				} else if (decryptedHeader.flags == WUPER_FLAG_ACK) { // ACK
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = SPIRIT1_AES_INACTIVE;

					if ((WUPER_txState == SPIRIT1_TX_WAITING_ACK)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {
						node->status = WUPER_NODE_STATUS_CONNECTED;
						WUPER_txState = SPIRIT1_TX_RECEIVED_ACK;
					}

				} else if (decryptedHeader.flags == WUPER_FLAG_SEQERR) {
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = SPIRIT1_AES_INACTIVE;

					if ((WUPER_txState == SPIRIT1_TX_WAITING_ACK)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {
						node->status = WUPER_NODE_STATUS_SETSEQ;
						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
										WUPER_PROTOCOL_VERSION, WUPER_FLAG_SETSEQ,
										decryptedHeader.reserved, WUPER_ADDRESS_MIX(decryptedHeader.reserved, node->sequenceID), 0);

						WUPER_AESState = SPIRIT1_AES_ENCRYPTING_RESPONSE;
						SpiritAesWriteDataIn(tmpBuf, 16);
						SpiritSpiCommandStrobes(COMMAND_AES_ENC);
					}

				} else if (decryptedHeader.flags == WUPER_FLAG_SETSEQ) {
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = SPIRIT1_AES_INACTIVE;

					if ((node->status == WUPER_NODE_STATUS_SEQERR)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
										WUPER_PROTOCOL_VERSION, WUPER_FLAG_SETSEQ,
										WUPER_ADDRESS_UNMIX(decryptedHeader.reserved, node->sequenceID), decryptedHeader.reserved, 0);

						node->status = WUPER_NODE_STATUS_CONNECTED;
						node->sequenceID = decryptedHeader.reserved;

						WUPER_AESState = SPIRIT1_AES_ENCRYPTING_RESPONSE;
						SpiritAesWriteDataIn(tmpBuf, 16);
						SpiritSpiCommandStrobes(COMMAND_AES_ENC);

						trafficStatistics.packetSeqSet++;
					} else if ((node->status == WUPER_NODE_STATUS_SETSEQ)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {

						node->status = WUPER_NODE_STATUS_CONNECTED;
						node->sequenceID = decryptedHeader.reserved;
						trafficStatistics.packetSeqSet++;
					} else {
						node->status = WUPER_NODE_STATUS_UNKNOWN;
					}

				} else {	// Unknown flag
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = SPIRIT1_AES_INACTIVE;
				}

			} else {
				/* This is assuming that only one encrypted packet is in the buffer.
				 * Most of the code written was designed to account for the possibility
				 * of multiple buffered (encrypted) packets, but in order to correctly
				 * skip an incorrect packet this feature was removed. Otherwise some
				 * kind of packet stack information would be needed. Although this
				 * multi packet buffering might seem a good thing it might cause trouble
				 * if packet parsing took too long and ACK packet would be sent after
				 * ACK timeout in the source device.
				 */
				WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
				WUPER_AESState = SPIRIT1_AES_INACTIVE;
			}

		} else if (WUPER_AESState == SPIRIT1_AES_DECRYPTING_BODY) {
			uint8_t tmpBuf[16];
			SpiritAesReadDataOut(tmpBuf, 16);
			WUPER_SFP_rxBufferDecryptPos += 16;

			uint8_t dataWritten = 16;
			if (dataWritten > decryptedHeader.dataLength)
				dataWritten = decryptedHeader.dataLength;

			WUPER_rxBufferPutData(tmpBuf, WUPER_SFP_rxBufferDataEndPos, dataWritten);
			WUPER_SFP_rxBufferDataEndPos += dataWritten;
			decryptedHeader.dataLength -= dataWritten;

			if (WUPER_SFP_rxBufferEncryptedAvailable() > 0) {
				WUPER_AESState = (decryptedHeader.dataLength == 0 ? SPIRIT1_AES_DECRYPTING_HEADER : SPIRIT1_AES_DECRYPTING_BODY);
				WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
				SpiritAesWriteDataIn(tmpBuf, 16);
				SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
			} else {
				WUPER_AESState = SPIRIT1_AES_INACTIVE;
			}
		}
	}

}

void WUPER_Init(SFPStream *stream, uint32_t guid[4]) {
	SPIRIT1_Init();

	WUPER_txState = SPIRIT1_TX_INACTIVE;
	WUPER_AESState = SPIRIT1_AES_INACTIVE;

	WUPER_deviceAddress = guid[0] ^ guid[1] ^ guid[2] ^ guid[3]; // XXX: generate from GUID

	uint32_t i;
	for (i=0; i<16; i++) {
		WUPER_AESKey[i] = i;
	}
	SpiritAesWriteKey(WUPER_AESKey);

	WUPER_SFP_rxBufferWritePos = 0;
	WUPER_SFP_rxBufferDataEndPos = 0;
	WUPER_SFP_rxBufferReadPos = 0;
	WUPER_SFP_rxBufferDecryptPos = 0;
	WUPER_SFP_rxBufferEncryptedEndPos = 0;
	WUPER_SFP_rxBufferDiscardNextPacket = 0;

	stream->available = WUPER_Stream_available;
	stream->read = WUPER_Stream_read;
	stream->readByte = WUPER_Stream_readByte;
	stream->write = WUPER_Stream_write;
}

uint32_t WUPER_GetDeviceAddress(void) {
	return WUPER_deviceAddress;
}

void WUPER_GetTrafficStatistics(WUPERTrafficStatistics *stats) {
	stats->packetSendTotal = trafficStatistics.packetSendTotal;
	stats->packetSendSuccess = trafficStatistics.packetSendSuccess;
	stats->packetSendRetry = trafficStatistics.packetSendRetry;
	stats->packetSeqSet = trafficStatistics.packetSeqSet;
}

void WUPER_SetDestinationAddress(uint32_t addr) {
	WUPER_destinationAddress = addr;
}

void WUPER_SetNetworkID(uint8_t id) {
	// wUPER network ID is really the address field of the Basic packet
	SpiritPktCommonSetDestinationAddress(id); // Set TX destination address
	SpiritSpiWriteRegisters(PCKT_FLT_GOALS_TX_ADDR_BASE, 1, &id); // Set RX destination address
}

void WUPER_SetAESKey(uint8_t key[16]) {
	SpiritAesWriteKey(key);
}

void WUPER_SetRFParameters(uint32_t frequency, uint32_t datarate, uint8_t modulation, uint32_t fdev) {
	SpiritSpiCommandStrobes(COMMAND_SABORT);

	SRadioInit spiritRadio;

	spiritRadio.lFrequencyBase = frequency;
	spiritRadio.nXtalOffsetPpm = 0;

	spiritRadio.lDatarate = datarate;

	if ((modulation & 0x7) == 1) { // GFSK modulation
		spiritRadio.xModulationSelect = (modulation & BIT3 ? GFSK_BT1 : GFSK_BT05);
	} else {
		spiritRadio.xModulationSelect = FSK;
	}

	spiritRadio.lFreqDev = fdev;

	spiritRadio.lBandwidth = 150000;

	spiritRadio.cChannelNumber = 0;
	spiritRadio.nChannelSpace = 300000;

	SpiritPktCommonWhitening(modulation & BIT6 ? S_ENABLE : S_DISABLE);
	SpiritPktCommonFec(modulation & BIT7 ? S_ENABLE : S_DISABLE);

	SpiritRadioInit(&spiritRadio);

	SpiritSpiCommandStrobes(COMMAND_RX);
}



/*
 * Stream functions
 */
uint32_t WUPER_Stream_available(void) {
	return WUPER_SFP_rxBufferAvailable();
}

uint32_t WUPER_Stream_read(uint8_t *buf, uint32_t len) {
	if (len > WUPER_SFP_rxBufferAvailable())
		len = WUPER_SFP_rxBufferAvailable();

	uint32_t i;
	for (i=0; i<len; i++)
		buf[i] = WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferReadPos++ & WUPER_SFP_RX_BUFFER_MASK];

	return len;
}

uint8_t  WUPER_Stream_readByte(void) {
	return WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferReadPos++ & WUPER_SFP_RX_BUFFER_MASK];
}

void SPIRIT1_Stream_writePacket(uint8_t *data, uint32_t len) {
	uint8_t tmpBuf[16];

	WUPER_Node *node = NULL;
	uint8_t i;
	for (i=0; i<WUPER_nodeCount; i++) {
		if (WUPER_nodes[i].address == WUPER_destinationAddress) {
			node = &WUPER_nodes[i];
			break;
		}
	}
	if (node == NULL) return;

	trafficStatistics.packetSendTotal++;

	WUPER_txState = SPIRIT1_TX_SENDING;

	uint8_t retriesLeft = WUPER_RETRY_COUNT;
	do {
		do {
			SpiritRefreshStatus();
			if (g_xStatus.MC_STATE == MC_STATE_RX)
				SpiritSpiCommandStrobes(COMMAND_SABORT); // SABORT
		} while (g_xStatus.MC_STATE != MC_STATE_READY);

		SpiritSpiCommandStrobes(COMMAND_FLUSHTXFIFO); // Clear TX FIFO

		/* Enter TX state */
		SpiritSpiCommandStrobes(COMMAND_LOCKTX); // Lock TX
		do {
			SpiritRefreshStatus();
		} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

		SpiritPktBasicSetPayloadLength((len+16+15) & 0xFFF0);

		// Construct header
		for (i=0; i<16; i++)
			tmpBuf[i] = 0;

		tmpBuf[0] = WUPER_deviceAddress >> 24;
		tmpBuf[1] = WUPER_deviceAddress >> 16;
		tmpBuf[2] = WUPER_deviceAddress >> 8;
		tmpBuf[3] = WUPER_deviceAddress;
		tmpBuf[4] = node->address >> 24;
		tmpBuf[5] = node->address >> 16;
		tmpBuf[6] = node->address >> 8;
		tmpBuf[7] = node->address;
		tmpBuf[8] = WUPER_PROTOCOL_VERSION;
		tmpBuf[9] = WUPER_FLAG_DATA;
		tmpBuf[10] = node->sequenceID >> 8;
		tmpBuf[11] = node->sequenceID;
		tmpBuf[14] = len >> 8;
		tmpBuf[15] = len;

		// Encrypt header
		WUPER_AESState = SPIRIT1_AES_ENCRYPTING;
		SpiritAesWriteDataIn(tmpBuf, 16);
		SpiritSpiCommandStrobes(COMMAND_AES_ENC);
		while (WUPER_AESState != SPIRIT1_AES_INACTIVE);
		SpiritAesReadDataOut(tmpBuf, 16);

		// Send header header
		SpiritSpiWriteLinearFifo(16, tmpBuf); // Write header to TX FIFO
		SpiritSpiCommandStrobes(COMMAND_TX); // TX

		// Send remaining data
		uint32_t remainingData = len;
		uint8_t *dataPtr = data;
		while (remainingData) {
			// Write data to 16 byte block
			for (i=0; i<16; i++) {
				if (remainingData) {
					tmpBuf[i] = *dataPtr++;
					remainingData--;
				} else {
					tmpBuf[i] = 0; // zero-fill padding
				}
			}

			// Encrypt block
			WUPER_AESState = SPIRIT1_AES_ENCRYPTING;
			SpiritAesWriteDataIn(tmpBuf, 16);
			SpiritSpiCommandStrobes(COMMAND_AES_ENC);
			while (WUPER_AESState != SPIRIT1_AES_INACTIVE);
			SpiritAesReadDataOut(tmpBuf, 16);

			// Wait until TX FIFO has enough space
			while (SpiritLinearFifoReadNumElementsTxFifo() > 80); // while (96-FIFO element count < 16)

			// Write block to FIFO
			SpiritSpiWriteLinearFifo(16, tmpBuf);
		}

		// Wait until data is sent
		do {
			SpiritRefreshStatus();
		} while (g_xStatus.MC_STATE != MC_STATE_READY);


		// Go to RX state
		SpiritSpiCommandStrobes(COMMAND_LOCKRX); // Lock RX
		do {
			SpiritRefreshStatus();
		} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

		SpiritSpiCommandStrobes(COMMAND_RX); // RX
		do {
			SpiritRefreshStatus();
		} while (g_xStatus.MC_STATE != MC_STATE_RX);


		// Wait for ACK with a timeout
		WUPER_txState = SPIRIT1_TX_WAITING_ACK;
		Time_setCountdown(5000);
		while (Time_isCountdownRunning()) {
			if (WUPER_txState == SPIRIT1_TX_RECEIVED_ACK) break;
		}

		if (WUPER_txState == SPIRIT1_TX_RECEIVED_ACK) {
			node->sequenceID++;
			trafficStatistics.packetSendSuccess++;
			break;
		}
		trafficStatistics.packetSendRetry++;
	} while (retriesLeft--);

	WUPER_txState = SPIRIT1_TX_INACTIVE;

	/*
	 * It is assumed that persistent RX is enabled
	 * and no additional state change is required.
	 */
}

void WUPER_Stream_write(uint8_t *buf, uint32_t len) {
	SPIRIT1_Stream_writePacket(buf, len);
}

void SPIRIT1_SendResponse(uint8_t data[16]) {

	WUPER_txState = SPIRIT1_TX_SENDING;

	do {
		SpiritRefreshStatus();
		if (g_xStatus.MC_STATE == MC_STATE_RX)
			SpiritSpiCommandStrobes(COMMAND_SABORT); // SABORT
	} while (g_xStatus.MC_STATE != MC_STATE_READY);

	SpiritSpiCommandStrobes(COMMAND_FLUSHTXFIFO); // Clear TX FIFO

	/* Enter TX state */
	SpiritSpiCommandStrobes(COMMAND_LOCKTX); // Lock TX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

	SpiritPktBasicSetPayloadLength(16);

	// Send header
	SpiritSpiWriteLinearFifo(16, data); // Write header to TX FIFO
	SpiritSpiCommandStrobes(COMMAND_TX); // TX

	// Wait until data is sent
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_READY);

	WUPER_txState = SPIRIT1_TX_INACTIVE;

	// Go to RX state
	SpiritSpiCommandStrobes(COMMAND_LOCKRX); // Lock RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

	SpiritSpiCommandStrobes(COMMAND_RX); // RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_RX);
}

void WUPER_SetResponsePacket(uint8_t buf[16], uint32_t srcAddr, uint32_t dstAddr,
		uint8_t version, uint8_t flags, uint16_t sequenceID, uint16_t reserved, uint16_t length) {

	buf[0] = srcAddr >> 24;
	buf[1] = srcAddr >> 16;
	buf[2] = srcAddr >> 8;
	buf[3] = srcAddr;
	buf[4] = dstAddr >> 24;
	buf[5] = dstAddr >> 16;
	buf[6] = dstAddr >> 8;
	buf[7] = dstAddr;
	buf[8] = version;
	buf[9] = flags;
	buf[10] = sequenceID >> 8;
	buf[11] = sequenceID;
	buf[12] = reserved >> 8;
	buf[13] = reserved;
	buf[14] = length >> 8;
	buf[15] = length;
}

/*
 * Node functions
 */
WUPERResult WUPER_AddNode(uint32_t addr) {
	if (WUPER_nodeCount >= WUPER_MAX_NODE_COUNT) return WUPER_ERR;

	if (WUPER_ContainsNode(addr)) return WUPER_ERR;

	WUPER_nodes[WUPER_nodeCount].address = addr;
	WUPER_nodes[WUPER_nodeCount].sequenceID = addr ^ Time_getSystemTime()*0x5FA209C7;
	WUPER_nodes[WUPER_nodeCount].status = WUPER_NODE_STATUS_UNKNOWN;
	WUPER_nodeCount++;

	return WUPER_OK;
}

WUPERResult WUPER_DeleteNode(uint32_t addr) {
	uint8_t foundNode = 0;
	uint8_t i;
	for (i=0; i<WUPER_nodeCount; i++) {
		if (foundNode) {
			WUPER_nodes[i-1].address = WUPER_nodes[i].address;
			WUPER_nodes[i-1].sequenceID = WUPER_nodes[i].sequenceID;
			WUPER_nodes[i-1].status = WUPER_nodes[i].status;
		} else if (WUPER_nodes[i].address == addr) {
			foundNode = 1;
		}
	}
	if (!foundNode) return WUPER_ERR;

	WUPER_nodeCount--;

	return WUPER_OK;
}

uint8_t WUPER_ContainsNode(uint32_t addr) {
	uint8_t i;
	for (i=0; i<WUPER_nodeCount; i++) {
		if (WUPER_nodes[i].address == addr)
			return 1;
	}

	return 0;
}

WUPERResult WUPER_ClearNodes(void) {
	WUPER_nodeCount = 0;
	return WUPER_OK;
}

uint32_t	WUPER_GetNodeCount(void) {
	return WUPER_nodeCount;
}

uint32_t	WUPER_GetNodeAddress(uint32_t pos) {
	if (pos < WUPER_nodeCount) return WUPER_nodes[pos].address;

	return -1;
}
