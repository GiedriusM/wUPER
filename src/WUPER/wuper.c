/**
 * @file	wuper.c
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

#include "WUPER/wuper.h"

#include "SPIRIT_Config.h"

#define XTAL_FREQUENCY	52000000
#define CLOCK_FREQUENCY	26000000

/*
 * wUPER engine variables
 */

volatile static enum {
	WUPER_TX_INACTIVE,
	WUPER_TX_SENDING,
	WUPER_TX_WAITING_ACK,
	WUPER_TX_RECEIVED_ACK
} WUPER_txState;

volatile static enum {
	WUPER_AES_INACTIVE=0,
	WUPER_AES_ENCRYPTING=1,
	WUPER_AES_DECRYPTING_HEADER=2,
	WUPER_AES_DECRYPTING_BODY=3,
	WUPER_AES_ENCRYPTING_RESPONSE=4
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
	uint16_t inSignalIndicator;
	uint16_t outSignalIndicator;
} WUPER_Node;

static WUPER_Node WUPER_nodes[WUPER_MAX_NODE_COUNT];
static uint8_t WUPER_nodeCount;


/* Spirit config variables */
static WUPERSettings WUPER_settings;
static uint32_t WUPER_deviceAddress;
static uint32_t WUPER_destinationAddress;
static volatile uint8_t WUPER_csmaBackOffFlag;


/* SFP buffer variables */
#define WUPER_SFP_RX_BUFFER_SIZE_N	8
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

static void WUPER_InitPins(void);

static void WUPER_InitSPI(void);
static uint8_t WUPER_SPITrans(uint8_t data);
static void WUPER_SPISlaveEnable(void);
static void WUPER_SPISlaveDisable(void);

uint32_t WUPER_Stream_available(void);
uint32_t WUPER_Stream_read(uint8_t *buf, uint32_t len);
uint8_t  WUPER_Stream_readByte(void);
void	 WUPER_Stream_write(uint8_t *buf, uint32_t len);

void WUPER_SendResponse(uint8_t data[16]);
void WUPER_SetResponsePacket(uint8_t data[16], uint32_t srcAddr, uint32_t dstAddr,
		uint8_t version, uint8_t flags, uint16_t sequenceID, uint16_t reserved, uint16_t length);


/*
 * Interrupt handler
 */
volatile SpiritIrqs tmpIrqs[32];
volatile uint8_t  tmpStates[32];
volatile uint32_t tmpCount  = 0;
void FLEX_INT7_IRQHandler() {
	NVIC_DisableIRQ(FLEX_INT7_IRQn);

	SpiritIrqs interrupts = {0};
	SpiritIrqGetStatus(&interrupts);

	if (*((uint32_t*)&interrupts) != 0) {
		tmpIrqs[tmpCount] = interrupts;
		tmpStates[tmpCount] = WUPER_AESState;
		tmpCount = (tmpCount+1) & (32-1);
	}

	if (WUPER_SFP_rxBufferDecryptPos == WUPER_SFP_rxBufferEncryptedEndPos
			&& WUPER_SFP_rxBufferDecryptPos == WUPER_SFP_rxBufferWritePos) {
		WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferDataEndPos;
		WUPER_SFP_rxBufferEncryptedEndPos = WUPER_SFP_rxBufferDataEndPos;
		WUPER_SFP_rxBufferWritePos = WUPER_SFP_rxBufferDataEndPos;
	}

	if (interrupts.IRQ_MAX_BO_CCA_REACH) {
		WUPER_csmaBackOffFlag = 1;
	}

	if (interrupts.IRQ_RX_DATA_DISC // RX packet discarded (filter)
			|| (interrupts.IRQ_RX_DATA_READY && WUPER_txState == WUPER_TX_SENDING)) { // CSMA state
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
				WUPER_SPISlaveEnable();
				WUPER_SPITrans(0x01);	// Read
				WUPER_SPITrans(0xFF); // RX FIFO
				while (available--) {
					WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferWritePos++ & WUPER_SFP_RX_BUFFER_MASK] = WUPER_SPITrans(0xFF);
				}
				WUPER_SPISlaveDisable();
			}
		}

		if (interrupts.IRQ_RX_DATA_READY) { // Packet received
			uint16_t payloadLength = SpiritPktBasicGetReceivedPktLength();
			uint8_t available = SpiritLinearFifoReadNumElementsRxFifo();


			if ((available > WUPER_SFP_rxBufferTmpFree())		// Not enough buffer space
					|| WUPER_SFP_rxBufferDiscardNextPacket	// marked to be discarded
					|| ((payloadLength & 0xF) != 0)	// payload is not a multiple of 16 (AES block size)
					|| (WUPER_AESState != WUPER_AES_INACTIVE)) { // previous packet is being processed

				WUPER_SFP_rxBufferWritePos = WUPER_SFP_rxBufferEncryptedEndPos;
				WUPER_SFP_rxBufferDiscardNextPacket = 0;
				SpiritSpiCommandStrobes(COMMAND_FLUSHRXFIFO); // Clear RX FIFO
			} else {
				if (available > 0) {
					WUPER_SPISlaveEnable();
					WUPER_SPITrans(0x01);	// Read
					WUPER_SPITrans(0xFF); // RX FIFO
					while (available--) {
						WUPER_SFP_rxBuffer[WUPER_SFP_rxBufferWritePos++ & WUPER_SFP_RX_BUFFER_MASK] = WUPER_SPITrans(0xFF);
					}
					WUPER_SPISlaveDisable();
				}

				WUPER_SFP_rxBufferEncryptedEndPos = WUPER_SFP_rxBufferWritePos;

				if (WUPER_AESState == WUPER_AES_INACTIVE) {
					uint8_t tmpBuf[16];
					WUPER_AESState = WUPER_AES_DECRYPTING_HEADER;
					WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
					SpiritAesWriteDataIn(tmpBuf, 16);
					SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
				}
			}
		}
	}

	// Handle decryption/encryption
	if (interrupts.IRQ_AES_END) {
		if (WUPER_AESState == WUPER_AES_ENCRYPTING) { // Encrypted data for transmission (not ACK)
			WUPER_AESState = WUPER_AES_INACTIVE;
		} else if (WUPER_AESState == WUPER_AES_ENCRYPTING_RESPONSE) {
			uint8_t tmpBuf[16];
			SpiritAesReadDataOut(tmpBuf, 16);

			WUPER_SendResponse(tmpBuf);

			if (WUPER_SFP_rxBufferEncryptedAvailable() > 0) {
				WUPER_AESState = WUPER_AES_DECRYPTING_BODY;
				WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
				SpiritAesWriteDataIn(tmpBuf, 16);
				SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
			} else {
				WUPER_AESState = WUPER_AES_INACTIVE;
			}
		} else if (WUPER_AESState == WUPER_AES_DECRYPTING_HEADER) {
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

						node->inSignalIndicator = SpiritQiGetRssi();

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
											WUPER_PROTOCOL_VERSION, WUPER_FLAG_ACK, node->sequenceID, node->inSignalIndicator, 0);

						node->sequenceID++;
					} else {
						node->status = WUPER_NODE_STATUS_SEQERR;
						WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
											WUPER_PROTOCOL_VERSION, WUPER_FLAG_SEQERR,
											decryptedHeader.sequenceID, node->sequenceID, 0);
					}

					SpiritAesWriteDataIn(tmpBuf, 16);
					WUPER_AESState = WUPER_AES_ENCRYPTING_RESPONSE;
					SpiritSpiCommandStrobes(COMMAND_AES_ENC);

				} else if (decryptedHeader.flags == WUPER_FLAG_ACK) { // ACK
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = WUPER_AES_INACTIVE;

					if ((WUPER_txState == WUPER_TX_WAITING_ACK)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {
						node->status = WUPER_NODE_STATUS_CONNECTED;
						node->outSignalIndicator = decryptedHeader.reserved;
						WUPER_txState = WUPER_TX_RECEIVED_ACK;
					}

				} else if (decryptedHeader.flags == WUPER_FLAG_SEQERR) {
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = WUPER_AES_INACTIVE;

					if ((WUPER_txState == WUPER_TX_WAITING_ACK)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {
						node->status = WUPER_NODE_STATUS_SETSEQ;
						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
										WUPER_PROTOCOL_VERSION, WUPER_FLAG_SETSEQ,
										decryptedHeader.reserved, WUPER_ADDRESS_MIX(decryptedHeader.reserved, node->sequenceID), 0);

						WUPER_AESState = WUPER_AES_ENCRYPTING_RESPONSE;
						SpiritAesWriteDataIn(tmpBuf, 16);
						SpiritSpiCommandStrobes(COMMAND_AES_ENC);
					}

				} else if (decryptedHeader.flags == WUPER_FLAG_SETSEQ) {
					WUPER_SFP_rxBufferDecryptPos = WUPER_SFP_rxBufferEncryptedEndPos;
					WUPER_AESState = WUPER_AES_INACTIVE;

					if ((node->status == WUPER_NODE_STATUS_SEQERR)
							&& (decryptedHeader.sequenceID == node->sequenceID)) {

						WUPER_SetResponsePacket(tmpBuf, WUPER_deviceAddress, decryptedHeader.srcAddress,
										WUPER_PROTOCOL_VERSION, WUPER_FLAG_SETSEQ,
										WUPER_ADDRESS_UNMIX(decryptedHeader.reserved, node->sequenceID), decryptedHeader.reserved, 0);

						node->status = WUPER_NODE_STATUS_CONNECTED;
						node->sequenceID = decryptedHeader.reserved;

						WUPER_AESState = WUPER_AES_ENCRYPTING_RESPONSE;
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
					WUPER_AESState = WUPER_AES_INACTIVE;
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
				WUPER_AESState = WUPER_AES_INACTIVE;
			}

		} else if (WUPER_AESState == WUPER_AES_DECRYPTING_BODY) {
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
				WUPER_AESState = (decryptedHeader.dataLength == 0 ? WUPER_AES_DECRYPTING_HEADER : WUPER_AES_DECRYPTING_BODY);
				WUPER_rxBufferGetData(tmpBuf, WUPER_SFP_rxBufferDecryptPos, 16);
				SpiritAesWriteDataIn(tmpBuf, 16);
				SpiritSpiCommandStrobes(COMMAND_AES_KEY_DEC);
			} else {
				WUPER_AESState = WUPER_AES_INACTIVE;
			}
		}
	}

	NVIC_EnableIRQ(FLEX_INT7_IRQn);
}

void WUPER_Init(SFPStream *stream, uint32_t guid[4]) {
	WUPER_txState = WUPER_TX_INACTIVE;
	WUPER_AESState = WUPER_AES_INACTIVE;

	WUPER_deviceAddress = guid[0] ^ guid[1] ^ guid[2] ^ guid[3]; // XXX: generate from GUID

	WUPER_SFP_rxBufferWritePos = 0;
	WUPER_SFP_rxBufferDataEndPos = 0;
	WUPER_SFP_rxBufferReadPos = 0;
	WUPER_SFP_rxBufferDecryptPos = 0;
	WUPER_SFP_rxBufferEncryptedEndPos = 0;
	WUPER_SFP_rxBufferDiscardNextPacket = 0;

	WUPER_settings.frequency = 868000000;
	WUPER_settings.datarate = 500;
	WUPER_settings.modulation.type = WUPER_MOD_FSK;
	WUPER_settings.modulation.BT1 = 0;
	WUPER_settings.modulation.whitening = 0;
	WUPER_settings.modulation.fec = 0;
	WUPER_settings.freqDev = 15000;
	WUPER_settings.txPowerdBm = 11;
	WUPER_settings.networkID = 0;
	WUPER_settings.sendRetryCount = 2;
	WUPER_settings.ackWaitTimeout = 5000;

	WUPER_InitPins();

	WUPER_InitSPI();

	WUPER_Shutdown();

	WUPER_Restart();

	stream->available = WUPER_Stream_available;
	stream->read = WUPER_Stream_read;
	stream->readByte = WUPER_Stream_readByte;
	stream->write = WUPER_Stream_write;
}

void WUPER_Restart(void) {
	/* Turn shutdown OFF */
	LPC_GPIO->CLR[1] = (1 << 29);

	/* Wait for device to boot up */
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_READY);

	/* Send configurations*/
	SpiritRadioSetXtalFrequency(XTAL_FREQUENCY);
	SpiritGeneralSetSpiritVersion(SPIRIT_VERSION_3_0);

	WUPER_SetRFSettings(&WUPER_settings);

	SpiritPktCommonSetTransmittedCtrlField(('0' << 24) | ('P' << 16) | ('F' << 8) | 'S');
	SpiritPktCommonSetCtrlReference(('0' << 24) | ('P' << 16) | ('F' << 8) | 'S');
	SpiritPktCommonSetCtrlMask(0xFFFFFFFF);
	SpiritPktCommonFilterOnControlField(S_ENABLE);

	SpiritTimerSetRxTimeoutStopCondition(NO_TIMEOUT_STOP);
	SpiritRadioPersistenRx(S_ENABLE);

	SpiritAesMode(S_ENABLE);
	SpiritAesWriteKey(WUPER_settings.aesKey);

	CsmaInit csmaInit = {
		.xCsmaPersistentMode = S_DISABLE,
		.xMultiplierTbit = TBIT_TIME_128,
		.xCcaLength = TCCA_TIME_4,

		.cMaxNb = 7,
		.cBuPrescaler = 35, // 34,7kHz/35 -> BU about 1ms
	};
	SpiritCsmaInit(&csmaInit);
	SpiritCsmaSeedReloadMode(S_DISABLE);
	SpiritQiSetRssiThreshold(150);
	SpiritQiSetCsMode(CS_MODE_STATIC_3DB);

	SpiritIrqs irqInit = {0};
	irqInit.IRQ_RX_DATA_READY = S_SET;
	irqInit.IRQ_RX_DATA_DISC = S_SET;
	irqInit.IRQ_RX_FIFO_ERROR = S_SET;
	irqInit.IRQ_RX_FIFO_ALMOST_FULL = S_SET;
	irqInit.IRQ_AES_END = S_SET;
	irqInit.IRQ_MAX_BO_CCA_REACH = S_SET;

	SpiritIrqInit(&irqInit);
	SpiritIrqGetStatus(&irqInit); // Reset IRQ flags

	SGpioInit gpioInit = {
			SPIRIT_GPIO_0, // TODO: change this to other GPIO leg (use GPIO0 for temperature)
			SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
			SPIRIT_GPIO_DIG_OUT_IRQ
	};
	SpiritGpioInit(&gpioInit);


	/* Enable IRQ */
	LPC_SYSCON->PINTSEL[7] = 0+11;

	LPC_GPIO_PIN_INT->ISEL |= (1 << 7);	// Set PMODE=level sensitive
	LPC_GPIO_PIN_INT->IENR |= (1 << 7);	// Enable level interrupt.
	LPC_GPIO_PIN_INT->IENF &= ~(1 << 7);// Set active level LOW.

	LPC_GPIO_PIN_INT->RISE = (1 << 7);	// Clear rising edge (sort of) flag
	LPC_GPIO_PIN_INT->FALL = (1 << 7);	// Clear falling edge (sort of) flag
	NVIC_SetPriority(7, 2);
	NVIC_EnableIRQ(FLEX_INT7_IRQn);

	/* Enter RX state */
	SpiritSpiCommandStrobes(COMMAND_LOCKRX); // Lock RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_LOCK);

	SpiritSpiCommandStrobes(COMMAND_RX); // RX
	do {
		SpiritRefreshStatus();
	} while (g_xStatus.MC_STATE != MC_STATE_RX);
}

void WUPER_Shutdown(void) {
	/* Disable IRQ */
	NVIC_DisableIRQ(FLEX_INT7_IRQn);
	LPC_SYSCON->PINTSEL[7] = 0;

	LPC_GPIO_PIN_INT->ISEL &= ~(1 << 7); // Reset to default
	LPC_GPIO_PIN_INT->IENR &= ~(1 << 7); // Reset to default
	LPC_GPIO_PIN_INT->IENF &= ~(1 << 7); // Reset to default

	LPC_GPIO_PIN_INT->RISE = (1 << 7);	// Clear interrupt flag
	LPC_GPIO_PIN_INT->FALL = (1 << 7);	// Clear interrupt flag

	/* Turn shutdown ON */
	LPC_GPIO->SET[1] = (1 << 29);
}

/*
 * Pin and SPI functions
 */
#define LPC_PIN_FUNCTION_MASK	(BIT7 | 7)		// FUNC bits + AD bit
#define LPC_PIN_MODE_MASK		(3 << 3)

static void WUPER_InitPins(void) {
	/* GPIO config */
	// Shutdown
	LPC_IOCON->PIO1_29 &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors
	LPC_GPIO->DIR[1] |= (1 << 29);				// Set output
	LPC_GPIO->SET[1] = (1 << 29);				// Turn Shutdown ON
	// GPIO0
	LPC_IOCON->TDI_PIO0_11 = (LPC_IOCON->TDI_PIO0_11 & ~LPC_PIN_MODE_MASK) | 0x01 | (0x2 << 3);
	LPC_GPIO->DIR[0] &= ~(1 << 11);				// Set input
	// GPIO1
	LPC_IOCON->TMS_PIO0_12 = (LPC_IOCON->TMS_PIO0_12 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 12);				// Set input
	// GPIO2
	LPC_IOCON->TDO_PIO0_13 = (LPC_IOCON->TDO_PIO0_13 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 13);				// Set input
	// GPIO3
	LPC_IOCON->TRST_PIO0_14 = (LPC_IOCON->TRST_PIO0_14 & ~LPC_PIN_MODE_MASK) | 0x01;
	LPC_GPIO->DIR[0] &= ~(1 << 14);				// Set input
	// SSEL
	LPC_IOCON->PIO0_20 &= ~LPC_PIN_MODE_MASK;	// Remove pull-up/down resistors
	LPC_GPIO->DIR[0] |= (1 << 20);				// Set output
	LPC_GPIO->SET[0] = (1 << 20);				// SSEL high

	LPC_IOCON->PIO1_21 = (LPC_IOCON->PIO1_21 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // MISO0
	LPC_IOCON->PIO0_21 = (LPC_IOCON->PIO0_21 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // MOSI0
	LPC_IOCON->PIO1_20 = (LPC_IOCON->PIO1_20 & ~LPC_PIN_FUNCTION_MASK) | 0x02; // SCK0
}

static void WUPER_InitSPI(void) {
	LPC_SYSCON->PRESETCTRL |= BIT2; 	// de-assert SPI1
	LPC_SYSCON->SYSAHBCLKCTRL |= BIT18;	// enable SPI1 clock
	LPC_SYSCON->SSP1CLKDIV = 1; //48MHz

	LPC_SSP1->CR1 = 0;		// Master mode, SPI disabled
	LPC_SSP1->CPSR = 24;	// 48MHz/24 = 2MHz
	LPC_SSP1->CR0 = (8-1) | (0 << 4) | (0 << 6) | ((2-1) << 8); // 8bits, SPI mode 0, divider=2
	LPC_SSP1->IMSC = 0;		// Interrupts disabled
	LPC_SSP1->CR1 = BIT1;	// Master mode, SPI enabled

	while (LPC_SSP1->SR & BIT4);	// wait while BUSY (reading or writing)

	while (LPC_SSP1->SR & BIT2) {	// Read while Rx FIFO not empty
		LPC_SSP1->DR;
	}
}

static uint8_t WUPER_SPITrans(uint8_t data) {
	LPC_SSP1->DR = data;
	while (!(LPC_SSP1->SR & BIT2));
	return LPC_SSP1->DR;
}

static void WUPER_SPISlaveEnable(void) {
	NVIC_DisableIRQ(FLEX_INT7_IRQn);
	LPC_GPIO->CLR[0] = (1 << 20);	// SSEL low

	// CSs to SCLK delay (should be 2us, but less still works)
	volatile uint32_t i;
	for(i=0;i<100;i++);
}

static void WUPER_SPISlaveDisable(void) {
	LPC_GPIO->SET[0] = (1 << 20);	// SSEL high
	NVIC_EnableIRQ(FLEX_INT7_IRQn);
}

StatusBytes SdkEvalSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	WUPER_SPISlaveEnable();

	uint16_t status = (WUPER_SPITrans(0x00) << 8) | WUPER_SPITrans(cRegAddress);

	uint8_t i;
	for (i=0; i<cNbBytes; i++) {
		WUPER_SPITrans(*pcBuffer++);
	}

	WUPER_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer) {
	WUPER_SPISlaveEnable();

	uint16_t status = (WUPER_SPITrans(0x01) << 8) | WUPER_SPITrans(cRegAddress);

	uint8_t i;
	for (i=0; i<cNbBytes; i++)
		*pcBuffer++ = WUPER_SPITrans(0xFF);

	WUPER_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiCommandStrobes(uint8_t cCommandCode) {
	WUPER_SPISlaveEnable();
	uint16_t status = (WUPER_SPITrans(0x80) << 8) | WUPER_SPITrans(cCommandCode);
	WUPER_SPISlaveDisable();

	return *((StatusBytes*)&status);
}

StatusBytes SdkEvalSpiWriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	return SdkEvalSpiWriteRegisters(0xFF, cNbBytes, pcBuffer);
}

StatusBytes SdkEvalSpiReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer) {
	return SdkEvalSpiReadRegisters(0xFF, cNbBytes, pcBuffer);
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

void WUPER_Stream_writePacket(uint8_t *data, uint32_t len) {
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

	WUPER_txState = WUPER_TX_SENDING;

	uint8_t retriesLeft = WUPER_settings.sendRetryCount;
	do {
		do {
			SpiritRefreshStatus();
			if (g_xStatus.MC_STATE == MC_STATE_RX)
				SpiritSpiCommandStrobes(COMMAND_SABORT); // SABORT
		} while (g_xStatus.MC_STATE != MC_STATE_READY);

		SpiritSpiCommandStrobes(COMMAND_FLUSHTXFIFO); // Clear TX FIFO

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
		WUPER_AESState = WUPER_AES_ENCRYPTING;
		SpiritAesWriteDataIn(tmpBuf, 16);
		SpiritSpiCommandStrobes(COMMAND_AES_ENC);
		while (WUPER_AESState != WUPER_AES_INACTIVE); // Wait for data to be encrypted
		SpiritAesReadDataOut(tmpBuf, 16);


		// Enable CSMA
		WUPER_csmaBackOffFlag = 0;
		SpiritCsma(S_ENABLE);

		// Send header
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
			WUPER_AESState = WUPER_AES_ENCRYPTING;
			SpiritAesWriteDataIn(tmpBuf, 16);
			SpiritSpiCommandStrobes(COMMAND_AES_ENC);
			while (WUPER_AESState != WUPER_AES_INACTIVE); // Wait for the encryption to end
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
		SpiritCsma(S_DISABLE);


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
		WUPER_txState = WUPER_TX_WAITING_ACK;
		Time_setCountdown(WUPER_settings.ackWaitTimeout);
		while (Time_isCountdownRunning()) {
			if (WUPER_txState == WUPER_TX_RECEIVED_ACK) break;
		}

		if (WUPER_txState == WUPER_TX_RECEIVED_ACK) {
			node->sequenceID++;
			trafficStatistics.packetSendSuccess++;
			break;
		}
		trafficStatistics.packetSendRetry++;
	} while (retriesLeft--);

	WUPER_txState = WUPER_TX_INACTIVE;

	/*
	 * It is assumed that persistent RX is enabled
	 * and no additional state change is required.
	 */
}

void WUPER_Stream_write(uint8_t *buf, uint32_t len) {
	WUPER_Stream_writePacket(buf, len);
}

void WUPER_SendResponse(uint8_t data[16]) {

	if (WUPER_txState == WUPER_TX_SENDING) { // Just in case some bug slipped through
		return;
	}

	uint32_t tmpState = WUPER_txState;

	WUPER_txState = WUPER_TX_SENDING;

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

	WUPER_txState = tmpState;

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

uint32_t WUPER_GetNodeCount(void) {
	return WUPER_nodeCount;
}

uint32_t WUPER_GetNodeAddress(uint32_t pos) {
	if (pos < WUPER_nodeCount) return WUPER_nodes[pos].address;

	return -1;
}

void WUPER_GetNodeInfo(uint32_t addr, uint16_t *inSignalQuality, uint16_t *outSignalQuality) {
	*inSignalQuality = -1;
	*outSignalQuality = -1;

	uint8_t i;
	for (i=0; i<WUPER_nodeCount; i++) {
		if (WUPER_nodes[i].address == addr) {
			*inSignalQuality = WUPER_nodes[i].inSignalIndicator;
			*outSignalQuality = WUPER_nodes[i].outSignalIndicator;
		}
	}
}

/*
 * WUPER interface functions
 */
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

void WUPER_SetAESKey(uint8_t key[16]) {
	uint8_t i;
	for (i=0; i<16; i++)
		WUPER_settings.aesKey[i] = key[i];

	SpiritAesWriteKey(WUPER_settings.aesKey);
}

void WUPER_SetRFSettings(WUPERSettings *settings) {
	WUPER_settings = *settings;

	SpiritRefreshStatus();
	SpiritStatus spiritStatus = g_xStatus;
	if (spiritStatus.MC_STATE != MC_STATE_READY) {
		SpiritSpiCommandStrobes(COMMAND_SABORT);
	}

	SRadioInit spiritRadio;
	PktBasicInit spiritBasicPacketInit;
	PktBasicAddressesInit spiritBasicPacketAddresses;

	spiritRadio.lFrequencyBase = WUPER_settings.frequency;
	spiritRadio.lDatarate = WUPER_settings.datarate;
	if (WUPER_settings.modulation.type == WUPER_MOD_GFSK) { // GFSK modulation
		spiritRadio.xModulationSelect = (WUPER_settings.modulation.BT1 ? GFSK_BT1 : GFSK_BT05);
	} else {
		spiritRadio.xModulationSelect = FSK;
	}
	spiritRadio.lFreqDev = WUPER_settings.freqDev;
	spiritRadio.lBandwidth = 150000;
	spiritRadio.cChannelNumber = 0;
	spiritRadio.nChannelSpace = 300000;
	spiritRadio.nXtalOffsetPpm = 0; // TODO: calculate xtal offset

	spiritBasicPacketInit.xPreambleLength = PKT_PREAMBLE_LENGTH_04BYTES;
	spiritBasicPacketInit.xSyncLength = PKT_SYNC_LENGTH_4BYTES;
	spiritBasicPacketInit.lSyncWords = ('C' << 24) | (('N' | 0x80) << 16) | ('Y' << 8) | ('S' | 0x80);
	spiritBasicPacketInit.xFixVarLength = PKT_LENGTH_VAR;
	spiritBasicPacketInit.cPktLengthWidth = 8;
	spiritBasicPacketInit.xCrcMode = PKT_CRC_MODE_16BITS_1;
	spiritBasicPacketInit.xControlLength = PKT_CONTROL_LENGTH_4BYTES;
	spiritBasicPacketInit.xAddressField = S_ENABLE;
	spiritBasicPacketInit.xFec = (WUPER_settings.modulation.fec ? S_ENABLE : S_DISABLE);
	spiritBasicPacketInit.xDataWhitening = (WUPER_settings.modulation.whitening ? S_ENABLE : S_DISABLE);

	// XXX: BUG! Device and broadcast address filter masks
	// are interchanged (or their addresses)
	spiritBasicPacketAddresses.xFilterOnMyAddress = S_DISABLE;
	spiritBasicPacketAddresses.cMyAddress = WUPER_settings.networkID;
	spiritBasicPacketAddresses.xFilterOnMulticastAddress = S_DISABLE;
	spiritBasicPacketAddresses.cMulticastAddress = 0;
	spiritBasicPacketAddresses.xFilterOnBroadcastAddress = S_ENABLE;
	spiritBasicPacketAddresses.cBroadcastAddress = 0;


	// Send Spirit configuration
	SpiritRadioInit(&spiritRadio); // XXX: WCP value is incorrect (according to datasheet)
	SpiritRadioSetPALeveldBm(0, WUPER_settings.txPowerdBm);
	SpiritRadioSetPALevelMaxIndex(0);

	SpiritPktBasicInit(&spiritBasicPacketInit);
	SpiritPktBasicAddressesInit(&spiritBasicPacketAddresses);
	SpiritPktCommonSetDestinationAddress(WUPER_settings.networkID); // Address (network ID)

	// Return to RX state if needed
	if (spiritStatus.MC_STATE == MC_STATE_RX)
		SpiritSpiCommandStrobes(COMMAND_RX);

}

void WUPER_GetRFSettings(WUPERSettings *settings) {
	*settings = WUPER_settings;
}
