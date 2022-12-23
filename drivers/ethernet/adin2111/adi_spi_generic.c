/*
 *---------------------------------------------------------------------------
 *
 * Copyright (c) 2020, 2021 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc.
 * and its licensors.By using this software you agree to the terms of the
 * associated Analog Devices Software License Agreement.
 *
 *---------------------------------------------------------------------------
 */

#include "adi_spi_generic.h"
#include <zephyr.h>

/** define size of data buffers, DMA check  */
#define BUFFERSIZE 2000

/* Transmit data buffer 2 bytes overhead for command*/
DMA_BUFFER_ALIGN(uint8_t spiRxBuf[BUFFERSIZE + 2], 4);
DMA_BUFFER_ALIGN(uint8_t spiTxBuf[BUFFERSIZE + 2], 4);

/* 16-bit data used to test CRC Data fill mode */
#define CRC16CCITT_POLYNOMIAL_BE    (0x1021)
#define CRC16CCITT_POLYNOMIAL_LE    (0x8408)

/* 16-bit data used to test CRC Data fill mode */
#define CRC16_SEED_VALUE            0xFFFFu

#define CRC8CCITT_POLYNOMIAL_LE     (0xe0)

/* 8-bit data used to test CRC Data fill mode */
#define CRC8CCITT_POLYNOMIAL_BE     (0x07)
#define CRC8CCITT_POLYNOMIAL_LE     (0xe0)

/* 8-bit data used to test CRC Data fill mode */

/* 8-bit data used to test CRC Data fill mode */
#define CRC8CCITT_POLYNOMIAL_BE     (0x07)
#define CRC8CCITT_POLYNOMIAL_LE     (0xe0)

/* 8-bit data used to test CRC Data fill mode */
#define CRC8_SEED_VALUE             0xFFu

#if defined(SPI_CRC_EN)
static uint8_t crc8_le(uint8_t crc, uint8_t *buf, uint32_t numbytes, uint32_t numbits, uint8_t poly)
{
    uint32_t i, j, len;

    len = numbytes + (numbits > 0 ? 1 : 0);
    for (i = 0; i < len; i++)
    {
        uint32_t bits;
        if (i == numbytes)
        {
            bits = numbits;
            crc ^= buf[i] & ((1 << bits) - 1);
        }
        else
        {
            bits = 8;
            crc ^= buf[i];
        }

        for (j = 0; j < bits; j++)
        {
            if (crc & 0x1)
            {
                crc >>= 1;
                crc ^= poly;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static uint8_t crc8_be(uint8_t crc, uint8_t *buf, uint32_t numbytes, uint32_t numbits, uint8_t poly)
{
    uint32_t i, j, len;

    len = numbytes + (numbits > 0 ? 1 : 0);
    for (i = 0; i < len; i++)
    {
        uint32_t bits;
        if (i == numbytes)
        {
            bits = numbits;
            crc ^= buf[i] >> (8 - bits) << (8 - bits);
        }
        else
        {
            bits = 8;
            crc ^= buf[i];
        }

        for (j = 0; j < bits; j++)
        {
            if (crc & 0x80)
            {
                crc <<= 1;
                crc ^= poly;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint8_t CrcBlock8(uint8_t *StartAddr, uint32_t NumBytes, uint32_t NumBits, bool bLsbFirst)
{
    uint8_t crc;

    /* Initialize CRC with the same seed as in TestCrc */
    crc = 0x00;//CRC8_SEED_VALUE;

    if (bLsbFirst)
    {
        return crc8_le(crc, StartAddr, NumBytes, NumBits, CRC8CCITT_POLYNOMIAL_LE);
    }
    else
    {
        return crc8_be(crc, StartAddr, NumBytes, NumBits, CRC8CCITT_POLYNOMIAL_BE);
    }
}
#endif

void spiCallback(void *pCBParam, uint32_t Event, void *pArg)
{
    adi_mac_Device_t        *hDevice = (adi_mac_Device_t *)pCBParam;
    uint32_t                tail;
    adi_mac_FrameHeader_t   header;
    uint32_t                byteOffset;
    uint8_t                 *pRxBuf;
    adi_mac_SpiState_e      spiState;
    uint32_t                expectedFcs = 0;
    uint8_t                 timestampBytes[8];

    (void)pArg;

    /* Set state early so queuing more Rx descriptors from callback does not fail due to */
    /* hDevice->spiState still being set to ADI_MAC_SPI_STATE_RX_FRAME or ADI_MAC_SPI_STATE_TX_FRAME. */
    spiState = hDevice->spiState;
    hDevice->spiState = ADI_MAC_SPI_STATE_READY;
    hDevice->spiErr = Event;

    if (spiState == ADI_MAC_SPI_STATE_RX_FRAME)
    {
        /* We need to read the header early because we may need to decide which priority queue to use */
        /* "+1" for the turnaround byte*/
        byteOffset = ADI_SPI_HEADER_SIZE + 1;

#if defined(SPI_CRC_EN)
        byteOffset++;
#endif
        /* Frame header for timestamp information */
        header = *(adi_mac_FrameHeader_t *)&spiRxBuf[byteOffset];
        header.VALUE16 = HTON16(header.VALUE16);

        adi_mac_RxFifoPrio_e prio = (adi_mac_RxFifoPrio_e)header.PRI;
#if defined(ADI_MAC_ENABLE_RX_QUEUE_HI_PRIO)
        /* The transaction size is recorded in the current active queue, we need to save that */
        /* and restore in case we switch queues.                                              */
        uint32_t trxSizeBackup = hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize;
        if (prio == ADI_MAC_RX_FIFO_PRIO_LOW)
        {
            hDevice->pRxQueue = &hDevice->rxQueueLp;
        }
        else
        {
            hDevice->pRxQueue = &hDevice->rxQueueHp;
        }
        hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize = trxSizeBackup;
#endif
        if (!queueIsEmpty(hDevice->pRxQueue))
        {
            pRxBuf = hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->pBuf;

            /* Adjust buffer size for frame header */
            byteOffset += ADI_FRAME_HEADER_SIZE;
            hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize -= ADI_FRAME_HEADER_SIZE;

            /* Get timestamp and adjust buffer size for Timestamp */
            if (header.TIME_STAMP_PRESENT)
            {
                if (hDevice->timestampFormat == ADI_MAC_TS_FORMAT_64B_1588)
                {
                    /* 64-bit timestamps. */
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestampExt = HTON32((*(uint32_t *)&spiRxBuf[byteOffset]));
                    memcpy(&timestampBytes[4], &spiRxBuf[byteOffset], 4);
                    byteOffset += 4;
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestamp = HTON32((*(uint32_t *)&spiRxBuf[byteOffset]));
                    memcpy(&timestampBytes[0], &spiRxBuf[byteOffset], 4);
                    byteOffset += 4;
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize -= 8;
                    /* MAC_CalculateParity returns 1 if timestampBytes has odd parity, and TIME_STAMP_PARITY is 0 if timestampBytes has odd parity. */
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestampValid = (MAC_CalculateParity(timestampBytes, 8) != header.TIME_STAMP_PARITY);
                }
                else
                {
                    /* 32-bit timestamps. */
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestampExt = 0;
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestamp = HTON32((*(uint32_t *)&spiRxBuf[byteOffset]));
                    memcpy(&timestampBytes[0], &spiRxBuf[byteOffset], 4);
                    byteOffset += 4;
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize -= 4;
                    /* MAC_CalculateParity returns 1 if timestampBytes has odd parity, and TIME_STAMP_PARITY is 0 if timestampBytes has odd parity. */
                    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->timestampValid = (MAC_CalculateParity(timestampBytes, 4) != header.TIME_STAMP_PARITY);
                }
            }

            memcpy(pRxBuf, (const uint8_t *)&spiRxBuf[byteOffset], hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize);

            /* Adjust buffer size for FCS */
            hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize -= FCS_SIZE;

            hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->prio = prio;
#if defined(CONFIG_ETH_ADIN2111)
            hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->port = header.PORT;
#endif
            if (hDevice->fcsCheckEn)
            {
                uint32_t actualFcs;
                memcpy(&actualFcs, &pRxBuf[hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize], FCS_SIZE);
                expectedFcs = ADI_HAL_FCS_CALCULATE(hDevice->adinDevice, pRxBuf, hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize);
                if (expectedFcs != actualFcs)
                {
                    Event |= ADI_MAC_CALLBACK_STATUS_FCS_ERROR;
                }
            }
            tail = hDevice->pRxQueue->tail;
            queueRemove(hDevice->pRxQueue);

#if defined(CONFIG_ETH_ADIN2111)
            /* Update the dynamic forwarding table */
            if (hDevice->cbFunc[ADI_MAC_EVT_DYN_TBL_UPDATE] != NULL)
            {
                hDevice->cbFunc[ADI_MAC_EVT_DYN_TBL_UPDATE](hDevice->adinDevice, Event, hDevice->pRxQueue->pEntries[tail].pBufDesc);
            }
#endif
            if (hDevice->pRxQueue->pEntries[tail].pBufDesc->cbFunc)
            {
                hDevice->pRxQueue->pEntries[tail].pBufDesc->cbFunc(hDevice->adinDevice, Event, hDevice->pRxQueue->pEntries[tail].pBufDesc);
            }
        }

        hDevice->state = ADI_MAC_STATE_READY;
        hDevice->spiState = ADI_MAC_SPI_STATE_READY;

        if (!queueIsEmpty(&hDevice->txQueue))
        {
            ADI_HAL_SET_PENDING_IRQ(hDevice->adinDevice);
        }

        if (!hDevice->pendingCtrl)
        {
            /* SPI transaction finished, enable the MAC interrupt. */
            ADI_HAL_ENABLE_IRQ(hDevice->adinDevice);
        }
    }
    else
    {
        if (spiState == ADI_MAC_SPI_STATE_TX_FRAME)
        {
            tail = hDevice->txQueue.tail;
            queueRemove(&hDevice->txQueue);

            hDevice->state = ADI_MAC_STATE_READY;

            /* Decrement the reference count, and call the callback function only if the reference  */
            /* count is 0. This ensures that if the intent was to send the buffer to both ports, it */
            /* will be returned to the buffer pool only after sending to both ports has completed.  */
            hDevice->txQueue.pEntries[tail].pBufDesc->refCount--;
            if (hDevice->txQueue.pEntries[tail].pBufDesc->cbFunc && (!hDevice->txQueue.pEntries[tail].pBufDesc->refCount))
            {
                hDevice->txQueue.pEntries[tail].pBufDesc->cbFunc(hDevice->adinDevice, Event, hDevice->txQueue.pEntries[tail].pBufDesc);
            }

            if (!hDevice->pendingCtrl)
            {
                /* SPI transaction finished, enable the MAC interrupt. */
                ADI_HAL_ENABLE_IRQ(hDevice->adinDevice);
            }
        }
    }

}

adi_eth_Result_e MAC_Write(adi_mac_Device_t *hDevice, uint16_t regAddr, void *pBuf, uint32_t nBytes, bool blocking)
{
    adi_eth_Result_e    result = ADI_ETH_SUCCESS;
    uint32_t            byteOffset = ADI_SPI_HEADER_SIZE;
    uint32_t            byteCount = nBytes;
    uint8_t             *pData = (uint8_t *)pBuf;
    adi_mac_SpiHeader_t spiHeader;
    bool                useDma = false;
    uint32_t            timeout;
#if defined (SPI_CRC_EN)
    uint8_t             tmpcrc = 0;
#endif

    if (hDevice == NULL)
    {
        result = ADI_ETH_INVALID_HANDLE;
        goto end;
    }

    if (hDevice->state == ADI_MAC_STATE_UNINITIALIZED)
    {
        result = ADI_ETH_DEVICE_UNINITIALIZED;
        goto end;
    }

    if (byteCount > BUFFERSIZE)
    {
        result = ADI_ETH_INVALID_PARAM;
        goto end;
    }

    /* Wait for ongoing SPI transaction to finish, with a timeout. */
    timeout = ADI_SPI_TIMEOUT;
    do
    {
        timeout--;
    } while (timeout && (hDevice->spiState != ADI_MAC_SPI_STATE_READY));

    if (!timeout)
    {
        result = ADI_ETH_COMM_TIMEOUT;
        goto end;
    }

    /* Create the SPI header and populate it at the start of the SPI Tx buffer */
    spiHeader.CD = ADI_MAC_SPI_TRANSACTION_CONTROL;
    spiHeader.FD = ADI_MAC_SPI_HALF_DUPLEX;
    spiHeader.RW = ADI_MAC_SPI_WRITE;
    spiHeader.ADDR = regAddr;
    spiTxBuf[0] = spiHeader.VALUE16 >> 8;
    spiTxBuf[1] = spiHeader.VALUE16 & 0xFF;

#if defined(SPI_CRC_EN)
    spiTxBuf[2] = CrcBlock8(spiTxBuf, ADI_SPI_HEADER_SIZE, 0, false);
    if(regAddr != ADDR_MAC_TX)
    {
        byteCount = ((nBytes) + (nBytes / ADI_MAC_SPI_ACCESS_SIZE)) + 1;
    }
    else
    {
        byteCount = nBytes;
        byteOffset += 1;
    }
#endif

#if defined (SPI_CRC_EN)
    for (unsigned int i = 0u,j = 0u; i < byteCount  ; i++)
    {
        if (regAddr != ADDR_MAC_TX)
        {
            if((i > 0))
            {
                if((i % (ADI_MAC_SPI_ACCESS_SIZE + 1)) == 0)
                {
                    tmpcrc = CrcBlock8((pData + (j - ADI_MAC_SPI_ACCESS_SIZE)), ADI_MAC_SPI_ACCESS_SIZE, 0, false);
                    spiTxBuf[i + byteOffset] = tmpcrc;
                }
                else
                {
                  spiTxBuf[i + byteOffset] = pData[j++];
                }
            }
        }
        else
        {
            spiTxBuf[i + byteOffset] = pData[i];
        }
#else
    for (unsigned int i = 0u; i < byteCount  ; i++)
    {
        spiTxBuf[i + byteOffset] = pData[i];
#endif
    }

    byteCount = byteCount + byteOffset;

    useDma = (byteCount >= MIN_SIZE_FOR_DMA);

    /* Platform dependent function */
    if (regAddr == ADDR_MAC_TX)
    {
        hDevice->spiState = ADI_MAC_SPI_STATE_TX_FRAME;
    }
    else
    {
        hDevice->spiState = ADI_MAC_SPI_STATE_TX;
    }

    /* Platform dependent function */
    ADI_HAL_SPI_READ_WRITE(hDevice->adinDevice, spiTxBuf, spiRxBuf, byteCount, useDma);

    if (blocking)
    {
        /* This can potentially lock up the driver, will need to be loooked at (add timeout?). */
        while (hDevice->spiState != ADI_MAC_SPI_STATE_READY)
        {
            k_yield();
        }
    }
end:

    return result;
}

/* Write a full frame to the MAC device */
static adi_eth_Result_e MAC_WriteFrame(adi_mac_Device_t *hDevice, adi_mac_FrameStruct_t *pFrame)
{
    adi_eth_Result_e        result = ADI_ETH_SUCCESS;
    uint32_t                byteCount;
    adi_mac_SpiHeader_t     spiHeader;
    uint32_t                byteIdx;
    bool                    useDma = false;

    if (hDevice == NULL)
    {
        result = ADI_ETH_INVALID_HANDLE;
        goto end;
    }

    if (hDevice->state == ADI_MAC_STATE_UNINITIALIZED)
    {
        result = ADI_ETH_DEVICE_UNINITIALIZED;
        goto end;
    }

    /* Check if the SPI master is already in use */
    /* If it is, return and retry later. */
    if (hDevice->spiState != ADI_MAC_SPI_STATE_READY)
    {
        result = ADI_ETH_COMM_BUSY;
        goto end;
    }

    hDevice->spiState = ADI_MAC_SPI_STATE_TX_FRAME;

    byteCount = pFrame->pBufDesc->trxSize + ADI_FRAME_HEADER_SIZE + ADI_SPI_HEADER_SIZE;
    /* If SPI CRC is enabled, we have an extra byte for CRC. */
#if defined(SPI_CRC_EN)
    byteCount++;
#endif
    if (byteCount > BUFFERSIZE)
    {
        result = ADI_ETH_PARAM_OUT_OF_RANGE;
        goto end;
    }

    byteIdx = 0;
    /* SPI header, this is placed in the first 2 bytes of the SPI transmite buffer */
    spiHeader.CD = ADI_MAC_SPI_TRANSACTION_CONTROL;
    spiHeader.FD = ADI_MAC_SPI_HALF_DUPLEX;
    spiHeader.RW = ADI_MAC_SPI_WRITE;
    spiHeader.ADDR = ADDR_MAC_TX;
    spiTxBuf[byteIdx++] = spiHeader.VALUE16 >> 8;
    spiTxBuf[byteIdx++] = spiHeader.VALUE16 & 0xFF;

    /* If SPI CRC is enabled, add it for the SPI header. Note there is no CRC */
    /* for rest of the transaction (frame header + frame). */
#if defined(SPI_CRC_EN)
    spiTxBuf[byteIdx++] = CrcBlock8(&spiTxBuf[0], ADI_SPI_HEADER_SIZE, 0, false);
#endif

    /* Append frame header */
    spiTxBuf[byteIdx++] = pFrame->header.VALUE16 >> 8;
    spiTxBuf[byteIdx++] = pFrame->header.VALUE16 & 0xFF;

    /* Copy the frame contents to the SPI transmit buffer. */
    memcpy(&spiTxBuf[byteIdx], pFrame->pBufDesc->pBuf, pFrame->pBufDesc->trxSize);

    /* Odd-size frames need an extra byte */
    if (pFrame->pBufDesc->trxSize & 1)
    {
        byteCount++;
    }

    /* Determine if it's worth using DMA based on the transaction size. */
    useDma = (byteCount >= MIN_SIZE_FOR_DMA);

    /* Platform dependent function*/
    ADI_HAL_SPI_READ_WRITE(hDevice->adinDevice, spiTxBuf, spiRxBuf, byteCount, useDma);

end:
    return result;
}

adi_eth_Result_e MAC_Read(adi_mac_Device_t *hDevice, uint16_t regAddr, void *pBuf, uint32_t nBytes, bool blocking)
{
    adi_eth_Result_e        result = ADI_ETH_SUCCESS;
    uint32_t                TX_HEADER = 2;
    uint32_t                fillCount = 0;
    uint32_t                byteCount = nBytes;
    uint8_t                 *pData = (uint8_t *)pBuf;
    adi_mac_SpiHeader_t     spiHeader;
    bool                    useDma = false;
    uint32_t                timeout;
#if defined(SPI_CRC_EN)
    uint32_t                dataCounter = 0;
#endif

    if (hDevice == NULL)
    {
        result = ADI_ETH_INVALID_HANDLE;
        goto end;
    }

    if (hDevice->state == ADI_MAC_STATE_UNINITIALIZED)
    {
        result = ADI_ETH_DEVICE_UNINITIALIZED;
        goto end;
    }

    // Todo: Should this account for CRC/turnaround bytes?
    if (byteCount > (BUFFERSIZE - 2))
    {
      result = ADI_ETH_INVALID_PARAM;
      goto end;
    }

    /* Wait for ongoing SPI transaction to finish, with a timeout. */
    timeout = ADI_SPI_TIMEOUT;
    do
    {
        timeout--;
    } while (timeout && (hDevice->spiState != ADI_MAC_SPI_STATE_READY));

    if (!timeout)
    {
        result = ADI_ETH_COMM_TIMEOUT;
        goto end;
    }

    spiHeader.CD = ADI_MAC_SPI_TRANSACTION_CONTROL;
    spiHeader.FD = ADI_MAC_SPI_HALF_DUPLEX;
    spiHeader.RW = ADI_MAC_SPI_READ;
    spiHeader.ADDR = regAddr;
    spiTxBuf[0] = spiHeader.VALUE16 >> 8;
    spiTxBuf[1] = spiHeader.VALUE16 & 0xFF;

#if defined(SPI_CRC_EN)
    spiTxBuf[2] = CrcBlock8(spiTxBuf, ADI_SPI_HEADER_SIZE, 0, false);
    if(regAddr != ADDR_MAC_P1_RX)
    {
        /* Bytes for registers + 1 byte CRC per register + turnaround */
        fillCount = ((nBytes) + (nBytes / ADI_MAC_SPI_ACCESS_SIZE)) + 1;
    }
    else
    {
        /* Bytes for registers + turnaround */
        fillCount = nBytes + 1;
    }
    byteCount = TX_HEADER + fillCount + 1;
#else
    /* Bytes for registers + turnaround */
    fillCount = nBytes + 1;
    byteCount = TX_HEADER + fillCount;
#endif

    for (uint32_t i = (byteCount - fillCount); i < byteCount; i++)
    {
        spiTxBuf[i] = 0;
    }

    useDma = (byteCount >= MIN_SIZE_FOR_DMA);

#if defined(CONFIG_ETH_ADIN2111)
    if ((regAddr == ADDR_MAC_P1_RX) || (regAddr == ADDR_MAC_P2_RX))
#else
    if (regAddr == ADDR_MAC_P1_RX)
#endif
    {

        if (queueIsEmpty(hDevice->pRxQueue))
        {
            result = ADI_ETH_QUEUE_EMPTY;
            goto end;
        }

        if (nBytes > hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->bufSize)
        {
            result = ADI_ETH_BUFFER_TOO_SMALL;
            goto end;
        }
        hDevice->spiState = ADI_MAC_SPI_STATE_RX_FRAME;
    }
    else
    {
        hDevice->spiState = ADI_MAC_SPI_STATE_RX;
    }

    /* Platform dependent function */
    ADI_HAL_SPI_READ_WRITE(hDevice->adinDevice, spiTxBuf, spiRxBuf, byteCount, useDma);

    if (blocking)
    {
        /* This can potentially lock up the driver, will need to be looked at (add timeout?). */
        while (hDevice->spiState != ADI_MAC_SPI_STATE_READY) {
            k_yield();
        }

        /* 1 is the turnaround byte */
        uint32_t dataOffset = TX_HEADER + 1;
#if defined(SPI_CRC_EN)
        dataOffset++;
#endif

#if defined(CONFIG_ETH_ADIN2111)
        if ((regAddr == ADDR_MAC_P1_RX) || (regAddr == ADDR_MAC_P2_RX))
#else
        if (regAddr == ADDR_MAC_P1_RX)
#endif
        {
            /* Frame read */
            memcpy(pData, &spiRxBuf[dataOffset], nBytes);
        }
        else
        {
            /* Control read */
#if !defined(SPI_CRC_EN)
            memcpy(pData, &spiRxBuf[dataOffset], nBytes);
#else
            for (uint32_t i = dataOffset; i < dataOffset + fillCount - 1; i += (ADI_MAC_SPI_ACCESS_SIZE + 1))
            {
                if (spiRxBuf[i + ADI_MAC_SPI_ACCESS_SIZE] != CrcBlock8(&spiRxBuf[i], ADI_MAC_SPI_ACCESS_SIZE, 0, false))
                {
                    result = ADI_ETH_CRC_ERROR;
                    goto end;
                }
                *(ADI_MAC_SPI_ACCESS_UNIT_TYPE *)&pData[dataCounter] = *(ADI_MAC_SPI_ACCESS_UNIT_TYPE *)&spiRxBuf[i];
                dataCounter += ADI_MAC_SPI_ACCESS_SIZE;
            }
#endif
        }
    }

end:

    if (result != ADI_ETH_SUCCESS)
    {
        hDevice->state = ADI_MAC_STATE_READY;
        ADI_HAL_ENABLE_IRQ(hDevice->adinDevice);
    }

    return result;
}

adi_eth_Result_e MAC_SendFrame(adi_mac_Device_t *hDevice, adi_mac_FrameStruct_t *pFrame)
{
  adi_eth_Result_e          result = ADI_ETH_SUCCESS;
  uint16_t                  nBytes;
  uint32_t                  txFifoAvail;
  adi_mac_State_e           stateBackup;


    if (hDevice == NULL)
    {
        result = ADI_ETH_INVALID_HANDLE;
        goto end;
    }

    stateBackup = hDevice->state;
    if (hDevice->state == ADI_MAC_STATE_UNINITIALIZED)
    {
        result = ADI_ETH_DEVICE_UNINITIALIZED;
        goto end;
    }

    if ((hDevice->spiState != ADI_MAC_SPI_STATE_READY) || (hDevice->state != ADI_MAC_STATE_READY))
    {
        result = ADI_ETH_COMM_BUSY;
        goto end;
    }

    hDevice->state = ADI_MAC_STATE_TX_FRAME;

    if ((result = MAC_ReadRegister(hDevice, ADDR_MAC_TX_SPACE, &txFifoAvail)) != ADI_ETH_SUCCESS)
    {
        goto end;
    }

    nBytes = pFrame->pBufDesc->trxSize + ADI_FRAME_HEADER_SIZE;

    /* TX_SPACE contains the number of halfwords available in the TxFIFO, multiply by 2 to match the frame size in bytes */
    if (2 * txFifoAvail < nBytes + ADI_SPI_TX_FIFO_BUFFER)
    {
        result = ADI_ETH_INSUFFICIENT_FIFO_SPACE;
        goto end;
    }

    if ((result = MAC_WriteRegister(hDevice, ADDR_MAC_TX_FSIZE, nBytes)) != ADI_ETH_SUCCESS)
    {
        goto end;
    }

    if ((result = MAC_WriteFrame(hDevice, pFrame))!= ADI_ETH_SUCCESS)
    {
        goto end;
    }

end:

    if (result != ADI_ETH_SUCCESS)
    {
        hDevice->state = stateBackup;
    }

    return result;
}

adi_eth_Result_e MAC_RecvFrame(adi_mac_Device_t *hDevice, uint32_t port)
{
    adi_eth_Result_e    result = ADI_ETH_SUCCESS;
    uint32_t            rxSize = 0;
    uint32_t            regAddr;

    (void)port;

    if (hDevice == NULL)
    {
        result = ADI_ETH_INVALID_HANDLE;
        goto end;
    }

    if (hDevice->state == ADI_MAC_STATE_UNINITIALIZED)
    {
        result = ADI_ETH_DEVICE_UNINITIALIZED;
        goto end;
    }

    regAddr = ADDR_MAC_P1_RX_FSIZE;
#if defined(CONFIG_ETH_ADIN2111)
    if (port)
    {
        regAddr = ADDR_MAC_P2_RX_FSIZE;
    }
#endif

    hDevice->state = ADI_MAC_STATE_RX_FRAME;

    if ((result = MAC_ReadRegister(hDevice, regAddr, &rxSize)) != ADI_ETH_SUCCESS)
    {
        goto end;
    }

    /* Do not try to read a frame if size is 0 */
    if (!rxSize)
    {
        goto end;
    }

    /* Allow the user to swap in another buffer/buffer descriptor, with a suitable size for the received frame. */
    if (hDevice->cbFunc[ADI_MAC_EVT_RX_FRAME_RDY] != NULL)
    {
        hDevice->cbFunc[ADI_MAC_EVT_RX_FRAME_RDY](hDevice->adinDevice, rxSize, (void *)hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc);
    }

    hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->trxSize = rxSize;

    /* Odd-size frames need an extra byte */
    if (rxSize & 1)
    {
        rxSize++;
    }

    regAddr = ADDR_MAC_P1_RX;
#if defined(CONFIG_ETH_ADIN2111)
    if (port)
    {
        regAddr = ADDR_MAC_P2_RX;
    }
#endif

    result = MAC_Read(hDevice, regAddr, hDevice->pRxQueue->pEntries[hDevice->pRxQueue->tail].pBufDesc->pBuf, rxSize, false);
    if (result != ADI_ETH_SUCCESS)
    {
        goto end;
    }

end:
  return result;
}
