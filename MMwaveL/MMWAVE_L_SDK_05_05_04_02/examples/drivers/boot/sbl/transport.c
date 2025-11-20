/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ti_drivers_open_close.h"
#include "transport.h"
#include "sbl.h"
#include "crc16.h"

/**************************************************************************
 ************************** Local Definitions *****************************
 **************************************************************************/
#define SBL_UART_CONSOLE                0U
#define SBL_UART_LOG_BUFF_SIZE          128U
#define SBL_XMODEM_HEADER_SIZE          3U
#define SBL_XMODEM_BUFFER_SIZE          1024U
#define SBL_XMODEM_CRC_SIZE             2U
#define SBL_XMODEM_DATABUFFER_SIZE      (SBL_XMODEM_BUFFER_SIZE + SBL_XMODEM_HEADER_SIZE + SBL_XMODEM_CRC_SIZE + 1U)
#define SBL_SOH                         0x01U
#define SBL_STX                         0x02U
#define SBL_EOT                         0x04U
#define SBL_ACK                         0x06U
#define SBL_NAK                         0x15U
#define SBL_CAN                         0x18U

/**************************************************************************
 *************************** Function Definitions *************************
 **************************************************************************/
static int32_t SBL_verifyCRC(uint8_t* dataBuffer, uint32_t dataLength, uint32_t verifyCRC);
// static void SBL_discardInput(void);

void SBL_UARTsend(uint8_t* buff, uint32_t numBytes)
{
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf = buff;
    trans.count = numBytes;
    UART_write(gUartHandle[SBL_UART_CONSOLE], &trans);
    return;
}

int32_t SBL_UARTrecv(uint8_t* buff, uint32_t numBytes)
{
    int32_t retVal = MINUS_ONE;
    UART_Transaction trans;
    UART_Transaction_init(&trans);
    trans.buf = buff;
    trans.count = numBytes;
    trans.timeout = 1000u;
    retVal = UART_read(gUartHandle[SBL_UART_CONSOLE], &trans);
    if (retVal == 0)
    {
        retVal = numBytes;
    }
    else
    {
        retVal = MINUS_ONE;
    }
    return retVal;
}
/*!
 *  @b Description
 *  @n
 *      This function is used to verify the CRC or checksum of a block of data.
 *
 *  @param[in]  dataBuffer
 *      Pointer to the data buffer.
 *  @param[in]  dataLength
 *      Size of the data buffer.
 *  @param[in]  verifyCRC
 *      Flag indicating whether CRC or checksum has to be verified.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */
// static int32_t SBL_verifyCRC(uint8_t* dataBuffer, uint32_t dataLength, uint32_t verifyCRC)
// {
    // int32_t     retVal = MINUS_ONE;
    // checkpoint = 5;

    // if (dataLength != 0)
    // {
        // if (verifyCRC)
        // {
            // uint16_t        CRC16Bit;
            // uint16_t        rxCRC;

            // /* Compute the 16 bit CRC. */
            // sbl_computeCrc16Cpu((uint16_t*)dataBuffer, dataLength,&CRC16Bit);

            // /* Read the CRC from the packet. */
            // rxCRC = (dataBuffer[dataLength] << 8U) + dataBuffer[dataLength + 1U];

            // if (CRC16Bit == rxCRC)
            // {
                // retVal = 0;
            // }
        // }
        // else
        // {
            // uint8_t     checksum = 0;
            // uint32_t    index;

            // /* Add all the bytes while dropping any carry overs to compute checksum. */
            // for (index = 0; index < dataLength; index++)
            // {
                // checksum += dataBuffer[index];
            // }

            // /* Compare to the 8 bit checksum received in the packet */
            // if (checksum == dataBuffer[dataLength])
            // {
                // retVal = 0;
            // }
        // }
    // }
    // return retVal;
// }

static int32_t SBL_verifyCRC(uint8_t* dataBuffer, uint32_t dataLength, uint32_t verifyCRC)
{
    int32_t     retVal = MINUS_ONE;

    if (dataLength != 0)
    {
        if (verifyCRC)
        {
            uint16_t        CRC16Bit;
            uint16_t        rxCRC;

            /* Compute the 16 bit CRC. */
            CRC16Bit = crc16_ccitt(dataBuffer, dataLength);

            /* Read the CRC from the packet. */
            rxCRC = (dataBuffer[dataLength] << 8U) + dataBuffer[dataLength + 1U];

            if (CRC16Bit == rxCRC)
            {
                retVal = 0;
            }
        }
        else
        {
            uint8_t     checksum = 0;
            uint32_t    index;

            /* Add all the bytes while dropping any carry overs to compute checksum. */
            for (index = 0; index < dataLength; index++)
            {
                checksum += dataBuffer[index];
            }

            /* Compare to the 8 bit checksum received in the packet */
            if (checksum == dataBuffer[dataLength])
            {
                retVal = 0;
            }
        }
    }
    return retVal;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to discard the incoming UART stream incase an error is detected.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
 static void SBL_discardInput(void)
{
    uint8_t     rxData;

    SBL_UARTrecv((uint8_t*)&rxData, 1U);
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function is used to read a input character from the UART peripheral.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */
int32_t SBL_transportRead(uint8_t* buffer, uint32_t size)
{
    UART_Transaction trans;
    int32_t     retVal;

    UART_Transaction_init(&trans);
    trans.buf = buffer;
    trans.count = size;
    trans.timeout = 1000u;
    /* Read using the UART peripheral */
    retVal = UART_read(gUartHandle[SBL_UART_CONSOLE], &trans);

    return retVal;
 }

/*!
 *  @b Description
 *  @n
 *      Takes the Formatted input string and dumps it on the UART console
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval Not Applicable
 */
void SBL_printf(const char *pcFormat, ...)
{
    UART_Transaction trans;
    uint8_t*        pcTemp = NULL;
    uint8_t         logUartBuff[SBL_UART_LOG_BUFF_SIZE + 1U];
    va_list         list;
    int32_t         iRet = 0;

    pcTemp = &logUartBuff[0U];

    UART_Transaction_init(&trans);
    
    va_start(list, pcFormat);
    iRet = vsnprintf((char *)pcTemp, SBL_UART_LOG_BUFF_SIZE, (const char *)pcFormat, list);
    va_end(list);
    
    trans.buf = pcTemp;
    trans.count = iRet;
    
    if((iRet > MINUS_ONE) && (iRet < SBL_UART_LOG_BUFF_SIZE))
    {
        UART_write(gUartHandle[SBL_UART_CONSOLE], &trans);
    }
    else
    {
        //Error
        while(1)
        {
        }
    }
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function downloads the application meta image file over UART using XMODEM.
 *      The downloaded file is written to SFLASH pointed to by the flash address.
 *      XMODEM receive routine is implemented based on the following sender<->receiver handshake.
 *      NAK and CAN handshake is not shown below.
 *
 *      RECEIVER ("s -k foo.bar")       SENDER ("foo.bar open x.x minutes")
 *
 *          C
 *                                          STX 01 FE Data[1024] CRC CRC
 *          ACK
 *                                          STX 02 FD Data[1024] CRC CRC
 *          ACK
 *                                          SOH 03 FC Data[128] CRC CRC
 *          ACK
 *                                          SOH 04 FB Data[100] CPMEOF[28] CRC CRC
 *          ACK
 *                                          EOT
 *          ACK
 *
 *  @param[in]  qspiFlashHandle
 *      Handle of QSPI Flash module.
 *  @param[in]  flashAddr
 *      Address of SFLASH location where the application meta image is written to.
 *  @param[in]  maxSize
 *      Maximum size of meta image.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Number of bytes read.
 *  @retval
 *      Error   -   SBL Error code.
 */

int32_t SBL_transportDownloadFile(Flash_Handle qspiFlashHandle, uint32_t flashAddr, uint32_t maxSize)
{
    uint8_t     dataBuffer[SBL_XMODEM_DATABUFFER_SIZE];
    uint8_t     txData, rxData;
    uint32_t    dataBufferSize = 0, computeCRC;
    uint32_t    index, extraDataLen;
     uint8_t     optionCRC;
    uint32_t    discardPacket = 0;
    int32_t     dataLength = 0;
    int32_t     retrans = 0;
    int32_t     retVal = 0;
    uint8_t     packetId = 1;
    uint8_t*    ptrData;

    SBL_printf("Start the image download using XMODEM over UART\r\n");

    /* Initiate the transfer by requesting the transmission with CRC. */
    optionCRC = 'C';
    computeCRC = 1;

    while (1)
    {
        for(index = 0; index < SBL_XMODEM_MAX_WAIT; index++)
        {
            /* Start the transfer with the 16 byte CRC */
            if (optionCRC)
            {
                SBL_UARTsend(&optionCRC, 1U);
            }
            retVal = SBL_UARTrecv((uint8_t*)&rxData,1);

            /* Check if 1 byte was read? */
            if (retVal != 1U)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        if (retVal == 1)
        {
            switch (rxData)
            {
                case SBL_SOH:
                {
                    /* Data will be sent in 128 byte chunks. */
                    dataBufferSize = 128U;
                    goto receive;
                }
                case SBL_STX:
                {
                    /* Data will be sent in 1024 byte chunks. */
                    dataBufferSize = 1024U;
                    goto receive;
                }
                case SBL_EOT:
                {
                    /* Transfer is done. Return the number of bytes read. */
                    sblObj.trans.numACKS++;
                    txData = SBL_ACK;
                      SBL_UARTsend(&txData, 1U);
                    goto exitGetFile;
                }
                case SBL_CAN:
                {
                    /* Check if Cancel wasn't sent by mistake. Check for a 2nd CAN message */
                    retVal = SBL_UARTrecv((uint8_t*)&rxData, 1U);

                    /* Check if 1 byte was read? */
                    if (retVal != 1U)
                    {
                        continue;
                    }
                    else
                    {
                        if (rxData == SBL_CAN)
                        {
                            SBL_discardInput();
                            sblObj.trans.numACKS++;
                            txData = SBL_ACK;
                            SBL_UARTsend(&txData, 1U);
                            dataLength = SBL_ECANCEL;
                            goto exitGetFile;
                        }
                        else
                        {
                            sblObj.trans.numNAKS++;
                            txData = SBL_NAK;
                            SBL_UARTsend(&txData, 1U);
                        }
                    }
                    break;
                }
                default:
                {
                    printf("Debug: Unsupported character received\n");
                    SBL_discardInput();
                    sblObj.trans.numNAKS++;
                    txData = SBL_NAK;
                    SBL_UARTsend(&txData, 1U);
                    continue;
                }
            }
        }
        else
        {
            /* Request for transmission with CRC failed. Try checksum mode */
            if (optionCRC == 'C')
            {
                /* Proceed with no CRC */
                optionCRC = SBL_NAK;
                computeCRC = 0;
                continue;
            }

            /* Both CRC and non-CRC tries were unsuccessful.
             * Cancel the download. */
            SBL_discardInput();
            txData = SBL_CAN;
            SBL_UARTsend(&txData, 1U);
            SBL_UARTsend(&txData, 1U);
            SBL_UARTsend(&txData, 1U);
            dataLength = SBL_ESYNC;
            goto exitGetFile;
        }
receive:
        /* We have succesfully started the transfer. We don't need to send 'C' anymore */
        optionCRC = 0;

        discardPacket = 0;

        /* Total datalength to read = 128 or 1024 bytes based on 0 byte of header + (16 bit CRC or 8 bit checksum) + 3 bytes of header */
        ptrData = dataBuffer;

        /* Start by storing the first byte of header */
        *ptrData++ = rxData;

        /* We have already read 1st byte. */
        extraDataLen = SBL_XMODEM_HEADER_SIZE - 1U;

        if (computeCRC)
        {
            /* Read 16 bit CRC. */
            extraDataLen += 2U;
        }
        else
        {
            /* Read 8 bit checksum. */
            extraDataLen += 1U;
        }
        /* Check if we should start a valid packet receive? */
        if (dataBufferSize == 0)
        {
            discardPacket = 1U;
        }
        else
        {
            /* Read the remaining bytes */
            for(index = 0; index < (dataBufferSize + extraDataLen); index++)
            {
                 retVal = SBL_UARTrecv((uint8_t*)&rxData, 1U);

                if (retVal != 1U)
                {
                    discardPacket = 1U;
                    break;
                }
                else
                {
                    *ptrData++ = rxData;
                }
            }
        }
        /* Check if the entire packet was received.
         * The sum header of the 2 bytes is 0XFF.
         * The packet Id should either be current packet or
         * previous packet(In this case don't send a NACK, it will cause a retransmission. ACK the packet but don't store it).
         * The CRC or checksum of the data is valid.
         */
        if ((discardPacket != 1U) &&
            ((dataBuffer[1U] + dataBuffer[2U]) == 0xFFU) &&
            (((dataBuffer[1U] == (uint8_t)packetId - 1U)) || (dataBuffer[1U] == packetId)) &&
            (SBL_verifyCRC(&dataBuffer[3U], dataBufferSize, computeCRC) == 0))
        {
            if (dataBuffer[1] == packetId)
            {
                dataLength += dataBufferSize;

                if (dataLength > maxSize)
                {
                    /* Cancel the download and return error. */
                    dataLength = SBL_EMAXMEM;
                    goto exitGetFile;
                }
                else
                {
                    Flash_write(qspiFlashHandle, flashAddr,(uint8_t *)&dataBuffer[3], dataBufferSize);

                    flashAddr += dataBufferSize;
                    packetId++;
                    sblObj.trans.numRxPackets++;
                }
            }

            /* Packet was successfully received and stored. ACK the packet */
            sblObj.trans.numACKS++;
            txData = SBL_ACK;
            SBL_UARTsend(&txData, 1U);
            continue;
        }
        else
        {
            /* Failure: Check if we exceeded the limit of unsuccessful retransmissions */
            if (retrans++ > SBL_XMODEM_MAX_RETRANSMISSIONS)
            {
                sblObj.trans.retransErrors++;
                SBL_discardInput();
                txData = SBL_CAN;
                SBL_UARTsend(&txData, 1U);
                SBL_UARTsend(&txData, 1U);
                SBL_UARTsend(&txData, 1U);
                dataLength = SBL_ERETRANS;
                goto exitGetFile;
            }

            /* One of the 4 conditions above was not met. Send a NACK requesting retransmission */
            sblObj.trans.numNAKS++;
            SBL_discardInput();
            txData = SBL_NAK;
            SBL_UARTsend(&txData, 1U);
            continue;
        }
    }
exitGetFile:
    return dataLength;
}


