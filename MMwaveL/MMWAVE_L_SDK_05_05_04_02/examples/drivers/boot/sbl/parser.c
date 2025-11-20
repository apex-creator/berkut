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

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <string.h>
#include "app_errors.h"
#include "parser.h"
#include "sbl.h"
#include "drivers/crc.h"
#include "parser_image.h"
#include "parser_image_validity.h"
#include "ti_drivers_config.h"
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
uint8_t c_ReadBuffer[3U][M_READ_BUFFER_SIZE];

T_METAHEADER                      t_Metaheader;
T_PARSER_CONTEXT                  t_ParserCtx;
T_BUFFER_PARSER_TASK_CTX          t_BuffParserTaskCtx;
T_BUFFER_PARSER_TASK_CTX          t_PartChunkParserTaskCtx;
T_PARSER_PING_PONG_BUFFER_CTX     t_PingPongBuffCtx;
/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @fn parser_init
 * @brief Initialize the parser variables
 *
 * @b Description @n
 * This function initializes the parser strucutres and variables
 *
 * @param[in] None
 *
 * @return None
 *
 * @b Assumption:
 * -----------------------------------------------------------------------------
 * <B> Taceability Information </B> @n
 *
 * <table>
 * <tr> <th> @b Requirements-Ids <td>
 *
 * <tr> <th> @b Architecture-Ids <td>
 *
 * <tr> <th> @b Design-Ids <td>
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
void parser_init(void)
{
    uint32_t w_loopCnt;

    memset((void*)&t_Metaheader, 0U, sizeof(T_METAHEADER));
    memset((void*)&t_ImagePrms, 0U, sizeof(T_PARSER_IMAGE_PRMS));
    memset((void*)&t_BuffParserTaskCtx, 0U, sizeof(T_BUFFER_PARSER_TASK_CTX));
    memset((void*)&t_PingPongBuffCtx, 0U, sizeof(T_PARSER_PING_PONG_BUFFER_CTX));

    parser_imageValidParamsInit();

    t_PingPongBuffCtx.c_WriteBuffIdx = M_PING_BUFFER_INDEX;
    t_PingPongBuffCtx.c_ParseBuffIdx = M_PONG_BUFFER_INDEX;
    t_PingPongBuffCtx.t_BufferParams[0U].c_BufferState = M_PARSER_BUFFER_EMPTY;
    t_PingPongBuffCtx.t_BufferParams[1U].c_BufferState = M_PARSER_BUFFER_EMPTY;

    t_ParserCtx.c_MetaHeaderInvalid  = 0U;
    t_ParserCtx.c_ParserState        = M_PARSER_BUFFER_PARSER_STATE_IDLE;
    t_ParserCtx.c_AppImagePresent    = 0U;
    t_ParserCtx.c_NumRprcFilesParsed = 0U;

    t_CfgFilePrms.w_NumCfgFiles = 0U;
    for (w_loopCnt = 0U; w_loopCnt < M_PARSER_MAX_NUM_CONFIG_FILE; w_loopCnt++)
    {
        (void)memset((void*)&t_CfgFilePrms.c_ConfigFile[w_loopCnt][0U], 0U,
                     M_PARSER_CONFIG_FILE_SIZE);
        t_CfgFilePrms.w_ConfigFileSize[w_loopCnt] = 0U;
        t_CfgFilePrms.c_IsConfigValid[w_loopCnt] = 0U;
        t_CfgFilePrms.c_IsConfigApplied[w_loopCnt] = 0U;
    }
    sblObj.errorStatus = sblObj.errorStatus & 0xFFFFFFFF00000000U;
}

/*!*****************************************************************************
 * @fn parser_buffMgmt
 * @brief Buffer Management function
 *
 * @b Description @n
 * This function Manages the buffer for various boot interfaces
 *
 * @param[in] p_parseBuff   -   pointer to the buffer to be parsed
 * @param[in] w_bufflen     -   buffer length
 * @param[in] c_islastBuff  -   flag specifying if it is the last buffer
 * @param[in] c_intfType    -   Interface Type
 *
 * @return None
 *
 * @b Assumption:
 * -----------------------------------------------------------------------------
 * <B> Taceability Information </B> @n
 *
 * <table>
 * <tr> <th> @b Requirements-Ids <td>
 *
 * <tr> <th> @b Architecture-Ids <td>
 *
 * <tr> <th> @b Design-Ids <td>
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
void parser_buffMgmt(uint8_t* p_parseBuff, uint32_t w_bufflen, uint8_t c_islastBuff,
                       uint8_t c_intfType)
{

    if (c_intfType == M_PARSER_INTERFACE_QSPI)
    {
        parser_buffer(p_parseBuff, w_bufflen);
    }
    else
    {
        /*! Invalid interface */
    }
}

/*!*****************************************************************************
 * @fn parser_buffer
 * @brief Parses the buffer received over any boot interface
 *
 * @b Description @n
 * This function parses the Metaimage Buffer to load the Image to RAM
 *
 * @param[in] p_parseBuff   -   Buffer to be parsed
 * @param[in] w_bufflen     -   buffer length
 *
 * @return None
 *
 * @b Assumption:
 * -----------------------------------------------------------------------------
 * <B> Taceability Information </B> @n
 *
 * <table>
 * <tr> <th> @b Requirements-Ids <td>
 *
 * <tr> <th> @b Architecture-Ids <td>
 *
 * <tr> <th> @b Design-Ids <td>
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
void parser_buffer(uint8_t* p_parseBuff, uint32_t w_bufflen)
{
    uint32_t w_bytesRead = 0U;
    int32_t xw_imageValidStatus;
    uint8_t* p_readPtr = p_parseBuff;

    do {
        switch (t_ParserCtx.c_ParserState)
        {
            case M_PARSER_BUFFER_PARSER_STATE_IDLE:
                t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_META_HEADER;
                break;

            case M_PARSER_BUFFER_PARSER_META_HEADER:
                w_bytesRead = parser_parseMetaHeader(p_readPtr);

                if (w_bytesRead == 0U)
                {
                    t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_COMPLETED;
                }
                else
                {
                    if (t_Metaheader.w_ImageSize <= w_bufflen)
                    {
                        w_bufflen = t_Metaheader.w_ImageSize;
                    }

                    w_bufflen = w_bufflen - w_bytesRead;
                    p_readPtr = p_readPtr + w_bytesRead;
                }
                break;

            case M_PARSER_BUFFER_PARSER_RPRC_DWLD:
                xw_imageValidStatus = parser_checkImageValidity(p_readPtr, w_bufflen);

                if (M_PARSER_CHECK_ERROR_STATUS)
                {
                    t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_COMPLETED;
                }
                else
                {
                    if (xw_imageValidStatus == 0)
                    {
                        w_bytesRead = parser_parseRprcImage(p_readPtr, w_bufflen);
                    }

                    w_bufflen = w_bufflen - w_bytesRead;
                    p_readPtr = p_readPtr + w_bytesRead;
                }
                break;

            default:
                t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_COMPLETED;
                break;

        }
    } while ((w_bufflen != 0U) &&
             (t_ParserCtx.c_ParserState != M_PARSER_BUFFER_PARSER_COMPLETED));
}

/*!*****************************************************************************
 * @fn parser_parseMetaHeader
 * @brief Parses the Metaimage Header
 *
 * @b Description @n
 * This function parses the Metaimage header
 *
 * @param[in] p_readPtr   -   Buffer to be parsed
 *
 * @return Number of parsed bytes
 *
 * @b Assumption:
 * -----------------------------------------------------------------------------
 * <B> Taceability Information </B> @n
 *
 * <table>
 * <tr> <th> @b Requirements-Ids <td>
 *
 * <tr> <th> @b Architecture-Ids <td>
 *
 * <tr> <th> @b Design-Ids <td>
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
uint32_t parser_parseMetaHeader(uint8_t* p_readPtr)
{
    uint32_t  w_metaheaderStart;
    uint32_t  w_metaheaderEnd;
    uint32_t  w_numBytesRead = 0U;
    uint32_t  w_metaheader[8U + (M_PARSER_NUM_FILES_SUPPORTED*8U)];
    uint32_t  w_computedCrc = 0U;

    uint32_t* p_metaheaderStart = (uint32_t*)p_readPtr;
    uint8_t   c_cnt;

    t_ParserCtx.c_MetaHeaderInvalid = 0U;

    w_metaheaderStart = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    t_Metaheader.w_NumFiles = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    t_Metaheader.w_ImageBootMode = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    t_Metaheader.w_HdrCrc = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    t_Metaheader.w_BootVector = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    t_Metaheader.w_ImageSize = *(uint32_t*)p_readPtr;
    p_readPtr = p_readPtr + 4U;
    w_numBytesRead = w_numBytesRead + 4U;

    if (w_metaheaderStart != M_PARSER_META_HDR_START)
    {
        t_ParserCtx.c_MetaHeaderInvalid = 1U;
        M_PARSER_SET_ERROR_STATUS(M_PARSER_METAHEADER_NOT_FOUND);
        w_numBytesRead = 0U;
    }
    else if (t_Metaheader.w_NumFiles > M_PARSER_NUM_FILES_SUPPORTED)
    {
        t_ParserCtx.c_MetaHeaderInvalid = 1U;
        M_PARSER_SET_ERROR_STATUS(M_PARSER_METAHEADER_NUMFILES_ERROR);
        w_numBytesRead = 0U;
    }
    else
    {
        for (c_cnt = 0U; c_cnt < t_Metaheader.w_NumFiles; c_cnt++)
        {
            t_Metaheader.t_ImageDetails[c_cnt].c_FileType = *(uint8_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            t_Metaheader.t_ImageDetails[c_cnt].w_MagicWord = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            t_Metaheader.t_ImageDetails[c_cnt].w_FileOffset = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            t_Metaheader.t_ImageDetails[c_cnt].w_FileCrc = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            t_Metaheader.t_ImageDetails[c_cnt].w_FileSize = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            t_Metaheader.t_ImageDetails[c_cnt].c_FilePresent = 1U;

            /*!
             * Check if the Image is for APP subsystem
             */
            if (((t_Metaheader.t_ImageDetails[c_cnt].w_MagicWord & 0xFFFF0000U) \
                                            >> 16U) == M_PARSER_APP_SUBSYSTEM)
            {
                t_ParserCtx.c_AppImagePresent = 1U;
            }

            p_readPtr = p_readPtr + 8U;
            w_numBytesRead = w_numBytesRead + 8U;
        }

        if (t_ParserCtx.c_AppImagePresent == 0U)
        {
            t_ParserCtx.c_MetaHeaderInvalid = 1U;
            M_PARSER_SET_ERROR_STATUS(M_PARSER_MSSIMAGE_NOT_FOUND);
            w_numBytesRead = 0U;
        }
        else
        {
            t_Metaheader.w_ShMemAlloc = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            w_metaheaderEnd = *(uint32_t*)p_readPtr;
            p_readPtr = p_readPtr + 4U;
            w_numBytesRead = w_numBytesRead + 4U;

            DebugP_assert(w_metaheaderEnd == M_PARSER_META_HDR_END);

            memcpy((void*)&w_metaheader[0U], (void*)p_metaheaderStart,
                   w_numBytesRead);

            /*!
             * settting the CRC field to zero to calculate CRC
             */
            w_metaheader[3U] = 0U;
            sbl_computeCrc32Cpu(&w_metaheader[0U], w_numBytesRead, &w_computedCrc);

            /*!
             * Integrity check for Metaheader
             */
            if (w_computedCrc != t_Metaheader.w_HdrCrc)
            {
                t_ParserCtx.c_MetaHeaderInvalid = 1U;
                M_PARSER_SET_ERROR_STATUS(M_PARSER_METAHEADER_CRC_FAILURE);
                w_numBytesRead = 0U;
            }
            else
            {
                /*!
                 * Ensure that the RPRC starts at 64 byte alignment
                 */
                if ((w_numBytesRead % 64U) != 0U)
                {
                     w_numBytesRead = w_numBytesRead + (64U - (w_numBytesRead%64));
                }
                t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_RPRC_DWLD;
            }
        }
    }

    return (w_numBytesRead);
}

/*
* END OF parser.c
*/
