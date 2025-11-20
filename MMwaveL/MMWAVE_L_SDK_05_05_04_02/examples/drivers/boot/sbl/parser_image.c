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
#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include "parser_image.h"
#include "parser.h"
#include "app_errors.h"
#include "sbl.h"

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
T_PARSER_IMAGE_PRMS t_ImagePrms;
T_CONFIG_FILE_PRMS  t_CfgFilePrms;
/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */
/*!*****************************************************************************
 * @fn parser_parseRprcImage
 * @brief parse RPRC Image
 *
 * @b Description @n
 * Function for Parsing RPRC Image
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return number of bytes parsed
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
uint32_t parser_parseRprcImage(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_remainingLen = w_bufflen;
    uint32_t w_bytesRead;
    uint32_t w_totalBytesRead = 0U;

    do {
        if (t_ImagePrms.c_RprcState ==
                                    M_PARSER_RPRC_PARSER_STATE_IDLE)
        {
            w_bytesRead = parser_parseRprcHeader(p_readPtr, w_remainingLen);
            w_remainingLen = w_remainingLen - w_bytesRead;
            p_readPtr = p_readPtr + w_bytesRead;
            w_totalBytesRead = w_totalBytesRead + w_bytesRead;
        }

        if (t_ImagePrms.c_RprcState ==
                                    M_PARSER_RPRC_PARSER_SECTION_HEADER)
        {
            w_bytesRead = parser_parseRprcSecHeader(p_readPtr, w_remainingLen);
            w_remainingLen = w_remainingLen - w_bytesRead;
            p_readPtr = p_readPtr + w_bytesRead;
            w_totalBytesRead = w_totalBytesRead + w_bytesRead;
        }

        if (t_ImagePrms.c_RprcState ==
                                    M_PARSER_RPRC_PARSER_SECTION_DOWNLOAD)
        {
            w_bytesRead = parser_parseRprcSecContent(p_readPtr, w_remainingLen);
            w_remainingLen = w_remainingLen - w_bytesRead;
            p_readPtr = p_readPtr + w_bytesRead;
            w_totalBytesRead = w_totalBytesRead + w_bytesRead;
        }

        if (t_ImagePrms.c_RprcState ==
                                    M_PARSER_RPRC_PARSER_ALIGNMENT_BYTES)
        {
            w_bytesRead = parser_readAlignmentBytes(p_readPtr, w_remainingLen);
            w_remainingLen = w_remainingLen - w_bytesRead;
            p_readPtr = p_readPtr + w_bytesRead;
            w_totalBytesRead = w_totalBytesRead + w_bytesRead;
        }
    } while (w_remainingLen != 0U);

    if ((t_ImagePrms.c_RprcState == M_PARSER_RPRC_PARSER_STATE_IDLE) &&
        (t_ParserCtx.c_NumRprcFilesParsed == t_Metaheader.w_NumFiles))
    {
        t_ParserCtx.c_ParserState = M_PARSER_BUFFER_PARSER_COMPLETED;
    }

    return (w_totalBytesRead);
}

/*!*****************************************************************************
 * @fn parser_parseRprcHeader
 * @brief parse RPRC Header
 *
 * @b Description @n
 * Function for Parsing RPRC Header
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return number of bytes parsed
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
uint32_t parser_parseRprcHeader(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_remainingBytes = w_bufflen;
    uint32_t w_bytesRead = 0U;
    uint32_t w_header;
    uint32_t w_fileCount;

    while ((w_remainingBytes >= 8U) &&
                (t_ImagePrms.c_HdrState != M_PARSER_RPRC_HDR_STATE_END))
    {
        switch (t_ImagePrms.c_HdrState)
        {
            case M_PARSER_RPRC_HDR_STATE_MAGIC:
                w_header = *(uint32_t*)p_readPtr;
                if (w_header != M_PARSER_RPRC_MAGIC_WORD)
                {
                    M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_HDR_NOT_FOUND);
                    w_bytesRead = w_bufflen;
                    w_remainingBytes = 0U;
                }
                else
                {
                    p_readPtr = p_readPtr + 8U;
                    w_bytesRead = w_bytesRead + 8U;
                    w_remainingBytes = w_remainingBytes - 8U;
                    t_ImagePrms.c_HdrState = M_PARSER_RPRC_HDR_STATE_LEN;
                }
                break;

            case M_PARSER_RPRC_HDR_STATE_LEN:
                p_readPtr = p_readPtr + 4U;
                w_bytesRead = w_bytesRead + 4U;
                t_ImagePrms.w_NumSections = *(uint32_t*)p_readPtr;
                p_readPtr = p_readPtr + 4U;
                w_bytesRead = w_bytesRead + 4U;
                w_remainingBytes = w_remainingBytes - 8U;
                t_ImagePrms.c_HdrState = M_PARSER_RPRC_HDR_STATE_RSVD;
                break;

            case M_PARSER_RPRC_HDR_STATE_RSVD:
                p_readPtr = p_readPtr + 8U;
                w_bytesRead = w_bytesRead + 8U;
                w_remainingBytes = w_remainingBytes - 8U;
                t_ImagePrms.c_HdrState = M_PARSER_RPRC_HDR_STATE_END;
                t_ImagePrms.c_RprcState =
                                        M_PARSER_RPRC_PARSER_SECTION_HEADER;
                t_ParserCtx.c_NumRprcFilesParsed =
                                        t_ParserCtx.c_NumRprcFilesParsed + 1U;
                break;

            default:
                break;
        }
    }

    if(!M_PARSER_CHECK_ERROR(M_PARSER_RPRC_HDR_NOT_FOUND))
    {
        if (t_ImagePrms.c_HdrState == M_PARSER_RPRC_HDR_STATE_END)
        {
            w_fileCount = t_ParserCtx.c_NumRprcFilesParsed - 1U;
            t_ImagePrms.w_FileSize =
                            t_Metaheader.t_ImageDetails[w_fileCount].w_FileSize;
            t_ImagePrms.w_FileType =
                            t_Metaheader.t_ImageDetails[w_fileCount].w_MagicWord;

            if ((t_ImagePrms.w_FileSize % 64U) != 0U)
            {
                t_ImagePrms.w_NumAlignBytes =
                                (64U - (t_ImagePrms.w_FileSize % 64U));
            }
            else
            {
                t_ImagePrms.w_NumAlignBytes = 0U;
            }
        }

        t_ImagePrms.w_ImageSizeRead = t_ImagePrms.w_ImageSizeRead +
                                            w_bytesRead;
    }

    return (w_bytesRead);
}


/*!*****************************************************************************
 * @fn parser_parseRprcSecHeader
 * @brief parse RPRC Section header
 *
 * @b Description @n
 * Function for Parsing RPRC Section header
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return number of bytes parsed
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
uint32_t parser_parseRprcSecHeader(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_remainingBytes = w_bufflen;
    uint32_t w_bytesRead = 0U;
    uint32_t w_offset = 0U;

    while ((w_remainingBytes >= 8U) &&
                (t_ImagePrms.t_SectPrms.c_SectState != M_PARSER_SEC_STATE_HDR_END))
    {
        switch (t_ImagePrms.t_SectPrms.c_SectState)
        {
            case M_PARSER_SEC_STATE_START_ADDR:
                t_ImagePrms.t_SectPrms.dw_SecStrtAddr =
                                                            *(uint64_t*)p_readPtr;
                p_readPtr = p_readPtr + 8U;
                w_bytesRead = w_bytesRead + 8U;
                w_remainingBytes = w_remainingBytes - 8U;
                t_ImagePrms.t_SectPrms.c_SectState = M_PARSER_SEC_STATE_SEC_LEN;
                break;

            case M_PARSER_SEC_STATE_SEC_LEN:
                t_ImagePrms.t_SectPrms.w_SecLen = *(uint32_t*)p_readPtr;
                p_readPtr = p_readPtr + 8U;
                w_bytesRead = w_bytesRead + 8U;
                w_remainingBytes = w_remainingBytes - 8U;
                t_ImagePrms.t_SectPrms.c_SectState = M_PARSER_SEC_STATE_RSVD;
                break;

            case M_PARSER_SEC_STATE_RSVD:
                p_readPtr = p_readPtr + 8U;
                w_bytesRead = w_bytesRead + 8U;
                w_remainingBytes = w_remainingBytes - 8U;
                t_ImagePrms.t_SectPrms.c_SectState = M_PARSER_SEC_STATE_HDR_END;
                t_ImagePrms.c_RprcState =
                                        M_PARSER_RPRC_PARSER_SECTION_DOWNLOAD;
                break;

            default:
                break;
        }
    }

    if (t_ImagePrms.t_SectPrms.c_SectState == M_PARSER_SEC_STATE_HDR_END)
    {
        if ((t_ImagePrms.t_SectPrms.w_SecLen % 8U) != 0U)
        {
            t_ImagePrms.t_SectPrms.w_SecLen =
                t_ImagePrms.t_SectPrms.w_SecLen +
                (8U - (t_ImagePrms.t_SectPrms.w_SecLen % 8U));
        }

        w_offset =
        parser_calculateImageOffset((uint32_t)t_ImagePrms.t_SectPrms.dw_SecStrtAddr,
                                    t_ImagePrms.t_SectPrms.w_SecLen);
        t_ImagePrms.t_SectPrms.p_SectionPtr =
        (uint32_t*)((uint32_t)t_ImagePrms.t_SectPrms.dw_SecStrtAddr + w_offset);

        t_ImagePrms.t_SectPrms.w_WrittenLen = 0U;
    }

    if (M_PARSER_CHECK_ERROR_STATUS)
    {
        w_bytesRead = w_bufflen;
    }

    t_ImagePrms.w_ImageSizeRead = t_ImagePrms.w_ImageSizeRead +
                                        w_bytesRead;

    return (w_bytesRead);
}

/*!*****************************************************************************
 * @fn parser_parseRprcSecContent
 * @brief parse RPRC Section content
 *
 * @b Description @n
 * Function for Parsing RPRC Section content
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return number of bytes parsed
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
uint32_t parser_parseRprcSecContent(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_bytesRead = 0U;
    uint32_t w_bytesToWrite;
    uint32_t w_size;
    uint8_t c_cnt;
    void *p_memAddRet;

    if (t_ImagePrms.t_SectPrms.w_SecLen >
                                t_ImagePrms.t_SectPrms.w_WrittenLen)
    {
        if (w_bufflen >= (t_ImagePrms.t_SectPrms.w_SecLen -
                                t_ImagePrms.t_SectPrms.w_WrittenLen))
        {
            w_bytesToWrite = t_ImagePrms.t_SectPrms.w_SecLen -
                                t_ImagePrms.t_SectPrms.w_WrittenLen;
        }
        else
        {
            w_bytesToWrite = w_bufflen;
        }

        p_memAddRet = memcpy((void*)t_ImagePrms.t_SectPrms.p_SectionPtr,
                             (const void*)p_readPtr, w_bytesToWrite);

        t_ImagePrms.t_SectPrms.w_WrittenLen =
                            t_ImagePrms.t_SectPrms.w_WrittenLen + w_bytesToWrite;
        DebugP_assert(p_memAddRet == (void*)t_ImagePrms.t_SectPrms.p_SectionPtr);

        w_bytesRead = w_bytesToWrite;

        t_ImagePrms.t_SectPrms.p_SectionPtr =
                    t_ImagePrms.t_SectPrms.p_SectionPtr + (w_bytesRead/4U);

        t_ImagePrms.w_ImageSizeRead = t_ImagePrms.w_ImageSizeRead + w_bytesRead;

    }

    if (t_ImagePrms.t_SectPrms.w_WrittenLen ==
                                    t_ImagePrms.t_SectPrms.w_SecLen)
    {
        t_ImagePrms.w_NumSectionsWritten =
                                t_ImagePrms.w_NumSectionsWritten + 1U;

        if (t_ImagePrms.w_NumSections >
                                    t_ImagePrms.w_NumSectionsWritten)
        {
            t_ImagePrms.c_RprcState =
                                            M_PARSER_RPRC_PARSER_SECTION_HEADER;
            p_memAddRet = memset((void*)&t_ImagePrms.t_SectPrms, 0U,
                                 sizeof(T_PARSER_SECT_PRMS));
            DebugP_assert(p_memAddRet == (void*)&t_ImagePrms.t_SectPrms);
        }
        else
        {
            if (t_ImagePrms.w_ImageSizeRead != t_ImagePrms.w_FileSize)
            {
                M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_FILE_LENGTH_MISMATCH);
                w_bytesRead = w_bufflen;
            }
            else
            {
                if ((((t_ImagePrms.w_FileType & 0xFFFF0000U) >> 16U) ==
                                                    M_PARSER_CONFIG_SUBSYSTEM))
                {
                    w_size = parser_applyConfigFile(
                             (uint32_t*)&t_CfgFilePrms.c_ConfigFile[0U][0U],
                             t_CfgFilePrms.w_ConfigFileSize[0U]);

                    DebugP_assert(w_size == t_CfgFilePrms.w_ConfigFileSize[0U]);
                    t_CfgFilePrms.c_IsConfigApplied[0U] = 1U;
                }
                else
                {
                    for (c_cnt = 1U; c_cnt < M_PARSER_MAX_NUM_CONFIG_FILE; c_cnt++)
                    {
                        if ((t_CfgFilePrms.c_IsConfigValid[c_cnt] == 1U) && \
                            (t_CfgFilePrms.c_IsConfigApplied[c_cnt] != 1U))
                        {
                             w_size =
                             parser_applyConfigFile(
                             (uint32_t*)&t_CfgFilePrms.c_ConfigFile[c_cnt][0U],
                             t_CfgFilePrms.w_ConfigFileSize[c_cnt]);

                            DebugP_assert(w_size == t_CfgFilePrms.w_ConfigFileSize[c_cnt]);
                            t_CfgFilePrms.c_IsConfigApplied[c_cnt] = 1U;
                            break;
                        }
                    }
                }

                if (t_ImagePrms.w_NumAlignBytes != 0U)
                {
                    t_ImagePrms.c_RprcState =
                                        M_PARSER_RPRC_PARSER_ALIGNMENT_BYTES;
                }
                else
                {
                    t_ImagePrms.c_RprcState =
                                                M_PARSER_RPRC_PARSER_STATE_IDLE;
                    p_memAddRet = memset((void*)&t_ImagePrms, 0U,
                                         sizeof(T_PARSER_IMAGE_PRMS));

                    DebugP_assert(p_memAddRet == (void*)&t_ImagePrms);
                }
            }
        }

    }
    return (w_bytesRead);
}

/*!*****************************************************************************
 * @fn parser_readAlignmentBytes
 * @brief read Alignment bytes
 *
 * @b Description @n
 * Function for Reading the alignment bytes
 *
 * @param[in] p_readPtr -   pointer to the buffer to be read
 * @param[in] w_bufflen -   buffer length to be read
 *
 * @return number of bytes read
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
uint32_t parser_readAlignmentBytes(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_bytesRead = 0U;
    void *p_memAddRet;

    if (w_bufflen >= t_ImagePrms.w_NumAlignBytes)
    {
        p_readPtr = p_readPtr + t_ImagePrms.w_NumAlignBytes;
        w_bytesRead = t_ImagePrms.w_NumAlignBytes;
        t_ImagePrms.c_RprcState = M_PARSER_RPRC_PARSER_STATE_IDLE;

        p_memAddRet = memset((void*)&t_ImagePrms, 0U,
                             sizeof(T_PARSER_IMAGE_PRMS));

        DebugP_assert(p_memAddRet == (void*)&t_ImagePrms);
    }
    else
    {
        p_readPtr = p_readPtr + w_bufflen;
        w_bytesRead = w_bufflen;
        t_ImagePrms.w_NumAlignBytes =
                            t_ImagePrms.w_NumAlignBytes - w_bufflen;
        t_ImagePrms.c_RprcState = M_PARSER_RPRC_PARSER_ALIGNMENT_BYTES;
    }

    return (w_bytesRead);
}

/*!*****************************************************************************
 * @fn parser_applyConfigFile
 * @brief Applying the Configuration file
 *
 * @b Description @n
 * Function for Apply the Configuration file
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return number of bytes parsed
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
uint32_t parser_applyConfigFile(uint32_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t  w_remainingBytes = w_bufflen;
    uint32_t* p_writeAddr;
    uint32_t  w_writeValue;

    do {
        p_writeAddr = (uint32_t*)*p_readPtr;
        p_readPtr = p_readPtr + 1U;
        w_writeValue = *p_readPtr;
        p_readPtr = p_readPtr + 1U;

        *p_writeAddr = w_writeValue;
        w_remainingBytes = w_remainingBytes - 8U;
    }
    while (w_remainingBytes != 0U);

    return(w_bufflen);
}

/*!*****************************************************************************
 * @fn parser_calculateImageOffset
 * @brief Calculate offset for a section
 *
 * @b Description @n
 * Function for calculating the offset for Sections from APP Core point of view
 *
 * @param[in] w_sectionPtr  -   pointer to the section start
 * @param[in] w_sectionLen  -   Section Length
 *
 * @return Offset for the section
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
uint32_t parser_calculateImageOffset(uint32_t w_sectionPtr, uint32_t w_sectionLen)
{
    uint32_t w_filetypeVal = t_ImagePrms.w_FileType;
    uint32_t w_offset;

    if (((w_filetypeVal & 0xFFFF0000U) >> 16U) == M_PARSER_APP_SUBSYSTEM)
    {
        /*!
         * Check if the Image boot mode is eclipse or append
         */
        if (t_Metaheader.w_ImageBootMode == 1U)
        {
            if ((w_sectionPtr + w_sectionLen) <= M_PARSER_APP_ECLIPSE_REG1_END)
            {
                w_offset = M_PARSER_APP_ECLIPSE_REG1_OFFSET;
            }
            else if ((w_sectionPtr >= M_PARSER_APP_ECLIPSE_REG2_START) &&
                     ((w_sectionPtr + w_sectionLen) <= M_PARSER_APP_ECLIPSE_REG2_END))
            {
                w_offset = M_PARSER_APP_ECLIPSE_REG2_OFFSET;
            }
            else
            {
                w_offset = 0U;
                M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_APP_FILE_OFFSET_MISMATCH);
            }
        }
        else
        {
            if ((w_sectionPtr >= M_PARSER_APP_APPEND_REG1_START) &&
                ((w_sectionPtr + w_sectionLen) <= M_PARSER_APP_APPEND_REG1_END))
            {
                w_offset = M_PARSER_APP_APPEND_REG1_OFFSET;
            }
            else
            {
                w_offset = 0U;
                M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_APP_FILE_OFFSET_MISMATCH);
            }
        }
    }
    else if (((w_filetypeVal & 0xFFFF0000U) >> 16U) == M_PARSER_FEC_SUBSYSTEM)
    {
        if (sblObj.fecEclipseMode == 1U)
        {
            if (((w_sectionPtr + w_sectionLen) <= M_PARSER_FEC_ECLIPSE_REG1_END)
                && (sblObj.fecShRamPresent == 1))
            {
                w_offset = M_PARSER_FEC_ECLIPSE_REG1_OFFSET;
            }
            else if ((w_sectionPtr >= M_PARSER_FEC_ECLIPSE_REG2_START) &&
                        ((w_sectionPtr + w_sectionLen) <= M_PARSER_FEC_ECLIPSE_REG2_END))
            {
                w_offset = M_PARSER_FEC_ECLIPSE_REG2_OFFSET;
            }
            else
            {
                w_offset = 0U;
                M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_FEC_FILE_OFFSET_MISMATCH);
            }
        }
        else
        {
            if ((w_sectionPtr >= M_PARSER_FEC_APPEND_REG1_START) &&
                ((w_sectionPtr + w_sectionLen) <= M_PARSER_FEC_APPEND_REG1_END))
            {
                w_offset = M_PARSER_FEC_APPEND_REG1_OFFSET;
            }
           else if ((w_sectionPtr >= M_PARSER_FEC_APPEND_REG2_START) &&
                ((w_sectionPtr + w_sectionLen) <= M_PARSER_FEC_APPEND_REG2_END)
                && (sblObj.fecShRamPresent == 1))
            {
                w_offset = M_PARSER_FEC_APPEND_REG2_OFFSET;
            }
            else
            {
                w_offset = 0U;
                M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_FEC_FILE_OFFSET_MISMATCH);
            }
        }
    }
    else if (((w_filetypeVal & 0xFFFF0000U) >> 16U) == M_PARSER_CONFIG_SUBSYSTEM)
    {
        if ((w_sectionPtr == 0U) && (w_sectionLen <= M_PARSER_CONFIG_FILE_SIZE))
        {
            if (t_CfgFilePrms.w_NumCfgFiles < M_PARSER_MAX_NUM_CONFIG_FILE)
            {
                w_offset =
                    (uint32_t)&t_CfgFilePrms.c_ConfigFile[t_CfgFilePrms.w_NumCfgFiles][0U];
                t_CfgFilePrms.w_ConfigFileSize[t_CfgFilePrms.w_NumCfgFiles] = w_sectionLen;
                t_CfgFilePrms.c_IsConfigValid[t_CfgFilePrms.w_NumCfgFiles] = 1U;
                t_CfgFilePrms.w_NumCfgFiles =
                                t_CfgFilePrms.w_NumCfgFiles + 1U;
            }
            else
            {
                /*! Can't accomodate another config file */
                w_offset = 0U;
            }
        }
        else
        {
            w_offset = 0U;
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_CONFIG_FILE_OFFSET_MISMATCH);
        }
    }
    else
    {
        w_offset = 0U;
        M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_INVALID_FIELD);
    }

    return(w_offset);
}
/*
* END OF parser_image.c
*/
