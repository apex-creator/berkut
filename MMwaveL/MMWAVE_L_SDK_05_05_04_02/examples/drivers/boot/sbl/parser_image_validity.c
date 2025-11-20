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
#include "parser_image_validity.h"
#include "parser.h"
#include "parser_image.h"
#include "app_errors.h"
#include "sbl.h"

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
T_PARSER_IMAGE_VALID_CTX t_ImgValidCtx;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @fn parser_imageValidParamsInit
 * @brief Initialize Image Validation parameters
 *
 * @b Description @n
 * This function Initializes the structure used for Image Validation
 *
 * @param[in] Non
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
void parser_imageValidParamsInit(void)
{
    t_ImgValidCtx.w_ValidSize = 0U;
    t_ImgValidCtx.w_ValidSizeRead = 0U;
    t_ImgValidCtx.w_NumAlignBytes = 0U;
    t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_START;
    t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_FIRST;
    t_ImgValidCtx.c_NoFileIncr = 0U;
    t_ImgValidCtx.c_NumFilesWritten = 0U;
}

/*!*****************************************************************************
 * @fn parser_checkImageValidity
 * @brief State machine for Validity process
 *
 * @b Description @n
 * This function implements the state machine for Image Validatio process
 *
 * @param[in] p_readPtr -   pointer to the buffer to be parsed
 * @param[in] w_bufflen -   buffer length to be parsed
 *
 * @return 0    - Success
 *         -1   - Failure
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
int32_t parser_checkImageValidity(uint8_t* p_readPtr, uint32_t w_bufflen)
{
    uint32_t w_remainingBytes = w_bufflen;
    uint32_t w_bytesRead = 0U;
    int32_t xw_status;

    do {
        switch (t_ImgValidCtx.c_ValidState)
        {
            case M_PARSER_VALID_STATE_START:
                t_ImgValidCtx.c_NumFilesWritten =
                        t_ImgValidCtx.c_NumFilesWritten + 1U;
                t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_FIRST;
                t_ImgValidCtx.w_ValidSize =
                t_Metaheader.t_ImageDetails[t_ImgValidCtx.c_NumFilesWritten -1U].w_FileSize;
                t_ImgValidCtx.w_ValidSizeRead = 0U;

                if ((t_ImgValidCtx.w_ValidSize % 64U) != 0U)
                {
                    t_ImgValidCtx.w_NumAlignBytes = (64U -
                                            (t_ImgValidCtx.w_ValidSize % 64U));
                }
                else
                {
                    t_ImgValidCtx.w_NumAlignBytes = 0U;
                }

                t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_PROCESS;
                t_ImgValidCtx.c_NoFileIncr = 0U;
                if (w_remainingBytes < M_PARSER_RPRC_HDR_LENGTH)
                {
                    t_ImgValidCtx.c_NoFileIncr = 1U;
                }
                xw_status = 0;
                break;

            case M_PARSER_VALID_STATE_PROCESS:
                w_bytesRead = parser_computeValidLength(w_remainingBytes);
                xw_status = parser_imageValidation(p_readPtr, w_bytesRead,
                t_ImgValidCtx.c_NumFilesWritten, t_ImgValidCtx.c_ValidBlkType);
                w_remainingBytes = w_remainingBytes - w_bytesRead;
                p_readPtr = p_readPtr + w_bytesRead;
                t_ImgValidCtx.w_ValidSizeRead = t_ImgValidCtx.w_ValidSizeRead +
                                                w_bytesRead;
                break;

            case M_PARSER_VALID_STATE_ALIGN:
                if (t_ImgValidCtx.w_NumAlignBytes != 0U)
                {
                    if (w_remainingBytes >= t_ImgValidCtx.w_NumAlignBytes)
                    {
                        w_bytesRead = t_ImgValidCtx.w_NumAlignBytes;
                        t_ImgValidCtx.w_NumAlignBytes = 0U;
                        t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_START;
                    }
                    else
                    {
                        w_bytesRead = w_remainingBytes;
                        t_ImgValidCtx.w_NumAlignBytes =
                                    t_ImgValidCtx.w_NumAlignBytes - w_bytesRead;
                        t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_ALIGN;
                    }
                    w_remainingBytes = w_remainingBytes - w_bytesRead;
                    p_readPtr = p_readPtr + w_bytesRead;
                }
                else
                {
                    t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_START;
                }
                xw_status = 0;
                break;
            default:
                xw_status = -1;
                break;
        }
    } while (w_remainingBytes != 0U);

    return(xw_status);
}


/*!*****************************************************************************
 * @fn parser_computeValidLength
 * @brief Computes the length to be validated
 *
 * @b Description @n
 * This function computes length of the buffer, which is to be validated
 *
 * @param[in] w_remainingBytes  -   Number of remaining bytes in buffer
 *
 * @return validation length
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
uint32_t parser_computeValidLength(uint32_t w_remainingBytes)
{
    uint32_t w_retLength;

    if (t_ImgValidCtx.w_ValidSizeRead == 0U)
    {
        if (w_remainingBytes >= t_ImgValidCtx.w_ValidSize)
        {
            w_retLength = t_ImgValidCtx.w_ValidSize;
        }
        else
        {
            w_retLength = w_remainingBytes;
        }

        if (w_retLength == t_ImgValidCtx.w_ValidSize)
        {
            t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_ALIGN;
            t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_FULL;
        }
        else
        {
            t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_PROCESS;
            t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_FIRST;
        }
    }
    else if (t_ImgValidCtx.w_ValidSize >
                         (w_remainingBytes + t_ImgValidCtx.w_ValidSizeRead))
    {
        w_retLength = w_remainingBytes;
        t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_PROCESS;
        t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_INTERMEDIATE;
    }
    else
    {
        w_retLength = t_ImgValidCtx.w_ValidSize - t_ImgValidCtx.w_ValidSizeRead;
        t_ImgValidCtx.c_ValidState = M_PARSER_VALID_STATE_ALIGN;
        t_ImgValidCtx.c_ValidBlkType = M_PARSER_CRC_DATA_BLK_FINAL;
    }

    return(w_retLength);
}

/*!*****************************************************************************
 * @fn parser_imageValidation
 * @brief state machine for CRC validation
 *
 * @b Description @n
 * This function implements the CRC Validation state machine
 *
 * @param[in] p_readPtr  -   pointer to the buffer to be valdiated
 * @param[in] w_bufflen  -   buffer length to be validated
 * @param[in] c_FileType -   File Id
 * @param[in] c_ValidBlkType - Data Block Type ( FIRST, Full, Intermediate, Final)
 *
 * @return  0   -   Success
 *         -1   -   Failure
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
int32_t parser_imageValidation(uint8_t* p_readPtr, uint32_t w_bufflen,
                              uint8_t c_FileType, uint8_t c_ValidBlkType)
{
    uint32_t w_crcComputed, w_crcExpected;
    int32_t xw_status;

    if (c_ValidBlkType == M_PARSER_CRC_DATA_BLK_FULL)
    {
        sbl_computeCrc32Cpu((uint32_t*)p_readPtr,w_bufflen,&w_crcComputed);
        w_crcExpected = t_Metaheader.t_ImageDetails[t_ImgValidCtx.c_NumFilesWritten - 1U].w_FileCrc;

        xw_status = (w_crcComputed == w_crcExpected)? 0 : -1;
    }
    else if (c_ValidBlkType == M_PARSER_CRC_DATA_BLK_FIRST)
    {
        sbl_configureCrc32Cpu();
        sbl_sendDataCrc32Cpu((uint32_t*)p_readPtr,w_bufflen);

        xw_status = 0U;
    }
    else if (c_ValidBlkType == M_PARSER_CRC_DATA_BLK_INTERMEDIATE)
    {
        sbl_sendDataCrc32Cpu((uint32_t*)p_readPtr,w_bufflen);
        xw_status = 0U;
    }
    else if (c_ValidBlkType == M_PARSER_CRC_DATA_BLK_FINAL)
    {
        sbl_sendDataCrc32Cpu((uint32_t*)p_readPtr,w_bufflen);
        w_crcComputed = sbl_getCrc32Cpu();
        w_crcExpected = t_Metaheader.t_ImageDetails[t_ImgValidCtx.c_NumFilesWritten - 1U].w_FileCrc;

        xw_status = (w_crcComputed == w_crcExpected)? 0 : -1;
    }
    else
    {
        xw_status = -1;
    }

    if (xw_status != 0)
    {
        if (t_ImgValidCtx.c_NumFilesWritten == 1U)
        {
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_IMG1_AUTH_FAILURE);
        }
        else if (t_ImgValidCtx.c_NumFilesWritten == 2U)
        {
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_IMG2_AUTH_FAILURE);
        }
        else if (t_ImgValidCtx.c_NumFilesWritten == 3U)
        {
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_IMG3_AUTH_FAILURE);
        }
        else if (t_ImgValidCtx.c_NumFilesWritten == 4U)
        {
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_IMG4_AUTH_FAILURE);
        }
        else
        {
            M_PARSER_SET_ERROR_STATUS(M_PARSER_RPRC_IMG1_AUTH_FAILURE);
        }
    }

    return(xw_status);
}

/*
* END OF parser_image_validity.c
*/
