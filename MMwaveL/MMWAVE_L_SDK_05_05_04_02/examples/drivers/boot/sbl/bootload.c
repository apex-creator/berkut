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
#include "bootload.h"
#include "parser.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "board/flash.h"
#include "app_errors.h"

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */
/** Flash address mapping with the file types */
extern uint32_t w_FlashAddr[8U];
/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @fn bootload_switchBuffer
 * @brief Function to Swap buffers
 *
 * @b Description @n
 *  This is a utility function to swap 2 buffer pointers
 *
 * @param[in] pp_loadBuffer     - double pointer to load buffer
 * @param[in] pp_parsebuffer    - double pointer to parse buffer
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
void bootload_switchBuffer(uint8_t** pp_loadBuffer, uint8_t** pp_parseBuffer)
{
    /*!
     * load buffer will become parse buffer
     */
    *pp_parseBuffer = *pp_loadBuffer;

    if (*pp_loadBuffer == &c_ReadBuffer[M_PING_BUFFER_INDEX][0U])
    {
        *pp_loadBuffer = &c_ReadBuffer[M_PONG_BUFFER_INDEX][0U];
    }
    else
    {
        *pp_loadBuffer = &c_ReadBuffer[M_PING_BUFFER_INDEX][0U];
    }
}

/*!*****************************************************************************
 * @fn bootload_qspi
 * @brief QSPI Boot interface
 *
 * @b Description @n
 * This function tries to load the image from Flash interface
 *
 * @param[in] None
 *
 * @return success - M_BOOTINTF_QSPI_RET_SUCCESS
 *         failure - M_BOOTLOAD_QSPI_RET_SFDP_ERROR
 *                 - M_BOOTLOAD_QSPI_RET_IMAGE_ERROR
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
int32_t bootload_qspi(uint32_t w_metaimageOffset)
{
    uint8_t* p_loadBuff = NULL;
    uint8_t* p_parseBuff = NULL;
    uint32_t w_totalLength;
    uint32_t w_numBlocks;
    uint32_t w_remainingBytes;
    uint32_t w_loopCount;
    uint32_t w_index = 0U;
    int32_t xw_retVal;

    /*!
     * Initialize the parser
     */
    parser_init();

    /*!
     * point current buffer to the PING
     */
    p_loadBuff = &c_ReadBuffer[M_PING_BUFFER_INDEX][0U];

    /*!
     * Read the first Buffer packet
     */
    Flash_read(gFlashHandle[CONFIG_FLASH0], w_metaimageOffset + w_index, p_loadBuff, M_READ_BUFFER_SIZE);
    /*!
     * Since we need to get metaimage info, wait for the transfer to
     * complete before staring the next read
     */
    bootload_switchBuffer(&p_loadBuff, &p_parseBuff);
    /*!
     * Parse the first chunk
     */
    parser_buffMgmt(p_parseBuff, M_READ_BUFFER_SIZE, 0U, M_PARSER_INTERFACE_QSPI);
    /*!
     * Check if the Metaimage Header is present
     */
    if (t_ParserCtx.c_MetaHeaderInvalid == 1U)
    {
        xw_retVal = M_BOOTLOAD_QSPI_RET_IMAGE_ERROR;
    }
    else if (t_ParserCtx.c_ParserState == M_PARSER_BUFFER_PARSER_COMPLETED)
    {
        /*!
         * Check for Image Error
         */
        if (M_PARSER_CHECK_ERROR_STATUS)
        {
            xw_retVal = M_BOOTLOAD_QSPI_RET_IMAGE_ERROR;
        }
        else
        {
            /* less than 1 chunk case */
            xw_retVal = M_BOOTLOAD_QSPI_RET_SUCCESS;
        }
    }
    else
    {
        /*!
         * Get Metaimage info
         */
        w_totalLength = t_Metaheader.w_ImageSize;
        w_numBlocks = (w_totalLength / M_READ_BUFFER_SIZE);
        w_index += M_READ_BUFFER_SIZE;

        if (w_numBlocks > 1U)
        {
            /*!
             * Loop through all blocks
             */
            for (w_loopCount = 1U; w_loopCount < w_numBlocks; w_loopCount++)
            {
                if (w_loopCount == 1U)
                {
                    Flash_read(gFlashHandle[CONFIG_FLASH0], w_metaimageOffset + w_index, p_loadBuff, M_READ_BUFFER_SIZE);
                    bootload_switchBuffer(&p_loadBuff, &p_parseBuff);
                    w_index += M_READ_BUFFER_SIZE;
                }
                else
                {
                    Flash_read(gFlashHandle[CONFIG_FLASH0], w_metaimageOffset + w_index, p_loadBuff, M_READ_BUFFER_SIZE);
                    parser_buffMgmt(p_parseBuff, M_READ_BUFFER_SIZE, 0U, M_PARSER_INTERFACE_QSPI);
                    bootload_switchBuffer(&p_loadBuff, &p_parseBuff);
                    w_index += M_READ_BUFFER_SIZE;
                }
                if (t_ParserCtx.c_ParserState == M_PARSER_BUFFER_PARSER_COMPLETED)
                {
                    break;
                }
            }
            /*!
             * Parse the last full block
             */
            if (w_loopCount == w_numBlocks)
            {
                parser_buffMgmt(p_parseBuff, M_READ_BUFFER_SIZE, 0U,
                                M_PARSER_INTERFACE_QSPI);
            }
        }

        /*!
         * Get the remaining bytes (partial block)
         */
        w_remainingBytes = (w_totalLength - w_index);
        if ((w_remainingBytes != 0U) && (t_ParserCtx.c_ParserState != M_PARSER_BUFFER_PARSER_COMPLETED))
        {
            Flash_read(gFlashHandle[CONFIG_FLASH0], w_metaimageOffset + w_index, p_loadBuff, w_remainingBytes);
            bootload_switchBuffer(&p_loadBuff, &p_parseBuff);
            w_index += w_remainingBytes;
            parser_buffMgmt(p_parseBuff, w_remainingBytes, 1U, M_PARSER_INTERFACE_QSPI);
        }
        else
        {
            /*!
             * Call the interface with zero length to close interface
             */
            parser_buffMgmt(p_parseBuff, 0U, 1U, M_PARSER_INTERFACE_QSPI);
        }
        /*!
         * Check for any errors
         */
        if (M_PARSER_CHECK_ERROR_STATUS)
        {
            xw_retVal = M_BOOTLOAD_QSPI_RET_IMAGE_ERROR;
        }
        else
        {
            xw_retVal = M_BOOTLOAD_QSPI_RET_SUCCESS;
        }
    }

    return (xw_retVal);
}

/*
* END OF bootload.c
*/
