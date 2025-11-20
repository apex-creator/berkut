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
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef PARSER_H
#define PARSER_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
#define M_READ_BUFFER_SIZE      (2048U)
#define M_PING_BUFFER_INDEX     (0U)
#define M_PONG_BUFFER_INDEX     (1U)
#define M_PART_BUFFER_INDEX     (2U) /*! Used only for fast SPI download */

/*!
 * @brief Interface for parser
 */
#define M_PARSER_INTERFACE_QSPI         (0U)
#define M_PARSER_INTERFACE_SPI_NORMAL   (1U)
#define M_PARSER_INTERFACE_SPI_FAST     (2U)
#define M_PARSER_INTERFACE_UART         (3U)

/*!
 * @brief Parser States
 */
#define M_PARSER_BUFFER_PARSER_STATE_IDLE   (0U)
#define M_PARSER_BUFFER_PARSER_META_HEADER  (1U)
#define M_PARSER_BUFFER_PARSER_CERT_DWLD    (2U)
#define M_PARSER_BUFFER_PARSER_RPRC_DWLD    (3U)
#define M_PARSER_BUFFER_PARSER_COMPLETED    (4U)
#define M_PARSER_BUFFER_PARSER_APP_SWITCH   (5U)

/*!
 * @brief Maximum number files supported by the parser
 */
#define M_PARSER_NUM_FILES_SUPPORTED        (5U)

/*!
 * @brief Metaheader start word
 */
#define M_PARSER_META_HDR_START             (0x5254534DU)

/*!
 * @brief Metaheader End word
 */
#define M_PARSER_META_HDR_END               (0x444E454DU)

/*!
 * @brief APP subsystem identifier.
 */
#define M_PARSER_APP_SUBSYSTEM              (0x3551U)

/*!
* @brief FEC subsystem identifier.
*/
#define M_PARSER_FEC_SUBSYSTEM              (0xB551U)

/*!
 * @brief CONFIG subsystem identifier.
 */
#define M_PARSER_CONFIG_SUBSYSTEM           (0xCF91U)

/*! Macro definitions for ping/pong based parsing */
#define M_PARSER_SPI_BUFFER_PARSER_SIZE             (M_READ_BUFFER_SIZE)
#define M_PARSER_BUFFER_EMPTY                       (0U)
#define M_PARSER_BUFFER_FILLING                     (1U)
#define M_PARSER_BUFFER_FULL                        (2U)

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*!
 * @brief structure containing image details
 */
typedef struct
{
    uint32_t w_MagicWord;       /**< magic word of image      */
    uint32_t w_FileOffset;      /**< offset of the magic word */
    uint32_t w_FileSize;        /**< Image size               */
    uint32_t w_FileCrc;         /**< Image CRC                */
    uint32_t w_Reserved;        /**< Reserved                 */
    uint8_t  c_FilePresent;     /**< Image valid flag         */
    uint8_t  c_FileType;        /**< Image id                 */
}T_IMAGE_DETAILS;

/*!
 * @brief structure containing all the metaheader details
 */
typedef struct
{
    uint32_t          w_NumFiles;                            /**< Num of files present       */
    uint32_t          w_ImageBootMode;                       /**< Boot Mode (eclipse/append) */
    uint32_t          w_HdrCrc;                              /**< Metaheader CRC             */
    uint32_t          w_BootVector;                          /**< Boot Vector                     */
    uint32_t          w_ImageSize;                           /**< Multicore image size       */
    uint32_t          w_ShMemAlloc;                          /**< shared memory allocation   */
    T_IMAGE_DETAILS t_ImageDetails[M_PARSER_NUM_FILES_SUPPORTED]; /**< All image details          */
}T_METAHEADER;

/*!
 * @brief structure for parsing context
 */
typedef struct
{
    uint8_t c_ParserState;
    uint8_t c_MetaHeaderInvalid;
    uint8_t c_AppImagePresent;
    uint8_t c_NumRprcFilesParsed;
}T_PARSER_CONTEXT;

/*!
 * @brief Buffer parser task context
 */
typedef struct
{
    uint8_t* p_ParseBuff;
    uint32_t w_Bufflen;
    uint8_t  c_IsLastChunk;
    uint8_t  c_IntfType;
}T_BUFFER_PARSER_TASK_CTX;

/*!
 * @brief Ping/Pong buffer params
 */
typedef struct
{
    volatile uint8_t  c_BufferState;
    uint32_t w_CurrBufferIdx;
}T_PARSER_BUFFER_PARAMS;

/*!
 * @brief Ping/Pong buffers context
 */
typedef struct
{
    uint8_t                   c_WriteBuffIdx;
    uint8_t                   c_ParseBuffIdx;
    T_PARSER_BUFFER_PARAMS  t_BufferParams[2U];
    uint8_t                   c_IsPartChunkTrigPending;
}T_PARSER_PING_PONG_BUFFER_CTX;

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */
extern T_METAHEADER        t_Metaheader;
extern T_PARSER_CONTEXT    t_ParserCtx;
extern uint8_t c_ReadBuffer[3U][M_READ_BUFFER_SIZE];

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
void parser_init(void);
void parser_buffMgmt(uint8_t* p_parseBuff, uint32_t w_bufflen, uint8_t c_islastBuff,
                     uint8_t c_intfType);
void parser_buffer(uint8_t* p_parseBuff, uint32_t w_bufflen);
uint32_t parser_parseMetaHeader(uint8_t* p_readPtr);
uint32_t parser_parseRprcImage(uint8_t* p_readPtr, uint32_t w_bufflen);

#ifdef __cplusplus
}
#endif

#endif
/*
 * END OF parser.h
 */
