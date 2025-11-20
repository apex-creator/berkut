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
#ifndef PARSER_IMAGE_H
#define PARSER_IMAGE_H

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
/*!
 * @brief SMacros for configuration files
 */
#define M_PARSER_CONFIG_FILE_SIZE       (256U)  /**< Config file size max length in bytes */
#define M_PARSER_MAX_NUM_CONFIG_FILE    (4U)     /**< Max no of config files */

/*!
 * @brief States for Section parsing
 */
#define M_PARSER_SEC_STATE_START_ADDR   (0U)
#define M_PARSER_SEC_STATE_SEC_LEN      (1U)
#define M_PARSER_SEC_STATE_RSVD         (2U)
#define M_PARSER_SEC_STATE_HDR_END      (3U)

/*!
 * @brief States for RPRC header parsing
 */
#define M_PARSER_RPRC_HDR_STATE_MAGIC   (0U)
#define M_PARSER_RPRC_HDR_STATE_LEN     (1U)
#define M_PARSER_RPRC_HDR_STATE_RSVD    (2U)
#define M_PARSER_RPRC_HDR_STATE_END     (3U)

/*!
 * @brief States for RPRC Image parsing
 */
#define M_PARSER_RPRC_PARSER_STATE_IDLE         (0U)
#define M_PARSER_RPRC_PARSER_SECTION_HEADER     (1U)
#define M_PARSER_RPRC_PARSER_SECTION_DOWNLOAD   (2U)
#define M_PARSER_RPRC_PARSER_ALIGNMENT_BYTES    (3U)

/*!
 * @brief RPRC Header lengths
 */
#define M_PARSER_RPRC_HDR_LENGTH     (24U)
#define M_PARSER_RPRC_SEC_HDR_LENGTH (24U)

/*!
 * @brief RPRC Magic word
 */
#define M_PARSER_RPRC_MAGIC_WORD     (0x43525052U)

/*!
 * @brief Eclipse mode regions for APP core
 */
#define M_PARSER_APP_ECLIPSE_REG1_END       (0x40000U)
#define M_PARSER_APP_ECLIPSE_REG2_START     (0x440000U)
#define M_PARSER_APP_ECLIPSE_REG2_END       (0x4C0000U)
#define M_PARSER_APP_ECLIPSE_REG1_OFFSET    (0x400000U)
#define M_PARSER_APP_ECLIPSE_REG2_OFFSET    (0x0U)

/*!
 * @brief Append mode regions for APP core
 */
#define M_PARSER_APP_APPEND_REG1_START      (0x400000U)
#define M_PARSER_APP_APPEND_REG1_END        (0x4C0000U)
#define M_PARSER_APP_APPEND_REG1_OFFSET     (0x0U)

/*!
 * @brief Eclipse mode regions for FEC core
 */
#define M_PARSER_FEC_ECLIPSE_REG1_END       (0x18000U)
#define M_PARSER_FEC_ECLIPSE_REG1_OFFSET    (0x21208000U)
#define M_PARSER_FEC_ECLIPSE_REG2_START     (0x200000U)
#define M_PARSER_FEC_ECLIPSE_REG2_END       (0x208000U)
#define M_PARSER_FEC_ECLIPSE_REG2_OFFSET    (0x21000000U)


/*!
 * @brief Append mode regions for FEC core
 */
#define M_PARSER_FEC_APPEND_REG1_START      (0x200000U)
#define M_PARSER_FEC_APPEND_REG1_END        (0x208000U)
#define M_PARSER_FEC_APPEND_REG2_START      (0x208000U)
#define M_PARSER_FEC_APPEND_REG2_END        (0x220000U)
#define M_PARSER_FEC_APPEND_REG1_OFFSET     (0x21000000U)
#define M_PARSER_FEC_APPEND_REG2_OFFSET     (0x21000000U)
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*!
 * @brief structure containing Configuration file parameters
 */
typedef struct
{
    uint8_t  c_ConfigFile[M_PARSER_MAX_NUM_CONFIG_FILE][M_PARSER_CONFIG_FILE_SIZE];
    uint32_t w_ConfigFileSize[M_PARSER_MAX_NUM_CONFIG_FILE];
    uint8_t  c_IsConfigValid[M_PARSER_MAX_NUM_CONFIG_FILE];
    uint8_t  c_IsConfigApplied[M_PARSER_MAX_NUM_CONFIG_FILE];
    uint32_t w_NumCfgFiles;
}T_CONFIG_FILE_PRMS;

/*!
 * @brief structure containing section parser parameters
 */
typedef struct
{
    uint64_t  dw_SecStrtAddr;       /**< section start address         */
    uint32_t* p_SectionPtr;         /**< section current address       */
    uint32_t  w_SecLen;             /**< section  length               */
    uint32_t  w_WrittenLen;         /**< Length of the section written */
    uint8_t   c_SectState;          /**< section parser state          */
}T_PARSER_SECT_PRMS;

/*!
 * @brief structure containing image parser parameters
 */
typedef struct
{
    uint32_t w_NumSections;               /**< Number of Sections              */
    uint32_t w_NumSectionsWritten;        /**< Number of sections written      */
    uint32_t w_ImageSizeRead;             /**< Length of Image read in Bytes   */
    uint32_t w_FileSize;                  /**< Size of the image               */
    uint32_t w_FileType;                  /**< image type                      */
    uint32_t w_NumAlignBytes;             /**< Number of alignment bytes       */
    T_PARSER_SECT_PRMS t_SectPrms;      /**< section parser parameters       */
    uint8_t  c_RprcState;                 /**< State of the RPRC parser        */
    uint8_t  c_HdrState;                  /**< State of the RPRC Header parser */
}T_PARSER_IMAGE_PRMS;

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */
extern T_PARSER_IMAGE_PRMS t_ImagePrms;
extern T_CONFIG_FILE_PRMS  t_CfgFilePrms;
/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
uint32_t parser_parseRprcImage(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_parseRprcHeader(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_parseRprcSecHeader(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_parseRprcSecContent(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_readAlignmentBytes(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_applyConfigFile(uint32_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_calculateImageOffset(uint32_t w_sectionPtr, uint32_t w_sectionLen);
#ifdef __cplusplus
}
#endif

#endif
/*
 * END OF parser_image.h
 */
