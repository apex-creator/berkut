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
#ifndef PARSER_IMAGE_VALIDITY_H
#define PARSER_IMAGE_VALIDITY_H

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
 * @brief Parser Validation Stages
 */
#define M_PARSER_VALID_STATE_START    (0U)    /*! Valid State start   */
#define M_PARSER_VALID_STATE_PROCESS  (1U)    /*! Valid state process */
#define M_PARSER_VALID_STATE_END      (2U)    /*! Valid state end     */
#define M_PARSER_VALID_STATE_ALIGN    (3U)    /*! Valid state align   */

/*!
 * @brief Parser CRC Data block stages
 */
#define M_PARSER_CRC_DATA_BLK_FIRST         (0x00U)
#define M_PARSER_CRC_DATA_BLK_INTERMEDIATE  (0x01U)
#define M_PARSER_CRC_DATA_BLK_FINAL         (0x02U)
#define M_PARSER_CRC_DATA_BLK_FULL          (0x03U)
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*!
 * @brief typedef for Image Validity context structure
 */
typedef struct
{
    uint32_t w_ValidSize;
    uint32_t w_ValidSizeRead;
    uint32_t w_NumAlignBytes;
    uint8_t  c_ValidState;
    uint8_t  c_ValidBlkType;
    uint8_t  c_NoFileIncr;
    uint8_t  c_NumFilesWritten;
} T_PARSER_IMAGE_VALID_CTX;

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
void parser_imageValidParamsInit(void);
int32_t parser_checkImageValidity(uint8_t* p_readPtr, uint32_t w_bufflen);
uint32_t parser_computeValidLength(uint32_t w_remainingBytes);
int32_t parser_imageValidation(uint8_t* p_readPtr, uint32_t w_bufflen,
                              uint8_t c_FileType, uint8_t c_ValidBlkType);
#ifdef __cplusplus
}
#endif

#endif
/*
 * END OF parser_image_validity.h
 */
