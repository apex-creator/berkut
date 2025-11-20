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
#ifndef APP_ERRORS_H
#define APP_ERRORS_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <stdint.h>
#include "sbl.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @brief Bit map for various errors during boot
 * Bits         Description
 * 31:0         Meta image parser error codes
 */
#define M_PARSER_RPRC_IMG1_AUTH_FAILURE             ((uint64_t)(0x0000000000000001U))  /* BIT0  */
#define M_PARSER_RPRC_IMG2_AUTH_FAILURE             ((uint64_t)(0x0000000000000002U))  /* BIT1  */
#define M_PARSER_RPRC_IMG3_AUTH_FAILURE             ((uint64_t)(0x0000000000000004U))  /* BIT2  */
#define M_PARSER_RPRC_IMG4_AUTH_FAILURE             ((uint64_t)(0x0000000000000008U))  /* BIT3  */
#define M_PARSER_RPRC_HDR_NOT_FOUND                 ((uint64_t)(0x0000000000000010U))  /* BIT4  */
#define M_PARSER_METAHEADER_NOT_FOUND               ((uint64_t)(0x0000000000000020U))  /* BIT5  */
#define M_PARSER_RPRC_FILE_LENGTH_MISMATCH          ((uint64_t)(0x0000000000000040U))  /* BIT6  */
#define M_PARSER_RPRC_APP_FILE_OFFSET_MISMATCH      ((uint64_t)(0x0000000000000080U))  /* BIT7  */
#define M_PARSER_RPRC_FEC_FILE_OFFSET_MISMATCH      ((uint64_t)(0x0000000000000100U))  /* BIT8  */
#define M_PARSER_RPRC_INVALID_FIELD                 ((uint64_t)(0x0000000000000200U))  /* BIT9  */
#define M_PARSER_MSSIMAGE_NOT_FOUND                 ((uint64_t)(0x0000000000000400U))  /* BIT10 */
#define M_PARSER_METAHEADER_NUMFILES_ERROR          ((uint64_t)(0x0000000000000800U))  /* BIT11 */
#define M_PARSER_METAHEADER_CRC_FAILURE             ((uint64_t)(0x0000000000001000U))  /* BIT12 */
#define M_PARSER_RPRC_CONFIG_FILE_OFFSET_MISMATCH   ((uint64_t)(0x0000000000002000U))  /* BIT13 */
#define M_PARSER_QSPI_READ_TIME_OUT                 ((uint64_t)(0x0000000000004000U))  /* BIT14 */

/*!
 * @brief Macro for checking Parser Error
 */
#define M_PARSER_CHECK_ERROR_STATUS   ((sblObj.errorStatus & \
                                                 0x00000000FFFFFFFFU) != 0U)
/*!
 * @brief Macro for checking specific Parser Error
 */
#define M_PARSER_CHECK_ERROR(x)     ((sblObj.errorStatus & x) != 0U)

/*!
 * @brief Macro for setting specific Parser Error
 */
#define M_PARSER_SET_ERROR_STATUS(x)  (sblObj.errorStatus |= x)

/*!
 * @brief Macro for setting specific error
 */
#define M_BOOT_SET_ERROR_STATUS(x)  (sblObj.errorStatus |= x)

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */


#ifdef __cplusplus
}
#endif

#endif
/*
 * END OF app_errors.h
 */
