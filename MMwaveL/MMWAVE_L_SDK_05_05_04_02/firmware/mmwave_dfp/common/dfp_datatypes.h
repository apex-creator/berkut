/*!*****************************************************************************
 * @file dfp_datatypes.h
 *
 * @brief Declares basic data types for DFP FECSS (FEC) and mmWaveLink (RL).
 *
 * @note
 * <B> © Copyright 2022, Texas Instruments Incorporated – www.ti.com </B>
 * @n
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * @n
 * <ul>
 *   <li> Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 * @n
 *   <li> Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * @n
 *   <li> Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 * </ul>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
 * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
 * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @addtogroup MMWAVE_DFP_DATA mmWave DFP Standards Data Types
 * @{
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     13Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef DFP_DATATYPE_H
#define DFP_DATATYPE_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */

/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */

/*!
 * @name Standards C data types typedefs
 * @note The bit widths are def
 * @{
 */
typedef unsigned char          UINT8;
typedef unsigned short         UINT16;
typedef unsigned int           UINT32;
typedef unsigned long long     UINT64;
typedef signed long long       SINT64;
typedef signed char            SINT8;
typedef signed short           SINT16;
typedef signed int             SINT32;
typedef float                  FLOAT32;
typedef double                 DOUBLE64;
typedef volatile UINT64        REG64;
typedef volatile UINT32        REG32;
typedef volatile UINT16        REG16;
typedef volatile UINT8         REG8;

/*! @} */

/*!
 * @name Standards custom data types
 * @{
 */


/*! @} */
/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name Standards MACROS
 * @{
 */

/*!
 * @brief Standard MACROs
 */
#define M_NULL                 (0)
#define M_NULL_PTR             ((void*)0)
#define M_INVALID_BYTE         ((UINT8)0xFFU)
#define M_INVALID_HWORD        ((UINT16)0xFFFFU)
#define M_INVALID_WORD         ((UINT32)0xFFFFFFFFU)
#define M_TRUE                 ((UINT8)1U)
#define M_FALSE                ((UINT8)0U)
#define M_LOGICAL_TRUE         (M_TRUE != M_FALSE)
#define M_LOGICAL_FALSE        (M_TRUE == M_FALSE)
#define M_MAX_WORD_VALUE       ((UINT32)0xFFFFFFFFU)
#define M_MAX_HWORD_VALUE      ((UINT16)0xFFFFU)
#define M_MAX_BYTE_VALUE       ((UINT8)0xFFU)
#define M_MAX_SINT32_VALUE     ((SINT32)0x7FFFFFFF)
#define M_MIN_SINT32_VALUE     ((SINT32)0x80000000)
#define M_MAX_UINT32_VALUE     ((UINT32)0xFFFFFFFF)
#define M_ENABLE               ((UINT8)1U)
#define M_DISABLE              ((UINT8)0U)

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*! @} */
#endif
/*
 * END OF DFP_DATATYPE_H
 */

