/*!*****************************************************************************
 * @file fe_registers.h
 *
 * @brief Register driver macros and functions for DFP
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
 * @addtogroup FECSSLIB_DEVICE_REGIF_MODULE Device Register Interface Driver functions
 * @{
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     07Mar2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_REGISTER_H
#define FE_REGISTER_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */

/*------------------------------------------------------------------------------
 * Read Macros
 *------------------------------------------------------------------------------
 */
/*!
 * @name FECSS Library Register Interface Driver module MACRO Functions
 * @{
 */

/*!
 * @name FECSS Library Register Interface Read Macro functions
 * @{
 */

/*!*****************************************************************************
 * @brief Read 32bit register value from address
 *
 * @b Description @n
 * This MACRO function reads 32bit register value from address.
 * SOC Device Type: Read register address using Pointer
 * FE Device Type: Read register address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_reg        - Address of the register
 * @param[out] w_regValue   - Register content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * REG32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT32 w_pmuCtrl;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG32_READ(c_devInd, p_mpu_ctrl_reg, w_pmuCtrl, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_REG32_READ(c_devInd, p_reg, w_regValue, xw_return) { \
    (w_regValue) = *((REG32*)(p_reg)); \
    (xw_return) = M_DFP_RET_CODE_OK; \
}
#else
#define M_REG32_READ(c_devInd, p_reg, w_regValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Read bit-field value from a register
 *
 * @b Description @n
 * This MACRO function reads bit-field value of a register from a address. Read
 * 32bit value using M_REG32_READ function and extract bit-field info using
 * structure type.
 *
 * @param[in]  c_devInd        - Device Index, applicable only for FE devices
 * @param[in]  p_reg           - Address of the register
 * @param[out] w_fieldValue    - Value of the bit-filed to be read
 * @param[in]  t_registerType  - Union type of the register (Typedef)
 * @param[in]  t_fieldName     - Bit-field name in structure
 * @param[out] xw_return       - Return status
 *
 * @b Example @n
 * @code
 * REG32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT8  c_enable;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG_STRUCT_FIELD_READ(c_devInd, p_mpu_ctrl_reg, c_enable, \
 *                              T_SYS_CORE_MPU_CTRL, b1_Enable, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#define M_REG_STRUCT_FIELD_READ(c_devInd, p_reg, w_fieldValue, \
                        t_registerType, t_fieldName, xw_return) { \
    t_registerType z_local; \
    M_REG32_READ(c_devInd, (p_reg), z_local.b32_Reg, (xw_return)); \
    (w_fieldValue) = z_local.bits.t_fieldName; \
}

/* Read macros */
/*! @} */

/*------------------------------------------------------------------------------
 * Register Write Macros
 *------------------------------------------------------------------------------
 */

/*!
 * @name FECSS Library Register Interface Write Macro functions
 * @{
 */

/*!*****************************************************************************
 * @brief Write 32bit register value to address
 *
 * @b Description @n
 * This MACRO function writes 32bit register value to address.
 * SOC Device Type: Write to register address using Pointer
 * FE Device Type: Write to register address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_reg        - Address of the register
 * @param[in]  w_regValue   - Register content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * REG32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT32 w_pmuCtrl = 0xC0FFEE4U;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG32_WRITE(c_devInd, p_mpu_ctrl_reg, w_pmuCtrl, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_REG32_WRITE(c_devInd, p_reg, w_regValue, xw_return) { \
    *((REG32*)(p_reg)) = (w_regValue); \
    (xw_return) = M_DFP_RET_CODE_OK; \
    M_DFP_LOG_ADDR_DATA(p_reg, w_regValue); \
}
#else
#define M_REG32_WRITE(c_devInd, p_reg, w_regValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Write to a bit-field of a register
 *
 * @b Description @n
 * This MACRO function writes a value to a bit-field of a register using address, data.
 * Read 32bit value using M_REG32_READ function and modify bit-field value using
 * structure type and write to register using M_REG32_WRITE.
 *
 * @param[in]  c_devInd        - Device Index, applicable only for FE devices
 * @param[in]  p_reg           - Address of the register
 * @param[in]  w_fieldValue    - Value of the bit-filed to be read
 * @param[in]  t_registerType  - Union type of the register (Typedef)
 * @param[in]  t_fieldName     - Bit-field name in structure
 * @param[out] xw_return       - Return status
 *
 * @b Example @n
 * @code
 * REG32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT8  c_enable = 1U;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG_STRUCT_FIELD_WRITE(c_devInd, p_mpu_ctrl_reg, c_enable, \
 *          T_SYS_CORE_MPU_CTRL, b1_Enable, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#define M_REG_STRUCT_FIELD_WRITE(c_devInd, p_reg, w_fieldValue, \
            t_registerType, t_fieldName, xw_return) { \
    t_registerType z_local; \
    T_RETURNTYPE xw_returnM; \
    M_REG32_READ(c_devInd, (p_reg), z_local.b32_Reg, (xw_returnM)); \
    (xw_return) = xw_returnM; \
    z_local.bits.t_fieldName = (w_fieldValue); \
    M_REG32_WRITE(c_devInd, (p_reg), z_local.b32_Reg, (xw_returnM)); \
    (xw_return) += xw_returnM; \
}

/*! @} */

/*------------------------------------------------------------------------------
 * Register Update and Compare Macros
 *------------------------------------------------------------------------------
 */
/*!
 * @name FECSS Library Register Interface Safe Write Macro functions
 * @{
 */

/*!*****************************************************************************
 * @brief Safe Write to a bit-field of a register
 *
 * @b Description @n
 * This MACRO function writes and readback a value to a bit-field of a register
 * using address, data.
 * Read 32bit value using M_REG32_READ function and modify bit-field value using
 * structure type and write to register using M_REG32_WRITE. The written data is
 * readback using M_REG32_READ and compared; and the status is update.
 *
 * @param[in]       c_devInd        - Device Index, applicable only for FE devices
 * @param[in]       p_reg           - Address of the register
 * @param[in]       w_fieldValue    - Value of the bit-filed to be read
 * @param[in]       t_registerType  - Union type of the register (Typedef)
 * @param[in]       t_fieldName     - Bit-field name in structure
 * @param[in, out]  w_regCmpSts     - Read back comparision status
 * @param[out]      xw_return       - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT8  c_enable =1U;
 * UINT32 w_regCmpSts = 0U;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG_STRUCT_FIELD_SWRITE(c_devInd, p_mpu_ctrl_reg, c_enable, T_SYS_CORE_MPU_CTRL, \
 * b1_Enable, w_regCmpSts, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#define M_REG_STRUCT_FIELD_SWRITE(c_devInd, p_reg, w_fieldValue, t_registerType, \
            t_fieldName, w_regCmpSts, xw_return) { \
    t_registerType z_local; \
    UINT32 w_readBack; \
    T_RETURNTYPE xw_returnM; \
    M_REG32_READ(c_devInd, (p_reg), z_local.b32_Reg, xw_returnM); \
    (xw_return) = xw_returnM; \
    z_local.bits.t_fieldName = (w_fieldValue); \
    M_REG32_WRITE(c_devInd, (p_reg), z_local.b32_Reg, xw_returnM); \
    (xw_return) += xw_returnM; \
    M_REG32_READ(c_devInd, (p_reg), w_readBack, xw_returnM); \
    (xw_return) += xw_returnM; \
    (w_regCmpSts) |= (z_local.b32_Reg ^ w_readBack); \
}

/*!*****************************************************************************
 * @brief Safe Write 32bit register value to address
 *
 * @b Description @n
 * This MACRO function writes and readback 32bit register value to address.
 * Write using M_REG32_WRITE. The written data is readback using M_REG32_READ
 * and compared; and the status is update.
 *
 * @param[in]       c_devInd        - Device Index, applicable only for FE devices
 * @param[in]       p_reg           - Address of the register
 * @param[in]       w_regValue      - Register content
 * @param[in, out]  w_regCmpSts     - Read back comparision status
 * @param[out]      xw_return       - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_mpu_ctrl_reg = 0xE000ED94;
 * UINT8  w_pmuCtrl = 0xC0FFEE4U;
 * UINT32 w_regCmpSts = 0U;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_REG32_SWRITE(c_devInd, p_mpu_ctrl_reg, w_pmuCtrl, w_regCmpSts, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#define M_REG32_SWRITE(c_devInd, p_reg, w_regValue, w_regCmpSts, xw_return) { \
    UINT32 w_readBack; \
    T_RETURNTYPE xw_returnM; \
    M_REG32_WRITE(c_devInd, p_reg, w_regValue, xw_returnM); \
    (xw_return) = xw_returnM; \
    M_REG32_READ(c_devInd, p_reg, w_readBack, xw_returnM); \
    (xw_return) += xw_returnM; \
    w_regCmpSts |= (w_regValue ^ w_readBack); \
}

/* Register Update and Compare Macros */
/*! @} */

/*------------------------------------------------------------------------------
 * Memory Read Macro functions
 *------------------------------------------------------------------------------
 */
/*!
 * @name FECSS Library Memory Read Macro functions
 * @{
 */

/*!*****************************************************************************
 * @brief Read 32bit value from HW memory
 *
 * @b Description @n
 * This MACRO function reads 32bit memory content from address.
 * SOC Device Type: Read memory address using Pointer
 * FE Device Type: Read memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] w_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_memAdd = 0x0000ED94;
 * UINT32  w_memValue;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM32_READ(p_memAdd, w_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM32_READ(c_devInd, p_memAdd, w_memValue, xw_return) { \
    (w_memValue) = *((UINT32*)(p_memAdd)); \
    (xw_return) = M_DFP_RET_CODE_OK; \
}
#else
#define M_MEM32_READ(c_devInd, p_memAdd, w_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Read 16bit value from HW memory
 *
 * @b Description @n
 * This MACRO function reads 16bit memory content from address.
 * SOC Device Type: Read memory address using Pointer
 * FE Device Type: Read memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] w_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT316 *p_memAdd = 0x0000ED96;
 * UINT16  h_memValue;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM32_READ(p_memAdd, h_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM16_READ(c_devInd, p_memAdd, h_memValue, xw_return) { \
    (h_memValue) = *((UINT16*)(p_memAdd)); \
    (xw_return) = M_DFP_RET_CODE_OK; \
}
#else
#define M_MEM16_READ(c_devInd, p_memAdd, h_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Read 8bit value from HW memory
 *
 * @b Description @n
 * This MACRO function reads 8bit memory content from address.
 * SOC Device Type: Read memory address using Pointer
 * FE Device Type: Read memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] c_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT8 *p_memAdd = 0x0000ED95;
 * UINT8  c_memValue;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM8_READ(p_memAdd, c_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM8_READ(c_devInd, p_memAdd, c_memValue, xw_return) { \
    (c_memValue) = *((UINT8*)(p_memAdd)); \
    (xw_return) = M_DFP_RET_CODE_OK; \
}
#else
#define M_MEM8_READ(c_devInd, p_memAdd, c_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Read N words (32bit data) from a HW memory
 *
 * @b Description @n
 * This MACRO function reads N words (4 bytes) from HW memory address.
 * SOC Device Type: Read memory address using Pointer
 * FE Device Type: Read memory address using Comm IF callback function
 *
 * @b Assumption:
 * 1. In case of FE, the callback function is supported during
 * mmWaveLink interface with application \n
 * 2. The address is 4 bytes aligned
 *
 * @param[in]  c_devInd           - Device Index, applicable only for FE devices
 * @param[in]  p_destMemAdd       - 4 bytes aligned Destination SW memory address
 * @param[in]  p_sourceHwAdd      - 4 bytes aligned source Hw memory address
 * @param[in]  h_numOfWords       - Num of words (4 bytes)
 * @param[out] xw_return          - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_sourceHwAdd = 0x0000ED94;
 * UINT32 w_destSwAdd[10U];
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM_READ(c_devInd, &w_destSwAdd[0U], p_sourceHwAdd, 10U, xw_return)
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM_READ(c_devInd, p_destMemAdd, p_sourceHwAdd, h_numOfWords, xw_return) { \
    (xw_return) = fe_hwMemcopy((UINT32*)(p_destMemAdd), (const UINT32*)(p_sourceHwAdd), \
            (UINT16)(h_numOfWords)); \
}
#else
#define M_MEM_READ(c_devInd, p_destMemAdd, p_sourceHwAdd, h_numOfWords, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/* FECSS Lib Memory Read Macro functions group */
/*! @} */

/*!
 * @name FECSS Library Memory Write Macro functions
 * @{
 */
/*!*****************************************************************************
 * @brief Write 32bit value to HW memory
 *
 * @b Description @n
 * This MACRO function writes 32bit value to a HW memory address.
 * SOC Device Type: Write memory address using Pointer
 * FE Device Type: Write memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] w_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_memAdd = 0x0000ED94;
 * UINT32  w_memValue = 0xAAAAAAAAU;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM32_WRITE(c_devInd, p_memAdd, w_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM32_WRITE(c_devInd, p_memAdd, w_memValue, xw_return) { \
    *((UINT32*)(p_memAdd)) = (w_memValue); \
    (xw_return) = M_DFP_RET_CODE_OK; \
    M_DFP_LOG_ADDR_DATA(p_memAdd, w_memValue); \
}
#else
#define M_MEM32_WRITE(c_devInd, p_memAdd, w_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Write 16bit value to HW memory
 *
 * @b Description @n
 * This MACRO function writes 16bit value to a HW memory address.
 * SOC Device Type: Write memory address using Pointer
 * FE Device Type: Write memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] h_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT16 *p_memAdd = 0x0000ED96;
 * UINT16  h_memValue = 0xAAAAU;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM16_WRITE(c_devInd, p_memAdd, h_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM16_WRITE(c_devInd, p_memAdd, h_memValue, xw_return) { \
    *((UINT16*)(p_memAdd)) = (h_memValue); \
    (xw_return) = M_DFP_RET_CODE_OK; \
    M_DFP_LOG_ADDR_DATA(p_memAdd, h_memValue); \
}
#else
#define M_MEM16_WRITE(c_devInd, p_memAdd, h_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Write 8bit value to HW memory
 *
 * @b Description @n
 * This MACRO function writes 8bit value to a HW memory address.
 * SOC Device Type: Write memory address using Pointer
 * FE Device Type: Write memory address using Comm IF callback function
 *
 * @b Assumption: In case of FE, the callback function is supported during
 * mmWaveLink interface with application
 *
 * @param[in]  c_devInd     - Device Index, applicable only for FE devices
 * @param[in]  p_memAdd     - Address of memory
 * @param[out] c_memValue   - Memory content
 * @param[out] xw_return    - Return status
 *
 * @b Example @n
 * @code
 * UINT8 *p_memAdd = 0x0000ED95;
 * UINT8  c_memValue = 0xAAU;
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM8_WRITE(c_devInd, p_memAdd, c_memValue, xw_return);
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM8_WRITE(c_devInd, p_memAdd, c_memValue, xw_return) { \
    *((UINT8*)(p_memAdd)) = (c_memValue); \
    (xw_return) = M_DFP_RET_CODE_OK; \
    M_DFP_LOG_ADDR_DATA(p_memAdd, c_memValue); \
}
#else
#define M_MEM8_WRITE(c_devInd, p_memAdd, c_memValue, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/*!*****************************************************************************
 * @brief Write N words (32bit data) to a HW memory
 *
 * @b Description @n
 * This MACRO function writes N words (4 bytes) to HW memory address.
 * SOC Device Type: Write memory address using Pointer
 * FE Device Type: Write memory address using Comm IF callback function
 *
 * @b Assumption:
 * 1. In case of FE, the callback function is supported during
 * mmWaveLink interface with application \n
 * 2. The address is 4 bytes aligned
 *
 * @param[in]  c_devInd           - Device Index, applicable only for FE devices
 * @param[in]  p_destHwAdd        - 4 bytes aligned Destination SW memory address
 * @param[in]  p_sourceAdd        - 4 bytes aligned source Hw memory address
 * @param[in]  h_numOfWords       - Num of words (4 bytes), max 65536 words
 * @param[out] xw_return          - Return status
 *
 * @b Example @n
 * @code
 * UINT32 *p_destHwAdd = 0x0000ED94;
 * UINT32 w_sourceSwAdd[10U];
 * T_RETURNTYPE xw_return;
 * UINT8 c_devInd = M_DFP_DEVICE_INDEX_0;
 * M_MEM_WRITE(c_devInd, &p_destHwAdd, &w_sourceSwAdd[0U], 10U, xw_return)
 * @endcode
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-474, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
#if (M_DFP_DEVICE_PLATFORM_TYPE == M_DFP_DEVICE_PLATFORM_SOC)
#define M_MEM_WRITE(c_devInd, p_destHwAdd, p_sourceAdd, h_numOfWords, xw_return) { \
    (xw_return) = fe_hwMemcopy((UINT32*)(p_destHwAdd), (const UINT32*)(p_sourceAdd), \
            (UINT16)(h_numOfWords)); \
}
#else
#define M_MEM_WRITE(c_devInd, p_destHwAdd, p_sourceAdd, h_numOfWords, xw_return) { \
    /*! Front-End only devices are not supported in this DFP */ \
    (xw_return) = M_DFP_RET_CODE_REG_READBACK_ERROR; \
}
#endif

/* FECSS Lib Memory Write Macro functions group */
/*! @} */

/* FECSS Lib Register IF Macro functions group */
/*! @} */

#endif /* FE_REGISTER_H */

/* End FECSSLIB_DEVICE_REGIF_MODULE Group */
/*! @} */

/*
 * END of fe_registers.h file
 */
