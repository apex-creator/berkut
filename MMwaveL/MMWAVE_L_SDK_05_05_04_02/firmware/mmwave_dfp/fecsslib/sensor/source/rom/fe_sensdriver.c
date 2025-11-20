/*!*****************************************************************************
 * @file fe_sensdriver.c
 *
 * @brief FECSSLib Sensor Module driver functions
 *
 * @b Description @n
 * This file provides FECSS library Sensor Module HW interface driver function services.
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
 * @addtogroup FECSSLIB_SENSOR_DRV_MODULE Sensor device Driver API functions
 *  @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     09July2021  TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#include <fecsslib/fecsslib.h>
#include <fecsslib/common/fe_registers.h>
#include <fecsslib/sensor/include/fe_sensdriver.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/****************************FECSSLib Static Functions*************************/
/*!
 * @name FECSS Library Sensor module Driver static function calls
 * @{
 */

/*! @} */

/******************************FECSSLib Sensor Driver APIs*********************/

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile common DFE RX config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Digital frontend (DFE) RX chirp
 * profile common configuration. The chirp RX digout sample rate, num of samples,
 * num of bits and FIR filter selection config are done part of this driver function.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_dfeParams - pointer to params data @ref U_FE_SENS_RX_DFE_REG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-441, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configDfeParams(UINT8 c_devIndex, \
                const U_FE_SENS_RX_DFE_REG *p_dfeParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    REG32 *p_regDfeRxCfg = M_FE_SENS_DFE_RX_CFG_REG_ADDRESS;
    UINT32 w_regValue;
    U_FE_SENS_RX_DFE_REG u_hwRegVal;

    /*! -# Read DFE RX CFG register for modify and write */
    M_REG32_READ(c_devIndex, p_regDfeRxCfg, w_regValue, xw_return);
    u_hwRegVal.b32_Reg = w_regValue;

    /*! -# Modify the DFE RX CFG register values as per input api configuration */
    u_hwRegVal.bits.b8_DigSampRate = p_dfeParams->bits.b8_DigSampRate;
    u_hwRegVal.bits.b12_NumSamps   = p_dfeParams->bits.b12_NumSamps;
    u_hwRegVal.bits.b3_OutBitSel   = p_dfeParams->bits.b3_OutBitSel;
    u_hwRegVal.bits.b2_FirSel      = p_dfeParams->bits.b2_FirSel;

    /*! -# Write new configuration value to DFE register */
    w_regValue = u_hwRegVal.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regDfeRxCfg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile common DFE RX config GET API driver function
 *
 * @b Description @n
 * This Sensor API driver function get configuration of the Digital frontend (DFE) RX chirp
 * profile common configuration. The chirp RX digout sample rate, num of samples,
 * num of bits and FIR filter selection config are done part of this driver function.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_dfeParams - pointer to params data @ref U_FE_SENS_RX_DFE_REG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-442, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_dfeParamsGet(UINT8 c_devIndex, \
                U_FE_SENS_RX_DFE_REG *p_dfeParams)
{
    T_RETURNTYPE xw_return;
    REG32 *p_regDfeRxCfg = M_FE_SENS_DFE_RX_CFG_REG_ADDRESS;

    /*! -# Read DFE RX CFG register into the union */
    M_REG32_READ(c_devIndex, p_regDfeRxCfg, p_dfeParams->b32_Reg, xw_return);
    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile common Chirp Timer (CT) config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Chirp Timer (CT) profile common configuration.
 * The chirp TX MIMO pattern and chirp ramp end time config are done part of this driver function.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ctComnParams  - pointer to params data @ref T_FE_SENS_CT_COMN_PROFILE
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-443, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configCtComnParams(UINT8 c_devIndex, \
                const T_FE_SENS_CT_COMN_PROFILE *p_ctComnParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtTxMimoPtrn  = (REG32*)(&p_structChirpTimer->r_TxMimoPattern.b32_Reg);
    REG32 *p_regCtRampendTime = (REG32*)(&p_structChirpTimer->r_RampEndTime.b32_Reg);
    REG32 *p_regCtHpfFastInitEn  = (REG32*)(&p_structChirpTimer->r_MiscelleniousSettings.b32_Reg);
    REG32 *p_regCtCrdEn  = \
        (REG32*)(&p_structChirpTimer->r_ControlledRampDownSettings_1.b32_Reg);
    REG32 *p_regCtCrdNslope  = \
        (REG32*)(&p_structChirpTimer->r_ControlledRampDownSettings_2.b32_Reg);
    REG32 *p_regCtHpfDuration  = \
        (REG32*)(&p_structChirpTimer->r_HpfFastInitSynthTiming_1Reg.b32_Reg);
    REG32 *p_regCtPaBlankEn  = (REG32*)(&p_structChirpTimer->r_PaBlanking.b32_Reg);

    UINT32 w_regValue,
           w_apiVal;

    /*! -# Write r_TxMimoPattern register with API input config */
    w_regValue = (UINT32)p_ctComnParams->c_ChirpTxMimoPatSel;
    M_REG32_WRITE(c_devIndex, p_regCtTxMimoPtrn, w_regValue, xw_return);

    /*! -# Write r_RampEndTime register with API input config */
    w_regValue = (UINT32)p_ctComnParams->h_ChirpRampEndTime;
    M_REG32_WRITE(c_devIndex, p_regCtRampendTime, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_HpfFastInitSynthTiming_1Reg register with API input config
     *  bits [15:8]  = b8_HpfFastInitPulseWidth.
     */
    w_regValue = (UINT32)p_ctComnParams->c_HpfFastInitDuration;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regCtHpfDuration, (UINT8)w_regValue, \
            U_CT_HPF_FAST_INIT_SYNTH_TIMING_1_REG, b8_HpfFastInitPulseWidth, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_MiscelleniousSettings register with API input config
     *  bits [3]  = b1_HpfFastInitEn.
     *     - Negate the API Value before writing to Hw
     */
    w_regValue = ((UINT32)p_ctComnParams->c_MiscSettings & \
                    ((UINT32)1U << M_RL_SENS_MISC_HPF_FAST_INIT_DIS_BIT)) >> \
                    M_RL_SENS_MISC_HPF_FAST_INIT_DIS_BIT;
    w_regValue = (~w_regValue) & 0x1U;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regCtHpfFastInitEn, (UINT8)w_regValue, \
            U_CT_MISCELLENIOUS_SETTINGS, b1_HpfFastInitEn, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_ControlledRampDownSettings_2 register with API input config
     *  bits [14:0]  = b15_CrdNslopeMag.
     */
    w_regValue = (UINT32)p_ctComnParams->h_CrdNSlopeMag;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regCtCrdNslope, (UINT16)w_regValue, \
            U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_2, b15_CrdNslopeMag, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_ControlledRampDownSettings_1 register with API input config
     *  bits [1:0]  = b1_CrdDitherEn, b1_CrdEn.
     *     - Negate the API Value before writing to Hw
     */
    M_REG32_READ(c_devIndex, p_regCtCrdEn, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue &= ~M_SENS_CRD_DITHER_EN_MASK;
    w_apiVal = ((UINT32)p_ctComnParams->c_MiscSettings & \
                    ((UINT32)M_SENS_CRD_DITHER_EN_MASK << M_RL_SENS_MISC_CRD_DIS_BIT)) >> \
                    M_RL_SENS_MISC_CRD_DIS_BIT;
    w_apiVal = (~w_apiVal & M_SENS_CRD_DITHER_EN_MASK);
    w_regValue |= w_apiVal;
    M_REG32_WRITE(c_devIndex, p_regCtCrdEn, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_PaBlanking register with API input config
     *  bits [1:0]  = b1_EnDfePaBlanking, b1_EnPaBlankingFeature.
     */
    M_REG32_READ(c_devIndex, p_regCtPaBlankEn, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue &= ~M_SENS_PA_BLANK_DFE_EN_MASK;
    w_apiVal = ((UINT32)p_ctComnParams->c_MiscSettings & \
                    ((UINT32)1U << M_RL_SENS_MISC_PA_BLANK_EN_BIT));
    if ((UINT32)0U != w_apiVal)
    {
        w_regValue |= M_SENS_PA_BLANK_DFE_EN_MASK;
    }
    M_REG32_WRITE(c_devIndex, p_regCtPaBlankEn, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile common Chirp Timer (CT) config Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function get configuration of the Chirp Timer (CT) profile common configuration.
 * The chirp TX MIMO pattern and chirp ramp end time config are done part of this driver function.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ctComnParams  - pointer to params data @ref T_FE_SENS_CT_COMN_PROFILE
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-444, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctComnParamsGet(UINT8 c_devIndex, \
                 T_FE_SENS_CT_COMN_PROFILE *p_ctComnParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtTxMimoPtrn  = (REG32*)(&p_structChirpTimer->r_TxMimoPattern.b32_Reg);
    REG32 *p_regCtRampendTime = (REG32*)(&p_structChirpTimer->r_RampEndTime.b32_Reg);
    REG32 *p_regCtHpfFastInitEn  = (REG32*)(&p_structChirpTimer->r_MiscelleniousSettings.b32_Reg);
    REG32 *p_regCtCrdEn  = \
        (REG32*)(&p_structChirpTimer->r_ControlledRampDownSettings_1.b32_Reg);
    REG32 *p_regCtCrdNslope  = \
        (REG32*)(&p_structChirpTimer->r_ControlledRampDownSettings_2.b32_Reg);
    REG32 *p_regCtHpfDuration  = \
        (REG32*)(&p_structChirpTimer->r_HpfFastInitSynthTiming_1Reg.b32_Reg);
    REG32 *p_regCtPaBlankEn  = (REG32*)(&p_structChirpTimer->r_PaBlanking.b32_Reg);

    UINT32 w_regValue;

    /*! -# Read r_TxMimoPattern register  */
    M_REG32_READ(c_devIndex, p_regCtTxMimoPtrn, w_regValue, xw_return);
    p_ctComnParams->c_ChirpTxMimoPatSel = (UINT8)w_regValue;

    /*! -# Read r_RampEndTime register */
    M_REG32_READ(c_devIndex, p_regCtRampendTime, w_regValue, xw_return1);
    p_ctComnParams->h_ChirpRampEndTime = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_HpfFastInitSynthTiming_1Reg register
     *  bits [15:8]  = b8_HpfFastInitPulseWidth.
     */
    M_REG_STRUCT_FIELD_READ(c_devIndex, p_regCtHpfDuration, w_regValue, \
            U_CT_HPF_FAST_INIT_SYNTH_TIMING_1_REG, b8_HpfFastInitPulseWidth, xw_return1);
    p_ctComnParams->c_HpfFastInitDuration = (UINT8)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_MiscelleniousSettings register
     *  bits [3]  = b1_HpfFastInitEn.
     *     - Negate the API Value before updating
     */
    M_REG_STRUCT_FIELD_READ(c_devIndex, p_regCtHpfFastInitEn, w_regValue, \
            U_CT_MISCELLENIOUS_SETTINGS, b1_HpfFastInitEn, xw_return1);
    w_regValue = (~w_regValue) & 0x1U;
    p_ctComnParams->c_MiscSettings |= (UINT8)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_ControlledRampDownSettings_2 register
     *  bits [14:0]  = b15_CrdNslopeMag.
     */
    M_REG_STRUCT_FIELD_READ(c_devIndex, p_regCtCrdNslope, w_regValue, \
            U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_2, b15_CrdNslopeMag, xw_return1);
    p_ctComnParams->h_CrdNSlopeMag = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_ControlledRampDownSettings_1 register
     *  bits [1:0]  = b1_CrdDitherEn, b1_CrdEn.
     *     - Negate the API Value before writing to Hw
     */
    M_REG32_READ(c_devIndex, p_regCtCrdEn, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue = (~w_regValue) & M_SENS_CRD_DITHER_EN_MASK;
    w_regValue = w_regValue << M_RL_SENS_MISC_CRD_DIS_BIT;
    p_ctComnParams->c_MiscSettings |= (UINT8)w_regValue;

    /*! -# Read r_PaBlanking register
     *  bits [1:0]  = b1_EnDfePaBlanking, b1_EnPaBlankingFeature.
     */
    M_REG32_READ(c_devIndex, p_regCtPaBlankEn, w_regValue, xw_return1);
    xw_return += xw_return1;
    w_regValue = w_regValue & M_SENS_PA_BLANK_DFE_EN_MASK;
    if (w_regValue == M_SENS_PA_BLANK_DFE_EN_MASK)
    {
        p_ctComnParams->c_MiscSettings |= (UINT8)1U << M_RL_SENS_MISC_PA_BLANK_EN_BIT;
    }

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Chirp Timer (CT) Resolution config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Chirp Timer (CT) timing/RF freq resolution.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] c_resolType     - Chirp timer Timing and RF freq resolution type
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-445, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configCtResolution(UINT8 c_devIndex, const UINT8 c_resolType)
{
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtRes = (REG32*)(&p_structChirpTimer->r_ChirpTimerProgResolution.b32_Reg);
    UINT32 w_regValue;
    T_RETURNTYPE xw_return;

    /*! -# Configure Chirp timer resolution global setting */
    w_regValue = (UINT32)c_resolType;
    M_REG32_WRITE(c_devIndex, p_regCtRes, w_regValue, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Chirp Timer (CT) Resolution Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function to Get the Chirp Timer (CT) timing/RF freq resolution.
 *
 * @b Assumption: The FECSS power up is done before issuing this API.
 *
 * @param[in]  c_devIndex      - Device Index
 * @param[out] p_resolType     - Chirp timer Timing and RF freq resolution type
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-446, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_getCtResolution(UINT8 c_devIndex, UINT8 *p_resolType)
{
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtRes = (REG32*)(&p_structChirpTimer->r_ChirpTimerProgResolution.b32_Reg);
    UINT32 w_regValue;
    T_RETURNTYPE xw_return;

    /*! -# Read Chirp timer resolution global setting */
    M_REG32_READ(c_devIndex, p_regCtRes, w_regValue, xw_return);
    /*! -# Update the result */
    *p_resolType = (UINT8)(w_regValue & (UINT32)M_FE_SENS_CT_RESOLUTION_MASK);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile timing Chirp Timer (CT) config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Chirp Timer (CT) profile timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable config are done part of this driver function.
 *
 * @b Assumption: The Sensor chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ctTimeParams  - pointer to params data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-447, MMWAVE_DFP_REQ-472, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configCtTimeParams(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_ctTimeParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Write r_IdleTime register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_IdleTime.b32_Reg);
    w_regValue = (UINT32)p_ctTimeParams->h_ChirpIdleTime;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return);

    /*! -# Write r_AdcStartTime register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_AdcStartTime.b32_Reg);
    w_regValue = (UINT32)p_ctTimeParams->h_ChirpAdcStartTime;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_TxStartTime register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_TxStartTime.b32_Reg);
    w_regValue = (UINT32)p_ctTimeParams->xh_ChirpTxStartTime;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_FreqSlope register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_FreqSlope.b32_Reg);
    w_regValue = (UINT32)p_ctTimeParams->xh_ChirpRfFreqSlope;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_FreqStart register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_FreqStart.b32_Reg);
    w_regValue = (UINT32)p_ctTimeParams->w_ChirpRfFreqStart;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_TxEnBpmSettings register with API input config
     *    - combine TX enable [3:0] and TX BPM enable [7:4] in one register
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_TxEnBpmSettings.b32_Reg);
    w_regValue = (UINT32)(p_ctTimeParams->h_ChirpTxEnSel) |
                        ((UINT32)p_ctTimeParams->h_ChirpTxBpmEnSel << (UINT32)4U);
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor chirp profile timing Chirp Timer (CT) config Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function get the configuration of the Chirp Timer (CT) profile timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable config are done part of this driver function.
 *
 * @b Assumption: The Sensor chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ctTimeParams  - pointer to params data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-448, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctTimeParamsGet(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_ctTimeParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Clear response structure */
    xw_return = fe_hwMemclr((UINT32*)p_ctTimeParams, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG)/sizeof(UINT32));

    /*! -# Read r_IdleTime register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_IdleTime.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->h_ChirpIdleTime = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_AdcStartTime register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_AdcStartTime.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->h_ChirpAdcStartTime = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_TxStartTime register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_TxStartTime.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->xh_ChirpTxStartTime = (SINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_FreqSlope register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_FreqSlope.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->xh_ChirpRfFreqSlope = (SINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_FreqStart register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_FreqStart.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->w_ChirpRfFreqStart = w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_TxEnBpmSettings register
     *    - combine TX enable [3:0] and TX BPM enable [7:4] in one register
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_TxEnBpmSettings.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctTimeParams->h_ChirpTxEnSel = (UINT16)(w_regValue & (UINT32)0x0FU);
    p_ctTimeParams->h_ChirpTxBpmEnSel = (UINT16)((w_regValue & (UINT32)0xF0) >> 4U);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor per chirp timing Chirp Timer (CT) config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Chirp Timer (CT) per chirp timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable are part of per chirp config LUT array, this driver function configures
 * length and repeat count of these param LUTs.
 *
 * @b Assumption: The Sensor chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ctPerChirpCfgParams - pointer to params data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-449, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configCtPerChirpHw(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_ctPerChirpCfgParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;
    U_CT_PERCHIRP_PARAMETER_LEN1 u_ctPerChirpLen1;
    U_CT_PERCHIRP_PARAMETER_LEN2 u_ctPerChirpLen2;
    U_CT_PERCHIRP_PARAMETER_LEN3 u_ctPerChirpLen3;
    U_CT_PERCHIRP_PARAMETER_LEN4 u_ctPerChirpLen4;

    /*! -# Write r_PerchirpParameterLen1 register with API input config
     *  bits [15:0]  = b16_PerchirpFreqStartLen.
     *  bits [31:16] = b16_PerchirpFreqSlopeLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen1.b32_Reg);
    u_ctPerChirpLen1.bits.b16_PerchirpFreqStartLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_START] - (UINT16)1U;
    u_ctPerChirpLen1.bits.b16_PerchirpFreqSlopeLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] - (UINT16)1U;
    w_regValue = (UINT32)u_ctPerChirpLen1.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return);

    /*! -# Write r_PerchirpParameterLen2 register with API input config
     *  bits [15:0]  = b16_PerchirpIdleTimeLen.
     *  bits [31:16] = b16_PerchirpAdcStartTimeLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen2.b32_Reg);
    u_ctPerChirpLen2.bits.b16_PerchirpIdleTimeLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_IDLE_TIME] - (UINT16)1U;
    u_ctPerChirpLen2.bits.b16_PerchirpAdcStartTimeLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_ADC_START_TIME] - (UINT16)1U;
    w_regValue = (UINT32)u_ctPerChirpLen2.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpParameterLen3 register with API input config
     *  bits [15:0]  = b16_PerchirpTxStartTimeLen.
     *  bits [31:16] = b16_PerchirpTxEnLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen3.b32_Reg);
    u_ctPerChirpLen3.bits.b16_PerchirpTxStartTimeLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_START_TIME] - (UINT16)1U;
    u_ctPerChirpLen3.bits.b16_PerchirpTxEnLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_ENABLE] - (UINT16)1U;
    w_regValue = (UINT32)u_ctPerChirpLen3.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpParameterLen4 register with API input config
     *  bits [15:0]  = b16_PerchirpTxBpmLen.
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen4.b32_Reg);
    u_ctPerChirpLen4.bits.b16_PerchirpTxBpmLen = \
        p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_BPM_ENABLE] - (UINT16)1U;
    u_ctPerChirpLen4.bits.b16_Padfield_0 = 0U;
    w_regValue = (UINT32)u_ctPerChirpLen4.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpFreqStartRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpFreqStartRepCnt.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_START] - \
                    (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpFreqSlopeRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpFreqSlopeRepCnt.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] - \
        (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpIdleTimeRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpIdleTimeRepCnt.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_IDLE_TIME] - \
        (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpAdcStartTimeRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpAdcStartTimeRepCnt.b32_Reg);
    w_regValue = \
        (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_ADC_START_TIME] - \
            (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpTxStartTimeRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxStartTimeRepCnt.b32_Reg);
    w_regValue = \
        (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_START_TIME] - \
        (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpTxEnRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxEnRepCnt.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_ENABLE] - \
        (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpTxBpmRepCnt register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxBpmRepCnt.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_BPM_ENABLE] - \
        (UINT32)1U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor per chirp timing Chirp Timer (CT) config Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function get the configuration the Chirp Timer (CT) per chirp timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable are part of per chirp config LUT array, this driver function configures
 * length and repeat count of these param LUTs.
 *
 * @b Assumption: The Sensor chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ctPerChirpCfgParams - pointer to params data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-450, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctPerChirpHwCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_ctPerChirpCfgParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Clear response structure */
    xw_return = fe_hwMemclr((UINT32*)p_ctPerChirpCfgParams, sizeof(T_RL_API_SENS_PER_CHIRP_CFG)/sizeof(UINT32));

    /*! -# Read r_PerchirpParameterLen1 register
     *  bits [15:0]  = b16_PerchirpFreqStartLen.
     *  bits [31:16] = b16_PerchirpFreqSlopeLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen1.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_START] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = \
                                ((UINT16)(w_regValue >> (UINT32)16U)) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterLen2 register
     *  bits [15:0]  = b16_PerchirpIdleTimeLen.
     *  bits [31:16] = b16_PerchirpAdcStartTimeLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen2.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_IDLE_TIME] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = \
                                (UINT16)(w_regValue >> (UINT32)16U) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterLen3 register
     *  bits [15:0]  = b16_PerchirpTxStartTimeLen.
     *  bits [31:16] = b16_PerchirpTxEnLen
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen3.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_START_TIME] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_ENABLE] = \
                                (UINT16)(w_regValue >> (UINT32)16U) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterLen4 register
     *  bits [15:0]  = b16_PerchirpTxBpmLen.
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterLen4.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpFreqStartRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpFreqStartRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_START] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpFreqSlopeRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpFreqSlopeRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpIdleTimeRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpIdleTimeRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_IDLE_TIME] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpAdcStartTimeRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpAdcStartTimeRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpTxStartTimeRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxStartTimeRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_START_TIME] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpTxEnRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxEnRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_ENABLE] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Read r_PerchirpTxBpmRepCnt register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpTxBpmRepCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCfgParams->h_ParamRptCount[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = \
                                            (UINT16)(w_regValue & (UINT32)0x0000FFFFU) + (UINT16)1U;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor per chirp timing Chirp Timer (CT) control API driver function
 *
 * @b Description @n
 * This Sensor API driver function controls the Chirp Timer (CT) per chirp timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable are part of per chirp config LUT array, this driver function controls (ON/OFF)
 * and selects the right start offset address.
 *
 * @b Assumption: The Sensor per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ctPerChirpCtrl  - pointer to params data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-451, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctrlCtPerChirpHw(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_ctPerChirpCtrl)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;
    U_CT_PERCHIRP_PARAMETER_ADDR1 u_ctPerChirpAdd1;
    U_CT_PERCHIRP_PARAMETER_ADDR2 u_ctPerChirpAdd2;
    U_CT_PERCHIRP_PARAMETER_ADDR3 u_ctPerChirpAdd3;

    /*! -# Write r_PerchirpParameterAddr1 register with API input config
     *  bits [15:0]  = b16_PerchirpFreqStartAddr.
     *  bits [31:16] = b16_PerchirpFreqSlopeAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr1.b32_Reg);
    u_ctPerChirpAdd1.bits.b16_PerchirpFreqStartAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_START];
    u_ctPerChirpAdd1.bits.b16_PerchirpFreqSlopeAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_SLOPE];
    w_regValue = (UINT32)u_ctPerChirpAdd1.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return);

    /*! -# Write r_PerchirpParameterAddr2 register with API input config
     *  bits [15:0]  = b16_PerchirpIdleTimeAddr.
     *  bits [31:16] = b16_PerchirpAdcStartTimeAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr2.b32_Reg);
    u_ctPerChirpAdd2.bits.b16_PerchirpIdleTimeAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_IDLE_TIME];
    u_ctPerChirpAdd2.bits.b16_PerchirpAdcStartTimeAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_ADC_START_TIME];
    w_regValue = (UINT32)u_ctPerChirpAdd2.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpParameterAddr3 register with API input config
     *  bits [15:0]  = b16_PerchirpTxStartTimeAddr.
     *  bits [31:16] = b16_PerchirpTxEnAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr3.b32_Reg);
    u_ctPerChirpAdd3.bits.b16_PerchirpTxStartTimeAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_START_TIME];
    u_ctPerChirpAdd3.bits.b16_PerchirpTxEnAddr = \
            p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_ENABLE];
    w_regValue = (UINT32)u_ctPerChirpAdd3.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Read modify Write r_PerchirpParameterAddr4 register with API input config
     *  bits [15:0]  = b16_PerchirpTxBpmAddr.
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr4.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_BPM_ENABLE];
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regCtReg, (UINT16)w_regValue, \
            U_CT_PERCHIRP_PARAMETER_ADDR4, b16_PerchirpTxBpmAddr, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_PerchirpParameterEn register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterEn.b32_Reg);
    w_regValue = (UINT32)p_ctPerChirpCtrl->h_PerChirpParamCtrl;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor per chirp timing Chirp Timer (CT) control Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function get the control params of the Chirp Timer (CT) per chirp timing configuration.
 * The chirp idle time, ADC start time, TX start time, RF Slope, RF start freq, TX and
 * BPM enable are part of per chirp config LUT array, this driver function controls (ON/OFF)
 * and selects the right start offset address.
 *
 * @b Assumption: The Sensor per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ctPerChirpCtrl  - pointer to params data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-452, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctPerChirpHwCtrlGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_ctPerChirpCtrl)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Clear response structure */
    xw_return = fe_hwMemclr((UINT32*)p_ctPerChirpCtrl, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL)/sizeof(UINT32));

    /*! -# Read r_PerchirpParameterAddr1 register
     *  bits [15:0]  = b16_PerchirpFreqStartAddr.
     *  bits [31:16] = b16_PerchirpFreqSlopeAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr1.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_START] = \
                                                        (UINT16)(w_regValue & (UINT32)0x0000FFFFU);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = \
                                            (UINT16)(w_regValue >> (UINT32)16U);
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterAddr2 register
     *  bits [15:0]  = b16_PerchirpIdleTimeAddr.
     *  bits [31:16] = b16_PerchirpAdcStartTimeAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr2.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_IDLE_TIME] = \
                                                        (UINT16)(w_regValue & (UINT32)0x0000FFFFU);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = \
                                            (UINT16)(w_regValue >> (UINT32)16U);
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterAddr3 register
     *  bits [15:0]  = b16_PerchirpTxStartTimeAddr.
     *  bits [31:16] = b16_PerchirpTxEnAddr
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr3.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_START_TIME] = \
                                                        (UINT16)(w_regValue & (UINT32)0x0000FFFFU);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_ENABLE] = \
                                            (UINT16)((UINT32)w_regValue >> (UINT32)16U);
    xw_return += xw_return1;

    /*! -#  Read r_PerchirpParameterAddr4 register
     *  bits [15:0]  = b16_PerchirpTxBpmAddr.
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterAddr4.b32_Reg);
    M_REG_STRUCT_FIELD_READ(c_devIndex, p_regCtReg, w_regValue, \
            U_CT_PERCHIRP_PARAMETER_ADDR4, b16_PerchirpTxBpmAddr, xw_return1);
    p_ctPerChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = \
                                                        (UINT16)(w_regValue & (UINT32)0x0000FFFFU);
    xw_return += xw_return1;

    /*! -# Read r_PerchirpParameterEn register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_PerchirpParameterEn.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctPerChirpCtrl->h_PerChirpParamCtrl = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Chirp Timer (CT) burst config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Chirp Timer (CT) burst configuration.
 * The config parameters like num of chirps, num of chirp accumulation, Burst period,
 * num of bursts.
 *
 * @b Assumption: The Sensor chirp profile common and time cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ctBurstParams - pointer to params data @ref T_FE_SENS_CT_BURST_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-453, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configCtBurstParams(UINT8 c_devIndex, \
                const T_FE_SENS_CT_BURST_CFG *p_ctBurstParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Write r_NumChirpsAndBursts register with API input config
     *    - combine num chirps [15:0] and num bursts [31:16] in one register
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_NumChirpsAndBursts.b32_Reg);
    w_regValue = ((UINT32)p_ctBurstParams->h_NumOfChirpsInBurst | \
                    ((UINT32)p_ctBurstParams->h_NumOfBurstsInFrame << (UINT16)16U));
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return);

    /*! -# Write r_ChirpAccumSettings register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpAccumSettings.b32_Reg);
    w_regValue = (UINT32)p_ctBurstParams->c_NumOfChirpsAccum;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_BurstPeriodicity register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_BurstPeriodicity.b32_Reg);
    w_regValue = (UINT32)p_ctBurstParams->w_BurstPeriodicity;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# The r_BurstStartOffsetTime register is used by RFS, application
     * shall not update this register
     */

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Chirp Timer (CT) burst config Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function get configuration of the Chirp Timer (CT) burst configuration.
 * The config parameters like num of chirps, num of chirp accumulation, Burst period,
 * num of bursts.
 *
 * @b Assumption: The Sensor chirp profile common and time cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ctBurstParams - pointer to params data @ref T_FE_SENS_CT_BURST_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-454, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctBurstParamsGet(UINT8 c_devIndex, \
                T_FE_SENS_CT_BURST_CFG *p_ctBurstParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    UINT32 w_regValue;

    /*! -# Read r_NumChirpsAndBursts register with API input config
     *    - combine num chirps [15:0] and num bursts [31:16] in one register
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_NumChirpsAndBursts.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return);
    p_ctBurstParams->h_NumOfChirpsInBurst = (UINT16)(w_regValue & (UINT32)0xFFFFU);
    p_ctBurstParams->h_NumOfBurstsInFrame = (UINT16)(w_regValue >> (UINT32)16U);

    /*! -# Read r_ChirpAccumSettings register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpAccumSettings.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctBurstParams->c_NumOfChirpsAccum = (UINT8)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_BurstPeriodicity register with API input config */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_BurstPeriodicity.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctBurstParams->w_BurstPeriodicity = w_regValue;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) frame config API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Frame Timer (FT) frame configuration.
 * The config parameters like Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The Sensor chirp profile common and time cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[in] p_ftFrameParams - pointer to params data @ref T_FE_SENS_FT_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-455, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_configFtFrameParams(UINT8 c_devIndex, \
                const T_FE_SENS_FT_FRAME_CFG *p_ftFrameParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    REG32 *p_regFtReg;
    UINT32 w_regValue;

    /*! -# Write r_FramePeriodicity register with API input config
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FramePeriodicity.b32_Reg);
    w_regValue = (UINT32)p_ftFrameParams->w_FramePeriodicity;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return);

    /*! -# Write r_NumFrames register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_NumFrames.b32_Reg);
    w_regValue = (UINT32)p_ftFrameParams->h_NumOfFrames;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# The r_FrameStartOffsetIntrTime_1 register is used by RFS, application
     * shall not update this register
     */

    /*! -# Write r_FrameStartOffsetIntrTime_2 register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameStartOffsetIntrTime_2.b32_Reg);
    w_regValue = (UINT32)p_ftFrameParams->w_FrameEvent0TimeCfg;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Write r_FrameStartOffsetIntrTime_3 register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameStartOffsetIntrTime_3.b32_Reg);
    w_regValue = (UINT32)p_ftFrameParams->w_FrameEvent1TimeCfg;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) frame config Get API driver function
 *
 * @b Description @n
 * This Sensor API driver function Get configuration of the Frame Timer (FT) frame configuration.
 * The config parameters like Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The Sensor chirp profile common and time cfg is done before issuing this API.
 *
 * @param[in] c_devIndex      - Device Index
 * @param[out] p_ftFrameParams - pointer to params data @ref T_FE_SENS_FT_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-456, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ftFrameParamsGet(UINT8 c_devIndex, \
                T_FE_SENS_FT_FRAME_CFG *p_ftFrameParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    REG32 *p_regFtReg;
    UINT32 w_regValue;

    /*! -# Read r_FramePeriodicity register with API input config
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FramePeriodicity.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return);
    p_ftFrameParams->w_FramePeriodicity = w_regValue;

    /*! -# Read r_NumFrames register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_NumFrames.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftFrameParams->h_NumOfFrames = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_FrameStartOffsetIntrTime_2 register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameStartOffsetIntrTime_2.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftFrameParams->w_FrameEvent0TimeCfg = w_regValue;
    xw_return += xw_return1;

    /*! -# Read r_FrameStartOffsetIntrTime_3 register with API input config */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameStartOffsetIntrTime_3.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftFrameParams->w_FrameEvent1TimeCfg = w_regValue;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) Sensor start API driver function
 *
 * @b Description @n
 * This Sensor API driver function configures the Frame Timer (FT) sensor start mode
 * chirp timer start signal LB and triggers the frame.
 *
 * @b Assumption: The Sensor frame cfg is done before issuing this API.
 *
 * @param[in] c_devIndex          - Device Index
 * @param[in] p_ftSensStartParams - pointer to params data @ref T_RL_API_SENSOR_START_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-457, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ftSensorStart(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_ftSensStartParams)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    REG32 *p_regFtReg;
    REG32 *p_regTopFuse;
    UINT32 w_regValue;
    U_FT_FRAME_TRIGGERING_REGS u_ftframeTrigReg = {0};

    /*! -# Write r_FrameTrigTimerVal register with API input config
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameTrigTimerVal.b32_Reg);
    w_regValue = (UINT32)p_ftSensStartParams->w_FrameTrigTimerVal;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return);

    /*! -# Clear FT r_TrigStopDone register before frame trigger
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_TrigStopDone.b32_Reg);
    w_regValue = (UINT32)M_FE_SENS_FT_CT_START_STOP_DONE_CLEAR;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Clear CT r_TrigStopDone register before frame trigger
     */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpTimerTrigStopDone.b32_Reg);
    w_regValue = (UINT32)M_FE_SENS_FT_CT_START_STOP_DONE_CLEAR;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Clear CT r_ChirpTimerTrig register before frame trigger, this will make sure
     *  CT is triggered by frame timer   */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpTimerTrig.b32_Reg);
    w_regValue = (UINT32)0U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Update Ct trigger mode mux for hardware-triggered mode */
    p_regTopFuse = M_FE_SENS_F_MODE_MUX_REG_ADDRESS;
    if ((M_RL_SENS_FRAME_HW_LOW_PWR_TRIG == p_ftSensStartParams->c_FrameTrigMode) ||
        (M_RL_SENS_FRAME_HW_LOW_JIT_TRIG == p_ftSensStartParams->c_FrameTrigMode))
    {
        w_regValue   = (UINT32)0x1U;
    }
    else
    {
        w_regValue   = (UINT32)0x0U;
    }
    M_REG32_WRITE(c_devIndex, p_regTopFuse, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Write r_FrameTriggeringRegs register with API input config
     *   - Combine trigger mode [1:0] and LB en [2]
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameTriggeringRegs.b32_Reg);
    u_ftframeTrigReg.bits.b2_FrameTrigMode = \
            p_ftSensStartParams->c_FrameTrigMode & (UINT8)0x3U;
    u_ftframeTrigReg.bits.b1_LoopStartFrameToDigSync = \
            p_ftSensStartParams->c_ChirpStartSigLbEn & (UINT8)0x1U;
    w_regValue = (UINT32)u_ftframeTrigReg.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Trigger Frame using r_FrameTriggeringRegs.bits.b3_FrameTrigReg register
     *   - b3_FrameTrigReg [5:3] = 0x7
     *   - The frame timer is started and the pre frame settings will be done by RFS before
     * chirp timer is triggered by FT.
     */
    u_ftframeTrigReg.bits.b3_FrameTrigReg = (UINT8)M_FE_SENS_FT_SENSOR_START_STOP_KEY;
    w_regValue = (UINT32)u_ftframeTrigReg.b32_Reg;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) Sensor stop API driver function
 *
 * @b Description @n
 * This Sensor API driver function stops the Frame Timer (FT) sensor at frame
 * boundary.
 *
 * @b Assumption: The Sensor frame start is done before issuing this API.
 *
 * @param[in] c_devIndex          - Device Index
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-458, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ftSensorStop(UINT8 c_devIndex)
{
    T_RETURNTYPE xw_return;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    REG32 *p_regFtReg;
    UINT32 w_regValue;

    /*!
     * -# Write M_FE_SENS_FT_SENSOR_START_STOP_KEY to
     * r_FrameTriggeringRegs.b3_FrameStopReg register
     */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameTriggeringRegs.b32_Reg);
    w_regValue = (UINT32)M_FE_SENS_FT_SENSOR_START_STOP_KEY;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFtReg, (UINT8)w_regValue, \
            U_FT_FRAME_TRIGGERING_REGS, b3_FrameStopReg, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) Sensor timer force stop API driver function
 *
 * @b Description @n
 * This Sensor API driver function resets the Frame Timer (FT) and chirp timer (FT)
 * sensor. This function forcefully stops the all sensor timers for error recovery.
 *
 * @b Assumption: The Sensor frame start is done before issuing this API.
 *
 * @param[in] c_devIndex          - Device Index
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-459, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ftSensorForceStop(UINT8 c_devIndex)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FECRCM_REGS *p_structFecRcm = M_FE_FECSS_RCM_REG_START_ADDRESS;
    REG32 *p_regFecRcmReg;
    REG32 *p_regFtResetReg = M_FE_SENS_APPSS_FT_RESET_REG_ADDRESS;
    UINT32 w_regValue,
           w_appssRegValue;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;
    REG32 *p_regFtReg;

    /*! -# Clear FT r_TrigStopDone register  */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_TrigStopDone.b32_Reg);
    w_regValue = (UINT32)M_FE_SENS_FT_CT_START_STOP_DONE_CLEAR;
    M_REG32_WRITE(c_devIndex, p_regFtReg, w_regValue, xw_return);

    /*! -# Clear CT r_TrigStopDone register  */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpTimerTrigStopDone.b32_Reg);
    w_regValue = (UINT32)M_FE_SENS_FT_CT_START_STOP_DONE_CLEAR;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Reset the frame timer
     */
    M_REG32_READ(c_devIndex, p_regFtResetReg, w_appssRegValue, xw_return1);
    xw_return += xw_return1;
    w_regValue = w_appssRegValue | M_FE_SENS_FT_RESET_REG_MASK;
    M_REG32_WRITE(c_devIndex, p_regFtResetReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Reset the chirp timer interpreter
     */
    p_regFecRcmReg  = (REG32*)(&p_structFecRcm->r_Blockreset0.b32_Reg);
    w_regValue = M_FE_SENS_TIMER_RESET_KEY;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcmReg, (UINT8)w_regValue, \
            U_FECRCM_BLOCKRESET0, b3_FecPerchirpMemIntrp, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Reset the chirp timer
     */
    p_regFecRcmReg  = (REG32*)(&p_structFecRcm->r_Blockreset0.b32_Reg);
    w_regValue = M_FE_SENS_TIMER_RESET_KEY;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcmReg, (UINT8)w_regValue, \
            U_FECRCM_BLOCKRESET0, b3_FecChirpTimer, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Release Reset to chirp timer
     */
    p_regFecRcmReg  = (REG32*)(&p_structFecRcm->r_Blockreset0.b32_Reg);
    w_regValue = M_FE_SENS_TIMER_RESET_RELEASE_KEY;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcmReg, (UINT8)w_regValue, \
            U_FECRCM_BLOCKRESET0, b3_FecChirpTimer, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Release Reset to chirp timer interpreter
     */
    p_regFecRcmReg  = (REG32*)(&p_structFecRcm->r_Blockreset0.b32_Reg);
    w_regValue = M_FE_SENS_TIMER_RESET_RELEASE_KEY;
    M_REG_STRUCT_FIELD_WRITE(c_devIndex, p_regFecRcmReg, (UINT8)w_regValue, \
            U_FECRCM_BLOCKRESET0, b3_FecPerchirpMemIntrp, xw_return1);
    xw_return += xw_return1;

    /*!
     * -# Release Reset to frame timer
     */
    w_regValue = w_appssRegValue & ~(M_FE_SENS_FT_RESET_REG_MASK);
    M_REG32_WRITE(c_devIndex, p_regFtResetReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Clear CT r_ChirpTimerTrig register and go back to default state */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpTimerTrig.b32_Reg);
    w_regValue = (UINT32)0U;
    M_REG32_WRITE(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Chirp Timer (CT) status get API driver function
 *
 * @b Description @n
 * This Sensor API driver function gets the status of chirp timer
 *
 * @b Assumption: The Sensor cfg is done before issuing this API.
 *
 * @param[in]  c_devIndex    - Device Index
 * @param[out] p_ctSensSts   - pointer to Status params data @ref T_FE_SENS_CT_STATUS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-460, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ctSensorStatus(UINT8 c_devIndex, \
                T_FE_SENS_CT_STATUS *p_ctSensSts)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_regValue;
    T_CHIRPTIMER_REGS *p_structChirpTimer = M_FE_SENS_CT_REG_START_ADDRESS;
    REG32 *p_regCtReg;

    /*! -# Read chirp timer chirp count */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_ChirpCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return);
    p_ctSensSts->w_ChirpCount = w_regValue;

    /*! -# Read chirp timer burst count */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_BurstCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctSensSts->h_BurstCount = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read chirp timer burst timer value */
    p_regCtReg  = (REG32*)(&p_structChirpTimer->r_BurstPeriodTimer.b32_Reg);
    M_REG32_READ(c_devIndex, p_regCtReg, w_regValue, xw_return1);
    p_ctSensSts->w_BurstPeriodTimerVal = w_regValue;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS sensor Frame Timer (FT) status get API driver function
 *
 * @b Description @n
 * This Sensor API driver function gets the status of frame timer
 *
 * @b Assumption: The Sensor cfg is done before issuing this API.
 *
 * @param[in]  c_devIndex    - Device Index
 * @param[out] p_ftSensSts   - pointer to Status params data @ref T_FE_SENS_FT_STATUS
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-461, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE sens_ftSensorStatus(UINT8 c_devIndex, \
                T_FE_SENS_FT_STATUS *p_ftSensSts)
{
    T_RETURNTYPE xw_return,
                 xw_return1;
    UINT32 w_regValue;
    T_FRAMETIMER_REGS *p_structFrameTimer = M_FE_SENS_FT_REG_START_ADDRESS;
    REG32 *p_regFtReg;

    /*! -# Read frame timer frame start / stop status */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_TrigStopDone.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return);
    p_ftSensSts->c_FrameStartStopStatus = (UINT8)w_regValue;

    /*! -# Read frame timer frame count */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameCnt.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftSensSts->h_FrameCount = (UINT16)w_regValue;
    xw_return += xw_return1;

    /*! -# Read frame timer reference timer value */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FrameRefTimer.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftSensSts->w_FrameRefTimerVal = w_regValue;
    xw_return += xw_return1;

    /*! -# Read frame timer frame counter value */
    p_regFtReg  = (REG32*)(&p_structFrameTimer->r_FramePeriodTimer.b32_Reg);
    M_REG32_READ(c_devIndex, p_regFtReg, w_regValue, xw_return1);
    p_ftSensSts->w_FramePeriodTimerVal = w_regValue;
    xw_return += xw_return1;

    /*! -# Return the status xw_return */
    return xw_return;
}


/* End of group */
/*! @} */

/*
 * END OF fe_sensdriver.c
 */
