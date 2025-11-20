/*!*****************************************************************************
 * @file fe_sensor.c
 *
 * @brief FECSSLib Sensor Module API functions
 *
 * @b Description @n
 * This file provides FECSS library Sensor Module API function services to mmWaveLink
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
 * @addtogroup FECSSLIB_SENSOR_API FECSSLib Sensor Control driver API functions
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
 * @name FECSS Library Sensor module API Driver static function calls
 * @{
 */

/*! @} */

/******************************FECSSLib Sensor Driver APIs*********************/

/*!*****************************************************************************
 * @brief FECSS Sensor Profile common configuration API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function configures the FMCW radar chirp profile common
 * parameters like sample rate, num of samples, ramp end time, TX/RX gain etc.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_CHIRP_PROF_COMN_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-428, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensChirpProfComnCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    U_FE_SENS_RX_DFE_REG u_dfeParams = {0};
    T_FE_SENS_CT_COMN_PROFILE z_ctComnParams = {0};
    T_FE_API_RFS_CHIRP_PROFILE_CMD z_rfsChirpProfParms = {0};

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if ((p_cmdData->c_DigOutputSampRate < M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_12P5M) || \
            (p_cmdData->c_DigOutputSampRate > M_RL_SENS_DIG_OUT_SAMP_RATE_MIN_1M))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_SAMP_RATE error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_SAMP_RATE;
        }
        else if (p_cmdData->c_DigOutputBitsSel > M_RL_SENS_DIG_OUT_16BITS)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_DIG_BITS error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_DIG_BITS;
        }
        else if (p_cmdData->c_DfeFirSel > M_RL_SENS_DFE_FIR_SHORT_FILT)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_DFEFIR_SEL error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_DFEFIR_SEL;
        }
        else if (p_cmdData->c_VcoMultiChipMode > M_RL_SENS_VCO_MULT_CHIP_SINGLE)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_VCO_MC_SEL error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_VCO_MC_SEL;
        }
        else if ((p_cmdData->h_NumOfAdcSamples < M_RL_SENS_NUM_ADC_SAMPL_MIN) || \
                (p_cmdData->h_NumOfAdcSamples > M_RL_SENS_NUM_ADC_SAMPL_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_NUMADC_SAMP error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_NUMADC_SAMP;
        }
        else if ((p_cmdData->c_ChirpTxMimoPatSel != M_RL_SENS_TX_MIMO_PATRN_DIS) && \
                (p_cmdData->c_ChirpTxMimoPatSel != M_RL_SENS_TX_MIMO_PATRN_2TX_TDMA) && \
                (p_cmdData->c_ChirpTxMimoPatSel != M_RL_SENS_TX_MIMO_PATRN_2TX_BPM))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_TX_MIMO error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_TX_MIMO;
        }
        else if ((p_cmdData->h_ChirpRampEndTime < M_RL_SENS_RAMP_END_TIME_MIN) || \
                (p_cmdData->h_ChirpRampEndTime > M_RL_SENS_RAMP_END_TIME_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_RAMPEND error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_RAMPEND;
        }
        else if (p_cmdData->c_ChirpRxHpfSel > M_RL_SENS_RX_HPF_SEL_2800KHZ)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_RAMPEND error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_HPF_SEL;
        }
        else if ((p_cmdData->c_HpfFastInitDuration < M_RL_SENS_HPF_FAST_INIT_DUR_MIN) || \
                (p_cmdData->c_HpfFastInitDuration > M_RL_SENS_HPF_FAST_INIT_DUR_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_HPF_FI_TIME error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_HPF_FI_TIME;
        }
        else if (p_cmdData->h_CrdNSlopeMag > M_RL_SENS_CRD_NSLOPE_MAG_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_CRD_NSLOPE error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_CRD_NSLOPE;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Configure the chirp Digital Frontend (DFE) Params */
        u_dfeParams.bits.b8_DigSampRate = (UINT8)p_cmdData->c_DigOutputSampRate;
        u_dfeParams.bits.b12_NumSamps   = (UINT16)p_cmdData->h_NumOfAdcSamples;
        u_dfeParams.bits.b3_OutBitSel   = (UINT8)p_cmdData->c_DigOutputBitsSel;
        u_dfeParams.bits.b2_FirSel      = (UINT8)p_cmdData->c_DfeFirSel;
        xw_return = sens_configDfeParams(c_devIndex, &u_dfeParams);

        /*! -# Configure the chirp timer common profile config Params */
        z_ctComnParams.c_ChirpTxMimoPatSel = p_cmdData->c_ChirpTxMimoPatSel;
        z_ctComnParams.h_ChirpRampEndTime = p_cmdData->h_ChirpRampEndTime;
        z_ctComnParams.c_MiscSettings = p_cmdData->c_MiscSettings;
        z_ctComnParams.c_HpfFastInitDuration = p_cmdData->c_HpfFastInitDuration;
        z_ctComnParams.h_CrdNSlopeMag = p_cmdData->h_CrdNSlopeMag;
        xw_return1 = sens_configCtComnParams(c_devIndex, &z_ctComnParams);
        xw_return += xw_return1;

        /*! -# Configure the chirp profile RFS analog config Params */
        z_rfsChirpProfParms.c_DigOutputSampRate   = p_cmdData->c_DigOutputSampRate;
        z_rfsChirpProfParms.c_VcoMultiChipMode    = p_cmdData->c_VcoMultiChipMode;
        z_rfsChirpProfParms.c_ChirpRxHpfSel       = p_cmdData->c_ChirpRxHpfSel;
        xw_return1 = rfs_cmdChirpProfCfg(c_devIndex, &z_rfsChirpProfParms);
        xw_return += xw_return1;
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensChirpProfComnCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS Sensor Profile common configuration GET API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function get confguration of the FMCW radar chirp profile common
 * parameters like sample rate, num of samples, ramp end time, TX/RX gain etc.
 *
 * @b Assumption: The FECSS RF ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData   - pointer to response data @ref T_RL_API_SENS_CHIRP_PROF_COMN_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-429, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensChirpProfComnCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_COMN_CFG *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    U_FE_SENS_RX_DFE_REG u_dfeParams = {0};
    T_FE_SENS_CT_COMN_PROFILE z_ctComnParams = {0};
    T_FE_API_RFS_CHIRP_PROFILE_CMD z_rfsChirpProfParms = {0};

    /*! -# getting configuration of the chirp Digital Frontend (DFE) Params */
    xw_return = sens_dfeParamsGet(c_devIndex, &u_dfeParams);

    /*! -# getting configuration of the chirp timer common profile config Params */
    xw_return1 = sens_ctComnParamsGet(c_devIndex, &z_ctComnParams);
    xw_return += xw_return1;

    /*! -# getting configuration of the chirp profile RFS analog config Params */
    xw_return1 = rfs_cmdChirpProfCfgGet(c_devIndex, &z_rfsChirpProfParms);
    xw_return += xw_return1;

    /*! -# Clear response structure */
    xw_return1 = fe_hwMemclr((UINT32*)p_resData, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG)/sizeof(UINT32));
    xw_return += xw_return1;

    if (M_DFP_RET_CODE_OK == xw_return)
    {
        p_resData->c_DigOutputSampRate  = (UINT8)u_dfeParams.bits.b8_DigSampRate;
        p_resData->h_NumOfAdcSamples    = (UINT16)u_dfeParams.bits.b12_NumSamps;
        p_resData->c_DigOutputBitsSel   = (UINT8)u_dfeParams.bits.b3_OutBitSel;
        p_resData->c_DfeFirSel          = (UINT8)u_dfeParams.bits.b2_FirSel;

        p_resData->c_ChirpTxMimoPatSel   =   z_ctComnParams.c_ChirpTxMimoPatSel;
        p_resData->h_ChirpRampEndTime    =   z_ctComnParams.h_ChirpRampEndTime;
        p_resData->c_MiscSettings        =   z_ctComnParams.c_MiscSettings;
        p_resData->c_HpfFastInitDuration =   z_ctComnParams.c_HpfFastInitDuration;
        p_resData->h_CrdNSlopeMag        =   z_ctComnParams.h_CrdNSlopeMag;

        p_resData->c_VcoMultiChipMode   = z_rfsChirpProfParms.c_VcoMultiChipMode;
        p_resData->c_ChirpRxHpfSel      = z_rfsChirpProfParms.c_ChirpRxHpfSel;

    }
    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensChirpProfComnCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Profile time configuration API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function configures the FMCW radar chirp profile timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-430, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensChirpProfTimeCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    UINT16 h_adcStartFract = (p_cmdData->h_ChirpAdcStartTime & \
                                M_RL_SENS_CHIRP_ADCSTART_TIME_FRAC_MASK);
    UINT8 c_adcSkipSamp = (UINT8)((UINT16)(p_cmdData->h_ChirpAdcStartTime) >> \
                                (UINT16)M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET);
    UINT8 c_devType = p_feDrvData->z_ClientCbData.c_DeviceType;
    UINT8 c_ctResolType,
          c_ctFreqResol;
    UINT32 w_rfFreqLow,
           w_rfFreqHigh;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        /*! -# Get chirp timer frequency resolution type */
        xw_return1 = sens_getCtResolution(c_devIndex, &c_ctResolType);
        c_ctFreqResol = c_ctResolType & M_RL_FECSS_CT_FREQ_RESOL_MASK;

        /*! -# 77G devices error check */
        if (M_DFP_DEVICETYPE_1432 == c_devType)
        {
            /*! -# HR check */
            if ((UINT8)0U != c_ctFreqResol)
            {
                w_rfFreqLow  = M_RL_SENS_CHIRP_RFFREQ_HR_76G;
                w_rfFreqHigh = M_RL_SENS_CHIRP_RFFREQ_HR_81G;
            }
            /*! -# LR check */
            else
            {
                w_rfFreqLow  = M_RL_SENS_CHIRP_RFFREQ_LR_76G;
                w_rfFreqHigh = M_RL_SENS_CHIRP_RFFREQ_LR_81G;
            }
        }
        /*! -# 60G devices error check */
        else
        {
            /*! -# HR check */
            if ((UINT8)0U != c_ctFreqResol)
            {
                w_rfFreqLow  = M_RL_SENS_CHIRP_RFFREQ_HR_57G;
                w_rfFreqHigh = M_RL_SENS_CHIRP_RFFREQ_HR_64G;
            }
            /*! -# LR check */
            else
            {
                w_rfFreqLow  = M_RL_SENS_CHIRP_RFFREQ_LR_57G;
                w_rfFreqHigh = M_RL_SENS_CHIRP_RFFREQ_LR_64G;
            }
        }

        if (M_DFP_RET_CODE_OK != xw_return1)
        {
            xw_return = xw_return1;
        }
        else if ((p_cmdData->h_ChirpIdleTime < M_RL_SENS_CHIRP_IDLE_TIME_MIN) || \
            (p_cmdData->h_ChirpIdleTime > M_RL_SENS_CHIRP_IDLE_TIME_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_IDLE_TIME error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_IDLE_TIME;
        }
        else if (h_adcStartFract > M_RL_SENS_CHIRP_ADCSTART_TIME_FRAC_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_ADCFRAC_TIME error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_ADCFRAC_TIME;
        }
        else if (c_adcSkipSamp > (UINT8)M_RL_SENS_CHIRP_ADC_SKIP_SAMP_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_ADCSKIP_SAMP error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_ADCSKIP_SAMP;
        }
        else if ((p_cmdData->xh_ChirpTxStartTime < M_RL_SENS_CHIRP_TXSTART_TIME_MIN) || \
            (p_cmdData->xh_ChirpTxStartTime > M_RL_SENS_CHIRP_TXSTART_TIME_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_TXSTART_TIME error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_TXSTART_TIME;
        }
        else if ((p_cmdData->xh_ChirpRfFreqSlope < M_RL_SENS_CHIRP_RFSLOPE_TIME_MIN) || \
            (p_cmdData->xh_ChirpRfFreqSlope > M_RL_SENS_CHIRP_RFSLOPE_TIME_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_RF_SLOPE error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_RF_SLOPE;
        }
        /*! -# RF freq error check */
        else if ((p_cmdData->w_ChirpRfFreqStart < w_rfFreqLow) || \
                (p_cmdData->w_ChirpRfFreqStart > w_rfFreqHigh))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_TXBPM_EN error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_RF_FREQ;
        }
        else if (p_cmdData->h_ChirpTxEnSel > M_RL_SENS_CHIRP_TX_EN_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_TX_EN error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_TX_EN;
        }
        else if (p_cmdData->h_ChirpTxBpmEnSel > M_RL_SENS_CHIRP_TXBPM_EN_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_TXBPM_EN error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_TXBPM_EN;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Configure the chirp Digital Frontend (DFE) Params */
        xw_return = sens_configCtTimeParams(c_devIndex, p_cmdData);
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensChirpProfTimeCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Profile time configuration Get API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function get the configuration of the FMCW radar chirp profile timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData   - pointer to response data @ref T_RL_API_SENS_CHIRP_PROF_TIME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-431, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensChirpProfTimeCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    xw_return = sens_ctTimeParamsGet(c_devIndex, p_resData);

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensChirpProfTimeCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Per chirp time param configuration API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function configures the FMCW radar per chirp timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable array length and repeat count.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-432, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensPerChirpCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return = M_DFP_RET_CODE_OK;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    UINT8 c_perChirpPrm;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        /*! -# Check the length and rpt count error check for all per chirp params */
        c_perChirpPrm = M_RL_SENS_PER_CHIRP_FREQ_START;

        while ((c_perChirpPrm < M_RL_SENS_PER_CHIRP_MAX_PARAMS) && (xw_return == M_DFP_RET_CODE_OK))
        {
            if ((p_cmdData->h_ParamArrayLen[c_perChirpPrm] < \
                    M_RL_SENS_PER_CHIRP_ARRAY_LEN_MIN) || \
                (p_cmdData->h_ParamArrayLen[c_perChirpPrm] > M_RL_SENS_PER_CHIRP_ARRAY_LEN_MAX))
            {
                /*! -# Return M_DFP_RET_CODE_SENS_INVAL_PERCHRP_LEN error code */
                xw_return = M_DFP_RET_CODE_SENS_INVAL_PERCHRP_LEN;
            }
            else if ((p_cmdData->h_ParamRptCount[c_perChirpPrm] < \
                    M_RL_SENS_PER_CHIRP_RPT_COUNT_MIN) || \
                (p_cmdData->h_ParamRptCount[c_perChirpPrm] > M_RL_SENS_PER_CHIRP_RPT_COUNT_MAX))
            {
                /*! -# Return M_DFP_RET_CODE_SENS_INVAL_PERCHRP_RPCNT error code */
                xw_return = M_DFP_RET_CODE_SENS_INVAL_PERCHRP_RPCNT;
            }
            else
            {
                /* No Update */
            }

            c_perChirpPrm++;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Configure the per chirp HW Params */
        xw_return = sens_configCtPerChirpHw(c_devIndex, p_cmdData);
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensPerChirpCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}


/*!*****************************************************************************
 * @brief FECSS Sensor Per chirp time param configuration Get API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function get configuration of the FMCW radar per chirp timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable array length and repeat count.
 *
 * @b Assumption: The chirp profile common cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData   - pointer to response data @ref T_RL_API_SENS_PER_CHIRP_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-433, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensPerChirpCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    xw_return = sens_ctPerChirpHwCfgGet(c_devIndex, p_resData);

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensPerChirpCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Per chirp time param control API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function controls (ON/OFF) the FMCW radar per chirp timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable and configures the array address.
 *
 * @b Assumption: The per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-434, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensPerChirpCtrl_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    UINT8 c_perChirpPrm;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        /*! -# Check the address error check for all per chirp params */
        for (c_perChirpPrm = M_RL_SENS_PER_CHIRP_FREQ_START; \
                c_perChirpPrm < M_RL_SENS_PER_CHIRP_MAX_PARAMS; c_perChirpPrm++)
        {
            if (((UINT16)0U != \
                (p_cmdData->h_ParamArrayStartAdd[c_perChirpPrm] & \
                    M_RL_SENS_PER_CHIRP_LUT_4BYTE_ADD_MASK)) || \
                (p_cmdData->h_ParamArrayStartAdd[c_perChirpPrm] > \
                    M_RL_SENS_PER_CHIRP_LUT_STRT_ADDR_MAX))
            {
                /*! -# Return M_DFP_RET_CODE_SENS_INVAL_PERCHRP_ADDR error code */
                xw_return = M_DFP_RET_CODE_SENS_INVAL_PERCHRP_ADDR;
                break;
            }
            else
            {
                xw_return = M_DFP_RET_CODE_OK;
            }
        }

        /*! -# If error check is success then error check control param */
        if ((p_cmdData->h_PerChirpParamCtrl > M_RL_SENS_PER_CHIRP_CTRL_MAX) && \
            (M_DFP_RET_CODE_OK == xw_return))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_PERCHRP_CTRL error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_PERCHRP_CTRL;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Control the per chirp HW Params */
        xw_return = sens_ctrlCtPerChirpHw(c_devIndex, p_cmdData);
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensPerChirpCtrl API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Per chirp time param control Get API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function Get the control params of (ON/OFF) the FMCW radar per chirp timing
 * parameters like idle time, ADC start time, TX start time, RF Slope, RF start freq,
 * TX and BPM enable and configures the array address.
 *
 * @b Assumption: The per chirp cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData   - pointer to response data @ref T_RL_API_SENS_PER_CHIRP_CTRL
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-435, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensPerChirpCtrlGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    xw_return = sens_ctPerChirpHwCtrlGet(c_devIndex, p_resData);

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensPerChirpCtrlGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Frame configuration API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function configures the FMCW radar chirp, burst and frame
 * configuration parameters like num of chirps, num of chirp accumulation, Burst period,
 * num of bursts, Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The chirp profile common & time cfg are done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENS_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-436, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensFrameCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_SENS_FRAME_CFG *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    T_FE_SENS_CT_BURST_CFG z_ctBurstCfg;
    T_FE_SENS_FT_FRAME_CFG z_ftFrameCfg;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if ((p_cmdData->h_NumOfChirpsInBurst < M_RL_SENS_NUM_CHIRPS_MIN) || \
            (p_cmdData->h_NumOfChirpsInBurst > M_RL_SENS_NUM_CHIRPS_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_NUM_CHIRPS error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_NUM_CHIRPS;
        }
        else if (((UINT8)1U == p_cmdData->c_NumOfChirpsAccum) || \
            (p_cmdData->c_NumOfChirpsAccum > M_RL_SENS_NUM_CHIRPS_ACCUM_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_NUM_ACCUM error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_NUM_ACCUM;
        }
        else if ((p_cmdData->w_BurstPeriodicity < M_RL_SENS_BURST_PERIOD_MIN) || \
            (p_cmdData->w_BurstPeriodicity > M_RL_SENS_BURST_PERIOD_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_BURST_PERD error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_BURST_PERD;
        }
        else if ((p_cmdData->h_NumOfBurstsInFrame < M_RL_SENS_NUM_BURSTS_MIN) || \
            (p_cmdData->h_NumOfBurstsInFrame > M_RL_SENS_NUM_BURSTS_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_NUM_BURST error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_NUM_BURST;
        }
        else if ((p_cmdData->w_FramePeriodicity < M_RL_SENS_FRAME_PERIOD_MIN) || \
            (p_cmdData->w_FramePeriodicity > M_RL_SENS_FRAME_PERIOD_MAX))
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_FRAME_PERD error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_FRAME_PERD;
        }
        else if ((p_cmdData->h_NumOfFrames) > M_RL_SENS_NUM_FRAMES_MAX)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_NUM_FRAME error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_NUM_FRAME;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Configure the chirp timer (CT) chirp and burst params */
        z_ctBurstCfg.h_NumOfChirpsInBurst = p_cmdData->h_NumOfChirpsInBurst;
        z_ctBurstCfg.c_NumOfChirpsAccum   = p_cmdData->c_NumOfChirpsAccum;
        z_ctBurstCfg.w_BurstPeriodicity   = p_cmdData->w_BurstPeriodicity;
        z_ctBurstCfg.h_NumOfBurstsInFrame = p_cmdData->h_NumOfBurstsInFrame;
        xw_return = sens_configCtBurstParams(c_devIndex, &z_ctBurstCfg);

        /*! -# Configure the Frame timer (FT) frame Params if the skip frame config is disabled */
        if ((UINT8)0U == (p_cmdData->c_MiscSetupMask & M_RL_SENS_FRAME_SKIP_FT_CONFIG))
        {
            z_ftFrameCfg.w_FramePeriodicity   = p_cmdData->w_FramePeriodicity;
            z_ftFrameCfg.h_NumOfFrames        = p_cmdData->h_NumOfFrames;
            z_ftFrameCfg.w_FrameEvent0TimeCfg = p_cmdData->w_FrameEvent0TimeCfg;
            z_ftFrameCfg.w_FrameEvent1TimeCfg = p_cmdData->w_FrameEvent1TimeCfg;
            xw_return1 = sens_configFtFrameParams(c_devIndex, &z_ftFrameCfg);
            xw_return += xw_return1;
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensFrameCfg API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Frame configuration Get API driver function
 *
 * @b Description @n
 * This FECSS sensor  API driver function get configuration of the FMCW radar chirp, burst and frame
 * configuration parameters like num of chirps, num of chirp accumulation, Burst period,
 * num of bursts, Frame period, num of frames and frame event configurations.
 *
 * @b Assumption: The chirp profile common & time cfg are done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_SENS_FRAME_CFG
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-437, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensFrameCfgGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENS_FRAME_CFG *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    T_FE_SENS_CT_BURST_CFG z_ctBurstCfg = {0};
    T_FE_SENS_FT_FRAME_CFG z_ftFrameCfg = {0};

    xw_return = sens_ctBurstParamsGet(c_devIndex, &z_ctBurstCfg);
    xw_return += sens_ftFrameParamsGet(c_devIndex, &z_ftFrameCfg);

    xw_return += fe_hwMemclr((UINT32*)p_resData, sizeof(T_RL_API_SENS_FRAME_CFG)/sizeof(UINT32));

    if (xw_return == M_DFP_RET_CODE_OK)
    {
        /*! -# Read Configuration the chirp timer (CT) chirp and burst params */
        p_resData->h_NumOfChirpsInBurst   = z_ctBurstCfg.h_NumOfChirpsInBurst;
        p_resData->c_NumOfChirpsAccum     = z_ctBurstCfg.c_NumOfChirpsAccum;
        p_resData->w_BurstPeriodicity     = z_ctBurstCfg.w_BurstPeriodicity;
        p_resData->h_NumOfBurstsInFrame   = z_ctBurstCfg.h_NumOfBurstsInFrame;

        /*! -# Read Configuration the Frame timer (FT) frame Params */
        p_resData->w_FramePeriodicity     = z_ftFrameCfg.w_FramePeriodicity;
        p_resData->h_NumOfFrames          = z_ftFrameCfg.h_NumOfFrames;
        p_resData->w_FrameEvent0TimeCfg   = z_ftFrameCfg.w_FrameEvent0TimeCfg;
        p_resData->w_FrameEvent1TimeCfg   = z_ftFrameCfg.w_FrameEvent1TimeCfg;
    }
    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensFrameCfgGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor start API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function configures and starts the FMCW radar frames,
 * the configuration parameters like frame trigger mode, frame start signal LB enable and
 * frame start timer value.
 *
 * @b Assumption: The frame cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENSOR_START_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-438, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensorStart_rom(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    T_FE_API_RFS_CW_SENSOR_START_CMD z_rfsCwSensStartCfg = {0};
    T_FE_API_RFS_LIVE_MON_SENS_START_CMD z_rfsSensStartCfg = {0};

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if (p_cmdData->c_FrameTrigMode > M_RL_SENS_FRAME_CT_OVRD_TRIG)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_FRAME_TRIG error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_FRAME_TRIG;
        }
        else if (p_cmdData->c_ChirpStartSigLbEn > M_RL_SENS_CT_START_SIG_LB_ENA)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_CT_START_LB error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_CT_START_LB;
        }
        else if (p_cmdData->c_FrameLivMonEn > M_RL_SENS_LIVE_MON_MAX_ENA_MASK)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_LIVE_MON_CFG error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_LIVE_MON_CFG;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Configure the RFS sensor start live monitor config Params */
        z_rfsSensStartCfg.c_FrameLivMonEn = p_cmdData->c_FrameLivMonEn;
        xw_return = rfs_cmdSensStartCfg(c_devIndex, &z_rfsSensStartCfg);

        /*!
         * -# Configure the Frame timer (FT) Frame start params if trigger
         * mode 0 to 3
         */
        if (p_cmdData->c_FrameTrigMode <= M_RL_SENS_FRAME_HW_LOW_JIT_TRIG)
        {
            xw_return += sens_ftSensorStart(c_devIndex, p_cmdData);
        }
        /*!
         * -# Configure the RFS test mode CW and CR override mode if trigger
         * mode is 4 and 5
         */
        else
        {
            z_rfsCwSensStartCfg.c_SensorTrigMode = p_cmdData->c_FrameTrigMode;
            xw_return += rfs_cmdCwSensorStart(c_devIndex, &z_rfsCwSensStartCfg);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensorStart API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Stop API driver function
 *
 * @b Description @n
 * This FECSS sensor API driver function stops the FMCW radar frames as per input mode.
 * The configuration parameters frame stop mode is used to stop frame or CW tone or
 * forceful error recovery stop.
 *
 * @b Assumption: The frame cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_SENSOR_STOP_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-439, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensorStop_rom(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_STOP_CMD *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    T_FE_API_RFS_CW_SENSOR_STOP_CMD z_rfsCwSensStopCfg = {0};

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if (p_cmdData->c_FrameStopMode > M_RL_SENS_FORCE_STOP_FRAME)
        {
            /*! -# Return M_DFP_RET_CODE_SENS_INVAL_FRAME_TRIG error code */
            xw_return = M_DFP_RET_CODE_SENS_INVAL_FRAME_STOP;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then configure the sensor */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*!
         * -# Stop the Frame timer (FT) at frame boundary if mode is 0
         */
        if (M_RL_SENS_STOP_FRAME == p_cmdData->c_FrameStopMode)
        {
            xw_return = sens_ftSensorStop(c_devIndex);
        }
        /*!
         * -# else call RFS CW stop command and sensor force stop timers
         */
        else
        {
            /*! -# if mode is force frame stop call sens_ftSensorForceStop to reset all timers */
            if (M_RL_SENS_FORCE_STOP_FRAME == p_cmdData->c_FrameStopMode)
            {
                xw_return = sens_ftSensorForceStop(c_devIndex);
            }

            /*! -# Call rfs_cmdCwSensorStop to stop all the sensor activities in RFS */
            z_rfsCwSensStopCfg.c_SensorStopMode = p_cmdData->c_FrameStopMode;
            xw_return += rfs_cmdCwSensorStop(c_devIndex, &z_rfsCwSensStopCfg);
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensorStart API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS Sensor Status get API driver function
 *
 * @b Description @n
 * This FECSS sensor Status Get API driver function reads the status of Radar
 * Sensor frame and chrip timers. This API provides chirp and frame timer chirp,
 * burst and frame counts.
 *
 * @b Assumption: The sensor cfg is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_resData   - pointer to response data @ref T_RL_API_SENSOR_STATUS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-440, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_sensorStatusGet_rom(UINT8 c_devIndex, \
                T_RL_API_SENSOR_STATUS_RSP *p_resData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_SENS_CT_STATUS z_ctStatus = {0};
    T_FE_SENS_FT_STATUS z_ftStatus = {0};

    /*!
     * -# Get chirp timer Status
     */
    xw_return = sens_ctSensorStatus(c_devIndex, &z_ctStatus);
    p_resData->h_BurstCount          = z_ctStatus.h_BurstCount;
    p_resData->w_ChirpCount          = z_ctStatus.w_ChirpCount;
    p_resData->w_BurstPeriodTimerVal = z_ctStatus.w_BurstPeriodTimerVal;

    /*!
     * -# Get frame timer Status
     */
    xw_return1 = sens_ftSensorStatus(c_devIndex, &z_ftStatus);
    xw_return += xw_return1;

    p_resData->h_FrameCount              = z_ftStatus.h_FrameCount;
    p_resData->c_FrameStartStopStatus    = z_ftStatus.c_FrameStartStopStatus;
    p_resData->w_FrameRefTimerVal        = z_ftStatus.w_FrameRefTimerVal;
    p_resData->w_FramePeriodTimerVal     = z_ftStatus.w_FramePeriodTimerVal;

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_sensorStatusGet API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_sensor.c
 */
