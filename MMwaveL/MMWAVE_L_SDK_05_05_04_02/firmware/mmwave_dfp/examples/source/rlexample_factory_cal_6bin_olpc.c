/*!*****************************************************************************
 * @file rlexample_factory_cal_6bin_olpc.c
 *
 * @brief mmWave DFP Low Factory Calibration Example
 *
 * @b Description @n
 * This example demonstrates factory calibration with 6-bin OLPC Tx calibration configuration
 * sequence for mmWaveLow devices
 *
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
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     15Mar2023   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */

#include <mmwavelink/mmwavelink.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <mmwavelink/include/rl_monitor.h>

#include <examples/include/rlexample.h>

/*******************************************************************************
 * TYPE DEFINITIONS
 *******************************************************************************
 */

typedef struct
{
    UINT8   c_BackOffMap[M_DFP_MAX_TX_CHANNELS];
    UINT16  h_TxCodes[M_DFP_MAX_TX_CHANNELS];
} T_RL_EXAMPLE_TX_OLPC_BIN;

/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/*!*****************************************************************************
 * @brief Factory Calibration 6-bin OLPC Example
 *
 * @b Description @n
 * This example runs factory calibration with temperature override to generate Tx
 * coefficients for 6 bins. Application can apply these coefficients to Tx channels in runtime
 * using Tx-CLPC runtime calibration API in override mode.
 *
 * 6 temperature bins are defined by dividing -40⁰C to 140⁰C range in 6 uniform sections:
 *
 * | Bin Index | Temperature Range | Mid Point |
 * |-----------|-------------------|-----------|
 * | 0         | -40 to -10        |   -25     |
 * | 1         | -10 to  20        |     5     |
 * | 2         |  20 to  50        |    35     |
 * | 3         |  50 to  80        |    65     |
 * | 4         |  80 to 110        |    95     |
 * | 5         | 110 to 140        |   125     |
 *
 * @param[in]   c_deviceType    : mmWave DFP Device Type [ @ref M_DFP_DEVICETYPE_6432, @ref M_DFP_DEVICETYPE_1432 ]
 * @param[in]   h_numLoops      : Number of test loops
 * @param[out]  xw_errorCode    : Error-code in case of failure
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
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
T_RETURNTYPE rlExample_factoryCalibOlpcLoop(
                UINT8                               c_deviceType,
                UINT16                              h_numLoops,
                const T_DFP_CLIENT_CB_DATA*         p_refLinkCbData)
{
    /** -# T_RETURNTYPE type is SINT32 value */
    T_RETURNTYPE    xw_errorCode    = M_DFP_RET_CODE_OK;
    UINT8           c_logEnable     = M_ENABLE;
    UINT16          h_loopIndex     = 0U;

    /** -# Setup device index and device map for SoC platform */
    UINT32          w_deviceMap     = M_DFP_DEVICE_MAP_DEVICE_0;
    UINT8           c_deviceIndex   = 0U;


    /** -# Initialize mmWaveLink using @ref rl_mmWaveLinkInit API
     *      - Update @ref T_DFP_CLIENT_CB_DATA according to the selected device
     */
    T_DFP_CLIENT_CB_DATA    z_mmWaveLinkClientCbData = *p_refLinkCbData;
    z_mmWaveLinkClientCbData.c_DeviceType = c_deviceType;

    xw_errorCode = rl_mmWaveLinkInit(w_deviceMap, z_mmWaveLinkClientCbData);
    if(xw_errorCode != M_DFP_RET_CODE_OK)
    {
        rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_mmWaveLinkInit failed with %d in iteration %d\n",
                xw_errorCode, h_loopIndex);
    }

    T_RL_EXAMPLE_TX_OLPC_BIN z_txOlpcLut[6U] = {0};

    /** -# @b FOR required number of iterations: <OL type="a"> */
    while ((h_loopIndex < h_numLoops) && (M_DFP_RET_CODE_OK == xw_errorCode))
    {
        /** <LI> Power-on FECSS using @ref rl_fecssDevPwrOn */
        T_RL_API_FECSS_DEV_PWR_ON_CMD z_fecssDevOnApiData =
        {
            .h_XtalClkFreq      = M_RL_FECSS_XTAL_CLK_FREQ_40M,     /**< 40 MHz Crystal */
            .c_ClkSourceSel     = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS running on 80MHz fast clock */
            # if defined(_MODE_INSTR_)
            .c_PowerMode        = M_RL_FECSS_PWR_ON_MODE_WARM,      /**< Warm boot mode */
            # else
            .c_PowerMode        = M_RL_FECSS_PWR_ON_MODE_COLD,      /**< Cold boot mode */
            # endif
            .c_FecBootCfg       = 0U,                               /**< Enable boot-time self-tests */
            .c_ChirpTimerResol  = M_RL_FECSS_CHIRP_TIMER_RES_11,    /**< Frequency and time in high resolution */
        };

        xw_errorCode = rl_fecssDevPwrOn(c_deviceIndex, &z_fecssDevOnApiData);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssDevPwrOn failed with %d in iteration %d\n",
                xw_errorCode, h_loopIndex);
            break;
        }

        /** <LI> Configure RF front end enables with @ref rl_fecssRfPwrOnOff */
        T_RL_API_FECSS_RF_PWR_CFG_CMD z_rfsRfpwrOn =
        {
            .h_RxChCtrlBitMask = (1U << M_RL_MAX_RX_CHANNELS) - 1U,
            .h_TxChCtrlBitMask = (1U << M_RL_MAX_TX_CHANNELS) - 1U,
            .c_MiscCtrl        = (1U << M_RL_RF_MISC_CTRL_RDIF_CLK),
        };

        xw_errorCode = rl_fecssRfPwrOnOff(c_deviceIndex, &z_rfsRfpwrOn);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfPwrOnOff power on failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Enable APLL using @ref rl_fecssDevClockCtrl API */
        T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOn =
        {
            .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
            .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
            .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_ON_CAL       /**< Calibrate and turn-on the APLL */
        };
        xw_errorCode = rl_fecssDevClockCtrl(c_deviceIndex, &z_fecssClkOn);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssDevClockCtrl APLL on failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Calculate mid-band frequency of the functional chirp and select calibration slope */
        UINT16 h_calFreqCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ?
            ((M_RL_SENS_CHIRP_RFFREQ_LR_57G + M_RL_SENS_CHIRP_RFFREQ_LR_64G) / 2U) :
            ((M_RL_SENS_CHIRP_RFFREQ_LR_76G + M_RL_SENS_CHIRP_RFFREQ_LR_81G) / 2U);

        UINT16 xh_calSlopeCode = (M_DFP_DEVICETYPE_6432 == c_deviceType) ? (UINT16)0x4DU : (UINT16)0x3A;

        UINT8  c_rxGainSel = 38U;

        /** <LI> Setup factory calibration API configuration @ref T_RL_API_FECSS_RF_FACT_CAL_CMD
         * with just temperature override enabled  */
        T_RL_API_FECSS_RF_FACT_CAL_CMD z_fecssFactCalCtrl =
        {
            .h_CalCtrlBitMask       = 0xCE,                 /**< Enable Synth, PD, LODIST, Rx and Tx calibrations */
            .c_MiscCalCtrl          = M_RL_FECSS_CAL_TX_CAL_TEMP_OVERRIDE,      /**< Run all calibration with Tx calibration temperature override */
            .h_CalRfFreq            = h_calFreqCode,        /**< Lowest frequency (Low Resolution) */
            .xh_CalRfSlope          = xh_calSlopeCode,      /**< Recommended calibration slope */
            .c_CalRxGainSel         = c_rxGainSel,          /**< Rx gain 38dB Gain */
            .c_CalTxBackOffSel[M_RL_TX_CHANNEL_0] = 20U,    /**< Tx0: 10dB backoff from target power */
            .c_CalTxBackOffSel[M_RL_TX_CHANNEL_1] = 20U,    /**< Tx1: 10dB backoff from target power */
            .c_TxPwrCalTxEnaMask[M_RL_TX_CHANNEL_0] = 0x3U, /**< Enable Tx0 and Tx1 for Tx0 calibration */
            .c_TxPwrCalTxEnaMask[M_RL_TX_CHANNEL_1] = 0x3U, /**< Copy data from Tx0 for Tx1 */
            .xc_CalTempBinOverrides =
            {
                [0U]    = -12,
                [1U]    =   2,
                [2U]    =  17,
            },
        };

        T_RL_API_FECSS_RF_FACT_CAL_RSP  z_fecssFactCalRsp;
        T_RL_API_FECSS_FACT_CAL_DATA    z_factoryCalData = {0};

        /** <LI> Run factory calibration using @ref rl_fecssRfFactoryCal API */
        xw_errorCode = rl_fecssRfFactoryCal(c_deviceIndex, &z_fecssFactCalCtrl, &z_fecssFactCalRsp);

        /** <LI> Check calibration status */
        if ((M_DFP_RET_CODE_OK == xw_errorCode) && ((UINT8)0xCE != z_fecssFactCalRsp.h_CalRunStatus))
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: Factory calibrations failed! Run Status: %d in iteration %d\n",
                z_fecssFactCalRsp.h_CalRunStatus, h_loopIndex);
            xw_errorCode = M_DFP_RET_CODE_FATAL_ERROR;
        }
        else if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfFactoryCal failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
        }

        /** <LI> Get calibration data using @ref rl_fecssRfFactoryCalDataGet */
        xw_errorCode = rl_fecssRfFactoryCalDataGet(c_deviceIndex, &z_factoryCalData);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfFactoryCalDataGet failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Store Tx data for first three bins to the LUT */
        for (UINT8 c_binIndex = 0U; c_binIndex < 3U; c_binIndex++)
        {
            for (UINT8 c_txIndex = 0U; c_txIndex < M_DFP_MAX_TX_CHANNELS; c_txIndex++)
            {
                z_txOlpcLut[c_binIndex].c_BackOffMap[c_txIndex] = z_factoryCalData.z_RxTxCalData.c_CalTxBackOffMap[c_txIndex];
                z_txOlpcLut[c_binIndex].h_TxCodes[c_txIndex]    = z_factoryCalData.z_RxTxCalData.h_CalTxBiasCodes[c_txIndex][c_binIndex];
            }
        }

        /** <LI> Run factory calibration using @ref rl_fecssRfFactoryCal API with skip Tx cal and
         * temperature override enabled for rest of the bins */
        z_fecssFactCalCtrl.h_CalCtrlBitMask = 1U << M_RL_FECSS_DEV_CAL_TX_PWR;
        z_fecssFactCalCtrl.h_CalCtrlBitMask = M_RL_FECSS_CAL_TX_CAL_TEMP_OVERRIDE | M_RL_FECSS_CAL_TX_CLPC_CAL_DIS;
        z_fecssFactCalCtrl.xc_CalTempBinOverrides[0U] = 32;
        z_fecssFactCalCtrl.xc_CalTempBinOverrides[1U] = 47;
        z_fecssFactCalCtrl.xc_CalTempBinOverrides[2U] = 62;
        xw_errorCode = rl_fecssRfFactoryCal(c_deviceIndex, &z_fecssFactCalCtrl, &z_fecssFactCalRsp);

        /** <LI> Check calibration status */
        if ((M_DFP_RET_CODE_OK == xw_errorCode) && (z_fecssFactCalCtrl.h_CalCtrlBitMask != z_fecssFactCalRsp.h_CalRunStatus))
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: Factory calibrations with overrides failed! Run Status: %d in iteration %d\n",
                z_fecssFactCalRsp.h_CalRunStatus, h_loopIndex);
            xw_errorCode = M_DFP_RET_CODE_FATAL_ERROR;
        }
        else if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfFactoryCal with overrides failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
        }

        /** <LI> Get Tx calibration data using @ref */
        T_RL_API_FECSS_RXTX_CAL_DATA z_rxTxCalData;
        xw_errorCode = rl_fecssRfRxTxCalDataGet(c_deviceIndex, &z_rxTxCalData);
        if (M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfRxTxCalDataGet failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
        }


        /** <LI> Store Tx data for last three bins to the LUT */
        for (UINT8 c_binIndex = 3U; c_binIndex < 6U; c_binIndex++)
        {
            for (UINT8 c_txIndex = 0U; c_txIndex < M_DFP_MAX_TX_CHANNELS; c_txIndex++)
            {
                z_txOlpcLut[c_binIndex].c_BackOffMap[c_txIndex] = z_factoryCalData.z_RxTxCalData.c_CalTxBackOffMap[c_txIndex];
                z_txOlpcLut[c_binIndex].h_TxCodes[c_txIndex]    = z_factoryCalData.z_RxTxCalData.h_CalTxBiasCodes[c_txIndex][c_binIndex - 3U];
            }
        }

        /** <LI> Turn-off RF enables using @ref rl_fecssRfPwrOnOff API */
        T_RL_API_FECSS_RF_PWR_CFG_CMD z_rfsRfpwrOff = {0};
        xw_errorCode = rl_fecssRfPwrOnOff(c_deviceIndex, &z_rfsRfpwrOff);

        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssRfPwrOnOff power off failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Turn-off APLL using @ref rl_fecssDevClockCtrl API */
                T_RL_API_FECSS_DEV_CLK_CTRL_CMD z_fecssClkOff =
        {
            .c_DevClkCtrl   = M_RL_FECSS_DEV_CLK_SRC_FCLK,      /**< FECSS core on fast clock */
            .c_FtClkCtrl    = M_RL_FECSS_FT_CLK_SRC_XTAL,       /**< Frame Timer on XTAL clock */
            .c_ApllClkCtrl  = M_RL_FECSS_APLL_CTRL_OFF           /**< Calibrate and turn-on the APLL */
        };
        xw_errorCode = rl_fecssDevClockCtrl(c_deviceIndex, &z_fecssClkOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssDevClockCtrl APLL off failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }

        /** <LI> Turn-off FECSS using @ref rl_fecssDevPwrOff API */
        T_RL_API_FECSS_DEV_PWR_OFF_CMD z_fecssDevOff =
        {
            # if defined(_MODE_INSTR_)
            .c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON,          /**< FECSS memory retention on */
            # else
            .c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_OFF,         /**< FECSS memory retention off */
            # endif
        };
        xw_errorCode = rl_fecssDevPwrOff(c_deviceIndex, &z_fecssDevOff);
        if(M_DFP_RET_CODE_OK != xw_errorCode)
        {
            rlExample_logger(c_logEnable, "ERROR: rlExample_factoryCalibOlpcLoop: rl_fecssDevPwrOff failed with %d in iteration %d\n",
                    xw_errorCode, h_loopIndex);
                break;
        }
        /** <LI> Wait for 10ms */
        rlExample_delayUs(10U * 1000U);

        if(M_DFP_RET_CODE_OK != xw_errorCode){break;}

        h_loopIndex++;
    } /** </OL> */

    /** -# Deinit mmWaveLink using @ref rl_mmWaveLinkDeInit */
    xw_errorCode += rl_mmWaveLinkDeInit(w_deviceMap);

    return xw_errorCode;
}

/*
 * END OF rlexample_factory_cal_6bin_olpc.c FILE
 */