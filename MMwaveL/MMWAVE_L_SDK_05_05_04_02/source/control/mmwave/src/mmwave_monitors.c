/**
 *   @file  mmwave_monitors.c
 *
 *   @brief
 *      The file implements the mmWave RF monitor module which interfaces
 *      with the mmWave Monitor API
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2023 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <control/mmwave/include/mmwave_internal.h>
#include <math.h>

/* Factors for configuration, refer to DFP documentation for these */
#define FREQERRTHRES80MHZ 80000
#define RFCHIRPMONSTARTTIMFACT 5
#define RFFREQERRTHRESFACT64xx 32.552
#define MAXSYNTHFREQERRFACT64xx 1.017
#define MAXSYNTHFREQERRFACT14xx 1.356
#define RFFREQERRTHRESFACT14xx 43.408
#define RFSTRTFREQFACT64xx 1.172
#define RFFREQSLOPEFACT64xx 28.61 
#define RFSTRTFREQFACT14xx 1.562
#define RFFREQSLOPEFACT14xx 28.61
#define RFSPLLVOLRES 0.00176 // 1LSB = 1.8 / 1024
#define RFSPMCLKDCVOLFACT 0.00703
#define RFSLOOPBACKPHASEFACT 0.00549 // 1LSB = 360 deg / 2^16
#define PERCHIRPLUTRAMARRAYBASEADDR 0x21880000
/**************************************************************************
 ***************************** Local Functions ****************************
 **************************************************************************/




/**************************************************************************
 ************************ mmWave Monitor Functions ***************************
 **************************************************************************/

int32_t MMWaveMon_enable(MMWave_MCB* ptrMMWaveMCB)
{
    int32_t retVal = -1;
    retVal = rl_monEnableTrig(M_DFP_DEVICE_INDEX_0, &(ptrMMWaveMCB->mmwaveMonitorCfg));
    return retVal;
}

int64_t MMWaveMon_getStatus(MMWave_MCB* ptrMMWaveMCB)
{
    uint64_t retVal = 0;
    /*Lower 32 Bits of Monitor status*/
    uint64_t retVal_l=0;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* status = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*Storing upper 32 Bits of Monitor status*/
    retVal = (uint64_t)(ptrMMWaveMCB->mmwaveMonitorCfg.w_MonitorEnable[1] & status->z_MonTrigReponse.w_MonitorStatus[1]);
    /*Shifting upper 32 Bits*/
    retVal = retVal <<32U;
    /*Storing lower 32 Bits of Monitor status*/
    retVal_l=(uint64_t)(ptrMMWaveMCB->mmwaveMonitorCfg.w_MonitorEnable[0] & status->z_MonTrigReponse.w_MonitorStatus[0]);
    /*Appending lower 32 Bits to Upper 32 Bits of Monitor status*/
    retVal= retVal | retVal_l;
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get Synth Freq Monitor Results
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      Synth Frequency Monitor results
 */
MMWave_MonSynthFreqRes_res MMWaveMon_getSynthFreqMonres()
{
    MMWave_MonSynthFreqRes_res val;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    val.status = (res->z_MonLiveSynthFreqResult.c_MonStatus);
    val.freqErrFailCount = (res->z_MonLiveSynthFreqResult.w_FreqErrFailCount);
     #if SOC_XWRL64XX
    /* Multiplying by 32.552 kHz i.e. 3 * (APLL_FREQ) / (144 * 2^13) to get resolution in kHz */
        val.maxSynthFreqErr = (res->z_MonLiveSynthFreqResult.xw_MaxSynthFreqErr)*(MAXSYNTHFREQERRFACT64xx);
    #endif
    #if SOC_XWRL14XX
    /* Multiplying by 43.408 kHz i.e. 4 * (APLL_FREQ) / (144 * 2^13) to get resolution in kHz */
        val.maxSynthFreqErr = (res->z_MonLiveSynthFreqResult.xw_MaxSynthFreqErr)*(MAXSYNTHFREQERRFACT14xx);
    #endif 
    

    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get address of Rx Saturation Live Monitor Results
 * 
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled
 *
 *  @retval
 *      pointer pointing to address of Rx Saturation Live Monitor Base Address
 */
uint32_t *MMWaveMon_getRxSatLiveMonres()
{
    return (uint32_t *)(PERCHIRPLUTRAMARRAYBASEADDR);
}

/**
 *  @b Description
 *  @n
 *      The function is used to get PLL Control Voltage Monitor Results
 *
 *  @param[in] enamask
 *      PLL control voltage monitor enable masks
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      PLL Control Voltages
 */
MMWave_MonPllVolRes_res MMWaveMon_getPllVolMonres(uint8_t enamask)
{
    MMWave_MonPllVolRes_res val = {0.0,0.0,0.0};
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*Getting APLL CTRL Voltage if its enabled*/
    if(enamask & 0x1)
    {
        val.apllV = (res->z_MonPllCtrlVoltResult.h_ApllCtrlVolt) * RFSPLLVOLRES;
       
    }
    /*Getting SYNTH MIN CTRL Voltage & SYNTH MAX CTRL Voltage if its enabled*/
    if(enamask & 0x2)
    {
        val.synthMinV = (res->z_MonPllCtrlVoltResult.h_SynthMinCtrlVolt) * RFSPLLVOLRES;
        val.synthMaxV = (res->z_MonPllCtrlVoltResult.h_SynthMaxCtrlVolt) * RFSPLLVOLRES;
        
    }
    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get Loopback monitor BPM Phase Error
 *
 *  @param[in]  val
 *      Loop Back Phase Error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      Loop Back BPM Phase Error
 */
float MMWaveMon_lbPhaseError(float val)
{
    val = fmod((val+180+720),360);
    val = val - (float)180;
    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get TX RX Loop Back result of various transmitters
 *
 *  @param[in]  txInstance
 *      Instance of Transmitter
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      Gain, Phase and Noise
 */
MMWave_MonTxRxLb_res MMWaveMon_getTxRxLbres(uint8_t txInstance)
{
    MMWave_MonTxRxLb_res val;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
        /*Loop Back Input Power in dBm*/
        val.lbInputPower= (res->z_MonTxNRxLbResult[txInstance].xh_RxInputPower) * 0.1;
        /*Loop Back Power in dBm*/
        val.lbPower[0]=(res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbPower[0]) * 0.1;
        val.lbPower[1]=(res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbPower[1]) * 0.1;
        val.lbPower[2]=(res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbPower[2]) * 0.1;
        /*Loop Back Phase*/
        val.lbPhase[0]=(res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbPhase[0]) * RFSLOOPBACKPHASEFACT;
        val.lbPhase[1]=(res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbPhase[1]) * RFSLOOPBACKPHASEFACT;
        val.lbPhase[2]=(res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbPhase[2]) * RFSLOOPBACKPHASEFACT;
        /*Loop Back Noise*/
        val.lbNoisedbm[0] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbNoise[0]);
        val.lbNoisedbm[1] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbNoise[1]);
        val.lbNoisedbm[2] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbNoise[2]);
        /*Loop Back Power with BPM ON in dBm*/
        val.lbBpmPower[0] = (res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbBpmPower[0]) * 0.1;
        val.lbBpmPower[1] = (res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbBpmPower[1]) * 0.1;
        val.lbBpmPower[2] = (res->z_MonTxNRxLbResult[txInstance].xh_TxNRxLbBpmPower[2]) * 0.1;
        /*Loop Back Phase with BPM ON*/
        val.lbBpmPhase[0] = (res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbBpmPhase[0]) * RFSLOOPBACKPHASEFACT;
        val.lbBpmPhase[1] = (res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbBpmPhase[1]) * RFSLOOPBACKPHASEFACT;
        val.lbBpmPhase[2] = (res->z_MonTxNRxLbResult[txInstance].h_TxNRxLbBpmPhase[2]) * RFSLOOPBACKPHASEFACT;
        /*Loop Back Noise with BPM ON*/
        val.lbBpmNoisedbm[0] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbBpmNoise[0]);
        val.lbBpmNoisedbm[1] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbBpmNoise[1]);
        val.lbBpmNoisedbm[2] = (res->z_MonTxNRxLbResult[txInstance].xc_TxNRxLbBpmNoise[2]); 
        /*Loop Back BPM Gain error*/
        val.lbBPMGainError[0] = (val.lbBpmPower[0]- val.lbPower[0]);
        val.lbBPMGainError[1] = (val.lbBpmPower[1]- val.lbPower[1]);
        val.lbBPMGainError[2] = (val.lbBpmPower[2]- val.lbPower[2]);
        /*Loop Back BPM Phase error*/
        val.lbBPMPhaseError[0] = MMWaveMon_lbPhaseError(val.lbBpmPhase[0]- val.lbPhase[0] -180);
        val.lbBPMPhaseError[1] = MMWaveMon_lbPhaseError(val.lbBpmPhase[1]- val.lbPhase[1] -180);
        val.lbBPMPhaseError[2] = MMWaveMon_lbPhaseError(val.lbBpmPhase[2]- val.lbPhase[2] -180);
        /*Loop Back Rx Gain Mismatch*/
        val.lbRxGainMismatch[0]=  val.lbPower[1] - val.lbPower[0];
        val.lbRxGainMismatch[1]=  val.lbPower[2] - val.lbPower[0];
        /*Loop Back Rx Phase Mismatch*/
        val.lbRxPhaseMismatch[0] = MMWaveMon_lbPhaseError(val.lbPhase[1]-val.lbPhase[0]);
        val.lbRxPhaseMismatch[1] = MMWaveMon_lbPhaseError(val.lbPhase[2]-val.lbPhase[0]);
    

    return val;
}

 /**
 *  @b Description
 *  @n
 *      The function is used to get TX Power of various transmitters
 *
 *  @param[in]  txInstance
 *      Instance of Transmitter
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      Power value in dBm
 */
float MMWaveMon_getTXnPow(uint8_t txInstance)
{
    float val;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*TX power in dBm*/
    val = (res->z_MonTxNPwrResult[txInstance].xh_TxNPwrVal) * 0.1;
    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get TX Power ball break of various transmitters
 *
 *  @param[in]  txInstance
 *      Instance of Transmitter
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      Incident Power and reflected power value in dBm
 */
MMWave_MonTxnPowBB_res MMWaveMon_getTXnPowBB(uint8_t txInstance)
{
    MMWave_MonTxnPowBB_res val;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*TX power at incident PD*/
    val.txIncPow = (res->z_MonTxNBbResult[txInstance].xh_TxNIncidPwrVal) * 0.1;
    /*TX power at reflected PD*/
    val.txRefPow = (res->z_MonTxNBbResult[txInstance].xh_TxNReflPwrVal) * 0.1;
    /*TX return Loss*/
    val.txReturnLoss=(val.txRefPow-val.txIncPow);
    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get TX Dc Signal result of various transmitters
 *
 *  @param[in]  txInstance
 *      Instance of Transmitter
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *   Result Status (Pass/Fail) for TX supply GPADC DC signals.
 *     | Bit Field   | Definition  |
 *     |-------------|-----------  |
 *     | Bit[0]      | TXn_VDDA_SUPPLY signal Status      |
 *     | Bit[1]      | TXn_1V_CLUSTER_SUPPLY signal Status     |
 *     | Bit[15:2]  | Reserved     |
 */
uint16_t MMWaveMon_getTXnDcSig(uint8_t txInstance)
{
    uint16_t val;
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*status of DC Signal Monitor*/
    val = res->z_MonTxNDcSigResult[txInstance].h_TxDcMonStatus;
    return val;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get RX HPF DC Signal result
 *
 *  @param[in]  val
 *      Place holder to put result data
 *  @param[in]  enamask
 *      Which all monitors are enabled.
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *  None
 */
void MMWaveMon_getRxHpfDcSig(MMWave_MonRxHpfDcSig_res* val)
{
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
        /*Status of RX HPF DC Monitor*/
        val->rxDcMonstat = res->z_MonRxHpfDcSigResult.w_RxDcMonStatus;
        /*HPF Inband Power*/
        val->RxHpfInbandPwrdB[0] = (res->z_MonRxHpfDcSigResult.xh_RxHpfInBandPwr[0]) * 0.1;
        val->RxHpfInbandPwrdB[1] = (res->z_MonRxHpfDcSigResult.xh_RxHpfInBandPwr[1]) * 0.1;
        val->RxHpfInbandPwrdB[2] = (res->z_MonRxHpfDcSigResult.xh_RxHpfInBandPwr[2]) * 0.1;
        /*HPF Cutoff Power*/
        val->RxHpfCutoffPwrdB[0] = (res->z_MonRxHpfDcSigResult.xh_RxHpfCutOffPwr[0]) * 0.1;
        val->RxHpfCutoffPwrdB[1] = (res->z_MonRxHpfDcSigResult.xh_RxHpfCutOffPwr[1]) * 0.1;
        val->RxHpfCutoffPwrdB[2] = (res->z_MonRxHpfDcSigResult.xh_RxHpfCutOffPwr[2]) * 0.1;
        /*HPF Cutoff Attenuation*/
        val->RxHpfCutoffAtten[0] = (val->RxHpfInbandPwrdB[0]-val->RxHpfCutoffPwrdB[0]);
        val->RxHpfCutoffAtten[1] = (val->RxHpfInbandPwrdB[1]-val->RxHpfCutoffPwrdB[1]);
        val->RxHpfCutoffAtten[2] = (val->RxHpfInbandPwrdB[2]-val->RxHpfCutoffPwrdB[2]); 

}

/**
 *  @b Description
 *  @n
 *      The function is used to get PM CLK DC Signal Monitor Results
 *
 *  @param[in]  val
 *      Place holder to put result data
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *   Respective monitor is enabled and status of that monitor is successful
 *
 *  @retval
 *      None
 */
void MMWaveMon_getPmClkDcMonres(MMWave_MonPmClkDcSig_res* val)
{
    T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT* res = M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS;
    /*PM CLK DC Monitor Status*/
    val->pmClkDcMonstat = res->z_MonPmClkDcSigResult.w_PmClkDcMonStatus;
    /*The REF1_0P45V Reference voltage GPADC measurement value*/
    val->volref1V = (res->z_MonPmClkDcSigResult.c_VRef1Val) * RFSPMCLKDCVOLFACT;
    /*The REF2_0P6V Reference voltage GPADC measurement value*/
    val->volref2V = (res->z_MonPmClkDcSigResult.c_VRef2Val) * RFSPMCLKDCVOLFACT;
    /*The The REF3_0P9V Reference voltage GPADC measurement value*/
    val->volref3V = (res->z_MonPmClkDcSigResult.c_VRef3Val) * RFSPMCLKDCVOLFACT;
    /*The REF4_VION_IN Reference voltage GPADC measurement value*/
    val->volref4V = (res->z_MonPmClkDcSigResult.c_VRef4Val) * RFSPMCLKDCVOLFACT;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure Synth frequency monitor
 *
 *  @param[in] cfg
 *      Synth frequency Configuration
 * 
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_TxnSynthFreqCfg ()
{
   uint32_t retVal = -1;
   float freqErrThresh;
   T_RL_API_MON_LIVE_SYNTH_FREQ_CFG rlcfg;
   /*Multiplied by 5 to get resolution of 0.2us*/
   rlcfg.xc_ChirpMonStartTime = (int8_t)(6*RFCHIRPMONSTARTTIMFACT);
   #if SOC_XWRL64XX
    /* Dividing 80 MHz by 3 * (APLL_FREQ) / (144 * 2^8) to get resolution of 32.552 kHz */
        freqErrThresh= (FREQERRTHRES80MHZ)/RFFREQERRTHRESFACT64xx;
        rlcfg.h_FreqErrThreshold= (uint16_t) ceil(freqErrThresh);
    #endif
    #if SOC_XWRL14XX
    /* Dividing 80 MHz by 4 * (APLL_FREQ) / (144 * 2^8) to get resolution of 43.408 kHz */
        freqErrThresh= (FREQERRTHRES80MHZ)/RFFREQERRTHRESFACT14xx;
        rlcfg.h_FreqErrThreshold= (uint16_t) ceil(freqErrThresh);
    #endif 
   retVal = rl_monLiveSynthFreqCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);
   return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure Rx Saturation Live monitor
 *
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_RxSatLiveCfg ()
{
   uint32_t retVal = -1;
   T_RL_API_MON_LIVE_RX_SATURATION_CFG rlcfg;

   rlcfg.h_PerChirpRamStartAdd = 0;

   retVal = rl_monLiveRxSaturationCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);
   return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure PLL Control Voltage monitor
 *
 *  @param[in] enamask
 *      PLL control voltage monitor enable masks
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_PllCtrlVolCfg(uint8_t enamask)
{
    uint32_t retVal = -1;
    T_RL_API_MON_PLL_CTRL_VOLT_CFG rlcfg;
    rlcfg.c_PllMonEnaMask = enamask;
    retVal = rl_monPllCtrlVoltCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure TX RX Loop Back monitor
 *
 *  @param[in] txInstance
 *      TX instance
 *  @param[in] cfg
 *      Tx Rx Loop Back Monitor Configuration
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_TxRxLbCfg (uint8_t txInstance, MMWave_MonTxRxLb_cfg* cfg)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0, slope = 0.0;
    T_RL_API_MON_TXN_RX_LB_CFG rlcfg;

    rlcfg.c_TxIndSel = txInstance;
    rlcfg.c_MonEnaCtrl = cfg->monenbl;
    rlcfg.c_MonRxTxCodesSel = cfg->txrxCodeSel;
    rlcfg.c_MonRxGainCode = cfg->rxGainCode;
    rlcfg.h_MonTxBiasCodes = cfg->txBiasCode;
    #if SOC_XWRL64XX
    if(cfg->rfFreqGhz < 57 || cfg->rfFreqGhz > 64 || cfg->rffreqSlopeMhz < -399 || cfg->rffreqSlopeMhz > 399)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT64xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT64xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    #if SOC_XWRL14XX
    if(cfg->rfFreqGhz < 76 || cfg->rfFreqGhz > 81 || cfg->rffreqSlopeMhz < -533 || cfg->rffreqSlopeMhz > 533)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT14xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT14xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    retVal = rl_monTxNRxLbCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure TX Power monitor
 *
 *  @param[in] txInstance
 *      TX instance
 *  @param[in] cfg
 *      Tx Power Monitor Configuration
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_TxnPowCfg (uint8_t txInstance, MMWave_MonTxnPow_cfg* cfg)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0, slope = 0.0;
    T_RL_API_MON_TXN_POWER_CFG rlcfg;

    rlcfg.c_TxIndSel = txInstance;
    rlcfg.c_MonTxCodesSel = cfg->txBiasSel;
    rlcfg.h_MonTxBiasCodes = cfg->txBiasCode;
    #if SOC_XWRL64XX
    if(cfg->rfFreqGhz < 57 || cfg->rfFreqGhz > 64 || cfg->rffreqSlopeMhz < -399 || cfg->rffreqSlopeMhz > 399 || cfg->txBackoff > 26)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT64xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT64xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    #if SOC_XWRL14XX
    if(cfg->rfFreqGhz < 76 || cfg->rfFreqGhz > 81 || cfg->rffreqSlopeMhz < -533 || cfg->rffreqSlopeMhz > 533 || cfg->txBackoff > 26)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT14xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT14xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    rlcfg.c_TxBackoffMap = cfg->txBackoff * 2;
    retVal = rl_monTxNPowerCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure TX Power Ball Break monitor
 *
 *  @param[in] txInstance
 *      TX instance
 *  @param[in] cfg
 *      Tx Power Monitor Configuration
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_TxnPowBBCfg (uint8_t txInstance, MMWave_MonTxnPow_cfg* cfg)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0, slope = 0.0;
    T_RL_API_MON_TXN_BB_CFG rlcfg;

    rlcfg.c_TxIndSel = txInstance;
    rlcfg.c_MonTxCodesSel = cfg->txBiasSel;
    rlcfg.h_MonTxBiasCodes = cfg->txBiasCode;
    #if SOC_XWRL64XX
    if(cfg->rfFreqGhz < 57 || cfg->rfFreqGhz > 64 || cfg->rffreqSlopeMhz < -399 || cfg->rffreqSlopeMhz > 399 || cfg->txBackoff > 26)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT64xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT64xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    #if SOC_XWRL14XX
    if(cfg->rfFreqGhz < 76 || cfg->rfFreqGhz > 81 || cfg->rffreqSlopeMhz < -533 || cfg->rffreqSlopeMhz > 533 || cfg->txBackoff > 26)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT14xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT14xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    rlcfg.c_TxBackoffMap = cfg->txBackoff * 2;
    retVal = rl_monTxNBbCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure TX DC Signal monitor
 *
 *  @param[in] txInstance
 *      TX instance
 *  @param[in] cfg
 *      DC Signal Monitor Configuration
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_TxnDcSigCfg (uint8_t txInstance, MMWave_MonTxnDcSig_cfg* cfg)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0, slope = 0.0;
    T_RL_API_MON_TXN_DCSIG_CFG rlcfg;

    rlcfg.c_TxIndSel = txInstance;
    rlcfg.c_MonTxCodesSel = cfg->txBiasSel;
    rlcfg.h_MonTxBiasCodes = cfg->txBiasCode;
    #if SOC_XWRL64XX
    if(cfg->rfFreqGhz < 57 || cfg->rfFreqGhz > 64 || cfg->rffreqSlopeMhz < -399 || cfg->rffreqSlopeMhz > 399)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT64xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT64xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    #if SOC_XWRL14XX
    if(cfg->rfFreqGhz < 76 || cfg->rfFreqGhz > 81 || cfg->rffreqSlopeMhz < -533 || cfg->rffreqSlopeMhz > 533)
    {
        return retVal;
    }
        strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT14xx;
        rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
        slope = (cfg->rffreqSlopeMhz * 1000)/RFFREQSLOPEFACT14xx;
        rlcfg.xh_RfFreqSlope = (int16_t) ceil(slope);
    #endif
    retVal = rl_monTxNDcSigCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure RX HPF DC Signal monitor
 *
 *  @param[in] cfg
 *      Rx HPF DC Signal Monitor Configuration
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_RxHpfDcSigCfg (MMWave_MonRxHpfDcSig_cfg* cfg)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0;
    T_RL_API_MON_RX_HPF_DCSIG_CFG rlcfg;

    rlcfg.c_MonEnaCtrl = cfg->monenbl;
    rlcfg.c_RxHpfSel = cfg->rxHpfSel;
    #if SOC_XWRL64XX
    if(cfg->rfFreqGhz < 57 || cfg->rfFreqGhz > 64)
    {
        return retVal;
    }
    strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT64xx;
    rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
    #endif
    #if SOC_XWRL14XX
    if(cfg->rfFreqGhz < 76 || cfg->rfFreqGhz > 81)
    {
        return retVal;
    }
    strtFreq = (cfg->rfFreqGhz * 1000)/RFSTRTFREQFACT14xx;
    rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
    #endif
    retVal = rl_monRxHpfDcSigCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure PM Clock DC signal monitor
 *
 *  @param[in] strtFreqGhz
 *      Monitor RF start frequency
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      NA
 *
 *  @retval
 *      Success - 0, Failure - Error code < 0
 */
int32_t MMWaveMon_PmClkDcSigCfg(float strtFreqGhz)
{
    uint32_t retVal = -1;
    float strtFreq = 0.0;
    T_RL_API_MON_PMCLK_DCSIG_CFG rlcfg;
    #if SOC_XWRL64XX
    if(strtFreqGhz < 57 || strtFreqGhz > 64)
    {
        return retVal;
    }
    strtFreq = (strtFreqGhz * 1000)/RFSTRTFREQFACT64xx;
    rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
    #endif
    #if SOC_XWRL14XX
    if(strtFreqGhz < 76 || strtFreqGhz > 81)
    {
        return retVal;
    }
    strtFreq = (strtFreqGhz * 1000)/RFSTRTFREQFACT14xx;
    rlcfg.h_RfFreqStart = (uint16_t) ceil(strtFreq);
    #endif
    retVal = rl_monPmClkDcSigCfg(M_DFP_DEVICE_INDEX_0,&rlcfg);
    return retVal;
}