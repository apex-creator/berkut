/**
 *   @file  mmwave_link_common.c
 *
 *   @brief
 *      The file implements the common mmWave control module which interfaces
 *      with the mmWave Link API
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016-2021 Texas Instruments, Inc.
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
/* Uncomment below to enable debugp traces*/
#define DebugP_log_ENABLED 1

#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Includes from MCU Plus SDK */
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HeapP.h>
#include <drivers/hw_include/hw_types.h>
#include <common/syscommon.h>

#include <drivers/hw_include/xwrL64xx/cslr_soc.h>
#include <drivers/hw_include/cslr_adcbuf.h>
#include <drivers/soc.h>

#include <control/mmwave/mmwave.h>
#include <control/mmwave/include/mmwave_internal.h>

/**************************************************************************
 ***************************** Local Functions ****************************
 **************************************************************************/

static int32_t MMWave_configureProfileChirp(MMWave_CtrlCfg * ptrControlCfg, int32_t* errCode);
static int32_t MMWave_deinitMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
static int32_t MMWave_configureFrame(MMWave_MCB * ptrMMWaveMCB, MMWave_CtrlCfg * ptrControlCfg, int32_t* errCode);


/**************************************************************************
 ************************ mmWave Link Functions ***************************
 **************************************************************************/

int32_t MMWave_LinkInitCommon(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal = MINUS_ONE;

    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.h_XtalClkFreq = M_RL_FECSS_XTAL_CLK_FREQ_40M;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_ClkSourceSel = M_RL_FECSS_DEV_CLK_SRC_FCLK;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_PowerMode = M_RL_FECSS_PWR_ON_MODE_COLD;

    // For RPMF
    if(ptrMMWaveMCB->initCfg.iswarmstart)
    {
      ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_PowerMode = M_RL_FECSS_PWR_ON_MODE_WARM;
    }

    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_FecBootCfg = 0U;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_ChirpTimerResol = M_RL_FECSS_CHIRP_TIMER_RES_00; /* Low res */
    /* Power on the Device: */
    retVal = (int32_t)rl_fecssDevPwrOn(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevPwrOnCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to initialize and power on the BSS. Encode the error code to account
         * for the subsystem error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Get the version information: */
    retVal = MMWave_deviceGetVersion(ptrMMWaveMCB, errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }

    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_DevClkCtrl = M_RL_FECSS_DEV_CLK_SRC_FCLK;
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_FtClkCtrl = M_RL_FECSS_FT_CLK_SRC_XTAL;
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_ON_CAL;
    // For RPMF
    if(ptrMMWaveMCB->initCfg.iswarmstart)
    {
       ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_ON;
    }
    /* FECSS Device Clock Control */
    retVal = rl_fecssDevClockCtrl(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }

#if 0
    //TODO: RFStatusGet function pointer isn't available yet on RFS
    retVal = rl_fecssRfStatusGet(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecRFStsGetRsp);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }
#endif

#if 0
    /* Do we need to restore calibration data? */
    if(ptrMMWaveMCB->initCfg.ptrFactoryCalibData != NULL)
    {
        retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->initCfg.ptrFactoryCalibData));
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }
    }

    if (TRUE == ptrMMWaveMCB->initCfg.isFactoryCalEnabled)
    {
        retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->initCfg.ptrAteCalibration));
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }

        retVal = rl_fecssRfFactoryCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd, &ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp);
        if((retVal != M_DFP_RET_CODE_OK) || 
           (ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp.h_CalRunStatus != ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask) || 
           (ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp.h_CalResStatus != 
           ( ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask | RFS_FACTORY_CAL_VALIDITY_STATUS)))
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFSBOOTCAL, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }
#endif

exit:
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function which is invoked
 *      to ensure that the spawn function is invoked in a different execution
 *      context
 *
 *  @param[in]  fxn
 *      Pointer to the function to be executed in a different context
 *  @param[in]  pValue
 *      Pointer of data to be passed to the function
 *  @param[in]  flags
 *      Flag to indicate preference
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t MMWave_spawn
(
    T_DFP_OSI_SPAWN_HANDLER    fxn,
    const void*             pValue,
    uint32_t                flags
)
{
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uintptr_t               key;
    int32_t                 retVal = 0;

    DebugP_logInfo("MMWave_spawn. Adding 0x%x\n",(uintptr_t)fxn);

    /* Critical Section: The spawn free list is a critical resource which is accessed
     * from multiple contexts */
    key = HwiP_disable();
    ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnFreeList);
    HwiP_restore (key);

    /* Did we get an entry? */
    if (ptrSpawnFxnNode != NULL)
    {
        /* YES: Populate the entry */
        ptrSpawnFxnNode->spawnEntry = fxn;
        ptrSpawnFxnNode->arg        = pValue;

        DebugP_logInfo("MMWave_spawn add list\n");

        /* Critical Section: The spawn active list is a critical resource which is accessed
         * from multiple contexts */
        key = HwiP_disable();
        MMWave_listAdd ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnActiveList, (MMWave_ListNode*)ptrSpawnFxnNode);
        HwiP_restore (key);

        /* Keep track of the number of spawn messages which have been received */
        gMMWave_MCB.spawnCounter++;
        /* Wake up the mmWave execution thread  */
        SemaphoreP_post(&gMMWave_MCB.linkSemHandle);
    }
    else
    {
        /* Error: No span free node was present. This can happen if all the spawn functions
         * have been taken up and the execute mmWave control API has not been invoked. Increment
         * statistics to report this condition */
        gMMWave_MCB.spawnOverflow++;
        DebugP_logError("SPAWN error!!!\n");
        /* Setup the return value to indicate an error. */
        retVal = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the specified profile
 *      and corresponding chirp configuration. The following order is preserved in
 *      the function:
 *          - Profile configuration
 *          - Chirp configuration
 *
 *  @param[in]  ptrControlCfg
 *      Pointer to the control config
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_configureProfileChirp(MMWave_CtrlCfg * ptrControlCfg, int32_t* errCode)
{

    MMWave_ProfileHandle  * ptrProfileHandle;
    int32_t                 retVal;
    int32_t                 index;
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfg;
    T_RL_API_SENS_DYN_PWR_SAVE_DIS             profileDynPwrSaveDis;
    MMWave_ChirpHandle      chirpHandle;
    T_RL_API_SENS_PER_CHIRP_CFG            chirpCfg;
    T_RL_API_SENS_PER_CHIRP_CTRL           chirpCtrl;
    uint32_t                numChirps;
    uint32_t                chirpIndex;
    uint32_t                u32DevIdx;
    float                   minPowSaveIdleTime, minIdleTime, rampEndTimeus, minrampEndTimeus, sampFreqMhz;
    int32_t                skipSamp, adcSamp,excSamp;

    float startFreqGHz, slopeMHzperUS, bandwidthGHz, maxFreqGhz;
    uint8_t isDevAOP = SOC_isDeviceAOP();


    memset((void *)&profileDynPwrSaveDis, 0, sizeof(T_RL_API_SENS_DYN_PWR_SAVE_DIS));

    /* Loop across all devices to send profile and chirp configurations */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {

        /* Get the first profile handler of the selected device */
        ptrProfileHandle = &ptrControlCfg->frameCfg[u32DevIdx].profileHandle[0];

        /* Cycle through all the profile(s) which have been specified. */
        for (index = 0; index < MMWAVE_MAX_PROFILE; index++)
        {
            /* Do we have a valid profile? */
            if (ptrProfileHandle[index] == NULL)
            {
                /* NO: Skip to the next profile */
                continue;
            }

            /* YES: Get the profile configuration */
            if (MMWave_getProfileCfg(ptrProfileHandle[index], &profileCfg, &profileTimeCfg, errCode) < 0)
            {
                /* Error: Unable to get the profile configuration. Setup the return value */
                retVal = MINUS_ONE;
                goto end;
            }
            /* Check if RAMP end time configured is correct */
            rampEndTimeus = profileCfg.h_ChirpRampEndTime / 10.0; // Considering Low resolution mode
            skipSamp      = (profileTimeCfg.h_ChirpAdcStartTime) >> 10;
            adcSamp       = profileCfg.h_NumOfAdcSamples;
            sampFreqMhz   = (100.0 / profileCfg.c_DigOutputSampRate);

            if(isDevAOP == 1)
            {
                startFreqGHz                       =  (float)((profileTimeCfg.w_ChirpRfFreqStart * 300.0) / (1000.0 * 256.0));
                slopeMHzperUS                      =  (float)((profileTimeCfg.xh_ChirpRfFreqSlope * 3.0 * 100.0 * 100.0) / (1048576.0));
                bandwidthGHz                       =   rampEndTimeus * slopeMHzperUS * 1.e-3;

                if(slopeMHzperUS < 0)
                {
                    maxFreqGhz = startFreqGHz;
                }
                else
                {
                    maxFreqGhz = startFreqGHz + bandwidthGHz;
                }

                if(maxFreqGhz > MAX_AOP_FREQ_GHZ)
                {
                    *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                    retVal   = MINUS_ONE;
                    goto end;
                }
            }

            /*  Setting excSamp if sampling frequency is greater than or equal to 1.5 Msps to:
                    Short filter : -5
                    Long filter  : -8
            */
            if (sampFreqMhz>=1.5)
            {
                if (profileCfg.c_DfeFirSel == 1)
                {
                    excSamp = -5;
                }
                else
                {
                    excSamp = -8;
                }
            }
            /*  Setting excSamp if sampling frequency is less than 1.5 Msps to:
                    Short filter : -4
                    Long filter  : -4
            */
            else
            {
                excSamp=-4;
            }

            /* Check for rampend time and idle time*/
            if ((rampEndTimeus + (profileTimeCfg.h_ChirpIdleTime * 0.1)) < ((skipSamp + adcSamp +4)/sampFreqMhz))
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }

            minrampEndTimeus = (skipSamp + adcSamp + excSamp)/sampFreqMhz;

            if(rampEndTimeus < minrampEndTimeus)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            
            #if defined (SOC_XWRL64XX)
            /* Check if CRD NSLOPE Magnitude is less than 0xC00 or not */
            if(profileCfg.h_CrdNSlopeMag > 0x0C00)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            #else
            /* Check if CRD NSLOPE Magnitude is less than 0x925 or not */
            if(profileCfg.h_CrdNSlopeMag > 0x925)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            #endif

            /* Configure the profile using the mmWave Link API */
            retVal = rl_sensChirpProfComnCfg(u32DevIdx, &profileCfg);

            if (retVal != M_DFP_RET_CODE_OK)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }

            minPowSaveIdleTime = MAX(6.0 - (profileTimeCfg.xh_ChirpTxStartTime * 0.02), 3.1);
            minIdleTime = MAX(4.1 - (profileTimeCfg.xh_ChirpTxStartTime * 0.02), 3.1);
	        /* Check if Chirp Idle time is supported */
            if((profileTimeCfg.h_ChirpIdleTime * 0.1) < minIdleTime)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }
            else
            {
                /* Check if Inter-Chirp Power Saving should be disabled. By default it is enabled */
                if(((profileTimeCfg.h_ChirpIdleTime * 0.1) < minPowSaveIdleTime))
                {   
                    /*enable Tx and LO power save always*/
                    /*
                    TX PA POWERSAVE ENABLED
                    TX LO POWERSAVE ENABLED
                    RX RF POWERSAVE DISABLED
                    RX BB POWERSAVE DISABLED
                    RX LO POWERSAVE DISABLED
                    */
                    profileDynPwrSaveDis.c_InterChirpPsDis = 0x1C;
                    retVal = rl_sensDynPwrSaveDis(u32DevIdx, &profileDynPwrSaveDis);
                    if (retVal != M_DFP_RET_CODE_OK)
                    {
                        *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                        retVal   = MINUS_ONE;
                        goto end;
                    }
                
                }
            }

            /* Configure the profile using the mmWave Link API */
            retVal = rl_sensChirpProfTimeCfg(u32DevIdx, &profileTimeCfg);

            if (retVal != M_DFP_RET_CODE_OK)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }

            /* Get the number of chirps configured and attached to the profile: */
            if (MMWave_getNumChirps(ptrProfileHandle[index], &numChirps, errCode) < 0)
            {
                /* Error: Unable to get the number of chirps. Error code is already setup */
                retVal = MINUS_ONE;
                goto end;
            }

            /* For the profile; Cycle through all the chirps and configure them. */
            for (chirpIndex = 1U; chirpIndex <= numChirps; chirpIndex++)
            {
                /* Get the Chirp Handle associated at the specified index */
                if (MMWave_getChirpHandle(ptrProfileHandle[index], chirpIndex, &chirpHandle, errCode) < 0)
                {
                    /* Error: Unable to get the chirp handle. Error code is already setup */
                    retVal = MINUS_ONE;
                    goto end;
                }

                /* Get the chirp configuration: */
                if (MMWave_getChirpCfg(chirpHandle, &chirpCfg, &chirpCtrl, errCode) < 0)
                {
                    /* Error: Unable to get the chirp configuration. Error code is already setup */
                    retVal = MINUS_ONE;
                    goto end;
                }

                /* TODO: Cleanup this is adv chirp config and not needed for OOB */
                /* Set the chirp configuration in the mmWave link */
                //retVal = rl_sensPerChirpCfg(u32DevIdx, &chirpCfg);
                //if (retVal != M_DFP_RET_CODE_OK)
                //{
                //    *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECHIRPCFG, retVal);
                //    retVal   = MINUS_ONE;
                //    goto end;
                //}

                /* Set the chirp control in the mmWave link */
                //retVal = rl_sensPerChirpCtrl(u32DevIdx, &chirpCtrl);
                //if (retVal != M_DFP_RET_CODE_OK)
                //{
                //    *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_ECHIRPCFG, retVal);
                //    retVal   = MINUS_ONE;
                //    goto end;
                //}
            }
        }
    }

    /* Control comes here implies that the profile & chirp was configured successfully */
    retVal = 0;

end:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the specified frame
 *      configuration.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control config
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

static int32_t MMWave_configureFrame(MMWave_MCB * ptrMMWaveMCB, MMWave_CtrlCfg * ptrControlCfg, int32_t * errCode)
{
    MMWave_ProfileHandle  * ptrProfileHandle;
    
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          stProfileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          stProfileTimeCfg;
    T_RL_API_SENS_FRAME_CFG          * pstFrameCfg;
    float rampEndTimeus;
    uint8_t numChirpsAccum;
    uint16_t chirpIdleTime, chirpsInBurst;
    float burstPeriodicity;
    int32_t  retVal;
    uint32_t u32ProfIdx;
    uint32_t u32DevIdx;

    /* Get the number of ADC samples (it is expected that all profiles across
     *   all devices have the same number) */
    ptrProfileHandle = &ptrControlCfg->frameCfg[0].profileHandle[0];

    for(u32ProfIdx = 0U; u32ProfIdx < MMWAVE_MAX_PROFILE; u32ProfIdx++)
    {
        /* Check if this index holds a profile handle */
        if(ptrProfileHandle[u32ProfIdx] != NULL)
        {
            /* Get the profile configuration */
            MMWave_getProfileCfg(ptrProfileHandle[u32ProfIdx], &stProfileCfg, &stProfileTimeCfg, errCode);

            break;
        }
    }

    /* Loop across all devices to send profile and chirp configurations */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {
        /* Get the frame configuration of the device */
        pstFrameCfg = &ptrControlCfg->frameCfg[u32DevIdx].frameCfg;
        rampEndTimeus = stProfileCfg.h_ChirpRampEndTime / 10.0; // Considering Low resolution mode
        numChirpsAccum = pstFrameCfg->c_NumOfChirpsAccum;
        /* Dividing by 10 as pstFrameCfg->w_BurstPeriodicity is already multiplied by 10 */
        burstPeriodicity = ((float)(pstFrameCfg->w_BurstPeriodicity))/10.0;
        /* Dividing by 10 as stProfileTimeCfg.h_ChirpIdleTime is already multiplied by 10 */
        chirpIdleTime = stProfileTimeCfg.h_ChirpIdleTime/10;
        chirpsInBurst =pstFrameCfg->h_NumOfChirpsInBurst;

         /*Ensuring minimum inter burst idle time*/
        if(numChirpsAccum ==0) //No accumulation
        {
            if ((burstPeriodicity) < ((rampEndTimeus + (chirpIdleTime))*chirpsInBurst +115)) 
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
                retVal   = MINUS_ONE;
                break;
            }
        }
        else
        {
            if ((burstPeriodicity) < ((rampEndTimeus + (chirpIdleTime))*(chirpsInBurst*numChirpsAccum)+115)) 
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
                retVal   = MINUS_ONE;
                break;
            }
        }
        retVal = rl_sensFrameCfg(u32DevIdx, pstFrameCfg);
        if (retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
            retVal   = MINUS_ONE;
            break;
        }
        
    }

   return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the supplied
 *      configuration
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_configLink
(
    MMWave_MCB*         ptrMMWaveMCB,
    MMWave_CtrlCfg*     ptrControlCfg,
    int32_t*            errCode
)
{
    int32_t retVal;

    /**************************************************************************
     * Frame Mode:
     * Order of operations as specified by the mmWave Link are
     *  - Profile configuration
     *  - Chirp configuration
     *  - Frame configuration
     **************************************************************************/
    retVal = MMWave_configureProfileChirp(ptrControlCfg, errCode);

    if(retVal < 0)
    {
        goto end;
    }

    /* Set the frame configuration */
    retVal = MMWave_configureFrame(ptrMMWaveMCB, ptrControlCfg, errCode);
    if(retVal < 0)
    {
        goto end;
    }

    /* Set the frame configuration: */
    DebugP_logInfo ("rlSetFrameConfig...\n");
    retVal = rl_sensFrameCfg(M_DFP_DEVICE_INDEX_0, &ptrControlCfg->frameCfg[0].frameCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rlSetFrameConfig error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

    /* Set the Temperature measurement configuration: */
    DebugP_logInfo ("rlTempMeasConfig...\n");
    retVal = rl_fecssTempMeasCfg(M_DFP_DEVICE_INDEX_0, &ptrControlCfg->frameCfg[0].tempCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the temp measurement configuration failed */
        DebugP_logError("rlTempMeasConfig error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }
//#ifdef DFP_LOOPBACK_ENABLED
#if 0
    T_RL_API_SENS_LOOPBACK_CFG sensLoopbackCfg;
    sensLoopbackCfg.c_LbFreqSel = 20U; /* 5MHz */
    sensLoopbackCfg.c_IfaLbGainIndex = 0xB2U; /* 0dB */
    sensLoopbackCfg.c_LoPaLbCmnGainIndex = 48U; /* 0dB */
    sensLoopbackCfg.c_LoLbGainIndex = 96U; /* 0dB */
    sensLoopbackCfg.c_PaLbGainIndex = 48U; /* 0dB */
    sensLoopbackCfg.h_ExtLbTxBpmEnSel = 3U; /* External LB settings */
    retVal = rl_sensLoopBackCfg(M_DFP_DEVICE_INDEX_0, &sensLoopbackCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rl_loopbackCfg error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

    T_RL_API_SENS_LOOPBACK_ENA   sensLoopbackEna;
    /* RFS RFS sens LB ena api - DIG  LB */
    sensLoopbackEna.c_LbEnable = 0x1U;
    retVal = rl_sensLoopBackEna(M_DFP_DEVICE_INDEX_0, &sensLoopbackEna);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rl_loopbackEna error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

#endif

    /* Set the return value to be success. */
    retVal = 0;

end:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave link. This function can only be
 *      invoked once the configuration has been completed successfully.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_startLink (MMWave_MCB* ptrMMWaveMCB, const MMWave_StrtCfg* ptrStrtcfg, int32_t* errCode)
{

    int32_t  retVal;
    T_RL_API_SENSOR_START_CMD sensStartCmd;

    /* Sensor frame trigger cfg api */
    sensStartCmd.c_FrameTrigMode = ptrStrtcfg->frameTrigMode; // M_RL_SENS_FRAME_SW_TRIG; /* SW trigger */
    sensStartCmd.c_ChirpStartSigLbEn = ptrStrtcfg->chirpStartSigLbEn; //M_RL_SENS_CT_START_SIG_LB_DIS;
    sensStartCmd.c_FrameLivMonEn = ptrStrtcfg->frameLivMonEn; /* SYNTH, RX_SAT enabled - DKS GPADC pending */
    sensStartCmd.w_FrameTrigTimerVal = ptrStrtcfg->frameTrigTimerVal; // 0U; /* Read timer and write DKS */
    retVal = rl_sensSensorStart(M_DFP_DEVICE_INDEX_0, &sensStartCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Starting the sensor failed */
        * errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ESENSOR, retVal);

        retVal = MINUS_ONE;
        goto exit;
    }

    /* Control comes here indicates that the sensor has been started successfully */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_stopLink (const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal;
    T_RL_API_SENSOR_STOP_CMD    sensStopCmd;
    /* Are we operating in Chirp or Continuous mode? */
    /******************************************************************************
     * CHIRP or ADVANCED: Stop the sensor
     ******************************************************************************/
    /* Stop Master device */
    sensStopCmd.c_FrameStopMode = M_RL_SENS_STOP_FRAME;
    retVal = rl_sensSensorStop(M_DFP_DEVICE_INDEX_0, &sensStopCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* All other are treated as FATAL error */
        * errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ESENSOR, retVal);
        retVal    = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to execute the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_executeLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uintptr_t               key;

    /* Semaphore has been posted; process any active jobs in the spawn active list.  */
    while (1)
    {
        /* Critical Section: The spawn active list is a critical resource which is accessed
         * from multiple contexts */

         DebugP_logInfo("MMWave_executeLink: will get  node from active list\n");

        key = HwiP_disable();
        ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnActiveList);
        HwiP_restore (key);

        /* Is there an active entry to be processed */
        if (ptrSpawnFxnNode == NULL)
        {
            /* No other spawn nodes to be processed; we are done break out of the loop*/
            DebugP_logInfo("MMWave_executeLink:No other spawn nodes to be processed\n");
            break;
        }
        else
        {
            /* Execute the spawn function */
            DebugP_logInfo("MMWave_executeLink: Execute the spawn function 0x%x\n",(uintptr_t)(ptrSpawnFxnNode->spawnEntry));
            ptrSpawnFxnNode->spawnEntry (0, ptrSpawnFxnNode->arg);
            DebugP_logInfo("MMWave_executeLink: Finished executing the spawn function 0x%x\n",(uintptr_t)(ptrSpawnFxnNode->spawnEntry));

            /* Critical Section: The spawn free list is a critical resource which is accessed
             * from multiple contexts */
            key = HwiP_disable();
            MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);
            HwiP_restore (key);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      The function is used to close the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_closeLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal = MINUS_ONE;
    uint32_t u32DevIdx;

    /* Link is not operational: */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {
        //TODO:
#if 0
        ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_OFF;
        retVal = rl_fecssRfPwrOnOff(u32DevIdx, &ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
            retVal = MINUS_ONE;
        }
#endif
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the version of the various components
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deviceGetVersion(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t retVal = MINUS_ONE;

    retVal = rl_mmWaveDfpVerGet(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->version);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to get the device version */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        DebugP_logError("Error MMWave_deviceGetVersion\n");
        retVal   = MINUS_ONE;
    }

    DebugP_logInfo ("RF MMWave LIB Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_GenVerNum, ptrMMWaveMCB->version.z_MmwlLibVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_MmwlLibVersion.c_BuildVerNum);
    DebugP_logInfo ("RF MMWave LIB Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_Date, ptrMMWaveMCB->version.z_MmwlLibVersion.c_Month, ptrMMWaveMCB->version.z_MmwlLibVersion.c_Year);

    DebugP_logInfo ("RF FECSS LIB Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_FecssLibVersion.c_GenVerNum, ptrMMWaveMCB->version.z_FecssLibVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_FecssLibVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_FecssLibVersion.c_BuildVerNum);
    DebugP_logInfo ("RF FECSS LIB Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_FecssLibVersion.c_Date, ptrMMWaveMCB->version.z_FecssLibVersion.c_Month, ptrMMWaveMCB->version.z_FecssLibVersion.c_Year);

    DebugP_logInfo ("RFS ROM Version Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsRomVersion.c_GenVerNum, ptrMMWaveMCB->version.z_RfsRomVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_RfsRomVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_RfsRomVersion.c_BuildVerNum);
    DebugP_logInfo ("RFS ROM Version Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsRomVersion.c_Date, ptrMMWaveMCB->version.z_RfsRomVersion.c_Month, ptrMMWaveMCB->version.z_RfsRomVersion.c_Year);

    DebugP_logInfo ("RF RFS Patch Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_GenVerNum, ptrMMWaveMCB->version.z_RfsPatchVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_RfsPatchVersion.c_BuildVerNum);
    DebugP_logInfo ("RF RFS Patch Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_Date, ptrMMWaveMCB->version.z_RfsPatchVersion.c_Month, ptrMMWaveMCB->version.z_RfsPatchVersion.c_Year);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the mmWave Link
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_deinitMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t     retVal;
    T_RL_API_FECSS_DEV_PWR_OFF_CMD devPwrOffCmd;

    // For RPMF
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_OFF;
    retVal = rl_fecssDevClockCtrl(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
    }

    /* Power off the Device: */
    devPwrOffCmd.c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON;
    retVal = (int32_t)rl_fecssDevPwrOff(M_DFP_DEVICE_INDEX_0, &devPwrOffCmd);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to power off the BSS */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EDEINIT, retVal);
        retVal   = MINUS_ONE;
    }

    retVal = rl_mmWaveLinkDeInit(ptrMMWaveMCB->deviceMap);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to power off the BSS */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EDEINIT, retVal);
        retVal   = MINUS_ONE;
    }

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deinitLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t                 retVal = 0;
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uintptr_t               key;

    /* Deinitialize the mmWave Link: */
    retVal = MMWave_deinitMMWaveLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to deinitialize the mmWave link; error code is already setup */
        goto exit;
    }

    /* Cycle through and cleanup the active spawn lists: There might be some entries in
     * the Active list which still need to be handled but because we are shutting down
     * the module we simply flush out the entries. */
    key = HwiP_disable();
    ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnActiveList);
    while (ptrSpawnFxnNode != NULL)
    {
        /* Add this back to the free list: */
        MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);

        /* Get the next entry from the active list: */
        ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnActiveList);
    }
    HwiP_restore (key);

    /* Control comes here implies that the deinitialization of the module was successful. */
    retVal = 0;

exit:
    return retVal;
}

