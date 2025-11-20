/**
 *   @file  mmwave_link_mailbox.c
 *
 *   @brief
 *      The file implements the mmWave control module which interfaces
 *      with the mmWave Link API and communicates with the BSS
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
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>

#include <control/mmwave/mmwave.h>
#include <control/mmwave/include/mmwave_internal.h>


#define RFS_ALL_CAL_VALIDITY_STATUS 0xEF

/**************************************************************************
 ***************************** Local Functions ****************************
 **************************************************************************/
static int32_t MMWave_maskHostIRQ(T_DFP_PLT_HWI_HDL fd);
static int32_t MMWave_unmaskHostIRQ(T_DFP_PLT_HWI_HDL fd);
static int32_t MMWave_registerInterruptHandler(uint8_t c_deviceIndex, T_DFP_PLT_ISR_HANDLER p_isrHandler, void* p_value, T_DFP_PLT_HWI_HDL* p_hwiHdl);
static int32_t MMWave_deregisterInterruptHandler(T_DFP_PLT_HWI_HDL p_hwiHdl);
static int32_t MMWave_initMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
static int32_t MMWave_SysDelay(uint32_t delayMicroSec);
static uint32_t MMWave_TimeStamp(void);
static uint64_t MMWave_EnterCriticalRegion(void);
static void MMWave_ExitCriticalRegion(uint64_t key);

/* APPSS Debug Data */
uint32_t AppDebugData[16];
#define FECSS_TIMESTAMP_COUNTER_MASK    (0x0FFFFFU)
#define RUNTIMECALIBTEMP1 10.0
#define RUNTIMECALIBTEMP2 15.0

uint32_t MMWave_ISRHandle = 18U;
/**************************************************************************
 ************************ mmWave Link Functions ***************************
 **************************************************************************/

static int32_t MMWave_SysDelay(uint32_t delayMicroSec)
{
    ClockP_usleep(delayMicroSec/10);
    return 0;
}

static uint32_t MMWave_TimeStamp(void)
{
    return 0;
}

static uint64_t MMWave_EnterCriticalRegion(void)
{
    HwiP_disable();
    return 0;
}

static void MMWave_ExitCriticalRegion(uint64_t key)
{
    HwiP_enable();
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to mask the
 *      interrupts. Since the mmWave control module is using the Mailbox
 *      communication interface the driver is handling all the interrupt
 *      management. This function is a dummy stub.
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static int32_t MMWave_maskHostIRQ(T_DFP_PLT_HWI_HDL fd)
{
    uint32_t intNum = (uint32_t) fd;
    HwiP_enableInt(intNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to unmask the
 *      interrupts to indicate that the message has been successfully handled
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static int32_t MMWave_unmaskHostIRQ(T_DFP_PLT_HWI_HDL fd)
{
    /* The Mailbox driver indicates to the remote endpoint that the message
     * have been processed. The mmWave is now capable of receiving another message */
    uint32_t intNum = (uint32_t) fd;
    HwiP_disableInt(intNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to register the
 *      interrupt handler. In the case of the Mailbox the driver is responsible
 *      for the interrupt management. This function is a dummy stub
 *
 *  @param[in]  deviceIndex
 *      Device for which the interrupt is to be registered
 *  @param[in]  pHandler
 *      ISR Handler
 *  @param[in]  pValue
 *      Argument to the ISR
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Always returns 0
 */
static int32_t MMWave_registerInterruptHandler
(uint8_t deviceIndex, T_DFP_PLT_ISR_HANDLER isrHandler, \
                    void* value, T_DFP_PLT_HWI_HDL* hwiHdl)
{
    HwiP_Object HwiObj;
    HwiP_Params HwiParams;

    HwiP_Params_init(&HwiParams);
    HwiParams.intNum = (uint32_t)MMWave_ISRHandle;
    HwiParams.callback = (HwiP_FxnCallback) isrHandler;
    HwiParams.isPulse = 0;
    HwiP_construct(&HwiObj, &HwiParams);

    gMMWave_MCB.mmwavelinkInterruptFunc = isrHandler;
    *hwiHdl = (T_DFP_PLT_HWI_HDL *) &MMWave_ISRHandle;
    return 0;
}

static int32_t MMWave_deregisterInterruptHandler
(T_DFP_PLT_HWI_HDL hwiHdl)
{
    uint32_t intNum = (uint32_t) hwiHdl;
    HwiP_disableInt(intNum);
    return 0;
}



/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave Link
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
static int32_t MMWave_initMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal = MINUS_ONE;

    /* Reset the client context: */
    memset ((void *)&ptrMMWaveMCB->fecClientCbData, 0, sizeof(T_DFP_CLIENT_CB_DATA));

    ptrMMWaveMCB->deviceMap = M_DFP_DEVICE_MAP_DEVICE_0;

    /* Device Platform related initialization */
    ptrMMWaveMCB->fecClientCbData.c_PlatformType     = M_DFP_DEVICE_PLATFORM_TYPE;
#ifdef SOC_XWRL64XX
    ptrMMWaveMCB->fecClientCbData.c_DeviceType       = M_DFP_DEVICETYPE_6432;
#else
    ptrMMWaveMCB->fecClientCbData.c_DeviceType       = M_DFP_DEVICETYPE_1432;
#endif
    ptrMMWaveMCB->fecClientCbData.c_ApiErrorCheckDis = M_FALSE;
    ptrMMWaveMCB->fecClientCbData.h_ApiRespTimeoutMs = (UINT16)2U; /* DKS update later */
    ptrMMWaveMCB->fecClientCbData.c_ApiDbglogEn      = 1U;

    /* OSI related initialization */
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexCreate = &MMWave_osalMutexCreate;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexLock   = &MMWave_osalMutexLock;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexUnLock = &MMWave_osalMutexUnlock;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexDelete = &MMWave_osalMutexDelete;

    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemCreate = &MMWave_osalSemCreate;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemWait   = &MMWave_osalSemWait;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemSignal = &MMWave_osalSemSignal;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemDelete = &MMWave_osalSemDelete;

    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Queue.p_OsiSpawn = NULL;

    /* Platform related initialization */
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_Delay                = &MMWave_SysDelay;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_TimeStamp            = &MMWave_TimeStamp;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.w_TimeStampCounterMask = FECSS_TIMESTAMP_COUNTER_MASK;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_RegisterIsr          = &MMWave_registerInterruptHandler;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_DeRegisterIsr        = &MMWave_deregisterInterruptHandler;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_MaskIsr              = &MMWave_maskHostIRQ;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_UnMaskIsr            = &MMWave_unmaskHostIRQ;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_EnterCriticalRegion  = &MMWave_EnterCriticalRegion;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_ExitCriticalRegion   = &MMWave_ExitCriticalRegion;


    /* Debug related initialization */
    ptrMMWaveMCB->fecClientCbData.z_DbgCb.p_RfsdbgData  = &AppDebugData[0];
    ptrMMWaveMCB->fecClientCbData.z_DbgCb.p_Print  = NULL;

    /* Communication Interface related initialization */
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfOpen   = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfRead   = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfWrite  = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfClose  = M_NULL;


    retVal = (int32_t)rl_mmWaveLinkInit(ptrMMWaveMCB->deviceMap, ptrMMWaveMCB->fecClientCbData);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to initialize and power on the BSS. Encode the error code to account
         * for the subsystem error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    retVal = (int32_t)MMWave_LinkInitCommon(ptrMMWaveMCB, errCode);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        retVal   = MINUS_ONE;
        goto exit;
    }
    /* Link has been setup successfully. */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in] ptrCalibrationData
 *      Optional pointer to the calibration data which needs to be
 *      restored.
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
int32_t MMWave_openLink
(
    MMWave_MCB*                 ptrMMWaveMCB,
    int32_t*                    errCode
)
{
    int32_t                        retVal = MINUS_ONE;
    MMWave_temperatureStats        tempStatus = {0U};
    uint8_t                        tempBinIndex = 8U; /* Default Value*/
    T_RL_API_FECSS_TEMP_MEAS_CMD   tempCfg;
    float                          oldTempAvg, tempAvg, tempDiff;
    uint8_t                        tempChngd = 0;
    static uint8_t                 oldtempBinIndex = 0xFF;
    static int16_t                 mmwaveOldTemps[3] = {0xFFF,0xFFF,0xFFF};
    static uint16_t                clpcTxcodes[2];
    static int8_t                  clpcCalTemp;

    tempCfg.h_TempCtrlBitMask = 0x311U;

    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     ****************************************************************************************/

    /* Set the Temperature measurement configuration: */
    retVal = rl_fecssTempMeasCfg(M_DFP_DEVICE_INDEX_0, &tempCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the temp measurement configuration failed */
        DebugP_logError("rlTempMeasConfig error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Get temperature readings. */
    MMWave_getTemperatureReport(&tempStatus);

    /* Checking average of Tx, Rx, PM temperatures */
    oldTempAvg = (mmwaveOldTemps[0] + mmwaveOldTemps[1] + mmwaveOldTemps[2])/3;
    tempAvg = (tempStatus.tempValue[0] + tempStatus.tempValue[1] + tempStatus.tempValue[2])/3;
    tempDiff = fabs(tempAvg - oldTempAvg);

    /* Check if digital temperature is valid */
    if((tempStatus.tempStatus & 0x200U) == 0x200U)
    {
        if (tempAvg < 0)
        {
            /*LOW, The Device temperature is < 0deg C */
            tempBinIndex = 0x0U;
        }
        else if (tempAvg < 85)
        {
            /* MID, The Device temperature is >= 0deg C and < 85deg C */
            tempBinIndex = 0x8U;
        }
        else
        {
            /* HIGH, The Device temperature is >= 85deg C */
            tempBinIndex = 0x10U;
        }
    }

    if((tempDiff > RUNTIMECALIBTEMP1) && (tempBinIndex != oldtempBinIndex)) /* Tempbin crossing with hysteresis */
    { 
        oldtempBinIndex = tempBinIndex;
        if (ptrMMWaveMCB->openCfg.runTxCLPCCalib == 0)
        {
            /* Imp Note: Enable PD calibration (0xCE) only for safety (Monitor) applications */
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xCAU;
        }
        else
        {
           /* When TX CLPC is enabled, PD calibration is required even for non safety applications */
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xCEU;
        }
        tempChngd = 1;
        /* Reset static clutter history if applicable (on algo side) */
    }
    else if(tempDiff > RUNTIMECALIBTEMP2)
    {
        if (ptrMMWaveMCB->openCfg.runTxCLPCCalib == 0)
        {
            /* Imp Note: Enable PD calibration (0xCE) only for safety (Monitor) applications */
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xCAU;
            /* No need to reset static clutter history (on algo side) as tempbin is same */
        }
        else
        {
           /* When TX CLPC is enabled. PD calibration is required even for non safety applications */
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xCEU;
           /* Reset static clutter history if applicable (on algo side) */
        }
        tempChngd = 1;
    }
    else /* Just LUT updates */
    {

        /* Just run the Tx,Rx and LODIST calibrations */
        ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xC8U;
        tempChngd = 0;
        /* No need to reset static clutter history (on algo side) as tempbin is same */
    }
    
    /* Are we supporting runtime calibration or not? */
    if(ptrMMWaveMCB->openCfg.useRunTimeCalib != 0)
    {
        /* Are we supporting custom calibration or not? */
        if (ptrMMWaveMCB->openCfg.useCustomCalibration == true)
        {
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = ptrMMWaveMCB->openCfg.customCalibrationEnableMask;
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = tempBinIndex;
            retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
        }
        else
        {
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = tempBinIndex;
            retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
        }

        /* Check if run-time calibration return status. */
        if ((retVal != M_DFP_RET_CODE_OK) || 
        /* Check if all runtime calibrations run are passed */
        (ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp.h_CalRunStatus != ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask))
        {
            /* Error: Unable to set the calibration time unit */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALPERIOD, retVal);
            retVal   = MINUS_ONE;
            goto exit;
        }
    }

    /* Is CLPC calibration enabled */
    if(ptrMMWaveMCB->openCfg.runTxCLPCCalib != 0)
    {
        retVal   = MINUS_ONE;
        if(tempChngd == 1)
        {
            // Run the TX CLPC Calibration
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->c_CalMode = 0x0u;
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->h_TxStgOverrideCodes[0] = 0x0u;
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->h_TxStgOverrideCodes[1] = 0x0u;
	        ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->xc_OverrideCalTemp = 0;
            retVal = rl_fecssRlRuntimeTxClpcCal(M_DFP_DEVICE_INDEX_0, ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd, &ptrMMWaveMCB->openCfg.fecTxclpcCalRsp);
            if ((retVal != M_DFP_RET_CODE_OK) ||  (ptrMMWaveMCB->openCfg.fecTxclpcCalRsp.c_CalRunStatus != 1))
            {
                /* Error: Unable to run TX CLPC*/
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ETXCLPCCAL, retVal);
                retVal   = MINUS_ONE;
                goto exit;
            }
            /* Save the TX bias codes after CLPC run*/
            clpcTxcodes[0] = ptrMMWaveMCB->openCfg.fecTxclpcCalRsp.h_TxStgCodes[0];
            clpcTxcodes[1] = ptrMMWaveMCB->openCfg.fecTxclpcCalRsp.h_TxStgCodes[1];
            clpcCalTemp = ptrMMWaveMCB->openCfg.fecTxclpcCalRsp.xc_CalibrationTemp;
        }
        else
        {
            // Run the TX CLPC Calibration in Override mode
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->c_CalMode = 0x1u;
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->h_TxStgOverrideCodes[0] = clpcTxcodes[0];
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->h_TxStgOverrideCodes[1] = clpcTxcodes[1];
            ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd->xc_OverrideCalTemp = clpcCalTemp;

            retVal = rl_fecssRlRuntimeTxClpcCal(M_DFP_DEVICE_INDEX_0, ptrMMWaveMCB->openCfg.ptrfecTxclpcCalCmd, &ptrMMWaveMCB->openCfg.fecTxclpcCalRsp);
            if ((retVal != M_DFP_RET_CODE_OK) ||  (ptrMMWaveMCB->openCfg.fecTxclpcCalRsp.c_CalRunStatus != 1))
            {
                /* Error: Unable to run TX CLPC*/
                *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ETXCLPCCAL, retVal);
                retVal   = MINUS_ONE;
                goto exit;
            }

        }
    }

    if(tempChngd == 1)
    {
        mmwaveOldTemps[0] = tempStatus.tempValue[0];
        mmwaveOldTemps[1] = tempStatus.tempValue[1];
        mmwaveOldTemps[2] = tempStatus.tempValue[2];
        tempChngd = 0;
    }
    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave link.
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
int32_t MMWave_initLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t                 retVal = 0;
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uint32_t                index;

    /* Initialize and setup the spawn lists */
    for (index = 0U; index < MMWAVE_MAX_NUM_SPAWN_LIST; index++)
    {
        /* Get the pointer to the spawn node */
        ptrSpawnFxnNode = &ptrMMWaveMCB->spawnTable[index];

        /* Initialize the spawn node */
        memset ((void*)ptrSpawnFxnNode, 0, sizeof(MMWave_SpawnFxnNode));

        /* Add the node to the free list: */
        MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);
    }

    /* Reset the active list: */
    ptrMMWaveMCB->ptrSpawnFxnActiveList = NULL;

    /* YES: Setup the mmWave Link */
    if (MMWave_initMMWaveLink (ptrMMWaveMCB, errCode) == 0)
    {
        /* Successfully initialized the mmWave Link: */
        retVal = 0;
    }
    else
    {
        /* Error: Unable to setup the mmWave Link */
        retVal = MINUS_ONE;
    }

    return retVal;
}

