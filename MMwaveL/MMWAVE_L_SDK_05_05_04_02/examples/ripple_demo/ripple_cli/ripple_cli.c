/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* Standard Include Files. */
#include <stdio.h>
/* MCU Plus Include Files. */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
/*OS Include Files. */
#include "FreeRTOS.h"
#include "task.h"
/*Local Header Files*/
#include "ripple_cli.h"
#include "ripple_link.h"
#include "ripple_flash.h"
#include "ripple.h"
/*Drivers Include Files*/
#include <drivers/power.h>
#include <utils/mathutils/mathutils.h>
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>
/*Firmware Include Files*/
#include <mmwavelink/include/rl_device.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#if defined (SOC_XWRL64XX)
#define DEVICE_STRING "xWRL6432"
#else
#define DEVICE_STRING "xWRL1432"
#endif
/*! Min macro */
#define MIN(x,y) ((x) < (y) ? (x) : (y))

/*Uncomment this for Low power mode verification - bit-matching with uninterrupted power mode*/
//#define LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION

 /* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_STORE_MAGIC            (0x7CB28DF9U)
/* ========================================================================== */
/*                         Global & External Variables                        */
/* ========================================================================== */
extern uint8_t gATECalibDataStorage[];
extern MMWave_MCB gMMWave_MCB;
extern MmwDemo_MSS_MCB gMmwMssMCB;


CLI_MCB     gCLI;
float       gSocClk = 40000000; /*Hz TODO: Map from Syscfg generated config*/

TaskHandle_t gCliTask;
StaticTask_t gCliTaskObj;
StackType_t  gCliTskStack[CLI_TASK_STACK_SIZE] __attribute__((aligned(32)));

uint8_t gATECalibDataStorage[(ATE_CALIB_DATA_LENGTH + 4)] __attribute__((aligned(8))) = {0};
MmwDemo_calibData gFactoryCalibDataStorage __attribute__((aligned(8))) = {0};
uint32_t gnumberofchirpstosend=100;
volatile unsigned long long demoStartTime = 0, a2;
volatile unsigned long long b,c,f;
/* ========================================================================== */
/*                         External Functions                                 */
/* ========================================================================== */
extern void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern int32_t MmwDemo_openSensor();
extern int32_t MmwDemo_configSensor(void);
extern int32_t ripple_link_startSensor(void);
/* ========================================================================== */
/*                  CLI Functions Declarations                                */
/* ========================================================================== */
static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[]);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
static void CLI_task(void* args)
{   
    uint8_t cmdString[READ_LINE_BUFSIZE];
    char* tokenizedArgs[CLI_MAX_ARGS];
    char* ptrCLICommand;
    char delimitter[] = " \r\n";
    int32_t status, cliStatus;
    uint32_t argIndex,index;
    CLI_CmdTableEntry*      ptrCLICommandEntry;

     /* Loop around forever: */
    while (1)
    {
        /* Demo Prompt: */
        DebugP_log (gCLI.cfg.cliPrompt);

        /* Reset the command string: */
        memset ((void *)&cmdString[0], 0, sizeof(cmdString));
        
        /*Reading Commands from UART Port*/
        status = DebugP_readLine((char*)&cmdString[0], READ_LINE_BUFSIZE);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error reading\n");
        }

        /* Reset all the tokenized arguments: */
        memset ((void *)&tokenizedArgs, 0, sizeof(tokenizedArgs));
        argIndex      = 0;
        ptrCLICommand = (char*)&cmdString[0];

        /* Set the CLI status: */
        cliStatus = -1;

        /* The command has been entered we now tokenize the command message */
        while (1)
        {
            /* Tokenize the arguments: */
            tokenizedArgs[argIndex] = strtok(ptrCLICommand, delimitter);
            if (tokenizedArgs[argIndex] == NULL)
                break;

            /* Increment the argument index: */
            argIndex++;
            if (argIndex >= CLI_MAX_ARGS)
                break;

            /* Reset the command string */
            ptrCLICommand = NULL;            
        }

        /* Were we able to tokenize the CLI command? */
        if (argIndex == 0)
            continue;

        /* Cycle through all the registered CLI commands: */
        for (index = 0; index < gCLI.numCLICommands; index++)
        {
            ptrCLICommandEntry = &gCLI.cfg.tableEntry[index];

            /* Do we have a match? */
            if (strcmp(ptrCLICommandEntry->cmd, tokenizedArgs[0]) == 0)
            {
                /* YES: Pass this to the CLI registered function */
                cliStatus = ptrCLICommandEntry->cmdHandlerFxn (argIndex, tokenizedArgs);
                if (cliStatus == 0)
                {
                    DebugP_log ("Done\r\n");
                }
                else
                {
                    DebugP_log ("Error %d\r\n", cliStatus);
                }
                break;
            }
        }

        
        /* Did we get a matching CLI command? */
        if (index == gCLI.numCLICommands)
        {
            /* Was the CLI command found? */
            if (cliStatus == -1)
            {
                /* No: The command was still not found */
                DebugP_log ("'%s' is not recognized as a CLI command\r\n", tokenizedArgs[0]);
            }
        }
    }
    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.cliInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);
}

/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command SensorStop
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveSensorStop (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        DebugP_log ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    int32_t err;
    /* Populate the SensorStop configuration: */
    gMmwMssMCB.sensorStop.c_FrameStopMode  = atoi (argv[1]);
    MMWave_stop(gMmwMssMCB.ctrlHandle,&err);
    MMWave_close(gMmwMssMCB.ctrlHandle,&err);
    /* Delete the exisitng profile as we receive a new configuration*/
    MMWave_delProfile(gMmwMssMCB.ctrlHandle,gMmwMssMCB.mmwCtrlCfg.frameCfg[0].profileHandle[0],&err);
    /* Free up all the edma channels and close the EDMA interface */
    mmwDemo_freeDmaChannels(gEdmaHandle[0]);
    Drivers_edmaClose();
    EDMA_deinit();
    /* Demo Stopped*/
    gMmwMssMCB.oneTimeConfigDone = 0;
    /* Re-init the EDMA interface for next configuration*/
    EDMA_init();
    Drivers_edmaOpen();
    gMmwMssMCB.sensorStopCount = 1;
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is a Internal Function for calculating Number of Rx from Rx Map
 *
 *  @retval
 *      Not Applicable.
 */
int rxMaptoRxNum(int n){
    int a[10],i;
    int rxNum=0;    
    for(i=0;n>0;i++)    
    {    
        a[i]=n%2;    
        n=n/2;    
    }
    for(i=i-1;i>=0;i--)    
    { 
        rxNum+=a[i];       
    }         
    return rxNum;  
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command ChannelCfg
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveChannelCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Populate the frame configuration: */
    gMmwMssMCB.channelCfg.h_RxChCtrlBitMask  = atoi (argv[1]);
    gMmwMssMCB.channelCfg.h_TxChCtrlBitMask  = atoi (argv[2]);
    gMmwMssMCB.channelCfg.c_MiscCtrl         = atoi (argv[4]);
    if((gMmwMssMCB.channelCfg.h_RxChCtrlBitMask & 2) == 2)
    {
        gMmwMssMCB.angleDimension = 2;
    }
    else
    {
        gMmwMssMCB.angleDimension = 1;
    }
    gMmwMssMCB.rxnum=rxMaptoRxNum(gMmwMssMCB.channelCfg.h_RxChCtrlBitMask);
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command FrameCfg
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveFrameCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate the frame configuration: */
    gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst      = atoi (argv[1]);
    gMmwMssMCB.frameCfg.c_NumOfChirpsAccum        = atoi (argv[2]);
    gMmwMssMCB.burstPeriod                        = atof (argv[3]); /*us*/
    gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame      = atoi (argv[4]);
    gMmwMssMCB.frameCfg.w_FramePeriodicity        = ((atof (argv[5])) * gSocClk)/1000; /* x crystal_clk_MHz x 1000*/
    gMmwMssMCB.frameCfg.h_NumOfFrames             = atoi (argv[6]);

    gMmwMssMCB.frameCfg.w_BurstPeriodicity = 10.0 * gMmwMssMCB.burstPeriod;
    gMmwMssMCB.numDopplerBins = mathUtils_pow2roundup(gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame);
    gnumberofchirpstosend=gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command ChirpTimingCfg
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveChirpTimingCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* TODO: Assumes Low res clock and to change */
    /* Populate the Chirp Timing configuration: */
    gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime      = 10.0 * atof (argv[1]);
    gMmwMssMCB.profileTimeCfg.h_ChirpAdcStartTime  = (atoi (argv[2])) << 10; /*num of skip samples*/
    /* Front End Firmware expects the ChirpTxStartTime in resolution of 20ns, hence multiply by 50 */
    gMmwMssMCB.profileTimeCfg.xh_ChirpTxStartTime  = 50.0 * (atof (argv[3]));
    gMmwMssMCB.chirpSlope                          = atof (argv[4]); /*MHz/us*/
    gMmwMssMCB.startFreq                           = atof (argv[5]); /*GHz*/
    #ifdef SOC_XWRL64XX
    gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope  = (gMmwMssMCB.chirpSlope * 1048576.0)/(3* 100 * 100);
    gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart   = (gMmwMssMCB.startFreq * 1000.0 * 256.0)/(300);
    #else
    gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope  = (gMmwMssMCB.chirpSlope * 1048576.0)/(4* 100 * 100);
    gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart   = (gMmwMssMCB.startFreq * 1000.0 * 256.0)/(400);
    #endif
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command ChirpCommonCfg
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveChirpCommonCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Populate the Chirp Common configuration: */
    gMmwMssMCB.profileComCfg.c_DigOutputSampRate  = atoi (argv[1]); /*Range 8 to 100*/
    gMmwMssMCB.profileComCfg.c_DigOutputBitsSel   = atoi (argv[2]);
    gMmwMssMCB.profileComCfg.c_DfeFirSel          = atoi (argv[3]);
    gMmwMssMCB.profileComCfg.h_NumOfAdcSamples    = atoi (argv[4]);
    gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel  = atoi (argv[5]);
    /* Chirp Common Config: MiscSettings and HPFFastInit Duration */
    gMmwMssMCB.profileComCfg.c_MiscSettings       = 0U;
    gMmwMssMCB.profileComCfg.c_HpfFastInitDuration= 15U; /* 15uSec*/
    gMmwMssMCB.profileComCfg.h_ChirpRampEndTime   = 10.0 * atof (argv[6]);
    gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel      = atoi (argv[7]);
    gMmwMssMCB.adcSamplingRate = 100.0/gMmwMssMCB.profileComCfg.c_DigOutputSampRate; /*Range 1MHz to 12.5MHz*/
    gMmwMssMCB.numRangeBins = mathUtils_pow2roundup(gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/2; /*Real only sampling*/
    if (gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 1 || gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 0)
    {
        /* TDM-MIMO*/
        gMmwMssMCB.isBpmEnabled = 0;
    }
    else if (gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel == 4)
    {
        /* BPM-MIMO*/
        gMmwMssMCB.isBpmEnabled = 1;
    }
    else
    {
        CLI_write ("Error: c_ChirpTxMimoPatSel must have value either 1 (TDM-MIMO) or 4 (BPM-MIMO)\n");
        return -1;
    }
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end
 *
 *  @param[in]  ptrCalibData     	Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibRestore(MmwDemo_calibData  *ptrCalibData)
{
    int32_t    retVal = 0;
    uint32_t   flashOffset;

    /* Get Flash Offset */
    flashOffset = gMmwMssMCB.factoryCalCfg.flashOffset;

    /* Read calibration data */
    if(ripple_flashRead(flashOffset, (uint8_t *)ptrCalibData, sizeof(MmwDemo_calibData) )< 0)
    {
        /* Error: only one can be enable at at time */
        CLI_write ("Error: MmwDemo failed when reading calibration data from flash.\r\n");
        return -1;
    }

    /* Validate Calib data Magic number */
    if(ptrCalibData->magic != MMWDEMO_CALIB_STORE_MAGIC)
    {
        /* Header validation failed */
        CLI_write ("Error: MmwDemo calibration data header validation failed.\r\n");
        return -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibSave(MmwDemo_calibData  *ptrCalibrationData)
{
    uint32_t				flashOffset;
    int32_t 				retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.factoryCalCfg.flashOffset;

    /* Flash calibration data */
    retVal = ripple_flashWrite(flashOffset, (uint8_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if(retVal < 0)
    {
        /* Flash write failed */
        CLI_write ("Error: MmwDemo failed flashing calibration data with error[%d].\n", retVal);
    }

    return retVal;
}
/**
 *  @b Description
 *  @n
 *      This is the Function for performing factory Calibrations. It is called before sensorStart by sensorStart handler Function.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmwDemo_factoryCal(void)
{
    ATE_CalibData    *ateCalib = (ATE_CalibData *)&gATECalibDataStorage;
    uint16_t         calRfFreq = 0U;
    MMWave_calibCfg  factoryCalCfg = {0U};
    int32_t          retVal = SystemP_SUCCESS;
    int32_t          errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t          mmWaveErrorCode;
    int16_t          subsysErrorCode;

    /* Enable sensor boot time calibration: */
    factoryCalCfg.isFactoryCalEnabled = true;
    /*
    * @brief  FECSS RFS Boot calibration control:
    * | bits [0] | RESERVED
    * | bits [1] | VCO calibration ON/OFF control
    * | bits [2] | PD calibration ON/OFF control
    * | bits [3] | LODIST calibration ON/OFF control.
    * | bits [4] | RESERVED 
    * | bits [5] | RX IFA calibration ON/OFF control.
    * | bits [6] | RX Gain calibration ON/OFF control.
    * | bits [7] | TX power calibration ON/OFF control.
    */
    factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask = 0xCAU;
    factoryCalCfg.fecRFFactoryCalCmd.c_MiscCalCtrl = 0x0U;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalRxGainSel = gMmwMssMCB.factoryCalCfg.rxGain;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[0] = gMmwMssMCB.factoryCalCfg.txBackoffSel;
    factoryCalCfg.fecRFFactoryCalCmd.c_CalTxBackOffSel[1] = gMmwMssMCB.factoryCalCfg.txBackoffSel;

#if SOC_XWRL64XX
    /* Calculate Calibrtaion Rf Frequency. */
    calRfFreq = (gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart) + \
                ((((gMmwMssMCB.chirpSlope * 256.0)/300) * (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime * 0.1)) / 2);
    factoryCalCfg.fecRFFactoryCalCmd.xh_CalRfSlope = 0x4Du; /* 2.2Mhz per uSec*/
#else
    /* Calculate Calibrtaion Rf Frequency. */
    calRfFreq = (gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart) + \
                ((((gMmwMssMCB.chirpSlope * 256.0)/400) * (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime * 0.1) / 2);
    factoryCalCfg.fecRFFactoryCalCmd.xh_CalRfSlope = 0x3Au; /* 2.2Mhz per uSec*/
#endif

    factoryCalCfg.fecRFFactoryCalCmd.h_CalRfFreq = calRfFreq;
    factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[0] = 0x3;
    factoryCalCfg.fecRFFactoryCalCmd.c_TxPwrCalTxEnaMask[1] = 0x1;

    factoryCalCfg.ptrAteCalibration = (uint8_t *)&gATECalibDataStorage[4];

    if(gMmwMssMCB.factoryCalCfg.restoreEnable != 0U)
    {
        if(MmwDemo_calibRestore(&gFactoryCalibDataStorage) < 0)
        {
            CLI_write ("Error: MmwDemo failed restoring calibration data from flash.\r\n");
            MmwDemo_debugAssert (0);
        }

        /* Populate calibration data pointer. */
        factoryCalCfg.ptrFactoryCalibData = &gFactoryCalibDataStorage.calibData;

        /* Disable boot calibration. */
        factoryCalCfg.isFactoryCalEnabled = false;
    }

    retVal = MMWave_factoryCalibConfig(gMmwMssMCB.ctrlHandle, &factoryCalCfg, &errCode);
    if (retVal != SystemP_SUCCESS)
    {

        /* Error: Unable to perform boot calibration */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        CLI_write ("Error: mmWave Control Initialization failed [Error code %d] [errorLevel %d] [mmWaveErrorCode %d] [subsysErrorCode %d]\n", errCode, errorLevel, mmWaveErrorCode, subsysErrorCode);
        if (mmWaveErrorCode == MMWAVE_ERFSBOOTCAL)
        {
            CLI_write ("Debug: Boot Calibration failure\n");
            ateCalib->validityFlag = 0x0U; /* Flag to indicate to re-run ATE calibration */
        }
        else
        {
            MmwDemo_debugAssert (0);
        }
    }

    /* Save calibration data in flash */
    if(gMmwMssMCB.factoryCalCfg.saveEnable != 0)
    {
        gFactoryCalibDataStorage.magic = MMWDEMO_CALIB_STORE_MAGIC;
        retVal = rl_fecssRfFactoryCalDataGet(M_DFP_DEVICE_INDEX_0, &gFactoryCalibDataStorage.calibData);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            /* Error: Calibration data restore failed */
            CLI_write("Error: MMW demo failed rl_fecssRfFactoryCalDataGet with Error[%d]\n", retVal);
            retVal = SystemP_FAILURE;
        }

        /* Save data in flash */
        retVal = MmwDemo_calibSave(&gFactoryCalibDataStorage);
        if(retVal <0) {
            CLI_write("Error: MMW demo failed Calibration Save with Error[%d]\n", retVal);
            MmwDemo_debugAssert (0);
        }
    }

    return retVal;
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command SensorStart
 *
 *  @retval
 *      Not Applicable.
 */
int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[])
{
    int32_t errCode = 0;
    int32_t retVal = SystemP_SUCCESS;
    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Populate the SensorStart configuration: */
    gMmwMssMCB.sensorStart.c_FrameTrigMode      = atoi (argv[1]);
    gMmwMssMCB.sensorStart.c_ChirpStartSigLbEn  = atoi (argv[2]); /* TODO: Remove */
    gMmwMssMCB.sensorStart.c_FrameLivMonEn      = atoi (argv[3]);
    gMmwMssMCB.sensorStart.w_FrameTrigTimerVal  = atoi (argv[4]);

    gMmwMssMCB.chirpsize=2*gMmwMssMCB.profileComCfg.h_NumOfAdcSamples*gMmwMssMCB.rxnum;
    /* Calculate CRD NSLOPE Magnitude */
    {
        float scale, rfBandwidth, rampDownTime;
#ifdef SOC_XWRL64XX
        scale = 65536./(3*100*100);
#else
        scale = 65536./(4*100*100);
#endif
        rfBandwidth = (gMmwMssMCB.profileComCfg.h_ChirpRampEndTime*0.1) * gMmwMssMCB.chirpSlope; /*In MHz/usec*/
        rampDownTime = MIN((gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime*0.1-1.0), 6.0); /*In usec*/
        gMmwMssMCB.profileComCfg.h_CrdNSlopeMag = (uint16_t)  (scale * rfBandwidth / rampDownTime + 0.5);
    }
    /* Chirp Timing Configuration */
    {
        gMmwMssMCB.profileTimeCfg.h_ChirpTxEnSel       = gMmwMssMCB.channelCfg.h_TxChCtrlBitMask;
        gMmwMssMCB.profileTimeCfg.h_ChirpTxBpmEnSel    = 0U;
    }
    /* FEC Power Config */
   {
        /* RDIF clock enable control.*/
        gMmwMssMCB.channelCfg.c_MiscCtrl = 1U << M_RL_RF_MISC_CTRL_RDIF_CLK;
    }

    /* FECSS RF Power ON*/
    retVal = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &gMmwMssMCB.channelCfg);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        CLI_write ("Error: FECSS RF Power ON/OFF failed\r\n");
        retVal = SystemP_FAILURE;
    }
    
    /* Perform factory Calibrations. */
    retVal = mmwDemo_factoryCal();
    if(retVal != SystemP_SUCCESS)
    {
        CLI_write ("Error: mmWave factory calibration failed\r\n");
        retVal = SystemP_FAILURE;
    }
    CLI_MMWStart();

    return errCode;
}
/**
 *  @b Description
 *  @n  A Sub function called by 
 *      This is the Function that will open Sensor , populate default chirp Config and Start the Sensor
 *
 *  @retval
 *      Not Applicable.
 */
int32_t CLI_MMWStart(void)
{
    int32_t errCode = 0;
    demoStartTime = PRCMSlowClkCtrGet();
    
    gMmwMssMCB.mmwOpenCfg.useRunTimeCalib = true;
    gMmwMssMCB.mmwOpenCfg.useCustomCalibration = false;
    gMmwMssMCB.mmwOpenCfg.customCalibrationEnableMask = 0U;
    gMmwMssMCB.mmwOpenCfg.fecRDIFCtrlCmd.c_RdifEnable = M_RL_FECSS_RDIF_DIS;
    gMmwMssMCB.mmwOpenCfg.fecRDIFCtrlCmd.h_RdifSampleCount = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples;
    errCode = MmwDemo_openSensor();
    if(errCode != 0)
    {
        goto exit;
    }
    Mmwave_populateDefaultChirpControlCfg (&gMmwMssMCB.mmwCtrlCfg); /* regular frame config */
    errCode = MmwDemo_configSensor();
    if(errCode != 0)
    {
        goto exit;
    }

    errCode = ripple_link_startSensor();

    if(errCode != 0)
    {
        goto exit;
    }
exit:
    return errCode;
}

static int32_t CLI_help (int32_t argc, char* argv[])
{
    uint32_t    index;

    /* Display the banner: */
    CLI_write ("Help: This will display the usage of the CLI commands\n");
    CLI_write ("Command: Help Description\n");

    /* Cycle through all the registered CLI commands: */
    for (index = 0; index < gCLI.numCLICommands; index++)
    {
        /* Display the help string*/
        CLI_write ("%s: %s\n",
                    gCLI.cfg.tableEntry[index].cmd,
                   (gCLI.cfg.tableEntry[index].helpString == NULL) ?
                    "No help available" :
                    gCLI.cfg.tableEntry[index].helpString);
    }

    return 0;
}

void CLI_write (const char* format, ...)
{
    va_list     arg;
    char        logMessage[256];
    int32_t     sizeMessage;
    UART_Transaction trans;

    UART_Transaction_init(&trans);

    /* Format the message: */
    va_start (arg, format);
    sizeMessage = vsnprintf (&logMessage[0], sizeof(logMessage), format, arg);
    va_end (arg);

    /* If CLI_write is called before CLI init has happened, return */
    if (gCLI.cfg.UartHandle == NULL)
    {
        return;
    }

    trans.buf   = &logMessage[0U];
    trans.count = sizeMessage;

    /* Log the message on the UART CLI console: */
    /* Blocking Mode: */
    UART_write (gCLI.cfg.UartHandle, &trans);
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command CLI_MMWaveFactoryCalConfig
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveFactoryCalConfig (int32_t argc, char* argv[])
{
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\r\n");
        return -1;
    }

    /* Validate inputs */
    /* <Save> and <re-store> shouldn't be enabled in CLI*/
    if ( ((uint32_t) atoi(argv[1]) == 1) && ((uint32_t) atoi(argv[2] ) == 1))
    {
        CLI_write ("Error: Save and Restore can be enabled only one at a time\r\n");
        return -1;
    }

    /* Validate inputs */
    /* RxGain should be between 30db to 40db */
    if ( ((uint32_t) atoi(argv[3]) > 40U) || ((uint32_t) atoi(argv[3] ) < 30U))
    {
        CLI_write ("Error: Valid RxGain should be between 30db to 40db\r\n");
        return -1;
    }

    /* txBackoffSel should be between 0db to 26db */
    if ((uint32_t) atoi(argv[4]) > 26U)
    {
        CLI_write ("Error: Valid txBackoffSel should be between 0db to 26db\r\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.factoryCalCfg.saveEnable = (uint32_t) atoi(argv[1]);
    gMmwMssMCB.factoryCalCfg.restoreEnable = (uint32_t) atoi(argv[2]);
    gMmwMssMCB.factoryCalCfg.rxGain = (uint32_t) atoi(argv[3]);
    gMmwMssMCB.factoryCalCfg.txBackoffSel = (uint32_t)(2 * atoi(argv[4]));
    sscanf(argv[5], "0x%x", &gMmwMssMCB.factoryCalCfg.flashOffset);

    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the Handler Function for CLI command LowPwrModeEnable
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t CLI_MMWaveLowPwrModeEnable(int32_t argc, char* argv[])
{
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.lowPowerMode = atoi (argv[1]);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This function is called when the user wants to initialise the CLI.
 *
 *  @param taskPriority
 *      Priority you want to initialise the CLI with.  
 *
 *  @retval
 *      Not Applicable.
 */
void ripple_cli_init (uint8_t taskPriority){

    CLI_Cfg     cliCfg;
    uint32_t    cmds_cnt;

    cmds_cnt=0;

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /*Initialize flash Interface*/
    ripple_flashInit();


    /*Configuring the CLI*/
    cliCfg.cliPrompt="mmwDemo:/>";
    cliCfg.UartHandle= gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority=CLI_TASK_PRIORITY;
    

    cliCfg.tableEntry[cmds_cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<FrameStopMode>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveSensorStop;
    cmds_cnt++;

    cliCfg.tableEntry[cmds_cnt].cmd            = "channelCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<RxChCtrlBitMask> <TxChCtrlBitMask> <OvrrideCtrl> <MiscCtrl>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveChannelCfg;
    cmds_cnt++;

    cliCfg.tableEntry[cmds_cnt].cmd            = "chirpComnCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<DigOutputSampRate_Decim> <DigOutputBitsSel> <DfeFirSel> <NumOfAdcSamples> <ChirpTxMimoPatSel> <ChirpRampEndTime> <ChirpRxHpfSel>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveChirpCommonCfg;
    cmds_cnt++;

    cliCfg.tableEntry[cmds_cnt].cmd            = "chirpTimingCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<ChirpIdleTime> <ChirpAdcSkipSamples> <ChirpTxStartTime> <ChirpRfFreqSlope> <ChirpRfFreqStart>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveChirpTimingCfg;
    cmds_cnt++;


    cliCfg.tableEntry[cmds_cnt].cmd            = "frameCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<NumOfChirpsInBurst> <NumOfChirpsAccum> <BurstPeriodicity> <NumOfBurstsInFrame> <FramePeriodicity> <NumOfFrames>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveFrameCfg;
    cmds_cnt++;

    cliCfg.tableEntry[cmds_cnt].cmd            = "lowPowerCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<LowPowerModeEnable>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveLowPwrModeEnable;
    cmds_cnt++;

    cliCfg.tableEntry[cmds_cnt].cmd            = "factoryCalibCfg";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<save enable> <restore enable> <rxGain> <backoff0> <Flash offset>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveFactoryCalConfig;
    cmds_cnt++;

    
    cliCfg.tableEntry[cmds_cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cmds_cnt].helpString     = "<FrameTrigMode> <LoopBackEn> <FrameLivMonEn> <FrameTrigTimerVal>";
    cliCfg.tableEntry[cmds_cnt].cmdHandlerFxn  = CLI_MMWaveSensorStart;
    cmds_cnt++;  

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        DebugP_log ("Error: Unable to open the CLI\r\n");
        return;
    }
}



/**
 *  @b Description
 *  @n
 *      Intermediate function, it calls the CLI task.
 *
 *  @param ptrCLICfg
 *      takes in the user CLI Configs as pointer of type CLI_Cfg structure.  
 *
 *  @retval
 *      Not Applicable.
 */
int32_t CLI_open (CLI_Cfg* ptrCLICfg){
    uint32_t        index;
    /* Sanity Check: Validate the arguments */
    if (ptrCLICfg == NULL)
        return -1;
    /* Initialize the CLI MCB: */
    memset ((void*)&gCLI, 0, sizeof(CLI_MCB));
    /* Copy over the configuration: */
    memcpy ((void *)&gCLI.cfg, (void *)ptrCLICfg, sizeof(CLI_Cfg));
    /* Cycle through and determine the number of supported CLI commands: */
    for (index = 0; index < CLI_MAX_CMD; index++)
    {
        /* Do we have a valid entry? */
        if (gCLI.cfg.tableEntry[index].cmd == NULL)
        {
            /* NO: This is the last entry */
            break;
        }
        else
        {
            /* YES: Increment the number of CLI commands */
            gCLI.numCLICommands = gCLI.numCLICommands + 1;
        }
    }
    /* Do we have a CLI Prompt specified?  */
    if (gCLI.cfg.cliPrompt == NULL)
        gCLI.cfg.cliPrompt = "CLI:/>";

    /* The CLI provides a help command by default:
     * - Since we are adding this at the end of the table; a user of this module can also
     *   override this to provide its own implementation. */
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmd           = "help";
    gCLI.cfg.tableEntry[gCLI.numCLICommands].helpString    = NULL;
    gCLI.cfg.tableEntry[gCLI.numCLICommands].cmdHandlerFxn = CLI_help;

    /* Increment the number of CLI commands: */
    gCLI.numCLICommands++;

    gCliTask = xTaskCreateStatic( CLI_task,   /* Pointer to the function that implements the task. */
                                  "cli_task_main", /* Text name for the task.  This is to facilitate debugging only. */
                                  CLI_TASK_STACK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,              /* We are not using the task parameter. */
                                  ptrCLICfg->taskPriority,      /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gCliTskStack,  /* pointer to stack base */
                                  &gCliTaskObj );    /* pointer to statically allocated task object memory */
    configASSERT(gCliTask != NULL);

    return 0;    
}
