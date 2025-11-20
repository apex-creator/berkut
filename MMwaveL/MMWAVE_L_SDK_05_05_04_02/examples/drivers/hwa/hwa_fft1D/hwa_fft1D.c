/*
 *   @file  hwa_fft1D.c
 *
 *   @brief
 *      Test application code for the HWA FFT1D feature
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020-2023 Texas Instruments, Inc.
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

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <utils/mathutils/mathutils.h>

/* mmWave driver Include Files: */
#include <kernel/dpl/ClockP.h>
#include <ti_drivers_config.h>
#include <ti_board_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SystemP.h>
#include <drivers/hwa.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <utils/testlogger/logger.h>

/* FreeRTOS Task declarations. */
#define APP_CTRL_TASK_PRI    (7U)
#define APP_CTRL_TASK_STACK_SIZE (8*1024U)

/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define hwa_src     CSL_APP_HWA_DMA0_RAM_BANK0_BASE
#define hwa_dst     CSL_APP_HWA_DMA0_RAM_BANK2_BASE

/* Warning for src/dst from HWA MEM bank */
#define STR(X) #X
#define DEFER(M,...) M(__VA_ARGS__)
#define CUSTOM_WARNING(X) _Pragma(STR(GCC warning(X " at line " DEFER(STR,__LINE__))))
#define abs(x) ((x)<0 ? -(x) : (x))

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(void *arg);

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/* Task Stack variables.*/
uint8_t gMmwCtrlTskStack[APP_CTRL_TASK_STACK_SIZE] __attribute__((aligned(32)));

/* Global Variable: This is the handle to the mmWave module */
MMWave_Handle     gMMWaveHandle;
HWA_Handle        gHwaHandle;
SemaphoreP_Object gHwaDoneSem;

/**
 * @brief
 *  Initialize the MCPI Log Message Buffer
 */
MCPI_LOGBUF_INIT(9216);

/**************************************************************************
 ************************** Extern Definitions ****************************
 **************************************************************************/
extern void Mmwave_populateDefaultOpenCfg (MMWave_OpenCfg* ptrOpenCfg);
extern void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg);
extern void Mmwave_populateDefaultCalibrationCfg (MMWave_CalibrationCfg* ptrCalibrationCfg);
extern void Mmwave_ctrlTask(void* args);

/**************************************************************************
 *********************** mmWave Unit Test Functions ***********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Test implementation
 *
 *  @retval
 *      Not Applicable.
 */
void Mmwave_HwaConfig (void)
{
    HWA_ParamConfig     paramCfg;
    HWA_CommonConfig    commonCfg;
    uint32_t            paramIdx;
    uint32_t            numSamples = 128U; //based on h_NumOfAdcSamples in common_full.c
    int32_t             status = SystemP_SUCCESS;
    
    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }

    paramIdx = 0;
    memset(&paramCfg , 0, sizeof(HWA_ParamConfig));
    paramCfg.triggerMode = HWA_TRIG_MODE_RESERVED1;
    paramCfg.accelMode = HWA_ACCELMODE_FFT;
    paramCfg.source.srcAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(hwa_src);
    paramCfg.source.srcSign = HWA_SAMPLES_SIGNED;
    paramCfg.source.srcAcnt = numSamples - 1U;
    paramCfg.source.srcAIdx = sizeof(int16_t);
    paramCfg.source.srcBcnt = 0U;
    paramCfg.source.srcBIdx = 0U;  /* dont care as bcnt is 0 */
    paramCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.source.srcScale = 0x8;
    paramCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    paramCfg.source.srcConjugate = HWA_FEATURE_BIT_DISABLE;    /* no conjugate */
    paramCfg.source.bpmEnable = HWA_FEATURE_BIT_DISABLE;
    paramCfg.dest.dstAddr = HWADRV_ADDR_TRANSLATE_CPU_TO_HWA(hwa_dst);
    paramCfg.dest.dstSkipInit = 0U;
    paramCfg.dest.dstAcnt = numSamples - 1U;
    paramCfg.dest.dstAIdx = 2*sizeof(int16_t); /* x 2 for real and complex output */
    paramCfg.dest.dstBIdx = 0U;    /* dont care as bcnt is 0 */
    paramCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    paramCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    paramCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
    paramCfg.dest.dstScale = 0U;
    paramCfg.dest.dstConjugate = HWA_FEATURE_BIT_DISABLE;      /* no conjugate */
    paramCfg.accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_ENABLE;
    paramCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numSamples);
    paramCfg.accelModeArgs.fftMode.butterflyScaling = 0x0;
    paramCfg.accelModeArgs.fftMode.windowEn = HWA_FEATURE_BIT_DISABLE;
    paramCfg.accelModeArgs.fftMode.windowStart = 0U;
    paramCfg.accelModeArgs.fftMode.winSymm = HWA_FFT_WINDOW_NONSYMMETRIC;
    paramCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
    paramCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
    paramCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; /* FFT output */
    
    #if (abs(hwa_src - hwa_dst) < CSL_APP_HWA_BANK_SIZE)
        /* MEMACCESS config params warning */
        CUSTOM_WARNING("HWA Param src and dst addresses shall not be from same memory bank");
    #endif
    
    status += HWA_configParamSet(gHwaHandle, paramIdx, &paramCfg, NULL);
    if(SystemP_SUCCESS != status)
    {
        DebugP_log("Error: HWA_configParamSet failed with error: %d!!\r\n", status);
    }

    if(SystemP_SUCCESS == status)
    {
        /* Init Common Params */
        memset(&commonCfg, 0, sizeof(HWA_CommonConfig));
        commonCfg.configMask = HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                                HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                                HWA_COMMONCONFIG_MASK_NUMLOOPS |
                                HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                                HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                                HWA_COMMONCONFIG_MASK_LFSRSEED;
        
        commonCfg.paramStartIdx = 0U;
        commonCfg.paramStopIdx = 0U;
        commonCfg.numLoops = 1U;
        commonCfg.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
        commonCfg.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_DISABLE;
        commonCfg.fftConfig.lfsrSeed = 0x1234567;
        
        status = HWA_configCommon(gHwaHandle, &commonCfg);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_configCommon failed with error: %d\r\n", status);
        }
    }

    if(SystemP_SUCCESS == status)
    {
        /* Enable done interrupt */
        status = SemaphoreP_constructBinary(&gHwaDoneSem, 0);
        DebugP_assert(status == SystemP_SUCCESS);
        status = HWA_enableDoneInterrupt(gHwaHandle, HWAFFT_doneCallback, &gHwaDoneSem);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Error: HWA_enableDoneInterrupt failed with error: %d\r\n", status);
        }
    }
    
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enable HWA */
    status += HWA_enable(gHwaHandle, 1U);
    DebugP_assert(SystemP_SUCCESS == status);
}


/**
 *  @b Description
 *  @n
 *      Test implementation
 *
 *  @retval
 *      Not Applicable.
 */
void Mmwave_Test (void)
{
    MMWave_InitCfg          initCfg;
    MMWave_CtrlCfg          ctrlCfg;
    MMWave_OpenCfg          openCfg;
    MMWave_StrtCfg          strtCfg;
    int32_t                 errCode;
    MMWave_CalibrationCfg   calibrationCfg;
    int32_t                 retVal;
    MMWave_ErrorLevel       errorLevel;
    int16_t                 mmWaveErrorCode;
    int16_t                 subsysErrorCode;
    int32_t status;
    TaskP_Params gMmwCtrlTaskParams; /* this need not be global variable */
    TaskP_Object gMmwCtrlTask;

    /* Initialize the configuration: */
    memset ((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Initialize and setup the mmWave Control module */
    gMMWaveHandle = MMWave_init (&initCfg, &errCode);
    if (gMMWaveHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        DebugP_logError ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Log into the MCPI Test Logger: */
        MCPI_setFeatureTestResult ("MMWave MSS Initialization", MCPI_TestResult_FAIL);
        return;
    }
    
    DebugP_logInfo ("MMWave MSS Initialization\n");

    /* Configuring Number of Tx Antennas as 2 and Rx Antennas as 3 */
    initCfg.fecRFPwrCfgCmd.h_RxChCtrlBitMask = 0x7U;
    initCfg.fecRFPwrCfgCmd.h_TxChCtrlBitMask = 0x3U;
    /* The RDIF IP block clock shall be enabled before configuring and enabling the RDIF.*/
    initCfg.fecRFPwrCfgCmd.c_MiscCtrl = 1U << M_RL_RF_MISC_CTRL_RDIF_CLK;
    /* Configure the FECSS RF block power ON/OFF */
    retVal = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &initCfg.fecRFPwrCfgCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        /* Log into the MCPI Test Logger: */
        MCPI_setFeatureTestResult ("MMWave RF POWER ON/OFF:", MCPI_TestResult_FAIL);
        return;
    }
    DebugP_logInfo ("MMWave RF POWER ON/OFF Done\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/

    TaskP_Params_init(&gMmwCtrlTaskParams);
    gMmwCtrlTaskParams.name = "MMWAVE CTRL TASK";
    gMmwCtrlTaskParams.stackSize = APP_CTRL_TASK_STACK_SIZE;
    gMmwCtrlTaskParams.stack = gMmwCtrlTskStack;
    gMmwCtrlTaskParams.priority = APP_CTRL_TASK_PRI;
    gMmwCtrlTaskParams.args = NULL;
    gMmwCtrlTaskParams.taskMain = Mmwave_ctrlTask;

    status = TaskP_construct(&gMmwCtrlTask, &gMmwCtrlTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    Mmwave_populateDefaultOpenCfg (&openCfg);
    Mmwave_populateDefaultChirpControlCfg (&ctrlCfg); /* regular frame config */
    
    /************************************************************************
     * Open the mmWave:
     ************************************************************************/
    if (MMWave_open (gMMWaveHandle, &openCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        DebugP_logError ("Error: mmWave open failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Open", MCPI_TestResult_FAIL);
        return;
    }
   
    DebugP_logInfo ("MMWave MSS Open done.\n");

    /************************************************************************
     * Configure the mmWave:
     ************************************************************************/
    if (MMWave_config (gMMWaveHandle, &ctrlCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        DebugP_logError ("Error: mmWave configuration failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Configuration", MCPI_TestResult_FAIL);
        return;
    }
    
    DebugP_logInfo ("MMWave MSS Configuration done\n");

    /* Populate the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    Mmwave_populateDefaultCalibrationCfg (&calibrationCfg);

    // Configuring the Start Parameter
    strtCfg.frameTrigMode = M_RL_SENS_FRAME_SW_TRIG; // SW trigger
    strtCfg.chirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS; // Disable LoopBack
    strtCfg.frameLivMonEn = 0; //Disable all Live Monitors
    strtCfg.frameTrigTimerVal = 0;

    /************************************************************************
     * Start the mmWave:
     ************************************************************************/
    if (MMWave_start (gMMWaveHandle, &calibrationCfg,&strtCfg, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        DebugP_logError ("Error: mmWave start failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Start", MCPI_TestResult_FAIL);
        return;
    }
   
    DebugP_logInfo  ("MMWave MSS Start done\n");

    SemaphoreP_pend(&gHwaDoneSem, SystemP_WAIT_FOREVER);
    
    /* Disable HWA */
    status += HWA_enable(gHwaHandle, 0U);
    DebugP_assert(SystemP_SUCCESS == status);

    status += HWA_close(gHwaHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    DebugP_logInfo ("HWA FFT1D completed successfully.\n");

    ClockP_usleep(100 * 1000); // 100 milli seconds
    /************************************************************************
     * Stop the mmWave:
     ************************************************************************/
    retVal = MMWave_stop (gMMWaveHandle, &errCode);
    if (retVal < 0)
    {
        /* Error: Stopping the sensor failed. Decode the error code. */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Debug Message: */
        DebugP_logError ("Error Level: %s mmWave: %d Subsys: %d\n",
                       (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                       mmWaveErrorCode, subsysErrorCode);

        /* Did we fail because of an error? */
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error Level: The test has failed. */
            MCPI_setFeatureTestResult ("MMWave MSS Stop", MCPI_TestResult_FAIL);
            return;
        }
        else
        {
            /* Informational Level: The test has passed. Fall through...*/
        }
    }
    DebugP_logInfo ("MMWave MSS Stop done.\n");

    /************************************************************************
     * Close the mmWave:
     ************************************************************************/
    if (MMWave_close (gMMWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to configure the mmWave control module */
        DebugP_logError ("Error: mmWave close failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Close", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave MSS Close", MCPI_TestResult_PASS);
    DebugP_logInfo ("MMWave MSS close done.\n");
    
    /************************************************************************
     * Deinitialize the mmWave module:
     ************************************************************************/
    if (MMWave_deinit(gMMWaveHandle, &errCode) < 0)
    {
        /* Error: Unable to deinitialize the mmWave control module */
        DebugP_logError ("Error: mmWave Deinitialization failed [Error code %d]\n", errCode);
        MCPI_setFeatureTestResult ("MMWave MSS Deinitialized", MCPI_TestResult_FAIL);
        return;
    }
    MCPI_setFeatureTestResult ("MMWave MSS Deinitialized", MCPI_TestResult_PASS);
    return;

}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void hwa_fft1d(void* args)
{
    /* Debug Message: */
    DebugP_logInfo ("******************************************\n");
    DebugP_logInfo ("Debug: Launching mmwave\n");
    DebugP_logInfo ("******************************************\n");

    Drivers_open();
    Board_driversOpen();
    MCPI_Initialize();

    Mmwave_HwaConfig();
    
    Mmwave_Test();

    Board_driversClose();
    Drivers_close();

    DebugP_logInfo ("--- Test Completed ---\n");
}

/*
 * ISR Callback
 */
static void HWAFFT_doneCallback(void *arg)
{
    SemaphoreP_Object *doneSem;
    
    if(arg != NULL)
    {
        doneSem = (SemaphoreP_Object *)arg;
        SemaphoreP_post(doneSem);
    }

    return;
}
