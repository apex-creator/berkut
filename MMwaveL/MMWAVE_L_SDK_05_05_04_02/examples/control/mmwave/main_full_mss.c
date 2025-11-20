/*
 *   @file  main_full_mss.c
 *
 *   @brief
 *      Unit Test code for the mmWave 
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020-2021 Texas Instruments, Inc.
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
#define DebugP_LOG_ENABLED 1

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/* mmWave PDK Include Files: */
#include <kernel/dpl/ClockP.h>
#include <ti_drivers_config.h>
#include <ti_board_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/SystemP.h>

/* mmWave SDK Include Files: */
#include <control/mmwave/mmwave.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>
#include <utils/testlogger/logger.h>

/* FreeRTOS Task declarations. */
#define APP_TASK_PRI         (5U)
#define APP_CTRL_TASK_PRI    (7U)
#define APP_TASK_STACK_SIZE  (8*1024U)
#define APP_CTRL_TASK_STACK_SIZE (6*1024U)

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/

/* Task Stack variables.*/
uint8_t gMmwCtrlTskStack[APP_CTRL_TASK_STACK_SIZE] __attribute__((aligned(32)));

/* Global Variable: This is the handle to the mmWave module */
MMWave_Handle   gMMWaveHandle;

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
void mmwave_example_main(void* args)
{
    /* Debug Message: */
    DebugP_logInfo ("******************************************\n");
    DebugP_logInfo ("Debug: Launching mmwave\n");
    DebugP_logInfo ("******************************************\n");

    Drivers_open();
    Board_driversOpen();

    MCPI_Initialize();

    Mmwave_Test();

    Board_driversClose();
    Drivers_close();

    DebugP_logInfo ("--- Test Completed ---\n");
}
