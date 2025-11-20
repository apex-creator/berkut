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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>

/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>

/*OS Include Files. */
#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>

/*Drivers Include Files. */
#include <drivers/edma.h>
#include <drivers/uart.h>
#include <drivers/mcspi.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "drivers/power.h"
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>

/*Local Include Files. */
#include "ripple.h"
#include "ripple_link/ripple_link.h"
#include "ripple_cli/ripple_cli.h"
#include "ripple_pwr/ripple_pwr.h"

/* ========================================================================== */
/*                           Declaring Global variables                       */
/* ========================================================================== */
MmwDemo_MSS_MCB gMmwMssMCB = {0};
StackType_t gADCTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
StaticTask_t gADCTaskObj;
TaskHandle_t gRawADCTask;
HwiP_Object gHwiChirpAvailableHwiObject;
HwiP_Object gHwirawADCChirpAvailableHwiObject;
HwiP_Object gHwiFrameStartHwiObject;
/*Counter for number of chirps*/
volatile uint32_t gChirpAvailCnt = 0;
/*! @brief   Semaphore Object to pend main task */
SemaphoreP_Object RawADCSemHandle;
extern uint32_t gnumberofchirpstosend;

/** Power task objects*/
TaskHandle_t gPowerTask;
StaticTask_t gPowerTaskObj;
StackType_t gPowerTaskStack[POWER_TASK_SIZE] __attribute__((aligned(32)));
/* Power task semaphore objects*/
StaticSemaphore_t gPowerSemObj;
SemaphoreHandle_t gPowerSem;

/* In order to debug target code (set brake points, step over,...) set this variable below to 1 in CCS
 * expression window. It will prevent ISR mmwDemoFrameStartISR from forcing the code to stop */
volatile uint32_t gDebugTargetCode = 0;

uint32_t Cycleprofiler_getTimeStamp(void)
{
    uint32_t *frameRefTimer;
    frameRefTimer = (uint32_t *) 0x5B000020;
    return *frameRefTimer;
}

static void mmwDemoFrameStartISR(void *arg)
{
    uint32_t curCycle;
    MmwDemo_MSS_MCB *mmwMssMCB = (MmwDemo_MSS_MCB *) arg;

    HwiP_clearInt(CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);
    mmwMssMCB->frameStartTimeStampSlowClk = PRCMSlowClkCtrGet();
    curCycle = Cycleprofiler_getTimeStamp();
    /* For testing */
    mmwMssMCB->stats.framePeriod_us = (curCycle - mmwMssMCB->stats.frameStartTimeStamp)/FRAME_REF_TIMER_CLOCK_MHZ;
    mmwMssMCB->stats.frameStartTimeStamp = curCycle;

    gnumberofchirpstosend=gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
    

    mmwMssMCB->stats.frameStartIntCounter++;
}

int32_t mmwDemo_registerFrameStartInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START;
    hwiPrms.callback    = mmwDemoFrameStartISR;
    hwiPrms.args        = (void *) &gMmwMssMCB;
    status              = HwiP_construct(&gHwiFrameStartHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START);
    }

    return retVal;
}
/* Handler for RAw ADC data Task*/
void ripple_transferrawADC(void *args)
{   
    while(1){
        SemaphoreP_pend(&RawADCSemHandle, SystemP_WAIT_FOREVER);
        mmw_SPIWrite(gMmwMssMCB.loggingSpiHandle,(void *)CSL_APP_HWA_ADCBUF_RD_U_BASE,gMmwMssMCB.chirpsize);
        gnumberofchirpstosend--;
        if(gnumberofchirpstosend==0){
            if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE)){
            xSemaphoreGive(gPowerSem);
            }
        }
    }
}
        /* char text;
        // uint32_t regVal;
        // regVal = HW_RD_REG32(0x56060310);
        // regVal=regVal&0xf;
        // text=regVal+'0';
        // mmw_UartWrite(gMmwMssMCB.loggingUartHandle,(void *)(&text),4);//for verifying Low power mode*/
/*Chirp Available Interrupt ISR*/
void ripple_chirpAvailInterrupCallBackFunc(uintptr_t arg)
{
    SemaphoreP_post(&RawADCSemHandle);
    gChirpAvailCnt++;
}
int32_t ripple_registerChirpAvailInterrupt(void)
{
    int32_t           retVal = 0;
    int32_t           status = SystemP_SUCCESS;
    HwiP_Params       hwiPrms;


    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = 16 + CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN; 
    hwiPrms.callback    = (void*)ripple_chirpAvailInterrupCallBackFunc;
    hwiPrms.args        = NULL;
    status              = HwiP_construct(&gHwirawADCChirpAvailableHwiObject, &hwiPrms);

    if(SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN);
    }

    return retVal;
}

void ripple(void* args)
{   
    int32_t           errorCode = SystemP_SUCCESS;

    /* Peripheral Driver Initialization */
    Drivers_open();
    Board_driversOpen();


    /*SLICER LDO TLOAD CONTROL
    Value should be 0x0 during boot sequence to ensure stability while unloaded, then 0x1 to turn off
    all current loading after oscillator is enabled to reduce power and extend reliability.*/
    
    /*HWASS_SHRD_RAM, TPCCA and TPCCB memory have to be init before use. */
    /*APPSS SHRAM0 and APPSS SHRAM1 are initialized by RBL, hence no need to re-init again*/
    /*FECSS SHRAM (96KB) has to be initialized before use as RBL does not perform initialization.*/
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_TPCCA_INIT|SOC_RCM_MEMINIT_TPCCB_INIT|SOC_RCM_MEMINIT_FECSS_SHRAM_INIT);
    
    gMmwMssMCB.loggingUartHandle = gUartHandle[0];

    /* mmWave initialization*/
    mmw_link_Initialization(0);
    /* RPMF: Create a Task for Power Management Framework*/

    gPowerTask = xTaskCreateStatic( powerManagementTask,      /* Pointer to the function that implements the task. */
                                  "power",          /* Text name for the task.  This is to facilitate debugging only. */
                                  POWER_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  POWER_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPowerTaskStack,  /* pointer to stack base */
                                  &gPowerTaskObj ); /* pointer to statically allocated task object memory */
                                  
    /*RPMF: Create Semaphore for to pend Power Task*/
    gPowerSem = xSemaphoreCreateBinaryStatic(&gPowerSemObj);

    /* Create binary semaphore to pend Main task, */
    SemaphoreP_constructBinary(&gMmwMssMCB.demoInitTaskCompleteSemHandle, 0);
    errorCode = SemaphoreP_constructBinary(&gMmwMssMCB.cliInitTaskCompleteSemHandle, 0);

    DebugP_assert(SystemP_SUCCESS == errorCode);
    SemaphoreP_constructBinary(&RawADCSemHandle,0);

    errorCode = SemaphoreP_constructBinary(&RawADCSemHandle,0);
    DebugP_assert(SystemP_SUCCESS == errorCode);

    /* Register Chirp Available Interrupt */
    if(ripple_registerChirpAvailInterrupt() != 0){
        DebugP_log("Failed to register Chirp Available interrupt\n");
        DebugP_assert(0);
    }

    /*Reset the ADC Buffer*/
    *((volatile uint32_t*)0x56080020) = 1;
    *((volatile uint32_t*)0x5608002C) = 1;

    gRawADCTask = xTaskCreateStatic( ripple_transferrawADC,
                                    "ripple_transferrawADC",
                                    10*1024,
                                    NULL,
                                    RAWADCTASK_PRIORITY,
                                    gADCTaskStack,
                                    &gADCTaskObj);
    configASSERT(gRawADCTask != NULL);

    /* Register Frame Start Interrupt */
    if(mmwDemo_registerFrameStartInterrupt() != 0){
        DebugP_log("Failed to register frame start interrupts\n");
        DebugP_assert(0);
    }


    ripple_cli_init(CLI_TASK_PRIORITY);

     /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.demoInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);

    Board_driversClose();
    Drivers_close();
}

