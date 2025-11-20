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

#ifndef Bool
typedef unsigned short		Bool;		/* boolean */
#endif
/* Define TRUE/FALSE to go with Bool */
#ifndef TRUE
#define TRUE		((Bool) 1)
#define FALSE		((Bool) 0)
#endif
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/*Drivers Include Files. */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "drivers/power.h"
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ripple_pwr.h"
#include "./ripple.h" 
#include "./ripple_link/ripple_link.h"
#include <drivers/hw_include/cslr.h>
#include "drivers/power_xwrLx4xx.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
extern TaskHandle_t gCliTask;
extern TaskHandle_t gRawADCTask;
extern uint32_t gnumberofchirpstosend;
volatile uint32_t check=1;
uint8_t pgVersion;
volatile unsigned long long f,demoEndTime, slpTimeus,lpdsLatency;
extern volatile unsigned long long demoStartTime;
/* Power task semaphore objects*/
extern StaticSemaphore_t gPowerSemObj;
extern SemaphoreHandle_t gPowerSem;
extern Power_ModuleState Power_module;

double demoTimeus,frmPrdus;
volatile int mmwReinitStatus = 0;
extern MmwDemo_MSS_MCB gMmwMssMCB;
extern int32_t CLI_MMWStart(void);
extern StackType_t gADCTaskStack[MAIN_TASK_SIZE] __attribute__((aligned(32)));
extern StaticTask_t gADCTaskObj;
extern TaskHandle_t gRawADCTask;
/* Re-init Function Declarations*/
void PowerClock_init();
void Pinmux_init();
void QSPI_init();
void EDMA_init();
void Drivers_uartInit();
int power_idleresumehook(uint_fast16_t eventType, uintptr_t eventArg, uintptr_t clientArg);

/**
*  @b Description
*  @n
*      This function has the sequence to ungate the peripherals after idle3 exit
*
*/
void power_idle3resumehook()
{
     // Anything to do after idle3 exit
}

/**
*  @b Description
*  @n
*      This function has the sequence to gate the peripherals after idle3 exit
*
*/
void power_idle3entryhook()
{
     // Anything to do before idle3 entry
}

void power_LPDSentryhook(void)
{
    /*Anything to do before LPDS entry*/
}
/*Idle3 Handle; Currently Unused*/
int power_idleresumehook(uint_fast16_t eventType, uintptr_t eventArg, uintptr_t clientArg)
{
    while(1);
}
void power_LPDSresumehook(void)
{
    /* Re-Init MPU*/
    MpuP_init();
    
    /* Debug log init*/
    DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
    DebugP_logZoneEnable(DebugP_LOG_ZONE_WARN);

    /* UART console to use for reading input */
    DebugP_uartSetDrvIndex(CONFIG_UART0);

    /* Initialize Clock*/
    ClockP_init();

    PowerClock_init();

    /* Now we can do pinmux*/
    Pinmux_init();

    /*Re-initialize all peripheral drivers*/
    QSPI_init();
    EDMA_init();
    MCSPI_init();
    Power_init();
    Drivers_uartInit();
    Drivers_open();
    Board_driversOpen();

    gnumberofchirpstosend=gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
  
}

#ifdef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
/* ToDo This is for debugging, set to one to stop the code after power up, and reconnect CCS*/
volatile int gDebugLowPowerModeBreakFlag = 0;
#endif

/*RPMF: Power Management Task*/
void powerManagementTask(void *args)
{
    while(1)
    {
        
        /* Wait till the UART transmit is complete*/
        xSemaphoreTake(gPowerSem, portMAX_DELAY);
        if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE))
        {

            frmPrdus = (gMmwMssMCB.frameCfg.w_FramePeriodicity * 0.025);
            lpdsLatency = Power_getTransitionLatency(POWER_LPDS, Power_TOTAL_LATENCY);
            if(frmPrdus > (demoTimeus + lpdsLatency))
            {

                if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
                {
                    vTaskDelete(gRawADCTask);
                    /* Delete the CLI tasks*/
                    if (gCliTask != NULL)
                    {
                        vTaskDelete(gCliTask);
                        gCliTask = NULL;
                    }
                    /* Capture the time elaspsed till here*/
                    demoEndTime = PRCMSlowClkCtrGet();
                    /*Demo Run time for one frame (From Sensor Start till Completion of UART transmit)*/
                    demoTimeus = (demoEndTime - demoStartTime)*(30.5);

                    if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
                    {
                    frmPrdus = (gMmwMssMCB.frameCfg.w_FramePeriodicity * 0.025);

                    }
                    /* Time for Low Power State*/
                    slpTimeus = (unsigned long long)(frmPrdus - demoTimeus);

                    if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE)
                    {
                        /*Shutdown the FECSS after chirping
                        Retain FECSS Code Memory*/
                        int32_t err;
                        
                        // Get the Version of the Device.
                        pgVersion = SOC_getEfusePgVersion();
                        if(pgVersion==1){
                            PRCMSetSRAMRetention((PRCM_FEC_PD_SRAM_CLUSTER_2 | PRCM_FEC_PD_SRAM_CLUSTER_3), PRCM_SRAM_LPDS_RET);
                        }
                        /* MMW Closure in preparation for Low power state*/
                        MMWave_stop(gMmwMssMCB.ctrlHandle,&err);
                        MMWave_close(gMmwMssMCB.ctrlHandle,&err);
                        MMWave_deinit(gMmwMssMCB.ctrlHandle,&err);
                        /*RPMF driver call for getting to Low Power State*/
                        Power_enablePolicy();
                        Power_idleFunc(slpTimeus);
                        Power_disablePolicy();
                    }
                    if(gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE)
                    {
                        /* Wait till the frame period expires */
                        uint64_t frmPeriod = (uint32_t) (frmPrdus * 32.768e-3);
                        while ((gMmwMssMCB.frameStartTimeStampSlowClk + frmPeriod) > PRCMSlowClkCtrGet())
                        {
                        }
                        power_LPDSresumehook();
                    }

                    /* mmWave initialization as Warmstart*/
                    mmwReinitStatus = mmw_link_Initialization(1);

                    /* Start the Next frame after Low Power Exit*/

                    #ifdef LOW_POWER_DEEP_SLEEP_MODE_VERIFICATION
                    /* This loop is for debugging */
                    while(gDebugLowPowerModeBreakFlag)
                    {
                    }
                    if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE))
                    {
                        gDebugLowPowerModeBreakFlag = 1;
                    }
                    #endif
                    gRawADCTask = xTaskCreateStatic( ripple_transferrawADC,
                                "ripple_transferrawADC",
                                10*1024,
                                NULL,
                                RAWADCTASK_PRIORITY,
                                gADCTaskStack,
                                &gADCTaskObj);
                    configASSERT(gRawADCTask != NULL);
                    CLI_MMWStart();
                
                    
                }
            }
            else
            {
                /* Use Low Power config only when there is sufficient time for Deep Sleep.*/
                DebugP_log("Error: No Sufficient Time for getting into Low Power Modes.\n"); 
                DebugP_assert(0);
            }
        }
        
    }
}