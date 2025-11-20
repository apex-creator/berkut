/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HwiP.h>
#include <utils/mathutils/mathutils.h>
#include <drivers/hw_include/hw_types.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "drivers/power.h"
#include "drivers/power_xwrLx4xx.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/prcm.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>

//*****************************************************************************
// Defines
//*****************************************************************************

#define POWER_TASK_PRI  (2u)

#define POWER_TASK_SIZE (1024u)

#define SLEEP_TIME_TICKS (50000)    // Sleep For 50 Seconds
#define IDLE_TIME_TICKS (25000) // Idle mode for 25 seconds
StackType_t gPowerTaskStack[POWER_TASK_SIZE] __attribute__((aligned(32)));

StaticTask_t gPowerTaskObj;
TaskHandle_t gPowerTask;
StaticSemaphore_t gPowerSemObj;
SemaphoreHandle_t gPowerSem;
volatile int sel;


#define LPDS_USECASE      1U
#define IDLE_USECASE      2U

char bootRst[6][15] = {"PORZ", "Warm Reset", "Deep Sleep", "Soft Reset", "STC_WARM", "STC_PORZ"};
                     
char rdrWkp[6][15] = {"SLEEP COUNTER", "UART", "SPI", "GPIO/SYNCIN", "RTC CTR", "FRC INTERRUPT"};

extern Power_ModuleState Power_module;

//*****************************************************************************
// Typedefs
//*****************************************************************************


//*****************************************************************************
// Function prototypes
//*****************************************************************************
void Pinmux_init();
void TimerP_init();
void PowerClock_init(void);
void Drivers_uartInit(void);
void EDMA_init(void);
void setUseCase(void);
void power_main(void *args);
//*****************************************************************************
// Local Functions
//*****************************************************************************

void setUseCase(void)
{

}

void power_main(void *args)
{
    int bootInfoReg0, temp;
    uint32_t LPDSThtemp;
    Power_SleepState st;
    bootInfoReg0 = HW_RD_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_BOOT_INFO_REG0);
    temp = bootInfoReg0 & 0xF;
    DebugP_log("Reason for Reset is %s \r\n\n",bootRst[temp - 1]);

    while(1)
    {
        DebugP_log("\n*** Power Management Options : *** \n\r");
        DebugP_log("    1) for LPDS.  \n\r");
        DebugP_log("    2) for Idle Mode.  \n\r");
        DebugP_log("Please select Power Management mode :  ");
        DebugP_scanf("%d",&sel);
        DebugP_log("\n\r");

        if (LPDS_USECASE == sel)
        {
            DebugP_log("Entering LPDS.... \n\r");
            Power_enablePolicy();
            xSemaphoreTake(gPowerSem, SLEEP_TIME_TICKS);
            Power_disablePolicy();
            st = Power_getLowPowModeTaken();
            if(st == POWER_LPDS)
            {
                DebugP_log("\rExited LPDS!!\n\r");
                bootInfoReg0 = HW_RD_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_BOOT_INFO_REG0);
                temp = bootInfoReg0 & 0xF;
                DebugP_log("Reason for Reset is %s \r\n",bootRst[temp - 1]);
                if (temp == 0x3)
                {
                    temp = (bootInfoReg0 & 0xFC00) >> 10;
                    temp =  mathUtils_floorLog2(temp);
                    DebugP_log("Reason for Deep Sleep Wakeup is %s \r\n", rdrWkp[temp]);
                }
            }
            else if(st == POWER_IDLE)
            {
                LPDSThtemp = Power_getThresholds(POWER_LPDS);
                DebugP_log("\rIdle State is taken as the LPDS Threshold is %d ms but avaialble idle time is %d ms!!\n\r",(LPDSThtemp/1000),SLEEP_TIME_TICKS);
            }
        }
        /* On selecting Idle, the board will be kept under Idle3 state */
        /* Current Power Driver Implementation uses RTI to come out of Idle3 State */
        else if (IDLE_USECASE == sel)
        {
            DebugP_log("Entering IDLE....\n");
            Power_enablePolicy();
            xSemaphoreTake(gPowerSem, IDLE_TIME_TICKS);
            Power_disablePolicy();
            DebugP_log("\n\rExited IDLE!! \n\r");
        }
    }
}

void power_modes_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* first create the semaphores */
    gPowerSem = xSemaphoreCreateBinaryStatic(&gPowerSemObj);
    configASSERT(gPowerSem != NULL);
    vQueueAddToRegistry(gPowerSem, "Power Sem"); /* This makes the semaphore visible in ROV within CCS IDE */

    /* then create the tasks, order of task creation does not matter for this example */
    gPowerTask = xTaskCreateStatic( power_main,      /* Pointer to the function that implements the task. */
                                  "power",          /* Text name for the task.  This is to facilitate debugging only. */
                                  POWER_TASK_SIZE,  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  NULL,            /* We are not using the task parameter. */
                                  POWER_TASK_PRI,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  gPowerTaskStack,  /* pointer to stack base */
                                  &gPowerTaskObj ); /* pointer to statically allocated task object memory */
    configASSERT(gPongTask != NULL);

    // Board_driversClose();
    /* Dont close drivers to keep the UART driver open for console */
    // Drivers_close();
}

void power_LPDSresumehook(void)
{
    /* init debug log zones early */
    /* Debug log init */
    DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
    DebugP_logZoneEnable(DebugP_LOG_ZONE_WARN);
    /* UART console to use for reading input */
    DebugP_uartSetDrvIndex(CONFIG_UART0);
    TimerP_init();
    PowerClock_init();
    Pinmux_init();
    MCSPI_init();
    Drivers_uartInit();
    EDMA_init();
    Drivers_open();
    Board_driversOpen();
    xSemaphoreGive(gPowerSem);
}

int power_idleresumehook(uint_fast16_t eventType, uintptr_t eventArg, uintptr_t clientArg)
{
    xSemaphoreGive(gPowerSem);
    return Power_NOTIFYDONE;
}

/**
*  @b Description
*  @n
*      This function has the sequence to ungate the peripherals after idle3 exit
*      User can configure this sequence as per their power requirements
*
*/
void power_idle3resumehook()
{
     // Ungate UART 1
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), \
                              APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1, PRCM_GATE_CLK_ENABLE);
            PRCMPeripheralClkEnable(Power_module.dbRecords[7], PRCM_GATE_CLK_ENABLE);
            
    // Ungate UART 0
            PRCMPeripheralClkEnable(Power_module.dbRecords[6], PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), \
                              APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0, PRCM_GATE_CLK_ENABLE);

    // Power On the HWA after coming out of Idle, keeping the mode to Manual
            CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0x1);

            // Ensure HWA is On by checking the status register
            volatile int32_t hwaStat = 0;
            while(hwaStat == 0)
            {
                hwaStat = ((HW_RD_REG32((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_HWA_PD_EN)) & CSL_TOP_PRCM_PSCON_HWA_PD_EN_PSCON_HWA_PD_EN_HWA_PD_POWER_STATUS_MASK) >> CSL_TOP_PRCM_PSCON_HWA_PD_EN_PSCON_HWA_PD_EN_HWA_PD_POWER_STATUS_SHIFT);
            }
            
            // Ungate HWASS
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE2), APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS, PRCM_GATE_CLK_ENABLE);

            // Ungate I2C clock
            PRCMPeripheralClkEnable(Power_module.dbRecords[8], PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C, PRCM_GATE_CLK_ENABLE);

            // Ungate CRC, DCC, TPTC_A1, APP_PWM
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1, PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM, PRCM_GATE_CLK_ENABLE);

            // Ungate QSPI clock
            PRCMPeripheralClkEnable(Power_module.dbRecords[2], PRCM_GATE_CLK_ENABLE);
            CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI, PRCM_GATE_CLK_ENABLE);
            // Enable APP CLK and HWA CLK for both 128 KB shared RAM
            HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE ), 0xF);
}

/**
*  @b Description
*  @n
*      This function is user configurable hook before entering idle 3 state
*      User can configure this sequence as per their power requirements
*
*/
void power_idle3entryhook()
{

    // Gate CAN clock
        PRCMPeripheralClkEnable(Power_module.dbRecords[0], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN, PRCM_GATE_CLK_DISABLE);
    
    // Gate QSPI clock
        PRCMPeripheralClkEnable(Power_module.dbRecords[2], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI, PRCM_GATE_CLK_DISABLE);

        // Gate I2C clock
        PRCMPeripheralClkEnable(Power_module.dbRecords[8], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C, PRCM_GATE_CLK_DISABLE);

        // Disable PLL DIG
        CSL_REG32_FINS((CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN), PLLDIG_CTRL_PLLDIG_EN_PLLDIG_EN_CFG_PLLDIG_EN, 0x000);

        // SPI1 Smart enable and auto enable
        CSL_REG32_FINS((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_SPI1_SMART_IDLE), APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_ENABLE, 0x1);
        CSL_REG32_FINS((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_SPI1_SMART_IDLE), APP_CTRL_SPI1_SMART_IDLE_SPI1_SMART_IDLE_AUTO_EN, 0x1);

        // Gate SPI1 and SPI0
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0, PRCM_GATE_CLK_DISABLE);

        // Gate HWASS
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE2), APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS, PRCM_GATE_CLK_DISABLE);

        // Gate UART 0
        PRCMPeripheralClkEnable(Power_module.dbRecords[6], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0, PRCM_GATE_CLK_DISABLE);

        // Gate UART 1
        PRCMPeripheralClkEnable(Power_module.dbRecords[7], PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1, PRCM_GATE_CLK_DISABLE);

        // Gate CRC, DCC, TCTC_A1, APP_PWM
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_DCC, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE0), APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1, PRCM_GATE_CLK_DISABLE);
        CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_IPCFGCLKGATE1), APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM, PRCM_GATE_CLK_DISABLE);

        // Disable APP CLK and HWA CLK for both 128 KB shared RAM
        HW_WR_REG32((CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE ), 0x0);

        // Power State of the HWA domain is made powered down
        CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0x0);
        CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_HWA_PWR_REQ_PARAM ), TOP_PRCM_HWA_PWR_REQ_PARAM_HWA_PWR_REQ_PARAM_MODE, 0x0);

}
