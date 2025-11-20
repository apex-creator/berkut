/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>
#include <drivers/cslr_fec_ctrl.h>


/* define the unlock and lock values */
#define KICK_LOCK_VAL                           (0x00000000U)
#define IOMUX_KICK0_UNLOCK_VAL                  (0x83E70B13U)
#define IOMUX_KICK1_UNLOCK_VAL                  (0x95A4F1E0U)

#define APLL_CTRL_CLK_MDLL_REG1_OFFSET          (0x44U)
#define CSL_PRCM_RFANA_TOP_ISO_CTRL             (0x51CU)

/* Warm reset active low pulse width value. */
#define WARM_RSTN_PULSE_WIDTH                   (0x10U)        

const char *SOC_getCoreName(uint16_t coreId)
{
    static const char *coreIdNames[CSL_CORE_ID_MAX + 1U] =
    {
        "m4f0-0",
        "unknown"
    };
    const char *name;

    if(coreId < CSL_CORE_ID_MAX)
    {
        name = coreIdNames[coreId];
    }
    else
    {
        name = coreIdNames[CSL_CORE_ID_MAX];
    }
    return name;
}

int32_t SOC_clocksEnable(void)
{
    int32_t status = SystemP_SUCCESS;

    SOC_rcmEnableADPLLClock();
    // Run M4 with Fast Clock
    SOC_rcmSetM4ClockSrc(SOC_rcmM4ClockSrc_FAST_CLK);

    return status;
}

int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable)
{
    int32_t status = SystemP_SUCCESS;

    SOC_rcmEnablePeripheralClock(moduleId, enable);

    return status;
}

int32_t SOC_moduleSetClockFrequency(SOC_RcmPeripheralId moduleId, SOC_RcmPeripheralClockSource clkId, uint64_t clkRate)
{
    return (SOC_rcmSetPeripheralClock( moduleId, clkId, clkRate));
}

uint64_t SOC_getSelfCpuClk(void)
{
    uint64_t cpuClockRate = SOC_rcmGetM4Clock();

    return cpuClockRate;
}

void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_TOP_IO_MUX == domainId)
    {
        baseAddr = CSL_TOP_IO_MUX_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_IO_MUX_IOCFGKICK1);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);/* KICK 1 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_IO_MUX_IOCFGKICK0);
        CSL_REG32_WR(kickAddr, KICK_LOCK_VAL);/* KICK 0 */
    }

    return;
}

void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    if(SOC_DOMAIN_ID_TOP_IO_MUX == domainId)
    {
        baseAddr = CSL_TOP_IO_MUX_U_BASE;
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_IO_MUX_IOCFGKICK0);
        CSL_REG32_WR(kickAddr, IOMUX_KICK0_UNLOCK_VAL); /* KICK 0 */
        kickAddr = (volatile uint32_t *) (baseAddr + CSL_TOP_IO_MUX_IOCFGKICK1);
        CSL_REG32_WR(kickAddr, IOMUX_KICK1_UNLOCK_VAL); /* KICK 1 */
    }

    return;
}

void SOC_logAllClockHz()
{
    uint32_t clkHz;

    clkHz = SOC_rcmGetM4Clock();
    DebugP_log("M4F         = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_MCAN);
    DebugP_log("APPSS_MCAN   = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_LIN);
    DebugP_log("APPSS_LIN     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_QSPI);
    DebugP_log("APPSS_QSPI    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_RTI);
    DebugP_log("APPSS_RTIA    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_WDT);
    DebugP_log("APPSS_WDT     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_MCSPIA);
    DebugP_log("APPSS_SPI0    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_MCSPIB);
    DebugP_log("APPSS_SPI1    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_I2C);
    DebugP_log("APPSS_I2C     = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_UART0);
    DebugP_log("APPSS_UART0    = %9d Hz\r\n", clkHz);
    clkHz = SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId_APPSS_UART1);
    DebugP_log("APPSS_UART1    = %9d Hz\r\n", clkHz);
}

void SOC_memoryInit(uint16_t flag )
{   

   
    /* HWASS Shared RAM Memory Init. */
    SOC_rcmStartInitSharedRam(flag);

    /* TPCC Memory Init. */
    SOC_rcmStartMemInitTpcc(flag);

    /* Wait for HWASS Shared RAM Memory Init. */
    SOC_rcmWaitMemInitSharedRam(flag);

    /* Wait for TPCC Memory Init. */
    SOC_rcmWaitMemInitTpcc(flag);

    return;
}

void SOC_enableMDLLClock(void)
{
    uint32_t loop = 0;

    CSL_REG32_WR( (volatile uint32_t *)(CSL_APLL_CTRL_U_BASE + APLL_CTRL_CLK_MDLL_REG1_OFFSET), 0x88A02A08U);
    for(loop =0; loop < 100U; loop++){;}
    CSL_REG32_WR( (volatile uint32_t *)(CSL_APLL_CTRL_U_BASE + APLL_CTRL_CLK_MDLL_REG1_OFFSET), 0x88A02A09U);
    for(loop =0; loop < 100U; loop++){;}
    CSL_REG32_WR( (volatile uint32_t *)(CSL_APLL_CTRL_U_BASE + APLL_CTRL_CLK_MDLL_REG1_OFFSET), 0x88A0AA09U);
    for(loop =0; loop < 100U; loop++){;}
    CSL_REG32_WR( (volatile uint32_t *)(CSL_APLL_CTRL_U_BASE + APLL_CTRL_CLK_MDLL_REG1_OFFSET), 0x88A08A09U);
    for(loop =0; loop < 100U; loop++){;}
    CSL_REG32_WR( (volatile uint32_t *)(CSL_TOP_PRCM_U_BASE + CSL_PRCM_RFANA_TOP_ISO_CTRL), 0xFFFFE1FFU);

    return;
}

uint64_t SOC_virtToPhy(void *virtAddr)
{
    uintptr_t   temp = (uintptr_t) virtAddr;
    uint64_t    phyAddr = (uint64_t) temp;                  /* Default case */

    /* APP_CPU_RAM */
    if((temp >= CSL_APP_CPU_RAM_U_BASE) &&
        (temp < (CSL_APP_CPU_RAM_U_BASE + CSL_APP_CPU_RAM_SIZE)))
    {
        phyAddr |= CSL_APP_RAM_U_BASE;
    }


    /* APP_CPU_SHARED_RAM */
    if((temp >= CSL_APP_CPU_SHARED_RAM_U_BASE) &&
        (temp < (CSL_APP_CPU_SHARED_RAM_U_BASE + CSL_APP_CPU_SHARED_RAM_U_SIZE)))
    {
        phyAddr |= CSL_APP_SHARED_RAM_U_BASE;
    }

    return (phyAddr);
}

SOC_SysRstReason SOC_getSysRstReason(void)
{
    SOC_SysRstReason resetCause = 0U;
    CSL_app_ctrlRegs *appctrl = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;

    resetCause = CSL_FEXTR(appctrl->APPSS_BOOT_INFO_REG0,7U, 4U);

    return resetCause;
}

SOC_RstReason SOC_getRstReason(void)
{
    SOC_RstReason resetCause = 0U;
    CSL_app_ctrlRegs *appctrl = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;

    resetCause = CSL_FEXTR(appctrl->APPSS_BOOT_INFO_REG0, 3U, 0U);

    return resetCause;
}

void SOC_triggerWarmReset(void)/*GAP COMMENT ID:81 START*/
{
    CSL_top_prcmRegs *topPrcm = (CSL_top_prcmRegs*)CSL_TOP_PRCM_U_BASE;
    
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_TOP_PRCM, 0U); 

    CSL_FINS(topPrcm->RST_SOFT_RESET, TOP_PRCM_RST_SOFT_RESET_RST_SOFT_RESET_WARM_RSTN_PULSE_WIDTH, WARM_RSTN_PULSE_WIDTH);
    CSL_FINS(topPrcm->RST_SOFT_RESET, TOP_PRCM_RST_SOFT_RESET_RST_SOFT_RESET_WARM_RESET_REQN, 0x1U);
    
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_TOP_PRCM, 0U);    
}/*GAP COMMENT ID:81 END*/

void SOC_triggerSoftReset(void)/*GAP COMMENT ID:82 START*/
{
    CSL_app_rcmRegs *appRcm = (CSL_app_rcmRegs*)CSL_APP_RCM_U_BASE;
    
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_APP_RCM, 0U); 

    CSL_FINS(appRcm->RST_FSM_TRIG, APP_RCM_RST_FSM_TRIG_RST_FSM_TRIG_CPU, 0x07U);
    
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_APP_RCM, 0U);   
}/*GAP COMMENT ID:82 END*/
void SOC_setEpwmTbClk(uint32_t enable)
{
    CSL_app_rcmRegs *ptrAPPRCMRegs = (CSL_app_rcmRegs *)CSL_APP_RCM_U_BASE;

    if(enable != 0U)
    {
        CSL_REG32_FINS(&(ptrAPPRCMRegs->IPCFGCLKGATE1), 
                       APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM,
                       0x0U);
    }
    else
    {
        CSL_REG32_FINS(&(ptrAPPRCMRegs->IPCFGCLKGATE1), 
                       APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM,
                       0x7U);
    }
    
    return;
}

