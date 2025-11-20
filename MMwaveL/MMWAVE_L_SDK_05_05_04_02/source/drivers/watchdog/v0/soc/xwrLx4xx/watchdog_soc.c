/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  \file   watchdog_soc.c
 *
 *  \brief  File containing SOC related APIs to enable WDT.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/watchdog.h>
#include <drivers/hw_include/cslr_soc.h>

void Watchdog_releaseReset(Watchdog_Handle handle)
{
	CSL_app_rcmRegs* ptrAPPRCMRegs = (CSL_app_rcmRegs*)(CSL_APP_RCM_U_BASE);

    if (CSL_FEXT(ptrAPPRCMRegs->BLOCKRESET0,
                 APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD) != 0U)
    {
        CSL_FINS(ptrAPPRCMRegs->BLOCKRESET0,
                 APP_RCM_BLOCKRESET0_BLOCKRESET0_APP_WD,
                 0x0U);
    }

}

void Watchdog_configureWarmReset(Watchdog_Handle handle)
{
	CSL_top_prcmRegs* ptrTopPRCMRegs = (CSL_top_prcmRegs*)CSL_TOP_PRCM_U_BASE;

    CSL_FINS(ptrTopPRCMRegs->RST_WDT_RESET_EN,
             TOP_PRCM_RST_WDT_RESET_EN_RST_WDT_RESET_EN_WD_RESET_EN,
             0x1U);

}

void Watchdog_suspendControl(uint8_t suspendWdt)
{
    CSL_app_ctrlRegs* ptrAppssCtrlRegs = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;
    
    if(suspendWdt == 0U)
    {
        CSL_FINS(ptrAppssCtrlRegs->APPSS_DBG_ACK_CTL0,
             APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT,
             0x0U);
    }
    else
    {
        CSL_FINS(ptrAppssCtrlRegs->APPSS_DBG_ACK_CTL0,
             APP_CTRL_APPSS_DBG_ACK_CTL0_APPSS_DBG_ACK_CTL0_WDT,
             0x1U);
    }
    

}
