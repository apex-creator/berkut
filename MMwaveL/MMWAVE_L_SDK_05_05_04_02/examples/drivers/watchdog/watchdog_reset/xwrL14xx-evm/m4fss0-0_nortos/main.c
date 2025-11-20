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

#include <stdlib.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/prcm.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"

extern uint32_t __LOAD_DATA_START;
extern uint32_t __LOAD_DATA_END;
extern uint32_t __RUN_DATA_START;

void watchdog_reset_mode_main(void *args);

volatile unsigned long long test;
/* Sequence to force RBL to reload from flash and handle frontend if it is being used in application */
void handleWdtRst()
{
    volatile uint32_t fecPdstatus = 0;
    /* Halt the FECSS core */
    HW_WR_REG32(0x5a040018,0x207ff);

    /* Set to Manual Mode in FEC_PWR_REQ_PARAM register */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_MODE, 0);

    /* Do not retain any FECSS RAM */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_STATE ), TOP_PRCM_PSCON_FEC_PD_RAM_STATE_PSCON_FEC_PD_RAM_STATE_FEC_PD_MEM_SLEEP_STATE,0);
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE ), TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE_PSCON_FEC_PD_RAM_GRP4_STATE_FEC_PD_MEM_GRP4_SLEEP_STATE,0);

    /* Switch OFF the Front end */
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 0);
    do
    {
        fecPdstatus = ((HW_RD_REG32(CSL_TOP_PRCM_U_BASE+CSL_TOP_PRCM_PSCON_FEC_PD_EN) & 0x200) >> 9);
    }while(fecPdstatus == 0);

    /* Switch ON the Front end*/
    CSL_REG32_FINS((CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_FEC_PWR_REQ_PARAM ), TOP_PRCM_FEC_PWR_REQ_PARAM_FEC_PWR_REQ_PARAM_WAKEUP_OUT_STATE, 1);
    do
    {
        fecPdstatus = ((HW_RD_REG32(CSL_TOP_PRCM_U_BASE+CSL_TOP_PRCM_PSCON_FEC_PD_EN) & 0x200) >> 9);
    }while(fecPdstatus == 1);

    /* Enable Clock for FECSS */
    HW_WR_REG32(0x56060394,0);

    /* Wait for Clock enable. The following loop is to give us the necessary delay */
    for(int i =0;i<100;i++)
            {
                test = PRCMSlowClkCtrGet();
            }

    /*AON Registers fields are used by bootloader in warm reset*/
    uint32_t *pc_reg1 = (uint32_t *)0x5A040054;
    uint32_t *pc_reg2 = (uint32_t *)0x5A040058;
    /*setting pc parity, data integrity check parity,image length parity, boot vector parity, lbist in boot, perform integrity check to 0*/
    *pc_reg1 = *pc_reg1 & (uint32_t)0x00000000;
    /*setting boot vector to 0 to force RBL to reload from flash*/
    *pc_reg2 = *pc_reg2 & (uint32_t)0xFE000000;
    /*Issuing a controlled warm reset*/
    SOC_triggerWarmReset();
}

int main()
{
    SOC_RstReason rstrsn;
    rstrsn = SOC_getRstReason();
    if (rstrsn == SOC_RESET_REASON_WARM)
    {
        /* If user wants to reload complete image from flash during a watchdog reset following
        sequence can be followed. After exiting watchdog reset RBL is made to reload from flash
        by issuing a controlled warm reset with boot vector set to 0. Also when frontend is being
        used by the application, we have to ensure that frontend PD is ON to enable RBL load 
        frontend firmware.      
        */
        handleWdtRst();
    }
    
    /*if user does not want to reload image from flash and when frontend is not being used following will be
    required for resuming after a watchdog reset. This is to get a fresh copy of data section. If user
    is reloading image from flash then the following is not necessary and user can have data section only at 
    run address by updating the linker.
    */

    /* Copying the data section from load memory to run memory location.*/
    uint32_t data_size = ((uintptr_t)&__LOAD_DATA_END - (uintptr_t)&__LOAD_DATA_START);
    memcpy((void*)&__RUN_DATA_START,(void*)&__LOAD_DATA_START,data_size);
    
    System_init();
    Board_init();

    watchdog_reset_mode_main(NULL);

    Board_deinit();
    System_deinit();

    return 0;
}
