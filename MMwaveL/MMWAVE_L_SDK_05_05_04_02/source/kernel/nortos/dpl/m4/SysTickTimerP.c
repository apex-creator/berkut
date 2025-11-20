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
#include "SysTickTimerP.h"
#include <drivers/hw_include/cslr.h>

/* SysTick timer implementation for clock tick */
#define SYST_BASE   (0xE000E010U)
#define SYST_CSR    (volatile uint32_t *)((SYST_BASE) + 0x00U)
#define SYST_RVR    (volatile uint32_t *)((SYST_BASE) + 0x04U)
#define SYST_CVR    (volatile uint32_t *)((SYST_BASE) + 0x08U)

/**
 *  Design: MMWLPSDK-728
 */
void SysTickTimerP_Params_init(TimerP_Params *params)
{
    params->inputPreScaler = 1U; /* NOT USED */
    params->inputClkHz = 200U*1000000U;
    params->periodInUsec = 1000U;
    params->oneshotMode = 0U;
    params->enableOverflowInt = 1U;
}

/**
 *  Design: MMWLPSDK-729
 */
void SysTickTimerP_setup(TimerP_Params *params)
{
    volatile uint32_t *addr;
    uint32_t ctrlVal;
    uint32_t countVal, reloadVal;

    /* There is no pre-scaler support for SysTick and its value is ignored */
    DebugP_assert( params->inputClkHz != 0U);
    DebugP_assert( params->periodInUsec != 0U);
    /* usec period MUST divide 1sec in integer units */
    DebugP_assert( (1000000U % params->periodInUsec) == 0U );

    /* stop timer and clear pending interrupts */
    SysTickTimerP_stop();

    /* calculate count and reload value register value */
    countVal = ((uint32_t)
                        (((uint64_t)params->inputClkHz * params->periodInUsec)
                    / ((uint64_t)1000000U))
                ) - 1UL;
    /* keep reload value as 0, later if is auto-reload is enabled, it will be set a value > 0 */
    reloadVal = 0U;

    /* calculate control register value, keep timer disabled */
    ctrlVal = 0U;
    /* select clock source as CPU clock */
    ctrlVal |= (1U << 2);
    /* enable/disable interrupts */
    if(params->enableOverflowInt != 0U)
    {
        /* enable interrupt */
        ctrlVal |= (1U << 1);
    }

    if(params->oneshotMode==0U)
    {
        /* autoreload timer */
        reloadVal = countVal;
    }

    /* set timer control value */
    addr = SYST_CSR;
    *addr = ctrlVal;

    /* set reload value */
    addr = SYST_RVR;
    *addr = reloadVal;

    /* set count value */
    addr = SYST_CVR;
    *addr = countVal;

}

/**
 *  Design: MMWLPSDK-731
 */
/* base address not used since, address is fixed for SysTick in M4F */
void SysTickTimerP_start()
{
    volatile uint32_t *addr = SYST_CSR;

    /* start timer */
    *addr |= (0x1U << 0);
}

/**
 *  Design: MMWLPSDK-732
 */
/* base address not used since, address is fixed for SysTick in M4F */
void SysTickTimerP_stop()
{
    volatile uint32_t *addr = SYST_CSR;

    /* stop timer */
    *addr &= ~(0x1U << 0);
}

/**
 *  Design: MMWLPSDK-733
 */
/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_getCount()
{
    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return (0xFFFFFFFFU - CSL_REG32_RD(SYST_CVR));

}

/**
 *  Design: MMWLPSDK-734
 */
/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_getReloadCount()
{
    /* return 0xFFFFFFFF - value, since ClockP assumes in this format to calculate current time */
    return (0xFFFFFFFFU - CSL_REG32_RD(SYST_RVR));
}

/**
 *  Design: MMWLPSDK-735
 */
/* base address not used since, address is fixed for SysTick in M4F */
uint32_t SysTickTimerP_isOverflowed()
{
    volatile uint32_t *addr = SYST_CSR;

    return ((*addr >> 16) & 0x1U);
}