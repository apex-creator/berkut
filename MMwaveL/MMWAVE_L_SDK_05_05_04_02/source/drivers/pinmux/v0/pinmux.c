/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 *
 */

/**
 *  \file   pinmux.c
 *
 *  \brief  PINMUX Driver file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/pinmux.h>
#include <drivers/soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

static const PadToPin_t PadtoPinMap[8] =
{
    {PIN_PAD_AI, 0, 1},
    {PIN_PAD_AJ, 1, 1},
    {PIN_PAD_AL, 2, 0},
    {PIN_PAD_AM, 3, 1},
    {PIN_PAD_AU, 4, 1},
    {PIN_PAD_AV, 5, 0},
    {PIN_PAD_AW, 6, 1},
    {PIN_PAD_AX, 7, 1}
};
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId)
{
    uint32_t            baseAddr;
    volatile uint32_t  *regAddr;

    if(NULL != pinmuxCfg)
    {
        baseAddr = CSL_TOP_IO_MUX_U_BASE;
        SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);
        while(pinmuxCfg->offset != PINMUX_END)
        {
            regAddr = (volatile uint32_t *)(baseAddr + pinmuxCfg->offset);
            CSL_REG32_WR(regAddr, pinmuxCfg->settings);
            pinmuxCfg++;
        }
        SOC_controlModuleLockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);
    }

    return;
}

uint32_t Pinmux_Read(uint32_t pinOffset)
{
    uint32_t            baseAddr;
    volatile uint32_t  *regAddr, regVal;

    baseAddr = CSL_TOP_IO_MUX_U_BASE;
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);
    regAddr = (volatile uint32_t *)(baseAddr + pinOffset);
    regVal= CSL_REG32_RD(regAddr);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);

    return regVal;
}

uint32_t Pinmux_getMode(uint32_t pad)
{
    int i;

    for(i = 0; i< 7; i++)
    {
        if(pad ==PadtoPinMap[i].Pad)
        {
            break;
        }
    }
    return(PadtoPinMap[i].mode);
}

uint32_t Pinmux_getPinNum(uint32_t pad)
{
    int i;

    for(i = 0; i< 7; i++)
    {
        if(pad ==PadtoPinMap[i].Pad)
        {
            break;
        }
    }
    return(PadtoPinMap[i].gpioNum);
}

void Pinmux_inputoverride(uint32_t offset, bool state)
{
    uint32_t            baseAddr;
    baseAddr = CSL_TOP_IO_MUX_U_BASE;
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);
    uint32_t regValue = CSL_REG32_RD(baseAddr + offset);
    regValue &= ~(PIN_OVERRIDE_INPUT_ENABLE_CTRL| PIN_OVERRIDE_INPUT_ENABLE);
    regValue |= PIN_OVERRIDE_INPUT_ENABLE_CTRL;
    if (state){
        regValue |= PIN_OVERRIDE_INPUT_ENABLE;
    }
    CSL_REG32_WR(baseAddr + offset, regValue);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);

    return;
}

void Pinmux_outputoverride(uint32_t offset, bool state)
{
    uint32_t            baseAddr;
    baseAddr = CSL_TOP_IO_MUX_U_BASE;
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);
    volatile uint32_t regValue = CSL_REG32_RD(baseAddr + offset);
    regValue &= ~(PIN_OVERRIDE_OUTPUT_DISABLE_CTRL| PIN_OVERRIDE_OUTPUT_DISABLE);
    regValue |= PIN_OVERRIDE_OUTPUT_DISABLE_CTRL;
    if (state){
        regValue |= PIN_OVERRIDE_OUTPUT_DISABLE;
    }
    CSL_REG32_WR(baseAddr + offset, regValue);
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_TOP_IO_MUX, 0U);

    return;
}
