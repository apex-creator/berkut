/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef HWA_FFT_H_
#define HWA_FFT_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hwa.h>
#include <drivers/edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* One each for FFT and IFFT operations */
#define HWAFFT_RES_IDX_FFT              (0U) //index for FFT or Compression
#define HWAFFT_RES_IDX_IFFT             (1U) //index for IFFT or Decompression
#define HWAFFT_RES_IDX_MAX              (2U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* HWA FFT or IFFT Resource and Profile object */
struct HWAFFT_ResObject
{
    /* Resource variables */
    uint32_t                paramIdx;
    uint32_t                rdDmaCh;
    uint32_t                wrDmaCh;
    uint32_t                rdTcc;
    uint32_t                wrTcc;
    uint32_t                rdParamId;
    uint32_t                wrParamId;
    uint32_t                rdMemBankAddr;
    uint32_t                wrMemBankAddr;
    HWA_ParamConfig         paramCfg;
    EDMACCPaRAMEntry       rdDmaParam;
    EDMACCPaRAMEntry       wrDmaParam;

    /* Profile variables */
    uint32_t                totalCycle;
    uint32_t                float2FixedCycle;
    uint32_t                memcpy2HwaCycle;
    uint32_t                hwaCycle;
    uint32_t                memcpyFromHwaCycle;
    uint32_t                fixed2FloatCycle;
};

/* HWA FFT object */
struct HWAFFT_Object
{
    struct HWAFFT_ResObject resObj[HWAFFT_RES_IDX_MAX];
    int32_t                *fixedPointBuf;

    /* State variables */
    HWA_Handle              hwaHandle;
    uint32_t                edmaBaseAddr;
    uint32_t                edmaRegionId;
    HWA_CommonConfig        commonCfg;
    SemaphoreP_Object       paramDoneSem, doneSem;
    HWA_InterruptConfig     paramISRCfg;
};

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef HWA_FFT_H_ */

/** @} */
