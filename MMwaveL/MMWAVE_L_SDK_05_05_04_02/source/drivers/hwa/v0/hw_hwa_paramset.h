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
 *
 */

#ifndef HW_HWA_PARAMSET_H_
#define HW_HWA_PARAMSET_H_

/****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Definition for field TRIGMODE in Register PARAMn_0 */
#define PARAMn_0_TRIGMODE_BIT_START                         0U
#define PARAMn_0_TRIGMODE_BIT_END                           2U

/* Definition for field DMA2ACC_CHANNEL_TRIGSRC in Register PARAMn_0 */
#define PARAMn_0_DMA2ACC_CHANNEL_TRIGSRC_BIT_START          3U
#define PARAMn_0_DMA2ACC_CHANNEL_TRIGSRC_BIT_END            6U

/* Definition for field CM4INTREN in Register PARAMn_0 */
#define PARAMn_0_CM4INTREN_BIT_START                        7U
#define PARAMn_0_CM4INTREN_BIT_END                          7U

/* Definition for field DMATRIGEN in Register PARAMn_0 */
#define PARAMn_0_DMATRIGEN_BIT_START                        8U
#define PARAMn_0_DMATRIGEN_BIT_END                          8U

/* Definition for field ACC2DMA_CHANNEL_TRIGDST in Register PARAMn_0 */
#define PARAMn_0_ACC2DMA_CHANNEL_TRIGDST_BIT_START          9U
#define PARAMn_0_ACC2DMA_CHANNEL_TRIGDST_BIT_END            12U

/* Definition for field BPM_EN in Register PARAMn_0 */
#define PARAMn_0_BPM_EN_BIT_START                           13U
#define PARAMn_0_BPM_EN_BIT_END                             13U

/* Definition for field FFT_EN in Register PARAMn_0 */
#define PARAMn_0_FFT_EN_BIT_START                           14U
#define PARAMn_0_FFT_EN_BIT_END                             14U

/* Definition for field WINDOW_EN in Register PARAMn_0 */
#define PARAMn_0_WINDOW_EN_BIT_START                        15U
#define PARAMn_0_WINDOW_EN_BIT_END                          15U

/* Definition for field LOG2EN in Register PARAMn_0 */
#define PARAMn_0_LOG2EN_BIT_START                           16U
#define PARAMn_0_LOG2EN_BIT_END                             16U

/* Definition for field ABSEN in Register PARAMn_0 */
#define PARAMn_0_ABSEN_BIT_START                            17U
#define PARAMn_0_ABSEN_BIT_END                              17U

/* Definition for field CMULT_MODE in Register PARAMn_0 */
#define PARAMn_0_CMULT_MODE_BIT_START                       18U
#define PARAMn_0_CMULT_MODE_BIT_END                         20U

/* Definition for field ACCEL_MODE in Register PARAMn_0 */
#define PARAMn_0_ACCEL_MODE_BIT_START                       21U
#define PARAMn_0_ACCEL_MODE_BIT_END                         23U

/* Definition for field FFT_OUTPUT_MODE in Register PARAMn_0 */
#define PARAMn_0_FFT_OUTPUT_MODE_BIT_START                  24U
#define PARAMn_0_FFT_OUTPUT_MODE_BIT_END                    25U

/* Definition for field INTF_THRESH_SEL in Register PARAMn_0 */
#define PARAMn_0_INTF_THRESH_SEL_BIT_START                  26U
#define PARAMn_0_INTF_THRESH_SEL_BIT_END                    27U

/* Definition for field INTF_RESET_MODE in Register PARAMn_0 */
#define PARAMn_0_INTF_RESET_MODE_BIT_START                  28U
#define PARAMn_0_INTF_RESET_MODE_BIT_END                    29U

/* Definition for field DCEST_RESET_MODE in Register PARAMn_0 */
#define PARAMn_0_DCEST_RESET_MODE_BIT_START                 30U
#define PARAMn_0_DCEST_RESET_MODE_BIT_END                   31U

/* Definition for field SRCADDR in Register PARAMn_1 */
#define PARAMn_1_SRCADDR_BIT_START                          0U
#define PARAMn_1_SRCADDR_BIT_END                            15U

/* Definition for field DSTADDR in Register PARAMn_1 */
#define PARAMn_1_DSTADDR_BIT_START                          16U
#define PARAMn_1_DSTADDR_BIT_END                            31U

/* Definition for field SRCACNT in Register PARAMn_2 */
#define PARAMn_2_SRCACNT_BIT_START                          0U
#define PARAMn_2_SRCACNT_BIT_END                            11U

/* Definition for field SRCREAL in Register PARAMn_2 */
#define PARAMn_2_SRCREAL_BIT_START                          12U
#define PARAMn_2_SRCREAL_BIT_END                            12U

/* Definition for field SRC16b32b in Register PARAMn_2 */
#define PARAMn_2_SRC16B32B_BIT_START                        13U
#define PARAMn_2_SRC16B32B_BIT_END                          13U

/* Definition for field SRCSIGNED in Register PARAMn_2 */
#define PARAMn_2_SRCSIGNED_BIT_START                        14U
#define PARAMn_2_SRCSIGNED_BIT_END                          14U

/* Definition for field SRCCONJ in Register PARAMn_2 */
#define PARAMn_2_SRCCONJ_BIT_START                          15U
#define PARAMn_2_SRCCONJ_BIT_END                            15U

/* Definition for field DSTACNT in Register PARAMn_2 */
#define PARAMn_2_DSTACNT_BIT_START                          16U
#define PARAMn_2_DSTACNT_BIT_END                            27U

/* Definition for field DSTREAL in Register PARAMn_2 */
#define PARAMn_2_DSTREAL_BIT_START                          28U
#define PARAMn_2_DSTREAL_BIT_END                            28U

/* Definition for field DST16b32b in Register PARAMn_2 */
#define PARAMn_2_DST16B32B_BIT_START                        29U
#define PARAMn_2_DST16B32B_BIT_END                          29U

/* Definition for field DSTSIGNED in Register PARAMn_2 */
#define PARAMn_2_DSTSIGNED_BIT_START                        30U
#define PARAMn_2_DSTSIGNED_BIT_END                          30U

/* Definition for field DSTCONJ in Register PARAMn_2 */
#define PARAMn_2_DSTCONJ_BIT_START                          31U
#define PARAMn_2_DSTCONJ_BIT_END                            31U

/* Definition for field SRCAINDX in Register PARAMn_3 */
#define PARAMn_3_SRCAINDX_BIT_START                         0U
#define PARAMn_3_SRCAINDX_BIT_END                           15U

/* Definition for field DSTAINDX in Register PARAMn_3 */
#define PARAMn_3_DSTAINDX_BIT_START                         16U
#define PARAMn_3_DSTAINDX_BIT_END                           31U

/* Definition for field SRCBINDX in Register PARAMn_4 */
#define PARAMn_4_SRCBINDX_BIT_START                         0U
#define PARAMn_4_SRCBINDX_BIT_END                           15U

/* Definition for field DSTBINDX in Register PARAMn_4 */
#define PARAMn_4_DSTBINDX_BIT_START                         16U
#define PARAMn_4_DSTBINDX_BIT_END                           31U

/* Definition for field REG_BCNT in Register PARAMn_5 */
#define PARAMn_5_REG_BCNT_BIT_START                         0U
#define PARAMn_5_REG_BCNT_BIT_END                           11U

/* Definition for field REG_SRCSCAL in Register PARAMn_5 */
#define PARAMn_5_REG_SRCSCAL_BIT_START                      12U
#define PARAMn_5_REG_SRCSCAL_BIT_END                        15U

/* Definition for field REG_DSTSCAL in Register PARAMn_5 */
#define PARAMn_5_REG_DSTSCAL_BIT_START                      16U
#define PARAMn_5_REG_DSTSCAL_BIT_END                        19U

/* Definition for field REG_DST_SKIP_INIT in Register PARAMn_5 */
#define PARAMn_5_REG_DST_SKIP_INIT_BIT_START                20U
#define PARAMn_5_REG_DST_SKIP_INIT_BIT_END                  29U

/* Definition for field DCSUB_SEL in Register PARAMn_5 */
#define PARAMn_5_DCSUB_SEL_BIT_START                        30U
#define PARAMn_5_DCSUB_SEL_BIT_END                          30U

/* Definition for field DCSUB_EN in Register PARAMn_5 */
#define PARAMn_5_DCSUB_EN_BIT_START                         31U
#define PARAMn_5_DCSUB_EN_BIT_END                           31U

/* Definition for field INTF_ZERO_MODE in Register PARAMn_6 */
#define PARAMn_6_INTF_ZERO_MODE_BIT_START                   0U
#define PARAMn_6_INTF_ZERO_MODE_BIT_END                     1U

/* Definition for field FFTSIZE in Register PARAMn_6 */
#define PARAMn_6_FFTSIZE_BIT_START                          2U
#define PARAMn_6_FFTSIZE_BIT_END                            5U

/* Definition for field WINSYMM in Register PARAMn_6 */
#define PARAMn_6_WINSYMM_BIT_START                          6U
#define PARAMn_6_WINSYMM_BIT_END                            6U

/* Definition for field INTF_THRESH_EN in Register PARAMn_6 */
#define PARAMn_6_INTF_THRESH_EN_BIT_START                   7U
#define PARAMn_6_INTF_THRESH_EN_BIT_END                     7U

/* Definition for field BPMPHASE in Register PARAMn_6 */
#define PARAMn_6_BPMPHASE_BIT_START                         8U
#define PARAMn_6_BPMPHASE_BIT_END                           11U

/* Definition for field WINDOW_START in Register PARAMn_6 */
#define PARAMn_6_WINDOW_START_BIT_START                     12U
#define PARAMn_6_WINDOW_START_BIT_END                       21U

/* Definition for field BFLY_SCALING in Register PARAMn_6 */
#define PARAMn_6_BFLY_SCALING_BIT_START                     22U
#define PARAMn_6_BFLY_SCALING_BIT_END                       31U

/* Definition for field CIRCIRSHIFT in Register PARAMn_7 */
#define PARAMn_7_CIRCIRSHIFT_BIT_START                      0U
#define PARAMn_7_CIRCIRSHIFT_BIT_END                        11U

/* Definition for field TWIDINCR in Register PARAMn_7 */
#define PARAMn_7_TWIDINCR_BIT_START                         12U
#define PARAMn_7_TWIDINCR_BIT_END                           25U

/* Definition for field WINDOW_INTERP_FRACTION in Register PARAMn_7 */
#define PARAMn_7_WINDOW_INTERP_FRACTION_BIT_START           26U
#define PARAMn_7_WINDOW_INTERP_FRACTION_BIT_END             27U

/* Definition for field CIRCSHIFTWRAP in Register PARAMn_7 */
#define PARAMn_7_CIRCSHIFTWRAP_BIT_START                    28U
#define PARAMn_7_CIRCSHIFTWRAP_BIT_END                      31U

/* *
 @struct DSSHWACCPARAMRegs
 * @brief
 *   Module DSS_HW_ACC_PARAM Register Definition
 * @details
 *   This structure is used to access the DSS_HW_ACC_PARAM module registers.
 */

typedef volatile struct DSSHWACCPARAMRegs_t
{
    uint32_t    PARAMn_0;         /* Offset = 0x000 */
    uint32_t    PARAMn_1;           /* Offset = 0x004 */
    uint32_t    PARAMn_2;           /* Offset = 0x008 */
    uint32_t    PARAMn_3;           /* Offset = 0x00C */
    uint32_t    PARAMn_4;           /* Offset = 0x010 */
    uint32_t    PARAMn_5;           /* Offset = 0x014 */
    uint32_t    PARAMn_6;         /* Offset = 0x018 */
    uint32_t    PARAMn_7;         /* Offset = 0x01C */

} DSSHWACCPARAMRegs;

#ifdef __cplusplus
}
#endif

#endif /* HW_HWA_PARAMSET_H_ */
