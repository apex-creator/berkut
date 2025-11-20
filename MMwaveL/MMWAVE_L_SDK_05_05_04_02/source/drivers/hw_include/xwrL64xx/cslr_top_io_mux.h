/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_top_io_mux.h
*/
#ifndef CSLR_TOP_IO_MUX_H_
#define CSLR_TOP_IO_MUX_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PADAA_CFG_REG;
    volatile uint32_t PADAB_CFG_REG;
    volatile uint32_t PADAC_CFG_REG;
    volatile uint32_t PADAD_CFG_REG;
    volatile uint32_t PADAE_CFG_REG;
    volatile uint32_t PADAF_CFG_REG;
    volatile uint32_t PADAG_CFG_REG;
    volatile uint32_t PADAH_CFG_REG;
    volatile uint32_t PADAI_CFG_REG;
    volatile uint32_t PADAJ_CFG_REG;
    volatile uint32_t PADAK_CFG_REG;
    volatile uint32_t PADAL_CFG_REG;
    volatile uint32_t PADAM_CFG_REG;
    volatile uint32_t PADAN_CFG_REG;
    volatile uint32_t PADAO_CFG_REG;
    volatile uint32_t PADAP_CFG_REG;
    volatile uint32_t PADAQ_CFG_REG;
    volatile uint32_t PADAR_CFG_REG;
    volatile uint32_t PADAS_CFG_REG;
    volatile uint32_t PADAT_CFG_REG;
    volatile uint32_t PADAU_CFG_REG;
    volatile uint32_t PADAV_CFG_REG;
    volatile uint32_t PADAW_CFG_REG;
    volatile uint32_t PADAX_CFG_REG;
    volatile uint32_t USERMODEEN;
    volatile uint32_t PADGLBLCFGREG;
    volatile uint32_t IOCFGKICK0;
    volatile uint32_t IOCFGKICK1;
} CSL_top_io_muxRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TOP_IO_MUX_PADAA_CFG_REG                                           (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG                                           (0x00000004U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG                                           (0x00000008U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG                                           (0x0000000CU)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG                                           (0x00000010U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG                                           (0x00000014U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG                                           (0x00000018U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG                                           (0x0000001CU)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG                                           (0x00000020U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG                                           (0x00000024U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG                                           (0x00000028U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG                                           (0x0000002CU)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG                                           (0x00000030U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG                                           (0x00000034U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG                                           (0x00000038U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG                                           (0x0000003CU)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG                                           (0x00000040U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG                                           (0x00000044U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG                                           (0x00000048U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG                                           (0x0000004CU)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG                                           (0x00000050U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG                                           (0x00000054U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG                                           (0x00000058U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG                                           (0x0000005CU)
#define CSL_TOP_IO_MUX_USERMODEEN                                              (0x00000060U)
#define CSL_TOP_IO_MUX_PADGLBLCFGREG                                           (0x00000064U)
#define CSL_TOP_IO_MUX_IOCFGKICK0                                              (0x00000068U)
#define CSL_TOP_IO_MUX_IOCFGKICK1                                              (0x0000006CU)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PADAA_CFG_REG */

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAA_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAA_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAB_CFG_REG */

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAB_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAB_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAC_CFG_REG */

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAC_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAC_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAD_CFG_REG */

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAD_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAD_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAE_CFG_REG */

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAE_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAE_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAF_CFG_REG */

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAF_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAF_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAG_CFG_REG */

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAG_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAG_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAH_CFG_REG */

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAH_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAH_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAI_CFG_REG */

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAI_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAI_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAJ_CFG_REG */

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAJ_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAK_CFG_REG */

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAK_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAK_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAL_CFG_REG */

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAL_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAL_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAM_CFG_REG */

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAM_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAM_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAN_CFG_REG */

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAN_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAN_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAO_CFG_REG */

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAO_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAO_CFG_REG_RESETVAL                                  (0x00000130U)

/* PADAP_CFG_REG */

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PI_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PUPDSEL_RESETVAL                          (0x00000001U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAP_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAP_CFG_REG_RESETVAL                                  (0x000002D0U)

/* PADAQ_CFG_REG */

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PI_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PUPDSEL_RESETVAL                          (0x00000001U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAQ_CFG_REG_RESETVAL                                  (0x000002D0U)

/* PADAR_CFG_REG */

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PI_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAR_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAR_CFG_REG_RESETVAL                                  (0x000000D0U)

/* PADAS_CFG_REG */

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAS_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAS_CFG_REG_RESETVAL                                  (0x00000130U)

/* PADAT_CFG_REG */

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PI_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAT_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAT_CFG_REG_RESETVAL                                  (0x000000D0U)

/* PADAU_CFG_REG */

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAU_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAU_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAV_CFG_REG */

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAV_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAV_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAW_CFG_REG */

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAW_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAW_CFG_REG_RESETVAL                                  (0x000001F0U)

/* PADAX_CFG_REG */

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_FUNC_SEL_MASK                             (0x0000000FU)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_FUNC_SEL_SHIFT                            (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_FUNC_SEL_RESETVAL                         (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_FUNC_SEL_MAX                              (0x0000000FU)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_MASK                     (0x00000010U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_SHIFT                    (0x00000004U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_RESETVAL                 (0x00000001U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_MASK                          (0x00000020U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_SHIFT                         (0x00000005U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_RESETVAL                      (0x00000001U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_IE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_MASK                     (0x00000040U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_SHIFT                    (0x00000006U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_RESETVAL                 (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_CTRL_MAX                      (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_MASK                          (0x00000080U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_SHIFT                         (0x00000007U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_RESETVAL                      (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_OE_OVERRIDE_MAX                           (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PI_MASK                                   (0x00000100U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PI_SHIFT                                  (0x00000008U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PI_RESETVAL                               (0x00000001U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PI_MAX                                    (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PUPDSEL_MASK                              (0x00000200U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PUPDSEL_SHIFT                             (0x00000009U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PUPDSEL_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_PUPDSEL_MAX                               (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_SC1_MASK                                  (0x00000400U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_SC1_SHIFT                                 (0x0000000AU)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_SC1_RESETVAL                              (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_SC1_MAX                                   (0x00000001U)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_NU_MASK                                   (0xFFFFF800U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_NU_SHIFT                                  (0x0000000BU)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_NU_RESETVAL                               (0x00000000U)
#define CSL_TOP_IO_MUX_PADAX_CFG_REG_NU_MAX                                    (0x001FFFFFU)

#define CSL_TOP_IO_MUX_PADAX_CFG_REG_RESETVAL                                  (0x00000130U)

/* USERMODEEN */

#define CSL_TOP_IO_MUX_USERMODEEN_USERMODEEN_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_IO_MUX_USERMODEEN_USERMODEEN_SHIFT                             (0x00000000U)
#define CSL_TOP_IO_MUX_USERMODEEN_USERMODEEN_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_USERMODEEN_USERMODEEN_MAX                               (0xFFFFFFFFU)

#define CSL_TOP_IO_MUX_USERMODEEN_RESETVAL                                     (0x00000000U)

/* PADGLBLCFGREG */

#define CSL_TOP_IO_MUX_PADGLBLCFGREG_PADGLBLCFGREG_MASK                        (0xFFFFFFFFU)
#define CSL_TOP_IO_MUX_PADGLBLCFGREG_PADGLBLCFGREG_SHIFT                       (0x00000000U)
#define CSL_TOP_IO_MUX_PADGLBLCFGREG_PADGLBLCFGREG_RESETVAL                    (0x00000000U)
#define CSL_TOP_IO_MUX_PADGLBLCFGREG_PADGLBLCFGREG_MAX                         (0xFFFFFFFFU)

#define CSL_TOP_IO_MUX_PADGLBLCFGREG_RESETVAL                                  (0x00000000U)

/* IOCFGKICK0 */

#define CSL_TOP_IO_MUX_IOCFGKICK0_IOCFGKICK0_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_IO_MUX_IOCFGKICK0_IOCFGKICK0_SHIFT                             (0x00000000U)
#define CSL_TOP_IO_MUX_IOCFGKICK0_IOCFGKICK0_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_IOCFGKICK0_IOCFGKICK0_MAX                               (0xFFFFFFFFU)

#define CSL_TOP_IO_MUX_IOCFGKICK0_RESETVAL                                     (0x00000000U)

/* IOCFGKICK1 */

#define CSL_TOP_IO_MUX_IOCFGKICK1_IOCFGKICK1_MASK                              (0xFFFFFFFFU)
#define CSL_TOP_IO_MUX_IOCFGKICK1_IOCFGKICK1_SHIFT                             (0x00000000U)
#define CSL_TOP_IO_MUX_IOCFGKICK1_IOCFGKICK1_RESETVAL                          (0x00000000U)
#define CSL_TOP_IO_MUX_IOCFGKICK1_IOCFGKICK1_MAX                               (0xFFFFFFFFU)

#define CSL_TOP_IO_MUX_IOCFGKICK1_RESETVAL                                     (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
