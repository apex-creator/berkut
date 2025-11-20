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
 *  Name        : cslr_top_gio.h
*/
#ifndef CSLR_TOP_GIO_H_
#define CSLR_TOP_GIO_H_

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
    volatile uint32_t GIOGCR;
    volatile uint32_t GIOPWDN;
    volatile uint32_t GIOINTDET;
    volatile uint32_t GIOPOL;
    volatile uint32_t GIOENASET;
    volatile uint32_t GIOENACLR;
    volatile uint32_t GIOLVLSET;
    volatile uint32_t GIOLVLCLR;
    volatile uint32_t GIOFLG;
    volatile uint32_t GIOOFFA;
    volatile uint32_t GIOOFFB;
    volatile uint32_t GIOEMUA;
    volatile uint32_t GIOEMUB;
    volatile uint32_t GIODIRA;
    volatile uint32_t GIODINA;
    volatile uint32_t GIODOUTA;
    volatile uint32_t GIOSETA;
    volatile uint32_t GIOCLRA;
    volatile uint32_t GIOPDRA;
    volatile uint32_t GIOPULDISA;
    volatile uint32_t GIOPSLA;
    volatile uint32_t GIODIRB;
    volatile uint32_t GIODINB;
    volatile uint32_t GIODOUTB;
    volatile uint32_t GIOSETB;
    volatile uint32_t GIOCLRB;
    volatile uint32_t GIOPDRB;
    volatile uint32_t GIOPULDISB;
    volatile uint32_t GIOPSLB;
    volatile uint32_t GIODIRC;
    volatile uint32_t GIODINC;
    volatile uint32_t GIODOUTC;
    volatile uint32_t GIOSETC;
    volatile uint32_t GIOCLRC;
    volatile uint32_t GIOPDRC;
    volatile uint32_t GIOPULDISC;
    volatile uint32_t GIOPSLC;
    volatile uint32_t GIODIRD;
    volatile uint32_t GIODIND;
    volatile uint32_t GIODOUTD;
    volatile uint32_t GIOSETD;
    volatile uint32_t GIOCLRD;
    volatile uint32_t GIOPDRD;
    volatile uint32_t GIOPULDISD;
    volatile uint32_t GIOPSLD;
    volatile uint32_t GIODIRE;
    volatile uint32_t GIODINE;
    volatile uint32_t GIODOUTE;
    volatile uint32_t GIOSETE;
    volatile uint32_t GIOCLRE;
    volatile uint32_t GIOPDRE;
    volatile uint32_t GIOPULDISE;
    volatile uint32_t GIOPSLE;
    volatile uint32_t GIODIRF;
    volatile uint32_t GIODINF;
    volatile uint32_t GIODOUTF;
    volatile uint32_t GIOSETF;
    volatile uint32_t GIOCLRF;
    volatile uint32_t GIOPDRF;
    volatile uint32_t GIOPULDISF;
    volatile uint32_t GIOPSLF;
    volatile uint32_t GIODIRG;
    volatile uint32_t GIODING;
    volatile uint32_t GIODOUTG;
    volatile uint32_t GIOSETG;
    volatile uint32_t GIOCLRG;
    volatile uint32_t GIOPDRG;
    volatile uint32_t GIOPULDISG;
    volatile uint32_t GIOPSLG;
    volatile uint32_t GIODIRH;
    volatile uint32_t GIODINH;
    volatile uint32_t GIODOUTH;
    volatile uint32_t GIOSETH;
    volatile uint32_t GIOCLRH;
    volatile uint32_t GIOPDRH;
    volatile uint32_t GIOPULDISH;
    volatile uint32_t GIOPSLH;
    volatile uint32_t GIOSRCA;
    volatile uint32_t GIOSRCB;
    volatile uint32_t GIOSRCC;
    volatile uint32_t GIOSRCD;
    volatile uint32_t GIOSRCE;
    volatile uint32_t GIOSRCF;
    volatile uint32_t GIOSRCG;
    volatile uint32_t GIOSRCH;
} CSL_top_gioRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TOP_GIO_GIOGCR                                                     (0x00000000U)
#define CSL_TOP_GIO_GIOPWDN                                                    (0x00000004U)
#define CSL_TOP_GIO_GIOINTDET                                                  (0x00000008U)
#define CSL_TOP_GIO_GIOPOL                                                     (0x0000000CU)
#define CSL_TOP_GIO_GIOENASET                                                  (0x00000010U)
#define CSL_TOP_GIO_GIOENACLR                                                  (0x00000014U)
#define CSL_TOP_GIO_GIOLVLSET                                                  (0x00000018U)
#define CSL_TOP_GIO_GIOLVLCLR                                                  (0x0000001CU)
#define CSL_TOP_GIO_GIOFLG                                                     (0x00000020U)
#define CSL_TOP_GIO_GIOOFFA                                                    (0x00000024U)
#define CSL_TOP_GIO_GIOOFFB                                                    (0x00000028U)
#define CSL_TOP_GIO_GIOEMUA                                                    (0x0000002CU)
#define CSL_TOP_GIO_GIOEMUB                                                    (0x00000030U)

#define CSL_TOP_GIO_GIODIR(n)                            (0x34U + ((n) * 0x20U))
#define CSL_TOP_GIO_GIODIN(n)                            (0x38U + ((n) * 0x20U))
#define CSL_TOP_GIO_GIODOUT(n)                           (0x3CU + ((n) * 0x20U))
#define CSL_TOP_GIO_GIOSET(n)                            (0x40U + ((n) * 0x20U))
#define CSL_TOP_GIO_GIOCLR(n)                            (0x44U + ((n) * 0x20U))
#define CSL_TOP_GIO_GIOPDR(n)                            (0x48U + ((n) * 0x20U))
#define CSL_TOP_GIO_GIOPULDIS(n)                         (0x4CU + ((n) * 0x20U))
#define CSL_TOP_GIO_GIOPSL(n)                            (0x50U + ((n) * 0x20U))
#define CSL_TOP_GPIO_GIOSRC(n)                           (0x134U + ((n)) * 0x4U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* GIOGCR */

#define CSL_TOP_GIO_GIOGCR_RESET_MASK                                          (0x00000001U)
#define CSL_TOP_GIO_GIOGCR_RESET_SHIFT                                         (0x00000000U)
#define CSL_TOP_GIO_GIOGCR_RESET_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOGCR_RESET_MAX                                           (0x00000001U)

#define CSL_TOP_GIO_GIOGCR_NU0_MASK                                            (0xFFFFFFFEU)
#define CSL_TOP_GIO_GIOGCR_NU0_SHIFT                                           (0x00000001U)
#define CSL_TOP_GIO_GIOGCR_NU0_RESETVAL                                        (0x00000000U)
#define CSL_TOP_GIO_GIOGCR_NU0_MAX                                             (0x7FFFFFFFU)

#define CSL_TOP_GIO_GIOGCR_RESETVAL                                            (0x00000000U)

/* GIOPWDN */

#define CSL_TOP_GIO_GIOPWDN_GIOPWDN_MASK                                       (0x00000001U)
#define CSL_TOP_GIO_GIOPWDN_GIOPWDN_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPWDN_GIOPWDN_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPWDN_GIOPWDN_MAX                                        (0x00000001U)

#define CSL_TOP_GIO_GIOPWDN_NU_MASK                                            (0xFFFFFFFEU)
#define CSL_TOP_GIO_GIOPWDN_NU_SHIFT                                           (0x00000001U)
#define CSL_TOP_GIO_GIOPWDN_NU_RESETVAL                                        (0x00000000U)
#define CSL_TOP_GIO_GIOPWDN_NU_MAX                                             (0x7FFFFFFFU)

#define CSL_TOP_GIO_GIOPWDN_RESETVAL                                           (0x00000000U)

/* GIOINTDET */

#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_0_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_0_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_0_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_0_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_1_MASK                                 (0x0000FF00U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_1_SHIFT                                (0x00000008U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_1_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_1_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_2_MASK                                 (0x00FF0000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_2_SHIFT                                (0x00000010U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_2_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_2_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_3_MASK                                 (0xFF000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_3_SHIFT                                (0x00000018U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_3_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOINTDET_GIOINTDET_3_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOINTDET_RESETVAL                                         (0x00000000U)

/* GIOPOL */

#define CSL_TOP_GIO_GIOPOL_GIOPOL_0_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_0_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_0_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_0_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPOL_GIOPOL_1_MASK                                       (0x0000FF00U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_1_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_1_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_1_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPOL_GIOPOL_2_MASK                                       (0x00FF0000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_2_SHIFT                                      (0x00000010U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_2_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_2_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPOL_GIOPOL_3_MASK                                       (0xFF000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_3_SHIFT                                      (0x00000018U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_3_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPOL_GIOPOL_3_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPOL_RESETVAL                                            (0x00000000U)

/* GIOENASET */

#define CSL_TOP_GIO_GIOENASET_GIOENASET_0_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_0_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_0_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_0_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENASET_GIOENASET_1_MASK                                 (0x0000FF00U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_1_SHIFT                                (0x00000008U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_1_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_1_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENASET_GIOENASET_2_MASK                                 (0x00FF0000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_2_SHIFT                                (0x00000010U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_2_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_2_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENASET_GIOENASET_3_MASK                                 (0xFF000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_3_SHIFT                                (0x00000018U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_3_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENASET_GIOENASET_3_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENASET_RESETVAL                                         (0x00000000U)

/* GIOENACLR */

#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_0_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_0_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_0_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_0_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_1_MASK                                 (0x0000FF00U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_1_SHIFT                                (0x00000008U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_1_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_1_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_2_MASK                                 (0x00FF0000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_2_SHIFT                                (0x00000010U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_2_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_2_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_3_MASK                                 (0xFF000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_3_SHIFT                                (0x00000018U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_3_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOENACLR_GIOENACLR_3_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOENACLR_RESETVAL                                         (0x00000000U)

/* GIOLVLSET */

#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_0_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_0_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_0_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_0_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_1_MASK                                 (0x0000FF00U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_1_SHIFT                                (0x00000008U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_1_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_1_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_2_MASK                                 (0x00FF0000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_2_SHIFT                                (0x00000010U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_2_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_2_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_3_MASK                                 (0xFF000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_3_SHIFT                                (0x00000018U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_3_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLSET_GIOLVLSET_3_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLSET_RESETVAL                                         (0x00000000U)

/* GIOLVLCLR */

#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_0_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_0_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_0_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_0_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_1_MASK                                 (0x0000FF00U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_1_SHIFT                                (0x00000008U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_1_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_1_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_2_MASK                                 (0x00FF0000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_2_SHIFT                                (0x00000010U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_2_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_2_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_3_MASK                                 (0xFF000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_3_SHIFT                                (0x00000018U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_3_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOLVLCLR_GIOLVLCLR_3_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOLVLCLR_RESETVAL                                         (0x00000000U)

/* GIOFLG */

#define CSL_TOP_GIO_GIOFLG_GIOFLG_0_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_0_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_0_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_0_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOFLG_GIOFLG_1_MASK                                       (0x0000FF00U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_1_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_1_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_1_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOFLG_GIOFLG_2_MASK                                       (0x00FF0000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_2_SHIFT                                      (0x00000010U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_2_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_2_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOFLG_GIOFLG_3_MASK                                       (0xFF000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_3_SHIFT                                      (0x00000018U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_3_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOFLG_GIOFLG_3_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOFLG_RESETVAL                                            (0x00000000U)

/* GIOOFFA */

#define CSL_TOP_GIO_GIOOFFA_GIOOFFA_MASK                                       (0x0000003FU)
#define CSL_TOP_GIO_GIOOFFA_GIOOFFA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOOFFA_GIOOFFA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOOFFA_GIOOFFA_MAX                                        (0x0000003FU)

#define CSL_TOP_GIO_GIOOFFA_NU1_MASK                                           (0xFFFFFFC0U)
#define CSL_TOP_GIO_GIOOFFA_NU1_SHIFT                                          (0x00000006U)
#define CSL_TOP_GIO_GIOOFFA_NU1_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIOOFFA_NU1_MAX                                            (0x03FFFFFFU)

#define CSL_TOP_GIO_GIOOFFA_RESETVAL                                           (0x00000000U)

/* GIOOFFB */

#define CSL_TOP_GIO_GIOOFFB_GIOOFFB_MASK                                       (0x0000003FU)
#define CSL_TOP_GIO_GIOOFFB_GIOOFFB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOOFFB_GIOOFFB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOOFFB_GIOOFFB_MAX                                        (0x0000003FU)

#define CSL_TOP_GIO_GIOOFFB_NU2_MASK                                           (0xFFFFFFC0U)
#define CSL_TOP_GIO_GIOOFFB_NU2_SHIFT                                          (0x00000006U)
#define CSL_TOP_GIO_GIOOFFB_NU2_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIOOFFB_NU2_MAX                                            (0x03FFFFFFU)

#define CSL_TOP_GIO_GIOOFFB_RESETVAL                                           (0x00000000U)

/* GIOEMUA */

#define CSL_TOP_GIO_GIOEMUA_GIOEMUA_MASK                                       (0x0000003FU)
#define CSL_TOP_GIO_GIOEMUA_GIOEMUA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOEMUA_GIOEMUA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOEMUA_GIOEMUA_MAX                                        (0x0000003FU)

#define CSL_TOP_GIO_GIOEMUA_NU3_MASK                                           (0xFFFFFFC0U)
#define CSL_TOP_GIO_GIOEMUA_NU3_SHIFT                                          (0x00000006U)
#define CSL_TOP_GIO_GIOEMUA_NU3_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIOEMUA_NU3_MAX                                            (0x03FFFFFFU)

#define CSL_TOP_GIO_GIOEMUA_RESETVAL                                           (0x00000000U)

/* GIOEMUB */

#define CSL_TOP_GIO_GIOEMUB_GIOEMUB_MASK                                       (0x0000003FU)
#define CSL_TOP_GIO_GIOEMUB_GIOEMUB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOEMUB_GIOEMUB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOEMUB_GIOEMUB_MAX                                        (0x0000003FU)

#define CSL_TOP_GIO_GIOEMUB_NU4_MASK                                           (0xFFFFFFC0U)
#define CSL_TOP_GIO_GIOEMUB_NU4_SHIFT                                          (0x00000006U)
#define CSL_TOP_GIO_GIOEMUB_NU4_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIOEMUB_NU4_MAX                                            (0x03FFFFFFU)

#define CSL_TOP_GIO_GIOEMUB_RESETVAL                                           (0x00000000U)

/* GIODIRA */

#define CSL_TOP_GIO_GIODIRA_GIODIRA_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRA_GIODIRA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRA_GIODIRA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRA_GIODIRA_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRA_NU5_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRA_NU5_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRA_NU5_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRA_NU5_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRA_RESETVAL                                           (0x00000000U)

/* GIODINA */

#define CSL_TOP_GIO_GIODINA_GIODINA_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINA_GIODINA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINA_GIODINA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINA_GIODINA_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINA_NU11_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINA_NU11_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINA_NU11_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINA_NU11_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINA_RESETVAL                                           (0x00000000U)

/* GIODOUTA */

#define CSL_TOP_GIO_GIODOUTA_GIODOUTA_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTA_GIODOUTA_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTA_GIODOUTA_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTA_GIODOUTA_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTA_NU17_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTA_NU17_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTA_NU17_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTA_NU17_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTA_RESETVAL                                          (0x00000000U)

/* GIOSETA */

#define CSL_TOP_GIO_GIOSETA_GIODSETA_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETA_GIODSETA_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETA_GIODSETA_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETA_GIODSETA_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETA_NU23_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETA_NU23_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETA_NU23_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETA_NU23_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETA_RESETVAL                                           (0x00000000U)

/* GIOCLRA */

#define CSL_TOP_GIO_GIOCLRA_GIODCLRA_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRA_GIODCLRA_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRA_GIODCLRA_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRA_GIODCLRA_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRA_NU29_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRA_NU29_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRA_NU29_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRA_NU29_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRA_RESETVAL                                           (0x00000000U)

/* GIOPDRA */

#define CSL_TOP_GIO_GIOPDRA_GIOPDRA_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRA_GIOPDRA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRA_GIOPDRA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRA_GIOPDRA_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRA_NU35_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRA_NU35_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRA_NU35_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRA_NU35_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRA_RESETVAL                                           (0x00000000U)

/* GIOPULDISA */

#define CSL_TOP_GIO_GIOPULDISA_GIOPULDISA_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISA_GIOPULDISA_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISA_GIOPULDISA_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISA_GIOPULDISA_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISA_NU_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISA_NU_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISA_NU_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISA_NU_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISA_RESETVAL                                        (0x00000000U)

/* GIOPSLA */

#define CSL_TOP_GIO_GIOPSLA_GIOPSLA_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLA_GIOPSLA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLA_GIOPSLA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLA_GIOPSLA_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLA_NU35_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLA_NU35_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLA_NU35_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLA_NU35_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLA_RESETVAL                                           (0x00000000U)

/* GIODIRB */

#define CSL_TOP_GIO_GIODIRB_GIODIRB_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRB_GIODIRB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRB_GIODIRB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRB_GIODIRB_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRB_NU6_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRB_NU6_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRB_NU6_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRB_NU6_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRB_RESETVAL                                           (0x00000000U)

/* GIODINB */

#define CSL_TOP_GIO_GIODINB_GIODINB_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINB_GIODINB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINB_GIODINB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINB_GIODINB_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINB_NU12_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINB_NU12_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINB_NU12_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINB_NU12_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINB_RESETVAL                                           (0x00000000U)

/* GIODOUTB */

#define CSL_TOP_GIO_GIODOUTB_GIODOUTB_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTB_GIODOUTB_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTB_GIODOUTB_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTB_GIODOUTB_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTB_NU18_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTB_NU18_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTB_NU18_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTB_NU18_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTB_RESETVAL                                          (0x00000000U)

/* GIOSETB */

#define CSL_TOP_GIO_GIOSETB_GIODSETB_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETB_GIODSETB_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETB_GIODSETB_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETB_GIODSETB_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETB_NU24_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETB_NU24_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETB_NU24_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETB_NU24_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETB_RESETVAL                                           (0x00000000U)

/* GIOCLRB */

#define CSL_TOP_GIO_GIOCLRB_GIODCLRB_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRB_GIODCLRB_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRB_GIODCLRB_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRB_GIODCLRB_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRB_NU30_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRB_NU30_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRB_NU30_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRB_NU30_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRB_RESETVAL                                           (0x00000000U)

/* GIOPDRB */

#define CSL_TOP_GIO_GIOPDRB_GIOPDRB_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRB_GIOPDRB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRB_GIOPDRB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRB_GIOPDRB_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRB_NU36_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRB_NU36_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRB_NU36_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRB_NU36_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRB_RESETVAL                                           (0x00000000U)

/* GIOPULDISB */

#define CSL_TOP_GIO_GIOPULDISB_GIOPULDISB_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISB_GIOPULDISB_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISB_GIOPULDISB_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISB_GIOPULDISB_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISB_NU36_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISB_NU36_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISB_NU36_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISB_NU36_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISB_RESETVAL                                        (0x00000000U)

/* GIOPSLB */

#define CSL_TOP_GIO_GIOPSLB_GIOPSLB_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLB_GIOPSLB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLB_GIOPSLB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLB_GIOPSLB_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLB_NU36_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLB_NU36_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLB_NU36_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLB_NU36_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLB_RESETVAL                                           (0x00000000U)

/* GIODIRC */

#define CSL_TOP_GIO_GIODIRC_GIODIRC_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRC_GIODIRC_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRC_GIODIRC_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRC_GIODIRC_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRC_NU7_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRC_NU7_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRC_NU7_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRC_NU7_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRC_RESETVAL                                           (0x00000000U)

/* GIODINC */

#define CSL_TOP_GIO_GIODINC_GIODINC_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINC_GIODINC_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINC_GIODINC_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINC_GIODINC_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINC_NU13_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINC_NU13_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINC_NU13_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINC_NU13_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINC_RESETVAL                                           (0x00000000U)

/* GIODOUTC */

#define CSL_TOP_GIO_GIODOUTC_GIODOUTC_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTC_GIODOUTC_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTC_GIODOUTC_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTC_GIODOUTC_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTC_NU19_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTC_NU19_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTC_NU19_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTC_NU19_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTC_RESETVAL                                          (0x00000000U)

/* GIOSETC */

#define CSL_TOP_GIO_GIOSETC_GIODSETC_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETC_GIODSETC_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETC_GIODSETC_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETC_GIODSETC_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETC_NU25_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETC_NU25_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETC_NU25_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETC_NU25_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETC_RESETVAL                                           (0x00000000U)

/* GIOCLRC */

#define CSL_TOP_GIO_GIOCLRC_GIODCLRC_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRC_GIODCLRC_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRC_GIODCLRC_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRC_GIODCLRC_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRC_NU31_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRC_NU31_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRC_NU31_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRC_NU31_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRC_RESETVAL                                           (0x00000000U)

/* GIOPDRC */

#define CSL_TOP_GIO_GIOPDRC_GIOPDRC_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRC_GIOPDRC_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRC_GIOPDRC_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRC_GIOPDRC_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRC_NU37_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRC_NU37_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRC_NU37_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRC_NU37_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRC_RESETVAL                                           (0x00000000U)

/* GIOPULDISC */

#define CSL_TOP_GIO_GIOPULDISC_GIOPULDISC_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISC_GIOPULDISC_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISC_GIOPULDISC_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISC_GIOPULDISC_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISC_NU37_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISC_NU37_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISC_NU37_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISC_NU37_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISC_RESETVAL                                        (0x00000000U)

/* GIOPSLC */

#define CSL_TOP_GIO_GIOPSLC_GIOPSLC_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLC_GIOPSLC_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLC_GIOPSLC_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLC_GIOPSLC_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLC_NU37_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLC_NU37_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLC_NU37_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLC_NU37_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLC_RESETVAL                                           (0x00000000U)

/* GIODIRD */

#define CSL_TOP_GIO_GIODIRD_GIODIRD_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRD_GIODIRD_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRD_GIODIRD_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRD_GIODIRD_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRD_NU8_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRD_NU8_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRD_NU8_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRD_NU8_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRD_RESETVAL                                           (0x00000000U)

/* GIODIND */

#define CSL_TOP_GIO_GIODIND_GIODIND_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIND_GIODIND_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIND_GIODIND_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIND_GIODIND_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIND_NU14_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIND_NU14_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODIND_NU14_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIND_NU14_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIND_RESETVAL                                           (0x00000000U)

/* GIODOUTD */

#define CSL_TOP_GIO_GIODOUTD_GIODOUTD_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTD_GIODOUTD_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTD_GIODOUTD_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTD_GIODOUTD_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTD_NU20_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTD_NU20_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTD_NU20_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTD_NU20_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTD_RESETVAL                                          (0x00000000U)

/* GIOSETD */

#define CSL_TOP_GIO_GIOSETD_GIODSETD_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETD_GIODSETD_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETD_GIODSETD_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETD_GIODSETD_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETD_NU26_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETD_NU26_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETD_NU26_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETD_NU26_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETD_RESETVAL                                           (0x00000000U)

/* GIOCLRD */

#define CSL_TOP_GIO_GIOCLRD_GIODCLRD_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRD_GIODCLRD_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRD_GIODCLRD_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRD_GIODCLRD_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRD_NU32_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRD_NU32_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRD_NU32_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRD_NU32_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRD_RESETVAL                                           (0x00000000U)

/* GIOPDRD */

#define CSL_TOP_GIO_GIOPDRD_GIOPDRD_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRD_GIOPDRD_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRD_GIOPDRD_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRD_GIOPDRD_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRD_NU38_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRD_NU38_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRD_NU38_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRD_NU38_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRD_RESETVAL                                           (0x00000000U)

/* GIOPULDISD */

#define CSL_TOP_GIO_GIOPULDISD_GIOPULDISD_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISD_GIOPULDISD_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISD_GIOPULDISD_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISD_GIOPULDISD_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISD_NU38_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISD_NU38_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISD_NU38_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISD_NU38_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISD_RESETVAL                                        (0x00000000U)

/* GIOPSLD */

#define CSL_TOP_GIO_GIOPSLD_GIOPSLD_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLD_GIOPSLD_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLD_GIOPSLD_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLD_GIOPSLD_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLD_NU38_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLD_NU38_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLD_NU38_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLD_NU38_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLD_RESETVAL                                           (0x00000000U)

/* GIODIRE */

#define CSL_TOP_GIO_GIODIRE_GIODIRE_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRE_GIODIRE_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRE_GIODIRE_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRE_GIODIRE_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRE_NU9_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRE_NU9_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRE_NU9_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRE_NU9_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRE_RESETVAL                                           (0x00000000U)

/* GIODINE */

#define CSL_TOP_GIO_GIODINE_GIODINE_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINE_GIODINE_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINE_GIODINE_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINE_GIODINE_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINE_NU15_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINE_NU15_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINE_NU15_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINE_NU15_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINE_RESETVAL                                           (0x00000000U)

/* GIODOUTE */

#define CSL_TOP_GIO_GIODOUTE_GIODOUTE_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTE_GIODOUTE_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTE_GIODOUTE_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTE_GIODOUTE_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTE_NU21_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTE_NU21_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTE_NU21_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTE_NU21_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTE_RESETVAL                                          (0x00000000U)

/* GIOSETE */

#define CSL_TOP_GIO_GIOSETE_GIODSETE_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETE_GIODSETE_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETE_GIODSETE_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETE_GIODSETE_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETE_NU27_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETE_NU27_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETE_NU27_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETE_NU27_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETE_RESETVAL                                           (0x00000000U)

/* GIOCLRE */

#define CSL_TOP_GIO_GIOCLRE_GIODCLRE_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRE_GIODCLRE_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRE_GIODCLRE_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRE_GIODCLRE_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRE_NU33_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRE_NU33_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRE_NU33_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRE_NU33_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRE_RESETVAL                                           (0x00000000U)

/* GIOPDRE */

#define CSL_TOP_GIO_GIOPDRE_GIOPDRE_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRE_GIOPDRE_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRE_GIOPDRE_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRE_GIOPDRE_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRE_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRE_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRE_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRE_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRE_RESETVAL                                           (0x00000000U)

/* GIOPULDISE */

#define CSL_TOP_GIO_GIOPULDISE_GIOPULDISE_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISE_GIOPULDISE_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISE_GIOPULDISE_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISE_GIOPULDISE_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISE_NU39_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISE_NU39_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISE_NU39_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISE_NU39_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISE_RESETVAL                                        (0x00000000U)

/* GIOPSLE */

#define CSL_TOP_GIO_GIOPSLE_GIOPSLE_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLE_GIOPSLE_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLE_GIOPSLE_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLE_GIOPSLE_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLE_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLE_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLE_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLE_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLE_RESETVAL                                           (0x00000000U)

/* GIODIRF */

#define CSL_TOP_GIO_GIODIRF_GIODIRF_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRF_GIODIRF_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRF_GIODIRF_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRF_GIODIRF_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRF_NU10_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRF_NU10_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODIRF_NU10_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRF_NU10_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRF_RESETVAL                                           (0x00000000U)

/* GIODINF */

#define CSL_TOP_GIO_GIODINF_GIODINF_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINF_GIODINF_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINF_GIODINF_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINF_GIODINF_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINF_NU16_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINF_NU16_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINF_NU16_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINF_NU16_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINF_RESETVAL                                           (0x00000000U)

/* GIODOUTF */

#define CSL_TOP_GIO_GIODOUTF_GIODOUTF_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTF_GIODOUTF_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTF_GIODOUTF_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTF_GIODOUTF_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTF_NU22_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTF_NU22_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTF_NU22_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTF_NU22_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTF_RESETVAL                                          (0x00000000U)

/* GIOSETF */

#define CSL_TOP_GIO_GIOSETF_GIODSETF_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETF_GIODSETF_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETF_GIODSETF_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETF_GIODSETF_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETF_NU28_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETF_NU28_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETF_NU28_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETF_NU28_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETF_RESETVAL                                           (0x00000000U)

/* GIOCLRF */

#define CSL_TOP_GIO_GIOCLRF_GIODCLRF_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRF_GIODCLRF_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRF_GIODCLRF_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRF_GIODCLRF_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRF_NU34_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRF_NU34_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRF_NU34_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRF_NU34_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRF_RESETVAL                                           (0x00000000U)

/* GIOPDRF */

#define CSL_TOP_GIO_GIOPDRF_GIOPDRF_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRF_GIOPDRF_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRF_GIOPDRF_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRF_GIOPDRF_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRF_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRF_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRF_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRF_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRF_RESETVAL                                           (0x00000000U)

/* GIOPULDISF */

#define CSL_TOP_GIO_GIOPULDISF_GIOPULDISF_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISF_GIOPULDISF_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISF_GIOPULDISF_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISF_GIOPULDISF_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISF_NU40_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISF_NU40_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISF_NU40_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISF_NU40_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISF_RESETVAL                                        (0x00000000U)

/* GIOPSLF */

#define CSL_TOP_GIO_GIOPSLF_GIOPSLF_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLF_GIOPSLF_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLF_GIOPSLF_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLF_GIOPSLF_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLF_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLF_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLF_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLF_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLF_RESETVAL                                           (0x00000000U)

/* GIODIRG */

#define CSL_TOP_GIO_GIODIRG_GIODIRG_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRG_GIODIRG_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRG_GIODIRG_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRG_GIODIRG_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRG_NU9_MASK                                           (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRG_NU9_SHIFT                                          (0x00000008U)
#define CSL_TOP_GIO_GIODIRG_NU9_RESETVAL                                       (0x00000000U)
#define CSL_TOP_GIO_GIODIRG_NU9_MAX                                            (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRG_RESETVAL                                           (0x00000000U)

/* GIODING */

#define CSL_TOP_GIO_GIODING_GIODING_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODING_GIODING_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODING_GIODING_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODING_GIODING_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODING_NU15_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODING_NU15_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODING_NU15_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODING_NU15_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODING_RESETVAL                                           (0x00000000U)

/* GIODOUTG */

#define CSL_TOP_GIO_GIODOUTG_GIODOUTG_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTG_GIODOUTG_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTG_GIODOUTG_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTG_GIODOUTG_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTG_NU21_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTG_NU21_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTG_NU21_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTG_NU21_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTG_RESETVAL                                          (0x00000000U)

/* GIOSETG */

#define CSL_TOP_GIO_GIOSETG_GIODSETG_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETG_GIODSETG_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETG_GIODSETG_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETG_GIODSETG_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETG_NU27_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETG_NU27_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETG_NU27_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETG_NU27_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETG_RESETVAL                                           (0x00000000U)

/* GIOCLRG */

#define CSL_TOP_GIO_GIOCLRG_GIODCLRG_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRG_GIODCLRG_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRG_GIODCLRG_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRG_GIODCLRG_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRG_NU33_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRG_NU33_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRG_NU33_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRG_NU33_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRG_RESETVAL                                           (0x00000000U)

/* GIOPDRG */

#define CSL_TOP_GIO_GIOPDRG_GIOPDRG_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRG_GIOPDRG_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRG_GIOPDRG_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRG_GIOPDRG_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRG_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRG_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRG_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRG_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRG_RESETVAL                                           (0x00000000U)

/* GIOPULDISG */

#define CSL_TOP_GIO_GIOPULDISG_GIOPULDISG_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISG_GIOPULDISG_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISG_GIOPULDISG_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISG_GIOPULDISG_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISG_NU39_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISG_NU39_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISG_NU39_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISG_NU39_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISG_RESETVAL                                        (0x00000000U)

/* GIOPSLG */

#define CSL_TOP_GIO_GIOPSLG_GIOPSLG_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLG_GIOPSLG_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLG_GIOPSLG_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLG_GIOPSLG_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLG_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLG_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLG_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLG_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLG_RESETVAL                                           (0x00000000U)

/* GIODIRH */

#define CSL_TOP_GIO_GIODIRH_GIODIRH_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODIRH_GIODIRH_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRH_GIODIRH_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODIRH_GIODIRH_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODIRH_NU10_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODIRH_NU10_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODIRH_NU10_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODIRH_NU10_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODIRH_RESETVAL                                           (0x00000000U)

/* GIODINH */

#define CSL_TOP_GIO_GIODINH_GIODINH_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIODINH_GIODINH_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINH_GIODINH_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIODINH_GIODINH_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIODINH_NU16_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODINH_NU16_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIODINH_NU16_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIODINH_NU16_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODINH_RESETVAL                                           (0x00000000U)

/* GIODOUTH */

#define CSL_TOP_GIO_GIODOUTH_GIODOUTH_MASK                                     (0x000000FFU)
#define CSL_TOP_GIO_GIODOUTH_GIODOUTH_SHIFT                                    (0x00000000U)
#define CSL_TOP_GIO_GIODOUTH_GIODOUTH_RESETVAL                                 (0x00000000U)
#define CSL_TOP_GIO_GIODOUTH_GIODOUTH_MAX                                      (0x000000FFU)

#define CSL_TOP_GIO_GIODOUTH_NU22_MASK                                         (0xFFFFFF00U)
#define CSL_TOP_GIO_GIODOUTH_NU22_SHIFT                                        (0x00000008U)
#define CSL_TOP_GIO_GIODOUTH_NU22_RESETVAL                                     (0x00000000U)
#define CSL_TOP_GIO_GIODOUTH_NU22_MAX                                          (0x00FFFFFFU)

#define CSL_TOP_GIO_GIODOUTH_RESETVAL                                          (0x00000000U)

/* GIOSETH */

#define CSL_TOP_GIO_GIOSETH_GIODSETH_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOSETH_GIODSETH_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOSETH_GIODSETH_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOSETH_GIODSETH_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOSETH_NU28_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSETH_NU28_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSETH_NU28_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSETH_NU28_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSETH_RESETVAL                                           (0x00000000U)

/* GIOCLRH */

#define CSL_TOP_GIO_GIOCLRH_GIODCLRH_MASK                                      (0x000000FFU)
#define CSL_TOP_GIO_GIOCLRH_GIODCLRH_SHIFT                                     (0x00000000U)
#define CSL_TOP_GIO_GIOCLRH_GIODCLRH_RESETVAL                                  (0x00000000U)
#define CSL_TOP_GIO_GIOCLRH_GIODCLRH_MAX                                       (0x000000FFU)

#define CSL_TOP_GIO_GIOCLRH_NU34_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOCLRH_NU34_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOCLRH_NU34_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOCLRH_NU34_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOCLRH_RESETVAL                                           (0x00000000U)

/* GIOPDRH */

#define CSL_TOP_GIO_GIOPDRH_GIOPDRH_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPDRH_GIOPDRH_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRH_GIOPDRH_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPDRH_GIOPDRH_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPDRH_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPDRH_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPDRH_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPDRH_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPDRH_RESETVAL                                           (0x00000000U)

/* GIOPULDISH */

#define CSL_TOP_GIO_GIOPULDISH_GIOPULDISH_MASK                                 (0x000000FFU)
#define CSL_TOP_GIO_GIOPULDISH_GIOPULDISH_SHIFT                                (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISH_GIOPULDISH_RESETVAL                             (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISH_GIOPULDISH_MAX                                  (0x000000FFU)

#define CSL_TOP_GIO_GIOPULDISH_NU40_MASK                                       (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPULDISH_NU40_SHIFT                                      (0x00000008U)
#define CSL_TOP_GIO_GIOPULDISH_NU40_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPULDISH_NU40_MAX                                        (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPULDISH_RESETVAL                                        (0x00000000U)

/* GIOPSLH */

#define CSL_TOP_GIO_GIOPSLH_GIOPSLH_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOPSLH_GIOPSLH_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLH_GIOPSLH_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOPSLH_GIOPSLH_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOPSLH_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOPSLH_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOPSLH_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOPSLH_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOPSLH_RESETVAL                                           (0x00000000U)

/* GIOSRCA */

#define CSL_TOP_GIO_GIOSRCA_GIOSRCA_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCA_GIOSRCA_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCA_GIOSRCA_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCA_GIOSRCA_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCA_NU35_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCA_NU35_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCA_NU35_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCA_NU35_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCA_RESETVAL                                           (0x00000000U)

/* GIOSRCB */

#define CSL_TOP_GIO_GIOSRCB_GIOSRCB_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCB_GIOSRCB_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCB_GIOSRCB_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCB_GIOSRCB_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCB_NU36_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCB_NU36_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCB_NU36_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCB_NU36_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCB_RESETVAL                                           (0x00000000U)

/* GIOSRCC */

#define CSL_TOP_GIO_GIOSRCC_GIOSRCC_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCC_GIOSRCC_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCC_GIOSRCC_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCC_GIOSRCC_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCC_NU37_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCC_NU37_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCC_NU37_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCC_NU37_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCC_RESETVAL                                           (0x00000000U)

/* GIOSRCD */

#define CSL_TOP_GIO_GIOSRCD_GIOSRCD_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCD_GIOSRCD_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCD_GIOSRCD_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCD_GIOSRCD_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCD_NU38_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCD_NU38_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCD_NU38_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCD_NU38_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCD_RESETVAL                                           (0x00000000U)

/* GIOSRCE */

#define CSL_TOP_GIO_GIOSRCE_GIOSRCE_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCE_GIOSRCE_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCE_GIOSRCE_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCE_GIOSRCE_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCE_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCE_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCE_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCE_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCE_RESETVAL                                           (0x00000000U)

/* GIOSRCF */

#define CSL_TOP_GIO_GIOSRCF_GIOSRCF_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCF_GIOSRCF_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCF_GIOSRCF_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCF_GIOSRCF_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCF_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCF_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCF_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCF_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCF_RESETVAL                                           (0x00000000U)

/* GIOSRCG */

#define CSL_TOP_GIO_GIOSRCG_GIOSRCG_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCG_GIOSRCG_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCG_GIOSRCG_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCG_GIOSRCG_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCG_NU39_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCG_NU39_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCG_NU39_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCG_NU39_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCG_RESETVAL                                           (0x00000000U)

/* GIOSRCH */

#define CSL_TOP_GIO_GIOSRCH_GIOSRCH_MASK                                       (0x000000FFU)
#define CSL_TOP_GIO_GIOSRCH_GIOSRCH_SHIFT                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCH_GIOSRCH_RESETVAL                                   (0x00000000U)
#define CSL_TOP_GIO_GIOSRCH_GIOSRCH_MAX                                        (0x000000FFU)

#define CSL_TOP_GIO_GIOSRCH_NU40_MASK                                          (0xFFFFFF00U)
#define CSL_TOP_GIO_GIOSRCH_NU40_SHIFT                                         (0x00000008U)
#define CSL_TOP_GIO_GIOSRCH_NU40_RESETVAL                                      (0x00000000U)
#define CSL_TOP_GIO_GIOSRCH_NU40_MAX                                           (0x00FFFFFFU)

#define CSL_TOP_GIO_GIOSRCH_RESETVAL                                           (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
