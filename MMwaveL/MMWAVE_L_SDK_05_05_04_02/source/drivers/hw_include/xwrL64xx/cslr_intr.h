/********************************************************************
*
* MSS INTR Map Header file
*
* Copyright (C) 2020 Texas Instruments Incorporated.
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
#ifndef CSLR_INTR_H_
#define CSLR_INTR_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * @brief Defines for interrupt mapping of APPSS
 */
#define CSL_APPSS_INTR_WIC_IRQ                                                       (0U)
#define CSL_APPSS_INTR_APPSS_ESM_LO_IRQ                                              (1U)
#define CSL_APPSS_INTR_FEC_INTR0                                                     (2U)
#define CSL_APPSS_INTR_FEC_INTR1                                                     (3U)
#define CSL_APPSS_INTR_FEC_INTR2                                                     (4U)
#define CSL_APPSS_INTR_FEC_INTR3                                                     (5U)
#define CSL_APPSS_INTR_APPSS_GIO_INT0                                                (6U)
#define CSL_APPSS_INTR_APPSS_GIO_INT1                                                (7U)
#define CSL_APPSS_INTR_APPSS_SCI1_INT0                                               (8U)
#define CSL_APPSS_INTR_APPSS_SCI1_INT1                                               (9U)
#define CSL_APPSS_INTR_APPSS_LIN_INT0                                                (10U)
#define CSL_APPSS_INTR_APPSS_LIN_INT1                                                (11U)
#define CSL_APPSS_INTR_APPSS_DCC_DONE_INT                                            (12U)
#define CSL_APPSS_INTR_APPSS_QSPI_INTR_REQ                                           (13U)
#define CSL_APPSS_INTR_APPSS_SPI_IRQ_REQ                                             (14U)
#define CSL_APPSS_INTR_APPSS_TPCC1_INTAGG                                            (15U)
#define CSL_APPSS_INTR_APPSS_TPCC1_ERRAGG                                            (16U)
#define CSL_APPSS_INTR_APPSS_TPCC2_INTAGG                                            (17U)
#define CSL_APPSS_INTR_APPSS_TPCC2_ERRAGG                                            (18U)
#define CSL_APPSS_INTR_APPSS_I2C_INT                                                 (19U)
#define CSL_APPSS_INTR_APPSS_MCRC_INT                                                (20U)
#define CSL_APPSS_INTR_APPSS_MCAN_INT0                                               (21U)
#define CSL_APPSS_INTR_APPSS_MCAN_INT1                                               (22U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT1                                            (23U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT2                                            (24U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT3                                            (25U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT4                                            (26U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT5                                            (27U)
#define CSL_APPSS_INTR_APPSS_MCAN_FE_INT6_AND_SPI2_IRQ_REQ                           (28U)
#define CSL_APPSS_INTR_MUXED_APPSS_MCAN_FE_INT7_AND_DEBUGSS_TXDATA_AVAIL             (29U)
#define CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_CHIRP_START_AND_CHIRP_END              (30U)
#define CSL_APPSS_INTR_MUXED_FECSS_CHIRPTIMER_BURST_START_AND_BURST_END              (31U)
#define CSL_APPSS_INTR_FECSS_CHIRPTIMER_FRAME_END                                    (32U)
#define CSL_APPSS_INTR_FECSS_FRAMETIMER_FRAME_START                                  (33U)
#define CSL_APPSS_INTR_MUXED_FECSS_CHIRP_AVAIL_IRQ_AND_ADC_VALID_START_AND_SYNC_IN   (34U)
#define CSL_APPSS_INTR_MUXED_FECSS_FRAME_START_OFFSET_INTR_TIME1                     (35U)
#define CSL_APPSS_INTR_FECSS_FRAME_START_OFFSET_INTR_TIME2                           (36U)
#define CSL_APPSS_INTR_FECSS_FRAME_START_OFFSET_INTR_TIME3                           (37U)
#define CSL_APPSS_INTR_FECSS_BURST_START_OFFSET_TIME                                 (38U)
#define CSL_APPSS_INTR_SW_IRQ0                                                       (39U)
#define CSL_APPSS_INTR_SW_IRQ1                                                       (40U)
#define CSL_APPSS_INTR_SW_IRQ2                                                       (41U)
#define CSL_APPSS_INTR_SW_IRQ3                                                       (42U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_RTI2_INT_REQ0                                (43U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_RTI2_INT_REQ1                                (44U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_RTI2_INT_REQ2                                (45U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_RTI2_INT_REQ3                                (46U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_RTI2_TBINT_AND_GPADC_IFM_DONE                (47U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_CAP_EVT0_AND_RTI2_CAP_EVT0_AND_PWM_INT0      (48U)
#define CSL_APPSS_INTR_MUXED_APPSS_RTI1_CAP_EVT1_AND_RTI2_CAP_EVT1_AND_PWM_INT1      (49U)
#define CSL_APPSS_INTR_HWASS_LOOP_INT                                                (50U)
#define CSL_APPSS_INTR_HWASS_PARAMDONE_INT                                           (51U)
#define CSL_APPSS_INTR_SHA_S_INT                                                     (52U)
#define CSL_APPSS_INTR_SHA_P_INT                                                     (53U)
#define CSL_APPSS_INTR_TRNG_INT                                                      (54U)
#define CSL_APPSS_INTR_PKAE_INT                                                      (55U)
#define CSL_APPSS_INTR_AES_S_INT                                                     (56U)
#define CSL_APPSS_INTR_AES_P_INT                                                     (57U)
#define CSL_APPSS_INTR_HSM_MB_INT0                                                   (58U)
#define CSL_APPSS_INTR_HSM_MB_INT1                                                   (59U)
#define CSL_APPSS_INTR_APPSS_MUXED_PERIPH_ACCESS_ERRAGG                              (60U)
#define CSL_APPSS_INTR_APPSS_WKUP_INTR                                               (61U)
#define CSL_APPSS_INTR_APPSS_SCI2_INT0                                               (62U)
#define CSL_APPSS_INTR_APPSS_SCI2_INT1                                               (63U)

#ifdef __cplusplus
}
#endif
#endif /* CSLR_INTR_H_*/
