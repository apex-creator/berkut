/*!*****************************************************************************
 * @file reg_fecrcm.h
 *
 * @brief This file gives register definitions of FEC_RCM module
 *
 * @b Description @n
 * This file gives register definitions of FEC_RCM module
 *
 * @note
 * (C) Copyright 2021, Texas Instruments Incorporated. - TI web address www.ti.com
 * @n
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * @n
 * <ul>
 *   <li> Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 * @n
 *   <li> Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * @n
 *   <li> Neither the name of Texas Instruments Incorporated nor the names of
 *        its contributors may be used to endorse or promote products derived
 *        from this software without specific prior written permission.
 * </ul>
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES LOSS OF USE,
 * DATA, OR PROFITS OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date      Author     Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     6Jan2022  TI         NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef REG_FECRCM_H
#define REG_FECRCM_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */


/*------------------------------------------------------------------------------
 * PID -- Offset == 0x000
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b6_Minor                      :  6;           /*!< bits   5:  0 */
        UINT32 b2_Custom                     :  2;           /*!< bits   7:  6 */
        UINT32 b3_Major                      :  3;           /*!< bits  10:  8 */
        UINT32 b5_Misc                       :  5;           /*!< bits  15: 11 */
        UINT32 b16_Msb16                     : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_PID;

/*------------------------------------------------------------------------------
 * FEC_SYS_CLKCTL -- Offset == 0x004
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Padfield_0                 :  4;           /*!< bits   3:  0 */
        UINT32 b12_Srcsel                    : 12;           /*!< bits  15:  4 */
        UINT32 b12_Divr                      : 12;           /*!< bits  27: 16 */
        UINT32 b4_Padfield_1                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_SYS_CLKCTL;

/*------------------------------------------------------------------------------
 * FEC_SYS_CLKSTAT -- Offset == 0x008
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Currdivr                   :  4;           /*!< bits   3:  0 */
        UINT32 b8_Currclk                    :  8;           /*!< bits  11:  4 */
        UINT32 b20_Padfield_0                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_SYS_CLKSTAT;

/*------------------------------------------------------------------------------
 * FEC_RTI_CLKCTL -- Offset == 0x00C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Gate                       :  3;           /*!< bits   2:  0 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits   3:  3 */
        UINT32 b3_Srcsel                     :  3;           /*!< bits   6:  4 */
        UINT32 b9_Padfield_1                 :  9;           /*!< bits  15:  7 */
        UINT32 b12_Divr                      : 12;           /*!< bits  27: 16 */
        UINT32 b4_Padfield_2                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_RTI_CLKCTL;

/*------------------------------------------------------------------------------
 * FEC_RTI_CLKSTAT -- Offset == 0x010
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Currdivr                   :  4;           /*!< bits   3:  0 */
        UINT32 b2_Currclk                    :  2;           /*!< bits   5:  4 */
        UINT32 b26_Padfield_0                : 26;           /*!< bits  31:  6 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_RTI_CLKSTAT;

/*------------------------------------------------------------------------------
 * FEC_GPADC_CLKCTL -- Offset == 0x014
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Gate                       :  3;           /*!< bits   2:  0 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits   3:  3 */
        UINT32 b3_Srcsel                     :  3;           /*!< bits   6:  4 */
        UINT32 b9_Padfield_1                 :  9;           /*!< bits  15:  7 */
        UINT32 b12_Divr                      : 12;           /*!< bits  27: 16 */
        UINT32 b4_Padfield_2                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_GPADC_CLKCTL;

/*------------------------------------------------------------------------------
 * FEC_GPADC_CLKSTAT -- Offset == 0x018
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Currdivr                   :  4;           /*!< bits   3:  0 */
        UINT32 b2_Currclk                    :  2;           /*!< bits   5:  4 */
        UINT32 b26_Padfield_0                : 26;           /*!< bits  31:  6 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_GPADC_CLKSTAT;

/*------------------------------------------------------------------------------
 * FEC_RX_ADC_CLKCTL -- Offset == 0x024
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Gate                       :  3;           /*!< bits   2:  0 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits   3:  3 */
        UINT32 b6_Srcsel                     :  6;           /*!< bits   9:  4 */
        UINT32 b6_Padfield_1                 :  6;           /*!< bits  15: 10 */
        UINT32 b12_Divr                      : 12;           /*!< bits  27: 16 */
        UINT32 b4_Padfield_2                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_RX_ADC_CLKCTL;

/*------------------------------------------------------------------------------
 * FEC_RX_ADC_CLKSTAT -- Offset == 0x028
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Currdivr                   :  4;           /*!< bits   3:  0 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_RX_ADC_CLKSTAT;

/*------------------------------------------------------------------------------
 * FEC_MDIF_SER2X_CLKCTL -- Offset == 0x02C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Padfield_0                 :  4;           /*!< bits   3:  0 */
        UINT32 b12_Srcsel                    : 12;           /*!< bits  15:  4 */
        UINT32 b16_Padfield_1                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_MDIF_SER2X_CLKCTL;

/*------------------------------------------------------------------------------
 * FEC_MDIF_SER2X_CLKSTAT -- Offset == 0x030
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Padfield_0                 :  4;           /*!< bits   3:  0 */
        UINT32 b8_Currclk                    :  8;           /*!< bits  11:  4 */
        UINT32 b20_Padfield_1                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_MDIF_SER2X_CLKSTAT;

/*------------------------------------------------------------------------------
 * IPCFGCLKGATE0 -- Offset == 0x034
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Padfield_0                 :  3;           /*!< bits   2:  0 */
        UINT32 b3_FecRti                     :  3;           /*!< bits   5:  3 */
        UINT32 b3_FecCtrl                    :  3;           /*!< bits   8:  6 */
        UINT32 b9_Padfield_1                 :  9;           /*!< bits  17:  9 */
        UINT32 b3_AnaCfg                     :  3;           /*!< bits  20: 18 */
        UINT32 b3_DfeCfg                     :  3;           /*!< bits  23: 21 */
        UINT32 b3_Rampgen                    :  3;           /*!< bits  26: 24 */
        UINT32 b3_GpadcCtrl                  :  3;           /*!< bits  29: 27 */
        UINT32 b2_Padfield_2                 :  2;           /*!< bits  31: 30 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_IPCFGCLKGATE0;

/*------------------------------------------------------------------------------
 * IPCFGCLKGATE1 -- Offset == 0x038
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_FecFftCfg                  :  3;           /*!< bits   2:  0 */
        UINT32 b3_FecMdif                    :  3;           /*!< bits   5:  3 */
        UINT32 b3_FecDfe                     :  3;           /*!< bits   8:  6 */
        UINT32 b3_FecDfeOutClk               :  3;           /*!< bits  11:  9 */
        UINT32 b3_FecSocc                    :  3;           /*!< bits  14: 12 */
        UINT32 b17_Padfield_0                : 17;           /*!< bits  31: 15 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_IPCFGCLKGATE1;

/*------------------------------------------------------------------------------
 * BLOCKRESET0 -- Offset == 0x03C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_DfeCfg                     :  3;           /*!< bits   2:  0 */
        UINT32 b3_FecPerchirpMemIntrp        :  3;           /*!< bits   5:  3 */
        UINT32 b3_FecChirpTimer              :  3;           /*!< bits   8:  6 */
        UINT32 b3_FecVbusp                   :  3;           /*!< bits  11:  9 */
        UINT32 b3_FecSocc                    :  3;           /*!< bits  14: 12 */
        UINT32 b3_GpadcCtrl                  :  3;           /*!< bits  17: 15 */
        UINT32 b3_FecFftCfg                  :  3;           /*!< bits  20: 18 */
        UINT32 b3_FecMdif                    :  3;           /*!< bits  23: 21 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_BLOCKRESET0;

/*------------------------------------------------------------------------------
 * RST_WFICHECK -- Offset == 0x040
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Cm3                        :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST_WFICHECK;

/*------------------------------------------------------------------------------
 * RST_ASSERTDLY -- Offset == 0x044
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_Common                     :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST_ASSERTDLY;

/*------------------------------------------------------------------------------
 * RST2ASSERTDLY -- Offset == 0x048
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_Padfield_0                 :  8;           /*!< bits   7:  0 */
        UINT32 b8_Cm3                        :  8;           /*!< bits  15:  8 */
        UINT32 b16_Padfield_1                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST2ASSERTDLY;

/*------------------------------------------------------------------------------
 * FEC_CM3_POR_RST_CTRL -- Offset == 0x04C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Cm3PorRstAssert            :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_CM3_POR_RST_CTRL;

/*------------------------------------------------------------------------------
 * FEC_CM3_CFG -- Offset == 0x050
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ForceHclkActive            :  1;           /*!< bits   0:  0 */
        UINT32 b1_SleepHoldReqn              :  1;           /*!< bits   1:  1 */
        UINT32 b1_Wicenreq                   :  1;           /*!< bits   2:  2 */
        UINT32 b3_HclkGate                   :  3;           /*!< bits   5:  3 */
        UINT32 b26_Padfield_0                : 26;           /*!< bits  31:  6 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_CM3_CFG;

/*------------------------------------------------------------------------------
 * FEC_CM3_STATUS -- Offset == 0x054
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Wicenackn                  :  1;           /*!< bits   0:  0 */
        UINT32 b1_SleepHoldAckn              :  1;           /*!< bits   1:  1 */
        UINT32 b1_SysRstn                    :  1;           /*!< bits   2:  2 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_CM3_STATUS;

/*------------------------------------------------------------------------------
 * RST_FSM_TRIG -- Offset == 0x058
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Cm3                        :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST_FSM_TRIG;

/*------------------------------------------------------------------------------
 * RST_CAUSE -- Offset == 0x05C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_Common                     :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST_CAUSE;

/*------------------------------------------------------------------------------
 * RST_CAUSE_CLR -- Offset == 0x060
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Cpu                        :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_RST_CAUSE_CLR;

/*------------------------------------------------------------------------------
 * SYNTH_CLK_DIV2_CTRL -- Offset == 0x064
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Enable                     :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_SYNTH_CLK_DIV2_CTRL;

/*------------------------------------------------------------------------------
 * TEST_PATTERN_CLK_GATE -- Offset == 0x068
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_TestPatternClkGate         :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_TEST_PATTERN_CLK_GATE;

/*------------------------------------------------------------------------------
 * DFT_FECSS_LSTC_CLK_GATE -- Offset == 0x06C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_DftFecssLstcClkGate        :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_DFT_FECSS_LSTC_CLK_GATE;

/*------------------------------------------------------------------------------
 * DFT_FECSS_LSTC_CLK_VBUSP_GATE -- Offset == 0x070
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Enable                     :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_DFT_FECSS_LSTC_CLK_VBUSP_GATE;

/*------------------------------------------------------------------------------
 * FEC_RAM_CLOCK_GATE -- Offset == 0x074
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Enable                     :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_RAM_CLOCK_GATE;

/*------------------------------------------------------------------------------
 * FEC_ROM_CLOCK_GATE -- Offset == 0x078
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Enable                     :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_ROM_CLOCK_GATE;

/*------------------------------------------------------------------------------
 * FEC_CLOCK_GATE_REG1 -- Offset == 0x07C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Enable                    : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_CLOCK_GATE_REG1;

/*------------------------------------------------------------------------------
 * FEC_CLOCK_GATE_REG2 -- Offset == 0x080
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Enable                    : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_CLOCK_GATE_REG2;

/*------------------------------------------------------------------------------
 * FEC_LSTC_EN -- Offset == 0x084
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Enable                     :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FEC_LSTC_EN;

/*------------------------------------------------------------------------------
 * LOCK0_KICK0 -- Offset == 0x1008
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_LOCK0_KICK0;

/*------------------------------------------------------------------------------
 * LOCK0_KICK1 -- Offset == 0x100C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_LOCK0_KICK1;

/*------------------------------------------------------------------------------
 * intr_raw_status -- Offset == 0x1010
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ProtErr                    :  1;           /*!< bits   0:  0 */
        UINT32 b1_AddrErr                    :  1;           /*!< bits   1:  1 */
        UINT32 b1_KickErr                    :  1;           /*!< bits   2:  2 */
        UINT32 b1_ProxyErr                   :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_INTR_RAW_STATUS;

/*------------------------------------------------------------------------------
 * intr_enabled_status_clear -- Offset == 0x1014
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_EnabledProtErr             :  1;           /*!< bits   0:  0 */
        UINT32 b1_EnabledAddrErr             :  1;           /*!< bits   1:  1 */
        UINT32 b1_EnabledKickErr             :  1;           /*!< bits   2:  2 */
        UINT32 b1_EnabledProxyErr            :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_INTR_ENABLED_STATUS_CLEAR;

/*------------------------------------------------------------------------------
 * intr_enable -- Offset == 0x1018
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ProtErrEn                  :  1;           /*!< bits   0:  0 */
        UINT32 b1_AddrErrEn                  :  1;           /*!< bits   1:  1 */
        UINT32 b1_KickErrEn                  :  1;           /*!< bits   2:  2 */
        UINT32 b1_ProxyErrEn                 :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_INTR_ENABLE;

/*------------------------------------------------------------------------------
 * intr_enable_clear -- Offset == 0x101C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ProtErrEnClr               :  1;           /*!< bits   0:  0 */
        UINT32 b1_AddrErrEnClr               :  1;           /*!< bits   1:  1 */
        UINT32 b1_KickErrEnClr               :  1;           /*!< bits   2:  2 */
        UINT32 b1_ProxyErrEnClr              :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_INTR_ENABLE_CLEAR;

/*------------------------------------------------------------------------------
 * eoi -- Offset == 0x1020
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_Vector                     :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_EOI;

/*------------------------------------------------------------------------------
 * fault_address -- Offset == 0x1024
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FaultAddr                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FAULT_ADDRESS;

/*------------------------------------------------------------------------------
 * fault_type_status -- Offset == 0x1028
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b6_FaultType                  :  6;           /*!< bits   5:  0 */
        UINT32 b1_FaultNs                    :  1;           /*!< bits   6:  6 */
        UINT32 b25_Padfield_0                : 25;           /*!< bits  31:  7 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FAULT_TYPE_STATUS;

/*------------------------------------------------------------------------------
 * fault_attr_status -- Offset == 0x102C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_FaultPrivid                :  8;           /*!< bits   7:  0 */
        UINT32 b12_FaultRouteid              : 12;           /*!< bits  19:  8 */
        UINT32 b12_FaultXid                  : 12;           /*!< bits  31: 20 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FAULT_ATTR_STATUS;

/*------------------------------------------------------------------------------
 * fault_clear -- Offset == 0x1030
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FaultClr                   :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECRCM_FAULT_CLEAR;



/*------------------------------------------------------------------------------
 * FEC_RCM_REGS
 *------------------------------------------------------------------------------
 */
typedef volatile struct
{
    U_FECRCM_PID                               r_Pid                         ; /* Offset=0x0000 */
    U_FECRCM_FEC_SYS_CLKCTL                    r_FecSysClkctl                ; /* Offset=0x0004 */
    U_FECRCM_FEC_SYS_CLKSTAT                   r_FecSysClkstat               ; /* Offset=0x0008 */
    U_FECRCM_FEC_RTI_CLKCTL                    r_FecRtiClkctl                ; /* Offset=0x000C */
    U_FECRCM_FEC_RTI_CLKSTAT                   r_FecRtiClkstat               ; /* Offset=0x0010 */
    U_FECRCM_FEC_GPADC_CLKCTL                  r_FecGpadcClkctl              ; /* Offset=0x0014 */
    U_FECRCM_FEC_GPADC_CLKSTAT                 r_FecGpadcClkstat             ; /* Offset=0x0018 */
    UINT32                                     r_Padreg0[2]                  ; /* Offset=0x001C */
    U_FECRCM_FEC_RX_ADC_CLKCTL                 r_FecRxAdcClkctl              ; /* Offset=0x0024 */
    U_FECRCM_FEC_RX_ADC_CLKSTAT                r_FecRxAdcClkstat             ; /* Offset=0x0028 */
    U_FECRCM_FEC_MDIF_SER2X_CLKCTL             r_FecMdifSer2xClkctl          ; /* Offset=0x002C */
    U_FECRCM_FEC_MDIF_SER2X_CLKSTAT            r_FecMdifSer2xClkstat         ; /* Offset=0x0030 */
    U_FECRCM_IPCFGCLKGATE0                     r_Ipcfgclkgate0               ; /* Offset=0x0034 */
    U_FECRCM_IPCFGCLKGATE1                     r_Ipcfgclkgate1               ; /* Offset=0x0038 */
    U_FECRCM_BLOCKRESET0                       r_Blockreset0                 ; /* Offset=0x003C */
    U_FECRCM_RST_WFICHECK                      r_RstWficheck                 ; /* Offset=0x0040 */
    U_FECRCM_RST_ASSERTDLY                     r_RstAssertdly                ; /* Offset=0x0044 */
    U_FECRCM_RST2ASSERTDLY                     r_Rst2assertdly               ; /* Offset=0x0048 */
    U_FECRCM_FEC_CM3_POR_RST_CTRL              r_FecCm3PorRstCtrl            ; /* Offset=0x004C */
    U_FECRCM_FEC_CM3_CFG                       r_FecCm3Cfg                   ; /* Offset=0x0050 */
    U_FECRCM_FEC_CM3_STATUS                    r_FecCm3Status                ; /* Offset=0x0054 */
    U_FECRCM_RST_FSM_TRIG                      r_RstFsmTrig                  ; /* Offset=0x0058 */
    U_FECRCM_RST_CAUSE                         r_RstCause                    ; /* Offset=0x005C */
    U_FECRCM_RST_CAUSE_CLR                     r_RstCauseClr                 ; /* Offset=0x0060 */
    U_FECRCM_SYNTH_CLK_DIV2_CTRL               r_SynthClkDiv2Ctrl            ; /* Offset=0x0064 */
    U_FECRCM_TEST_PATTERN_CLK_GATE             r_TestPatternClkGate          ; /* Offset=0x0068 */
    U_FECRCM_DFT_FECSS_LSTC_CLK_GATE           r_DftFecssLstcClkGate         ; /* Offset=0x006C */
    U_FECRCM_DFT_FECSS_LSTC_CLK_VBUSP_GATE     r_DftFecssLstcClkVbuspGate    ; /* Offset=0x0070 */
    U_FECRCM_FEC_RAM_CLOCK_GATE                r_FecRamClockGate             ; /* Offset=0x0074 */
    U_FECRCM_FEC_ROM_CLOCK_GATE                r_FecRomClockGate             ; /* Offset=0x0078 */
    U_FECRCM_FEC_CLOCK_GATE_REG1               r_FecClockGateReg1            ; /* Offset=0x007C */
    U_FECRCM_FEC_CLOCK_GATE_REG2               r_FecClockGateReg2            ; /* Offset=0x0080 */
    U_FECRCM_FEC_LSTC_EN                       r_FecLstcEn                   ; /* Offset=0x0084 */
    UINT32                                     r_Padreg1[992]                ; /* Offset=0x0088 */
    U_FECRCM_LOCK0_KICK0                       r_Lock0Kick0                  ; /* Offset=0x1008 */
    U_FECRCM_LOCK0_KICK1                       r_Lock0Kick1                  ; /* Offset=0x100C */
    U_FECRCM_INTR_RAW_STATUS                   r_IntrRawStatus               ; /* Offset=0x1010 */
    U_FECRCM_INTR_ENABLED_STATUS_CLEAR         r_IntrEnabledStatusClear      ; /* Offset=0x1014 */
    U_FECRCM_INTR_ENABLE                       r_IntrEnable                  ; /* Offset=0x1018 */
    U_FECRCM_INTR_ENABLE_CLEAR                 r_IntrEnableClear             ; /* Offset=0x101C */
    U_FECRCM_EOI                               r_Eoi                         ; /* Offset=0x1020 */
    U_FECRCM_FAULT_ADDRESS                     r_FaultAddress                ; /* Offset=0x1024 */
    U_FECRCM_FAULT_TYPE_STATUS                 r_FaultTypeStatus             ; /* Offset=0x1028 */
    U_FECRCM_FAULT_ATTR_STATUS                 r_FaultAttrStatus             ; /* Offset=0x102C */
    U_FECRCM_FAULT_CLEAR                       r_FaultClear                  ; /* Offset=0x1030 */
} T_FECRCM_REGS;


#endif
/*
 * END OF REG_FECRCM_H
 */

