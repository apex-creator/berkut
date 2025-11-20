/*!*****************************************************************************
 * @file reg_fecctrl.h
 *
 * @brief This file gives register definitions of FEC_CTRL module
 *
 * @b Description @n
 * This file gives register definitions of FEC_CTRL module
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
#ifndef REG_FECCTRL_H
#define REG_FECCTRL_H

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
} U_FECCTRL_PID;

/*------------------------------------------------------------------------------
 * FESS_CTRL0 -- Offset == 0x004
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FessCtrl0                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL0;

/*------------------------------------------------------------------------------
 * FESS_CTRL1 -- Offset == 0x008
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FessCtrl1                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL1;

/*------------------------------------------------------------------------------
 * FESS_CTRL2 -- Offset == 0x00C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FessCtrl2                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL2;

/*------------------------------------------------------------------------------
 * FESS_CTRL3 -- Offset == 0x010
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FessCtrl3                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL3;

/*------------------------------------------------------------------------------
 * FECSS_DBG_ACK_CTRL -- Offset == 0x014
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Rti                        :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_DBG_ACK_CTRL;

/*------------------------------------------------------------------------------
 * FECSS_CM3_ROM_ECLIPSE -- Offset == 0x018
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_Memswap                    :  3;           /*!< bits   2:  0 */
        UINT32 b3_MemswapWait                :  3;           /*!< bits   5:  3 */
        UINT32 b26_Padfield_0                : 26;           /*!< bits  31:  6 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_CM3_ROM_ECLIPSE;

/*------------------------------------------------------------------------------
 * FECSS_CM3_STATUS_REG -- Offset == 0x01C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Memswap                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_CM3_STATUS_REG;

/*------------------------------------------------------------------------------
 * FECSS_SHARED_MEM_INIT -- Offset == 0x020
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInit                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHARED_MEM_INIT;

/*------------------------------------------------------------------------------
 * FECSS_SHARED_MEM_DONE -- Offset == 0x024
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitDone                :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHARED_MEM_DONE;

/*------------------------------------------------------------------------------
 * FECSS_SHARED_MEM_STATUS -- Offset == 0x028
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitStatus              :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHARED_MEM_STATUS;

/*------------------------------------------------------------------------------
 * FECSS_SHARED_MEM_CLK_GATE -- Offset == 0x02C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_HwaEnable                  :  1;           /*!< bits   0:  0 */
        UINT32 b1_FecEnable                  :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHARED_MEM_CLK_GATE;

/*------------------------------------------------------------------------------
 * FECSS_MEM_INIT_SLICE_SEL -- Offset == 0x030
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_CfgMemInitSliceSel         :  2;           /*!< bits   1:  0 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_MEM_INIT_SLICE_SEL;

/*------------------------------------------------------------------------------
 * FECSS_RAM_MEM_INIT -- Offset == 0x034
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInit                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_RAM_MEM_INIT;

/*------------------------------------------------------------------------------
 * FECSS_RAM_MEM_DONE -- Offset == 0x038
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitDone                :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_RAM_MEM_DONE;

/*------------------------------------------------------------------------------
 * FECSS_RAM_MEM_STATUS -- Offset == 0x03C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitStatus              :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_RAM_MEM_STATUS;

/*------------------------------------------------------------------------------
 * FECSS_GPADC_MEM_INIT -- Offset == 0x040
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInit                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_GPADC_MEM_INIT;

/*------------------------------------------------------------------------------
 * FECSS_GPADC_MEM_DONE -- Offset == 0x044
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitDone                :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_GPADC_MEM_DONE;

/*------------------------------------------------------------------------------
 * FECSS_GPADC_MEM_STATUS -- Offset == 0x048
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitStatus              :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_GPADC_MEM_STATUS;

/*------------------------------------------------------------------------------
 * FECSS_TIMING_ENGINE_MEM_INIT -- Offset == 0x04C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInit                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_TIMING_ENGINE_MEM_INIT;

/*------------------------------------------------------------------------------
 * FECSS_TIMING_ENGINE_MEM_DONE -- Offset == 0x050
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitDone                :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_TIMING_ENGINE_MEM_DONE;

/*------------------------------------------------------------------------------
 * FECSS_TIMING_ENGINE_MEM_STATUS -- Offset == 0x054
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemInitStatus              :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_TIMING_ENGINE_MEM_STATUS;

/*------------------------------------------------------------------------------
 * FECSS_SW_INT -- Offset == 0x058
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Pulse                      :  4;           /*!< bits   3:  0 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SW_INT;

/*------------------------------------------------------------------------------
 * FECSS_IPC_RFS -- Offset == 0x05C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_FecIntr                    :  4;           /*!< bits   3:  0 */
        UINT32 b28_Response                  : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IPC_RFS;

/*------------------------------------------------------------------------------
 * FECSS_IPC_BUSY_INT0 -- Offset == 0x060
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Flag                       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IPC_BUSY_INT0;

/*------------------------------------------------------------------------------
 * FECSS_IPC_BUSY_INT1 -- Offset == 0x064
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Flag                       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IPC_BUSY_INT1;

/*------------------------------------------------------------------------------
 * FECSS_IPC_BUSY_INT2 -- Offset == 0x068
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Flag                       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IPC_BUSY_INT2;

/*------------------------------------------------------------------------------
 * FECSS_IPC_BUSY_INT3 -- Offset == 0x06C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Flag                       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IPC_BUSY_INT3;

/*------------------------------------------------------------------------------
 * FECSS_WIC_STAT_CLR -- Offset == 0x070
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FecssWicStatClr           : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_WIC_STAT_CLR;

/*------------------------------------------------------------------------------
 * FECSS_WIC_STAT -- Offset == 0x074
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FecssWicStat              : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_WIC_STAT;

/*------------------------------------------------------------------------------
 * FECSS_WIC_MASK -- Offset == 0x078
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FecssWicMask              : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_WIC_MASK;

/*------------------------------------------------------------------------------
 * FECSS_WICEN -- Offset == 0x07C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecssWicen                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_WICEN;

/*------------------------------------------------------------------------------
 * FECSS_FORCEFCLKACTIVE -- Offset == 0x080
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecssForcefclkactive       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_FORCEFCLKACTIVE;

/*------------------------------------------------------------------------------
 * FECSS_IRQ_REQ -- Offset == 0x084
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Select                    : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_IRQ_REQ;

/*------------------------------------------------------------------------------
 * FECSS_ERRAGG_MASK0 -- Offset == 0x088
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecRcmRd                   :  1;           /*!< bits   0:  0 */
        UINT32 b1_FecRcmWr                   :  1;           /*!< bits   1:  1 */
        UINT32 b1_RadarCfgRd                 :  1;           /*!< bits   2:  2 */
        UINT32 b1_RadarCfgWr                 :  1;           /*!< bits   3:  3 */
        UINT32 b1_TimingEngineRd             :  1;           /*!< bits   4:  4 */
        UINT32 b1_TimingEngineWr             :  1;           /*!< bits   5:  5 */
        UINT32 b1_FecCtrlRd                  :  1;           /*!< bits   6:  6 */
        UINT32 b1_FecCtrlWr                  :  1;           /*!< bits   7:  7 */
        UINT32 b1_FecAnaRd                   :  1;           /*!< bits   8:  8 */
        UINT32 b1_FecAnaWr                   :  1;           /*!< bits   9:  9 */
        UINT32 b1_FecMpuRd                   :  1;           /*!< bits  10: 10 */
        UINT32 b1_FecAhbWr                   :  1;           /*!< bits  11: 11 */
        UINT32 b1_FecSharedMemErr            :  1;           /*!< bits  12: 12 */
        UINT32 b19_Padfield_0                : 19;           /*!< bits  31: 13 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_ERRAGG_MASK0;

/*------------------------------------------------------------------------------
 * FECSS_ERRAGG_STATUS0 -- Offset == 0x08C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecRcmRd                   :  1;           /*!< bits   0:  0 */
        UINT32 b1_FecRcmWr                   :  1;           /*!< bits   1:  1 */
        UINT32 b1_RadarCfgRd                 :  1;           /*!< bits   2:  2 */
        UINT32 b1_RadarCfgWr                 :  1;           /*!< bits   3:  3 */
        UINT32 b1_TimingEngineRd             :  1;           /*!< bits   4:  4 */
        UINT32 b1_TimingEngineWr             :  1;           /*!< bits   5:  5 */
        UINT32 b1_FecCtrlRd                  :  1;           /*!< bits   6:  6 */
        UINT32 b1_FecCtrlWr                  :  1;           /*!< bits   7:  7 */
        UINT32 b1_FecAnaRd                   :  1;           /*!< bits   8:  8 */
        UINT32 b1_FecAnaWr                   :  1;           /*!< bits   9:  9 */
        UINT32 b1_FecMpuRd                   :  1;           /*!< bits  10: 10 */
        UINT32 b1_FecAhbWr                   :  1;           /*!< bits  11: 11 */
        UINT32 b1_FecSharedMemErr            :  1;           /*!< bits  12: 12 */
        UINT32 b19_Padfield_0                : 19;           /*!< bits  31: 13 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_ERRAGG_STATUS0;

/*------------------------------------------------------------------------------
 * FECSS_ERRAGG_MASK1 -- Offset == 0x090
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Set                       : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_ERRAGG_MASK1;

/*------------------------------------------------------------------------------
 * FECSS_ERRAGG_STATUS1 -- Offset == 0x094
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Set                       : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_ERRAGG_STATUS1;

/*------------------------------------------------------------------------------
 * FECSS_INTMASK1 -- Offset == 0x098
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Set                       : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_INTMASK1;

/*------------------------------------------------------------------------------
 * FECSS_INTMASK2 -- Offset == 0x09C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Set                       : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_INTMASK2;

/*------------------------------------------------------------------------------
 * CFG_TIMEOUT_PCR1 -- Offset == 0x0A0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_CFG_TIMEOUT_PCR1;

/*------------------------------------------------------------------------------
 * FESS_CTRL37 -- Offset == 0x0A4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FessCtrl37                : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL37;

/*------------------------------------------------------------------------------
 * FESS_CTRL38 -- Offset == 0x0A8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl38                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL38;

/*------------------------------------------------------------------------------
 * FESS_CTRL39 -- Offset == 0x0AC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl39                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL39;

/*------------------------------------------------------------------------------
 * FESS_CTRL40 -- Offset == 0x0B0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl40                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL40;

/*------------------------------------------------------------------------------
 * FESS_CTRL41 -- Offset == 0x0B4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl41                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL41;

/*------------------------------------------------------------------------------
 * FESS_CTRL42 -- Offset == 0x0B8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl42                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL42;

/*------------------------------------------------------------------------------
 * FESS_CTRL43 -- Offset == 0x0BC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl43                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL43;

/*------------------------------------------------------------------------------
 * FESS_CTRL44 -- Offset == 0x0C0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl44                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL44;

/*------------------------------------------------------------------------------
 * FESS_CTRL45 -- Offset == 0x0C4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl45                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL45;

/*------------------------------------------------------------------------------
 * FESS_CTRL46 -- Offset == 0x0C8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl46                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL46;

/*------------------------------------------------------------------------------
 * FESS_CTRL47 -- Offset == 0x0CC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_Padfield_0                : 16;           /*!< bits  15:  0 */
        UINT32 b1_FessCtrl47                 :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL47;

/*------------------------------------------------------------------------------
 * FESS_CTRL48 -- Offset == 0x0D0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl48                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL48;

/*------------------------------------------------------------------------------
 * FESS_CTRL49 -- Offset == 0x0D4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_Padfield_0                : 16;           /*!< bits  15:  0 */
        UINT32 b1_FessCtrl49                 :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL49;

/*------------------------------------------------------------------------------
 * FESS_CTRL50 -- Offset == 0x0D8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FessCtrl50                 :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FESS_CTRL50;

/*------------------------------------------------------------------------------
 * RESERVED0 -- Offset == 0x0DC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_Reserved                  : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_RESERVED0;

/*------------------------------------------------------------------------------
 * APPSS_DYNAMIC_CLK_GATE_STATUS -- Offset == 0x0E0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Xbara                      :  1;           /*!< bits   0:  0 */
        UINT32 b1_Tptc1                      :  1;           /*!< bits   1:  1 */
        UINT32 b1_Tptc2                      :  1;           /*!< bits   2:  2 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_APPSS_DYNAMIC_CLK_GATE_STATUS;

/*------------------------------------------------------------------------------
 * FECSS_RAM_OWRITE_ERR -- Offset == 0x0E4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Err                        :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_RAM_OWRITE_ERR;

/*------------------------------------------------------------------------------
 * FECSS_RAM_OWRITE_ERR_ADDR -- Offset == 0x0E8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Address                   : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_RAM_OWRITE_ERR_ADDR;

/*------------------------------------------------------------------------------
 * FECSS_SHRD_RAM_OWRITE_ERR -- Offset == 0x0EC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Err                        :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHRD_RAM_OWRITE_ERR;

/*------------------------------------------------------------------------------
 * FECSS_SHRD_RAM_OWRITE_ERR_ADDR -- Offset == 0x0F0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Address                   : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_SHRD_RAM_OWRITE_ERR_ADDR;

/*------------------------------------------------------------------------------
 * FECSS_OWRITE_ERR_AGGR -- Offset == 0x0F4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Err                        :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_FECSS_OWRITE_ERR_AGGR;

/*------------------------------------------------------------------------------
 * HW_SPARE_RW0 -- Offset == 0x0F8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareRw0                : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_HW_SPARE_RW0;

/*------------------------------------------------------------------------------
 * HW_SPARE_RW1 -- Offset == 0x0FC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareRw1                : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_HW_SPARE_RW1;

/*------------------------------------------------------------------------------
 * CM3_CPU_HALT_HANDSHAKE -- Offset == 0x100
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FECCTRL_CM3_CPU_HALT_HANDSHAKE;

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
} U_FECCTRL_LOCK0_KICK0;

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
} U_FECCTRL_LOCK0_KICK1;

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
} U_FECCTRL_INTR_RAW_STATUS;

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
} U_FECCTRL_INTR_ENABLED_STATUS_CLEAR;

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
} U_FECCTRL_INTR_ENABLE;

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
} U_FECCTRL_INTR_ENABLE_CLEAR;

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
} U_FECCTRL_EOI;

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
} U_FECCTRL_FAULT_ADDRESS;

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
} U_FECCTRL_FAULT_TYPE_STATUS;

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
} U_FECCTRL_FAULT_ATTR_STATUS;

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
} U_FECCTRL_FAULT_CLEAR;



/*------------------------------------------------------------------------------
 * FEC_CTRL_REGS
 *------------------------------------------------------------------------------
 */
typedef volatile struct
{
    U_FECCTRL_PID                              r_Pid                         ; /* Offset=0x0000 */
    U_FECCTRL_FESS_CTRL0                       r_FessCtrl0                   ; /* Offset=0x0004 */
    U_FECCTRL_FESS_CTRL1                       r_FessCtrl1                   ; /* Offset=0x0008 */
    U_FECCTRL_FESS_CTRL2                       r_FessCtrl2                   ; /* Offset=0x000C */
    U_FECCTRL_FESS_CTRL3                       r_FessCtrl3                   ; /* Offset=0x0010 */
    U_FECCTRL_FECSS_DBG_ACK_CTRL               r_FecssDbgAckCtrl             ; /* Offset=0x0014 */
    U_FECCTRL_FECSS_CM3_ROM_ECLIPSE            r_FecssCm3RomEclipse          ; /* Offset=0x0018 */
    U_FECCTRL_FECSS_CM3_STATUS_REG             r_FecssCm3StatusReg           ; /* Offset=0x001C */
    U_FECCTRL_FECSS_SHARED_MEM_INIT            r_FecssSharedMemInit          ; /* Offset=0x0020 */
    U_FECCTRL_FECSS_SHARED_MEM_DONE            r_FecssSharedMemDone          ; /* Offset=0x0024 */
    U_FECCTRL_FECSS_SHARED_MEM_STATUS          r_FecssSharedMemStatus        ; /* Offset=0x0028 */
    U_FECCTRL_FECSS_SHARED_MEM_CLK_GATE        r_FecssSharedMemClkGate       ; /* Offset=0x002C */
    U_FECCTRL_FECSS_MEM_INIT_SLICE_SEL         r_FecssMemInitSliceSel        ; /* Offset=0x0030 */
    U_FECCTRL_FECSS_RAM_MEM_INIT               r_FecssRamMemInit             ; /* Offset=0x0034 */
    U_FECCTRL_FECSS_RAM_MEM_DONE               r_FecssRamMemDone             ; /* Offset=0x0038 */
    U_FECCTRL_FECSS_RAM_MEM_STATUS             r_FecssRamMemStatus           ; /* Offset=0x003C */
    U_FECCTRL_FECSS_GPADC_MEM_INIT             r_FecssGpadcMemInit           ; /* Offset=0x0040 */
    U_FECCTRL_FECSS_GPADC_MEM_DONE             r_FecssGpadcMemDone           ; /* Offset=0x0044 */
    U_FECCTRL_FECSS_GPADC_MEM_STATUS           r_FecssGpadcMemStatus         ; /* Offset=0x0048 */
    U_FECCTRL_FECSS_TIMING_ENGINE_MEM_INIT     r_FecssTimingEngineMemInit    ; /* Offset=0x004C */
    U_FECCTRL_FECSS_TIMING_ENGINE_MEM_DONE     r_FecssTimingEngineMemDone    ; /* Offset=0x0050 */
    U_FECCTRL_FECSS_TIMING_ENGINE_MEM_STATUS   r_FecssTimingEngineMemStatus  ; /* Offset=0x0054 */
    U_FECCTRL_FECSS_SW_INT                     r_FecssSwInt                  ; /* Offset=0x0058 */
    U_FECCTRL_FECSS_IPC_RFS                    r_FecssIpcRfs                 ; /* Offset=0x005C */
    U_FECCTRL_FECSS_IPC_BUSY_INT0              r_FecssIpcBusyInt0            ; /* Offset=0x0060 */
    U_FECCTRL_FECSS_IPC_BUSY_INT1              r_FecssIpcBusyInt1            ; /* Offset=0x0064 */
    U_FECCTRL_FECSS_IPC_BUSY_INT2              r_FecssIpcBusyInt2            ; /* Offset=0x0068 */
    U_FECCTRL_FECSS_IPC_BUSY_INT3              r_FecssIpcBusyInt3            ; /* Offset=0x006C */
    U_FECCTRL_FECSS_WIC_STAT_CLR               r_FecssWicStatClr             ; /* Offset=0x0070 */
    U_FECCTRL_FECSS_WIC_STAT                   r_FecssWicStat                ; /* Offset=0x0074 */
    U_FECCTRL_FECSS_WIC_MASK                   r_FecssWicMask                ; /* Offset=0x0078 */
    U_FECCTRL_FECSS_WICEN                      r_FecssWicen                  ; /* Offset=0x007C */
    U_FECCTRL_FECSS_FORCEFCLKACTIVE            r_FecssForcefclkactive        ; /* Offset=0x0080 */
    U_FECCTRL_FECSS_IRQ_REQ                    r_FecssIrqReq                 ; /* Offset=0x0084 */
    U_FECCTRL_FECSS_ERRAGG_MASK0               r_FecssErraggMask0            ; /* Offset=0x0088 */
    U_FECCTRL_FECSS_ERRAGG_STATUS0             r_FecssErraggStatus0          ; /* Offset=0x008C */
    U_FECCTRL_FECSS_ERRAGG_MASK1               r_FecssErraggMask1            ; /* Offset=0x0090 */
    U_FECCTRL_FECSS_ERRAGG_STATUS1             r_FecssErraggStatus1          ; /* Offset=0x0094 */
    U_FECCTRL_FECSS_INTMASK1                   r_FecssIntmask1               ; /* Offset=0x0098 */
    U_FECCTRL_FECSS_INTMASK2                   r_FecssIntmask2               ; /* Offset=0x009C */
    U_FECCTRL_CFG_TIMEOUT_PCR1                 r_CfgTimeoutPcr1              ; /* Offset=0x00A0 */
    U_FECCTRL_FESS_CTRL37                      r_FessCtrl37                  ; /* Offset=0x00A4 */
    U_FECCTRL_FESS_CTRL38                      r_FessCtrl38                  ; /* Offset=0x00A8 */
    U_FECCTRL_FESS_CTRL39                      r_FessCtrl39                  ; /* Offset=0x00AC */
    U_FECCTRL_FESS_CTRL40                      r_FessCtrl40                  ; /* Offset=0x00B0 */
    U_FECCTRL_FESS_CTRL41                      r_FessCtrl41                  ; /* Offset=0x00B4 */
    U_FECCTRL_FESS_CTRL42                      r_FessCtrl42                  ; /* Offset=0x00B8 */
    U_FECCTRL_FESS_CTRL43                      r_FessCtrl43                  ; /* Offset=0x00BC */
    U_FECCTRL_FESS_CTRL44                      r_FessCtrl44                  ; /* Offset=0x00C0 */
    U_FECCTRL_FESS_CTRL45                      r_FessCtrl45                  ; /* Offset=0x00C4 */
    U_FECCTRL_FESS_CTRL46                      r_FessCtrl46                  ; /* Offset=0x00C8 */
    U_FECCTRL_FESS_CTRL47                      r_FessCtrl47                  ; /* Offset=0x00CC */
    U_FECCTRL_FESS_CTRL48                      r_FessCtrl48                  ; /* Offset=0x00D0 */
    U_FECCTRL_FESS_CTRL49                      r_FessCtrl49                  ; /* Offset=0x00D4 */
    U_FECCTRL_FESS_CTRL50                      r_FessCtrl50                  ; /* Offset=0x00D8 */
    U_FECCTRL_RESERVED0                        r_Reserved0                   ; /* Offset=0x00DC */
    U_FECCTRL_APPSS_DYNAMIC_CLK_GATE_STATUS    r_AppssDynamicClkGateStatus   ; /* Offset=0x00E0 */
    U_FECCTRL_FECSS_RAM_OWRITE_ERR             r_FecssRamOwriteErr           ; /* Offset=0x00E4 */
    U_FECCTRL_FECSS_RAM_OWRITE_ERR_ADDR        r_FecssRamOwriteErrAddr       ; /* Offset=0x00E8 */
    U_FECCTRL_FECSS_SHRD_RAM_OWRITE_ERR        r_FecssShrdRamOwriteErr       ; /* Offset=0x00EC */
    U_FECCTRL_FECSS_SHRD_RAM_OWRITE_ERR_ADDR   r_FecssShrdRamOwriteErrAddr   ; /* Offset=0x00F0 */
    U_FECCTRL_FECSS_OWRITE_ERR_AGGR            r_FecssOwriteErrAggr          ; /* Offset=0x00F4 */
    U_FECCTRL_HW_SPARE_RW0                     r_HwSpareRw0                  ; /* Offset=0x00F8 */
    U_FECCTRL_HW_SPARE_RW1                     r_HwSpareRw1                  ; /* Offset=0x00FC */
    U_FECCTRL_CM3_CPU_HALT_HANDSHAKE           r_Cm3CpuHaltHandshake         ; /* Offset=0x0100 */
    UINT32                                     r_Padreg0[961]                ; /* Offset=0x0104 */
    U_FECCTRL_LOCK0_KICK0                      r_Lock0Kick0                  ; /* Offset=0x1008 */
    U_FECCTRL_LOCK0_KICK1                      r_Lock0Kick1                  ; /* Offset=0x100C */
    U_FECCTRL_INTR_RAW_STATUS                  r_IntrRawStatus               ; /* Offset=0x1010 */
    U_FECCTRL_INTR_ENABLED_STATUS_CLEAR        r_IntrEnabledStatusClear      ; /* Offset=0x1014 */
    U_FECCTRL_INTR_ENABLE                      r_IntrEnable                  ; /* Offset=0x1018 */
    U_FECCTRL_INTR_ENABLE_CLEAR                r_IntrEnableClear             ; /* Offset=0x101C */
    U_FECCTRL_EOI                              r_Eoi                         ; /* Offset=0x1020 */
    U_FECCTRL_FAULT_ADDRESS                    r_FaultAddress                ; /* Offset=0x1024 */
    U_FECCTRL_FAULT_TYPE_STATUS                r_FaultTypeStatus             ; /* Offset=0x1028 */
    U_FECCTRL_FAULT_ATTR_STATUS                r_FaultAttrStatus             ; /* Offset=0x102C */
    U_FECCTRL_FAULT_CLEAR                      r_FaultClear                  ; /* Offset=0x1030 */
} T_FECCTRL_REGS;

#endif
/*
 * END OF REG_FECCTRL_H
 */

