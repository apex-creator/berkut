/*!*****************************************************************************
 * @file reg_frametimer.h
 *
 * @brief This file gives register definitions of FRAME_TIMER module
 *
 * @b Description @n
 * This file gives register definitions of FRAME_TIMER module
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
 * 0.1     11Aug2021 TI         NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef REG_FT_H
#define REG_FT_H

/****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************
 */


/*---------------------------------------------------------------------------------------
 * PID -- Offset == 0x000
 *---------------------------------------------------------------------------------------
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
} U_FT_PID;

/*---------------------------------------------------------------------------------------
 * NUM_FRAMES -- Offset == 0x004
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_NumFrames                 : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_NUM_FRAMES;

/*---------------------------------------------------------------------------------------
 * FRAME_PERIODICITY -- Offset == 0x008
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FramePeriodicity          : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_PERIODICITY;

/*---------------------------------------------------------------------------------------
 * FRAME_TRIGGERING_REGS -- Offset == 0x00C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_FrameTrigMode              :  2;           /*!< bits   1:  0 */
        UINT32 b1_LoopStartFrameToDigSync    :  1;           /*!< bits   2:  2 */
        UINT32 b3_FrameTrigReg               :  3;           /*!< bits   5:  3 */
        UINT32 b3_FrameStopReg               :  3;           /*!< bits   8:  6 */
        UINT32 b23_Padfield_0                : 23;           /*!< bits  31:  9 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_TRIGGERING_REGS;

/*---------------------------------------------------------------------------------------
 * FRAME_TRIG_TIMER_VAL -- Offset == 0x010
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FrameTrigTimerVal         : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_TRIG_TIMER_VAL;

/*---------------------------------------------------------------------------------------
 * FRAME_START_OFFSET_INTR_TIME_1 -- Offset == 0x014
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FrameStartOffsetIntrTime_1 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_START_OFFSET_INTR_TIME_1;

/*---------------------------------------------------------------------------------------
 * FRAME_START_OFFSET_INTR_TIME_2 -- Offset == 0x018
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FrameStartOffsetIntrTime_2 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_START_OFFSET_INTR_TIME_2;

/*---------------------------------------------------------------------------------------
 * FRAME_START_OFFSET_INTR_TIME_3 -- Offset == 0x01C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FrameStartOffsetIntrTime_3 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_START_OFFSET_INTR_TIME_3;

/*---------------------------------------------------------------------------------------
 * FRAME_REF_TIMER -- Offset == 0x020
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FrameRefTimer             : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_REF_TIMER;

/*---------------------------------------------------------------------------------------
 * FRAME_PERIOD_TIMER -- Offset == 0x024
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FramePeriodTimer          : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_PERIOD_TIMER;

/*---------------------------------------------------------------------------------------
 * FRAME_CNT -- Offset == 0x028
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_FrameCnt                  : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FRAME_CNT;

/*---------------------------------------------------------------------------------------
 * FRAME_TIMER_TRIG_STOP_DONE -- Offset == 0x02C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FrameTimerTrigDone         :  1;           /*!< bits   0:  0 */
        UINT32 b1_FrameTimerStopDone         :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_TRIG_STOP_DONE;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG0 -- Offset == 0x030
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg0               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG0;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG1 -- Offset == 0x034
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg1               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG1;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG2 -- Offset == 0x038
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg2               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG2;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG3 -- Offset == 0x03C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg3               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG3;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG4 -- Offset == 0x040
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg4               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG4;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG5 -- Offset == 0x044
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg5               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG5;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG6 -- Offset == 0x048
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg6               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG6;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG7 -- Offset == 0x04C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg7               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG7;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG8 -- Offset == 0x050
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg8               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG8;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG9 -- Offset == 0x054
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg9               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_HW_SPARE_REG9;

/*---------------------------------------------------------------------------------------
 * MASK_ERROR_REG -- Offset == 0x058
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FrameTimerUndefinedStateMask :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_MASK_ERROR_REG;

/*---------------------------------------------------------------------------------------
 * ERROR_STATUS_REG -- Offset == 0x05C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FrameTimerUndefinedStateStatus :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_ERROR_STATUS_REG;

/*---------------------------------------------------------------------------------------
 * LOCK0_KICK0 -- Offset == 0x1008
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_LOCK0_KICK0;

/*---------------------------------------------------------------------------------------
 * LOCK0_KICK1 -- Offset == 0x100C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Value                     : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_LOCK0_KICK1;

/*---------------------------------------------------------------------------------------
 * intr_raw_status -- Offset == 0x1010
 *---------------------------------------------------------------------------------------
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
} U_FT_INTR_RAW_STATUS;

/*---------------------------------------------------------------------------------------
 * intr_enabled_status_clear -- Offset == 0x1014
 *---------------------------------------------------------------------------------------
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
} U_FT_INTR_ENABLED_STATUS_CLEAR;

/*---------------------------------------------------------------------------------------
 * intr_enable -- Offset == 0x1018
 *---------------------------------------------------------------------------------------
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
} U_FT_INTR_ENABLE;

/*---------------------------------------------------------------------------------------
 * intr_enable_clear -- Offset == 0x101C
 *---------------------------------------------------------------------------------------
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
} U_FT_INTR_ENABLE_CLEAR;

/*---------------------------------------------------------------------------------------
 * eoi -- Offset == 0x1020
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_Vector                     :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_EOI;

/*---------------------------------------------------------------------------------------
 * fault_address -- Offset == 0x1024
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_FaultAddr                 : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FAULT_ADDRESS;

/*---------------------------------------------------------------------------------------
 * fault_type_status -- Offset == 0x1028
 *---------------------------------------------------------------------------------------
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
} U_FT_FAULT_TYPE_STATUS;

/*---------------------------------------------------------------------------------------
 * fault_attr_status -- Offset == 0x102C
 *---------------------------------------------------------------------------------------
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
} U_FT_FAULT_ATTR_STATUS;

/*---------------------------------------------------------------------------------------
 * fault_clear -- Offset == 0x1030
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FaultClr                   :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FT_FAULT_CLEAR;



/*---------------------------------------------------------------------------------------
 * FRAME_TIMER_REGS
 *---------------------------------------------------------------------------------------
 */
typedef volatile struct
{
    U_FT_PID                                   r_Pid                         ; /* Offset=0x0000 */
    U_FT_NUM_FRAMES                            r_NumFrames                   ; /* Offset=0x0004 */
    U_FT_FRAME_PERIODICITY                     r_FramePeriodicity            ; /* Offset=0x0008 */
    U_FT_FRAME_TRIGGERING_REGS                 r_FrameTriggeringRegs         ; /* Offset=0x000C */
    U_FT_FRAME_TRIG_TIMER_VAL                  r_FrameTrigTimerVal           ; /* Offset=0x0010 */
    U_FT_FRAME_START_OFFSET_INTR_TIME_1        r_FrameStartOffsetIntrTime_1  ; /* Offset=0x0014 */
    U_FT_FRAME_START_OFFSET_INTR_TIME_2        r_FrameStartOffsetIntrTime_2  ; /* Offset=0x0018 */
    U_FT_FRAME_START_OFFSET_INTR_TIME_3        r_FrameStartOffsetIntrTime_3  ; /* Offset=0x001C */
    U_FT_FRAME_REF_TIMER                       r_FrameRefTimer               ; /* Offset=0x0020 */
    U_FT_FRAME_PERIOD_TIMER                    r_FramePeriodTimer            ; /* Offset=0x0024 */
    U_FT_FRAME_CNT                             r_FrameCnt                    ; /* Offset=0x0028 */
    U_FT_TRIG_STOP_DONE                        r_TrigStopDone                ; /* Offset=0x002C */
    U_FT_HW_SPARE_REG0                         r_HwSpareReg0                 ; /* Offset=0x0030 */
    U_FT_HW_SPARE_REG1                         r_HwSpareReg1                 ; /* Offset=0x0034 */
    U_FT_HW_SPARE_REG2                         r_HwSpareReg2                 ; /* Offset=0x0038 */
    U_FT_HW_SPARE_REG3                         r_HwSpareReg3                 ; /* Offset=0x003C */
    U_FT_HW_SPARE_REG4                         r_HwSpareReg4                 ; /* Offset=0x0040 */
    U_FT_HW_SPARE_REG5                         r_HwSpareReg5                 ; /* Offset=0x0044 */
    U_FT_HW_SPARE_REG6                         r_HwSpareReg6                 ; /* Offset=0x0048 */
    U_FT_HW_SPARE_REG7                         r_HwSpareReg7                 ; /* Offset=0x004C */
    U_FT_HW_SPARE_REG8                         r_HwSpareReg8                 ; /* Offset=0x0050 */
    U_FT_HW_SPARE_REG9                         r_HwSpareReg9                 ; /* Offset=0x0054 */
    U_FT_MASK_ERROR_REG                        r_MaskErrorReg                ; /* Offset=0x0058 */
    U_FT_ERROR_STATUS_REG                      r_ErrorStatusReg              ; /* Offset=0x005C */
    UINT32                                     r_Padreg0[1002]               ; /* Offset=0x0060 */
    U_FT_LOCK0_KICK0                           r_Lock0Kick0                  ; /* Offset=0x1008 */
    U_FT_LOCK0_KICK1                           r_Lock0Kick1                  ; /* Offset=0x100C */
    U_FT_INTR_RAW_STATUS                       r_IntrRawStatus               ; /* Offset=0x1010 */
    U_FT_INTR_ENABLED_STATUS_CLEAR             r_IntrEnabledStatusClear      ; /* Offset=0x1014 */
    U_FT_INTR_ENABLE                           r_IntrEnable                  ; /* Offset=0x1018 */
    U_FT_INTR_ENABLE_CLEAR                     r_IntrEnableClear             ; /* Offset=0x101C */
    U_FT_EOI                                   r_Eoi                         ; /* Offset=0x1020 */
    U_FT_FAULT_ADDRESS                         r_FaultAddress                ; /* Offset=0x1024 */
    U_FT_FAULT_TYPE_STATUS                     r_FaultTypeStatus             ; /* Offset=0x1028 */
    U_FT_FAULT_ATTR_STATUS                     r_FaultAttrStatus             ; /* Offset=0x102C */
    U_FT_FAULT_CLEAR                           r_FaultClear                  ; /* Offset=0x1030 */
} T_FRAMETIMER_REGS;

#endif
/*
 * END OF REG_FT_H
 */

