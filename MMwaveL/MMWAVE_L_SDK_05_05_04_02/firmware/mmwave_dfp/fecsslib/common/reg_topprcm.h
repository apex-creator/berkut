/*!*****************************************************************************
 * @file reg_topprcm.h
 *
 * @brief This file gives register definitions of TOP_PRCM module
 *
 * @b Description @n
 * This file gives register definitions of TOP_PRCM module
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
#ifndef REG_TOPPRCM_H
#define REG_TOPPRCM_H

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
} U_TOPPRCM_PID;

/*------------------------------------------------------------------------------
 * CLK_REQ_PARAM -- Offset == 0x004
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLK_REQ_PARAM;

/*------------------------------------------------------------------------------
 * APP_PWR_REQ_PARAM -- Offset == 0x008
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_APP_PWR_REQ_PARAM;

/*------------------------------------------------------------------------------
 * FEC_PWR_REQ_PARAM -- Offset == 0x00C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_FEC_PWR_REQ_PARAM;

/*------------------------------------------------------------------------------
 * HWA_PWR_REQ_PARAM -- Offset == 0x010
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_HWA_PWR_REQ_PARAM;

/*------------------------------------------------------------------------------
 * APP_CORE_SYSRESET_PARAM -- Offset == 0x014
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_APP_CORE_SYSRESET_PARAM;

/*------------------------------------------------------------------------------
 * FEC_CORE_SYSRESET_PARAM -- Offset == 0x018
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WakeupDelayCount          : 11;           /*!< bits  10:  0 */
        UINT32 b1_Mode                       :  1;           /*!< bits  11: 11 */
        UINT32 b1_WakeupOutState             :  1;           /*!< bits  12: 12 */
        UINT32 b4_GoToSleepDelay             :  4;           /*!< bits  16: 13 */
        UINT32 b1_DeepsleepOutState          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_FEC_CORE_SYSRESET_PARAM;

/*------------------------------------------------------------------------------
 * RELEASE_PAUSE -- Offset == 0x01C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ReleasePause               :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RELEASE_PAUSE;

/*------------------------------------------------------------------------------
 * WU_COUNTER_END -- Offset == 0x020
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WuCounterEnd              : 11;           /*!< bits  10:  0 */
        UINT32 b21_Padfield_0                : 21;           /*!< bits  31: 11 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WU_COUNTER_END;

/*------------------------------------------------------------------------------
 * WU_COUNTER_START -- Offset == 0x024
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WuCounterStart            : 11;           /*!< bits  10:  0 */
        UINT32 b21_Padfield_0                : 21;           /*!< bits  31: 11 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WU_COUNTER_START;

/*------------------------------------------------------------------------------
 * WU_COUNTER_PAUSE -- Offset == 0x028
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b11_WuCounterPause            : 11;           /*!< bits  10:  0 */
        UINT32 b21_Padfield_0                : 21;           /*!< bits  31: 11 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WU_COUNTER_PAUSE;

/*------------------------------------------------------------------------------
 * GTS_COUNTER_END -- Offset == 0x02C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_GtsCounterEnd              :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_GTS_COUNTER_END;

/*------------------------------------------------------------------------------
 * SLEEP_COUNTER_END -- Offset == 0x030
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b21_SleepCountEnd             : 21;           /*!< bits  20:  0 */
        UINT32 b11_Padfield_0                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_SLEEP_COUNTER_END;

/*------------------------------------------------------------------------------
 * WU_SOURCE_EN -- Offset == 0x034
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b6_WuSourceEn                 :  6;           /*!< bits   5:  0 */
        UINT32 b2_Padfield_0                 :  2;           /*!< bits   7:  6 */
        UINT32 b1_UartRxEdge                 :  1;           /*!< bits   8:  8 */
        UINT32 b1_SpiCsEdge                  :  1;           /*!< bits   9:  9 */
        UINT32 b1_GpioIntEdge                :  1;           /*!< bits  10: 10 */
        UINT32 b1_SyncinIoEdge               :  1;           /*!< bits  11: 11 */
        UINT32 b20_Padfield_1                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WU_SOURCE_EN;

/*------------------------------------------------------------------------------
 * UART_RTS_CLEAR -- Offset == 0x038
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_UartRtsClear               :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_UART_RTS_CLEAR;

/*------------------------------------------------------------------------------
 * RADAR_WAKEUP_STATUS -- Offset == 0x03C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_WakeupStatus               :  2;           /*!< bits   1:  0 */
        UINT32 b6_WakeupSource               :  6;           /*!< bits   7:  2 */
        UINT32 b1_WakeupStatusClear          :  1;           /*!< bits   8:  8 */
        UINT32 b7_Padfield_0                 :  7;           /*!< bits  15:  9 */
        UINT32 b1_RadarStateIsWakeUp         :  1;           /*!< bits  16: 16 */
        UINT32 b1_RadarStateIsIdle           :  1;           /*!< bits  17: 17 */
        UINT32 b1_RadarStateIsSleep          :  1;           /*!< bits  18: 18 */
        UINT32 b1_RadarStateIsGoToDeepSleep  :  1;           /*!< bits  19: 19 */
        UINT32 b1_RadarStateIsDeepSleep      :  1;           /*!< bits  20: 20 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RADAR_WAKEUP_STATUS;

/*------------------------------------------------------------------------------
 * RTC_COMPARE_LSB -- Offset == 0x040
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_RtcCounterCompareValueLsb : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTC_COMPARE_LSB;

/*------------------------------------------------------------------------------
 * RTC_COMPARE_MSB -- Offset == 0x044
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_RtcCounterCompareValueMsb : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTC_COMPARE_MSB;

/*------------------------------------------------------------------------------
 * RTC_COMPARE_EN -- Offset == 0x048
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_RtcCounterCompareEnable    :  1;           /*!< bits   0:  0 */
        UINT32 b1_RtcCounterCompareEnableStatus :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTC_COMPARE_EN;

/*------------------------------------------------------------------------------
 * RTC_COUNT_LSB -- Offset == 0x04C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_RtcCounterValueLsb        : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTC_COUNT_LSB;

/*------------------------------------------------------------------------------
 * RTC_COUNT_MSB -- Offset == 0x050
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_RtcCounterValueMsb        : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTC_COUNT_MSB;

/*------------------------------------------------------------------------------
 * PC_REGISTER1 -- Offset == 0x054
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister1               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER1;

/*------------------------------------------------------------------------------
 * PC_REGISTER2 -- Offset == 0x058
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister2               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER2;

/*------------------------------------------------------------------------------
 * PC_REGISTER3 -- Offset == 0x05C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister3               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER3;

/*------------------------------------------------------------------------------
 * PC_REGISTER4 -- Offset == 0x060
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister4               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER4;

/*------------------------------------------------------------------------------
 * PC_REGISTER5 -- Offset == 0x064
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister5               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER5;

/*------------------------------------------------------------------------------
 * PC_REGISTER6 -- Offset == 0x068
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister6               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER6;

/*------------------------------------------------------------------------------
 * PC_REGISTER7 -- Offset == 0x06C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister7               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER7;

/*------------------------------------------------------------------------------
 * PC_REGISTER8 -- Offset == 0x070
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_PcRegister8               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PC_REGISTER8;

/*------------------------------------------------------------------------------
 * WAKEUP_IO_MUX_SEL -- Offset == 0x074
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_WakeupIoMuxSel             :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WAKEUP_IO_MUX_SEL;

/*------------------------------------------------------------------------------
 * WAKEUP_INT_SOURCE_EN -- Offset == 0x078
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_WakeupInterruptSourceEn    :  2;           /*!< bits   1:  0 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_RadarDevicesleepWakeupInterruptEn :  1;           /*!< bits   8:  8 */
        UINT32 b23_Padfield_1                : 23;           /*!< bits  31:  9 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_WAKEUP_INT_SOURCE_EN;

/*------------------------------------------------------------------------------
 * PMS_POWER_MODE -- Offset == 0x07C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvPmsActiveModeSafe     :  3;           /*!< bits   2:  0 */
        UINT32 b1_OvPmsActiveMode            :  1;           /*!< bits   3:  3 */
        UINT32 b12_Padfield_0                : 12;           /*!< bits  15:  4 */
        UINT32 b1_LdoModeStatus              :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_POWER_MODE;

/*------------------------------------------------------------------------------
 * PMS_SRAM_GO_TO_SLEEP_DELAY -- Offset == 0x080
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_SramLdoGoToSleepDelay      :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_GO_TO_SLEEP_DELAY;

/*------------------------------------------------------------------------------
 * PMS_SRAM_GO_TO_SLEEP_TIME -- Offset == 0x400
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_SramLdoGoToSleepTime       :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_GO_TO_SLEEP_TIME;

/*------------------------------------------------------------------------------
 * PMS_SRAM_WAKEUP_DELAY -- Offset == 0x404
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_SramLdoWakeupDelay         :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_WAKEUP_DELAY;

/*------------------------------------------------------------------------------
 * PMS_SRAM_WAKEUP_TIME -- Offset == 0x408
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_SramLdoWakeupTime          :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_WAKEUP_TIME;

/*------------------------------------------------------------------------------
 * PMS_SRAM_LDO_EN -- Offset == 0x40C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvSramLdoEnSafe         :  3;           /*!< bits   2:  0 */
        UINT32 b1_OvSramLdoEn                :  1;           /*!< bits   3:  3 */
        UINT32 b12_Padfield_0                : 12;           /*!< bits  15:  4 */
        UINT32 b1_SelOvSramKaLdoEn           :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvSramKaLdoEn              :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_LDO_EN;

/*------------------------------------------------------------------------------
 * PMS_SLEEP_NO_RTA_CFG -- Offset == 0x410
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SramLdoSleepNoRtaMode      :  1;           /*!< bits   0:  0 */
        UINT32 b1_SramLdoCommonRtaMode       :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SLEEP_NO_RTA_CFG;

/*------------------------------------------------------------------------------
 * PMS_SRAM_LDO_TRIM -- Offset == 0x414
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvSramLdoVtrim          :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b6_OvSramLdoVtrim             :  6;           /*!< bits  21: 16 */
        UINT32 b10_Padfield_1                : 10;           /*!< bits  31: 22 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_LDO_TRIM;

/*------------------------------------------------------------------------------
 * PMS_SRAM_KA_TRIM -- Offset == 0x418
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvSramKaLdoVtrim        :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b6_OvSramKaLdoTrimRta         :  6;           /*!< bits  21: 16 */
        UINT32 b2_Padfield_1                 :  2;           /*!< bits  23: 22 */
        UINT32 b6_OvSramKaLdoTrimNoRta       :  6;           /*!< bits  29: 24 */
        UINT32 b2_Padfield_2                 :  2;           /*!< bits  31: 30 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_SRAM_KA_TRIM;

/*------------------------------------------------------------------------------
 * PMS_DIG_GO_TO_SLEEP_DELAY -- Offset == 0x41C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_DigLdoGoToSleepDelay       :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_GO_TO_SLEEP_DELAY;

/*------------------------------------------------------------------------------
 * PMS_DIG_GO_TO_SLEEP_TIME -- Offset == 0x420
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_DigLdoGoToSleepTime        :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_GO_TO_SLEEP_TIME;

/*------------------------------------------------------------------------------
 * PMS_DIG_WAKEUP_DELAY -- Offset == 0x424
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_DigLdoWakeupDelay          :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_WAKEUP_DELAY;

/*------------------------------------------------------------------------------
 * PMS_DIG_WAKEUP_TIME -- Offset == 0x428
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_DigLdoWakeupTime           :  5;           /*!< bits   4:  0 */
        UINT32 b27_Padfield_0                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_WAKEUP_TIME;

/*------------------------------------------------------------------------------
 * PMS_DIG_LDO_EN -- Offset == 0x42C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvDigLdoEnSafe          :  3;           /*!< bits   2:  0 */
        UINT32 b1_OvDigLdoEn                 :  1;           /*!< bits   3:  3 */
        UINT32 b12_Padfield_0                : 12;           /*!< bits  15:  4 */
        UINT32 b1_SelOvDigKaLdoEn            :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvDigKaLdoEn               :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_LDO_EN;

/*------------------------------------------------------------------------------
 * PMS_DIG_LDO_TRIM -- Offset == 0x430
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvDigLdoVtrim           :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b6_OvDigLdoVtrim              :  6;           /*!< bits  21: 16 */
        UINT32 b10_Padfield_1                : 10;           /*!< bits  31: 22 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_LDO_TRIM;

/*------------------------------------------------------------------------------
 * PMS_DIG_KA_TRIM -- Offset == 0x434
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvDigKaLdoVtrim         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b6_OvDigKaLdoVtrim            :  6;           /*!< bits  21: 16 */
        UINT32 b10_Padfield_1                : 10;           /*!< bits  31: 22 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_DIG_KA_TRIM;

/*------------------------------------------------------------------------------
 * PMS_PSCON_EN_WAIT_DELAY -- Offset == 0x438
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_PsconEnWaitDelay           :  8;           /*!< bits   7:  0 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  15:  8 */
        UINT32 b1_PsconEnForceActive         :  1;           /*!< bits  16: 16 */
        UINT32 b7_Padfield_1                 :  7;           /*!< bits  23: 17 */
        UINT32 b1_AllowPsconEnBeforeDigLdoActive :  1;           /*!< bits  24: 24 */
        UINT32 b7_Padfield_2                 :  7;           /*!< bits  31: 25 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_PSCON_EN_WAIT_DELAY;

/*------------------------------------------------------------------------------
 * PMS_BGAP_DIS_HIBERNATE -- Offset == 0x43C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_BgapDisHibernate           :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_DIS_HIBERNATE;

/*------------------------------------------------------------------------------
 * PMS_BGAP_DELAY1 -- Offset == 0x440
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b7_BgapEnterSleepDelay        :  7;           /*!< bits   6:  0 */
        UINT32 b9_Padfield_0                 :  9;           /*!< bits  15:  7 */
        UINT32 b10_BgapHibRefreshTime        : 10;           /*!< bits  25: 16 */
        UINT32 b6_Padfield_1                 :  6;           /*!< bits  31: 26 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_DELAY1;

/*------------------------------------------------------------------------------
 * PMS_BGAP_DELAY2 -- Offset == 0x444
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_BgapCapChargeTime          :  3;           /*!< bits   2:  0 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits   7:  3 */
        UINT32 b6_BgapPreCapChargeEnDelay    :  6;           /*!< bits  13:  8 */
        UINT32 b2_Padfield_1                 :  2;           /*!< bits  15: 14 */
        UINT32 b3_BgapRefCapChargeTime       :  3;           /*!< bits  18: 16 */
        UINT32 b13_Padfield_2                : 13;           /*!< bits  31: 19 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_DELAY2;

/*------------------------------------------------------------------------------
 * PMS_BGAP_EN_OVERRIDE -- Offset == 0x448
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvBgapEnSafe            :  3;           /*!< bits   2:  0 */
        UINT32 b1_OvBgapEn                   :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_EN_OVERRIDE;

/*------------------------------------------------------------------------------
 * PMS_BGAP_CAP_OVERRIDE1 -- Offset == 0x44C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvBgapCapSwEnz          :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvBgapCapSwEnz             :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvBgapCapChargeEn       :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvBgapCapChargeEn          :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_CAP_OVERRIDE1;

/*------------------------------------------------------------------------------
 * PMS_BGAP_CAP_OVERRIDE2 -- Offset == 0x450
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvBgapHibCapSwEn        :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvBgapHibCapSwEn           :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvBgapHibRefCapChargeEn :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvBgapHibRefCapChargeEn    :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PMS_BGAP_CAP_OVERRIDE2;

/*------------------------------------------------------------------------------
 * PSCON_APP_PD_EN -- Offset == 0x454
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvAppPdIsSleep          :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvAppPdIsSleep             :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_AppPdPowerStatus           :  1;           /*!< bits   8:  8 */
        UINT32 b1_AppPdResetStatus           :  1;           /*!< bits   9:  9 */
        UINT32 b22_Padfield_1                : 22;           /*!< bits  31: 10 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_APP_PD_EN;

/*------------------------------------------------------------------------------
 * PSCON_FEC_PD_EN -- Offset == 0x458
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvFecPdIsSleep          :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvFecPdIsSleep             :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_FecPdPowerStatus           :  1;           /*!< bits   8:  8 */
        UINT32 b1_FecPdResetStatus           :  1;           /*!< bits   9:  9 */
        UINT32 b22_Padfield_1                : 22;           /*!< bits  31: 10 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_FEC_PD_EN;

/*------------------------------------------------------------------------------
 * PSCON_HWA_PD_EN -- Offset == 0x45C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvHwaPdIsSleep          :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvHwaPdIsSleep             :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_HwaPdPowerStatus           :  1;           /*!< bits   8:  8 */
        UINT32 b1_HwaPdResetStatus           :  1;           /*!< bits   9:  9 */
        UINT32 b22_Padfield_1                : 22;           /*!< bits  31: 10 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_HWA_PD_EN;

/*------------------------------------------------------------------------------
 * PSCON_TEST_DBG_PD_EN -- Offset == 0x460
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvTestDbgPdIsSleep      :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvTestDbgPdIsSleep         :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_TestDbgPdPowerStatus       :  1;           /*!< bits   8:  8 */
        UINT32 b1_TestDbgPdResetStatus       :  1;           /*!< bits   9:  9 */
        UINT32 b22_Padfield_1                : 22;           /*!< bits  31: 10 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_TEST_DBG_PD_EN;

/*------------------------------------------------------------------------------
 * PSCON_APP_PD_RAM_STATE -- Offset == 0x464
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_AppPdMemSleepState         :  3;           /*!< bits   2:  0 */
        UINT32 b13_Padfield_0                : 13;           /*!< bits  15:  3 */
        UINT32 b3_AppPdMemActiveState        :  3;           /*!< bits  18: 16 */
        UINT32 b13_Padfield_1                : 13;           /*!< bits  31: 19 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_APP_PD_RAM_STATE;

/*------------------------------------------------------------------------------
 * PSCON_APP_PD_RAM_GRP1_STATE -- Offset == 0x468
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_AppPdMemGrp1SleepState     :  2;           /*!< bits   1:  0 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b2_AppPdMemGrp1ActiveState    :  2;           /*!< bits  17: 16 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_APP_PD_RAM_GRP1_STATE;

/*------------------------------------------------------------------------------
 * PSCON_APP_PD_RAM_GRP2_STATE -- Offset == 0x46C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_AppPdMemGrp2SleepState     :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b1_AppPdMemGrp2ActiveState    :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_APP_PD_RAM_GRP2_STATE;

/*------------------------------------------------------------------------------
 * PSCON_HWA_PD_RAM_GRP3_STATE -- Offset == 0x470
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_HwaPdMemGrp3SleepState     :  3;           /*!< bits   2:  0 */
        UINT32 b13_Padfield_0                : 13;           /*!< bits  15:  3 */
        UINT32 b3_HwaPdMemGrp3ActiveState    :  3;           /*!< bits  18: 16 */
        UINT32 b13_Padfield_1                : 13;           /*!< bits  31: 19 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_HWA_PD_RAM_GRP3_STATE;

/*------------------------------------------------------------------------------
 * PSCON_FEC_PD_RAM_STATE -- Offset == 0x474
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecPdMemSleepState         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b1_FecPdMemActiveState        :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_FEC_PD_RAM_STATE;

/*------------------------------------------------------------------------------
 * PSCON_FEC_PD_RAM_GRP4_STATE -- Offset == 0x478
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_FecPdMemGrp4SleepState     :  2;           /*!< bits   1:  0 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b2_FecPdMemGrp4ActiveState    :  2;           /*!< bits  17: 16 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_FEC_PD_RAM_GRP4_STATE;

/*------------------------------------------------------------------------------
 * PSCON_RAM_FORCE_SWITCH_EN -- Offset == 0x47C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MemAllSwitchForceEn        :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_RAM_FORCE_SWITCH_EN;

/*------------------------------------------------------------------------------
 * PSCON_RAM_SWITCH_EN_OVERRIDE1 -- Offset == 0x480
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvAppPdMemGrp1SwitchEn  :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvAppPdMemGrp1SwitchEn     :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvAppPdMemGrp2SwitchEn  :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvAppPdMemGrp2SwitchEn     :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_RAM_SWITCH_EN_OVERRIDE1;

/*------------------------------------------------------------------------------
 * PSCON_RAM_SWITCH_EN_OVERRIDE2 -- Offset == 0x484
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvHwaPdMemGrp3SwitchEn  :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvHwaPdMemGrp3SwitchEn     :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvFecPdMemGrp4SwitchEn  :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvFecPdMemGrp4SwitchEn     :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_RAM_SWITCH_EN_OVERRIDE2;

/*------------------------------------------------------------------------------
 * PSCOM_RAM_SWITCH_DELAY -- Offset == 0x488
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_UseMemSwitchDelayGlobal    :  1;           /*!< bits   0:  0 */
        UINT32 b2_GlobalMemSwitchHighresLowresDelay :  2;           /*!< bits   2:  1 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits   7:  3 */
        UINT32 b2_AppPdMemGrp1SwitchHighresLowresDelay :  2;           /*!< bits   9:  8 */
        UINT32 b2_AppPdMemGrp2SwitchHighresLowresDelay :  2;           /*!< bits  11: 10 */
        UINT32 b2_HwaPdMemGrp3SwitchHighresLowresDelay :  2;           /*!< bits  13: 12 */
        UINT32 b2_FecPdMemGrp4SwitchHighresLowresDelay :  2;           /*!< bits  15: 14 */
        UINT32 b16_Padfield_1                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCOM_RAM_SWITCH_DELAY;

/*------------------------------------------------------------------------------
 * PSCON_DFTRTA_OVERRIDE -- Offset == 0x48C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvDftrtaon              :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvDftrtaon                 :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvDftrtagood            :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvDftrtagood               :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_DFTRTA_OVERRIDE;

/*------------------------------------------------------------------------------
 * PSCON_VNWA_SWITCH_EN1 -- Offset == 0x490
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvAppPdMemVnwaSwitchEn  :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvAppPdMemVnwaSwitchEn     :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_SelOvAppPdMemGrp1VnwaSwitchEn :  1;           /*!< bits   8:  8 */
        UINT32 b1_OvAppPdMemGrp1VnwaSwitchEn :  1;           /*!< bits   9:  9 */
        UINT32 b6_Padfield_1                 :  6;           /*!< bits  15: 10 */
        UINT32 b1_SelOvAppPdMemGrp2VnwaSwitchEn :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvAppPdMemGrp2VnwaSwitchEn :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_2                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_VNWA_SWITCH_EN1;

/*------------------------------------------------------------------------------
 * PSCON_VNWA_SWITCH_EN2 -- Offset == 0x494
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvHwaPdMemGrp3VnwaSwitchEn :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvHwaPdMemGrp3VnwaSwitchEn :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_SelOvFecPdMemVnwaSwitchEn  :  1;           /*!< bits   8:  8 */
        UINT32 b1_OvFecPdMemVnwaSwitchEn     :  1;           /*!< bits   9:  9 */
        UINT32 b6_Padfield_1                 :  6;           /*!< bits  15: 10 */
        UINT32 b1_SelOvFecPdMemGrp4VnwaSwitchEn :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvFecPdMemGrp4VnwaSwitchEn :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_2                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_VNWA_SWITCH_EN2;

/*------------------------------------------------------------------------------
 * PSCON_SRAM_LDO_WEAK_PROCESS -- Offset == 0x498
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvVnwaSwitchWeakProcess :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvVnwaSwitchWeakProcess    :  1;           /*!< bits   1:  1 */
        UINT32 b6_Padfield_0                 :  6;           /*!< bits   7:  2 */
        UINT32 b1_VnwaSwitchWeakProcessAtDeepsleepExit :  1;           /*!< bits   8:  8 */
        UINT32 b23_Padfield_1                : 23;           /*!< bits  31:  9 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PSCON_SRAM_LDO_WEAK_PROCESS;

/*------------------------------------------------------------------------------
 * CLKM_OSC_CLK_REQ -- Offset == 0x49C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvOscClkReq             :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvOscClkReq                :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_OSC_CLK_REQ;

/*------------------------------------------------------------------------------
 * CLKM_OVERRIDE1 -- Offset == 0x4A0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvSlicerLdoEn           :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvSlicerLdoEn              :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvSlicerBiasEn          :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvSlicerBiasEn             :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_OVERRIDE1;

/*------------------------------------------------------------------------------
 * CLKM_OVERRIDE2 -- Offset == 0x4A4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvXtalEn                :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvXtalEn                   :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvSlicerEn              :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvSlicerEn                 :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_OVERRIDE2;

/*------------------------------------------------------------------------------
 * CLKM_OVERRIDE3 -- Offset == 0x4A8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvXtalDetEn             :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvXtalDetEn                :  1;           /*!< bits   1:  1 */
        UINT32 b14_Padfield_0                : 14;           /*!< bits  15:  2 */
        UINT32 b1_SelOvClkmOscillatorClkValid :  1;           /*!< bits  16: 16 */
        UINT32 b1_OvClkmOscillatorClkValid   :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_OVERRIDE3;

/*------------------------------------------------------------------------------
 * CLKM_OUTPUT_HOST_CLK_REQ_OVERRIDE -- Offset == 0x4AC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvHostClkReq            :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvHostClkReq               :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_OUTPUT_HOST_CLK_REQ_OVERRIDE;

/*------------------------------------------------------------------------------
 * CLKM_SEL_OV_XT_DRIVE -- Offset == 0x4B0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvLowXtDrive            :  1;           /*!< bits   0:  0 */
        UINT32 b5_OvLowXtDrive               :  5;           /*!< bits   5:  1 */
        UINT32 b10_Padfield_0                : 10;           /*!< bits  15:  6 */
        UINT32 b1_SelOvHighXtDrive           :  1;           /*!< bits  16: 16 */
        UINT32 b5_OvHighXtDrive              :  5;           /*!< bits  21: 17 */
        UINT32 b10_Padfield_1                : 10;           /*!< bits  31: 22 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_SEL_OV_XT_DRIVE;

/*------------------------------------------------------------------------------
 * CLKM_DELAY1 -- Offset == 0x4B4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b5_ClkmClkReqDelay            :  5;           /*!< bits   4:  0 */
        UINT32 b11_Padfield_0                : 11;           /*!< bits  15:  5 */
        UINT32 b5_ClkmExtendValidDelay       :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_DELAY1;

/*------------------------------------------------------------------------------
 * CLKM_DELAY2 -- Offset == 0x4B8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvClkmOscEnDelay        :  1;           /*!< bits   0:  0 */
        UINT32 b10_OvClkmOscEnDelay          : 10;           /*!< bits  10:  1 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits  15: 11 */
        UINT32 b1_SelOvClkmSlicerEnDelay     :  1;           /*!< bits  16: 16 */
        UINT32 b10_OvClkmSlicerEnDelay       : 10;           /*!< bits  26: 17 */
        UINT32 b5_Padfield_1                 :  5;           /*!< bits  31: 27 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_DELAY2;

/*------------------------------------------------------------------------------
 * CLKM_HOST_CLK_REQ_DELAY -- Offset == 0x4BC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvClkmHostClkReqDelay   :  1;           /*!< bits   0:  0 */
        UINT32 b10_OvClkmHostClkReqDelay     : 10;           /*!< bits  10:  1 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits  15: 11 */
        UINT32 b1_SelOvClkmHostClkReqPulseWidth :  1;           /*!< bits  16: 16 */
        UINT32 b10_OvClkmHostClkReqPulseWidth : 10;           /*!< bits  26: 17 */
        UINT32 b5_Padfield_1                 :  5;           /*!< bits  31: 27 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLKM_HOST_CLK_REQ_DELAY;

/*------------------------------------------------------------------------------
 * RST_OVERRIDE -- Offset == 0x4C0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_OvAppPdPorRstn             :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvFecPdPorRstn             :  1;           /*!< bits   1:  1 */
        UINT32 b1_OvHwaPdPorRstn             :  1;           /*!< bits   2:  2 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits   3:  3 */
        UINT32 b1_OvPsconPorRstn             :  1;           /*!< bits   4:  4 */
        UINT32 b27_Padfield_1                : 27;           /*!< bits  31:  5 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_OVERRIDE;

/*------------------------------------------------------------------------------
 * RST_SOFT_RESET -- Offset == 0x4C4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_WarmResetReqn              :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_WarmRstnPulseWidth         :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_SOFT_RESET;

/*------------------------------------------------------------------------------
 * RST_WDT_RESET_EN -- Offset == 0x4C8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_WdResetEn                  :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_WDT_RESET_EN;

/*------------------------------------------------------------------------------
 * RST_APP_PD_SOFT_RESET -- Offset == 0x4CC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_AppPdWarmResetReqn         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_AppPdWarmRstnPulseWidth    :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_APP_PD_SOFT_RESET;

/*------------------------------------------------------------------------------
 * RST_FEC_PD_SOFT_RESET -- Offset == 0x4D0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecPdWarmResetReqn         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_FecPdWarmRstnPulseWidth    :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_FEC_PD_SOFT_RESET;

/*------------------------------------------------------------------------------
 * RST_HWA_PD_SOFT_RESET -- Offset == 0x4D4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_HwaPdWarmResetReqn         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_HwaPdWarmRstnPulseWidth    :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_HWA_PD_SOFT_RESET;

/*------------------------------------------------------------------------------
 * RST_SOFT_APP_CORE_SYSRESET_REQ -- Offset == 0x4D8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_AppPdCoreResetReqn         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_AppPdCoreRstnPulseWidth    :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_SOFT_APP_CORE_SYSRESET_REQ;

/*------------------------------------------------------------------------------
 * RST_SOFT_FEC_CORE_SYSRESET_REQ -- Offset == 0x4DC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecPdCoreResetReqn         :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b5_FecPdCoreRstnPulseWidth    :  5;           /*!< bits  20: 16 */
        UINT32 b11_Padfield_1                : 11;           /*!< bits  31: 21 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RST_SOFT_FEC_CORE_SYSRESET_REQ;

/*------------------------------------------------------------------------------
 * SYS_RST_CAUSE -- Offset == 0x4E0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SysRstCause                :  3;           /*!< bits   2:  0 */
        UINT32 b13_Padfield_0                : 13;           /*!< bits  15:  3 */
        UINT32 b1_SysRstCauseClr             :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_SYS_RST_CAUSE;

/*------------------------------------------------------------------------------
 * DUBUGSS_DISABLE -- Offset == 0x4E4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_DebugssDisable             :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DUBUGSS_DISABLE;

/*------------------------------------------------------------------------------
 * SLOW_CLK_CLKCTL -- Offset == 0x4E8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SlowClkSrcSel              :  3;           /*!< bits   2:  0 */
        UINT32 b13_Padfield_0                : 13;           /*!< bits  15:  3 */
        UINT32 b2_SlowClkInUse               :  2;           /*!< bits  17: 16 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_SLOW_CLK_CLKCTL;

/*------------------------------------------------------------------------------
 * DEBUGSS_CLK_CLKCTL -- Offset == 0x4EC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b12_DebugssClkSrcSel          : 12;           /*!< bits  11:  0 */
        UINT32 b4_Padfield_0                 :  4;           /*!< bits  15: 12 */
        UINT32 b8_DebugssClkInUse            :  8;           /*!< bits  23: 16 */
        UINT32 b8_Padfield_1                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUGSS_CLK_CLKCTL;

/*------------------------------------------------------------------------------
 * EFUSE_10M_OSC_DISABLE -- Offset == 0x4F0
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvEfuse_10mhzOscDisable :  3;           /*!< bits   2:  0 */
        UINT32 b1_OvEfuse_10mhzOscDisable    :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_EFUSE_10M_OSC_DISABLE;

/*------------------------------------------------------------------------------
 * DEBUGSS_CLK_AUTOSWITCH -- Offset == 0x4F4
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_DebugssClkAutoswitchEn     :  2;           /*!< bits   1:  0 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUGSS_CLK_AUTOSWITCH;

/*------------------------------------------------------------------------------
 * RADAR_SAFTY_ERROR_REG -- Offset == 0x4F8
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_RadarStateFsmUnknownStateErrorReg :  1;           /*!< bits   0:  0 */
        UINT32 b1_SramLdoFsmUnknownStateErrorReg :  1;           /*!< bits   1:  1 */
        UINT32 b1_DigLdoFsmUnknownStateErrorReg :  1;           /*!< bits   2:  2 */
        UINT32 b1_BgapFsmUnknownStateErrorReg :  1;           /*!< bits   3:  3 */
        UINT32 b1_ClkmFsmUnknownStateErrorReg :  1;           /*!< bits   4:  4 */
        UINT32 b4_PsconLogicFsmUnknownStateErrorReg :  4;           /*!< bits   8:  5 */
        UINT32 b6_PsconMemFsmUnknownStateErrorReg :  6;           /*!< bits  14:  9 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits  15: 15 */
        UINT32 b1_SafetyErrorRegWrDis        :  1;           /*!< bits  16: 16 */
        UINT32 b1_SafetyErrorRegClear        :  1;           /*!< bits  17: 17 */
        UINT32 b14_Padfield_1                : 14;           /*!< bits  31: 18 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RADAR_SAFTY_ERROR_REG;

/*------------------------------------------------------------------------------
 * RELEASEFROMWIR_REG -- Offset == 0x4FC
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ReleasefromwirAppCoreRstn  :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b1_ReleasefromwirFecCoreRstn  :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RELEASEFROMWIR_REG;

/*------------------------------------------------------------------------------
 * HWA_PD_MEM_SHARE_REG -- Offset == 0x500
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b6_HwaPdMemShareAppssConfig   :  6;           /*!< bits   5:  0 */
        UINT32 b2_Padfield_0                 :  2;           /*!< bits   7:  6 */
        UINT32 b3_HwaPdMemShareFecssConfig   :  3;           /*!< bits  10:  8 */
        UINT32 b21_Padfield_1                : 21;           /*!< bits  31: 11 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_HWA_PD_MEM_SHARE_REG;

/*------------------------------------------------------------------------------
 * FRC_OSC_CLK_GATE -- Offset == 0x504
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_ClkGate                    :  3;           /*!< bits   2:  0 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_FRC_OSC_CLK_GATE;

/*------------------------------------------------------------------------------
 * MEMSWAP_REG -- Offset == 0x508
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_MmrAppmemswapLock          :  1;           /*!< bits   0:  0 */
        UINT32 b1_MmrFecmemswapLock          :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_MEMSWAP_REG;

/*------------------------------------------------------------------------------
 * LIMP_MODE_STATUS -- Offset == 0x50C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_LimpModeXtalClk            :  1;           /*!< bits   0:  0 */
        UINT32 b1_LimpModeRcosc10m           :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_LIMP_MODE_STATUS;

/*------------------------------------------------------------------------------
 * RTI_CLOCK_GATE_SLEEP_STATE -- Offset == 0x510
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_AppssRtiWdClkGateInSleep   :  1;           /*!< bits   0:  0 */
        UINT32 b7_Padfield_0                 :  7;           /*!< bits   7:  1 */
        UINT32 b1_FecssRtiClkGateInSleep     :  1;           /*!< bits   8:  8 */
        UINT32 b23_Padfield_1                : 23;           /*!< bits  31:  9 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RTI_CLOCK_GATE_SLEEP_STATE;

/*------------------------------------------------------------------------------
 * TOP_3318_LDO_EN_CTRL -- Offset == 0x514
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Top_3318KaLdoEn            :  1;           /*!< bits   0:  0 */
        UINT32 b7_Padfield_0                 :  7;           /*!< bits   7:  1 */
        UINT32 b1_Top_3318LdoEn              :  1;           /*!< bits   8:  8 */
        UINT32 b7_Padfield_1                 :  7;           /*!< bits  15:  9 */
        UINT32 b5_Top_3318KaLdoVtrim         :  5;           /*!< bits  20: 16 */
        UINT32 b3_Padfield_2                 :  3;           /*!< bits  23: 21 */
        UINT32 b5_Top_3318LdoVtrim           :  5;           /*!< bits  28: 24 */
        UINT32 b3_Padfield_3                 :  3;           /*!< bits  31: 29 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_3318_LDO_EN_CTRL;

/*------------------------------------------------------------------------------
 * RFANA_TOP_LDO_EN -- Offset == 0x518
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Rx1AdcDigLdoEn             :  1;           /*!< bits   0:  0 */
        UINT32 b1_Rx1IfaLdoEn                :  1;           /*!< bits   1:  1 */
        UINT32 b1_Rx2AdcDigLdoEn             :  1;           /*!< bits   2:  2 */
        UINT32 b1_Rx2IfaLdoEn                :  1;           /*!< bits   3:  3 */
        UINT32 b1_Rx3AdcDigLdoEn             :  1;           /*!< bits   4:  4 */
        UINT32 b1_Rx3IfaLdoEn                :  1;           /*!< bits   5:  5 */
        UINT32 b4_Spare0                     :  4;           /*!< bits   9:  6 */
        UINT32 b1_ApllCpLdoEn                :  1;           /*!< bits  10: 10 */
        UINT32 b1_ApllVcoLdoEn               :  1;           /*!< bits  11: 11 */
        UINT32 b1_ApllObufLdoEn              :  1;           /*!< bits  12: 12 */
        UINT32 b1_SynthVcoLdoEn              :  1;           /*!< bits  13: 13 */
        UINT32 b1_SynthDivLdoEn              :  1;           /*!< bits  14: 14 */
        UINT32 b1_SynthSdmLdoEn              :  1;           /*!< bits  15: 15 */
        UINT32 b1_TxAnaLdoEn                 :  1;           /*!< bits  16: 16 */
        UINT32 b1_LodistLdoEn                :  1;           /*!< bits  17: 17 */
        UINT32 b2_Spare1                     :  2;           /*!< bits  19: 18 */
        UINT32 b12_Padfield_0                : 12;           /*!< bits  31: 20 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RFANA_TOP_LDO_EN;

/*------------------------------------------------------------------------------
 * RFANA_TOP_ISO_CTRL -- Offset == 0x51C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Rx1AdcDigIso               :  1;           /*!< bits   0:  0 */
        UINT32 b1_Rx1IfaLdoIso               :  1;           /*!< bits   1:  1 */
        UINT32 b1_Rx2AdcDigLdoIso            :  1;           /*!< bits   2:  2 */
        UINT32 b1_Rx2IfaLdoIso               :  1;           /*!< bits   3:  3 */
        UINT32 b1_Rx3AdcDigLdoIso            :  1;           /*!< bits   4:  4 */
        UINT32 b1_Rx3IfaLdoIso               :  1;           /*!< bits   5:  5 */
        UINT32 b3_Spare0                     :  3;           /*!< bits   8:  6 */
        UINT32 b1_MdllClkIso                 :  1;           /*!< bits   9:  9 */
        UINT32 b1_ApllCpLdoIso               :  1;           /*!< bits  10: 10 */
        UINT32 b1_ApllVcoLdoIso              :  1;           /*!< bits  11: 11 */
        UINT32 b1_ApllObufLdoIso             :  1;           /*!< bits  12: 12 */
        UINT32 b1_SynthVcoLdoIso             :  1;           /*!< bits  13: 13 */
        UINT32 b1_SynthDivLdoIso             :  1;           /*!< bits  14: 14 */
        UINT32 b1_SynthSdmLdoIso             :  1;           /*!< bits  15: 15 */
        UINT32 b1_TxAnaLdoIso                :  1;           /*!< bits  16: 16 */
        UINT32 b1_LodistLdoIso               :  1;           /*!< bits  17: 17 */
        UINT32 b2_Spare1                     :  2;           /*!< bits  19: 18 */
        UINT32 b12_Padfield_0                : 12;           /*!< bits  31: 20 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_RFANA_TOP_ISO_CTRL;

/*------------------------------------------------------------------------------
 * CLK_CTRL_REG1_LDO_CLKTOP -- Offset == 0x520
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_LdoVoutCtrl                :  4;           /*!< bits   3:  0 */
        UINT32 b1_EnzLowBwCap                :  1;           /*!< bits   4:  4 */
        UINT32 b1_EnTestMode                 :  1;           /*!< bits   5:  5 */
        UINT32 b1_EnShrtCkt                  :  1;           /*!< bits   6:  6 */
        UINT32 b1_EnBypass                   :  1;           /*!< bits   7:  7 */
        UINT32 b3_LdoBwCtrl                  :  3;           /*!< bits  10:  8 */
        UINT32 b1_ScprtIbiasCtrl             :  1;           /*!< bits  11: 11 */
        UINT32 b1_EnablePmosPulldown         :  1;           /*!< bits  12: 12 */
        UINT32 b3_TloadCtrl                  :  3;           /*!< bits  15: 13 */
        UINT32 b4_TestmuxCtrl                :  4;           /*!< bits  19: 16 */
        UINT32 b4_BistmuxCtrl                :  4;           /*!< bits  23: 20 */
        UINT32 b8_Spare0                     :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLK_CTRL_REG1_LDO_CLKTOP;

/*------------------------------------------------------------------------------
 * CLK_CTRL_REG1_XO_SLICER -- Offset == 0x524
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_RtrimBiasXoSlicer          :  4;           /*!< bits   3:  0 */
        UINT32 b5_Spare0                     :  5;           /*!< bits   8:  4 */
        UINT32 b1_FastchargezBiasXoSlicer    :  1;           /*!< bits   9:  9 */
        UINT32 b1_SlicerHipwrXoSlicer        :  1;           /*!< bits  10: 10 */
        UINT32 b1_SlicerDccplXoSlicer        :  1;           /*!< bits  11: 11 */
        UINT32 b1_Spare1                     :  1;           /*!< bits  12: 12 */
        UINT32 b1_SlicerApllBypass           :  1;           /*!< bits  13: 13 */
        UINT32 b1_SlicerApllBypassDrv        :  1;           /*!< bits  14: 14 */
        UINT32 b17_Spare2                    : 17;           /*!< bits  31: 15 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_CLK_CTRL_REG1_XO_SLICER;

/*------------------------------------------------------------------------------
 * TOP_LDO_3318_CTRL_REG0 -- Offset == 0x528
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b2_Spare0                     :  2;           /*!< bits   1:  0 */
        UINT32 b1_LdoTestEn                  :  1;           /*!< bits   2:  2 */
        UINT32 b8_LdoSpare                   :  8;           /*!< bits  10:  3 */
        UINT32 b1_LdoEn                      :  1;           /*!< bits  11: 11 */
        UINT32 b15_LdoCtrl                   : 15;           /*!< bits  26: 12 */
        UINT32 b1_BistEn                     :  1;           /*!< bits  27: 27 */
        UINT32 b4_BistCtrl                   :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_3318_CTRL_REG0;

/*------------------------------------------------------------------------------
 * TOP_LDO_3318_CTRL_REG1 -- Offset == 0x52C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_KaldoSpare                : 16;           /*!< bits  15:  0 */
        UINT32 b15_KaldoCtrl                 : 15;           /*!< bits  30: 16 */
        UINT32 b1_KaldoEn                    :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_3318_CTRL_REG1;

/*------------------------------------------------------------------------------
 * TOP_LDO_DIG_CTRL_REG0 -- Offset == 0x530
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_DigldoTrim                 :  8;           /*!< bits   7:  0 */
        UINT32 b2_DldoBwCtrl                 :  2;           /*!< bits   9:  8 */
        UINT32 b1_DldoScEn                   :  1;           /*!< bits  10: 10 */
        UINT32 b1_DldoBistEn                 :  1;           /*!< bits  11: 11 */
        UINT32 b4_DldoBistCtrl               :  4;           /*!< bits  15: 12 */
        UINT32 b6_Spare0                     :  6;           /*!< bits  21: 16 */
        UINT32 b4_DkaldoTrim                 :  4;           /*!< bits  25: 22 */
        UINT32 b4_DkaldoRtrim                :  4;           /*!< bits  29: 26 */
        UINT32 b1_Spare1                     :  1;           /*!< bits  30: 30 */
        UINT32 b1_Spare2                     :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_DIG_CTRL_REG0;

/*------------------------------------------------------------------------------
 * TOP_LDO_DIG_CTRL_REG1 -- Offset == 0x534
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b13_Spare0                    : 13;           /*!< bits  12:  0 */
        UINT32 b1_DldoBypass                 :  1;           /*!< bits  13: 13 */
        UINT32 b1_DldoScTrim                 :  1;           /*!< bits  14: 14 */
        UINT32 b6_Spare1                     :  6;           /*!< bits  20: 15 */
        UINT32 b8_DldoTmuxCtrl               :  8;           /*!< bits  28: 21 */
        UINT32 b1_DkaldoRtrimEn              :  1;           /*!< bits  29: 29 */
        UINT32 b1_DldoTestEn                 :  1;           /*!< bits  30: 30 */
        UINT32 b1_DldoInrushCtrlEn           :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_DIG_CTRL_REG1;

/*------------------------------------------------------------------------------
 * TOP_LDO_SRAM_CTRL_REG0 -- Offset == 0x538
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_SldoTrim                   :  8;           /*!< bits   7:  0 */
        UINT32 b2_SldoBwCtrl                 :  2;           /*!< bits   9:  8 */
        UINT32 b1_SldoScEn                   :  1;           /*!< bits  10: 10 */
        UINT32 b1_SldoBistEn                 :  1;           /*!< bits  11: 11 */
        UINT32 b4_SldoBistCtrl               :  4;           /*!< bits  15: 12 */
        UINT32 b6_Spare0                     :  6;           /*!< bits  21: 16 */
        UINT32 b4_Spare1                     :  4;           /*!< bits  25: 22 */
        UINT32 b4_SkaldoRtrim                :  4;           /*!< bits  29: 26 */
        UINT32 b1_Spare2                     :  1;           /*!< bits  30: 30 */
        UINT32 b1_Spare3                     :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_SRAM_CTRL_REG0;

/*------------------------------------------------------------------------------
 * TOP_LDO_SRAM_CTRL_REG1 -- Offset == 0x53C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b10_Spare0                    : 10;           /*!< bits   9:  0 */
        UINT32 b3_Padfield_0                 :  3;           /*!< bits  12: 10 */
        UINT32 b1_SldoBypass                 :  1;           /*!< bits  13: 13 */
        UINT32 b1_SldoScTrim                 :  1;           /*!< bits  14: 14 */
        UINT32 b6_Spare1                     :  6;           /*!< bits  20: 15 */
        UINT32 b8_SldoTmuxCtrl               :  8;           /*!< bits  28: 21 */
        UINT32 b1_SkaldoRtrimEn              :  1;           /*!< bits  29: 29 */
        UINT32 b1_SldoTestEn                 :  1;           /*!< bits  30: 30 */
        UINT32 b1_SldoInrushCtrlEn           :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_TOP_LDO_SRAM_CTRL_REG1;

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
} U_TOPPRCM_LOCK0_KICK0;

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
} U_TOPPRCM_LOCK0_KICK1;

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
} U_TOPPRCM_INTR_RAW_STATUS;

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
} U_TOPPRCM_INTR_ENABLED_STATUS_CLEAR;

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
} U_TOPPRCM_INTR_ENABLE;

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
} U_TOPPRCM_INTR_ENABLE_CLEAR;

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
} U_TOPPRCM_EOI;

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
} U_TOPPRCM_FAULT_ADDRESS;

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
} U_TOPPRCM_FAULT_TYPE_STATUS;

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
} U_TOPPRCM_FAULT_ATTR_STATUS;

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
} U_TOPPRCM_FAULT_CLEAR;

/*------------------------------------------------------------------------------
 * DEBUG_LOGIC_PSCON_OVERRIDE_SEL -- Offset == 0x1800
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SelOvAppPdPonin            :  1;           /*!< bits   0:  0 */
        UINT32 b1_SelOvAppPdPgoodin          :  1;           /*!< bits   1:  1 */
        UINT32 b1_SelOvAppPdIso              :  1;           /*!< bits   2:  2 */
        UINT32 b1_SelOvFecPdPonin            :  1;           /*!< bits   3:  3 */
        UINT32 b1_SelOvFecPdPgoodin          :  1;           /*!< bits   4:  4 */
        UINT32 b1_SelOvFecPdIso              :  1;           /*!< bits   5:  5 */
        UINT32 b1_SelOvHwaPdPonin            :  1;           /*!< bits   6:  6 */
        UINT32 b1_SelOvHwaPdPgoodin          :  1;           /*!< bits   7:  7 */
        UINT32 b1_SelOvHwaPdIso              :  1;           /*!< bits   8:  8 */
        UINT32 b1_SelOvTestDbgPdPonin        :  1;           /*!< bits   9:  9 */
        UINT32 b1_SelOvTestDbgPdPgoodin      :  1;           /*!< bits  10: 10 */
        UINT32 b1_SelOvTestDbgPdIso          :  1;           /*!< bits  11: 11 */
        UINT32 b20_Padfield_0                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_LOGIC_PSCON_OVERRIDE_SEL;

/*------------------------------------------------------------------------------
 * DEBUG_LOGIC_PSCON_OVERRIDE_VAL -- Offset == 0x1804
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_OvAppPdPonin               :  1;           /*!< bits   0:  0 */
        UINT32 b1_OvAppPdPgoodin             :  1;           /*!< bits   1:  1 */
        UINT32 b1_OvAppPdIso                 :  1;           /*!< bits   2:  2 */
        UINT32 b1_OvFecPdPonin               :  1;           /*!< bits   3:  3 */
        UINT32 b1_OvFecPdPgoodin             :  1;           /*!< bits   4:  4 */
        UINT32 b1_OvFecPdIso                 :  1;           /*!< bits   5:  5 */
        UINT32 b1_OvHwaPdPonin               :  1;           /*!< bits   6:  6 */
        UINT32 b1_OvHwaPdPgoodin             :  1;           /*!< bits   7:  7 */
        UINT32 b1_OvHwaPdIso                 :  1;           /*!< bits   8:  8 */
        UINT32 b1_OvTestDbgPdPonin           :  1;           /*!< bits   9:  9 */
        UINT32 b1_OvTestDbgPdPgoodin         :  1;           /*!< bits  10: 10 */
        UINT32 b1_OvTestDbgPdIso             :  1;           /*!< bits  11: 11 */
        UINT32 b20_Padfield_0                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_LOGIC_PSCON_OVERRIDE_VAL;

/*------------------------------------------------------------------------------
 * DEBUG_MEM_PSCON_OVERRIDE_SEL_1 -- Offset == 0x1808
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvAppPdMemAonin         :  3;           /*!< bits   2:  0 */
        UINT32 b3_SelOvAppPdMemAgoodin       :  3;           /*!< bits   5:  3 */
        UINT32 b2_SelOvAppPdMemGrp1Aonin     :  2;           /*!< bits   7:  6 */
        UINT32 b2_SelOvAppPdMemGrp1Agoodin   :  2;           /*!< bits   9:  8 */
        UINT32 b1_SelOvAppPdMemGrp2Aonin     :  1;           /*!< bits  10: 10 */
        UINT32 b1_SelOvAppPdMemGrp2Agoodin   :  1;           /*!< bits  11: 11 */
        UINT32 b3_SelOvAppPdMemDftrtaon      :  3;           /*!< bits  14: 12 */
        UINT32 b3_SelOvAppPdMemDftrtagood    :  3;           /*!< bits  17: 15 */
        UINT32 b2_SelOvAppPdMemGrp1Dftrtaon  :  2;           /*!< bits  19: 18 */
        UINT32 b2_SelOvAppPdMemGrp1Dftrtagood :  2;           /*!< bits  21: 20 */
        UINT32 b1_SelOvAppPdMemGrp2Dftrtaon  :  1;           /*!< bits  22: 22 */
        UINT32 b1_SelOvAppPdMemGrp2Dftrtagood :  1;           /*!< bits  23: 23 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_SEL_1;

/*------------------------------------------------------------------------------
 * DEBUG_MEM_PSCON_OVERRIDE_SEL_2 -- Offset == 0x180C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_SelOvHwaPdMemGrp3Aonin     :  3;           /*!< bits   2:  0 */
        UINT32 b3_SelOvHwaPdMemGrp3Agoodin   :  3;           /*!< bits   5:  3 */
        UINT32 b3_SelOvHwaPdMemGrp3Dftrtaon  :  3;           /*!< bits   8:  6 */
        UINT32 b3_SelOvHwaPdMemGrp3Dftrtagood :  3;           /*!< bits  11:  9 */
        UINT32 b1_SelOvFecPdMemAonin         :  1;           /*!< bits  12: 12 */
        UINT32 b1_SelOvFecPdMemAgoodin       :  1;           /*!< bits  13: 13 */
        UINT32 b2_SelOvFecPdMemGrp4Aonin     :  2;           /*!< bits  15: 14 */
        UINT32 b2_SelOvFecPdMemGrp4Agoodin   :  2;           /*!< bits  17: 16 */
        UINT32 b1_SelOvFecPdMemDftrtaon      :  1;           /*!< bits  18: 18 */
        UINT32 b1_SelOvFecPdMemDftrtagood    :  1;           /*!< bits  19: 19 */
        UINT32 b2_SelOvFecPdMemGrp4Dftrtaon  :  2;           /*!< bits  21: 20 */
        UINT32 b2_SelOvFecPdMemGrp4Dftrtagood :  2;           /*!< bits  23: 22 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_SEL_2;

/*------------------------------------------------------------------------------
 * DEBUG_MEM_PSCON_OVERRIDE_VAL_1 -- Offset == 0x1810
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_OvAppPdMemAonin            :  3;           /*!< bits   2:  0 */
        UINT32 b3_OvAppPdMemAgoodin          :  3;           /*!< bits   5:  3 */
        UINT32 b2_OvAppPdMemGrp1Aonin        :  2;           /*!< bits   7:  6 */
        UINT32 b2_OvAppPdMemGrp1Agoodin      :  2;           /*!< bits   9:  8 */
        UINT32 b1_OvAppPdMemGrp2Aonin        :  1;           /*!< bits  10: 10 */
        UINT32 b1_OvAppPdMemGrp2Agoodin      :  1;           /*!< bits  11: 11 */
        UINT32 b3_OvAppPdMemDftrtaon         :  3;           /*!< bits  14: 12 */
        UINT32 b3_OvAppPdMemDftrtagood       :  3;           /*!< bits  17: 15 */
        UINT32 b2_OvAppPdMemGrp1Dftrtaon     :  2;           /*!< bits  19: 18 */
        UINT32 b2_OvAppPdMemGrp1Dftrtagood   :  2;           /*!< bits  21: 20 */
        UINT32 b1_OvAppPdMemGrp2Dftrtaon     :  1;           /*!< bits  22: 22 */
        UINT32 b1_OvAppPdMemGrp2Dftrtagood   :  1;           /*!< bits  23: 23 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_VAL_1;

/*------------------------------------------------------------------------------
 * DEBUG_MEM_PSCON_OVERRIDE_VAL_2 -- Offset == 0x1814
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b3_OvHwaPdMemGrp3Aonin        :  3;           /*!< bits   2:  0 */
        UINT32 b3_OvHwaPdMemGrp3Agoodin      :  3;           /*!< bits   5:  3 */
        UINT32 b3_OvHwaPdMemGrp3Dftrtaon     :  3;           /*!< bits   8:  6 */
        UINT32 b3_OvHwaPdMemGrp3Dftrtagood   :  3;           /*!< bits  11:  9 */
        UINT32 b1_OvFecPdMemAonin            :  1;           /*!< bits  12: 12 */
        UINT32 b1_OvFecPdMemAgoodin          :  1;           /*!< bits  13: 13 */
        UINT32 b2_OvFecPdMemGrp4Aonin        :  2;           /*!< bits  15: 14 */
        UINT32 b2_OvFecPdMemGrp4Agoodin      :  2;           /*!< bits  17: 16 */
        UINT32 b1_OvFecPdMemDftrtaon         :  1;           /*!< bits  18: 18 */
        UINT32 b1_OvFecPdMemDftrtagood       :  1;           /*!< bits  19: 19 */
        UINT32 b2_OvFecPdMemGrp4Dftrtaon     :  2;           /*!< bits  21: 20 */
        UINT32 b2_OvFecPdMemGrp4Dftrtagood   :  2;           /*!< bits  23: 22 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_VAL_2;

/*------------------------------------------------------------------------------
 * MCUCLKOUT_CLKCTL -- Offset == 0x1C00
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_McuclkoutClkSwGate         :  4;           /*!< bits   3:  0 */
        UINT32 b12_McuclkoutClkSrcSel        : 12;           /*!< bits  15:  4 */
        UINT32 b12_McuclkoutClkDivr          : 12;           /*!< bits  27: 16 */
        UINT32 b4_Padfield_0                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_MCUCLKOUT_CLKCTL;

/*------------------------------------------------------------------------------
 * MCUCLKOUT_CLKSTAT -- Offset == 0x1C04
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_McuclkoutClkCurrDivr       :  4;           /*!< bits   3:  0 */
        UINT32 b8_McuclkoutClkInUse          :  8;           /*!< bits  11:  4 */
        UINT32 b20_Padfield_0                : 20;           /*!< bits  31: 12 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_MCUCLKOUT_CLKSTAT;

/*------------------------------------------------------------------------------
 * DCDC_CTRL_REG1 -- Offset == 0x1C08
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_DcdcRstnReg                :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b1_DcdcClkEn                  :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DCDC_CTRL_REG1;

/*------------------------------------------------------------------------------
 * DCDC_CTRL_REG2 -- Offset == 0x1C0C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_DcdcDitherEn               :  1;           /*!< bits   0:  0 */
        UINT32 b15_Padfield_0                : 15;           /*!< bits  15:  1 */
        UINT32 b1_DcdcFreqAccMode            :  1;           /*!< bits  16: 16 */
        UINT32 b15_Padfield_1                : 15;           /*!< bits  31: 17 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DCDC_CTRL_REG2;

/*------------------------------------------------------------------------------
 * DCDC_CTRL_REG3 -- Offset == 0x1C10
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_DcdcMinFreqThr             :  8;           /*!< bits   7:  0 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  15:  8 */
        UINT32 b8_DcdcMaxFreqThr             :  8;           /*!< bits  23: 16 */
        UINT32 b8_Padfield_1                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DCDC_CTRL_REG3;

/*------------------------------------------------------------------------------
 * DCDC_SLOPE_REG -- Offset == 0x1C14
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b27_DcdcSlopeVal              : 27;           /*!< bits  26:  0 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits  31: 27 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_DCDC_SLOPE_REG;

/*------------------------------------------------------------------------------
 * APP_CPU_CLKCTL -- Offset == 0x1C18
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_Gate                       :  4;           /*!< bits   3:  0 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_APP_CPU_CLKCTL;

/*------------------------------------------------------------------------------
 * VNWA_SWITCH_SCREEN_ENABLE -- Offset == 0x1C1C
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
} U_TOPPRCM_VNWA_SWITCH_SCREEN_ENABLE;

/*------------------------------------------------------------------------------
 * PLLDIG_RST_SRC_SEL -- Offset == 0x1C20
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Sel                        :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_PLLDIG_RST_SRC_SEL;

/*------------------------------------------------------------------------------
 * MEM_POWERDOWN_ACCESS_ERR_DIS -- Offset == 0x1C24
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_Disable                    :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_MEM_POWERDOWN_ACCESS_ERR_DIS;

/*------------------------------------------------------------------------------
 * MEM_SWAP -- Offset == 0x1C28
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FecCpuEclipseStatus        :  1;           /*!< bits   0:  0 */
        UINT32 b1_AppCpuEclipseStatus        :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_MEM_SWAP;

/*------------------------------------------------------------------------------
 * SPARE_REG -- Offset == 0x1C2C
 *------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_ReadWrite                  :  8;           /*!< bits   7:  0 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  15:  8 */
        UINT32 b8_ReadOnly                   :  8;           /*!< bits  23: 16 */
        UINT32 b8_Padfield_1                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_TOPPRCM_SPARE_REG;



/*------------------------------------------------------------------------------
 * TOP_PRCM_REGS
 *------------------------------------------------------------------------------
 */
typedef volatile struct
{
    U_TOPPRCM_PID                              r_Pid                         ; /* Offset=0x0000 */
    U_TOPPRCM_CLK_REQ_PARAM                    r_ClkReqParam                 ; /* Offset=0x0004 */
    U_TOPPRCM_APP_PWR_REQ_PARAM                r_AppPwrReqParam              ; /* Offset=0x0008 */
    U_TOPPRCM_FEC_PWR_REQ_PARAM                r_FecPwrReqParam              ; /* Offset=0x000C */
    U_TOPPRCM_HWA_PWR_REQ_PARAM                r_HwaPwrReqParam              ; /* Offset=0x0010 */
    U_TOPPRCM_APP_CORE_SYSRESET_PARAM          r_AppCoreSysresetParam        ; /* Offset=0x0014 */
    U_TOPPRCM_FEC_CORE_SYSRESET_PARAM          r_FecCoreSysresetParam        ; /* Offset=0x0018 */
    U_TOPPRCM_RELEASE_PAUSE                    r_ReleasePause                ; /* Offset=0x001C */
    U_TOPPRCM_WU_COUNTER_END                   r_WuCounterEnd                ; /* Offset=0x0020 */
    U_TOPPRCM_WU_COUNTER_START                 r_WuCounterStart              ; /* Offset=0x0024 */
    U_TOPPRCM_WU_COUNTER_PAUSE                 r_WuCounterPause              ; /* Offset=0x0028 */
    U_TOPPRCM_GTS_COUNTER_END                  r_GtsCounterEnd               ; /* Offset=0x002C */
    U_TOPPRCM_SLEEP_COUNTER_END                r_SleepCounterEnd             ; /* Offset=0x0030 */
    U_TOPPRCM_WU_SOURCE_EN                     r_WuSourceEn                  ; /* Offset=0x0034 */
    U_TOPPRCM_UART_RTS_CLEAR                   r_UartRtsClear                ; /* Offset=0x0038 */
    U_TOPPRCM_RADAR_WAKEUP_STATUS              r_RadarWakeupStatus           ; /* Offset=0x003C */
    U_TOPPRCM_RTC_COMPARE_LSB                  r_RtcCompareLsb               ; /* Offset=0x0040 */
    U_TOPPRCM_RTC_COMPARE_MSB                  r_RtcCompareMsb               ; /* Offset=0x0044 */
    U_TOPPRCM_RTC_COMPARE_EN                   r_RtcCompareEn                ; /* Offset=0x0048 */
    U_TOPPRCM_RTC_COUNT_LSB                    r_RtcCountLsb                 ; /* Offset=0x004C */
    U_TOPPRCM_RTC_COUNT_MSB                    r_RtcCountMsb                 ; /* Offset=0x0050 */
    U_TOPPRCM_PC_REGISTER1                     r_PcRegister1                 ; /* Offset=0x0054 */
    U_TOPPRCM_PC_REGISTER2                     r_PcRegister2                 ; /* Offset=0x0058 */
    U_TOPPRCM_PC_REGISTER3                     r_PcRegister3                 ; /* Offset=0x005C */
    U_TOPPRCM_PC_REGISTER4                     r_PcRegister4                 ; /* Offset=0x0060 */
    U_TOPPRCM_PC_REGISTER5                     r_PcRegister5                 ; /* Offset=0x0064 */
    U_TOPPRCM_PC_REGISTER6                     r_PcRegister6                 ; /* Offset=0x0068 */
    U_TOPPRCM_PC_REGISTER7                     r_PcRegister7                 ; /* Offset=0x006C */
    U_TOPPRCM_PC_REGISTER8                     r_PcRegister8                 ; /* Offset=0x0070 */
    U_TOPPRCM_WAKEUP_IO_MUX_SEL                r_WakeupIoMuxSel              ; /* Offset=0x0074 */
    U_TOPPRCM_WAKEUP_INT_SOURCE_EN             r_WakeupIntSourceEn           ; /* Offset=0x0078 */
    U_TOPPRCM_PMS_POWER_MODE                   r_PmsPowerMode                ; /* Offset=0x007C */
    U_TOPPRCM_PMS_SRAM_GO_TO_SLEEP_DELAY       r_PmsSramGoToSleepDelay       ; /* Offset=0x0080 */
    UINT32                                     r_Padreg0[223]                ; /* Offset=0x0084 */
    U_TOPPRCM_PMS_SRAM_GO_TO_SLEEP_TIME        r_PmsSramGoToSleepTime        ; /* Offset=0x0400 */
    U_TOPPRCM_PMS_SRAM_WAKEUP_DELAY            r_PmsSramWakeupDelay          ; /* Offset=0x0404 */
    U_TOPPRCM_PMS_SRAM_WAKEUP_TIME             r_PmsSramWakeupTime           ; /* Offset=0x0408 */
    U_TOPPRCM_PMS_SRAM_LDO_EN                  r_PmsSramLdoEn                ; /* Offset=0x040C */
    U_TOPPRCM_PMS_SLEEP_NO_RTA_CFG             r_PmsSleepNoRtaCfg            ; /* Offset=0x0410 */
    U_TOPPRCM_PMS_SRAM_LDO_TRIM                r_PmsSramLdoTrim              ; /* Offset=0x0414 */
    U_TOPPRCM_PMS_SRAM_KA_TRIM                 r_PmsSramKaTrim               ; /* Offset=0x0418 */
    U_TOPPRCM_PMS_DIG_GO_TO_SLEEP_DELAY        r_PmsDigGoToSleepDelay        ; /* Offset=0x041C */
    U_TOPPRCM_PMS_DIG_GO_TO_SLEEP_TIME         r_PmsDigGoToSleepTime         ; /* Offset=0x0420 */
    U_TOPPRCM_PMS_DIG_WAKEUP_DELAY             r_PmsDigWakeupDelay           ; /* Offset=0x0424 */
    U_TOPPRCM_PMS_DIG_WAKEUP_TIME              r_PmsDigWakeupTime            ; /* Offset=0x0428 */
    U_TOPPRCM_PMS_DIG_LDO_EN                   r_PmsDigLdoEn                 ; /* Offset=0x042C */
    U_TOPPRCM_PMS_DIG_LDO_TRIM                 r_PmsDigLdoTrim               ; /* Offset=0x0430 */
    U_TOPPRCM_PMS_DIG_KA_TRIM                  r_PmsDigKaTrim                ; /* Offset=0x0434 */
    U_TOPPRCM_PMS_PSCON_EN_WAIT_DELAY          r_PmsPsconEnWaitDelay         ; /* Offset=0x0438 */
    U_TOPPRCM_PMS_BGAP_DIS_HIBERNATE           r_PmsBgapDisHibernate         ; /* Offset=0x043C */
    U_TOPPRCM_PMS_BGAP_DELAY1                  r_PmsBgapDelay1               ; /* Offset=0x0440 */
    U_TOPPRCM_PMS_BGAP_DELAY2                  r_PmsBgapDelay2               ; /* Offset=0x0444 */
    U_TOPPRCM_PMS_BGAP_EN_OVERRIDE             r_PmsBgapEnOverride           ; /* Offset=0x0448 */
    U_TOPPRCM_PMS_BGAP_CAP_OVERRIDE1           r_PmsBgapCapOverride1         ; /* Offset=0x044C */
    U_TOPPRCM_PMS_BGAP_CAP_OVERRIDE2           r_PmsBgapCapOverride2         ; /* Offset=0x0450 */
    U_TOPPRCM_PSCON_APP_PD_EN                  r_PsconAppPdEn                ; /* Offset=0x0454 */
    U_TOPPRCM_PSCON_FEC_PD_EN                  r_PsconFecPdEn                ; /* Offset=0x0458 */
    U_TOPPRCM_PSCON_HWA_PD_EN                  r_PsconHwaPdEn                ; /* Offset=0x045C */
    U_TOPPRCM_PSCON_TEST_DBG_PD_EN             r_PsconTestDbgPdEn            ; /* Offset=0x0460 */
    U_TOPPRCM_PSCON_APP_PD_RAM_STATE           r_PsconAppPdRamState          ; /* Offset=0x0464 */
    U_TOPPRCM_PSCON_APP_PD_RAM_GRP1_STATE      r_PsconAppPdRamGrp1State      ; /* Offset=0x0468 */
    U_TOPPRCM_PSCON_APP_PD_RAM_GRP2_STATE      r_PsconAppPdRamGrp2State      ; /* Offset=0x046C */
    U_TOPPRCM_PSCON_HWA_PD_RAM_GRP3_STATE      r_PsconHwaPdRamGrp3State      ; /* Offset=0x0470 */
    U_TOPPRCM_PSCON_FEC_PD_RAM_STATE           r_PsconFecPdRamState          ; /* Offset=0x0474 */
    U_TOPPRCM_PSCON_FEC_PD_RAM_GRP4_STATE      r_PsconFecPdRamGrp4State      ; /* Offset=0x0478 */
    U_TOPPRCM_PSCON_RAM_FORCE_SWITCH_EN        r_PsconRamForceSwitchEn       ; /* Offset=0x047C */
    U_TOPPRCM_PSCON_RAM_SWITCH_EN_OVERRIDE1    r_PsconRamSwitchEnOverride1   ; /* Offset=0x0480 */
    U_TOPPRCM_PSCON_RAM_SWITCH_EN_OVERRIDE2    r_PsconRamSwitchEnOverride2   ; /* Offset=0x0484 */
    U_TOPPRCM_PSCOM_RAM_SWITCH_DELAY           r_PscomRamSwitchDelay         ; /* Offset=0x0488 */
    U_TOPPRCM_PSCON_DFTRTA_OVERRIDE            r_PsconDftrtaOverride         ; /* Offset=0x048C */
    U_TOPPRCM_PSCON_VNWA_SWITCH_EN1            r_PsconVnwaSwitchEn1          ; /* Offset=0x0490 */
    U_TOPPRCM_PSCON_VNWA_SWITCH_EN2            r_PsconVnwaSwitchEn2          ; /* Offset=0x0494 */
    U_TOPPRCM_PSCON_SRAM_LDO_WEAK_PROCESS      r_PsconSramLdoWeakProcess     ; /* Offset=0x0498 */
    U_TOPPRCM_CLKM_OSC_CLK_REQ                 r_ClkmOscClkReq               ; /* Offset=0x049C */
    U_TOPPRCM_CLKM_OVERRIDE1                   r_ClkmOverride1               ; /* Offset=0x04A0 */
    U_TOPPRCM_CLKM_OVERRIDE2                   r_ClkmOverride2               ; /* Offset=0x04A4 */
    U_TOPPRCM_CLKM_OVERRIDE3                   r_ClkmOverride3               ; /* Offset=0x04A8 */
    U_TOPPRCM_CLKM_OUTPUT_HOST_CLK_REQ_OVERRIDE r_ClkmOutputHostClkReqOverride; /* Offset=0x04AC */
    U_TOPPRCM_CLKM_SEL_OV_XT_DRIVE             r_ClkmSelOvXtDrive            ; /* Offset=0x04B0 */
    U_TOPPRCM_CLKM_DELAY1                      r_ClkmDelay1                  ; /* Offset=0x04B4 */
    U_TOPPRCM_CLKM_DELAY2                      r_ClkmDelay2                  ; /* Offset=0x04B8 */
    U_TOPPRCM_CLKM_HOST_CLK_REQ_DELAY          r_ClkmHostClkReqDelay         ; /* Offset=0x04BC */
    U_TOPPRCM_RST_OVERRIDE                     r_RstOverride                 ; /* Offset=0x04C0 */
    U_TOPPRCM_RST_SOFT_RESET                   r_RstSoftReset                ; /* Offset=0x04C4 */
    U_TOPPRCM_RST_WDT_RESET_EN                 r_RstWdtResetEn               ; /* Offset=0x04C8 */
    U_TOPPRCM_RST_APP_PD_SOFT_RESET            r_RstAppPdSoftReset           ; /* Offset=0x04CC */
    U_TOPPRCM_RST_FEC_PD_SOFT_RESET            r_RstFecPdSoftReset           ; /* Offset=0x04D0 */
    U_TOPPRCM_RST_HWA_PD_SOFT_RESET            r_RstHwaPdSoftReset           ; /* Offset=0x04D4 */
    U_TOPPRCM_RST_SOFT_APP_CORE_SYSRESET_REQ   r_RstSoftAppCoreSysresetReq   ; /* Offset=0x04D8 */
    U_TOPPRCM_RST_SOFT_FEC_CORE_SYSRESET_REQ   r_RstSoftFecCoreSysresetReq   ; /* Offset=0x04DC */
    U_TOPPRCM_SYS_RST_CAUSE                    r_SysRstCause                 ; /* Offset=0x04E0 */
    U_TOPPRCM_DUBUGSS_DISABLE                  r_DubugssDisable              ; /* Offset=0x04E4 */
    U_TOPPRCM_SLOW_CLK_CLKCTL                  r_SlowClkClkctl               ; /* Offset=0x04E8 */
    U_TOPPRCM_DEBUGSS_CLK_CLKCTL               r_DebugssClkClkctl            ; /* Offset=0x04EC */
    U_TOPPRCM_EFUSE_10M_OSC_DISABLE            r_Efuse_10mOscDisable         ; /* Offset=0x04F0 */
    U_TOPPRCM_DEBUGSS_CLK_AUTOSWITCH           r_DebugssClkAutoswitch        ; /* Offset=0x04F4 */
    U_TOPPRCM_RADAR_SAFTY_ERROR_REG            r_RadarSaftyErrorReg          ; /* Offset=0x04F8 */
    U_TOPPRCM_RELEASEFROMWIR_REG               r_ReleasefromwirReg           ; /* Offset=0x04FC */
    U_TOPPRCM_HWA_PD_MEM_SHARE_REG             r_HwaPdMemShareReg            ; /* Offset=0x0500 */
    U_TOPPRCM_FRC_OSC_CLK_GATE                 r_FrcOscClkGate               ; /* Offset=0x0504 */
    U_TOPPRCM_MEMSWAP_REG                      r_MemswapReg                  ; /* Offset=0x0508 */
    U_TOPPRCM_LIMP_MODE_STATUS                 r_LimpModeStatus              ; /* Offset=0x050C */
    U_TOPPRCM_RTI_CLOCK_GATE_SLEEP_STATE       r_RtiClockGateSleepState      ; /* Offset=0x0510 */
    U_TOPPRCM_TOP_3318_LDO_EN_CTRL             r_Top_3318LdoEnCtrl           ; /* Offset=0x0514 */
    U_TOPPRCM_RFANA_TOP_LDO_EN                 r_RfanaTopLdoEn               ; /* Offset=0x0518 */
    U_TOPPRCM_RFANA_TOP_ISO_CTRL               r_RfanaTopIsoCtrl             ; /* Offset=0x051C */
    U_TOPPRCM_CLK_CTRL_REG1_LDO_CLKTOP         r_ClkCtrlReg1LdoClktop        ; /* Offset=0x0520 */
    U_TOPPRCM_CLK_CTRL_REG1_XO_SLICER          r_ClkCtrlReg1XoSlicer         ; /* Offset=0x0524 */
    U_TOPPRCM_TOP_LDO_3318_CTRL_REG0           r_TopLdo_3318CtrlReg0         ; /* Offset=0x0528 */
    U_TOPPRCM_TOP_LDO_3318_CTRL_REG1           r_TopLdo_3318CtrlReg1         ; /* Offset=0x052C */
    U_TOPPRCM_TOP_LDO_DIG_CTRL_REG0            r_TopLdoDigCtrlReg0           ; /* Offset=0x0530 */
    U_TOPPRCM_TOP_LDO_DIG_CTRL_REG1            r_TopLdoDigCtrlReg1           ; /* Offset=0x0534 */
    U_TOPPRCM_TOP_LDO_SRAM_CTRL_REG0           r_TopLdoSramCtrlReg0          ; /* Offset=0x0538 */
    U_TOPPRCM_TOP_LDO_SRAM_CTRL_REG1           r_TopLdoSramCtrlReg1          ; /* Offset=0x053C */
    UINT32                                     r_Padreg1[690]                ; /* Offset=0x0540 */
    U_TOPPRCM_LOCK0_KICK0                      r_Lock0Kick0                  ; /* Offset=0x1008 */
    U_TOPPRCM_LOCK0_KICK1                      r_Lock0Kick1                  ; /* Offset=0x100C */
    U_TOPPRCM_INTR_RAW_STATUS                  r_IntrRawStatus               ; /* Offset=0x1010 */
    U_TOPPRCM_INTR_ENABLED_STATUS_CLEAR        r_IntrEnabledStatusClear      ; /* Offset=0x1014 */
    U_TOPPRCM_INTR_ENABLE                      r_IntrEnable                  ; /* Offset=0x1018 */
    U_TOPPRCM_INTR_ENABLE_CLEAR                r_IntrEnableClear             ; /* Offset=0x101C */
    U_TOPPRCM_EOI                              r_Eoi                         ; /* Offset=0x1020 */
    U_TOPPRCM_FAULT_ADDRESS                    r_FaultAddress                ; /* Offset=0x1024 */
    U_TOPPRCM_FAULT_TYPE_STATUS                r_FaultTypeStatus             ; /* Offset=0x1028 */
    U_TOPPRCM_FAULT_ATTR_STATUS                r_FaultAttrStatus             ; /* Offset=0x102C */
    U_TOPPRCM_FAULT_CLEAR                      r_FaultClear                  ; /* Offset=0x1030 */
    UINT32                                     r_Padreg2[499]                ; /* Offset=0x1034 */
    U_TOPPRCM_DEBUG_LOGIC_PSCON_OVERRIDE_SEL   r_DebugLogicPsconOverrideSel  ; /* Offset=0x1800 */
    U_TOPPRCM_DEBUG_LOGIC_PSCON_OVERRIDE_VAL   r_DebugLogicPsconOverrideVal  ; /* Offset=0x1804 */
    U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_SEL_1   r_DebugMemPsconOverrideSel_1  ; /* Offset=0x1808 */
    U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_SEL_2   r_DebugMemPsconOverrideSel_2  ; /* Offset=0x180C */
    U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_VAL_1   r_DebugMemPsconOverrideVal_1  ; /* Offset=0x1810 */
    U_TOPPRCM_DEBUG_MEM_PSCON_OVERRIDE_VAL_2   r_DebugMemPsconOverrideVal_2  ; /* Offset=0x1814 */
    UINT32                                     r_Padreg3[250]                ; /* Offset=0x1818 */
    U_TOPPRCM_MCUCLKOUT_CLKCTL                 r_McuclkoutClkctl             ; /* Offset=0x1C00 */
    U_TOPPRCM_MCUCLKOUT_CLKSTAT                r_McuclkoutClkstat            ; /* Offset=0x1C04 */
    U_TOPPRCM_DCDC_CTRL_REG1                   r_DcdcCtrlReg1                ; /* Offset=0x1C08 */
    U_TOPPRCM_DCDC_CTRL_REG2                   r_DcdcCtrlReg2                ; /* Offset=0x1C0C */
    U_TOPPRCM_DCDC_CTRL_REG3                   r_DcdcCtrlReg3                ; /* Offset=0x1C10 */
    U_TOPPRCM_DCDC_SLOPE_REG                   r_DcdcSlopeReg                ; /* Offset=0x1C14 */
    U_TOPPRCM_APP_CPU_CLKCTL                   r_AppCpuClkctl                ; /* Offset=0x1C18 */
    U_TOPPRCM_VNWA_SWITCH_SCREEN_ENABLE        r_VnwaSwitchScreenEnable      ; /* Offset=0x1C1C */
    U_TOPPRCM_PLLDIG_RST_SRC_SEL               r_PlldigRstSrcSel             ; /* Offset=0x1C20 */
    U_TOPPRCM_MEM_POWERDOWN_ACCESS_ERR_DIS     r_MemPowerdownAccessErrDis    ; /* Offset=0x1C24 */
    U_TOPPRCM_MEM_SWAP                         r_MemSwap                     ; /* Offset=0x1C28 */
    U_TOPPRCM_SPARE_REG                        r_SpareReg                    ; /* Offset=0x1C2C */
} T_TOPPRCM_REGS;


#endif
/*
 * END OF REG_TOPPRCM_H
 */

