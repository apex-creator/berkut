/*!*****************************************************************************
 * @file reg_chirptimer.h
 *
 * @brief This file gives register definitions of TIMING_ENGINE module
 *
 * @b Description @n
 * This file gives register definitions of TIMING_ENGINE module
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
#ifndef REG_CT_H
#define REG_CT_H

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
} U_CT_PID;

/*---------------------------------------------------------------------------------------
 * CHIRP_TIMER_TRIG -- Offset == 0x004
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ChirpTimerTrigSrcSel       :  1;           /*!< bits   0:  0 */
        UINT32 b3_ChirpTimerTrigReg          :  3;           /*!< bits   3:  1 */
        UINT32 b3_ChirpTimerStopReg          :  3;           /*!< bits   6:  4 */
        UINT32 b1_ChirpTimerCwCzEn           :  1;           /*!< bits   7:  7 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CHIRP_TIMER_TRIG;

/*---------------------------------------------------------------------------------------
 * NUM_CHIRPS_AND_BURSTS -- Offset == 0x008
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_NumChirpsPerBurst         : 16;           /*!< bits  15:  0 */
        UINT32 b16_NumBurstsPerFrame         : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_NUM_CHIRPS_AND_BURSTS;

/*---------------------------------------------------------------------------------------
 * CHIRP_TIMER_PROG_RESOLUTION -- Offset == 0x00C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_FreqStartRes               :  1;           /*!< bits   0:  0 */
        UINT32 b1_TimingParamsRes            :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CHIRP_TIMER_PROG_RESOLUTION;

/*---------------------------------------------------------------------------------------
 * FREQ_START -- Offset == 0x010
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b24_FreqStart                 : 24;           /*!< bits  23:  0 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_FREQ_START;

/*---------------------------------------------------------------------------------------
 * FREQ_SLOPE -- Offset == 0x014
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_FreqSlope                 : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_FREQ_SLOPE;

/*---------------------------------------------------------------------------------------
 * IDLE_TIME -- Offset == 0x018
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_IdleTime                  : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_IDLE_TIME;

/*---------------------------------------------------------------------------------------
 * RAMP_END_TIME -- Offset == 0x01C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_RampEndTime               : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAMP_END_TIME;

/*---------------------------------------------------------------------------------------
 * ADC_START_TIME -- Offset == 0x020
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b10_AdcStartTimeFrac          : 10;           /*!< bits   9:  0 */
        UINT32 b6_NumSkipSamples             :  6;           /*!< bits  15: 10 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_ADC_START_TIME;

/*---------------------------------------------------------------------------------------
 * TX_START_TIME -- Offset == 0x024
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_TxStartTime               : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_TX_START_TIME;

/*---------------------------------------------------------------------------------------
 * TX_EN_BPM_SETTINGS -- Offset == 0x028
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b4_TxEn                       :  4;           /*!< bits   3:  0 */
        UINT32 b4_TxBpm                      :  4;           /*!< bits   7:  4 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_TX_EN_BPM_SETTINGS;

/*---------------------------------------------------------------------------------------
 * SYNTH_FREQ_MON_SETTINGS -- Offset == 0x02C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_SynthFreqMonEn             :  1;           /*!< bits   0:  0 */
        UINT32 b12_SynthFreqMonStartTime     : 12;           /*!< bits  12:  1 */
        UINT32 b12_SynthFreqMonResetSoccAccumTime : 12;           /*!< bits  24: 13 */
        UINT32 b7_Padfield_0                 :  7;           /*!< bits  31: 25 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_SYNTH_FREQ_MON_SETTINGS;

/*---------------------------------------------------------------------------------------
 * TX_MIMO_PATTERN -- Offset == 0x030
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_TxMimoPattern              :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_TX_MIMO_PATTERN;

/*---------------------------------------------------------------------------------------
 * BURST_PERIODICITY -- Offset == 0x034
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b24_BurstPeriodicity          : 24;           /*!< bits  23:  0 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_BURST_PERIODICITY;

/*---------------------------------------------------------------------------------------
 * BURST_START_OFFSET_TIME -- Offset == 0x038
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_BurstStartOffsetTime      : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_BURST_START_OFFSET_TIME;

/*---------------------------------------------------------------------------------------
 * CHIRP_ACCUM_SETTINGS -- Offset == 0x03C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_NumChirpAccum              :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CHIRP_ACCUM_SETTINGS;

/*---------------------------------------------------------------------------------------
 * MISCELLENIOUS_SETTINGS -- Offset == 0x040
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_DisableSynthFastReset1     :  1;           /*!< bits   0:  0 */
        UINT32 b1_DisableSynthFastReset2     :  1;           /*!< bits   1:  1 */
        UINT32 b1_DisableSynthFastReset3     :  1;           /*!< bits   2:  2 */
        UINT32 b1_HpfFastInitEn              :  1;           /*!< bits   3:  3 */
        UINT32 b28_Padfield_0                : 28;           /*!< bits  31:  4 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_MISCELLENIOUS_SETTINGS;

/*---------------------------------------------------------------------------------------
 * CONTROLLED_RAMP_DOWN_SETTINGS_1 -- Offset == 0x044
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_CrdEn                      :  1;           /*!< bits   0:  0 */
        UINT32 b1_CrdDitherEn                :  1;           /*!< bits   1:  1 */
        UINT32 b1_CrdLfsrLoadEn              :  1;           /*!< bits   2:  2 */
        UINT32 b24_CrdLfsrSeed               : 24;           /*!< bits  26:  3 */
        UINT32 b5_Padfield_0                 :  5;           /*!< bits  31: 27 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_1;

/*---------------------------------------------------------------------------------------
 * CONTROLLED_RAMP_DOWN_SETTINGS_2 -- Offset == 0x048
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b15_CrdNslopeMag              : 15;           /*!< bits  14:  0 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits  15: 15 */
        UINT32 b10_CrdFreqOffsetTime         : 10;           /*!< bits  25: 16 */
        UINT32 b6_Padfield_1                 :  6;           /*!< bits  31: 26 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_2;

/*---------------------------------------------------------------------------------------
 * CONTROLLED_RAMP_DOWN_SETTINGS_3 -- Offset == 0x04C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_CrdMaxFreqOffsetDither    : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_3;

/*---------------------------------------------------------------------------------------
 * AFE_POWER_SAVE_EN_REG -- Offset == 0x050
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_AfePowerSaveEnPaTx         :  1;           /*!< bits   0:  0 */
        UINT32 b1_AfePowerSaveEnLoTx         :  1;           /*!< bits   1:  1 */
        UINT32 b1_AfePowerSaveEnRfRx         :  1;           /*!< bits   2:  2 */
        UINT32 b1_AfePowerSaveEnBbRx         :  1;           /*!< bits   3:  3 */
        UINT32 b1_AfePowerSaveEnLoRx         :  1;           /*!< bits   4:  4 */
        UINT32 b1_InterburstAfePowerSaveEnPaTx :  1;           /*!< bits   5:  5 */
        UINT32 b1_InterburstAfePowerSaveEnLoTx :  1;           /*!< bits   6:  6 */
        UINT32 b1_InterburstAfePowerSaveEnRfRx :  1;           /*!< bits   7:  7 */
        UINT32 b1_InterburstAfePowerSaveEnBbRx :  1;           /*!< bits   8:  8 */
        UINT32 b1_InterburstAfePowerSaveEnLoRx :  1;           /*!< bits   9:  9 */
        UINT32 b22_Padfield_0                : 22;           /*!< bits  31: 10 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_AFE_POWER_SAVE_EN_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_OVERRIDE_REG -- Offset == 0x054
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_RapidEnPaTxallOverride     :  1;           /*!< bits   0:  0 */
        UINT32 b4_RapidEnLoTxOverride        :  4;           /*!< bits   4:  1 */
        UINT32 b4_RapidEnRfRxOverride        :  4;           /*!< bits   8:  5 */
        UINT32 b4_RapidEnBbRxOverride        :  4;           /*!< bits  12:  9 */
        UINT32 b1_RapidEnLoRxallOverride     :  1;           /*!< bits  13: 13 */
        UINT32 b14_RapidEnOverrideSel        : 14;           /*!< bits  27: 14 */
        UINT32 b4_Padfield_0                 :  4;           /*!< bits  31: 28 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_OVERRIDE_REG;

/*---------------------------------------------------------------------------------------
 * COUNTER -- Offset == 0x058
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_Counter                   : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_COUNTER;

/*---------------------------------------------------------------------------------------
 * CHIRP_CNT -- Offset == 0x05C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_ChirpCnt                  : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CHIRP_CNT;

/*---------------------------------------------------------------------------------------
 * BURST_PERIOD_TIMER -- Offset == 0x060
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_BurstPeriodTimer          : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_BURST_PERIOD_TIMER;

/*---------------------------------------------------------------------------------------
 * BURST_CNT -- Offset == 0x064
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_BurstCnt                  : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_BURST_CNT;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_PA_TX_RX_LO_TIME_REG -- Offset == 0x068
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnPaTxallEndTime      :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnLoRxallStartTime    :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnLoRxallEndTime      :  8;           /*!< bits  23: 16 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_PA_TX_RX_LO_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_LO_TX_START_TIME_REG -- Offset == 0x06C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnLoTx1StartTime      :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnLoTx2StartTime      :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnLoTx3StartTime      :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnLoTx4StartTime      :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_LO_TX_START_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_LO_TX_END_TIME_REG -- Offset == 0x070
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnLoTx1EndTime        :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnLoTx2EndTime        :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnLoTx3EndTime        :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnLoTx4EndTime        :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_LO_TX_END_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_RF_RX_START_TIME_REG -- Offset == 0x074
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnRfRx1StartTime      :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnRfRx2StartTime      :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnRfRx3StartTime      :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnRfRx4StartTime      :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_RF_RX_START_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_RF_RX_END_TIME_REG -- Offset == 0x078
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnRfRx1EndTime        :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnRfRx2EndTime        :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnRfRx3EndTime        :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnRfRx4EndTime        :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_RF_RX_END_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_BB_RX_START_TIME_REG -- Offset == 0x07C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnBbRx1StartTime      :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnBbRx2StartTime      :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnBbRx3StartTime      :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnBbRx4StartTime      :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_BB_RX_START_TIME_REG;

/*---------------------------------------------------------------------------------------
 * RAPID_EN_BB_RX_END_TIME_REG -- Offset == 0x080
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_RapidEnBbRx1EndTime        :  8;           /*!< bits   7:  0 */
        UINT32 b8_RapidEnBbRx2EndTime        :  8;           /*!< bits  15:  8 */
        UINT32 b8_RapidEnBbRx3EndTime        :  8;           /*!< bits  23: 16 */
        UINT32 b8_RapidEnBbRx4EndTime        :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_RAPID_EN_BB_RX_END_TIME_REG;

/*---------------------------------------------------------------------------------------
 * ADC_TIME_REG -- Offset == 0x084
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_AdcAnaEnRxallStartTime     :  8;           /*!< bits   7:  0 */
        UINT32 b8_AdcAnaEnRxallEndTime       :  8;           /*!< bits  15:  8 */
        UINT32 b8_AdcIntRstzEnRxallStartTime :  8;           /*!< bits  23: 16 */
        UINT32 b8_AdcIntRstzEnRxallEndTime   :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_ADC_TIME_REG;

/*---------------------------------------------------------------------------------------
 * HPF_FAST_INIT_SYNTH_FAST_RESET_TIMING_1_REG -- Offset == 0x088
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_HpfFastInitStartTime       :  8;           /*!< bits   7:  0 */
        UINT32 b8_HpfFastInitPulseWidth      :  8;           /*!< bits  15:  8 */
        UINT32 b8_SynthFastResetStartTime1   :  8;           /*!< bits  23: 16 */
        UINT32 b8_SynthFastResetEndTime1     :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HPF_FAST_INIT_SYNTH_TIMING_1_REG;

/*---------------------------------------------------------------------------------------
 * SYNTH_FAST_RESET_TIMING_2_3_REG -- Offset == 0x08C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_SynthFastResetStartTime2   :  8;           /*!< bits   7:  0 */
        UINT32 b8_SynthFastResetEndTime2     :  8;           /*!< bits  15:  8 */
        UINT32 b8_SynthFastResetStartTime3   :  8;           /*!< bits  23: 16 */
        UINT32 b8_SynthFastResetEndTime3     :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_SYNTH_FAST_RESET_TIMING_2_3_REG;

/*---------------------------------------------------------------------------------------
 * SOCC_TIMING_REG -- Offset == 0x090
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_ResetSoccStartTime         :  8;           /*!< bits   7:  0 */
        UINT32 b8_ResetSoccEndTime           :  8;           /*!< bits  15:  8 */
        UINT32 b8_SoccTestClkEnStartTime     :  8;           /*!< bits  23: 16 */
        UINT32 b8_SoccTestClkEnEndTime       :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_SOCC_TIMING_REG;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_EN -- Offset == 0x094
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_PerchirpFreqStartEn        :  1;           /*!< bits   0:  0 */
        UINT32 b1_PerchirpFreqSlopeEn        :  1;           /*!< bits   1:  1 */
        UINT32 b1_PerchirpIdleTimeEn         :  1;           /*!< bits   2:  2 */
        UINT32 b1_PerchirpAdcStartTimeEn     :  1;           /*!< bits   3:  3 */
        UINT32 b1_PerchirpTxStartTimeEn      :  1;           /*!< bits   4:  4 */
        UINT32 b1_PerchirpTxEnEn             :  1;           /*!< bits   5:  5 */
        UINT32 b1_PerchirpTxBpmEn            :  1;           /*!< bits   6:  6 */
        UINT32 b25_Padfield_0                : 25;           /*!< bits  31:  7 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_EN;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_ADDR1 -- Offset == 0x098
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpFreqStartAddr     : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpFreqSlopeAddr     : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_ADDR1;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_ADDR2 -- Offset == 0x09C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpIdleTimeAddr      : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpAdcStartTimeAddr  : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_ADDR2;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_ADDR3 -- Offset == 0x0A0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxStartTimeAddr   : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpTxEnAddr          : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_ADDR3;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_ADDR4 -- Offset == 0x0A4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxBpmAddr         : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpSatIndicationAddr : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_ADDR4;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_LEN1 -- Offset == 0x0A8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpFreqStartLen      : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpFreqSlopeLen      : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_LEN1;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_LEN2 -- Offset == 0x0AC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpIdleTimeLen       : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpAdcStartTimeLen   : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_LEN2;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_LEN3 -- Offset == 0x0B0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxStartTimeLen    : 16;           /*!< bits  15:  0 */
        UINT32 b16_PerchirpTxEnLen           : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_LEN3;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_PARAMETER_LEN4 -- Offset == 0x0B4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxBpmLen          : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_PARAMETER_LEN4;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_FREQ_START_REP_CNT -- Offset == 0x0B8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpFreqStartRepCnt   : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_FREQ_START_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_FREQ_SLOPE_REP_CNT -- Offset == 0x0BC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpFreqSlopeRepCnt   : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_FREQ_SLOPE_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_IDLE_TIME_REP_CNT -- Offset == 0x0C0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpIdleTimeRepCnt    : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_IDLE_TIME_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_ADC_START_TIME_REP_CNT -- Offset == 0x0C4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpAdcStartTimeRepCnt : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_ADC_START_TIME_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_TX_START_TIME_REP_CNT -- Offset == 0x0C8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxStartTimeRepCnt : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_TX_START_TIME_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_TX_EN_REP_CNT -- Offset == 0x0CC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxEnRepCnt        : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_TX_EN_REP_CNT;

/*---------------------------------------------------------------------------------------
 * PERCHIRP_TX_BPM_REP_CNT -- Offset == 0x0D0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_PerchirpTxBpmRepCnt       : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PERCHIRP_TX_BPM_REP_CNT;

/*---------------------------------------------------------------------------------------
 * CHIRP_TIMER_TRIG_STOP_DONE -- Offset == 0x0D4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ChirpTimerTrigDone         :  1;           /*!< bits   0:  0 */
        UINT32 b1_ChirpTimerStopDone         :  1;           /*!< bits   1:  1 */
        UINT32 b30_Padfield_0                : 30;           /*!< bits  31:  2 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_CHIRP_TIMER_TRIG_STOP_DONE;

/*---------------------------------------------------------------------------------------
 * PARITY_CFG_REG -- Offset == 0x0D8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ParamRamAddrCntrParityAndOutputMask :  1;           /*!< bits   0:  0 */
        UINT32 b1_ParamRamAddrCntrParityOrOutputMask :  1;           /*!< bits   1:  1 */
        UINT32 b1_ParamRamMemParityOutputMask :  1;           /*!< bits   2:  2 */
        UINT32 b1_ParityInversionEnable      :  1;           /*!< bits   3:  3 */
        UINT32 b1_ParamRamMemParityCompDis   :  1;           /*!< bits   4:  4 */
        UINT32 b1_ParamRamAddrCntrParityCompDis :  1;           /*!< bits   5:  5 */
        UINT32 b1_ChirpTimerUndefinedStateMask :  1;           /*!< bits   6:  6 */
        UINT32 b1_PerchirpMemIntrpUndefinedStateMask :  1;           /*!< bits   7:  7 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PARITY_CFG_REG;

/*---------------------------------------------------------------------------------------
 * PARITY_STATUS -- Offset == 0x0DC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_ParityParamRamNotEqual     :  1;           /*!< bits   0:  0 */
        UINT32 b1_ParityFreqStartAddrNotEqual :  1;           /*!< bits   1:  1 */
        UINT32 b1_ParityFreqSlopeAddrNotEqual :  1;           /*!< bits   2:  2 */
        UINT32 b1_ParityIdleTimeAddrNotEqual :  1;           /*!< bits   3:  3 */
        UINT32 b1_ParityTxStartTimeAddrNotEqual :  1;           /*!< bits   4:  4 */
        UINT32 b1_ParityAdcStartTimeAddrNotEqual :  1;           /*!< bits   5:  5 */
        UINT32 b1_ParityTxEnAddrNotEqual     :  1;           /*!< bits   6:  6 */
        UINT32 b1_ParityTxBpmAddrNotEqual    :  1;           /*!< bits   7:  7 */
        UINT32 b1_ParitySaturationIndicationAddrNotEqual :  1;           /*!< bits   8:  8 */
        UINT32 b1_ChirpTimerUndefinedStateEntered :  1;           /*!< bits   9:  9 */
        UINT32 b1_PerchirpMemUndefinedStateEntered :  1;           /*!< bits  10: 10 */
        UINT32 b21_Padfield_0                : 21;           /*!< bits  31: 11 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PARITY_STATUS;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG0 -- Offset == 0x0E0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg0               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG0;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG1 -- Offset == 0x0E4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg1               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG1;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG2 -- Offset == 0x0E8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg2               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG2;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG3 -- Offset == 0x0EC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg3               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG3;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG4 -- Offset == 0x0F0
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg4               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG4;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG5 -- Offset == 0x0F4
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg5               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG5;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG6 -- Offset == 0x0F8
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg6               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG6;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG7 -- Offset == 0x0FC
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg7               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG7;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG8 -- Offset == 0x100
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg8               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG8;

/*---------------------------------------------------------------------------------------
 * HW_SPARE_REG9 -- Offset == 0x104
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_HwSpareReg9               : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_HW_SPARE_REG9;

/*---------------------------------------------------------------------------------------
 * TAIL_COUNTING_TIME -- Offset == 0x108
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b8_TailCountingTime           :  8;           /*!< bits   7:  0 */
        UINT32 b24_Padfield_0                : 24;           /*!< bits  31:  8 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_TAIL_COUNTING_TIME;

/*---------------------------------------------------------------------------------------
 * MISC_RO_REGS -- Offset == 0x10C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_NextChirpStartFreqRo      : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_MISC_RO_REGS;

/*---------------------------------------------------------------------------------------
 * SYNTH_LOCK_STARTSLOPE_DELAY -- Offset == 0x110
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b16_SynthLockStartslopeDelay  : 16;           /*!< bits  15:  0 */
        UINT32 b16_Padfield_0                : 16;           /*!< bits  31: 16 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_SYNTH_LOCK_STARTSLOPE_DELAY;

/*---------------------------------------------------------------------------------------
 * PARAM_WAIT_UPDATE_CYCLES -- Offset == 0x114
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b6_ParamUpdateCycles          :  6;           /*!< bits   5:  0 */
        UINT32 b1_Padfield_0                 :  1;           /*!< bits   6:  6 */
        UINT32 b6_ParamWaitCycles            :  6;           /*!< bits  12:  7 */
        UINT32 b19_Padfield_1                : 19;           /*!< bits  31: 13 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PARAM_WAIT_UPDATE_CYCLES;

/*---------------------------------------------------------------------------------------
 * PA_BLANKING -- Offset == 0x118
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_EnPaBlankingFeature        :  1;           /*!< bits   0:  0 */
        UINT32 b1_EnDfePaBlanking            :  1;           /*!< bits   1:  1 */
        UINT32 b1_PaBlankingStatus           :  1;           /*!< bits   2:  2 */
        UINT32 b29_Padfield_0                : 29;           /*!< bits  31:  3 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_PA_BLANKING;

/*---------------------------------------------------------------------------------------
 * N_START_OVERRIDE_MUX_SEL -- Offset == 0x11C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_NStartOverrideMuxSel       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_START_OVERRIDE_MUX_SEL;

/*---------------------------------------------------------------------------------------
 * N_SLOPE_OVERRIDE_MUX_SEL -- Offset == 0x120
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_NSlopeOverrideMuxSel       :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_SLOPE_OVERRIDE_MUX_SEL;

/*---------------------------------------------------------------------------------------
 * N_STARTSLOPE_PULSE_OVERRIDE_MUX_SEL -- Offset == 0x124
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_NStartslopePulseOverrideMuxSel :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_STARTSLOPE_PULSE_OVERD_MUX_SEL;

/*---------------------------------------------------------------------------------------
 * N_START_OVERRIDE_VALUE -- Offset == 0x128
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b32_NStartOverrideValue       : 32;           /*!< bits  31:  0 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_START_OVERRIDE_VALUE;

/*---------------------------------------------------------------------------------------
 * N_SLOPE_OVERRIDE_VALUE -- Offset == 0x12C
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b22_NSlopeOverrideValue       : 22;           /*!< bits  21:  0 */
        UINT32 b10_Padfield_0                : 10;           /*!< bits  31: 22 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_SLOPE_OVERRIDE_VALUE;

/*---------------------------------------------------------------------------------------
 * N_STARTSLOPE_OVERRIDE_VALUE -- Offset == 0x130
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_NStartslopeOverrideValue   :  1;           /*!< bits   0:  0 */
        UINT32 b31_Padfield_0                : 31;           /*!< bits  31:  1 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_N_STARTSLOPE_OVERRIDE_VALUE;

/*---------------------------------------------------------------------------------------
 * MISC_OVERRIDE -- Offset == 0x134
 *---------------------------------------------------------------------------------------
 */
typedef union
{
    struct
    {
        UINT32 b1_RxHpfFastInitOverride      :  1;           /*!< bits   0:  0 */
        UINT32 b1_ResetSoccOverride          :  1;           /*!< bits   1:  1 */
        UINT32 b1_RxDfeResetOverride         :  1;           /*!< bits   2:  2 */
        UINT32 b1_AdcIntRstzRxallOverride    :  1;           /*!< bits   3:  3 */
        UINT32 b1_AdcAnaEnRxallOverride      :  1;           /*!< bits   4:  4 */
        UINT32 b4_TxBpmOverride              :  4;           /*!< bits   8:  5 */
        UINT32 b3_SynthFastResetEnOverride   :  3;           /*!< bits  11:  9 */
        UINT32 b1_RxHpfFastInitOverrideSel   :  1;           /*!< bits  12: 12 */
        UINT32 b1_ResetSoccOverrideSel       :  1;           /*!< bits  13: 13 */
        UINT32 b1_RxDfeResetOverrideSel      :  1;           /*!< bits  14: 14 */
        UINT32 b1_AdcIntRstzRxallOverrideSel :  1;           /*!< bits  15: 15 */
        UINT32 b1_AdcAnaEnRxallOverrideSel   :  1;           /*!< bits  16: 16 */
        UINT32 b4_TxBpmOverrideSel           :  4;           /*!< bits  20: 17 */
        UINT32 b3_SynthFastResetEnOverrideSel :  3;           /*!< bits  23: 21 */
        UINT32 b8_Padfield_0                 :  8;           /*!< bits  31: 24 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_CT_MISC_OVERRIDE;

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
} U_CT_LOCK0_KICK0;

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
} U_CT_LOCK0_KICK1;

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
} U_CT_INTR_RAW_STATUS;

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
} U_CT_INTR_ENABLED_STATUS_CLEAR;

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
} U_CT_INTR_ENABLE;

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
} U_CT_INTR_ENABLE_CLEAR;

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
} U_CT_EOI;

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
} U_CT_FAULT_ADDRESS;

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
} U_CT_FAULT_TYPE_STATUS;

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
} U_CT_FAULT_ATTR_STATUS;

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
} U_CT_FAULT_CLEAR;



/*---------------------------------------------------------------------------------------
 * TIMING_ENGINE_REGS
 *---------------------------------------------------------------------------------------
 */
typedef volatile struct
{
    U_CT_PID                                   r_Pid                         ; /* Offset=0x0000 */
    U_CT_CHIRP_TIMER_TRIG                      r_ChirpTimerTrig              ; /* Offset=0x0004 */
    U_CT_NUM_CHIRPS_AND_BURSTS                 r_NumChirpsAndBursts          ; /* Offset=0x0008 */
    U_CT_CHIRP_TIMER_PROG_RESOLUTION           r_ChirpTimerProgResolution    ; /* Offset=0x000C */
    U_CT_FREQ_START                            r_FreqStart                   ; /* Offset=0x0010 */
    U_CT_FREQ_SLOPE                            r_FreqSlope                   ; /* Offset=0x0014 */
    U_CT_IDLE_TIME                             r_IdleTime                    ; /* Offset=0x0018 */
    U_CT_RAMP_END_TIME                         r_RampEndTime                 ; /* Offset=0x001C */
    U_CT_ADC_START_TIME                        r_AdcStartTime                ; /* Offset=0x0020 */
    U_CT_TX_START_TIME                         r_TxStartTime                 ; /* Offset=0x0024 */
    U_CT_TX_EN_BPM_SETTINGS                    r_TxEnBpmSettings             ; /* Offset=0x0028 */
    U_CT_SYNTH_FREQ_MON_SETTINGS               r_SynthFreqMonSettings        ; /* Offset=0x002C */
    U_CT_TX_MIMO_PATTERN                       r_TxMimoPattern               ; /* Offset=0x0030 */
    U_CT_BURST_PERIODICITY                     r_BurstPeriodicity            ; /* Offset=0x0034 */
    U_CT_BURST_START_OFFSET_TIME               r_BurstStartOffsetTime        ; /* Offset=0x0038 */
    U_CT_CHIRP_ACCUM_SETTINGS                  r_ChirpAccumSettings          ; /* Offset=0x003C */
    U_CT_MISCELLENIOUS_SETTINGS                r_MiscelleniousSettings       ; /* Offset=0x0040 */
    U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_1       r_ControlledRampDownSettings_1; /* Offset=0x0044 */
    U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_2       r_ControlledRampDownSettings_2; /* Offset=0x0048 */
    U_CT_CONTROLLED_RAMP_DOWN_SETTINGS_3       r_ControlledRampDownSettings_3; /* Offset=0x004C */
    U_CT_AFE_POWER_SAVE_EN_REG                 r_AfePowerSaveEnReg           ; /* Offset=0x0050 */
    U_CT_RAPID_EN_OVERRIDE_REG                 r_RapidEnOverrideReg          ; /* Offset=0x0054 */
    U_CT_COUNTER                               r_Counter                     ; /* Offset=0x0058 */
    U_CT_CHIRP_CNT                             r_ChirpCnt                    ; /* Offset=0x005C */
    U_CT_BURST_PERIOD_TIMER                    r_BurstPeriodTimer            ; /* Offset=0x0060 */
    U_CT_BURST_CNT                             r_BurstCnt                    ; /* Offset=0x0064 */
    U_CT_RAPID_EN_PA_TX_RX_LO_TIME_REG         r_RapidEnPaTxRxLoTimeReg      ; /* Offset=0x0068 */
    U_CT_RAPID_EN_LO_TX_START_TIME_REG         r_RapidEnLoTxStartTimeReg     ; /* Offset=0x006C */
    U_CT_RAPID_EN_LO_TX_END_TIME_REG           r_RapidEnLoTxEndTimeReg       ; /* Offset=0x0070 */
    U_CT_RAPID_EN_RF_RX_START_TIME_REG         r_RapidEnRfRxStartTimeReg     ; /* Offset=0x0074 */
    U_CT_RAPID_EN_RF_RX_END_TIME_REG           r_RapidEnRfRxEndTimeReg       ; /* Offset=0x0078 */
    U_CT_RAPID_EN_BB_RX_START_TIME_REG         r_RapidEnBbRxStartTimeReg     ; /* Offset=0x007C */
    U_CT_RAPID_EN_BB_RX_END_TIME_REG           r_RapidEnBbRxEndTimeReg       ; /* Offset=0x0080 */
    U_CT_ADC_TIME_REG                          r_AdcTimeReg                  ; /* Offset=0x0084 */
    U_CT_HPF_FAST_INIT_SYNTH_TIMING_1_REG      r_HpfFastInitSynthTiming_1Reg ; /* Offset=0x0088 */
    U_CT_SYNTH_FAST_RESET_TIMING_2_3_REG       r_SynthFastResetTiming_2_3Reg ; /* Offset=0x008C */
    U_CT_SOCC_TIMING_REG                       r_SoccTimingReg               ; /* Offset=0x0090 */
    U_CT_PERCHIRP_PARAMETER_EN                 r_PerchirpParameterEn         ; /* Offset=0x0094 */
    U_CT_PERCHIRP_PARAMETER_ADDR1              r_PerchirpParameterAddr1      ; /* Offset=0x0098 */
    U_CT_PERCHIRP_PARAMETER_ADDR2              r_PerchirpParameterAddr2      ; /* Offset=0x009C */
    U_CT_PERCHIRP_PARAMETER_ADDR3              r_PerchirpParameterAddr3      ; /* Offset=0x00A0 */
    U_CT_PERCHIRP_PARAMETER_ADDR4              r_PerchirpParameterAddr4      ; /* Offset=0x00A4 */
    U_CT_PERCHIRP_PARAMETER_LEN1               r_PerchirpParameterLen1       ; /* Offset=0x00A8 */
    U_CT_PERCHIRP_PARAMETER_LEN2               r_PerchirpParameterLen2       ; /* Offset=0x00AC */
    U_CT_PERCHIRP_PARAMETER_LEN3               r_PerchirpParameterLen3       ; /* Offset=0x00B0 */
    U_CT_PERCHIRP_PARAMETER_LEN4               r_PerchirpParameterLen4       ; /* Offset=0x00B4 */
    U_CT_PERCHIRP_FREQ_START_REP_CNT           r_PerchirpFreqStartRepCnt     ; /* Offset=0x00B8 */
    U_CT_PERCHIRP_FREQ_SLOPE_REP_CNT           r_PerchirpFreqSlopeRepCnt     ; /* Offset=0x00BC */
    U_CT_PERCHIRP_IDLE_TIME_REP_CNT            r_PerchirpIdleTimeRepCnt      ; /* Offset=0x00C0 */
    U_CT_PERCHIRP_ADC_START_TIME_REP_CNT       r_PerchirpAdcStartTimeRepCnt  ; /* Offset=0x00C4 */
    U_CT_PERCHIRP_TX_START_TIME_REP_CNT        r_PerchirpTxStartTimeRepCnt   ; /* Offset=0x00C8 */
    U_CT_PERCHIRP_TX_EN_REP_CNT                r_PerchirpTxEnRepCnt          ; /* Offset=0x00CC */
    U_CT_PERCHIRP_TX_BPM_REP_CNT               r_PerchirpTxBpmRepCnt         ; /* Offset=0x00D0 */
    U_CT_CHIRP_TIMER_TRIG_STOP_DONE            r_ChirpTimerTrigStopDone      ; /* Offset=0x00D4 */
    U_CT_PARITY_CFG_REG                        r_ParityCfgReg                ; /* Offset=0x00D8 */
    U_CT_PARITY_STATUS                         r_ParityStatus                ; /* Offset=0x00DC */
    U_CT_HW_SPARE_REG0                         r_HwSpareReg0                 ; /* Offset=0x00E0 */
    U_CT_HW_SPARE_REG1                         r_HwSpareReg1                 ; /* Offset=0x00E4 */
    U_CT_HW_SPARE_REG2                         r_HwSpareReg2                 ; /* Offset=0x00E8 */
    U_CT_HW_SPARE_REG3                         r_HwSpareReg3                 ; /* Offset=0x00EC */
    U_CT_HW_SPARE_REG4                         r_HwSpareReg4                 ; /* Offset=0x00F0 */
    U_CT_HW_SPARE_REG5                         r_HwSpareReg5                 ; /* Offset=0x00F4 */
    U_CT_HW_SPARE_REG6                         r_HwSpareReg6                 ; /* Offset=0x00F8 */
    U_CT_HW_SPARE_REG7                         r_HwSpareReg7                 ; /* Offset=0x00FC */
    U_CT_HW_SPARE_REG8                         r_HwSpareReg8                 ; /* Offset=0x0100 */
    U_CT_HW_SPARE_REG9                         r_HwSpareReg9                 ; /* Offset=0x0104 */
    U_CT_TAIL_COUNTING_TIME                    r_TailCountingTime            ; /* Offset=0x0108 */
    U_CT_MISC_RO_REGS                          r_MiscRoRegs                  ; /* Offset=0x010C */
    U_CT_SYNTH_LOCK_STARTSLOPE_DELAY           r_SynthLockStartslopeDelay    ; /* Offset=0x0110 */
    U_CT_PARAM_WAIT_UPDATE_CYCLES              r_ParamWaitUpdateCycles       ; /* Offset=0x0114 */
    U_CT_PA_BLANKING                           r_PaBlanking                  ; /* Offset=0x0118 */
    U_CT_N_START_OVERRIDE_MUX_SEL              r_NStartOverrideMuxSel        ; /* Offset=0x011C */
    U_CT_N_SLOPE_OVERRIDE_MUX_SEL              r_NSlopeOverrideMuxSel        ; /* Offset=0x0120 */
    U_CT_N_STARTSLOPE_PULSE_OVERD_MUX_SEL      r_NStartslopePulseOverdMuxSel ; /* Offset=0x0124 */
    U_CT_N_START_OVERRIDE_VALUE                r_NStartOverrideValue         ; /* Offset=0x0128 */
    U_CT_N_SLOPE_OVERRIDE_VALUE                r_NSlopeOverrideValue         ; /* Offset=0x012C */
    U_CT_N_STARTSLOPE_OVERRIDE_VALUE           r_NStartslopeOverrideValue    ; /* Offset=0x0130 */
    U_CT_MISC_OVERRIDE                         r_MiscOverride                ; /* Offset=0x0134 */
    UINT32                                     r_Padreg0[948]                ; /* Offset=0x0138 */
    U_CT_LOCK0_KICK0                           r_Lock0Kick0                  ; /* Offset=0x1008 */
    U_CT_LOCK0_KICK1                           r_Lock0Kick1                  ; /* Offset=0x100C */
    U_CT_INTR_RAW_STATUS                       r_IntrRawStatus               ; /* Offset=0x1010 */
    U_CT_INTR_ENABLED_STATUS_CLEAR             r_IntrEnabledStatusClear      ; /* Offset=0x1014 */
    U_CT_INTR_ENABLE                           r_IntrEnable                  ; /* Offset=0x1018 */
    U_CT_INTR_ENABLE_CLEAR                     r_IntrEnableClear             ; /* Offset=0x101C */
    U_CT_EOI                                   r_Eoi                         ; /* Offset=0x1020 */
    U_CT_FAULT_ADDRESS                         r_FaultAddress                ; /* Offset=0x1024 */
    U_CT_FAULT_TYPE_STATUS                     r_FaultTypeStatus             ; /* Offset=0x1028 */
    U_CT_FAULT_ATTR_STATUS                     r_FaultAttrStatus             ; /* Offset=0x102C */
    U_CT_FAULT_CLEAR                           r_FaultClear                  ; /* Offset=0x1030 */
} T_CHIRPTIMER_REGS;

#endif
/*
 * END OF REG_CT_H
 */

