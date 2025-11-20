/*!*****************************************************************************
 * @file dfp_sensor.h
 *
 * @brief DFP mmWaveLink Sensor Module API Command and Response data structure header file
 *
 * @b Description @n
 * This is a mmWaveLink Sensor Module API Command and Response data structure
 * header file, same API data structure being used between FECSS and mmWaveLink Libraries
 *
 * @warning Application developer / User shall review Sensor APIs data structure,
 * API functions and handle the Errors.
 *
 * @note
 * <B> © Copyright 2022, Texas Instruments Incorporated - www.ti.com </B>
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS
 * IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
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
 * @addtogroup MMWL_SENSOR_API mmWaveLink Sensor Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     02Jun2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef DFP_SENSOR_H
#define DFP_SENSOR_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_CHIRP_PROF_COMN_CFG Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_DigOutputSampRate Field values
 */
#define M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_12P5M       ((UINT8)8U)
#define M_RL_SENS_DIG_OUT_SAMP_RATE_MIN_1M          ((UINT8)100U)

/*!
 * @brief Member .c_DigOutputBitsSel Field values
 */
#define M_RL_SENS_DIG_OUT_12BITS_4LSB_ROUND         ((UINT8)0U)
#define M_RL_SENS_DIG_OUT_12BITS_3LSB_ROUND         ((UINT8)1U)
#define M_RL_SENS_DIG_OUT_12BITS_2LSB_ROUND         ((UINT8)2U)
#define M_RL_SENS_DIG_OUT_12BITS_1LSB_ROUND         ((UINT8)3U)
#define M_RL_SENS_DIG_OUT_12BITS_0LSB_ROUND         ((UINT8)4U)
#define M_RL_SENS_DIG_OUT_16BITS                    ((UINT8)5U)

/*!
 * @brief Member .c_DfeFirSel Field values
 */
#define M_RL_SENS_DFE_FIR_LONG_FILT                 ((UINT8)0U)
#define M_RL_SENS_DFE_FIR_SHORT_FILT                ((UINT8)1U)

/*!
 * @brief Member .c_VcoMultiChipMode Field values
 */
#define M_RL_SENS_VCO_MULT_CHIP_SINGLE              ((UINT8)0U)
#define M_RL_SENS_VCO_MULT_CHIP_PRIMARY             ((UINT8)1U)
#define M_RL_SENS_VCO_MULT_CHIP_SECONDARY           ((UINT8)2U)

/*!
 * @brief Member .h_NumOfAdcSamples Field values
 */
#define M_RL_SENS_NUM_ADC_SAMPL_MIN                 ((UINT16)2U)
#define M_RL_SENS_NUM_ADC_SAMPL_MAX                 ((UINT16)2048U)

/*!
 * @brief Member .c_ChirpTxMimoPatSel Field values
 */
#define M_RL_SENS_TX_MIMO_PATRN_DIS                 ((UINT8)0U)
#define M_RL_SENS_TX_MIMO_PATRN_2TX_TDMA            ((UINT8)1U)
#define M_RL_SENS_TX_MIMO_PATRN_2TX_BPM             ((UINT8)4U)

/*!
 * @brief Member .h_ChirpRampEndTime Field values
 */
#define M_RL_SENS_RAMP_END_TIME_MIN                 ((UINT16)1U)
#define M_RL_SENS_RAMP_END_TIME_MAX                 ((UINT16)65535U)

/*!
 * @brief Member .c_ChirpRxHpfSel Field values
 */
#define M_RL_SENS_RX_HPF_SEL_300KHZ                 ((UINT8)0U)
#define M_RL_SENS_RX_HPF_SEL_350KHZ                 ((UINT8)1U)
#define M_RL_SENS_RX_HPF_SEL_700KHZ                 ((UINT8)2U)
#define M_RL_SENS_RX_HPF_SEL_1400KHZ                ((UINT8)3U)
#define M_RL_SENS_RX_HPF_SEL_2800KHZ                ((UINT8)4U)

/*!
 * @brief Member .c_HpfFastInitDuration Field values
 */
#define M_RL_SENS_HPF_FAST_INIT_DUR_MIN             ((UINT8)5)
#define M_RL_SENS_HPF_FAST_INIT_DUR_MAX             ((UINT8)200)

/*!
 * @brief Member .c_MiscSettings Field values
 */
#define M_RL_SENS_MISC_HPF_FAST_INIT_DIS_BIT        ((UINT8)0U)
#define M_RL_SENS_MISC_CRD_DIS_BIT                  ((UINT8)1U)
#define M_RL_SENS_MISC_CRD_DITHER_DIS_BIT           ((UINT8)2U)
#define M_RL_SENS_MISC_PA_BLANK_EN_BIT              ((UINT8)3U)

/*!
 * @brief Member .h_CrdNSlopeMag Field values
 */
#define M_RL_SENS_CRD_NSLOPE_MAG_MIN                ((UINT16)0)
#define M_RL_SENS_CRD_NSLOPE_MAG_MAX                ((UINT16)32767)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_CHIRP_PROF_TIME_CFG Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .h_ChirpIdleTime Field values
 */
#define M_RL_SENS_CHIRP_IDLE_TIME_MIN               ((UINT16)1U)
#define M_RL_SENS_CHIRP_IDLE_TIME_MAX               ((UINT16)65535U)

/*!
 * @brief Member .h_ChirpAdcStartTime Field values
 */
#define M_RL_SENS_CHIRP_ADCSTART_TIME_FRAC_MAX      ((UINT16)1023U)
#define M_RL_SENS_CHIRP_ADCSTART_TIME_FRAC_MASK     ((UINT16)0x03FF)

#define M_RL_SENS_CHIRP_ADC_SKIP_SAMP_MIN           ((UINT16)0U)
#define M_RL_SENS_CHIRP_ADC_SKIP_SAMP_MAX           ((UINT16)63U)
#define M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET        ((UINT16)10U)

/*!
 * @brief Member .xh_ChirpTxStartTime Field values
 */
#define M_RL_SENS_CHIRP_TXSTART_TIME_MIN            ((SINT16)-32767)
#define M_RL_SENS_CHIRP_TXSTART_TIME_MAX            ((SINT16)32767)

/*!
 * @brief Member .xh_ChirpRfFreqSlope Field values
 */
#define M_RL_SENS_CHIRP_RFSLOPE_TIME_MIN            ((SINT16)-32767)
#define M_RL_SENS_CHIRP_RFSLOPE_TIME_MAX            ((SINT16)32767)

/*!
 * @brief Member .w_ChirpRfFreqStart Field values High Resol 60G devices
 */
#define M_RL_SENS_CHIRP_RFFREQ_HR_57G               ((UINT32)0x00BE0000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_58G               ((UINT32)0x00C15555U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_59G               ((UINT32)0x00C4AAAAU)
#define M_RL_SENS_CHIRP_RFFREQ_HR_60G               ((UINT32)0x00C80000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_61G               ((UINT32)0x00CB5555U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_62G               ((UINT32)0x00CEAAAAU)
#define M_RL_SENS_CHIRP_RFFREQ_HR_63G               ((UINT32)0x00D20000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_64G               ((UINT32)0x00D55555U)
/*!
 * @brief Member .w_ChirpRfFreqStart Field values High Resol 77G devices
 */
#define M_RL_SENS_CHIRP_RFFREQ_HR_76G               ((UINT32)0x00BE0000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_77G               ((UINT32)0x00C08000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_78G               ((UINT32)0x00C30000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_79G               ((UINT32)0x00C58000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_80G               ((UINT32)0x00C80000U)
#define M_RL_SENS_CHIRP_RFFREQ_HR_81G               ((UINT32)0x00CA8000U)
/*!
 * @brief Member .w_ChirpRfFreqStart Field values Low Resol 60G devices
 */
#define M_RL_SENS_CHIRP_RFFREQ_LR_57G               (M_RL_SENS_CHIRP_RFFREQ_HR_57G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_58G               (M_RL_SENS_CHIRP_RFFREQ_HR_58G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_59G               (M_RL_SENS_CHIRP_RFFREQ_HR_59G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_60G               (M_RL_SENS_CHIRP_RFFREQ_HR_60G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_61G               (M_RL_SENS_CHIRP_RFFREQ_HR_61G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_62G               (M_RL_SENS_CHIRP_RFFREQ_HR_62G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_63G               (M_RL_SENS_CHIRP_RFFREQ_HR_63G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_64G               (M_RL_SENS_CHIRP_RFFREQ_HR_64G >> 8U)
/*!
 * @brief Member .w_ChirpRfFreqStart Field values Low Resol 77G devices
 */
#define M_RL_SENS_CHIRP_RFFREQ_LR_76G               (M_RL_SENS_CHIRP_RFFREQ_HR_76G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_77G               (M_RL_SENS_CHIRP_RFFREQ_HR_77G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_78G               (M_RL_SENS_CHIRP_RFFREQ_HR_78G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_79G               (M_RL_SENS_CHIRP_RFFREQ_HR_79G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_80G               (M_RL_SENS_CHIRP_RFFREQ_HR_80G >> 8U)
#define M_RL_SENS_CHIRP_RFFREQ_LR_81G               (M_RL_SENS_CHIRP_RFFREQ_HR_81G >> 8U)

/*!
 * @brief Member .h_ChirpTxEnSel Field values
 */
#define M_RL_SENS_CHIRP_TX_EN_MIN                   ((UINT16)0x0000U)
#define M_RL_SENS_CHIRP_TX_EN_MAX                   ((UINT16)0x0003U)

/*!
 * @brief Member .h_ChirpTxBpmEnSel Field values
 */
#define M_RL_SENS_CHIRP_TXBPM_EN_MIN                ((UINT16)0x0000U)
#define M_RL_SENS_CHIRP_TXBPM_EN_MAX                ((UINT16)0x0003U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_PER_CHIRP_CFG Data Structure field value macros
 * @{
 */
/*!
 * @brief Max per chirp parameters
 */
#define M_RL_SENS_PER_CHIRP_FREQ_START          ((UINT8)0U)
#define M_RL_SENS_PER_CHIRP_FREQ_SLOPE          ((UINT8)1U)
#define M_RL_SENS_PER_CHIRP_IDLE_TIME           ((UINT8)2U)
#define M_RL_SENS_PER_CHIRP_ADC_START_TIME      ((UINT8)3U)
#define M_RL_SENS_PER_CHIRP_TX_START_TIME       ((UINT8)4U)
#define M_RL_SENS_PER_CHIRP_TX_ENABLE           ((UINT8)5U)
#define M_RL_SENS_PER_CHIRP_BPM_ENABLE          ((UINT8)6U)
#define M_RL_SENS_PER_CHIRP_MAX_PARAMS          ((UINT8)7U)

/*!
 * @brief Member .h_ParamArrayLen Field values
 */
#define M_RL_SENS_PER_CHIRP_ARRAY_LEN_MIN       ((UINT16)1U)
#define M_RL_SENS_PER_CHIRP_ARRAY_LEN_MAX       ((UINT16)4096U)

/*!
 * @brief Member .h_ParamRptCount Field values
 */
#define M_RL_SENS_PER_CHIRP_RPT_COUNT_MIN       ((UINT16)1U)
#define M_RL_SENS_PER_CHIRP_RPT_COUNT_MAX       ((UINT16)32768U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_SENS_PER_CHIRP_CTRL Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .h_PerChirpParamCtrl Field values
 */
#define M_RL_SENS_PER_CHIRP_PARAM_DIS           ((UINT16)0U)
#define M_RL_SENS_PER_CHIRP_PARAM_ENA           ((UINT16)1U)
#define M_RL_SENS_PER_CHIRP_CTRL_MIN            ((UINT16)0x0U)
#define M_RL_SENS_PER_CHIRP_CTRL_MAX            ((UINT16)0x7FU)

/**
 * @name Per-Chirp Parameter Indices
 * @{
*/
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_FREQ_START          ((UINT8)0U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_FREQ_SLOPE          ((UINT8)1U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_IDLE_TIME           ((UINT8)2U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_ADC_START_TIME      ((UINT8)3U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_TX_START_TIME       ((UINT8)4U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_TX_EN               ((UINT8)5U)
#define M_RL_SENS_PER_CHIRP_PARAM_INDEX_BPM_EN              ((UINT8)6U)

/** @} */

/*!
 * @brief Member .h_ParamArrayStartAdd Field values
 */
#define M_RL_SENS_PER_CHIRP_LUT_STRT_ADDR_MIN   ((UINT16)0x0U)
#define M_RL_SENS_PER_CHIRP_LUT_STRT_ADDR_MAX   ((UINT16)0x1FFCU)
#define M_RL_SENS_PER_CHIRP_LUT_4BYTE_ADD_MASK  ((UINT16)0x3U)
#define M_RL_SENS_PER_CHIRP_LUT_ADD_MASK        ((UINT16)0x1FFFU)

/*!
 * @brief FECSS Per chirp LUT 8kB RAM start address - FEC cluster1 memory (retained)
 */
#define M_RL_SENS_PER_CHIRP_LUT_HW_START_ADDRESS    (0x21880000U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_FRAME_CFG Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .h_NumOfChirpsInBurst Field values
 */
#define M_RL_SENS_NUM_CHIRPS_MIN                ((UINT16)1U)
#define M_RL_SENS_NUM_CHIRPS_MAX                ((UINT16)65535U)

/*!
 * @brief Member .c_NumOfChirpsAccum Field values
 */
#define M_RL_SENS_NUM_CHIRPS_ACCUM_DIS          ((UINT8)0U)
#define M_RL_SENS_NUM_CHIRPS_ACCUM_MIN          ((UINT8)2U)
#define M_RL_SENS_NUM_CHIRPS_ACCUM_MAX          ((UINT8)64U)

/*!
 * @brief Member .w_BurstPeriodicity Field values
 */
#define M_RL_SENS_BURST_PERIOD_MIN              ((UINT32)10U)
#define M_RL_SENS_BURST_PERIOD_MAX              ((UINT32)16777215U)

/*!
 * @brief Member .h_NumOfBurstsInFrame Field values
 */
#define M_RL_SENS_NUM_BURSTS_MIN                ((UINT16)1U)
#define M_RL_SENS_NUM_BURSTS_MAX                ((UINT16)4096U)

/*!
 * @brief Member .w_FramePeriodicity Field values
 */
#define M_RL_SENS_FRAME_PERIOD_MIN              ((UINT32)100U)
#define M_RL_SENS_FRAME_PERIOD_MAX              ((UINT32)4294967295U)

/*!
 * @brief Member .h_NumOfFrames Field values
 */
#define M_RL_SENS_NUM_FRAMES_MIN                ((UINT16)0U)
#define M_RL_SENS_NUM_FRAMES_MAX                ((UINT16)65535U)

/*!
 * @brief Member .w_FrameEvent0TimeCfg and w_FrameEvent1TimeCfg Field values
 */
#define M_RL_SENS_FRAME_EVENT_TIME_MIN          ((UINT32)0U)
#define M_RL_SENS_FRAME_EVENT_TIME_MAX          ((UINT32)4294967295U)

/**
 * @brief Member of .c_MiscSetupMask field values
*/
#define M_RL_SENS_FRAME_SKIP_FT_CONFIG          ((UINT8)1U << 0U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENSOR_START_CMD Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .c_FrameTrigMode Field values
 */
#define M_RL_SENS_FRAME_SW_TRIG                 ((UINT8)0U)
#define M_RL_SENS_FRAME_SW_TIMER_TRIG           ((UINT8)1U)
#define M_RL_SENS_FRAME_HW_LOW_PWR_TRIG         ((UINT8)2U)
#define M_RL_SENS_FRAME_HW_LOW_JIT_TRIG         ((UINT8)3U)
#define M_RL_SENS_FRAME_CW_CZ_TRIG              ((UINT8)4U)
#define M_RL_SENS_FRAME_CT_OVRD_TRIG            ((UINT8)5U)

/*!
 * @brief Member .c_ChirpStartSigLbEn Field values
 */
#define M_RL_SENS_CT_START_SIG_LB_DIS           ((UINT8)0U)
#define M_RL_SENS_CT_START_SIG_LB_ENA           ((UINT8)1U)

/*!
 * @brief Member .c_FrameLivMonEn Live monitor enable start bits
 */
#define M_RL_SENS_LIVE_MON_SYNTH_FREQ           ((UINT8)0U)
#define M_RL_SENS_LIVE_MON_RX_SATURATION        ((UINT8)1U)
#define M_RL_SENS_LIVE_MON_GPADC_CTM            ((UINT8)2U)
#define M_RL_SENS_LIVE_MON_MAX_ENA_MASK         ((UINT8)0x7U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_LOOPBACK_CFG Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .c_LbFreqSel Field values
 */
#define M_RL_SENS_LB_FREQ_MIN               ((UINT8)1U)
#define M_RL_SENS_LB_FREQ_MAX               ((UINT8)200U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_SENSOR_STOP_CMD Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .c_FrameStopMode Field values
 */
#define M_RL_SENS_STOP_FRAME                ((UINT8)0U)
#define M_RL_SENS_STOP_CWCZ_MODE            ((UINT8)1U)
#define M_RL_SENS_FORCE_STOP_FRAME          ((UINT8)2U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_SENS_DYN_PWR_SAVE_DIS Data Structure field value macros
 * @{
 */
/*!
 * @brief Member .c_InterChirpPsDis start bit
 */
#define M_RL_SENS_IC_TX_PA_PS_DIS               ((UINT8)0U)
#define M_RL_SENS_IC_TX_LO_PS_DIS               ((UINT8)1U)
#define M_RL_SENS_IC_RX_RF_PS_DIS               ((UINT8)2U)
#define M_RL_SENS_IC_RX_BB_PS_DIS               ((UINT8)3U)
#define M_RL_SENS_IC_RX_LO_PS_DIS               ((UINT8)4U)
#define M_RL_MAX_SENS_IC_PS_DIS_CONTROLS        ((UINT8)5U)

/*!
 * @brief Member .c_InterBurstPsDis start bit
 */
#define M_RL_SENS_IB_RX_TX_LO_PS_DIS            ((UINT8)0U)
#define M_RL_SENS_IB_RX_SYNTH_LDO_PS_DIS        ((UINT8)1U)
#define M_RL_SENS_IB_RX_RF1P8_LDO_PS_DIS        ((UINT8)2U)
#define M_RL_SENS_IB_RX_RF1P2_LDO_PS_DIS        ((UINT8)3U)
#define M_RL_SENS_IB_RX_ADC_CLK_PS_DIS          ((UINT8)4U)
#define M_RL_SENS_IB_SYNTH_LDO_SETL_DIS         ((UINT8)5U)
#define M_RL_MAX_SENS_IB_PS_DIS_CONTROLS        ((UINT8)6U)

/*! @} */


/**
 * @name Members for .c_LbEnable field
 * @{
*/
#define M_RL_SENS_LOOPBACK_DISABLE              ((UINT8)0U)
#define M_RL_SENS_LOOPBACK_DIG                  ((UINT8)1U)
#define M_RL_SENS_LOOPBACK_IFA                  ((UINT8)2U)
#define M_RL_SENS_LOOPBACK_LO                   ((UINT8)3U)
#define M_RL_SENS_LOOPBACK_PA                   ((UINT8)4U)
#define M_RL_SENS_LOOPBACK_EXTERNAL_WG          ((UINT8)5U)
/** @} */



/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Sensor Module API Type defines
 * @{
 */

/*!
 * @brief mmWaveLink @ref rl_sensChirpProfComnCfg and @ref rl_sensChirpProfComnCfgGet API
 * command/response Data structure \n
 * FECSS sensor chirp profile common config command data structure contains the common radar
 * chirp profile parameters like common digital parameters, timing info and RF TX/RX gain and
 * filter settings. These chirp profile common parameters are common settings for all chirps,
 * all the chirps in the frame will use these profile parameters.
 */
typedef struct
{
    /*!
     * @brief  Digital output Sampling rate for chirp ADC samples is encoded in 1 byte
     * (8 bit unsigned number) \n
     * Chirp ADC Sampling Rate in MSPS = APLL_FREQ / (c_DigOutputSampRate * 4) \n
     * Fw Valid Range: 8 to 100 \n
     * Recommended sampling rate can be configured as per below table. \n
     * | Value   | ADC sample rate |
     * |---------|-----------  |
     * | 8       | 12.5 MSPS   |
     * | 10      | 10 MSPS     |
     * | 13      | 7.692 MSPS  |
     * | 15      | 6.667 MSPS  |
     * | 20      | 5 MSPS      |
     * | 25      | 4 MSPS      |
     * | 40      | 2.5 MSPS    |
     * | 50      | 2 MSPS      |
     * | 60      | 1.667 MSPS  |
     * | 80      | 1.25 MSPS   |
     * | 100     | 1 MSPS      |
     *
     */
    UINT8 c_DigOutputSampRate;

    /*!
     * @brief  Digital output sample bits select, this field governs which bits of the FECSS DFE's
     * internal 16-bit signed data path is sent as output. The digital filter chain output is
     * always sent as a 16-bit bus per RX. In the cases where only 12 bits are sent, they are sent
     * on the LSBs, and the remaining 4 MSBs in the interface are sign extended.  \n
     * The output bit selection options as below:
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | Digital sample output is 12 MSB bits of DFE after rounding 4 LSBs    |
     * | 1       | Digital sample output is 12 bits after rounding 3 LSBs & clipping 1 MSB |
     * | 2       | Digital sample output is 12 bits after rounding 2 LSBs & clipping 2 MSB |
     * | 3       | Digital sample output is 12 bits after rounding 1 LSBs & clipping 3 MSB |
     * | 4       | Digital sample output is 12 LSB bits after clipping 4 MSB |
     * | 5       | Digital sample output is 16 bits |
     * Valid Range: 0 to 5 \n
     */
    UINT8 c_DigOutputBitsSel;

    /*!
     * @brief  The final stage FIR filter's characteristics can be selected as below.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | Long Filter (90% visibility): This provides visibility to a larger range of IF frequencies: 0 to 0.45 x Sampling Rate. Beyond that, the filter response starts drooping & enters filter transition band.  |
     * | 1       | Short Filter (80% visibility): This provides faster settled outputs but the IF frequency range visible is 0 to 0.40 x Sampling Rate. Beyond that, the filter response starts drooping & enters filter transition band. |
     */
    UINT8 c_DfeFirSel;

    /*!
     * @brief  The sensor VCO multi chip cascade mode.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | SINGLE_CHIP VCO mode. \n Default Mode. |
     * | 1       | MULTI_CHIP_PRIMARY VCO mode. \n @b NOTE: This mode is not supported in this DFP / Devices. |
     * | 2       | MULTI_CHIP_SECONDARY VCO mode. \n @b NOTE: This mode is not supported in this DFP / Devices. |
     * Default: 0x00
     */
    UINT8 c_VcoMultiChipMode;

    /*!
     * @brief  Digital output Number of ADC samples to capture in a chirp for each RX channels.
     * 16bits unsigned number. \n
     * Valid Range: 2 to 2048
     */
    UINT16 h_NumOfAdcSamples;

    /*!
     * @brief  Chirp Profile TX BPM MIMO Pattern selection. This TX and BPM patterns are
     * generated across chirps in a frame as per below table.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | Disabled, Default setting. The CHIRP_TX_EN_SEL and CHIRP_TX_BPM_EN_SEL registers take effect in determining the TX enable and BPM controls to the TX AFE for all chirps in the frame. |
     * | 1       | TDMA_2TX pattern. The CHIRP_TX_EN_SEL register is ignored and is internally varied as [01; 10; 01; 10; …] in chirp 0, 1, 2, 3, … and so on in the frame. Here the 2 bits refer to [TX1; TX0]. Other TXs if present are always disabled. The CHIRP_TX_BPM_EN_SEL register continues to take effect directly. |
     * | 4       | BPM_2TX pattern. The CHIRP_TX_EN_SEL register takes effect directly for all chirps in the frame. But CHIRP_TX_BPM_EN_SEL register is ignored and is internally varied as [00; 10; 00; 10; …] in chirp 0, 1, 2, 3, … and so on in the frame. Here the 2 bits refer to [TX1; TX0] BPM. Other TXs if present are always having 0 BPM. |
     * Valid Values: 0, 1 and 4 \n
     * @b Note: If CHIRP_TX_MIMO_PAT_SEL is enabled (non zero) then the per chirp PER_CHIRP_TX_EN and
     * PER_CHIRP_BPM_EN settings are not supported. It is recommend to disable the per chirp
     * config for these parameters.
     */
    UINT8 c_ChirpTxMimoPatSel;

    /*!
     * @brief  The chirp miscellaneous controls. This HPF fast init, Synth Controlled ramp down
     * and PA blanking features can be disabled using this control param, these settings are
     * enabled by default in device except PA blanking. \n
     * | Value   | Definition |
     * |---------|----------- |
     * |   0     |  Enabled   |
     * |   1     |  Disabled  |
     * Bit field Definition: \n
     * | Bit Field   | Definition |
     * |-----------  |----------- |
     * | Bit[0]      | HPF_FAST_INIT disable. The HPF Fast init feature can be disabled for each chirp using this option. \n If enabled, the Chirp Timer generates RX_HPF_FAST_INIT signal for fast initialization of RX HPF through a temporary increase in HPF bandwidth (20x BW) at the start of the ramp, starts 100ns before TX_START_TIME of chirp and ends after HPF_FAST_INIT_DURATION. \n  The HPF Fast init duration in a chirp can be programmed using HPF_FAST_INIT_DURATION field in this API. \n Default: Enabled |
     * | Bit[1]      | SYNTH_CRD disable. The SYNTH Control Ramp Down (CRD) feature can be disabled for each chirp using this option. \n The Host can configure the Chirp profile with Controlled Ramp Down feature in order to avoid undershoot / overshoot of synthesizer and mitigate the fixed frequency RF spurs. \n The CRD_NSLOPE_MAG in a chirp can be programmed using CRD_NSLOPE_MAG field in this API. \n Default: Enabled |
     * | Bit[2]      | SYNTH_CRD_DITHER disable. The SYNTH Control Ramp Down (CRD) Dither feature can be disabled for each chirp using this option. This feature is applicable only if CRD is enabled. \n If enabled then for 260ns duration after the end of chirp ramp, the frequency is kept at a constant dither value Δf lower than the previous chirp end frequency. The dither magnitude Δf is varied randomly every chirp, uniformly between 0 and 0x06D4 Freq Code (2048.4375MHz in 60GHz and 2731.25MHz in 77GHz devices). \n Default: Enabled |
     * | Bit[3]      | PA_BLANKING enable. Device supports PA blanking through an input signal to the device in order to stop the transmission from the device instantaneously. \n This feature can be particularly useful, when the device is used in presence of other RF transceiver devices for various purposes (e.g. 5G communication). \n Write value 1 to enable the feature \n Default: Disabled (0) |
     * | Bits[7:4]  | Reserved |
     * Recommended Default: 0x00
     */
    UINT8 c_MiscSettings;

    /*!
     * @brief  HPF Fast Init Duration. This duration shall be programmed in accordance with
     * ADC start time of chirps. \n
     * 1LSB = 40/ APLL_FREQ (Typical 100ns with 40MHz XTAL) \n
     * Range: 5 to 100 \n
     * Recommended Default Value: 15 (1.5us) \n
     */
    UINT8 c_HpfFastInitDuration;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved;

    /*!
     * @brief CRD NSLOPE Magnitude
     *
     * The chirp Control Ramp Down slope magnitude. The sign of the
     * slope is auto detected based on sign of the chirp programmed in sensor.
     *
     * | RF Frequency   | Resolution |
     * |----------------- |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * (APLL_FREQ) / 2^20 = ~457.763 kHz/us , unsigned number \n Valid range (Typical with 40MHz XTAL): 0x0 to 0xC00 (1406 MHz/us) |
     * | 77GHz | 1 LSB = 4 * 400 * (APLL_FREQ) / 2^20 = ~610.351 kHz/us , unsigned number \n Valid range (Typical with 40MHz XTAL): 0x0 to 0x925 (1428 MHz/us) |
     *
     * - Recommended Formula to compute slope: Slope (kHz/us) = RF Bandwidth (kHz)/ min(CHIRP_IDLE_TIME - 1us, 6us) \n RF bandwidth in kHz = ramp time * slope \n
     * - @b Note: CRD Ramp down time must be less than min(CHIRP_IDLE_TIME - 1U, 6us)
     */
    UINT16 h_CrdNSlopeMag;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved;

    /*!
     * @brief  Chirp Profile Ramp end Time. This is a common ramp end time value for all chirps in
     * a frame. This field indicates the FMCW frequency ramping time in the chirp cycle. \n
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 8/APLL_FREQ (20ns with 40MHz XTAL), unsigned number \n Valid Range: 1 to 65535 (~1.3ms) |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 40/APLL_FREQ (100ns with 40MHz XTAL), unsigned number \n Valid Range: 1 to 65535 (~6.5ms) |
     * Ensure that the total frequency sweep is either within these ranges: \n
     * 77G Devices: 76 - 81 GHz \n
     * 60G Devices: 57 - 64GHz \n
     * Note: Not Applicable to CW_CZ_TRIG Mode.
     */
    UINT16 h_ChirpRampEndTime;

    /*!
     * @brief  Chirp Profile HPF corner frequency. This is a common HPF corner frequency value
     * for all chirps in a frame.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | 175kHz HPF corner frequency |
     * | 1       | 350kHz HPF corner frequency |
     * | 2       | 700kHz HPF corner frequency |
     * | 3       | 1400kHz HPF corner frequency |
     * Valid Range: 0 to 3
     */
    UINT8 c_ChirpRxHpfSel;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3[4];

} T_RL_API_SENS_CHIRP_PROF_COMN_CFG;

/*!
 * @brief mmWaveLink @ref rl_sensChirpProfTimeCfg and @ref rl_sensChirpProfTimeCfgGet API
 * command/response Data structure \n
 * FECSS sensor chirp profile common config command data structure contains the common radar
 * chirp profile parameters like FMCW RF parameters and timing info. These chirp profile common
 * parameters are common settings for all chirps, all the chirps in the frame will use these
 * profile parameters. The per chirp config (PER_CHIRP_CFG) API can be used to vary/dither
 * certain these parameters in every chirp.
 */
typedef struct
{
    /*!
     * @brief  Chirp Profile Idle Time. This is a common idle time value for all chirps in a
     * frame. If PER_CHIRP_CFG idle time is enabled then this value is discarded.
     * This field indicates the idle time in the chirp cycle, before starting of frequency ramp. \n
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 8/APLL_FREQ (20ns with 40MHz XTAL), unsigned number \n Valid Range: 1 to 65535 (~1.3ms) |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 40/APLL_FREQ (100ns with 40MHz XTAL), unsigned number \n Valid Range: 1 to 65535 (~6.5ms) |
     */
    UINT16 h_ChirpIdleTime;

    /*!
     * @brief  Chirp Profile ADC start Time and skip samples. This is a common adc start time
     * value for all chirps in a frame. If PER_CHIRP_CFG adc start time is enabled then this value
     * is discarded. \n
     * | Bit Filed   | Definition |
     * |-----------  |----------- |
     * | Bits[9:0] | Reserved |
     * | Bits[15:10] | Number of ADC Samples to be Skipped. \n This field indicates the number of samples to be skipped from knee of the ramp, which controls ADC sampling start time. \n 1 LSB=1 ADC sample \n Valid range: 0 to 63 |
     * Recommended value: max(24, 3us*ADCSamplingRate)
     */
    UINT16 h_ChirpAdcStartTime;

    /*!
     * @brief  Chirp Profile TX start Time. This is a common TX start time value for all chirps in
     * a frame. If PER_CHIRP_CFG idle time is enabled then this value is discarded. \n
     * This field indicates the TX start time in the chirp cycle with respect to the knee of the
     * ramp. \n
     * 1 LSB = 8/APLL_FREQ (Typical 20ns with 40MHz XTAL), signed number \n
     * Recommended Range: -150 to 150 (+/- 3us)
     * @note The CHIRP_PROG_RESOL_SEL is not applicable for this parameter.
     */
    SINT16 xh_ChirpTxStartTime;

    /*!
     * @brief  Chirp Profile RF Frequency Slope. This is a common RF frequency slope value for all
     * chirps in a frame. If PER_CHIRP_CFG frequency slope is enabled then this value is
     * discarded. \n
     * This field indicates the required FMCW slope of the chirp. \n
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -13972 to 13972 (+/- ~399MHz/us) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -13972 to 13972 (+/- ~533MHz/us) |
     * Note: Not Applicable to CW_CZ_TRIG Mode, the slope is always set to value 0.
     */
    SINT16 xh_ChirpRfFreqSlope;

    /*!
     * @brief  Chirp Profile RF start Frequency. This is a common RF start frequency value for all
     * chirps in a frame. If PER_CHIRP_CFG start frequency is enabled then this value is
     * discarded. \n
     * This field indicates the required start frequency of the chirp. \n
     * CHIRP_TIMER_RESOL_SEL b0 = 1 (High Resolution Mode) (24 bits): \n
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^24) * 2^6 = ~4.578 kHz \n Valid Range: 0x00BE0000 (57GHz) to 0x00D55555 (64GHz) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^24) * 2^6 = ~6.103 kHz \n Valid Range: 0x00BE0000 (76GHz) to 0x00CA8000(81GHz) |
     * CHIRP_TIMER_RESOL_SEL b0 = 0 (Low Resolution Mode) (16 bits): \n
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz) to 0x0000D555 (64GHz) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz) to 0x0000CA80(81GHz) |
     */
    UINT32 w_ChirpRfFreqStart;

    /*!
     * @brief  Chirp Profile TX enable. This is a common TX enable value for all chirps in a
     * frame. If PER_CHIRP_CFG TX enable is enabled then this value is discarded. \n
     * This field indicates the TX's enabled for the chirp. \n
     * | Value   | Definition |
     * |-------  |----------- |
     * | 0       | TX is Disabled |
     * | 1       | TX is Enabled  |
     * Bit definition: \n
     * | Bit Field   | Definition |
     * |-----------  |----------- |
     * | Bit 0       | TX0 Enable control |
     * | Bit 1       | TX1 Enable control |
     * | Bits [15:2] | Reserved |
     */
    UINT16 h_ChirpTxEnSel;

    /*!
     * @brief  Chirp Profile TX BPM (Binary Phase Modulation) enable. This is a common TX BPM
     * enable value for all chirps in a frame. If PER_CHIRP_CFG TX enable is enabled then this
     * value is discarded. \n
     * This field indicates the TX BPM's enabled for the chirp. \n
     * | Value   | Definition |
     * |-------  |----------- |
     * | 0       | TX BPM is Disabled |
     * | 1       | TX BPM is Enabled  |
     * Bit definition: \n
     * | Bit Field   | Definition |
     * |-----------  |----------- |
     * | Bit 0       | TX0 BPM Enable control |
     * | Bit 1       | TX1 BPM Enable control |
     * | Bits [15:2] | Reserved |
     */
    UINT16 h_ChirpTxBpmEnSel;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2[4U];

} T_RL_API_SENS_CHIRP_PROF_TIME_CFG;

/*!
 * @brief mmWaveLink @ref rl_sensPerChirpCfg and @ref rl_sensPerChirpCfgGet API
 * command/response Data structure \n
 * FECSS sensor per chirp config command data structure contains the per chirp parameter
 * waveform pattern generation configuration. \n
 * This API shall be used to configure the chirp generation pattern to generate a advance waveform
 * by using unique chirp parameters for every chirp in a burst or frame. \n
 * This API provides ability to program unique chirp parameters for every chirp selected from
 * configurable look-up-table (PER_CHIRP_LUT). The configurable look-up-table is an ARRAY of
 * values loaded into a pre-configured “Generic HW Chirp Parameter LUT”. The size of the generic
 * LUT is 8kB and user has the flexibility to program any number of unique values for each chirp
 * parameters. Thus the user can generate any radar waveform using this API.
 * | Generic PER_CHIRP_LUT Address  | Generic PER_CHIRP_LUT Size | Address Offset Alignment |
 * |------------------------------  |--------------------------- | ------------------------ |
 * | 0x21880000                     | 8192 Bytes                 |  4 Bytes                 |
 * The Application can directly store the per chirp parameter values ARRAY in this PER_CHIRP_LUT
 * RAM and call rl_sensPerChirpCfg and rl_sensPerChirpCtrl APIs to configure the FECSS
 * HW. \n
 * The parameter ARRAY start offset address PARAM_ARR_START_ADD shall be multiple of 4 bytes. \n
 * Each of below PER_CHIRP_PARAM inherits its definitions such as bit-width and number format from
 * the rl_sensChirpProfileTimeCfg API. Further, each row in the Per-Chirp Configuration RAM
 * holds one entry, and each row occupies a few bytes (in some cases lesser than a byte).
 * | PER_CHIRP_PARAM | PER_CHIRP_LUT row definition |
 * |---------------  |---------------------------   |
 * | FREQ_START      | 1 row = 4 bytes (In both High and Low Resolution) |
 * | FREQ_SLOPE      | 1 row = 2 bytes |
 * | IDLE_TIME       | 1 row = 2 bytes |
 * | ADC_START_TIME  | 1 row = 2 bytes |
 * | TX_START_TIME   | 1 row = 2 bytes |
 * | TX_EN           | 1 row = 4 bits |
 * | TX_BPM          | 1 row = 4 bits |
 *
 * Using this API, two types of control can be achieved on each parameters of a chirp. \n
 *  -# Identical chirps: Repeating chirp waveforms with identical parameters every chirp
 * (similar to PROFILE_TIME_CFG), to achieve this only one value can be programmed in LUT
 * (PER_CHIRP_LUT) for each parameter, and program PARAM_ARRAY_LEN (N) = 1 and
 * PARAM_RPT_COUNT (M) = 1 \n
 *   -# Unique chirps: Variation of chirp parameters such as FREQ_START, FREQ_SLOPE, IDLE_TIME,
 * ADC_START_TIME, TX_START_TIME, TX_EN, and TX_BPM on a per-chirp basis, to achieve this index
 * the parameter ARRAY every PARAM_RPT_COUNT (M) chirps for PARAM_ARRAY_LEN (N) times to generate
 * unique sequence of M * N chirps. \n
 * \note Application Shall not modify PER_CHIRP_LUT contents when active frame chirps are using
 * the RAM in runtime. In case customer need to update the chirp parameters across frame then
 * good to update the RAM in ping pong fashion. The PER_CHIRP_LUT RAM contents are retained in
 * Deep sleep entry/exit, application is not required to reconfigure this RAM every deep sleep
 * cycle.
 * \note This API is Not Applicable for CW_CZ_TRIG Mode.
 */
typedef struct
{
    /*!
     * @brief  Length (N) of the parameter Array in CHIRP_LUT. The number of parameter elements
     * or rows in ARRAY. \n
     * The chirp sequence will reset every N chirps. \n
     * Valid range: 1 - 4096  \n
     * The PARAM_ARRAY_LEN (N) should be integer multiple of PARAM_RPT_COUNT(M). \n
     * | Value    | Definition |
     * |-------   |----------- |
     * |   1      | Identical chirps. Fixed 0th indexed Array. |
     * | 2 - 4096 | Chirp sequence reset every N chirps.|
     * Byte Definition: \n
     * | Bytes        | Definition |
     * |-------       |----------- |
     * | Bytes[1:0]   | Length of the FREQ_START_ARRAY in CHIRP_LUT |
     * | Bytes[3:2]   | Length of the FREQ_SLOPE_ARRAY in CHIRP_LUT |
     * | Bytes[5:4]   | Length of the IDLE_TIME_ARRAY in CHIRP_LUT |
     * | Bytes[7:6]   | Length of the ADC_START_TIME_ARRAY in CHIRP_LUT |
     * | Bytes[9:8]   | Length of the TX_START_TIME_ARRAY in CHIRP_LUT |
     * | Bytes[11:10] | Length of the TX_EN_ARRAY in CHIRP_LUT |
     * | Bytes[13:12] | Length of the BPM_EN_ARRAY in CHIRP_LUT |
     * | Bytes[23:14] | Reserved |
     */
    UINT16 h_ParamArrayLen[12U];

    /*!
     * @brief  Repetition count (M) of chirp parameter (CHIRP_RPT_COUNT). \n
     * The chirp parameter will be used from param ARRAY for M chirps before switching new entry
     * into next row. \n
     * Valid range: 1 - 32768 \n
     * | Value     | Definition |
     * |-------    |----------- |
     * | 1 - 32768 | Index to ARRAY will be incremented once after every M chirps and corresponding ARRAY value is used. |
     * Byte Definition: \n
     * | Bytes        | Definition |
     * |-------       |----------- |
     * | Bytes[1:0]   | Repetition count of the FREQ_START parameter |
     * | Bytes[3:2]   | Repetition count of the FREQ_SLOPE parameter |
     * | Bytes[5:4]   | Repetition count of the IDLE_TIME parameter |
     * | Bytes[7:6]   | Repetition count of the ADC_START_TIME parameter |
     * | Bytes[9:8]   | Repetition count of the TX_START_TIME parameter |
     * | Bytes[11:10] | Repetition count of the TX_EN parameter |
     * | Bytes[13:12] | Repetition count of the BPM_EN parameter |
     * | Bytes[23:14] | Reserved |
     */
    UINT16 h_ParamRptCount[12U];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

} T_RL_API_SENS_PER_CHIRP_CFG;

/*!
 * @brief mmWaveLink @ref rl_sensPerChirpCtrl and @ref rl_sensPerChirpCtrlGet API
 * command/response Data structure \n
 * FECSS sensor per chirp control command data structure contains the per chirp parameter
 * enable / disable control and start address offset of PER_CHIRP_LUT RAM.  \n
 * This API also configures the h_ParamArrayStartAdd for all chirp parameters. \n
 * The parameter ARRAY start offset address h_ParamArrayStartAdd shall be multiple of 4 bytes.  \n
 * \note This API is Not Applicable for CW_CZ_TRIG Mode.
 */
typedef struct
{
    /*!
     * @brief  Per chirp parameter control settings. This field can be used to enable / disable
     * the per chirp parameters.  \n
     * | Value    | Definition |
     * |-------   |----------- |
     * |   0      | Per chirp parameter is Disabled. The chirp timer uses the chirp parameter settings from rl_sensChirpProfTimeCfg API. |
     * |   1      | Per chirp parameter is Enabled. The chirp timer uses the chirp parameter settings from PER_CHIRP_LUT as defined by rl_sensPerChirpCfg API. |
     * Bit Field Definition: \n
     * | Bit Field   | Definition |
     * |-------      |----------- |
     * | Bit 0       | FREQ_START parameter enable control  |
     * | Bit 1       | FREQ_SLOPE parameter enable control  |
     * | Bit 2       | IDLE_TIME parameter enable control  |
     * | Bit 3       | ADC_START parameter enable control  |
     * | Bit 4       | TX_START parameter enable control  |
     * | Bit 5       | TX_EN parameter enable control  |
     * | Bit 6       | BPM_EN parameter enable control  |
     * | Bits [15:7] | Reserved |
     */
    UINT16 h_PerChirpParamCtrl;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  The per chirp parameter ARRAY start offset address. This field is used to program
     * the start offset address of parameter ARRAY in PER_CHIRP_LUT RAM. \n
     * The start address offset shall be multiple of 4 bytes. The max addressable space 8kB. \n
     * Valid values: 0, 4, 8 to 0x1FFC \n
     * Byte Definition: \n
     * | Bytes        | Definition |
     * |-------       |----------- |
     * | Bytes[1:0]   | The start address offset of FREQ_START_ARRAY |
     * | Bytes[3:2]   | The start address offset of FREQ_SLOPE_ARRAY |
     * | Bytes[5:4]   | The start address offset of IDLE_TIME_ARRAY |
     * | Bytes[7:6]   | The start address offset of ADC_START_TIME_ARRAY |
     * | Bytes[9:8]   | The start address offset of TX_START_TIME_ARRAY |
     * | Bytes[11:10] | The start address offset of TX_EN_ARRAY |
     * | Bytes[13:12] | The start address offset of BPM_EN_ARRAY |
     * | Bytes[23:14] | Reserved |
     */
    UINT16 h_ParamArrayStartAdd[12U];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_SENS_PER_CHIRP_CTRL;

/*!
 * @brief mmWaveLink @ref rl_sensFrameCfg and @ref rl_sensFrameCfgGet API
 * command/response Data structure \n
 * FECSS sensor frame config command data structure contains the frame and burst chirp counts
 * and timing parameters.  \n
 * The sensor Frame contains following timing components.  \n
 *  -# Chirps
 *    - Chirp idle time
 *    - Chirp ramp end time
 *  -# Bursts
 *    - Number of chirps
 *    - Chirp Accumulation
 *    - Burst Periodicity
 *  -# Frames
 *    - Frame Periodicity
 * The active time of burst BURST_ACTIVE_TIME = NUM_CHIRPS_IN_BURST * NUM_CHIRPS_ACCUM *
 * (CHIRP_IDLE_TIME + CHIRP_RAMP_END_TIME). \n
 * The Burst periodicity BURST_PERIODICITY = BURST_ACTIVE_TIME + BURST_IDLE_TIME. \n
 * The active time of frame FRAME_ACTIVE_TIME = NUM_BURSTS_IN_FRAME * BURST_PERIODICITY. \n
 * The Frame periodicity FRAME_PERIODICITY = FRAME_ACTIVE_TIME + FRAME_IDLE_TIME.  \n
 * Refer section "Frame, Burst and Chirp Timing Information" in ICD for more details on
 * timing info. \n
 * \note This API is Not Applicable for CW_CZ_TRIG Mode.
 */
typedef struct
{
    /*!
     * @brief  Number of Chirps in a Burst. \n
     * This field indicates the number of chirps to be generated per burst. \n
     * Valid Range: 1 to 65535 chirps \n
     */
    UINT16 h_NumOfChirpsInBurst;

    /*!
     * @brief  Number of accumulation per chirp (NUM_CHIRPS_ACCUM) \n
     * This field indicates the Number of chirps to be accumulated before sending the ADC data
     * out in DFE, this can be used to increase the SNR without increasing the number of chirps
     * to process in the DSP/HW accelerator. \n
     * | Value     | Definition |
     * |-------    |----------- |
     * | 0         | Chirp accumulation feature is Disabled |
     * | 2 - 64    | Chirp accumulation feature is Enabled |
     * Valid Range: 0 to 64 chirps \n
     * The total chirps in a burst is c_NumOfChirpsAccum * h_NumOfChirpsInBurst. \n
     * If PER_CHIRP_CFG is enabled then the CHIRP_RPT_COUNT shall be integer multiple of
     * NUM_CHIRPS_ACCUM to avoid any change in chirp parameters during the accumulation.
     */
    UINT8 c_NumOfChirpsAccum;

    /*!
     * @brief Miscellaneous Setup Mask
     * This mask should be used to configure/ select miscellaneous settings. Bit-wise description
     * of available options is as follows:
     * | Bit(s)   | Description |
     * |----------|-------------|
     * | Bit 0    | Skip Frame Configuration @n Value 0: Configure frame timer and chirp timer @n Value 1: Skip frame timer configuration and only configure Chirp Timer |
     * | Bits 7-1 | Reserved |
     *
     * @b Details @n
     * -# <b> Skip Frame Timer Configuration </b>
     *    - @b Background: Frame configuration API configures burst related information to Chirp Timer,
     *      which is responsible for triggering bursts and chirps. This peripheral is associated with FECSS Sub-system.
     *      All the frame related parameters are passed to a separate peripheral called Frame Timer which is associated with
     *      AppSS.
     *    - @b Use-Case: Since frame timer runs on the XTAL, it can run in background even when FECSS and AppSS are
     *      powered down in inter-framer duration. Chirp timer (CT) runs on the APLL clock and hence looses all the configurations ones FECSS is powered off.
     *      CT must be reconfigured after exit from sleep. This option should be enabled only to configure CT while FT is still running
     */
    UINT8 c_MiscSetupMask;

    /*!
     * @brief  Burst Periodicity. This field indicates the period of the burst, 24bit counter. \n
     * The BURST_PERIODICITY = BURST_ACTIVE_TIME + BURST_IDLE_TIME. \n
     * Where active time of burst BURST_ACTIVE_TIME = NUM_CHIRPS_IN_BURST * NUM_CHIRPS_ACCUM *
     * (CHIRP_IDLE_TIME + CHIRP_RAMP_END_TIME).
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 8/APLL_FREQ (20ns with 40MHz XTAL), unsigned number \n Valid Range: 10 (0.2 us) to 16777215 (335 ms) |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 40/APLL_FREQ (100ns with 40MHz XTAL), unsigned number \n Valid Range: 10 (1 us) to 16777215 (1677 ms) |
     */
    UINT32 w_BurstPeriodicity;

    /*!
     * @brief  Number of bursts per frame. \n
     * This field indicates the number of bursts to be generated in a frame. \n
     * Valid range:1 to 4096
     */
    UINT16 h_NumOfBurstsInFrame;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Frame Periodicity. This field indicates the period of the frame, 32bit counter. \n
     * This field indicates the frame periodicity, the time gap between successive frame starts. \n
     * The FRAME_PERIODICITY = FRAME_ACTIVE_TIME + FRAME_IDLE_TIME. \n
     * Where active time of frame FRAME_ACTIVE_TIME = NUM_BURSTS_IN_FRAME * BURST_PERIODICITY. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL
     * clock frequencies. \n
     * Typical values: few 10us to 1 second. \n
     * Valid Range: 100 to 4294967295. \n
     * Note: This field is not applicable in HW trigger FRAME_TRIGGER_MODE.
     */
    UINT32 w_FramePeriodicity;

    /*!
     * @brief  Number of Frames. This field indicates the number of frames to generate. \n
     * Valid Range: 0 to 65535. \n
     * If value is zero (0) then frame timer generates endless frames. RL_SENSOR_START_STOP API
     * can be used to stop the chirps. \n
     * Note: This field is not applicable in HW trigger FRAME_TRIGGER_MODE. \n
     */
    UINT16 h_NumOfFrames;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Frame Timer Event 0 Interrupt Time configuration. \n
     * This field indicates the time at which General Purpose Interrupt 0 should be raised,
     * wrt the frame trigger start time FRAME_START_SIGNAL. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL
     * clock frequencies. \n
     * Valid Range: 0 to 4294967295 cycles \n
     * Refer FRAME_TRIGGER_MODE for FRAME_START_SIGNAL timing. \n
     * Note: If application is not interested in this event, Disable the interrupt in M4. This
     * event can be used to pre-configure the FECSS for next frame preparation. \n
     */
    UINT32 w_FrameEvent0TimeCfg;

    /*!
     * @brief  Frame Timer Event 1 Interrupt Time configuration. \n
     * This field indicates the time at which General Purpose Interrupt 1 should be raised,
     * wrt the frame trigger start time FRAME_START_SIGNAL. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL
     * clock frequencies. \n
     * Valid Range: 0 to 4294967295 cycles \n
     * Refer FRAME_TRIGGER_MODE for FRAME_START_SIGNAL timing. \n
     * Note: If application is not interested in this event, Disable the interrupt in M4. This
     * event can be used to pre-configure the FECSS for next frame preparation. \n
     */
    UINT32 w_FrameEvent1TimeCfg;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_SENS_FRAME_CFG;

/*!
 * @brief mmWaveLink @ref rl_sensSensorStart API command Data structure \n
 * FECSS sensor start command data structure contains the frame trigger mode configuration
 * for sensor start. \n
 * The multiple sensor start API trigger shall be avoided while frames are running in HW.
 * The RL_SENSOR_STATUS_GET API can be used to read the status of the sensor before issuing
 * successive sensor start or stop APIs.
 * @warning Application must wait for at least 100us after the frame end event before reading live monitor results
 */
typedef struct
{
    /*!
     * @brief  Frame Trigger Mode. \n
     * This field indicates how the application can instruct the FECSS frame / burst timer to
     * trigger the sensor frames or bursts. \n
     * The sensor Frame will be triggered based on the Mode selected in this API.
     * | Value | Definition |
     * |-------|----------- |
     * | 0     | Frame SW immediate trigger Mode (SW_TRIG). \n In this mode, the Frame Timer generates frames immediately upon detecting the trigger. \n The frame timer FRAME_START_SIGNAL starts immediately after this SW API trigger, the chirp timer will be triggered by CHIRP_START_SIGNAL after an inherent delay of 25us (Delta1 pre frame setup time), the 1st chirp of frame will start after a fixed delay of 65us (Delta2 pre burst setup time). \n The frame timer will stop after generating NUM_OF_FRAMES. |
     * | 1     | Frame SW timer based trigger Mode (SW_TIMER_TRIG). \n This mode is similar to SW Immediate Trigger Mode, except that instead of immediate action, the Frame Timer further waits for the Frame Timer's Frame Reference Timer to reach the value programmed in FRAME_TRIG_TIMER_VAL and then generates frames. \n The frame timer FRAME_START_SIGNAL starts once timer value matches the reference counter, the chirp timer will be triggered by CHIRP_START_SIGNAL after an inherent delay of 25us (Delta1 pre frame setup time), the 1st chirp of frame will start after a fixed delay of 65us (Delta2 pre burst setup time). \n The frame timer will stop after generating NUM_OF_FRAMES. |
     * | 2     | Frame HW trigger low power Mode (HW_LOW_PWR_TRIG). \n This mode is similar to SW Immediate Trigger Mode, except that instead of immediate SW trigger, the Frame Timer waits for a rising edge in the device's DIG_SYNC_IN input pin, and then generates a frames. \n The frame timer FRAME_START_SIGNAL starts immediately after raising edge of DIG_SYNC_IN pulse, the chirp timer will be triggered by CHIRP_START_SIGNAL after an inherent delay of 25us (Delta1 pre frame setup time), the 1st chirp of frame will start after a fixed delay of 65us us (Delta2 pre burst setup time). \n In this low power mode the DIG_SYNC_IN pulse is sampled by XTAL clock, hence the frame start jitter is higher. \n Note that only one frame is generated per trigger in this mode. The application shall issue RL_SENSOR_START API for every frame in advance to DIG_SYNC_IN pulse. \n The FRAME_PERIODICITY and NUM_OF_FRAMES are not applicable in HW trigger mode. \n Note: The application shall time keep and generate pre frame event to issue RL_SENSOR_START 10us in advance to DIG_SYNC_IN pulse every frame. The APLL clock and FECSS device shall be ON prior to RL_SENSOR_START.  \n This Trigger Mode is supported only in SINGLE_CHIP mode. |
     * | 3     | Frame HW trigger low jitter Mode (HW_LOW_JIT_TRIG). \n This mode is similar to HW Trigger low power Mode, except that the APLL clock to chirp timer is enabled before DIG_SYNC_IN pulse trigger for low jitter sampling and chirp timer waits for a rising edge of DIG_SYNC_IN pulse for chirp generation. \n The frame timer FRAME_START_SIGNAL starts immediately after this API trigger and the DIG_SYNC_IN pulse generates CHIRP_START_SIGNAL. The DIG_SYNC_IN pulse shall be generated after 25us (Delta1 pre frame setup time) from FRAME_START_SIGNAL, the 1st chirp of a frame will start after a fixed delay of 65us (Delta2 pre burst setup time). \n In this low jitter mode the DIG_SYNC_IN pulse is sampled by 400MHz APLL chirp timer clock, hence the power consumption is higher. \n Note that, only one frame is generated per trigger in this mode. The application shall issue RL_SENSOR_START API every frame in advance to DIG_SYNC_IN pulse. \n The FRAME_PERIODICITY and NUM_OF_FRAMES are not applicable in HW trigger mode. \n Note: The application shall time keep and generate pre frame event to issue RL_SENSOR_START 35us in advance to DIG_SYNC_IN pulse every frame. The APLL clock and FECSS device shall be ON prior to RL_SENSOR_START.  \n This Trigger Mode shall be used in MULTI_CHIP_SLAVE mode. This mode can be used in SINGLE_CHIP as well.  |
     * | 4     | CW CZ Trigger Mode (CW_CZ_TRIG). \n In this Continuous Wave Streaming CZ mode, sensor can transmit constant RF frequency CHIRP_RF_FREQ_START signal programmed in CHIRP_PROFILE continuously with zero slope for testing/characterization purpose. \n The RL_SENSOR_STOP API shall be issued to stop the CW signal.  \n Note: The Minimum Setup time 100us is needed before start of CW transfer. |
     * | 5     | Chirp Timer Override Trigger Mode (CT_OVRD_TRIG). \n This is a internal test mode in which Frame timer is bypassed and a single frame can be triggered directly using chirp timer. \n Note: The Minimum Setup time 100us and stop time 50us shall be considered in idle time of the each burst. The application shall wait for entire burst duration before invoking next mmWaveLink API.  |
     */
    UINT8 c_FrameTrigMode;

    /*!
     * @brief  Chirp Timer (CT) start signal loopback enable control. \n
     * This field is reserved in SINGLE_CHIP mode and Applicable only in MULTI_CHIP mode. \n
     * This field can be used to generate DIG_SYNC_OUT signal from CHIRP_START_SIGNAL, this
     * control can be used to support cascade operation, where multiple devices can be
     * connected in cascade, with one designated as primary device and others as secondary devices
     * and synchronize all device chirp timer trigger. \n The primary device's CHIRP_START_SIGNAL
     * signal is to be communicated to all devices through DIG_SYNC_OUT and DIG_SYNC_IN pins. For
     * this, the primary's CHIRP_START_SIGNAL signal is output on DIG_SYNC_OUT pin. The primary's
     * DIG_SYNC_OUT pin is connected on board to DIG_SYNC_IN pins of all secondary devices and the
     * primary device through (looped on board). The primary's CHIRP_START_SIGNAL_LB_EN can be set
     * so that the Chirp Timer uses the looped back DIG_SYNC_IN (looped back version of
     * CHIRP_START_SIGNAL) as its input CHIRP_START_SIGNAL, instead of directly using the Frame
     * Timer's CHIRP_START_SIGNAL, so as to start the Chirp Timers of all devices in a
     * delay-matched manner.
     * | Value | Definition |
     * |-------|----------- |
     * | 0     | CHIRP_START_SIGNAL to DIG_SYNC_OUT Loopback Disable \n Default in Single chip mode.     |
     * | 1     | CHIRP_START_SIGNAL to DIG_SYNC_OUT Loopback Enable. The chirp timer is triggered on looped back DIG_SYNC_IN pulse.  |
     * MULTI_CHIP mode settings: \n
     * | FRAME_TRIGGER_MODE | MULTI_CHIP_PRIMARY | MULTI_CHIP_SECONDARY |
     * |--------------------|------------------- | -------------------- |
     * | 0 (SW_TRIG)  | Primary device SW_TRIG mode. \n CHIRP_START_SIGNAL_LB_EN= 1 | NA |
     * | 1 (SW_TIMER_TRIG)  | Primary device SW_TIMER_TRIG mode. \n CHIRP_START_SIGNAL_LB_EN= 1 | NA |
     * | 2 (HW_LOW_PWR_TRIG)  | NA | NA |
     * | 3 (HW_LOW_JIT_TRIG)  | Primary device HW_LOW_JIT_TRIG mode. \n CHIRP_START_SIGNAL_LB_EN= 0 | Default Functional Mode. \n CHIRP_START_SIGNAL_LB_EN= 0 |
     * | 4 (CW_CZ_TRIG)  | CW CZ Test Mode \n CHIRP_START_SIGNAL_LB_EN=0 | CW CZ Test Mode \n CHIRP_START_SIGNAL_LB_EN=0 |
     * | 5 (CHIRP_TIMER_TRIG)  | NA | NA |
     */
    UINT8 c_ChirpStartSigLbEn;


    /*!
     * @brief  Frame Live monitors enable control. 1 bit control per monitor. This is applicable
     * only for ASIL-B safety devices, in QM device this is reserved. \n
     * The live monitor is enabled in FECSS based on this control for every frame, the application
     * shall be responsible to read the results end of each frame. Refer sensor monitor section
     * for more info. \n
     * | Value | Definition |
     * |-------|----------- |
     * | 0     | Live monitor Disabled  |
     * | 1     | Live monitor Enabled  |
     * Bitfield Definition:
     * | Bitfield | Definition |
     * |-------|----------- |
     * | Bit[0]  | SYNTH_FREQ_LIVE_MON enable control. \n The monitor configuration shall be done using RL_MONITOR_SYNTH_FREQ_CFG API before enabling this bit.  |
     * | Bit[1]  | RX_SATURATION_LIVE_MON enable control. \n The monitor configuration shall be done using RL_MONITOR_RX_SATURATION_CFG API before enabling this bit.  |
     * | Bit[2]  | GPADC_CTM_LIVE_MON enable control. \n The monitor configuration shall be done using RL_MONITOR_GPADC_CTM_CFG API before enabling this bit.  \n Note: Reserved for Debug purpose only |
     * | Bits[7:3]  | Reserved  |
     */
    UINT8 c_FrameLivMonEn;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  Frame Trigger Timer Value. 32bit counter value. \n
     * This field is applicable only if FRAME_TRIG_MODE is set to SW timer based trigger Mode.
     * The Frame Timer generates the first frame when the Frame Reference Timer reaches the value
     * programmed in this register. Starting then, it generates subsequent frames based on
     * FRAME_PERIODICITY. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL
     * clock frequencies. \n
     * Valid Range: 0 to 4294967295 cycles \n
     * Note: Application shall read the frame timer free running reference timer using
     * RL_SENSOR_STATUS_GET API and add offset to program this field.
     */
    UINT32 w_FrameTrigTimerVal;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

} T_RL_API_SENSOR_START_CMD;

/*!
 * @brief mmWaveLink @ref rl_sensSensorStop API command Data structure \n
 * FECSS sensor stop command data structure contains the frame stop mode configuration
 * for sensor stop. \n
 * This API shall be used to stop the sensor in forced mode while frames are running. \n
 * The RL_SENSOR_STATUS_GET API can be used to read the status of the sensor before issuing
 * successive sensor start or stop APIs.
 */
typedef struct
{
    /*!
     * @brief  Frame Stop Mode. \n
     * This field indicates how the application can instruct the FECSS frame / burst timer to stop
     * the sensor frames or bursts. \n
     * The sensor can be stopped based on the Mode selected in this API. \n
     * | Value | Definition |
     * |-------|----------- |
     * | 0     | Stop Frame at Frame Boundary. \n In this mode, the Frame Timer stops further frame transmissions after the end of currently ongoing frame. \n This mode can be used to stop the frame early while generating infinite or finite NUM_OF_FRAMES. |
     * | 1     | Stop CW CZ Mode. \n In this mode, the Chirp Timer stops further CW signal transmissions. \n Note: The Minimum stop time 50us is needed after end of CW transfer. The application shall wait for this stop time before invoking next mmWaveLink API. |
     * | 2     | Forced Timer Stop (Error recovery). \n This is a error recovery mode in which the Chirp and frame Timers are forcefully stopped and reset to recover from any error. \n The chirp and frame timer interrupts may not be generated symmetrically in this mode. It is recommended to perform a clean FECSS power down and power up after this event. |
     *
     */
    UINT8 c_FrameStopMode;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_SENSOR_STOP_CMD;

/*!
 * @brief mmWaveLink @ref rl_sensStatusGet API response Data structure \n
 * FECSS Sensor status response data structure provides the sensor chirp and frame timer chirp,
 * burst and frame counts. This API can be used to get frame timer reference counter time stamp. \n
 * This API can be used to read the status of the sensor before issuing successive sensor start or
 * stop APIs.
 */
typedef struct
{
    /*!
     * @brief  Chirp count value from sensor chirp timer HW. \n
     * Starts from 0 for the first chirp of the burst and counts up to
     * NUM_CHIRPS_IN_BURST -1 for the last chirp of the burst. \n
     * Valid Range: 0 to 65534
     */
    UINT32 w_ChirpCount;

    /*!
     * @brief  Burst count value from sensor chirp timer HW. \n
     * Starts from 0 for the first burst of the frame and counts up to NUM_BURSTS_IN_FRAME-1
     * for the last burst of the frame. \n
     * Valid Range: 0 to 4095
     */
    UINT16 h_BurstCount;

    /*!
     * @brief  Frame count value from sensor frame timer HW. \n
     * Starts from 0 for the first frame and counts up to NUM_OF_FRAMES-1 for the last frame.
     * Valid Range: 0 to 65534 \n
     * Note: In case of infinite frames the this counter value will rollover.
     */
    UINT16 h_FrameCount;

    /*!
     * @brief  Frame Reference Timer counter value. \n
     * This value indicates the current value of the Frame Reference Timer.
     * This can be used in conjunction with the Timer Based Trigger Mode SW_TIMER_TRIG. \n
     * The timer is a free running 32 bit up counter running on XTAL clock, starting from 0 at
     * device power-on and rolling over each time after reaching 2^32 - 1 to 0 and continuing. \n
     * Resolution: 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz,
     * 38.4MHz and 40MHz XTAL clock frequencies.
     * Valid Range: 0 to 4294967295
     */
    UINT32 w_FrameRefTimerVal;

    /*!
     * @brief  Burst Period Timer counter value. \n
     * Down Counter for counting the burst period. Starts from BURST_PERIODICITY-1 at the
     * beginning of the burst and counts up to end of active burst. \n
     * This indicates the current value of the Burst Period Timer. The timer is a 32 bit down
     * counter running on 100MHz ADC clock, starting from BURST_PERIODICITY - 1 to end of active
     * burst (start of the idle time of burst) during each burst's duration. The value read represent the
     * counts remaining till the end of current burst including idle time. Next burst will be triggered,
     * if applicable, ones this timer reaches zero.
     * of burst. \n
     * Resolution: 1 LSB = 4\APLL_FREQ \n
     * Valid Range: 0 to 4294967295
     */
    UINT32 w_BurstPeriodTimerVal;

    /*!
     * @brief  Frame Period Timer counter value. \n
     * This indicates the current value of the Frame Period Timer. The timer is a 32 bit down
     * counter running on XTAL clock, starting from FRAME_PERIODICITY - 1 to 0 during each
     * frame's duration. The value read in this register indicates the remaining time before the
     * end of ongoing frame. This counter mostly reads value zero end of all the frames. \n
     * Resolution: 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz
     * and 40MHz XTAL clock frequencies. \n
     * Valid Range: 0 to 4294967295 \n
     */
    UINT32 w_FramePeriodTimerVal;

    /*!
     * @brief  Frame Start and Stop trigger Status \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0         | Frame start / stop command is not honored   |
     * | 1         | Frame start / stop command is honored       |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | Frame Start command Status. \n Note: This status is updated only if sensor START API SW_TRIG mode is issued to start the frame.|
     * | bits [1]    | Frame Stop command Status. \n Note: This status is updated only if sensor STOP API is issued to stop the frame.|
     * | bits [7:2]  | Reserved |
     * These status bits are cleared in every frame start command before triggering the
     * frame in sensor hardware.
     */
    UINT8 c_FrameStartStopStatus;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3[2];

} T_RL_API_SENSOR_STATUS_RSP;

/*!
 * @brief mmWaveLink @ref rl_sensDynPwrSaveDis and @ref rl_sensDynPwrSaveStsGet API
 * command / response Data structure \n
 * FECSS sensor dynamic power save data structure contains the various inter-chirp and
 * inter-burst dynamic power save disable option to override the default mode of operation. \n
 * By default all the power saving modes are enabled in FECSS. \n
 */
typedef struct
{
    /*!
     * @brief  Sensor inter-chirp (during chirp idle time) dynamic power save disable override
     * mode. \n This field provides various RX and TX chain dynamic rapid power save disable
     * option in inter-chirp idle time.
     * | Value | Definition |
     * |-------|----------- |
     * |   0   | Dynamic Power Save Enable (default)|
     * |   1   | Dynamic Power Save Disable |
     * Bit field definition: \n
     * | Bit Field | Definition |
     * |-------|----------- |
     * | Bit 0 | TX PA POWERSAVE_DISABLE control |
     * | Bit 1 | TX LO POWERSAVE_DISABLE control |
     * | Bit 2 | RX RF POWERSAVE_DISABLE control |
     * | Bit 3 | RX BB POWERSAVE_DISABLE control |
     * | Bit 4 | RX LO POWERSAVE_DISABLE control |
     * | Bits [7:5] | Reserved |
     * By default these power save modes are enabled in FECSS. \n
     * Recommended settings:
     *
     * | Usecase | Recommended value for c_InterChirpPsDis  |
     * |---------------------------------------------| -------------------------------------------- |
     * | h_ChirpIdleTime >= max of (6us - tx start time, 3.1us) | 0x00 (Power save Enabled) |
     * | h_ChirpIdleTime >= max of (4.1us - tx start time, 3.1us) and h_ChirpIdleTime < max of (6us - tx start time, 3.1us) | 0x1C (Rx Power Save Disabled, Tx and LO Power Save Enabled) |
     * | h_ChirpIdleTime < max of (4.1us - tx start time, 3.1us) | Not Supported idle time |
     *
     * Note: The minimum idle time can not go lower than max of (4.1us - tx start time, 3.1us).
     *
     */
    UINT8 c_InterChirpPsDis;

    /*!
     * @brief Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

} T_RL_API_SENS_DYN_PWR_SAVE_DIS;

/*!
 * @brief mmWaveLink @ref rl_sensLoopBackCfg API command Data structure \n
 * FECSS sensor loopback config data structure contains the various sensor Analog and Digital
 * loopback configuration settings.  \n
 * This is a configuration only API, this configuration is retained in FECSS RAM during deep sleep
 * or warm reset. \n
 * @note The application can avoid configuring the loopback settings repeatedly in every deep
 * sleep cycle as this configuration is retained in FECSS RAM during deep sleep. \n
 *
 */
typedef struct
{
    /*!
     * @brief  The loopback signal generator frequency selection. \n
     * The valid loopback frequency can be configured as per below table. \n
     * This loopback signal generator is used to enable DIG_LOOPBACK, IFA_LOOPBACK,
     * LO_LOOPBACK, PA_LOOPBACK and EXTERNAL_WG_LOOPBACK.  \n
     * The digital loopback test signal is square wave based on the Loopback Signal Generator, with
     * full-scale amplitude. The RX1 & RX2 digital test signals have an inherent delay of 80ns &
     * 160ns wrt RX0, resulting in phase offsets across them. \n
     * The Recommended Loopback frequencies:
     * | Value | Loopback Frequency |
     * |-------|----------- |
     * |   2   | 12.5 MHz |
     * |   4   | 6.25 MHz |
     * |   5   | 5 MHz |
     * |   8   | 3.125 MHz |
     * |   9   | 2.778 MHz |
     * |   10   | 2.5 MHz |
     * |   20   | 1.25 MHz |
     * |   25   | 1 MHz |
     * |   40   | 625 kHz |
     * |   50   | 500 kHz |
     * |   80   | 312.5 kHz |
     * |   100   | 250 kHz |
     * |   200   | 125 kHz |
     * The Loopback signal frequency in MHz = (APLL_FREQ / 16) / c_LbFreqSel \n
     * Valid Range: 2 to 200. \n
     */
    UINT8 c_LbFreqSel;

    /*!
     * @brief  IFA Loopback amplitude Gain. \n
     * This gain setting is applicable when IFA loopback path is enabled. \n
     * Following table included the recommended loopback settings for given IFA gain
     * | IFA_GAIN (dB) | LOW_BIAS (Bit[7]) | DAC_CODE (Bit[6:0]) |
     * |-------|----------- | ----------- |
     * | -10   | 0 | 40 |
     * | -8    | 1 | 63 |
     * | -6    | 1 | 50 |
     * | -4    | 1 | 39 |
     * | -2    | 1 | 31 |
     * | 0     | 1 | 25 |
     * | 2     | 1 | 20 |
     * | 4     | 1 | 16 |
     * | 6     | 1 | 12 |
     * | 8     | 1 | 10 |
     * | 10    | 1 |  8 |
     *
     * Recommended Values: TBD
     */
    UINT8 c_IfaLbGainIndex;

    /*!
     * @brief  The Gain Setting for common LO and PA loopback buffer. \n
     * This gain setting is applicable for both LO and PA loopback paths. The buffer
     * gain in relative dB scale for nominal process and temperature. \n
     * | COMN_LB_GAIN (dB) | CODE (Bit[7:0]) |
     * |-------| ----------- |
     * |  0    |  48 |
     * | -1    |  29 |
     * | -2    |  25 |
     * | -3    |  24 |
     * | -5    |  23 |
     * | -6    |  22 |
     * | -7    |  21 |
     * | -8    |  20 |
     * | -10   |  19 |
     * | -11   |  18 |
     * | -13   |  17 |
     * | -14   |  16 |
     * | -18   |  15 |
     * | -19   |  0  |
     *
     * Recommended Values: TBD
     */
    UINT8 c_LoPaLbCmnGainIndex;

    /*!
     * @brief  The Gain Setting for LO loopback buffer. \n
     * This gain setting is applicable when LO loopback path is enabled. The buffer gain
     * in relative dB scale for nominal process and temperature. \n
     * | LO_LB_GAIN (dB) | CODE (Bit[7:0]) |
     * |-------| ----------- |
     * |  0    |  96 |
     * | -1    |  51 |
     * | -2    |  47 |
     * | -3    |  44 |
     * | -4    |  42 |
     * | -5    |  40 |
     * | -6    |  38 |
     * | -7    |  36 |
     * | -8    |  35 |
     * | -9    |  34 |
     * | -10   |  33 |
     * | -11   |  32 |
     * | -12   |  31 |
     * | -13   |  30 |
     * | -14   |  29 |
     * | -15   |  28 |
     * | -16   |  27 |
     * | -17   |  26 |
     * | -18   |  25 |
     * | -19   |  24 |
     * | -21   |  23 |
     * | -22   |  22 |
     * | -23   |  21 |
     * | -25   |  20 |
     * | -26   |  19 |
     * | -27   |  18 |
     * | -30   |  0  |
     *
     * Recommended Values: TBD
     */
    UINT8 c_LoLbGainIndex;

    /*!
     * @brief  The Gain Setting for PA loopback buffer. \n
     * This gain setting is applicable when PA loopback path is enabled. The buffer gain
     * in relative dB scale for nominal process and temperature. \n
     * | LO_LB_GAIN (dB) | CODE (Bit[7:0]) |
     * |-------| ----------- |
     * |  0    |  48 |
     * | -1    |  27 |
     * | -2    |  25 |
     * | -3    |  24 |
     * | -4    |  23 |
     * | -5    |  22 |
     * | -6    |  21 |
     * | -7    |  20 |
     * | -10   |  19 |
     * | -11   |  18 |
     * | -13   |  17 |
     * | -15   |  16 |
     * | -17   |  15 |
     * | -18   |  14 |
     * | -19   |  0  |
     *
     * Recommended Values: TBD
     */
    UINT8 c_PaLbGainIndex;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  The External waveguide LB CW_CZ_MODE TX BPM (Binary Phase Modulation) enable. \n
     * This field enables the TX BPM in a CW chirp, this is applicable only in CW_CZ_MODE. \n
     * | Value   | Definition |
     * |-------  |----------- |
     * | 0       | CW MODE TX BPM is Disabled |
     * | 1       | CW MODE TX BPM is Enabled  |
     * Bit definition: \n
     * | Bit Field   | Definition |
     * |-----------  |----------- |
     * | Bit 0       | CW MODE TX0 BPM Enable control |
     * | Bit 1       | CW MODE TX1 BPM Enable control |
     * | Bits [15:2] | Reserved |
     */
    UINT16 h_ExtLbTxBpmEnSel;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3[2];

} T_RL_API_SENS_LOOPBACK_CFG;

/*!
 * @brief mmWaveLink @ref rl_sensLoopBackEna API command Data structure \n
 * FECSS sensor loopback enable data structure contains the enable/disable options for various
 * sensor Analog and Digital loopback configuration settings.  \n
 * @note Only one loopback option can be enabled at a time. \n
 *
 */
typedef struct
{
    /*!
     * @brief  The sensor loopback (LOOPBACK_EN) enable/disable option. \n
     * Only one loopback can be enabled at a time. \n
     *
     * | Value | Definition |
     * |-------|----------- |
     * |   0x00   | All LOOPBACKs disable |
     * |   0x01   | DIG_LOOPBACK enable |
     * |   0x02   | IFA_LOOPBACK enable |
     * |   0x03   | LO_LOOPBACK enable |
     * |   0x04   | PA_LOOPBACK enable |
     * |   0x05   | Enable EXTERNAL_WG_LOOPBACK. \n In this mode there is no internal loopback is enabled, the CHIRP_TX_BPM_EN_SEL (PROFILE_TIME_CFG API) and LB_FREQ_SEL can be used to modulate the TX signal, which will be useful to measure the TX power at ADC using external short waveguide loopback.  |
     */
    UINT8 c_LbEnable;

    /*!
     * @brief  Reserved \n
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved \n
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_SENS_LOOPBACK_ENA;


/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/*!
 * @name mmWaveLink Library Sensor module API function calls
 * @{
 */

/*! @} */


#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF dfp_sensor.h
 */


