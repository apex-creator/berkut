/*!*****************************************************************************
 * @file fe_sensdriver.h
 *
 * @brief FECSSLib sensor driver header file.
 *
 * @b Description @n
 * This FECSS library sensor driver header file defines FECSS sensor driver data structure,
 * MACROs and functions.
 *
 *
 * @note
 * <B> © Copyright 2022, Texas Instruments Incorporated – www.ti.com </B>
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
 * @addtogroup FECSSLIB_SENSOR_DRV_MODULE Sensor device Driver API functions
 *  @{
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     09July2021  TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_SENSDRIVER_H
#define FE_SENSDRIVER_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <fecsslib/common/reg_chirptimer.h>
#include <fecsslib/common/reg_frametimer.h>

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Sensor Driver module MACROs
 * @{
 */

/*!
 * @brief The FECSS Sensor DFE RX CFG register address
 */
#define M_FE_SENS_DFE_RX_CFG_REG_ADDRESS        ((REG32*)0x51020004U)
/*!
 * @brief The FECSS Sensor Chirp Timer register start address
 */
#define M_FE_SENS_CT_REG_START_ADDRESS          ((T_CHIRPTIMER_REGS*)0x51000000U)
/*!
 * @brief The FECSS Sensor Frame Timer register start address
 */
#define M_FE_SENS_FT_REG_START_ADDRESS          ((T_FRAMETIMER_REGS*)0x5B000000U)
/*!
 * @brief The FECSS FT mode mux register address
 */
#define M_FE_SENS_F_MODE_MUX_REG_ADDRESS        ((REG32*)0x5A020190U)
/*!
 * @brief The FECSS Sensor Frame Timer reset register in APPSS
 */
#define M_FE_SENS_APPSS_FT_RESET_REG_ADDRESS    ((REG32*)0x56040088U)


/*************************** Sensor driver Control config values *****************************/
/*!
 * @brief The Sensor CT resolution mask
 */
#define M_FE_SENS_CT_RESOLUTION_MASK                ((UINT8)0x03U)

/*!
 * @brief The Sensor FT Frame start / stop key
 */
#define M_FE_SENS_FT_SENSOR_START_STOP_KEY          (0x07U)

/*!
 * @brief The Sensor FT / CT start / stop done clear
 */
#define M_FE_SENS_FT_CT_START_STOP_DONE_CLEAR       ((UINT32)0x03U)

/*!
 * @brief The Sensor timer reset key
 */
#define M_FE_SENS_TIMER_RESET_KEY                   ((UINT32)0x07U)

/*!
 * @brief The Sensor reset release key
 */
#define M_FE_SENS_TIMER_RESET_RELEASE_KEY           ((UINT32)0x00U)

/*!
 * @brief The FECSS Sensor Frame Timer reset mask
 */
#define M_FE_SENS_FT_RESET_REG_MASK                 ((UINT32)0x01C0U)

/*!
 * @brief The FECSS Sensor CRD enable and Dither enable register mask
 */
#define M_SENS_CRD_DITHER_EN_MASK                   ((UINT32)0x3U)

/*!
 * @brief The FECSS Sensor PA blank and DFE enable register mask
 */
#define M_SENS_PA_BLANK_DFE_EN_MASK                 ((UINT32)0x3U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Sensor module Type defines
 * @{
 */

/*!
 * @brief Sensor chirp common profile Digital Frontend (DFE) parameters Data structure
 */
typedef union
{
    struct
    {
        UINT32 b4_Reserved1                  :  4;           /*!< bits   3:  0 */
        UINT32 b8_DigSampRate                :  8;           /*!< bits  11:  4 */
        UINT32 b12_NumSamps                  : 12;           /*!< bits  23: 12 */
        UINT32 b2_FirSel                     :  2;           /*!< bits  25: 24 */
        UINT32 b3_OutBitSel                  :  3;           /*!< bits  28: 26 */
        UINT32 b1_Reserved2                  :  1;           /*!< bits  29: 29 */
        UINT32 b1_Reserved3                  :  1;           /*!< bits  30: 30 */
        UINT32 b1_Reserved4                  :  1;           /*!< bits  31: 31 */
    } bits;  /*!<Struct */
    UINT32 b32_Reg;  /*!<Union */
} U_FE_SENS_RX_DFE_REG;

/*!
 * @brief Sensor chirp common profile Chirp timer (CT) parameters Data structure
 */
typedef struct
{

    /*!
     * @brief  Chirp Profile TX BPM MIMO Pattern selection. This TX and BPM patterns are
     * generated across chirps in a frame as per below table.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | Disabled, Default setting. The CHIRP_TX_EN_SEL and CHIRP_TX_BPM_EN_SEL registers take effect in determining the TX enable and BPM controls to the TX AFE for all chirps in the frame. |
     * | 1       | TDMA_2TX pattern. The CHIRP_TX_EN_SEL register is ignored and is internally varied as [01; 10; 01; 10; …] in chirp 0, 1, 2, 3, … and so on in the frame. Here the 2 bits refer to [TX1; TX0]. Other TXs if present are always disabled. The CHIRP_TX_BPM_EN_SEL register continues to take effect directly. |
     * | 4       | BPM_2TX pattern. The CHIRP_TX_EN_SEL register takes effect directly for all chirps in the frame. But CHIRP_TX_BPM_EN_SEL register is ignored and is internally varied as [00; 10; 00; 10; …] in chirp 0, 1, 2, 3, … and so on in the frame. Here the 2 bits refer to [TX1; TX0] BPM. Other TXs if present are always having 0 BPM. |
     * Valid Values: 0, 1 and 4 \n
     * Note: If CHIRP_TX_MIMO_PAT_SEL is enabled (non zero) then the per chirp PER_CHIRP_TX_EN and
     * PER_CHIRP_BPM_EN settings are not supported. It is recommend to disable the per chirp
     * config for these parameters.
     */
    UINT8 c_ChirpTxMimoPatSel;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Chirp Profile Ramp end Time. This is a common ramp end time value for all chirps in
     * a frame. This field indicates the FMCW frequency ramping time in the chirp cycle. \n
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 20ns, unsigned number \n Valid Range: 1 to 65535 (1.3ms) |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 100ns, unsigned number \n Valid Range: 1 to 65535 (6.5ms) |
     * Ensure that the total frequency sweep is either within these ranges: \n
     * 77G Devices: 76 - 81 GHz \n
     * 60G Devices: 56 - 64GHz \n
     */
    UINT16 h_ChirpRampEndTime;

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
     */
    UINT8 c_MiscSettings;

    /*!
     * @brief  HPF Fast Init Duration. This duration shall be programmed in accordance with
     * ADC start time of chirps. \n
     * 1LSB = 100ns \n
     * Range: 5 to 100 \n
     * Default Value: 15 (1.5us) \n
     */
    UINT8 c_HpfFastInitDuration;

    /*!
     * @brief  CRD NSLOPE Magnitude. The chirp Control Ramp Down slope magnitude. The sign of the
     * slope is auto detected based on sign of the chirp programmed in sensor.  \n
     * | RF Frequency   | Resolution |
     * |-----------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * 400MHz / 2^24 = ~28.610 kHz/us , unsigned number \n
Valid range: 0 to 32767 (937MHz/us) |
     * | 77GHz | 1 LSB = 4 * 400 * 400MHz / 2^24 = ~38.147 kHz/us , unsigned number \n
Valid range: 0 to 32767 (1249MHz/us) |
     * Default Value: 0x0a00
     */
    UINT16 h_CrdNSlopeMag;

} T_FE_SENS_CT_COMN_PROFILE;

/*!
 * @brief Sensor chirp timer burst parameters Data structure
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
     * @brief  Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  Burst Periodicity. This field indicates the period of the burst, 24bit counter. \n
     * The BURST_PERIODICITY = BURST_ACTIVE_TIME + BURST_IDLE_TIME. \n
     * Where active time of burst BURST_ACTIVE_TIME = NUM_CHIRPS_IN_BURST * NUM_CHIRPS_ACCUM *
     * (CHIRP_IDLE_TIME + CHIRP_RAMP_END_TIME).
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 20ns, unsigned number \n Valid Range: 10 (0.2 us) to 16777215 (335 ms) |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 100ns, unsigned number \n Valid Range: 10 (1 us) to 16777215 (1677 ms) |
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

} T_FE_SENS_CT_BURST_CFG;

/*!
 * @brief Sensor frame timer frame parameters Data structure
 */
typedef struct
{
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
     * Note: Disable the interrupt in M4 if application is not interested in this event. This
     * event can be used to setup FECSS for next frame preparation. \n
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
     * Note: Disable the interrupt in M4 if application is not interested in this event. This
     * event can be used to setup FECSS for next frame preparation. \n
     */
    UINT32 w_FrameEvent1TimeCfg;

} T_FE_SENS_FT_FRAME_CFG;

/*!
 * @brief Sensor chirp timer Status parameters Data structure
 */
typedef struct
{
    /*!
     * @brief  Chirp count value from sensor chirp timer HW. \n
     * Starts from 0 for the first chirp of the burst and counts up to
     * (NUM_CHIRPS_ACCUM * NUM_CHIRPS_IN_BURST) -1 for the last chirp of the burst.
     */
    UINT32 w_ChirpCount;

    /*!
     * @brief  Burst count value from sensor chirp timer HW. \n
     * Starts from 0 for the first burst of the frame and counts up to NUM_BURSTS_IN_FRAME-1
     * for the last burst of the frame.
     */
    UINT16 h_BurstCount;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Burst Period Timer counter value. \n
     * Down Counter for counting the burst period. Starts from BURST_PERIODICITY-1 at the
     * beginning of the burst and counts up to 0. \n
     * This indicates the current value of the Burst Period Timer. The timer is a 32 bit down
     * counter running on 100MHz ADC clock, starting from BURST_PERIODICITY – 1 to 0 during each
     * burst’s duration. The value read in this register indicates the remaining time before the
     * end (start) of this (next) burst.
     * | Resolution Mode   | Resolution Definition |
     * |-----------------  |----------- |
     * | High (c_ChirpTimerResol b1=1) | 1 LSB = 20ns, unsigned number  |
     * | Low (c_ChirpTimerResol b1=0) | 1 LSB = 100ns, unsigned number  |
     */
    UINT32 w_BurstPeriodTimerVal;

} T_FE_SENS_CT_STATUS;

/*!
 * @brief Sensor frame timer Status parameters Data structure
 */
typedef struct
{
    /*!
     * @brief  Frame Start and Stop trigger Status \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0         | Frame start / stop command is not honored   |
     * | 1         | Frame start / stop command is honored       |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | Frame Start command Status |
     * | bits [1]    | Frame Stop command Status |
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
     * @brief  Frame count value from sensor frame timer HW. \n
     * Starts from 0 for the first frame and counts up to NUM_OF_FRAMES-1 for the last frame.
     */
    UINT16 h_FrameCount;

    /*!
     * @brief  Frame Reference Timer counter value. \n
     * This value indicates the current value of the Frame Reference Timer.
     * This can be used in conjunction with the Timer Based Trigger Mode SW_TIMER_TRIG. \n
     * The timer is a free running 32 bit up counter running on XTAL clock, starting from 0 at
     * device power-on and rolling over each time after reaching 2^32 – 1 to 0 and continuing. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL clock
     * frequencies.
     */
    UINT32 w_FrameRefTimerVal;

    /*!
     * @brief  Frame Period Timer counter value. \n
     * This indicates the current value of the Frame Period Timer. The timer is a 32 bit down
     *  counter running on XTAL clock, starting from FRAME_PERIODICITY – 1 to 0 during each
     * frame’s duration. The value read in this register indicates the remaining time before the
     * end (start) of this (next) frame. \n
     * 1 LSB = 1 XTAL clock period. Device supports 25MHz, 26MHz, 38.4MHz and 40MHz XTAL
     * clock frequencies.
     */
    UINT32 w_FramePeriodTimerVal;

} T_FE_SENS_FT_STATUS;

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
T_RETURNTYPE sens_configDfeParams(UINT8 c_devIndex, \
                const U_FE_SENS_RX_DFE_REG *p_dfeParams);
T_RETURNTYPE sens_dfeParamsGet(UINT8 c_devIndex, \
                U_FE_SENS_RX_DFE_REG *p_dfeParams);
T_RETURNTYPE sens_configCtComnParams(UINT8 c_devIndex, \
                const T_FE_SENS_CT_COMN_PROFILE *p_ctComnParams);
T_RETURNTYPE sens_ctComnParamsGet(UINT8 c_devIndex, \
                T_FE_SENS_CT_COMN_PROFILE *p_ctComnParams);
T_RETURNTYPE sens_configCtResolution(UINT8 c_devIndex, const UINT8 c_resolType);
T_RETURNTYPE sens_getCtResolution(UINT8 c_devIndex, UINT8 *p_resolType);
T_RETURNTYPE sens_configCtTimeParams(UINT8 c_devIndex, \
                const T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_ctTimeParams);
T_RETURNTYPE sens_ctTimeParamsGet(UINT8 c_devIndex, \
                T_RL_API_SENS_CHIRP_PROF_TIME_CFG *p_ctTimeParams);
T_RETURNTYPE sens_configCtPerChirpHw(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CFG *p_ctPerChirpCfgParams);
T_RETURNTYPE sens_ctPerChirpHwCfgGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CFG *p_ctPerChirpCfgParams);
T_RETURNTYPE sens_ctrlCtPerChirpHw(UINT8 c_devIndex, \
                const T_RL_API_SENS_PER_CHIRP_CTRL *p_ctPerChirpCtrl);
T_RETURNTYPE sens_ctPerChirpHwCtrlGet(UINT8 c_devIndex, \
                T_RL_API_SENS_PER_CHIRP_CTRL *p_ctPerChirpCtrl);
T_RETURNTYPE sens_configCtBurstParams(UINT8 c_devIndex, \
                const T_FE_SENS_CT_BURST_CFG *p_ctBurstParams);
T_RETURNTYPE sens_ctBurstParamsGet(UINT8 c_devIndex, \
             T_FE_SENS_CT_BURST_CFG *p_ctBurstParams);
T_RETURNTYPE sens_configFtFrameParams(UINT8 c_devIndex, \
                const T_FE_SENS_FT_FRAME_CFG *p_ftFrameParams);
T_RETURNTYPE sens_ftFrameParamsGet(UINT8 c_devIndex, \
                T_FE_SENS_FT_FRAME_CFG *p_ftFrameParams);
T_RETURNTYPE sens_ftSensorStart(UINT8 c_devIndex, \
                const T_RL_API_SENSOR_START_CMD *p_ftSensStartParams);
T_RETURNTYPE sens_ftSensorStop(UINT8 c_devIndex);
T_RETURNTYPE sens_ftSensorForceStop(UINT8 c_devIndex);
T_RETURNTYPE sens_ctSensorStatus(UINT8 c_devIndex, \
                T_FE_SENS_CT_STATUS *p_ctSensSts);
T_RETURNTYPE sens_ftSensorStatus(UINT8 c_devIndex, \
                T_FE_SENS_FT_STATUS *p_ftSensSts);

#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF fe_sensdriver.h
 */


