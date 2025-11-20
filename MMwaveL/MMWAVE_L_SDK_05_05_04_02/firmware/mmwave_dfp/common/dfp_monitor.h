/*!*****************************************************************************
 * @file dfp_monitor.h
 *
 * @brief DFP mmWaveLink Monitor Module API Command and Response data structure header file
 *
 * @b Description @n
 * This is a mmWaveLink Monitor Module API Command and Response data structure
 * header file, same API data structure being used between FECSS and mmWaveLink Libraries
 *
 * @warning Application developer / User shall review Monitor APIs data structure,
 * API functions and handle the Errors.
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
 * @addtogroup MMWL_MONITOR_API mmWaveLink Monitor Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     19Sep2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef DFP_MONITOR_H
#define DFP_MONITOR_H

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
 * @name mmWaveLink Library T_RL_API_MON_ENABLE_TRIG Data Structure field value macros
 * @{
 */

/*!
 * @brief FECSS RFS Monitor controls start bit (Member .l_MonitorEnable)
 */
#define M_RL_MON_PLL_CTRL_VOLT                     ((UINT32)0U)
#define M_RL_MON_TX0_RX_LB                         ((UINT32)1U)
#define M_RL_MON_TX1_RX_LB                         ((UINT32)2U)
#define M_RL_MON_TX2_RX_LB                         ((UINT32)3U)
#define M_RL_MON_TX3_RX_LB                         ((UINT32)4U)
#define M_RL_MON_TX4_RX_LB                         ((UINT32)5U)
#define M_RL_MON_TX5_RX_LB                         ((UINT32)6U)
#define M_RL_MON_TX6_RX_LB                         ((UINT32)7U)
#define M_RL_MON_TX7_RX_LB                         ((UINT32)8U)
#define M_RL_MON_TX0_POWER                         ((UINT32)9U)
#define M_RL_MON_TX1_POWER                         ((UINT32)10U)
#define M_RL_MON_TX2_POWER                         ((UINT32)11U)
#define M_RL_MON_TX3_POWER                         ((UINT32)12U)
#define M_RL_MON_TX4_POWER                         ((UINT32)13U)
#define M_RL_MON_TX5_POWER                         ((UINT32)14U)
#define M_RL_MON_TX6_POWER                         ((UINT32)15U)
#define M_RL_MON_TX7_POWER                         ((UINT32)16U)
#define M_RL_MON_TX0_BB                            ((UINT32)17U)
#define M_RL_MON_TX1_BB                            ((UINT32)18U)
#define M_RL_MON_TX2_BB                            ((UINT32)19U)
#define M_RL_MON_TX3_BB                            ((UINT32)20U)
#define M_RL_MON_TX4_BB                            ((UINT32)21U)
#define M_RL_MON_TX5_BB                            ((UINT32)22U)
#define M_RL_MON_TX6_BB                            ((UINT32)23U)
#define M_RL_MON_TX7_BB                            ((UINT32)24U)
#define M_RL_MON_TX0_INTRNAL_DC_SIG                ((UINT32)25U)
#define M_RL_MON_TX1_INTRNAL_DC_SIG                ((UINT32)26U)
#define M_RL_MON_TX2_INTRNAL_DC_SIG                ((UINT32)27U)
#define M_RL_MON_TX3_INTRNAL_DC_SIG                ((UINT32)28U)
#define M_RL_MON_TX4_INTRNAL_DC_SIG                ((UINT32)29U)
#define M_RL_MON_TX5_INTRNAL_DC_SIG                ((UINT32)30U)
#define M_RL_MON_TX6_INTRNAL_DC_SIG                ((UINT32)31U)
#define M_RL_MON_TX7_INTRNAL_DC_SIG                ((UINT32)32U)
#define M_RL_MON_RX_HPF_INTRNAL_DC_SIG             ((UINT32)33U)
#define M_RL_MON_PM_CLK_INTRNAL_DC_SIG             ((UINT32)34U)
#define M_RL_MON_DFE_BIST_FFT_CHECK                ((UINT32)35U)
#define M_RL_MON_STATIC_REG_READBACK               ((UINT32)36U)

#define M_RL_MAX_FECSS_MONITORS                 ((UINT8)37U)

/*!
 * @brief FECSS RFS Monitor Fault Injection controls start bit (Member .w_FaultInjEnable)
 */
#define M_RL_MON_FAULT_SYNTH                    ((UINT32)0U)
#define M_RL_MON_FAULT_RX_IFA_GAIN_DROP         ((UINT32)1U)
#define M_RL_MON_FAULT_RX_HIGH_NOISE            ((UINT32)2U)
#define M_RL_MON_FAULT_RX_HPF_HIGH_CUTOFF       ((UINT32)3U)
#define M_RL_MON_FAULT_TX_GAIN_DROP             ((UINT32)4U)
#define M_RL_MON_FAULT_TX_PHASE_INVERSION       ((UINT32)5U)
#define M_RL_MON_FAULT_GPADC                    ((UINT32)6U)
#define M_RL_MON_FAULT_BIST_FFT_GAIN            ((UINT32)7U)
#define M_RL_MON_FAULT_FECSS_STATIC_REG         ((UINT32)8U)

#define M_RL_MAX_FECSS_MON_FAULTS               ((UINT8)9U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_MON_LIVE_SYNTH_FREQ_CFG Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .xc_ChirpMonStartTime Field values
 */
#define M_RL_MON_LIVE_SYNTH_FREQ_START_TIME_MIN     ((SINT8)-125)
#define M_RL_MON_LIVE_SYNTH_FREQ_START_TIME_MAX     ((SINT8)125)

/*!
 * @brief Member .h_FreqErrThreshold Field values
 */
#define M_RL_MON_LIVE_SYNTH_FREQ_ERR_THRESH_MIN     ((UINT16)1U)
#define M_RL_MON_LIVE_SYNTH_FREQ_ERR_THRESH_MAX     ((UINT16)65000U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_MON_LIVE_GPADC_CTM_INSTR Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_NumOfGpadcInstr Field values
 */
#define M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MIN       ((UINT8)1U)
#define M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MAX       ((UINT8)64U)

/*! @} */


/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Monitor Module API Type defines
 * @{
 */

/*!
 * @brief mmWaveLink @ref rl_monEnableTrig API command Data structure \n
 * This API shall be used to enable and trigger the monitors. All the enabled monitors will be
 * triggered and result will be stored in a predefined IPC RAM with fixed address location,
 * upon completion of all enabled monitors, the FECSS RFS core will send a IRQ event
 * @ref FEC_MON_EVENT (FEC_INTR2 - IRQ[4]) to APPSS (application core).
 * The application is responsible for reading out all the results from the predefined address.
 * The collective execution status of all the monitors are stored in the address 0x21200500,
 * upon receiving FEC_MON_EVENT, the APPSS can read this collective status before reading each of
 * the individual results. \n
 * The fault injection can be done to each of these enabled monitors in this API. Following table
 * shows the list of faults that can be injected for monitors. RFS firmware will automatically
 * inject the selected faults before running the corresponding monitor and remove it immediately,
 * the table shows the impacted monitors for each fault type injected.
 * \n
 * @note
 * -# Faults enabled for live monitors are not removed automatically
 * -# All loopbacks must be disabled before triggering monitors and injecting faults
 *
 *
 * | Monitors   \        Faults | Synth Fault | Rx IFA Gain Drop | Rx High Noise | HPF High Cutoff | Tx Gain Drop | Tx Phase Inversion | GPADC Fault | BIST FFT Gain Drop | Static Register Fault |
 * |----------------------------|-------------|------------------|---------------|-----------------|--------------|--------------------|-------------|--------------------|-----------------------|
 * | PLL_CTRL_VOLT_MON          |      x      |                  |               |                 |              |                    |             |                    |                       |
 * | TX0_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX1_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX2_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX3_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX4_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX5_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX6_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX7_RX_LB_MON              |             |         x        |               |    (Optional)   |              |          x         |             |                    |                       |
 * | TX0_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX1_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX2_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX3_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX4_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX5_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX6_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX7_POWER_MON              |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX0_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX1_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX2_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX3_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX4_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX5_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX6_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX7_BALL_BREAK_MON         |             |                  |               |                 |       x      |                    |             |                    |                       |
 * | TX0_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX1_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX2_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX3_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX4_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX5_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX6_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | TX7_INTRNAL_DC_SIG_MON     |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | RX_HPF_INTRNAL_DC_SIG_MON  |             |    (Optional)    |               |         x       |              |                    |      x      |                    |                       |
 * | PM_CLK_INTRNAL_DC_SIG_MON  |             |                  |               |                 |              |                    |      x      |                    |                       |
 * | DFE_BIST_FFT_MON           |             |                  |               |                 |              |                    |             |           x        |                       |
 * | STATIC_REG_READBACK        |             |                  |               |                 |              |                    |             |                    |            x          |
 * | Live Monitors              |      x      |                  |       x       |                 |              |                    |             |                    |                       |
 */
typedef struct
{
    /*!
     * @brief  FECSS Monitors enable control, 1 bit control per monitor.  \n
     *
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Monitor Disabled    |
     * | 1      | Monitor Enabled    |
     * Bit Field Definition:
     * | BitField   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | PLL_CTRL_VOLT monitor On/Off control |
     * | Bit[1]  | TX0_RX_LB monitor On/Off control |
     * | Bit[2]  | TX1_RX_LB monitor On/Off control |
     * | Bit[3]  | TX2_RX_LB monitor On/Off control (Reserved) |
     * | Bit[4]  | TX3_RX_LB monitor On/Off control (Reserved) |
     * | Bit[5]  | TX4_RX_LB monitor On/Off control (Reserved) |
     * | Bit[6]  | TX5_RX_LB monitor On/Off control (Reserved) |
     * | Bit[7]  | TX6_RX_LB monitor On/Off control (Reserved) |
     * | Bit[8]  | TX7_RX_LB monitor On/Off control (Reserved) |
     * | Bit[9]  | TX0_POWER monitor On/Off control |
     * | Bit[10] | TX1_POWER monitor On/Off control |
     * | Bit[11] | TX2_POWER monitor On/Off control (Reserved) |
     * | Bit[12] | TX3_POWER monitor On/Off control (Reserved) |
     * | Bit[13] | TX4_POWER monitor On/Off control (Reserved) |
     * | Bit[14] | TX5_POWER monitor On/Off control (Reserved) |
     * | Bit[15] | TX6_POWER monitor On/Off control (Reserved) |
     * | Bit[16] | TX7_POWER monitor On/Off control (Reserved) |
     * | Bit[17] | TX0_BB monitor On/Off control |
     * | Bit[18] | TX1_BB monitor On/Off control |
     * | Bit[19] | TX2_BB monitor On/Off control (Reserved) |
     * | Bit[20] | TX3_BB monitor On/Off control (Reserved) |
     * | Bit[21] | TX4_BB monitor On/Off control (Reserved) |
     * | Bit[22] | TX5_BB monitor On/Off control (Reserved) |
     * | Bit[23] | TX6_BB monitor On/Off control (Reserved) |
     * | Bit[24] | TX7_BB monitor On/Off control (Reserved) |
     * | Bit[25] | TX0_INTRNAL_DC_SIG monitor On/Off control |
     * | Bit[26] | TX1_INTRNAL_DC_SIG monitor On/Off control |
     * | Bit[27] | TX2_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[28] | TX3_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[29] | TX4_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[30] | TX5_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[31] | TX6_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[32] | TX7_INTRNAL_DC_SIG monitor On/Off control (Reserved) |
     * | Bit[33] | RX_HPF_INTRNAL_DC_SIG monitor On/Off control |
     * | Bit[34] | PM_CLK_INTRNAL_DC_SIG monitor On/Off control |
     * | Bit[35] | DFE_BIST_FFT_CHECK monitor On/Off control |
     * | Bit[36] | STATIC_REG_READBACK monitor On/Off control |
     * | Bit[63:37]  | Reserved   |
     *
     * Note: Application can trigger monitor with atleast one monitor enabled and expect
     * IRQ [4] - FEC_INTR2 using this API.
     */
    UINT32 w_MonitorEnable[2U];

    /*!
     * @brief  FECSS Monitors Fault Injection enable control, 1 bit control per monitor. \n
     * In case of fault injection, application has to inject one of these or multiple faults
     * according to app note and trigger the API.
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Fault Injection Disabled    |
     * | 1      | Fault Injection Enabled     |
     * Bit Field Definition:
     * | BitField   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | SYNTH_FAULT ON/OFF control   |
     * | Bit[1]  | RX_IFA_GAIN_DROP ON/OFF control   |
     * | Bit[2]  | RX_HIGH_NOISE ON/OFF control   |
     * | Bit[3]  | RX_HPF_HIGH_CUTOFF ON/OFF control   |
     * | Bit[4]  | TX_GAIN_DROP ON/OFF control   |
     * | Bit[5]  | TX_PHASE_INVERSION ON/OFF control   |
     * | Bit[6]  | GPADC_REF_VOLT_FAULT ON/OFF control   |
     * | Bit[7]  | DFE_BIST_FFT_FAULT ON/OFF control   |
     * | Bit[8]  | FECSS_STATIC_REG_FAULT ON/OFF control   |
     * | Bit[31:9]  | Reserved   |
     */
    UINT32 w_FaultInjEnable;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_ENABLE_TRIG;

/*!
 * @brief mmWaveLink @ref rl_monEnableTrig API response Data structure \n
 * FECSS Monitor collective response data structure contains the Monitor execution status.
 * The APPSS shall read this response upon receiving FEC_MON_EVENT (IRQ[4]). \n
 * Result IPC RAM base Address: 0x21200500 \n
 * Size: 20 bytes
 */
typedef struct
{
    /*!
     * @brief  FECSS Monitors run status, 1 bit status per monitor.
     * This is the status of the current monitor enable command trigger. \n
     *
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | NU / FAIL - Not Updated, Monitor trigger is disabled / Enabled Monitor has failed to complete.    |
     * | 1      | PASS - Enabled Monitor has successfully executed.    |
     * Bit Field Definition:
     * | BitField   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | PLL_CTRL_VOLT monitor status |
     * | Bit[1]  | TX0_RX_LB monitor status |
     * | Bit[2]  | TX1_RX_LB monitor status |
     * | Bit[3]  | TX2_RX_LB monitor status (Reserved) |
     * | Bit[4]  | TX3_RX_LB monitor status (Reserved) |
     * | Bit[5]  | TX4_RX_LB monitor status (Reserved) |
     * | Bit[6]  | TX5_RX_LB monitor status (Reserved) |
     * | Bit[7]  | TX6_RX_LB monitor status (Reserved) |
     * | Bit[8]  | TX7_RX_LB monitor status (Reserved) |
     * | Bit[9]  | TX0_POWER monitor status |
     * | Bit[10] | TX1_POWER monitor status |
     * | Bit[11] | TX2_POWER monitor status (Reserved) |
     * | Bit[12] | TX3_POWER monitor status (Reserved) |
     * | Bit[13] | TX4_POWER monitor status (Reserved) |
     * | Bit[14] | TX5_POWER monitor status (Reserved) |
     * | Bit[15] | TX6_POWER monitor status (Reserved) |
     * | Bit[16] | TX7_POWER monitor status (Reserved) |
     * | Bit[17] | TX0_BB monitor status |
     * | Bit[18] | TX1_BB monitor status |
     * | Bit[19] | TX2_BB monitor status (Reserved) |
     * | Bit[20] | TX3_BB monitor status (Reserved) |
     * | Bit[21] | TX4_BB monitor status (Reserved) |
     * | Bit[22] | TX5_BB monitor status (Reserved) |
     * | Bit[23] | TX6_BB monitor status (Reserved) |
     * | Bit[24] | TX7_BB monitor status (Reserved) |
     * | Bit[25] | TX0_INTRNAL_DC_SIG monitor status |
     * | Bit[26] | TX1_INTRNAL_DC_SIG monitor status |
     * | Bit[27] | TX2_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[28] | TX3_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[29] | TX4_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[30] | TX5_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[31] | TX6_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[32] | TX7_INTRNAL_DC_SIG monitor status (Reserved) |
     * | Bit[33] | RX_HPF_INTRNAL_DC_SIG monitor status |
     * | Bit[34] | PM_CLK_INTRNAL_DC_SIG monitor status |
     * | Bit[35] | DFE_BIST_FFT_CHECK monitor status |
     * | Bit[36] | STATIC_REG_READBACK monitor status |
     * | Bit[63:37]  | Reserved   |
     */
    UINT32 w_MonitorStatus[2U];

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_TRIG_RESPONSE;


/*!
 * @brief mmWaveLink @ref rl_monDbgPdMeas API command Data structure \n
 * This API shall be used to measure the PD power under certain test condition. \n
 * The FECSS monitor Debug PD measurement configuration command data structure
 * contains the PD measurement configurations.
 */
typedef struct
{
    /*!
     * @brief  Unique PD ID for measurement.
     * - Applicable only for critical PDs
     * - Use 0xFF for non-critical PDs
     */
    UINT8 c_PdId;

    /*!
     * @brief  PD LNA index for measurement.  \n
     */
    UINT8 c_PdLnaIndx;

    /*!
     * @brief  PD LNA Gain index for measurement
     * - 1LSB = 4dB
     * - Range: 1 to 9
     */
    UINT8 c_PdLnaGainIndx;

    /*!
     * @brief  PD TYPE.  \n
     * 0 - HPPD \n
     * 1 - LPPD
     */
    UINT8 c_PdType;

    /*!
     * @brief  PD Select for PD_LNA mux.
     * - Applicable only for non-critical PDs
     */
    UINT8 c_PdSelect;

    /*!
     * @brief  PD Signed magnitude format of the offset DAC value \n
     */
    UINT8 c_PdLnaOffsetDacVal;

    /*!
     * @brief  PD Number of GPADC measurements for average.  \n
     */
    UINT8 c_PdNAccum;

    /*!
     * @brief  PD Number of samples in each GPADC measurement.  \n
     */
    UINT8 c_PdNSamples;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_DBG_PD_MEAS_CMD;

/*!
 * @brief mmWaveLink @ref rl_monDbgPdMeas API response Data structure \n
 * FECSS PD Meas response data structure contains the PD Measurements.
 */
typedef struct
{
    /*!
     * @brief  FECSS PD measurement Accumulated GPADC values when RF is ON.
     */
    UINT32 w_SumRfON;

    /*!
     * @brief  FECSS PD measurement Accumulated GPADC values when RF is OFF.
     */
    UINT32 w_SumRfOff;

    /*!
     * @brief  FECSS PD measurement difference between w_SumRfON and w_SumRfOff \n
     * This is a GPADC code
     */
    UINT16 h_DeltaSum;

    /*!
     * @brief  FECSS PD measurement Power at the PD \n
     * 1 LSB = 0.1 dBm
     */
    SINT16 xh_PdPwrDb;

    /*!
     * @brief  FECSS PD measurement Squared RMS voltage at PD \n
     * 1 LSB = (31.25 uV)^2
     */
    UINT32 w_VpdRmsSqr;

    /*!
     * @brief  PD Measurement Status. \n
     * 0 - Fail \n 1 - Pass
     */
    UINT8 c_PdMeasSts;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved0;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_DBG_PD_MEAS_RSP;

/*!
 * @brief mmWaveLink @ref rl_monDbgTxPwrMeas API command Data structure \n
 * This API shall be used to measure the TX Power power under certain test condition. \n
 * The FECSS monitor Debug TX Power measurement configuration command data structure
 * contains the TX Power measurement configurations.
 */
typedef struct
{
    /*!
     * @brief  TX Index to be selected for measurement.  \n
     */
    UINT8 c_TxIndex;

    /*!
     * @brief  TX Power Number of GPADC measurements for average.  \n
     */
    UINT8 c_TxPwrNAccum;

    /*!
     * @brief  TX Back-off to be selected for measurement. \n
     * 1LSB: 0.5dB \n Valid Range: Even values (1dB step) from 0 (0dB) to 52 (26dB)
     */
    UINT8 c_TxBackoff;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved0;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_DBG_TXPWR_MEAS_CMD;

/*!
 * @brief mmWaveLink @ref rl_monDbgTxPwrMeas API response Data structure \n
 * FECSS TX Power Meas response data structure contains the TX Power Measurements.
 */
typedef struct
{
    /*!
     * @brief  FECSS TX Power measurement, Power measured from incident PD \n
     * 1 LSB = 0.1 dB, signed
     */
    SINT16 xh_TxIncidentPwr;

    /*!
     * @brief  FECSS TX Power measurement, Power measured from reflected PD \n
     * 1 LSB = 0.1 dB, signed
     */
    SINT16 xh_TxReflectedPwr;

    /*!
     * @brief  FECSS TX Power measurement, Voltage measured from incident PD  \n
     * 1 LSB = (31.25 uV)^2
     */
    UINT32 w_TxIncidentVolt;

    /*!
     * @brief  FECSS TX Power measurement, Voltage measured from reflected PD  \n
     * 1 LSB = (31.25 uV)^2
     */
    UINT32 w_TxReflectedVolt;

    /*!
     * @brief Corrected Tx Power
     * 1 LSB = 0.1dBm
     */
    SINT16 xh_TxPower;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_DBG_TXPWR_MEAS_RSP;

/*!
 * @brief mmWaveLink @ref rl_monLiveSynthFreqCfg API command Data structure \n
 * This API shall be used to configure the monitors. The live monitors can be enabled in sensor
 * start API, this live monitor runs in back ground when functional frames or test burst are
 * running. \n
 * The monitor result T_RL_API_MON_LIVE_SYNTH_FREQ_RESULT will be updated every end of active
 * frame in address 0x21200514. \n
 * @note
 * - The application can avoid configuring the Monitor repeatedly in every deep sleep cycle
 * as this configuration is retained in FEC RAM during deep sleep.
 * - Application must wait for at least 100us after the frame end event before reading live monitor results
 */
typedef struct
{
    /*!
     * @brief  Synth Frequency monitor start time. \n
     * This field determines when the monitoring starts in each chirp relative to the start of the
     * ramp. \n
     * 1 LSB = 80/APLL_FREQ (Typical 0.2 μs with 40MHz XTAL), signed number \n
     * Valid range:  -125 to +125 (-25 to 25 μs) \n
     * Recommended value: 4μs or above   \n
     */
    SINT8 xc_ChirpMonStartTime;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved1;

    /*!
     * @brief Synth Frequency monitor error threshold. Unsigned Number. \n
     * In Synth freq monitor the error of the measured instantaneous chirp frequency
     * w.r.t. the desired value is continuously compared against this threshold
     * during the live chirp. The comparison result is part of the monitoring report
     * message (Error bit is set if the measurement is above this threshold,
     * ever during the previous monitoring frame period). \n
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * (APLL_FREQ) / (144 * 2^8) = ~32.552 kHz \n  |
     * | 77GHz | 1 LSB = 4 * (APLL_FREQ) / (144 * 2^8) = ~43.408 kHz \n  |
     * Valid range: 1 to 65000  \n
     */
    UINT16 h_FreqErrThreshold;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_LIVE_SYNTH_FREQ_CFG;

/*!
 * @brief mmWaveLink @ref rl_monLiveSynthFreqCfg API response Data structure \n
 * The FECSS  SYNTH_FREQ monitor result data structure contains the monitor result. The result is
 * updated in IPC RAM every end of active frame interval, the application shall read the result
 * every frame. \n
 * Result IPC RAM base Address: 0x21200514 \n
 * Size: 24 bytes
 * @warning Application must wait for at least 100us after the frame end event before reading live monitor results
 */
typedef struct
{
    /*!
     * @brief  Monitor PASS/ FAIL Status. \n
     * Status flag indicating pass fail results corresponding to threshold checks under this
     * monitor. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Fail    |
     * | 1      | Pass     |
     *
     */
    UINT8 c_MonStatus;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved1;

    /*!
     * @brief Reserved.
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Max Synth Frequency error value. Signed Number. \n
     * This field indicates the maximum instantaneous frequency error measured during the last
     * monitoring frame period for which frequency monitoring has been enabled. \n
     * In Synth freq monitor the error of the measured instantaneous chirp frequency w.r.t. the
     * desired value. \n
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * (APLL_FREQ) / (144 * 2^13) = ~1.017 kHz \n  |
     * | 77GHz | 1 LSB = 4 * (APLL_FREQ) / (144 * 2^13) = ~1.356 kHz \n  |
     * Valid range: -2097152 to 2097151  \n
     */
    SINT32 xw_MaxSynthFreqErr;

    /*!
     * @brief  Frequency error Failure count. \n
     * This field indicates the number of times during chirping in the previous monitoring frame
     * period in which the measured frequency error violated the allowed threshold. \n
     * Frequency error threshold violation is counted every 10 ns. \n
     * Valid range: 0 to 524287.
     */
    UINT32 w_FreqErrFailCount;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     * @warning Value of this counter lags by 1 as the results are generated before the Frame End Event
     */
    UINT32 w_FrameCount;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

} T_RL_API_MON_LIVE_SYNTH_FREQ_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monLiveRxSaturationCfg API command Data structure \n
 * This API shall be used to configure the monitor result storage start address. The live monitors
 * can be enabled in sensor start API, this live monitor runs in background when
 * functional frames or test bursts are running. \n
 * The monitor result RL_MON_RX_SATURATION_RESULT is updated every chirp while framing in
 * PER_CHIRP_LUT RAM ARRAY (Base address 0x21880000) at offset address h_PerChirpRamStartAdd
 * programmed in this API.
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle
 * as this configuration is retained in FEC RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  The per chirp RAM start offset address for saturation monitor result storage. \n
     * This field is used to program the start offset address of saturation monitor result ARRAY
     * in PER_CHIRP_LUT RAM. \n
     * The start address offset shall be multiple of 4 bytes. The max addressable space 8kB. \n
     * The size of this ARRAY shall be sufficient to hold all chirp data in a frame. \n
     * Total size of RAM = 4 bytes per chirp * num of chirps in a burst * num of bursts in frame.
     * \n Each chirp will have 1 byte data for each RX, max 4 RX is supported (Device supports only
     * 3RX, the last byte is reserved). \n
     * Valid range: 0, 4, 8 to 8188 \n
     */
    UINT16 h_PerChirpRamStartAdd;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_LIVE_RX_SATURATION_CFG;

/*!
 * @brief mmWaveLink @ref rl_monLiveRxSaturationCfg API response Data structure \n
 * The FECSS RX_SATURATION monitor result data structure for per chirp contains the monitor
 * saturation result. The result is updated in PER_CHIRP_LUT RAM for every chirp, the application
 * shall read the complete result array every frame before next frame starts. \n
 * Result HW RAM base Address: 0x21880000 + h_PerChirpRamStartAdd \n
 * Size: 4 bytes per chirp * num of chirps in a burst * num of bursts in frame. \n
 * Below table describes the monitor data for a single chirp: \n
 * @warning Application must wait for at least 100us after the frame end event before reading live monitor results
 */
typedef struct
{
    /*!
     * @brief  The chirp RX saturation data contains 1 byte data for each RX, max 4 RX is
     * supported (Device supports only 3RX, the last byte is reserved). \n
     * Bitfield info:
     * | BitField | Definition  |
     * |----------|-----------  |
     * | 0       | Sat count bit-0    |
     * | 1       | Sat count bit-1    |
     * | 2       | Sat count bit-2    |
     * | 3       | Sat count bit-3    |
     * | 4       | Sat count bit-4    |
     * | 5       | Sat count bit-5    |
     * | 6       | Sat count bit-6    |
     * | 7       | PA blanking in chirp flag    |
     * Byte Field info:
     * | Byte Field | Definition  |
     * |----------|-----------  |
     * | 0       | RX 0    |
     * | 1       | RX 1    |
     * | 2       | RX 2    |
     * | 3       | Reserved  |
     */
    UINT8 c_ChirpMonData[M_DFP_MAX_RX_CHANNELS];

} T_RL_API_MON_LIVE_RX_SATURATION_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monLiveGpadcCtmCfg API command Data structure \n
 * This API shall be used to configure the monitor number of instructions. The live monitors can
 * be enabled in sensor start API, this live monitor runs in background when functional frames or
 * test bursts are running. \n
 * The monitor GAPDC instruction parameters are stored in a GPADC_INSTRUCTION RAM having base
 * address 0x50080400 to store max 64 instructions (512 bytes). The application can directly write
 * to this RAM with all the GAPDC instruction data for 64 measurements. \n
 * The monitor result RL_MON_GPADC_CTM_RESULT is updated every chirp while framing in
 * GPADC_RESULT_LUT RAM ARRAY (Base address 0x21800400). The application shall read the result end
 * of each frame. \n
 * GPADC Instruction RAM Parameters: \n
 * FECSS GPADC CTM monitor instruction RAM configuration data structure, the GPADC config and
 * param values are configured in this RAM. The Maximum 64 GPADC measurement (instruction) can be
 * stored in this RAM. These GPADC signals will be measured every chirp, provided sufficient ramp
 * time is programmed in all chirps. \n
 * Each GAPDC instruction is 8 bytes and total 512 bytes to store 64 GPADC instructions. \n
 * Byte Description: \n
 *  | Byte Field      | Definition  |
 *  |-----------------|-----------  |
 *  | Bytes[7:0]      | GPADC_INSTUCTION_0   |
 *  | Bytes[15:8]     | GPADC_INSTUCTION_1   |
 *  | Bytes[503:9]    | GPADC_INSTUCTION_2 to GPADC_INSTUCTION_62   |
 *  | Bytes[511:504]  | GPADC_INSTUCTION_63   |
 * @ref T_RL_API_MON_LIVE_GPADC_CTM_INSTR GPADC Instruction Description as below \n
 * @ref T_RL_API_MON_LIVE_GPADC_CTM_CFG is the configuration API.
 *
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle
 * as this configuration can be retained in HW RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  FECSS GPADC channel config value, 32bit value per channel.  \n
     * Max 8 GPADC signals supported. \n
     * The config value is a unique value for a internal or external signal can be configured
     * to measure the corresponding signal in any GPADC channel.
     */
    UINT32 w_ConfigVal;

    /*!
     * @brief  FECSS GPADC channel param value, 32bit value per channel. \n
     * Max 8 GPADC signal measurements are supported. The param value is a unique
     * value includes, channel control , num of collect and skip samples for
     * internal or external signals to measure the corresponding signal in any GPADC channel.  \n
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [7:0]   | GPADC channel control value, 8bit value per channel.  \n Max 8 GPADC signals supported. \n The control value is a unique parameter value for a internal or external signal can be configured to measure the corresponding signal in any GPADC channel. |
     * | bits [15:8]  | GPADC number of measurement collect samples. \n 1LSB = 1 collect sample, i.e time period of Operating clock frequency / 16 \n Range: 0 to 255 \n The value 0 can be used to insert a dummy instructions in a chirp. |
     * | bits [22:16]   | GPADC number of skip samples before measurement, 7bit value. \n Bitfield definition \n Bits[3:0] : 1LSB = 1 skip sample \n Range: 0 to 15 \n Skip samples: 0 to 15 \n Bits[6:4] : 1LSB = 2^N skip samples \n N Range: 0 to 7  \n Skip samples: 1 to 128 \n 1 skip sample period = Time period of Operating clock frequency      |
     * | bits [23]   | Chirp Break Enable. This feature can be used in CTM mode of operation to continue the GPADC signal measurement in next chirp. \n If this bit set in GPADC parameter for a signal then the measurement for next signal will be performed in next new chirp.  |
     * | bits [31:24]   | Reserved |
     *
     *  The GPADC collect sample period definition:
     * | XTAL clock Source   | GPADC Operating Clock   | 1LSB collect sample Period |
     * |  -----------------  |  ---------------------- |  -----------------------   |
     * | 25MHz               |  12.5MHz                |  1.28us                    |
     * | 26MHz               |  13MHz                  |  1.231us                   |
     * | 38.4MHz             |  12.8MHz                |  1.25us                    |
     * | 40MHz               |  13.3MHz                |  1.203us
     * The GPADC skip sample period definition:
     * | XTAL clock Source   | GPADC Operating Clock   | 1LSB skip sample Period |
     * |  -----------------  |  ---------------------- |  -----------------------   |
     * | 25MHz               |  12.5MHz                |  80ns                      |
     * | 26MHz               |  13MHz                  |  76.92ns                   |
     * | 38.4MHz             |  12.8MHz                |  78.125ns                  |
     * | 40MHz               |  13.3MHz                |  75.19ns                   |
     */
    UINT32 w_ParamVal;

} T_RL_API_MON_LIVE_GPADC_CTM_INSTR;

typedef struct
{
    /*!
     * @brief  The GPADC instruction RAM start address offset. \n
     * This field is used to start the GPADC signal measurement stored in LUT with an offset.
     * This feature helps to select a set of GPADC signals per sensor start from preloaded
     * instructions from LUT.  \n
     * Valid Range: 0 to 63 \n
     */
    UINT8 c_GpadcInstrStartOffset;

    /*!
     * @brief  The number of GPADC instructions. \n
     * This field is used to program the number of GPADC measurement signal instructions per chirp
     * in HW. All these GPADC signal instructions can be measured in a single chirp. \n
     * Valid Range: 1 to 64 \n
     * Note: \n
     * 1. The application has to be cautious while programming this count
     * (NUM_OF_GPADC_INSTRUCTIONS), the ramp time of all the chirps in a frame shall be sufficient
     * to measure all the programmed GPADC instructions (signals). \n
     * 2. The application shall read the GPADC_RESULT data out from this GPADC_RESULT_LUT RAM in
     * inter frame idle time before next frame starts.
     */
    UINT8 c_NumOfGpadcInstr;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_LIVE_GPADC_CTM_CFG;

/*!
 * @brief mmWaveLink @ref rl_monLiveGpadcCtmCfg API response Data structure \n
 * The FECSS GPADC_CTM monitor result data structure contains the monitor result for all GPADC
 * instructions. The result is updated in GPADC_RESULT_LUT RAM every chirp for all instructions,
 * the application shall read the complete result array every frame before next frame starts. \n
 * The GPADC result for a instruction (signal) contains Min, Max and Sum of all chirps in a frame.
 * Result HW RAM base Address: 0x21800400 \n
 * Size: 8 bytes * num of GPADC instructions, max 512 bytes for max 64 instructions.
 * Byte Description of GPADC_RESULT_LUT : \n
 *  | Byte Field      | Definition  |
 *  |-----------------|-----------  |
 *  | Bytes[7:0]      | GPADC_INSTUCTION_0_RESULT   |
 *  | Bytes[15:8]     | GPADC_INSTUCTION_1_RESULT    |
 *  | Bytes[503:9]    | GPADC_INSTUCTION_2_RESULT  to GPADC_INSTUCTION_62_RESULT    |
 *  | Bytes[511:504]  | GPADC_INSTUCTION_63_RESULT    |
 * @ref T_RL_API_MON_LIVE_GAPDC_CTM_RESULT describes the monitor data for a single instruction:
 * @warning Application must wait for at least 100us after the frame end event before reading live monitor results
 */
typedef struct
{
    /*!
     * @brief  The minimum GPADC measurement value in a frame (min of all chirps) for a
     * instruction. Data is 10bit value.
     */
    UINT16 h_GpadcMin;

    /*!
     * @brief  The maximum GPADC measurement value in a frame (max of all chirps) for a
     * instruction. Data is 10bit value.
     */
    UINT16 h_GpadcMax;

    /*!
     * @brief  The sum of all GPADC measurement value in a frame (sum of all chirps) for a
     * instruction. Data is 20bit value. \n
     * The application shall use total num of chirps in a frame to compute the average value.
     */
    UINT32 w_GpadcSum;

} T_RL_API_MON_LIVE_GAPDC_CTM_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monPllCtrlVoltCfg API command Data structure \n
 * The APLL and Synthesizer control voltage monitor cfg API shall be used to configure the
 * monitor. The monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame
 * idle time immediately after the trigger.  \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_PLL_CTRL_VOLT_RESULT will be updated end of monitor trigger in
 * address 0x2120052C. Upon completion of execution of all enabled monitors, the FECSS RFS core
 * will send a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read
 * out all the results from the predefined address. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle
 * as this configuration is retained in FEC RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  APLL & Synth control voltage monitor enable mask. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Monitor Disabled    |
     * | 1      | Monitor Enabled    |
     * Bit Field Definition:
     * | BitField   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | APLL_CTRL_VOLT_MON_EN. APLL control voltage monitor enable   |
     * | Bit[1]  | SYNTH_CTRL_VOLT_MON_EN. SYNTH VCO control voltage monitor enable   |
     * | Bit[7:2] | RESERVED |
     * Note: At least one of the monitor shall be enabled if monitor is triggered.
     */
    UINT8 c_PllMonEnaMask;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_PLL_CTRL_VOLT_CFG;

/*!
 * @brief mmWaveLink @ref rl_monPllCtrlVoltCfg API response Data structure \n
 * The FECSS PLL_CTRL_VOLT monitor result data structure contains the monitor result. The result
 * is updated in IPC RAM end of monitor execution, the application shall read the result once the
 * Monitor execution done IRQ event is received from FECSS. \n
 * Result IPC RAM base Address: 0x2120052C \n
 * Size: 28 bytes
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief APLL control voltage value. Unsigned number. \n The measured value of PLL control
     * voltage level. \n
     * Only the fields corresponding to the enabled monitors are valid. \n
     * 1LSB = 1.8V / 1024 \n
     * Valid range: 0 to 1023
     */
    UINT16 h_ApllCtrlVolt;

    /*!
     * @brief SYNTH VCO Min frequency control voltage value. Unsigned number. \n
     * The measured values of SYNTH control voltage level at Min RF frequency. \n
     * | Device Type   | Min RF Frequency  |
     * |---------|-----------  |
     * | 60GHz Device | 57GHz    |
     * | 77GHz Device | 76GHz     |
     * Only the fields corresponding to the enabled monitors are valid.
     * 1LSB = 1.8V / 1024 \n
     * Valid range: 0 to 1023
     */
    UINT16 h_SynthMinCtrlVolt;

    /*!
     * @brief SYNTH VCO Max frequency control voltage value. Unsigned number. \n
     * The measured values of SYNTH control voltage level at Max RF frequency. \n
     * | Device Type   | Max RF Frequency  |
     * |---------|-----------  |
     * | 60GHz Device | 64GHz    |
     * | 77GHz Device | 81GHz     |
     * Only the fields corresponding to the enabled monitors are valid.
     * 1LSB = 1.8V / 1024 \n
     * Valid range: 0 to 1023
     */
    UINT16 h_SynthMaxCtrlVolt;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved5;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_PLL_CTRL_VOLT_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monTxNRxLbCfg API command Data structure \n
 * The RFS TX[1:0] and RX loopback monitor cfg API shall be used to configure the monitor. The
 * monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame idle time
 * immediately after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_TXn_RX_LB_RESULT will be updated end of monitor trigger in address
 * 0x21200548. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * This API shall be issued multiple times with proper TX_INDEX_SEL to configure each TX, this
 * device supports max 2 TX channels. The TX results are stored in consecutive memory. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  TX index Selection for TX-RX PA LB monitor configuration. \n
     * This field is the Tx chain number for which the configuration is performed.  \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | The Monitor configuration will be applicable for TX0   |
     * | 1      | The Monitor configuration will be applicable for TX1   |
     * | 2 to 255   | Reserved   |
     * Application shall call this API multiple times with different TX_INDEX_SEL option to
     * monitor all TX chain. \n
     * This selection shall not exceed channel selection done in TX_CHAN_CTRL_MASK in RF
     * ON/OFF API. .
     */
    UINT8 c_TxIndSel;

    /*!
     * @brief  TX_RX_LB and BPM Monitor control. \n
     * Monitors for both BPM 0deg and 180deg can be triggered in one shot. This parameter is used
     * to enable/ disable each of them. \n
     * All the monitors will be done and reports will be updated using PA loopback if it is
     * enabled. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Disabled  |
     * | 1      | Enabled   |
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | TXn_RX_LB_MON enable bit. \n TX0 or TX1 to RX LB monitor with BPM=0deg enable control. \n Enable/ Disable of Monitor using In-band IF signal (2.77 MHz) with Tx BPM set to 0  |
     * | Bit[1]  | TXn_BPM_MON enable bit. \n TX0 or TX1 to BPM monitor with BPM=180deg enable control. \n Enable/ Disable of Monitor using In-band IF signal (2.77 MHz) with Tx BPM set to 180deg  |
     * | Bit[2]  | Rx input power measurement enable bit \n Accuracy of the reported Rx input power degrades with higher backoff |
     * Note: At least one monitor shall be enabled in this configuration.
     */
    UINT8 c_MonEnaCtrl;

    /*!
     * @brief  Monitor RX gain and TX bias code selection. \n
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | RX_GAIN_CODE selection. \n Value 0: Use Default code used for functional chirps from calibrated CAL_RX_GAIN_SEL. \n Value 1: Use Override value c_MonRxGainCode.  |
     * | Bit[1]  | TX_BIAS_CODE selection. \n Value 0: Use Default code used for functional chirps from calibrated CAL_TX_BIAS_SEL. \n Value 1: Use Override value h_MonTxBiasCodes.  |
     * | Bit[2]  | Loopback gain selection \n Value 0: Use low PA loopback gain (-10 dB) \n Value 1: Use loopback gain (0 dB) \n Recommended value: 1 (0dB) |
     * | Bits[7:3]  | Reserved |
     */
    UINT8 c_MonRxTxCodesSel;

    /*!
     * @brief  Monitor RX channels gain code override setting. This is a common IFA and RF gain code value for all chirps used in this monitor.  \n
     * Bit Definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [4:0]  | IFA_GAIN_CODE. \n Override IFA gain code. Valid Rage: 0 to 14. \n IFA_GAIN (dB) = IFA_GAIN_CODE * 2 - 10dB |
     * | bits [7:5]  | RF_GAIN_CODE. \n Override RF gain code. \n Valid Rage: 0 to 3. \n  Value 0 : MAX_GAIN (~39dB in 77GHz Device and ~34dB in 60GHz device. Value 1 : MAX_GAIN - 4dB \n Value 2 : MAX_GAIN - 7dB \n Value 3 : MAX_GAIN - 9dB \n Note: In xWRL6432 device only value 0 (MAX_GAIN) is valid. |
     *
     */
    UINT8 c_MonRxGainCode;

    /*!
     * @brief  Monitor TX channel Bias code override setting. \n
     * This is a common TX bias code value for all chirps used in this monitor. \n
     * TX_BIAS_CODE (2 bytes) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | Byte [0]  | TX_BIAS_CODE_1. Override TX Bias code 1 for given CAL_TX_BACK_OFF_SEL.  |
     * | Byte [1]  | TX_BIAS_CODE_2. Override TX Bias code 2 for given CAL_TX_BACK_OFF_SEL.  |
     */
    UINT16 h_MonTxBiasCodes;

    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x (APLL_FREQ) / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x (APLL_FREQ) / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  Monitor RF Frequency Slope. This is a common RF frequency slope value for all
     * chirps used in this monitor. \n
     * The typical ramp time used in FW for monitor chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range \n
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_RfFreqSlope;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_TXN_RX_LB_CFG;

/*!
 * @brief mmWaveLink @ref rl_monTxNRxLbCfg API response Data structure \n
 * The FECSS TXn_RX_LB monitor result data structure the monitor result for each TX. The result is
 * updated in IPC RAM end of monitor execution, the application shall read the result once the
 * Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x21200548 \n
 * Size: 56 bytes per TX, the TX0 and TX1 results are stored in contiguous memory, TX3 and TX4 are
 * reserved for future devices. The application can memory map RL_MON_TX_RX_LB_RESULT[4]
 * structure array to IPC address 0x21200548.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief Rx input Power
     * 1LSB = 0.1 dBm
     * @note: Accuracy of the Rx Input power will degrade with higher Tx back-off. It is recommended to measure
     * this power with least backoff possible (preferably 0dB).
     */
    SINT16 xh_RxInputPower;

    /*!
     * @brief TX_RX_LB_POWER value for all RX channels for TXn channel. \n
     * The measured TX PA loopback tone power at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB = 0.1dBm, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if
     * TXn_RX_LB_MON = 1 in configuration API.
     */
    SINT16 xh_TxNRxLbPower[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief TX_RX_LB_PHASE value for all RX channels for TXn channel. \n
     * The measured TX PA loopback tone phase at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB =  360 deg / 2^16 \n
     * The report is valid for enabled RX channels. These values are valid only if TXn_RX_LB_MON =
     * 1 in configuration API.
     */
    UINT16 h_TxNRxLbPhase[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief TX_RX_LB_BPM_POWER value for all RX channels for TXn channel with BPM ON
     * measurement.  \n
     * The measured TX PA loopback tone power at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB = 0.1dBm, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if
     * TXn_BPM_MON = 1 in configuration API.
     */
    SINT16 xh_TxNRxLbBpmPower[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief TX_RX_LB_BPM_PHASE value for all RX channels for TXn channel with BPM ON
     * measurement. \n
     * The measured TX PA loopback tone phase at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB =  360 deg / 2^16 \n
     * The report is valid for enabled RX channels. These values are valid only if TXn_BPM_MON =
     * 1 in configuration API.
     */
    UINT16 h_TxNRxLbBpmPhase[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief TRX_NOISE value for all RX channels for TXn channel. \n
     * The measured wideband RX noise at the RX[2:0] ADC input for selected monitor
     * configuration.  \n
     * 1 LSB = 1dBm, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if TXn_RX_LB_MON =
     * 1 in configuration API.
     */
    SINT8 xc_TxNRxLbNoise[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief TRX_BPM_NOISE value for all RX channels for TXn channel with BPM ON measurement. \n
     * The measured wideband RX noise at the RX[2:0] ADC input for selected monitor
     * configuration.  \n
     * 1 LSB = 1dBm, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if TXn_BPM_MON =
     * 1 in configuration API.
     */
    SINT8 xc_TxNRxLbBpmNoise[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_TXN_RX_LB_RESULT;


/*!
 * @brief mmWaveLink @ref rl_monTxNPowerCfg API command Data structure \n
 * The RFS TX[1:0] power monitor cfg API shall be used to configure the monitor. The monitor can
 * be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame idle time immediately
 * after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_TXn_PWR_RESULT will be updated end of monitor trigger in address
 * 0x21200628. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * This API shall be issued multiple times with proper TX_INDEX_SEL to configure each TX, this
 * device supports max 2 TX channels. The TX results are stored in consecutive memory. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  TX index Selection for TX-RX PA LB monitor configuration. \n
     * This field is the Tx chain number for which the configuration is performed.  \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | The Monitor configuration will be applicable for TX0   |
     * | 1      | The Monitor configuration will be applicable for TX1   |
     * | 2 to 255   | Reserved   |
     * Application shall call this API multiple times with different TX_INDEX_SEL option to
     * monitor all TX chain. \n
     * This selection shall not exceed channel selection done in TX_CHAN_CTRL_MASK in RF
     * ON/OFF API. .
     */
    UINT8 c_TxIndSel;

    /*!
     * @brief  Monitor TX bias code selection. \n
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | Reserved  |
     * | Bit[1]  | TX_BIAS_CODE selection. \n Value 0: Use Default code used for functional chirps from calibrated CAL_TX_BIAS_SEL. \n Value 1: Use Override value h_MonTxBiasCodes.  |
     * | Bits[7:2]  | Reserved |
     */
    UINT8 c_MonTxCodesSel;

    /*!
     * @brief  Monitor TX channel Bias code override setting. \n
     * This is a common TX bias code value for all chirps used in this monitor. \n
     * TX_BIAS_CODE (2 bytes) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | Byte [0]  | TX_BIAS_CODE_1. Override TX Bias code 1 for given CAL_TX_BACK_OFF_SEL.  |
     * | Byte [1]  | TX_BIAS_CODE_2. Override TX Bias code 2 for given CAL_TX_BACK_OFF_SEL.  |
     */
    UINT16 h_MonTxBiasCodes;

    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  Monitor RF Frequency Slope. This is a common RF frequency slope value for all
     * chirps used in this monitor. \n
     * The typical ramp time used in FW for monitor chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_RfFreqSlope;

    /*!
     * @brief  TX Back off value mapped for TX_BIAS_CODE override value. The FECSS RFS Fw needs
     * this mapping for PD measurement. \n
     * 1LSB: 0.5dB \n
     * Valid Range: Even values (1dB step) from 0 (0dB) to 52 (26dB)
     */
    UINT8 c_TxBackoffMap;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_TXN_POWER_CFG;

/*!
 * @brief mmWaveLink @ref rl_monTxNPowerCfg API response Data structure \n
 * The FECSS TXn_PWR monitor result data structure the monitor result for each TX. The result is
 * updated in IPC RAM end of monitor execution, the application shall read the result once the
 * Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x21200628 \n
 * Size: 16 bytes per TX, the TX0 and TX1 results are stored in contiguous memory, TX3 and TX4 are
 * reserved for future devices.  The application can memory map RL_MON_TX_PWR_RESULT[4]
 * structure array to IPC address 0x21200628.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief TX_PWR_VAL amplitude value for selected TXn channel. \n
     * The measured TX power for selected monitor configuration.  \n
     * 1 LSB = 0.1dBm, signed number.
     */
    SINT16 xh_TxNPwrVal;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_TXN_POWER_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monTxNBbCfg API command Data structure \n
 * The RFS TX[1:0] Ball break monitor cfg API shall be used to configure the monitor.
 * The monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame
 * idle time immediately after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_TXn_BB_RESULT will be updated end of monitor trigger in address
 * 0x21200668. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * This API shall be issued multiple times with proper TX_INDEX_SEL to configure each TX, this
 * device supports max 2 TX channels. The TX results are stored in consecutive memory. \n
 * @note:
 * - The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 * - Ball-break monitor's accuracy degrades with higher backoff.
 *
 * @b Recommendation @n
 * -# Use 0dB backoff (using the backoff override) to get the most accurate results
 */
typedef struct
{
    /*!
     * @brief  TX index Selection for TX-RX PA LB monitor configuration. \n
     * This field is the Tx chain number for which the configuration is performed.  \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | The Monitor configuration will be applicable for TX0   |
     * | 1      | The Monitor configuration will be applicable for TX1   |
     * | 2 to 255   | Reserved   |
     * Application shall call this API multiple times with different TX_INDEX_SEL option to
     * monitor all TX chain. \n
     * This selection shall not exceed channel selection done in TX_CHAN_CTRL_MASK in RF
     * ON/OFF API. .
     */
    UINT8 c_TxIndSel;

    /*!
     * @brief  Monitor TX bias code selection. \n
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | Reserved  |
     * | Bit[1]  | TX_BIAS_CODE selection. \n Value 0: Use Default code used for functional chirps from calibrated CAL_TX_BIAS_SEL. \n Value 1: Use Override value h_MonTxBiasCodes.  |
     * | Bits[7:2]  | Reserved |
     */
    UINT8 c_MonTxCodesSel;

    /*!
     * @brief  Monitor TX channel Bias code override setting. \n
     * This is a common TX bias code value for all chirps used in this monitor. \n
     * TX_BIAS_CODE (2 bytes) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | Byte [0]  | TX_BIAS_CODE_1. Override TX Bias code 1 for given CAL_TX_BACK_OFF_SEL.  |
     * | Byte [1]  | TX_BIAS_CODE_2. Override TX Bias code 2 for given CAL_TX_BACK_OFF_SEL.  |
     */
    UINT16 h_MonTxBiasCodes;

    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  Monitor RF Frequency Slope. This is a common RF frequency slope value for all
     * chirps used in this monitor. \n
     * The typical ramp time used in FW for monitor chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_RfFreqSlope;

    /*!
     * @brief  TX Back off value mapped for TX_BIAS_CODE override value. The FECSS RFS Fw needs
     * this mapping for PD measurement. \n
     * 1LSB: 0.5dB \n
     * Valid Range: Even values (1dB step) from 0 (0dB) to 52 (26dB)
     */
    UINT8 c_TxBackoffMap;

    /*!
     * @brief  Reserved.
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_TXN_BB_CFG;

/*!
 * @brief mmWaveLink @ref rl_monTxNBbCfg API response Data structure \n
 * The FECSS TXn_BB monitor result data structure the monitor result for each TX. The result is
 * updated in IPC RAM end of monitor execution, the application shall read the result once the
 * Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x21200668 \n
 * Size: 16 bytes per TX, the TX0 and TX1 results are stored in contiguous memory, TX3 and TX4 are
 * reserved for future devices.  The application can memory map RL_MON_TX_BB_RESULT[4]
 * structure array to IPC address 0x21200668. \n
 *
 * @note: Accuracy of the ball-break monitor degrades with back-off. It it recommended to run this
 * monitor with least backoff possible (preferably 0dB) for the most reliable results.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief TX_INC_PWR_VAL amplitude value for selected TXn channel. \n
     * The measured TX power at incident PD for selected monitor configuration.  \n
     * 1 LSB = 0.1dBm, signed number.
     */
    SINT16 xh_TxNIncidPwrVal;

    /*!
     * @brief TX_REFL_PWR_VAL amplitude value for selected TXn channel. \n
     * The measured TX power at reflected PD for selected monitor configuration.  \n
     * 1 LSB = 0.1dBm, signed number.
     */
    SINT16 xh_TxNReflPwrVal;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_TXN_BB_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monTxNDcSigCfg API command Data structure \n
 * The RFS TX[1:0] DC Signal monitor cfg API shall be used to configure the monitor.
 * The monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame
 * idle time immediately after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_TXn_DC_SIG_RESULT will be updated end of monitor trigger in address
 * 0x212006A8. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * This API shall be issued multiple times with proper TX_INDEX_SEL to configure each TX, this
 * device supports max 2 TX channels. The TX results are stored in consecutive memory. \n
 * The DC signals are measured and results are compared against internal threshold and sent part
 * of the report. The TXn channel is ON during this measurement and RX channels are disabled. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  TX index Selection for TX-RX PA LB monitor configuration. \n
     * This field is the Tx chain number for which the configuration is performed.  \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | The Monitor configuration will be applicable for TX0   |
     * | 1      | The Monitor configuration will be applicable for TX1   |
     * | 2 to 255   | Reserved   |
     * Application shall call this API multiple times with different TX_INDEX_SEL option to
     * monitor all TX chain. \n
     * This selection shall not exceed channel selection done in TX_CHAN_CTRL_MASK in RF
     * ON/OFF API. .
     */
    UINT8 c_TxIndSel;

    /*!
     * @brief  Monitor TX bias code selection. \n
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | Reserved  |
     * | Bit[1]  | TX_BIAS_CODE selection. \n Value 0: Use Default code used for functional chirps from calibrated CAL_TX_BIAS_SEL. \n Value 1: Use Override value h_MonTxBiasCodes.  |
     * | Bits[7:2]  | Reserved |
     */
    UINT8 c_MonTxCodesSel;

    /*!
     * @brief  Monitor TX channel Bias code override setting. \n
     * This is a common TX bias code value for all chirps used in this monitor. \n
     * TX_BIAS_CODE (2 bytes) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | Byte [0]  | TX_BIAS_CODE_1. Override TX Bias code 1 for given CAL_TX_BACK_OFF_SEL.  |
     * | Byte [1]  | TX_BIAS_CODE_2. Override TX Bias code 2 for given CAL_TX_BACK_OFF_SEL.  |
     */
    UINT16 h_MonTxBiasCodes;

    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  Monitor RF Frequency Slope. This is a common RF frequency slope value for all
     * chirps used in this monitor. \n
     * The typical ramp time used in FW for monitor chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_RfFreqSlope;

    /*!
     * @brief  Reserved.
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_TXN_DCSIG_CFG;

/*!
 * @brief mmWaveLink @ref rl_monTxNDcSigCfg API response Data structure \n
 * The FECSS TXn_DC_SIG monitor result data structure for monitor result for each TX. The result is
 * updated in IPC RAM end of monitor execution, the application shall read the result once the
 * Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x212006A8 \n
 * Size: 16 bytes per TX, the TX0 and TX1 results are stored in contiguous memory, TX3 and TX4 are
 * reserved for future devices.  The application can memory map RL_MON_TX_BB_RESULT[4]
 * structure array to IPC address 0x212006A8.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  TX_DC_MON status for selected TXn channel. \n
     * The measured TX supply GPADC DC signals for selected monitor configuration is compared with
     * internal thresholds and result status (Pass/fail) will be sent out for each signal type.
     * | Bit Field   | Definition  |
     * |-------------|-----------  |
     * | Bit[0]      | TXn_VDDA_SUPPLY signal Status      |
     * | Bit[1]      | TXn_1V_CLUSTER_SUPPLY signal Status     |
     * | Bit[15:2]  | Reserved     |
     */
    UINT16 h_TxDcMonStatus;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_TXN_DCSIG_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monRxHpfDcSigCfg API command Data structure \n
 * The RFS RX DC Signal monitor cfg API shall be used to configure the monitor.
 * The monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame
 * idle time immediately after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_RX_HPF_DC_SIG_RESULT will be updated end of monitor trigger in address
 * 0x212006E8. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * The DC signals are measured and results are compared against internal threshold and sent part
 * of the report. The TX channels are OFF during this measurement and RX channels are enabled as
 * per global configuration. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  RX_DC_SIG and HPF Monitor control. \n
     * Monitors for up to two different test conditions can be triggered in one shot.  This
     * parameter is used to enable/ disable each of them. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Disabled  |
     * | 1      | Enabled   |
     * | Bit Field   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | RX_DC_MON enable bit. \n A monitor chirp is triggered to measure the DC signals using GPADC.   |
     * | Bit[1]  | HPF_CUTOFF_MON enable bit. \n Two monitor chirps with IFA loopback are triggered to measure the powers at in-band frequency and HPF cutoff frequency. \n The first chirp for the in-band power measurement uses an IF signal frequency of 2.77MHz. The second chirp for the HPF cutoff power measurement uses an IF signal frequency matching the user programmed HPF corner frequency. |
     */
    UINT8 c_MonEnaCtrl;

    /*!
     * @brief  The HPF_CUTOFF_MON monitor Chirp HPF corner frequency.  \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | 175kHz HPF corner frequency  |
     * | 1      | 350kHz HPF corner frequency   |
     * | 2      | 700kHz HPF corner frequency   |
     * | 3      | 1400kHz HPF corner frequency   |
     * Valid Range: 0 to 3
     */
    UINT8 c_RxHpfSel;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_MON_RX_HPF_DCSIG_CFG;

/*!
 * @brief mmWaveLink @ref rl_monRxHpfDcSigCfg API response Data structure \n
 * The FECSS RX_HPF_DC_SIG monitor result data structure for monitor result.
 * The result is updated in IPC RAM end of monitor execution, the application shall read the
 * result once the Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x212006E8 \n
 * Size: 36 bytes.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  RX_DC_MON status. \n
     * The measured all RX channels supply and bias GPADC DC signals for selected monitor
     * configuration is compared with internal thresholds and result (pass/fail) will be sent out
     * for each signal type. \n
     *
     * | Bit Field   | Definition  |
     * |-------------|-----------  |
     * | Bit [0]     | RX0_ADC_I_DIG_LDO_VOUT_SENSE status |
     * | Bit [1]     | RX0_ADC_I_DIG_LDO_VIN_SENSE status |
     * | Bit [2]     | RX0_IFA_I_LDO_VOUT_SENSE status |
     * | Bit [3]     | RX0_IFA_I_LDO_VIN_SENSE status |
     * | Bit [4]     | RX0_VDD_LNA status |
     * | Bit [5]     | LODIST_RX01_VDDA_1P0V_SUPPLY status |
     * | Bit [6]     | RX1_ADC_I_DIG_LDO_VOUT_SENSE status |
     * | Bit [7]     | RX1_ADC_I_DIG_LDO_VIN_SENSE status |
     * | Bit [8]     | RX1_IFA_I_LDO_VOUT_SENSE status |
     * | Bit [9]     | RX1_IFA_I_LDO_VIN_SENSE status |
     * | Bit [10]    | RX1_VDD_LNA status |
     * | Bit [11]    | RX2_ADC_I_DIG_LDO_VOUT_SENSE status |
     * | Bit [12]    | RX2_ADC_I_DIG_LDO_VIN_SENSE status |
     * | Bit [13]    | RX2_IFA_I_LDO_VOUT_SENSE status |
     * | Bit [14]    | RX2_IFA_I_LDO_VIN_SENSE status |
     * | Bit [15]    | RX2_VDD_LNA status |
     * | Bit [16]    | LODIST_RX23_VDDA_1P0V_SUPPLY status |
     * | Bit [31:17] | Reserved status     |
     *
     * The report is valid for enabled RX channels. These values are valid only if
     * RX_DC_MON = 1 in configuration API.
     */
    UINT32 w_RxDcMonStatus;

    /*!
     * @brief  HPF_INBAND power is IFA LB in-band measurement value in dB for all RX channels. \n
     * The measured In-band power value at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB = 0.1dB, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if HPF_CUTOFF_MON
     * = 1 in configuration API.
     */
    SINT16 xh_RxHpfInBandPwr[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief  HPF_CUTOFF power is IFA LB measurement value in dB for all RX channels at selected
     * CHIRP_RX_HPF_SEL cutoff value. \n
     * The measured HPF cutoff value at the RX[2:0] ADC input for selected monitor
     * configuration. \n
     * 1 LSB = 0.1dB, signed number \n
     * The report is valid for enabled RX channels. These values are valid only if HPF_CUTOFF_MON
     * = 1 in configuration API.
     */
    SINT16 xh_RxHpfCutOffPwr[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_RX_HPF_DCSIG_RESULT;

/*!
 * @brief mmWaveLink @ref rl_monPmClkDcSigCfg API command Data structure \n
 * The RFS PM LO CLK DC Signal monitor cfg API shall be used to configure the monitor.
 * The monitor can be triggered in RL_MONITOR_ENABLE_TRIG API, this monitor runs in frame
 * idle time immediately after the trigger. \n
 * The application shall ensure that the monitor trigger is happening at the frame idle time and
 * sufficient time is allocated to run all enabled monitors. \n
 * The monitor result RL_MON_PMCLK_DC_SIG_RESULT will be updated end of monitor trigger in address
 * 0x2120070C. Upon completion of execution of all enabled monitors, the FECSS RFS core will send
 * a IRQ event (FEC_INTR2 - IRQ[4]) to APPSS (application core) as indication to read out all the
 * results from the predefined address. \n
 * The DC signals are measured and results are compared against internal threshold and sent part
 * of the report. The TX channels are OFF during this measurement and RX channels are enabled as
 * per global configuration. \n
 * Note: The application can avoid configuring the Monitor repeatedly in every deep sleep cycle as
 * this configuration is retained in FECSS RAM during deep sleep.
 */
typedef struct
{
    /*!
     * @brief  Monitor RF start Frequency. This is a common RF start frequency value for all
     * chirps used in this monitor. \n
     * Supported only Low Resolution Mode (16 bits): \n
     * Recommended configuration: Mid frequency of the operating band.
     *
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz with 40MHz XTAL) to 0x0000D555 (64GHz with 40MHz XTAL) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz with 40MHz XTAL) to 0x0000CA80(81GHz with 40MHz XTAL) |
     */
    UINT16 h_RfFreqStart;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_MON_PMCLK_DCSIG_CFG;

/*!
 * @brief mmWaveLink @ref rl_monPmClkDcSigCfg API response Data structure \n
 * The FECSS RX_PMCLK_DC_SIG monitor result data structure for monitor result.
 * The result is updated in IPC RAM end of monitor execution, the application shall read the
 * result once the Monitor execution done IRQ event received from FECSS.  \n
 * Result IPC RAM base Address: 0x2120070C \n
 * Size: 24 bytes.
 */
typedef struct
{
    /*!
     * @brief  Monitor Error code. 2's complement signed number. \n
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Success    |
     * | 2's complement negative number      | Failure     |
     *
     */
    SINT16 xh_ErrorCode;

    /*!
     * @brief  Reserved.
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  PM_CLK_DC_MON status. \n
     * The measured all PM, CLK supply and bias GPADC DC signals for selected monitor
     * configuration is compared with internal thresholds and result (pass/fail) will be sent out
     * for each signal type. \n
     *
     * | Bit Field   | Definition  |
     * |-------------|-----------  |
     * | Bit [0]     | PM_TOP_RF_LDO_1P2V_1P0V_VIN status |
     * | Bit [1]     | PM_TOP_RF_LDO_1P2V_1P0V_VOUT status |
     * | Bit [2]     | PM_TOP_VBG1P22V status |
     * | Bit [3]     | CLK_TOP_VIN_1P8VCO_SENSE_SCALED_0P5 status |
     * | Bit [4]     | CLK_TOP_VIN_1P8CLK_SENSE_SCALED_0P5 status |
     * | Bit [5]     | CLK_TOP_VDDA_LDO_CLKTOP_IOBUF_1P4V_APLL_SCALED_0P6 status |
     * | Bit [6]     | CLK_TOP_VDDA_LDO_APLL_VCO_1P4V_SCALED_0P6 status |
     * | Bit [7]     | CLK_TOP_VDDA_LDO_APLL_CP_1P0_SCALED_0P6 status |
     * | Bit [8]     | CLK_TOP_VDDD_LDO_SDM_1P4V_SCALED_0P6 status |
     * | Bit [9]     | CLK_TOP_VDDA_LDO_SYNTH_VCO_SCALED_0P6 status |
     * | Bit [10]    | CLK_TOP_VDDA_LDO_SYNTH_DIV_SCALED_0P6 status |
     * | Bit [11]    | CLK_TOP_VDDA_LDO_SLICER_1P4V_SCALED_0P6 status |
     * | Bit [12]    | LODIST_COM_VDDA_1P0V_SUPPLY status |
     * | Bit [13]    | LODIST_COM_LO_MULT_LDO_VOUT_SENSE_LOWV status |
     * | Bit [14]    | PM_TOP_VREF0P45 status |
     * | Bit [15]    | PM_TOP_VFB_0P6V status |
     * | Bit [16]    | PM_TOP_VREF_0P9V status |
     * | Bit [17]    | PM_TOP_SCALED_VIOIN status |
     * | Bit [18]    | CLK_TOP_SYNTH_DIVIDER_COMMON_MODE status |
     * | Bit [19]    | PM_TOP_DBC_REFSYS status |
     * | Bit [31:20] | Reserved status     |
     */
    UINT32 w_PmClkDcMonStatus;

    /*!
     * @brief  The REF1_0P45V Reference voltage GPADC measurement value. \n
     * 1LSB = 1.8V/256
     */
    UINT8 c_VRef1Val;

    /*!
     * @brief  The REF2_0P6V Reference voltage GPADC measurement value. \n
     * 1LSB = 1.8V/256
     */
    UINT8 c_VRef2Val;

    /*!
     * @brief  The REF3_0P9V Reference voltage GPADC measurement value. \n
     * 1LSB = 1.8V/256
     */
    UINT8 c_VRef3Val;

    /*!
     * @brief  The REF4_VION_IN Reference voltage GPADC measurement value. \n
     * 1LSB = 1.8V/256
     */
    UINT8 c_VRef4Val;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Monitor report Frame Count. \n
     * The 32 bit free running SW frame counter value of RFS. The application has to
     * take care of rollover. \n
     * 1LSB = Frame period \n
     * Valid range: 0 to 16777215
     */
    UINT32 w_FrameCount;

} T_RL_API_MON_PMCLK_DCSIG_RESULT;

/*!*****************************************************************************
 * @brief FECSS RFS IPC Monitor Result Data structure \n
 * The consolidated IPC Monitor Result data structures and memory map for 768 Bytes.
 * Start address: 0x21200500
 * Size: 704 Bytes
 */
typedef struct
{
    /*!
     * @brief  RFS Monitor trigger collective response, 20 Bytes, Address: 0x21200500
     */
    T_RL_API_MON_TRIG_RESPONSE z_MonTrigReponse;

    /*!
     * @brief  RFS Monitor live Synth freq result, 24 Bytes, Address: 0x21200514
     */
    T_RL_API_MON_LIVE_SYNTH_FREQ_RESULT z_MonLiveSynthFreqResult;

    /*!
     * @brief  RFS Monitor PLL control voltage result, 28 Bytes, Address: 0x2120052C
     */
    T_RL_API_MON_PLL_CTRL_VOLT_RESULT z_MonPllCtrlVoltResult;

    /*!
     * @brief  RFS Monitor TXn RX LB result, 56*4 Bytes, Address: 0x21200548
     * Supported for Max 4 TX channels in IPC.
     */
    T_RL_API_MON_TXN_RX_LB_RESULT z_MonTxNRxLbResult[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn Power result, 16*4 Bytes, Address: 0x21200628
     * Supported for Max 4 TX channels in IPC.
     */
    T_RL_API_MON_TXN_POWER_RESULT z_MonTxNPwrResult[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn BB result, 16*4 Bytes, Address: 0x21200668
     * Supported for Max 4 TX channels in IPC.
     */
    T_RL_API_MON_TXN_BB_RESULT z_MonTxNBbResult[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn DC Signal result, 16*4 Bytes, Address: 0x212006A8
     * Supported for Max 4 TX channels in IPC.
     */
    T_RL_API_MON_TXN_DCSIG_RESULT z_MonTxNDcSigResult[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor RX HPF and DC Signal result, 36 Bytes, Address: 0x212006E8
     */
    T_RL_API_MON_RX_HPF_DCSIG_RESULT z_MonRxHpfDcSigResult;

    /*!
     * @brief  RFS Monitor PM LO CLK DC Signal result, 24 Bytes, Address: 0x2120070C
     */
    T_RL_API_MON_PMCLK_DCSIG_RESULT z_MonPmClkDcSigResult;

    /*!
     * @brief  Reserved for monitor reports, 456 Bytes, address offset: 0x724
     */
    UINT32 w_Reserved1[39];

} T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT;

/*!
 * @brief FECSS RFS Live GPADC CTM Monitor Instruction RAM Data structure \n
 * Max 64 instructions can be stored in RAM.
 * Start address: 0x50080400
 * Size: 512 Bytes
 */
typedef struct
{
    /*!
     * @brief  RFS Monitor live GPADC CTM monitor instruction, max 64 instructions
     */
    T_RL_API_MON_LIVE_GPADC_CTM_INSTR \
        z_MonLiveGpadcCtmInstr[M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MAX];

} T_RL_FE_LIVE_GPADC_CTM_MON_INSTR_DATA_STRUCT;

/*!
 * @brief FECSS RFS Live GPADC CTM Monitor Result RAM Data structure \n
 * Max 64 instructions result can be stored in RAM.
 * Start address: 0x21800400
 * Size: 512 Bytes
 */
typedef struct
{
    /*!
     * @brief  RFS Monitor live GPADC CTM monitor result, max 64 instructions
     */
    T_RL_API_MON_LIVE_GAPDC_CTM_RESULT \
        z_MonLiveGpadcCtmResult[M_RL_MON_LIVE_GPADC_CTM_NUM_INSTR_MAX];

} T_RL_FE_LIVE_GPADC_CTM_MON_RESULT_DATA_STRUCT;


/*!*****************************************************************************
 * @brief FECSS RFS IPC Monitor Result 768 Bytes RAM start address- FEC cluster1 memory (retained)
 * @ref T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT for consolidated Monitor results
 */
#define M_RL_FE_MON_IPC_RESULT_RAM_START_ADDRESS      \
            ((T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT*)0x21200500U)

/*!
 * @brief FECSS RFS Live GPADC CTM Monitor GPADC_INSTRUCTION RAM start address -
 * FEC cluster2 memory (can be retained by application)
 * @ref T_RL_FE_LIVE_GPADC_CTM_MON_INSTR_DATA_STRUCT for consolidated Monitor instructions
 */
#define M_RL_FE_MON_GPADC_INSTR_RAM_START_ADDRESS      \
            ((T_RL_FE_LIVE_GPADC_CTM_MON_INSTR_DATA_STRUCT*)0x50080400U)

/*!
 * @brief FECSS RFS Live GPADC CTM Monitor GPADC_RESULT_LUT RAM start address -
 * FEC cluster2 memory (can be retained by application)
 * @ref T_RL_FE_LIVE_GPADC_CTM_MON_RESULT_DATA_STRUCT for consolidated Monitor results
 */
#define M_RL_FE_MON_GPADC_RESULT_RAM_START_ADDRESS      \
            ((T_RL_FE_LIVE_GPADC_CTM_MON_RESULT_DATA_STRUCT*)0x21800400)

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
 * @name mmWaveLink Library Monitor module API function calls
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
 * END OF dfp_monitor.h
 */


