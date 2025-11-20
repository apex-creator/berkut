/*!*****************************************************************************
 * @file rfs_command.h
 *
 * @brief FECSSLib RFS command API header file.
 *
 * @b Description @n
 * This FECSS library RFS command header file defines RFS command API data structure,
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
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     15Mar2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_RFS_CMD_H
#define FE_RFS_CMD_H

/*!*****************************************************************************
 \brief FECSSLib Library RF Scripter Module Driver API details.

 \section fecsslib_rfs_sec FECSSLib RFS Module
FECSSLib RFS Module is responsible for providing driver APIs for FECSS RFS control, IPC and command-response handling. Below block diagram shows the individual functional blocks of the Device Module.

@image html fecsslib_rfs.png "FECSSLib RFS Module"

 \subsection fecsslib_rfs_cmd_subsec FECSSLib RFS command API functions and Data Structures

The following FECSSLib RFS command APIs provides RFS control for FECSSLib/mmWaveLink APIs, refer function definition for details on data structure.
  - \ref rfs_rfsOpen                 - ROM/PATCH function
  - \ref rfs_rfsClose                - ROM/PATCH function
  - \ref rfs_cmdFwVerGet             - ROM/PATCH function
  - \ref rfs_cmdCpuStsGet            - ROM/PATCH function
  - \ref rfs_cmdFactCalDataSet       - ROM/PATCH function
  - \ref rfs_cmdFactCalDataGet       - ROM/PATCH function
  - \ref rfs_cmdRfPwrOnOff           - ROM/PATCH function
  - \ref rfs_cmdRfFactCal            - ROM/PATCH function
  - \ref rfs_cmdApllClkCtrl          - ROM/PATCH function
  - \ref rfs_cmdRfCalibrate          - ROM/PATCH function
  - \ref rfs_cmdRfClockBwCfg         - ROM/PATCH function
  - \ref rfs_cmdGpadcCfg             - ROM/PATCH function
  - \ref rfs_cmdGpadcTrig            - ROM/PATCH function
  - \ref rfs_cmdTempCfg              - ROM/PATCH function
  - \ref rfs_cmdTempTrig             - ROM/PATCH function
  - \ref rfs_cmdSensLbCfg            - ROM/PATCH function
  - \ref rfs_cmdSensLbEna            - ROM/PATCH function
  - \ref rfs_cmdFecssRdifCtrl        - ROM/PATCH function
  - \ref rfs_cmdRfStatusGet          - ROM/PATCH function

  - \ref rfs_cmdChirpProfCfg         - ROM/PATCH function
  - \ref rfs_cmdChirpProfCfgGet      - ROM/PATCH function
  - \ref rfs_cmdSensStartCfg         - ROM/PATCH function
  - \ref rfs_cmdCwSensorStart        - ROM/PATCH function
  - \ref rfs_cmdCwSensorStop         - ROM/PATCH function
  - \ref rfs_cmdSensDynPwrSaveDis    - ROM/PATCH function
  - \ref rfs_cmdSensDynPwrSaveStsGet - ROM/PATCH function

  - \ref rfs_cmdMonEnableTrig        - ROM/PATCH function
  - \ref rfs_cmdLiveSynthFreqCfg     - ROM/PATCH function
  - \ref rfs_cmdLiveRxSaturCfg       - ROM/PATCH function
  - \ref rfs_cmdLiveGpadcCtmCfg      - ROM/PATCH function
.

Link to Parent Directory: \ref FECSSLIB_RFS_API

   @addtogroup FECSSLIB_RFS_API FECSSLib RFS Control driver API functions
    @{
 @brief Below sections provides information about FECSSLib Library RFS module driver API functions and data structure and MACROs
********************************************************************************
*/

/*!
 * @defgroup FECSSLIB_RFS_DRV_MODULE RFS control Driver API functions
 * @brief FECSSLib RFS control driver functions and Data Structures
 * @ingroup FECSSLIB_RFS_API
 */

/*!
 * @defgroup FECSSLIB_RFS_CMD_MODULE RFS Command API functions
 * @brief FECSSLib RFS command API functions and Data Structures
 * @ingroup FECSSLIB_RFS_API
 */

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
 * @name FECSS Library RFS command module MACROs
 * @{
 */
/*!
 * @brief FECSS RFS Error response command ID
 */
#define M_FE_RFS_CMD_ID_ERROR_RESP          ((UINT16)0x0000U)

/*!
 * @brief FECSS RFS RF power ON/OFF command ID
 */
#define M_FE_RFS_CMD_ID_RF_PWR_ONOFF        ((UINT16)0x0001U)

/*!
 * @brief FECSS RFS Factory calibration command ID
 */
#define M_FE_RFS_CMD_ID_FACT_CAL            ((UINT16)0x0002U)

/*!
 * @brief FECSS RFS APLL Clk control command ID
 */
#define M_FE_RFS_CMD_ID_APLL_CLK_CTRL       ((UINT16)0x0003U)

/*!
 * @brief FECSS RFS runtime calibration command ID
 */
#define M_FE_RFS_CMD_ID_RUNTIME_CAL         ((UINT16)0x0004U)

/*!
 * @brief FECSS RFS RF clock BW config command ID
 */
#define M_FE_RFS_CMD_ID_RF_CLOCK_BW         ((UINT16)0x0005U)

/*!
 * @brief FECSS RFS GPADC trigger command ID
 */
#define M_FE_RFS_CMD_ID_GPADC_TRIG          ((UINT16)0x0006U)

/*!
 * @brief FECSS RFS Temperature trigger command ID
 */
#define M_FE_RFS_CMD_ID_TEMP_TRIG           ((UINT16)0x0007U)

/*!
 * @brief FECSS RFS Loopback enable command ID
 */
#define M_FE_RFS_CMD_ID_LB_ENA              ((UINT16)0x0008U)

/*!
 * @brief FECSS RFS RDIF config command ID
 */
#define M_FE_RFS_CMD_ID_RDIF_CFG            ((UINT16)0x0009U)

/*!
 * @brief FECSS RFS RF status GET command ID
 */
#define M_FE_RFS_CMD_ID_RF_STATUS_GET       ((UINT16)0x000AU)

/*!
 * @brief FECSS RFS RF Test chirp/CW sensor start command ID
 */
#define M_FE_RFS_CMD_ID_CW_SENS_START       ((UINT16)0x000BU)

/*!
 * @brief FECSS RFS RF Test chirp/CW sensor stop command ID
 */
#define M_FE_RFS_CMD_ID_CW_SENS_STOP        ((UINT16)0x000CU)

/*!
 * @brief FECSS RFS ATE Init trigger command ID (Internal use only)
 */
#define M_FE_RFS_CMD_ID_ATE_INIT            ((UINT16)0x000DU)

/*!
 * @brief FECSS RFS Debug command ID
 */
#define M_FE_RFS_CMD_ID_DBG_CFG             ((UINT16)0x000EU)

/*!
 * @brief FECSS RFS Sensor Dynamic power save disable command ID
 */
#define M_FE_RFS_CMD_SENS_DYN_PS_DIS        ((UINT16)0x000FU)

/*!
 * @brief FECSS RFS Sensor Dynamic power save disable Get command ID
 */
#define M_FE_RFS_CMD_SENS_DYN_PS_DIS_GET    ((UINT16)0x0010U)

/*!
 * @brief FECSS Tx runtime CLPC calibration command ID
 */
#define M_FE_RFS_CMD_ID_TX_RUNTIME_CLPC_CAL ((UINT16)0x0011U)

/*!
 * @brief FECSS RFS Monitor Enable and trigger command ID
 */
#define M_FE_RFS_CMD_MON_ENABLE_TRIGGER     ((UINT16)0x0040U)

/*!
 * @brief FECSS RFS Monitor Debug PD Meas command ID
 */
#define M_FE_RFS_CMD_MON_DBG_PD_MEAS        ((UINT16)0x0041U)

/*!
 * @brief FECSS RFS Monitor Debug TX Power Meas command ID
 */
#define M_FE_RFS_CMD_MON_DBG_TXPWR_MEAS     ((UINT16)0x0042U)

/*!
 * @brief RFS XTAL clock type index 25MHz, 26MHz, 38.4MHz and 40MHz
 */
#define M_FE_RFS_XTAL_CLK_FREQ_25M_INDX             ((UINT8)0U)
#define M_FE_RFS_XTAL_CLK_FREQ_26M_INDX             ((UINT8)1U)
#define M_FE_RFS_XTAL_CLK_FREQ_38P4M_INDX           ((UINT8)2U)
#define M_FE_RFS_XTAL_CLK_FREQ_40M_INDX             ((UINT8)3U)
#define M_FE_RFS_XTAL_CLK_FREQ_10M_INDX             ((UINT8)4U)

/*!
 * @brief RFS XTAL clock type index offset
 */
#define M_FE_RFS_XTAL_CLK_TYPE_OFFSET               ((UINT8)4U)

/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library RFS command module Type defines
 * @{
 */

/*!
 * @brief FECSS RFS Open Boot info Data structure - BIG endian structure is not supported now. TBD
 */
typedef struct
{
    /*!
     * @brief  RFS Clock Frequency, unit - 1LSB = 1/256MHz, FECSSLib can update
     * this dynamically, RFS does use this frequency in delay function.
     */
    UINT16 h_ClkFreq;

    /*!
     * @brief  RFS Power Mode bits[3:0], Cold = 0x0, Warm (memory retention) = 0xA \n
     *         RFS XTAL type  bits[7:4], 0 = 25MHz, 1 = 26MHz, 2 = 38.4MHz and 3 = 40MHz
     */
    UINT8 c_PwrModeAndXtalType;

    /*!
     * @brief  RFS API control configuration bits.
     * Bit field definition and values:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]   | RFS API error check disable control, \n Value 0x0 - Error check Enable \n  Value 0x1 - Error check Disable |
     * | bits [1]   | RFS API debug logger enable control, \n Value 0x0 - Logger Disable \n  Value 0x1 - Logger Enable |
     * | bits [7:2] | Reserved |
     */
    UINT8 c_RfsApiCtrl;

    /*!
     * @brief  The FEC RFS M3 boot mode configuration.  \n
     * Bit Definition: \n
     * | Bit Field | Definition |
     * |-----------|----------- |
     * | bits [0]   | FEC RFS M3 boot self-test disable flag. This field is applicable only for safety devices. \n Value 0 : Self test enable (default) \n  Value 1 : Self test disable |
     * | bits [7:1]  | Reserved |
     *
     */
    UINT8 c_FecBootCfg;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

} T_FE_RFS_BOOT_INFO;

/*!
 * @brief FECSS RFS Boot status Data structure, FECSSLib shall wait for boot complete and
 * read the info
 */
typedef struct
{
    /*!
     * @brief  RFS Boot Status, 0 - UnInit, 0xA - Pass, 0x5 -Fail
     */
    UINT8 c_BootStatus;

    /*!
     * @brief  RFS core IP boot self test status. \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0  |Test Failed  |
     * | 1 | Test Passed  |
     * Bit Field Definition:
     * | Bit Field   | Definition |
     * |---------|----------- |
     * | Bit[0]  | RFS M3 core MPU self test status  |
     * | Bit[1]  | RFS GPADC IP self test status  |
     * | Bit[2]  | RFS BIST FFT IP self test status  |
     * | Bit[7:3]  | Reserved  |
     */
    UINT8 c_RfsBootSelfTest;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_FE_RFS_BOOT_STS;

/*!
 * @brief FECSS RFS Command Error Response Data structure \n
 * In case of any error, the command ID 0x0000 will be updated in Response message with this data.
 * In case of success API, actual command ID will be sent part of response message.
 */
typedef struct
{
    /*!
     * @brief  RFS Command Error code
     */
    SINT32     xw_CmdErrorCode;

    /*!
     * @brief Id of the failed command
     */
    UINT16     h_CommandId;

    /*!
     * @brief  Reserved
     */
    UINT16     h_Reserved0;

    /*!
     * @brief  Reserved
     */
    UINT32     w_Reserved[2U];

} T_FE_API_RFS_ERROR_RSP;

/*!
 * @brief FECSS RFS FW version Get Response Data structure \n
 * The RFS ROM and Patch version numbers are read directly from IPC shared RAM.
 */
typedef struct
{
    /*!
     * @brief  RFS ROM version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_RomVersion;

    /*!
     * @brief  RFS Patch version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_PatchVersion;

} T_FE_API_RFS_FW_VER_GET_RSP;

/*!
 * @brief FECSS RFS Chirp profile Data structure \n
 * The Sensor chirp profile common configuration analog parameters are configured by
 * RFS core. These are common settings for all chirps, all the chirps in the frame will use these
 * profile parameters. \n
 */
typedef struct
{
    /*!
     * @brief  Digital output Sampling rate for chirp ADC samples is encoded in 1 byte
     * (8 bit unsigned number), the valid sampling rate can be configured as per below table. \n
     * | Value   | Chirp ADC sample rate |
     * |---------|-----------  |
     * | 8	     | 12.5 MHz    |
     * | 10	     | 10 MHz      |
     * | 12	     | 8.333 MHz   |
     * | 16	     | 6.25 MHz    |
     * | 20	     | 5 MHz       |
     * | 25	     | 4 MHz       |
     * | 32	     | 3.125 MHz   |
     * | 40	     | 2.5 MHz     |
     * | 50	     | 2 MHz       |
     * | 64	     | 1.5625 MHz  |
     * | 80	     | 1.25 MHz    |
     * | 100     | 1 MHz       |
     * Chirp ADC Sampling Rate in MHz = 400MHz / (c_DigOutputSampRate * 4) \n
     * Valid Range: 8 to 100 \n
     */
    UINT8 c_DigOutputSampRate;

    /*!
     * @brief  The sensor VCO multi chip cascade mode.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | SINGLE_CHIP VCO mode. \n Default Mode. |
     * | 1       | MULTI_CHIP_PRIMARY VCO mode. \n This mode is not supported in this DFP / Devices. |
     * | 2       | MULTI_CHIP_SECONDARY VCO mode. \n This mode is not supported in this DFP / Devices. |
     */
    UINT8 c_VcoMultiChipMode;

    /*!
     * @brief  Chirp Profile HPF corner frequency. This is a common HPF corner frequency value
     * for all chirps in a frame.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0       | 300kHz HPF corner frequency |
     * | 1       | 350kHz HPF corner frequency |
     * | 2       | 700kHz HPF corner frequency |
     * | 3       | 1400kHz HPF corner frequency |
     * | 4       | 2800kHz HPF corner frequency |
     * Valid Range: 0 to 4
     */
    UINT8 c_ChirpRxHpfSel;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3[6U];

} T_FE_API_RFS_CHIRP_PROFILE_CMD;

/*!
 * @brief FECSS RFS Live MON Sensor Start Data structure \n
 * The Sensor start live mon enable settings. \n
 */
typedef struct
{
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
     * | Bit[2]  | GPADC_CTM_LIVE_MON enable control. \n The monitor configuration shall be done using RL_MONITOR_GPADC_CTM_CFG API before enabling this bit.  |
     * | Bits[7:3]  | Reserved  |
     */
    UINT8 c_FrameLivMonEn;

    /*!
     * @brief  RFS Sensor Live Monitor Cfg Status. \n
     * Reset is done by FECSSLib every cold boot. This state is retained across deep sleep cycles.
     */
    UINT8  c_RfsLiveMonCfgStatus;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

} T_FE_API_RFS_LIVE_MON_SENS_START_CMD;

/*!
 * @brief FECSS RFS CW sensor start Data structure \n
 * The sensor CW mode and test burst trigger is done by RFS core, these test mode trigger
 * needs initial chirp timer / analog setup done by RFS. \n
 * The multiple sensor start command trigger shall be avoided while CW/burst is running in HW.
 */
typedef struct
{
    /*!
     * @brief  Sensor Trigger Mode. \n
     * This field indicates how the FECSSLib can instruct the RFS to trigger the CW mode or
     * a single test burst in sensor. \n
     * The sensor will be triggered based on the Mode selected in this API.
     * | Value | Definition |
     * |-------|----------- |
     * | 0 - 3 |  Reserved  |
     * | 4     | CW CZ Trigger Mode (CW_CZ_TRIG). \n In this Continuous Wave Streaming CZ mode, sensor can transmit constant RF frequency CHIRP_RF_FREQ_START signal programmed in CHIRP_PROFILE continuously with zero slope for testing/characterization purpose. \n The T_FE_API_RFS_CW_SENSOR_STOP_CMD RFS command shall be issued to stop the CW signal.  |
     * | 5     | Chirp Timer Override Trigger Mode (CHIRP_TIMER_TRIG). \n This is a internal test mode in which Frame timer is bypassed and a single frame can be triggered directly using chirp timer by RFS.  |
     */
    UINT8 c_SensorTrigMode;

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

} T_FE_API_RFS_CW_SENSOR_START_CMD;

/*!
 * @brief FECSS RFS CW sensor stop Data structure \n
 * This command shall be used to stop the sensor in forced mode while CW mode is running or
 * in case of any error recovery. The RFS reset the state of the sensor to idle state. \n
 */
typedef struct
{
    /*!
     * @brief  Sensor Stop Mode. \n
     * This field indicates how the FECSSLib can instruct the RFS sensor to stop the CW mode or
     * chirp timer and reset the state of the sensor to idle state. \n
     * The sensor can be stopped based on the Mode selected in this API. \n
     * | Value | Definition |
     * |-------|----------- |
     * | 0     | Reserved   |
     * | 1     | Stop CW CZ Mode. \n In this mode, the RFS stops the Chirp Timer in generating further CW signal transmissions. |
     * | 2     | Forced Timer Stop (Error recovery). \n This is a error recovery mode in which the Chirp and frame Timers are forcefully stopped and reset to recover from any error by FECSSLib and the state of the sensor will be reset to idle state by RFS. |
     *
     */
    UINT8 c_SensorStopMode;

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

} T_FE_API_RFS_CW_SENSOR_STOP_CMD;

/*!
 * @brief FECSS RFS Sensor count status structure \n
 */
typedef struct
{
    /*!
     * @brief  RFS Sensor Burst Start Count. \n
     * Reset is done by FECSSLib every new frame trigger, this can be used for debug purpose.
     */
    UINT32  w_BurstStartCount;

    /*!
     * @brief  RFS Sensor Burst End Count. \n
     * Reset is done by FECSSLib every new frame trigger, this can be used for debug purpose.
     */
    UINT32  w_BurstEndCount;

    /*!
     * @brief  RFS Sensor Frame Start Count. \n
     * Reset is done by FECSSLib every new frame trigger, this can be used for debug purpose.
     */
    UINT32  w_FrameStartCount;

    /*!
     * @brief  RFS Sensor Frame End Count. \n
     * Reset is done by FECSSLib every new frame trigger, this can be used for debug purpose.
     */
    UINT32  w_FrameEndCount;

    /*!
     * @brief  RFS Sensor Frame Free running Count. \n
     * Reset is done by FECSSLib every cold boot. This count is retained across deep sleep cycles.
     */
    UINT32  w_FrameFreeRunCount;

    /*!
     * @brief  RFS Sensor Monitor Cfg Status. \n
     * Reset is done by FECSSLib every cold boot. This state is retained across deep sleep cycles.
     */
    UINT32  w_RfsMonCfgStatus[2U];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved[1U];

} T_FE_API_RFS_SENSOR_STATUS;

/*!
 * @brief FECSS RFS IPC Data structure \n
 * The consolidated IPC data structures and memory map for 2048 Bytes
 * This memory holds RFS commands, Calibration and monitors results.
 * Start address: 0x21200000
 * Size: 2048 Bytes
 */
typedef struct
{
    /*!
     * @brief  IPC mailbox memory, 128 Bytes, address offset: 0x0
     */
    UINT32 w_IpcMailbox[32];

    /*!
     * @brief  Reserved for future use, 128 Bytes, address offset: 0x80
     */
    UINT32 w_Reserved1[32];

    /*!
     * @brief  RFS Boot info, 8 Bytes, address offset: 0x100
     */
    T_FE_RFS_BOOT_INFO z_RfsBootInfo;

    /*!
     * @brief  RFS Boot status, 8 Bytes, , address offset: 0x108
     */
    T_FE_RFS_BOOT_STS z_RfsBootSts;

    /*!
     * @brief  RFS FW version GET response, 16 Bytes, address offset: 0x110
     */
    T_FE_API_RFS_FW_VER_GET_RSP z_RfsFwVerRsp;

    /*!
     * @brief  RFS CPU status GET response, 52 Bytes, address offset: 0x120
     */
    T_RL_API_RFS_FAULT_STS_GET_RSP z_RfsCpuFaultSts;

    /*!
     * @brief  RFS GPADC CMD configuration, 72 Bytes, address offset: 0x154
     */
    T_RL_API_FECSS_GPADC_MEAS_CMD z_GpadcCfgCmd;

    /*!
     * @brief  RFS Temp Meas CMD configuration, 8 Bytes, address offset: 0x19c
     */
    T_RL_API_FECSS_TEMP_MEAS_CMD z_TempMeasCfgCmd;

    /*!
     * @brief  RFS Sensor Profile CMD configuration, 28 Bytes, address offset: 0x1A4
     */
    T_FE_API_RFS_CHIRP_PROFILE_CMD z_ProfileCfgCmd;

    /*!
     * @brief  RFS RF loopback CMD configuration, 32 Bytes, address offset: 0x1C0
     */
    T_RL_API_SENS_LOOPBACK_CFG z_RfLbCfgCmd;

    /*!
     * @brief  RFS Live Mon sensor start CMD configuration, 4 Bytes, address offset: 0x1D0
     */
    T_FE_API_RFS_LIVE_MON_SENS_START_CMD z_RfsLiveMonSensStartCmd;

    /*!
     * @brief  RFS Sensor Status, 32 Bytes, address offset: 0x1D4
     */
    T_FE_API_RFS_SENSOR_STATUS  z_RfsSensorStsRsp;

    /*!
     * @brief  RFS Monitor live synth freq CMD configuration, 12 Bytes, address offset: 0x1F4
     */
    T_RL_API_MON_LIVE_SYNTH_FREQ_CFG z_RfsMonLiveSynthFreqCmd;

    /*!
     * @brief  RFS Monitor live RX Saturation CMD configuration, 8 Bytes, address offset: 0x200
     */
    T_RL_API_MON_LIVE_RX_SATURATION_CFG z_RfsMonLiveRxSatCmd;

    /*!
     * @brief  RFS Monitor live GPADC CTM CMD configuration, 8 Bytes, address offset: 0x208
     */
    T_RL_API_MON_LIVE_GPADC_CTM_CFG z_RfsMonLiveGpadcCtmCmd;

    /*!
     * @brief  RFS Monitor PLL Control voltage CMD configuration, 8 Bytes, address offset: 0x210
     */
    T_RL_API_MON_PLL_CTRL_VOLT_CFG z_RfsMonPllCtrlVoltCmd;

    /*!
     * @brief  RFS Monitor TXn Rx LB CMD configuration, 16*4 Bytes, address offset: 0x218
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_TXN_RX_LB_CFG  z_RfsMonTxNRxLbCmd[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn Power CMD configuration, 16*4 Bytes, address offset: 0x258
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_TXN_POWER_CFG z_RfsMonTxNPwrCmd[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn BB CMD configuration, 16*4 Bytes, address offset: 0x298
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_TXN_BB_CFG z_RfsMonTxNBbCmd[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor TXn DC signal CMD configuration, 16*4 Bytes, address offset: 0x2D8
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_TXN_DCSIG_CFG z_RfsMonTxNDcSigCmd[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  RFS Monitor RX HPF and DC signal CMD configuration, 12 Bytes, address offset: 0x318
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_RX_HPF_DCSIG_CFG z_RfsMonRxHpfDcSigCmd;

    /*!
     * @brief  RFS Monitor PL LO CLK DC signal CMD configuration, 12 Bytes, address offset: 0x324
     * Reserved Memory for total 4 Tx.
     */
    T_RL_API_MON_PMCLK_DCSIG_CFG z_RfsMonPmClkDcSigCmd;

    /*!
     * @brief  Reserved for monitor cfg cmds, 80 Bytes, address offset: 0x330
     */
    UINT32 w_Reserved2[20];

    /*!
     * @brief  RFS calibration Data, 128 Bytes, address offset: 0x380
     */
    T_RL_API_FECSS_FACT_CAL_DATA z_RfsCalData;

    /*!
     * @brief  Reserved for future calibration, 256 Bytes, address offset: 0x400
     */
    UINT32 w_Reserved3[64];

    /*!
     * @brief  Reserved for Monitor results, 704 Bytes, address offset: 0x500
     * @ref T_RL_FE_RFS_IPC_MON_RESULT_DATA_STRUCT
     */
    UINT32 w_Reserved4[176];

    /*!
     * @brief  RFS debug memory, 64 Bytes, address offset: 0x7C0
     */
    UINT32 w_RfsDebugData[16];

} T_FE_RFS_IPC_DATA_STRUCTURE;

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */

/* Device commands */
M_LIB_EXPORT T_RETURNTYPE rfs_cmdFwVerGet_rom(UINT8 c_devIndex, \
                            T_FE_API_RFS_FW_VER_GET_RSP* p_fwVerData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdCpuStsGet_rom(UINT8 c_devIndex, \
                            T_RL_API_RFS_FAULT_STS_GET_RSP* p_cpuStsData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdFactCalDataSet_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_FACT_CAL_DATA* p_calData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdFactCalDataGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_FACT_CAL_DATA* p_calData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRxTxCalDataSet_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RXTX_CAL_DATA* p_calData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRxTxCalDataGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_RXTX_CAL_DATA* p_calData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfPwrOnOff_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_PWR_CFG_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfFactCal_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_FACT_CAL_CMD *p_cmdData, \
                            T_RL_API_FECSS_RF_FACT_CAL_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdApllClkCtrl_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfCalibrate_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RF_RUN_CAL_CMD *p_cmdData, \
                            T_RL_API_FECSS_RF_RUN_CAL_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfClockBwCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_CLK_BW_CFG_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdGpadcCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_GPADC_MEAS_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdGpadcTrig_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_GPADC_MEAS_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTempCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_TEMP_MEAS_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTempTrig_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_TEMP_MEAS_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdSensLbCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_LOOPBACK_CFG* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdSensLbEna_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_LOOPBACK_ENA* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdFecssRdifCtrl_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RDIF_CTRL_CMD* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_RF_STS_GET_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdDebugCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RFS_DBG_CFG_CMD *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRfTxRuntimeClpcCal_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD *p_apiCmdData, \
                            T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP* p_apiResData);

/* Sensor commands */
M_LIB_EXPORT T_RETURNTYPE rfs_cmdChirpProfCfg_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CHIRP_PROFILE_CMD* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdChirpProfCfgGet_rom(UINT8 c_devIndex, \
                            T_FE_API_RFS_CHIRP_PROFILE_CMD* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdSensStartCfg_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_LIVE_MON_SENS_START_CMD* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdCwSensorStart_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CW_SENSOR_START_CMD* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdCwSensorStop_rom(UINT8 c_devIndex, \
                            const T_FE_API_RFS_CW_SENSOR_STOP_CMD* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdSensDynPwrSaveDis_rom(UINT8 c_devIndex, \
                            const T_RL_API_SENS_DYN_PWR_SAVE_DIS* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdSensDynPwrSaveStsGet_rom(UINT8 c_devIndex, \
                            T_RL_API_SENS_DYN_PWR_SAVE_DIS* p_resData);
/* Monitor commands */
M_LIB_EXPORT T_RETURNTYPE rfs_cmdMonEnableTrig_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_ENABLE_TRIG* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdDbgPdMeas_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_DBG_PD_MEAS_CMD *p_cmdData, \
                            T_RL_API_MON_DBG_PD_MEAS_RSP* p_resData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdDbgTxPwrMeas_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_DBG_TXPWR_MEAS_CMD *p_cmdData, \
                            T_RL_API_MON_DBG_TXPWR_MEAS_RSP* p_resData);

M_LIB_EXPORT T_RETURNTYPE rfs_cmdLiveSynthFreqCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_SYNTH_FREQ_CFG* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdLiveRxSaturCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_RX_SATURATION_CFG* p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdLiveGpadcCtmCfg_rom(UINT8 c_devIndex, \
                            const T_RL_API_MON_LIVE_GPADC_CTM_CFG* p_cmdData);

M_LIB_EXPORT T_RETURNTYPE rfs_cmdPllCtrlVltMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_PLL_CTRL_VOLT_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTxNRxLbMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_RX_LB_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTxNPwrMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_POWER_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTxNBbMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_BB_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdTxNDcSigMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_TXN_DCSIG_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdRxHpfDcSigMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_RX_HPF_DCSIG_CFG *p_cmdData);
M_LIB_EXPORT T_RETURNTYPE rfs_cmdPmClkMonCfg_rom(UINT8 c_devIndex, \
                const T_RL_API_MON_PMCLK_DCSIG_CFG *p_cmdData);


#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rfs_driver.h
 */


