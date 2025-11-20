/*!*****************************************************************************
 * @file dfp_device.h
 *
 * @brief DFP mmWaveLink Device Module API Command and Response data structure header file
 *
 * @b Description @n
 * This is a mmWaveLink Device Module API Command and Response data structure
 * header file, same API data structure being used between FECSS and mmWaveLink Libraries
 *
 * @warning Application developer / User shall review Device APIs data structure,
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
 * @addtogroup MMWL_DEVICE_API mmWaveLink Device Control API functions
 * @{
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     12Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef DFP_DEVICE_H
#define DFP_DEVICE_H

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
 * @name mmWaveLink Library channel/IP/cfg control API macros
 * @{
 */

/*!
 * @brief 1bit control values used to control various FECSS APIs
 */
#define M_RL_API_CTRL_OFF                       ((UINT32)0x0U)
#define M_RL_API_CTRL_ON                        ((UINT32)0x1U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_FECSS_DEV_PWR_ON_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_PowerMode Field values
 */
#define M_RL_FECSS_PWR_ON_MODE_COLD             ((UINT8)0x00U)
#define M_RL_FECSS_PWR_ON_MODE_WARM             ((UINT8)0x0AU)

/*!
 * @brief Member .h_XtalClkFreq Field values \n
 * 25MHz, 26MHz, 38.4MHz and 40MHz values 6400, 6656, 9830 and 10240 respectively,
 * Unit 1LSB = 1/256MHz. \n
 * The 10MHz test clock is used for internal purpose only, not supported in production.
 */
#define M_RL_FECSS_XTAL_CLK_FREQ_10M            ((UINT16)2560U)
#define M_RL_FECSS_XTAL_CLK_FREQ_25M            ((UINT16)6400U)
#define M_RL_FECSS_XTAL_CLK_FREQ_26M            ((UINT16)6656U)
#define M_RL_FECSS_XTAL_CLK_FREQ_38P4M          ((UINT16)9830U)
#define M_RL_FECSS_XTAL_CLK_FREQ_40M            ((UINT16)10240U)
#define M_RL_FECSS_FCLK_CLK_FREQ_80M            ((UINT16)20480U)

/*!
 * @brief Member .c_ChirpTimerResol Field values \n
 * b[1:0] = b00 -> Chirp Freq and time params low resolution \n
 * b[1:0] = b01 -> Chirp Freq high resolution and time params low resolution \n
 * b[1:0] = b10 -> Chirp Freq low resolution and time params high resolution \n
 * b[1:0] = b11 -> Chirp Freq and time params high resolution \n
 */
#define M_RL_FECSS_CHIRP_TIMER_RES_00           ((UINT8)0x0U)
#define M_RL_FECSS_CHIRP_TIMER_RES_01           ((UINT8)0x1U)
#define M_RL_FECSS_CHIRP_TIMER_RES_10           ((UINT8)0x2U)
#define M_RL_FECSS_CHIRP_TIMER_RES_11           ((UINT8)0x3U)

#define M_RL_FECSS_CT_FREQ_RESOL_MASK           ((UINT8)0x1U)
#define M_RL_FECSS_CT_TIME_RESOL_OFFSET         ((UINT8)1U)

/*!
 * @brief Member .c_FecBootCfg Field value start bit \n
 */
#define M_RL_FECSS_FEC_BOOT_SELF_TEST_DIS       ((UINT8)0U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_FECSS_DEV_PWR_OFF_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_RetentionMode Field values
 */
#define M_RL_FECSS_PWR_DOWN_RET_OFF             ((UINT8)0x00U)
#define M_RL_FECSS_PWR_DOWN_RET_ON              ((UINT8)0x0AU)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_FECSS_DEV_CLK_CTRL_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_DevClkCtrl Field values
 */
#define M_RL_FECSS_DEV_CLK_SRC_XTAL             ((UINT8)0x00U)
#define M_RL_FECSS_DEV_CLK_SRC_FCLK             ((UINT8)0x0AU)
#define M_RL_FECSS_DEV_CLK_SRC_OFF              ((UINT8)0x05U)

/*!
 * @brief Member .c_FtClkCtrl Field values
 */
#define M_RL_FECSS_FT_CLK_SRC_XTAL              ((UINT8)0x00U)
#define M_RL_FECSS_FT_CLK_SRC_OFF               ((UINT8)0x05U)

/*!
 * @brief Member .c_ApllClkCtrl Field values
 */
#define M_RL_FECSS_APLL_CTRL_OFF                ((UINT8)0x00U)
#define M_RL_FECSS_APLL_CTRL_ON                 ((UINT8)0x0AU)
#define M_RL_FECSS_APLL_CTRL_ON_CAL             ((UINT8)0xAAU)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_FECSS_RF_PWR_CFG_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief FECSS RX channels supported, member .h_RxChCtrlBitMask start bit. \n
 * In xWRL6432 and xWRL1432 devices only 3 RX channels are supported.
 */
#define M_RL_RX_CHANNEL_0                       ((UINT32)0U)
#define M_RL_RX_CHANNEL_1                       ((UINT32)1U)
#define M_RL_RX_CHANNEL_2                       ((UINT32)2U)
#define M_RL_MAX_RX_CHANNELS                    ((UINT8)3U)

/*!
 * @brief FECSS TX channels supported, member .h_TxChCtrlBitMask start bit. \n
 * In xWRL6432 and xWRL1432 devices only 2 TX channels are supported.
 */
#define M_RL_TX_CHANNEL_0                       ((UINT32)0U)
#define M_RL_TX_CHANNEL_1                       ((UINT32)1U)
#define M_RL_MAX_TX_CHANNELS                    ((UINT8)2U)

/*!
 * @brief Member .c_MiscCtrl start bit
 */
#define M_RL_RF_MISC_CTRL_RDIF_CLK              ((UINT8)0U)
#define M_RL_RF_MISC_CTRL_RF_1V_LDO_BYPASS      ((UINT8)1U)
#define M_RL_MAX_RF_MISC_CONTROLS               ((UINT8)2U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_FECSS_RF_FACT_CAL_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief FECSS RFS device Calibration controls start bit
 */
#define M_RL_FECSS_DEV_CAL_APLL                 ((UINT32)0U)
#define M_RL_FECSS_DEV_CAL_VCO                  ((UINT32)1U)
#define M_RL_FECSS_DEV_CAL_PD                   ((UINT32)2U)
#define M_RL_FECSS_DEV_CAL_LODIST               ((UINT32)3U)
#define M_RL_FECSS_DEV_CAL_RESERVED             ((UINT32)4U)
#define M_RL_FECSS_DEV_CAL_RX_IFA               ((UINT32)5U)
#define M_RL_FECSS_DEV_CAL_RX_GAIN              ((UINT32)6U)
#define M_RL_FECSS_DEV_CAL_TX_PWR               ((UINT32)7U)
#define M_RL_MAX_FECSS_DEV_CALIBRATIONS         ((UINT8)8U)

/*!
 * @brief FECSS Calibration Temperature bin index
 */
#define M_RL_FECSS_CAL_TEMP_INDEX_LOW           ((UINT32)0U << 3U)
#define M_RL_FECSS_CAL_TEMP_INDEX_MID           ((UINT32)1U << 3U)
#define M_RL_FECSS_CAL_TEMP_INDEX_HIGH          ((UINT32)2U << 3U)
#define M_RL_FECSS_CAL_MAX_TEMP_BINS            ((UINT8)3U)

/*!
 * @brief Member .c_CalRxGainSel Field values
 */
#define M_RL_FECSS_CAL_RX_GAIN_MIN                       ((UINT8)16U)
#define M_RL_FECSS_CAL_RX_GAIN_MAX                       ((UINT8)59U)

/*!
 * @brief Member .c_CalRxGainSel Field extraction macros
 */
#define M_RL_FECSS_CAL_RX_GAIN_MASK                      ((UINT8)0x3FU)


/*!
 * @brief Member .c_CalTxBackOffSel Field values
 */
#define M_RL_FECSS_CAL_TX_BO_MIN                         ((UINT8)0U)
#define M_RL_FECSS_CAL_TX_BO_MAX                         ((UINT8)52U)

/*!
 * @brief Member .c_MiscCalCtrl Field values
 */
#define M_RL_FECSS_CAL_RF_GAIN_CAL_DIS                  ((UINT8)1U << (UINT8)0U)
#define M_RL_FECSS_CAL_TX_CLPC_CAL_DIS                  ((UINT8)1U << (UINT8)1U)
#define M_RL_FECSS_CAL_TX_CAL_TEMP_OVERRIDE             ((UINT8)1U << (UINT8)2U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_FECSS_GPADC_MEAS_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief FECSS RFS GPADC channel controls start bit, max 8 channels
 */
#define M_RL_FECSS_GPADC_CH_0                   ((UINT32)0U)
#define M_RL_FECSS_GPADC_CH_1                   ((UINT32)1U)
#define M_RL_FECSS_GPADC_CH_2                   ((UINT32)2U)
#define M_RL_FECSS_GPADC_CH_3                   ((UINT32)3U)
#define M_RL_FECSS_GPADC_CH_4                   ((UINT32)4U)
#define M_RL_FECSS_GPADC_CH_5                   ((UINT32)5U)
#define M_RL_FECSS_GPADC_CH_6                   ((UINT32)6U)
#define M_RL_FECSS_GPADC_CH_7                   ((UINT32)7U)
#define M_RL_FECSS_MAX_GPADC_CHANNELS           ((UINT8)8U)

/*! @} */

/*!
 * @name mmWaveLink Library T_RL_API_FECSS_TEMP_MEAS_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief FECSS RFS Temperature sensor controls start bit
 */
#define M_RL_FECSS_TEMP_SENS_RX0                ((UINT32)0U)
#define M_RL_FECSS_TEMP_SENS_RSVD0              ((UINT32)1U)
#define M_RL_FECSS_TEMP_SENS_RSVD1              ((UINT32)2U)
#define M_RL_FECSS_TEMP_SENS_RSVD2              ((UINT32)3U)
#define M_RL_FECSS_TEMP_SENS_TX0                ((UINT32)4U)
#define M_RL_FECSS_TEMP_SENS_RSVD3              ((UINT32)5U)
#define M_RL_FECSS_TEMP_SENS_RSVD4              ((UINT32)6U)
#define M_RL_FECSS_TEMP_SENS_RSVD5              ((UINT32)7U)
#define M_RL_FECSS_TEMP_SENS_PM                 ((UINT32)8U)
#define M_RL_FECSS_TEMP_SENS_DIG                ((UINT32)9U)
#define M_RL_FECSS_MAX_TEMP_SENSORS             ((UINT8)10U)

/*! @} */


/*!
 * @name mmWaveLink Library T_RL_API_FECSS_RDIF_CTRL_CMD Data Structure field value macros
 * @{
 */

/*!
 * @brief Member .c_RdifEnable Field values
 */
#define M_RL_FECSS_RDIF_DIS                     ((UINT8)0x00U)
#define M_RL_FECSS_RDIF_ENA                     ((UINT8)0x0AU)

/*!
 * @brief Member .c_RdifCfg enable start bit
 */
#define M_RL_FECSS_RDIF_SIDEBAND                ((UINT32)0U)
#define M_RL_FECSS_RDIF_CW_MODE                 ((UINT32)1U)
#define M_RL_FECSS_RDIF_SWIZZL_MODE             ((UINT32)2U)
#define M_RL_FECSS_RDIF_SCRAMBLER_MODE          ((UINT32)4U)
#define M_RL_FECSS_RDIF_LANE_RATE_UPDATE        ((UINT32)5U)

/*!
 * @brief Member .c_RdifCfg swizzling modes
 */
#define M_RL_FECSS_RDIF_SWIZZL_PIN0_BIT0_CYL1   ((UINT32)0x0U)
#define M_RL_FECSS_RDIF_SWIZZL_PIN3_BIT0_CYL1   ((UINT32)0x1U)
#define M_RL_FECSS_RDIF_SWIZZL_PIN0_BIT0_CYL3   ((UINT32)0x2U)
#define M_RL_FECSS_RDIF_SWIZZL_PIN3_BIT0_CYL3   ((UINT32)0x3U)

/*!
 * @brief Member .c_TestPatternEn Field values
 */
#define M_RL_FECSS_TEST_PATRN_DIS               ((UINT8)0x00U)
#define M_RL_FECSS_TEST_PATRN_ENA               ((UINT8)0x0AU)

/*!
 * @brief Member .c_LaneRateCfg Field values
 */
#define M_RL_FECSS_RDIF_LANE_RATE_400M          ((UINT8)0U)
#define M_RL_FECSS_RDIF_LANE_RATE_320M          ((UINT8)1U)
#define M_RL_FECSS_RDIF_LANE_RATE_200M          ((UINT8)2U)
#define M_RL_FECSS_RDIF_LANE_RATE_160M          ((UINT8)3U)
#define M_RL_FECSS_RDIF_LANE_RATE_100M          ((UINT8)4U)

/*! @} */


/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name mmWaveLink Library Device Module API Type defines
 * @{
 */

/*!
 * @brief mmWaveLink @ref rl_fecssDevPwrOn API command Data structure \n
 * FECSS power ON config command data structure contains the device power ON control,
 * power modes and Clock config settings. The Application shall powerup the FECSS before issuing
 * other functional APIs, similarly application shall power down the FECSS before entering deep
 * sleep mode or SW warm reset.
 */
typedef struct
{
    /*!
     * @brief  Device XTAL Clock Frequency, The Device supports 25MHz, 26MHz,
     * 38.4MHz and 40MHz clock frequencies. This information shall be passed on to
     * FECSS RFS component through this API. \n
     * Unit: 1 LSB = 1/256 MHz \n
     * Allowed XTAL clocks are 25MHz, 26MHz, 38.4MHz, 40MHz and 10MHz.  \n
     * The 10MHz test clock is supported only for internal test purpose, it is not
     * supported in production mode.
     * Allowed values are: 6400, 6656, 9830, 10240 and 2560 \n
     */
    UINT16 h_XtalClkFreq;

    /*!
     * @brief  FECSS Clock Source type, FECSS clock source selection type during the
     * power up. FECSS supports running on either XTAL clock or on Fast clock
     * (ADPLL / APLL 80MHz). This API configures FECSS clock based on this setting.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | XTAL clock source \n Supported XTAL frequencies: 25MHz, 26MHz, 38.4MHz, 40MHz |
     * | 0x0A  | FClock clock source, Fast clock can be either DIG_PLL or APLL 80MHz clock set by application \n Clock Frequency: DIG_PLL_FREQ/2 (Typical 80MHz with 40MHz XTAL) |
     */
    UINT8 c_ClkSourceSel;

    /*!
     * @brief  FECSS Power Up Mode, The power up mode can be either cold or warm boot type,
     *  the warm boot can be done only if power down is done with RETENTION_MODE = 0x0A to
     * retain the FECSS memory.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | Cold Boot. If POWER_MODE is 0x00 then, FECSS power up is performed in cold boot  and the RFS data memory (.data, .bss section) is re-initialized during powerup. The RFS code memory shall be initialized by boot loader/application before calling this API.  \n Typical use case: First time boot or nReset.  \n  |
     * | 0x0A  | Warm Boot(memory retention), If POWER_MODE is 0x0A then, FECSS power up is performed in warm boot and the RFS data memory (.data, .bss section) is retained,  un-initialized during power up. \n To support warm boot mode, the FECSS power down shall be done with RETENTION_MODE = 0x0A, this will retain the FECSS memory in deep sleep. \n Typical use case: Deep Sleep exit, Warm reset.  \n |
     */
    UINT8 c_PowerMode;

    /*!
     * @brief  The Chirp timer module timing engine resolution selection. The RF start
     * frequency resolution and timing related chirp parameters resolution selection. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | Low Resolution Mode   |
     * | 0x1       | High Resolution Mode  |
     * Bit Definition: \n
     * | Bit Field | Definition |
     * |-----------|----------- |
     * | 0   | Resolution of CHIRP_RF_FREQ_START. \n The Low Resolution Mode supports 16 bit CHIRP_RF_FREQ_START value and High Resolution Mode supports 24bit  CHIRP_RF_FREQ_START value. \n This resolution setting is applicable for RF_FREQ_START parameter in sensor PER_CHIRP_CFG and PROFILE_TIME_CFG APIs. |
     * | 1   | Resolution of Timing Related chirp parameters. \n The Low Resolution Mode supports 1 LSB = 40/APLL_FREQ (Typical 100ns with APLL_FREQ = 400MHz) and High Resolution Mode supports 1 LSB = 8/APLL_FREQ (Typical 20ns with APLL_FREQ = 400MHz). \n This resolution setting is applicable for IDLE_TIME and RAMP_END_TIME parameters in sensor PROFILE_TIME_CFG API, BURST_PERIODICITY parameter in FRAME_CFG API and IDLE_TIME parameter in PER_CHIRP_CFG API. |
     * Recommended to configure the CHIRP_TIMER_RESOL_SEL at boot once per power cycle,
     * the RFS subsystem has dependency on this setting, hence it is recommended not to update
     * this field dynamically. \n
     */
    UINT8 c_ChirpTimerResol;

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
    UINT16 h_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_FECSS_DEV_PWR_ON_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssDevPwrOff API command Data structure \n
 * FECSS power OFF config command data structure contains the device power mode setting. The
 * Application shall power down the FECSS before entering deep sleep mode or SW warm reset.
 */
typedef struct
{
    /*!
     * @brief  FECSS Memory Retention Mode, the mode can be either RETENTION OFF or RETENTION
     * ON type, the warm boot can be done only if power down is done with RETENTION_MODE = 0x0A
     * to retain the FECSS memory.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | RETENTION OFF. If RETENTION_MODE is 0x00 then, the FECSS RAM is not retained across deep sleep cycle. The successive FEC power up POWER_MODE supposed to be cold boot type.  \n  Typical use case: Hibernate.  \n |
     * | 0x0A  | RETENTION ON (memory retention enable), If RETENTION_MODE is 0x0A then, FECSS RAM is retained across deep sleep cycle. The successive FEC power up POWER_MODE supposed to be warm boot type. \n Typical use case: Deep Sleep Entry, Warm reset.  \n |
     */
    UINT8 c_RetentionMode;

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
    UINT32 w_Reserved3;

} T_RL_API_FECSS_DEV_PWR_OFF_CMD;

/*!
 * @brief DFP component FW version Data structure.
 */
typedef struct
{
    /*!
     * @brief  Generation version number
     */
    UINT8 c_GenVerNum;

    /*!
     * @brief  Major version number
     */
    UINT8 c_MajorVerNum;

    /*!
     * @brief  Minor version number
     */
    UINT8 c_MinorVerNum;

    /*!
     * @brief  Build version number
     */
    UINT8 c_BuildVerNum;

    /*!
     * @brief  Version Year
     */
    UINT8 c_Year;

    /*!
     * @brief  Version Month
     */
    UINT8 c_Month;

    /*!
     * @brief  Version Date
     */
    UINT8 c_Date;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved;

} T_DFP_COMP_FW_VER;

/*!
 * @brief mmWaveLink @ref rl_mmWaveDfpVerGet API Response Data structure. \n
 * mmWave DFP version data structure contains version information of mmWaveLink Library,
 * FECSS Library and RFS ROM and Patch firmware.
 */
typedef struct
{
    /*!
     * @brief  mmWaveLink Lib version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_MmwlLibVersion;

    /*!
     * @brief  FECSSLib version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_FecssLibVersion;

    /*!
     * @brief  RFS ROM version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_RfsRomVersion;

    /*!
     * @brief  RFS Patch version, 8 bytes
     */
    T_DFP_COMP_FW_VER     z_RfsPatchVersion;

} T_RL_API_DFP_FW_VER_GET_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfPwrOnOff API command Data structure. \n
 * FECSS RF power ON/OFF config command data structure contains the RF RX, TX channels and analog
 *  IP power ON/OFF controls. The Application shall powerup the FECSS RF channels and IPs before
 * issuing other sensor functional APIs, similarly application shall power down the FECSS RF
 * channels and IPs before entering deep sleep mode or SW warm reset or RF is not in use.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS RX channels ON/OFF control, 1 bit per channel. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF, RX channel will not be enabled in any functional frame, calibration and monitor trigger |
     * | 0x1       | ON, RX channel will be enabled automatically      |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]  | Rx Channel 0 |
     * | bits [1]  | Rx Channel 1 |
     * | bits [2]  | Rx Channel 2 |
     * | bits [15:3] | Reserved |
     * The RFS enables the RX channels as per this API setting automatically before below
     * functionalities and disables it in the end. \n
     *   - Calibration trigger
     *   - Functional frames
     *   - Monitor trigger
     *   - CW mode trigger
     *   - Test Burst trigger
     *   - RDIF Data transfer
     * In idle state, the Tx, RX channels are by default in power down state. Curing TX and RX
     * Calibrations and Monitors at least one RX channel shall be enabled, The RFS FW enables
     * RX channel 1 (middle) if this mask value is zero.
     */
    UINT16 h_RxChCtrlBitMask;

    /*!
     * @brief  FECSS RFS TX channels ON/OFF control, 1 bit per channel. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF, TX channel will not be enabled in any functional frame, calibration and monitor trigger |
     * | 0x1       | ON, TX channel will be enabled automatically      |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]  | Tx Channel 0 |
     * | bits [1]  | Tx Channel 1 |
     * | bits [15:2] | Reserved |
     * The RFS enables the TX channels as per this API setting automatically before below
     * functionalities and disables it in the end. \n
     *   - Calibration trigger
     *   - Functional frames
     *   - Monitor trigger
     *   - CW mode trigger
     *   - Test Burst trigger
     * In idle state, the TX channels are by default in power down state. \n
     * In TX Calibrations and Monitors, the corresponding TX channel shall be enabled,
     * The RFS FW enables corresponding TX channel if this mask value is zero.
     */
    UINT16 h_TxChCtrlBitMask;

    /*!
     * @brief Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  FECSS RFS FECSS Miscellaneous block / clock control, 1 bit per control. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | Block Disable |
     * | 0x1       | Block Enable  |
     * Bit field definition:
     * | Bit Field     | Definition  |
     * |---------      |-----------  |
     * | Bit [0]    | RDIF clock enable control. \n The RDIF IP block clock shall be enabled using this control before configuring and enabling the RDIF. \n This clock is derived from APLL clock, APLL shall be ON while enabling this clock.  |
     * | Bit [1]     | RF ANA 1V LDO bypass control. \n By default the internal LDO is used to drive 1V supply. \n To bypass the internal LDO, SET this bit. \n Note: External 1V supply shall be connected in case of bypass \n Note: Reserved for internal test only. |
     * | Bits [7:2]  | Reserved    |
     *
     */
    UINT8 c_MiscCtrl;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_RF_PWR_CFG_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssRfFactoryCal API command Data structure. \n
 * FECSS RF factory calibration configuration command data structure contains the device
 * calibration controls and input configurations, this API shall be used to configure and
 * trigger TI and Customer factory calibrations. \n
 * Note: Some of the calibrations marked in the table below are only required for early
 * pre-production samples, where the calibration values are not yet fused into the device at ATE.
 * These calibrations require a special ATE firmware build to be used. Refer
 * "API Programming Sequence for Factory Calibration" section at the end of the ICD document
 * regarding factory calibrations.
 * @note
 * -# All loopbacks must be disabled before running calibrations
 * -# Factory calibrations are recommended to run within 10ᵒC to 50ᵒC junction temperatures
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS Boot calibration control, 1 bit per channel.  \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF        |
     * | 0x1       | ON         |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]   | RESERVED |
     * | bits [1]   | VCO calibration ON/OFF control |
     * | bits [2]   | PD calibration ON/OFF control |
     * | bits [3]   | LODIST calibration ON/OFF control.|
     * | bits [4]   | RESERVED |
     * | bits [5] | RX IFA calibration ON/OFF control. \n Note: Required only for pre-production samples – to be used with ATE calibration FW build |
     * | bits [6] | RX Gain calibration ON/OFF control.|
     * | bits [7] | TX power calibration ON/OFF control.|
     * | bits [15:8] | Reserved |
     * The calibration results are stored in a dedicated memory, the calibration
     * store / restore APIs can be used to replace the calibration results.
     * @note Recommended to run VCO, PD and LODIST calibrations along with RX and TX boot calibrations.
     */
    UINT16 h_CalCtrlBitMask;

    /*!
     * @brief  Miscellaneous calibration control. \n
     * Bit field definition:
     * | Bit Field  | Definition |
     * |---------   |----------- |
     * | Bit[0]     | RF_GAIN_CAL_DIS, RF gain factory calibration disable control. If PD and chirp based RF gain calibration is done once, the repeated calibration PD measurements can be avoided by setting this bit during computation of RX gain codes for various CAL_RX_GAIN_SEL values.  \n Value 0 : Enable the chirp based calibration \n Value 1: Disable chirp based calibration |
     * | Bit[1]     | TX_CLPC_CAL_DIS, TX closed loop (CLPC) factory calibration disable control (override). \n Value 0 : Enable the chirp based TX CLPC calibration \n Value 1: Disable chirp based TX CLPC calibration |
     * | Bit[2]     | Tx Calibration Override Bin Temperature |
     * | bits [7:3] | RESERVED |
     */
    UINT8 c_MiscCalCtrl;

    /*!
     * @brief  Chirp Profile RX channels gain setting for calibration. This is a common RX and
     * RF gain value for all chirps in a frame. \n
     * The FEC analog is calibrated to this selected gain setting and RX_GAIN_CODES are computed
     * for three temperature bins. In case user has multiple chirp profiles, then store the
     * calibrated RX_GAIN_CODES for each of the CAL_RX_GAIN_SEL using
     * RL_FECSS_RF_RX_TX_CALIB_DATA_GET API and update the RX_GAIN_CODES dynamically in the
     * interframe time. \n
     * Bit Definition:  \n
     * | Bit Field   | Definition |
     * |---------|----------- |
     * | Bits[5:0]  | RX gain selection for each RX channels \n 1LSB: 1dB \n Valid FW Range for 60GHz: even values from 22dB to 42dB \n Valid Range for 77GHz: even values from 20dB to 42dB \n Recommended Range: 30dB to 40dB in both 60GHz and 77GHz devices |
     * | Bits[7:6]       | Reserved |
     */
    UINT8 c_CalRxGainSel;

    /*!
     * @brief  Chirp Profile TX channels power back-off setting for calibration. This is a common
     * TX back-off value for all chirps in a frame. \n
     * The FEC analog is calibrated to this selected back-off setting and TX_BIAS_CODES are
     * computed for three temperature bins. In case user has multiple chirp profiles, then store
     * the calibrated TX_BIAS_CODES for each of the CAL_TX_BACK_OFF_SEL using
     * RL_FECSS_RF_RX_TX_CALIB_DATA_GET API and update the TX_BIAS_CODES dynamically in the
     * interframe time. \n
     * Byte definition: \n
     * | Byte Field   | Definition |
     * |------------  |----------- |
     * | 0            | TX0 output power back-off value  |
     * | 1            | TX1 output power back-off value  |
     * | Byte [3:2]   | Reserved  |
     * 1LSB: 0.5dB \n
     * Valid Range: even values (1dB step) from 0 (0dB) to 52 (26dB) \n
     *
     * @note: Accuracy of the Factory calibration may degrade with backoff. Refer calibration and monitoring
     * AppNote for device specific recommendations.
     */
    UINT8 c_CalTxBackOffSel[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved0;

    /*!
     * @brief  Calibration chirp RF start Frequency. This is a common RF start frequency value
     * for all calibration chirps. \n
     * This field indicates the required start frequency of the chirp. \n
     * Recommended configuration: Mid frequency of the operating band
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x APLL_FREQ / 2^16) * 2^6 = ~1.172 MHz \n Valid Range (APLL_FREQ = 400MHz): 0x0000BE00 (57GHz) to 0x0000D555 (64GHz) |
     * | 77GHz | 1 LSB = (4 x APLL_FREQ / 2^16) * 2^6 = ~1.562 MHz \n Valid Range (APLL_FREQ = 400MHz): 0x0000BE00 (76GHz) to 0x0000CA80(81GHz) |
     *
     */
    UINT16 h_CalRfFreq;

    /*!
     * @brief  RFS calibration chirp RF Frequency Slope. This is a common RF frequency slope
     * value for all calibration chirps. \n
     * This field indicates the required FMCW slope of the chirp. @n
     * The typical ramp time used in FW for calibration chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_CalRfSlope;

    /*!
     * @brief  TX channel power calibration TX enable mask. \n
     * Number of transmitters to turn on during TX power calibration. During actual operation,
     * if more than 1 TXs are enabled during the chirp, then enabling the same TXs during
     * calibration will have better TX output power accuracy. \n
     * If all TX channels have same back-off then use this mask to pick the right common
     * calibrated bias code for all Tx channels.
     * | Byte Field   | Definition |
     * |--------------  |----------- |
     * | Byte[0] | TX channel 0 power calibration TX enable Mask. \n Bit[0] is TX0 control during calibration. If TX0 is disabled then, the TX1/ enabled calibration bias code is applied to TX0, in this case only one TX shall be enabled in the mask to pick the right bias code for TX0. The calibration is not performed for TX0 if this bit is 0. \n Bit[1] is TX1 control during calibration |
     * | Byte[1] | TX channel 1 power calibration TX Enable Mask. \n Bit[0] is TX0 control during calibration \n TX1 control during calibration. If TX1 is disabled then, the TX0/ enabled calibration bias code is applied to TX1, in this case only one TX shall be enabled in the mask to pick the right bias code for TX1. The calibration is not performed for TX1 if this bit is 0.|
     * | Byte[3:2] | Reserved |
     * Below bit fields can be used to control the TXn during the calibration.
     * | Bit Field   | Definition |
     * |--------------  |----------- |
     * | Bit [0] | TX0 selection \n Value 0: Disable \n Value 1: Enable |
     * | Bit [1] | TX1 selection \n Value 0: Disable \n Value 1: Enable |
     * | Bits [7:2] | Reserved |
     * Recommended Default: Byte[0] = 0x03 and Byte[1] = 0x01 \n
     * Example: \n
     * TX0 back-off: 0dB \n TX1 back-off: 0dB \n Enable TX calibration only for TX0 and
     * use same calibrated codes in TX0 and TX1. \n
     * Byte[0] = 0x3 (Enabled both TX in TX0 calibration) \n
     * Byte[1] = 0x1 (Pick TX0 bias code for TX1 and TX1 calibration is disabled)
     *
     */
    UINT8 c_TxPwrCalTxEnaMask[M_DFP_MAX_TX_CHANNELS];

    /**
     * @brief Reserved for Tx enable masks in future devices
     */
    UINT32 w_Reserved1;

    /*!
     * @brief TX temperature override values.
     * This field is valid if RX_TX_TEMP_BIN_OVRD bit is set in MISC_CAL_CTRL
     * - Range: -40C - 125C
     * - 1 LSB = 2C
     */
    SINT8 xc_CalTempBinOverrides[M_DFP_MAX_CAL_TEMP_BINS];

    /*!
     * @brief  Reserved 2 bytes
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved 8 bytes
     */
    UINT32 w_Reserved2[2U];

} T_RL_API_FECSS_RF_FACT_CAL_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssRfFactoryCal API response Data structure. \n
 * FECSS RF factory calibration result response data structure contains the
 * device calibration run and result validity status.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS factory calibration run status, 1 bit per calibration.  \n
     * This is a status of the current calibration command trigger.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | NU / FAIL - Not Updated, Calibration trigger is disabled. / Enabled calibration has failed.   |
     * | 0x1       | PASS - Enabled calibration has Passed.         |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | RESERVED |
     * | bits [1]    | VCO calibration status |
     * | bits [2]    | PD calibration status  |
     * | bits [3]    | LODIST calibration status  |
     * | bits [4]    | RESERVED |
     * | bits [5]    | RX IFA calibration status |
     * | bits [6]    | RX Gain calibration status |
     * | bits [7]    | TX power calibration status |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalRunStatus;

    /*!
     * @brief  FECSS RFS factory calibration result validity status, 1 bit per calibration.  \n
     * This is the latest validity status of all the calibrations. \n
     * The previous validity status is retained if latest calibration trigger fails.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | INVALID - Calibration data is invalid.    |
     * | 0x1       | VALID - Calibration data is valid.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | APLL calibration validity status. \n Note: Updates status of APLL calibration. |
     * | bits [1]    | VCO calibration validity status |
     * | bits [2]    | PD calibration validity status  |
     * | bits [3]    | LODIST calibration validity status  |
     * | bits [4]    | RESERVED |
     * | bits [5]    | RX IFA calibration validity status \n Note: Updates status of RX IFA calibration. |
     * | bits [6]    | RX Gain calibration validity status |
     * | bits [7]    | TX power calibration validity status |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalResStatus;

    /*!
     * @brief  FECSS RF calibration temperature value, The device temperature at which calibration
     * is executed. \n
     * 1LSB = 2 deg C \n
     * Range : -127 to 127 (-254 deg C to 254 deg C)  \n
     */
    SINT8 xc_CalibTemp;

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
    UINT32 w_Reserved4;

} T_RL_API_FECSS_RF_FACT_CAL_RSP;


/*!
 * @brief mmWaveLink @ref rl_fecssRlRuntimeTxClpcCal API configuration data structure
 * FECSS RF runtime Tx close loop calibration API configuration contains Tx calibration controls
 * and override options
 * @note
 * -# All loopbacks must be disabled before running the calibration
 *
 * @b Recommendation @n
 * -# Tx CLPC is recommended for non 0-db backoff use cases
 */
typedef struct
{
    /*!
     * @brief Tx calibration mode
     * TX CLPC calibration mode. This API can be used to either trigger the CLPC calibration in
     * targeted temperature and apply calibrated TX bias code to HW or just apply configured bias
     * code to HW. @n
     *
     * | Bit | Description |
     * |-----|-------------|
     * | 0   | Calibration Mode \n 0: [Default] Run Tx CLPC and apply bias codes to hardware \n 1: Override mode |
     * | 7-1 | Reserved |
     *
     * @b Notes: @n
     * -# <B> Mode 0: </B> can be used to build the bias code LUT externally for various temperature
     *      bins in runtime and store same in non-volatile memory to minimize the periodic
     *      TX calibrations in the field.
     * -# <B> Mode 1: </B> can be used to apply the constructed LUT bias code to HW in runtime
     *      when LUT data is available.
     */
    UINT8   c_CalMode;

    /*!
     * @brief Override mode temperature
     * Temperature for which Tx stage override codes are applied. This value is used to correctly
     * set temperature dependent internal bias Values
     * - 1 LSB = 2 C
     * - Valid Range: -40 - 125
    */
    SINT8    xc_OverrideCalTemp;

    /*!
     * @brief  Reserved
     */
    UINT8   c_Reserved0[2U];

    /*!
     * @brief Chirp Profile TX channels power back-off setting for calibration. This is a common
     * TX back-off value for all chirps in a frame.
     *
     * The FEC analog is calibrated to this selected back-off setting and TX_BIAS_CODES are
     * computed for three temperature bins. In case user has multiple chirp profiles, then store
     * the calibrated TX_BIAS_CODES for each of the CAL_TX_BACK_OFF_SEL using
     * RL_FECSS_RF_RX_TX_CALIB_DATA_GET API and update the TX_BIAS_CODES dynamically in the
     * interframe time. \n
     * @note: These values are needed irrespective on override mode setting (TX_CLPC_MODE)
     * Byte definition: \n
     * | Byte Field   | Definition |
     * |------------  |----------- |
     * | 0            | TX0 output power back-off value  |
     * | 1            | TX1 output power back-off value  |
     * | Byte [3:2]   | Reserved  |
     * 1LSB: 0.5dB \n
     * Valid Range: even values (1dB step) from 0 (0dB) to 52 (26dB) \n
     */
    UINT8 c_CalTxBackOffSel[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Calibration chirp RF start Frequency. This is a common RF start frequency value
     * for all calibration chirps. \n
     * This field indicates the required start frequency of the chirp.
     * | RF Frequency   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = (3 x 400MHz / 2^16) * 2^6 = ~1.172 MHz \n Valid Range: 0x0000BE00 (57GHz) to 0x0000D555 (64GHz) \n Recommended Default: 60.5GHz (0xC9AA) |
     * | 77GHz | 1 LSB = (4 x 400MHz / 2^16) * 2^6 = ~1.562 MHz \n Valid Range: 0x0000BE00 (76GHz) to 0x0000CA80(81GHz) \n Recommended Default: 78GHz (0xC300)|
     *
     */
    UINT16 h_CalRfFreq;

    /*!
     * @brief  RFS calibration chirp RF Frequency Slope. This is a common RF frequency slope
     * value for all calibration chirps. \n
     * This field indicates the required FMCW slope of the chirp. @n
     * The typical ramp time used in FW for calibration chirps are ~55us, the slope shall be selected such that RF BW is within SYNTH operating range
     *
     * | RF Slope   | Resolution |
     * |--------------  |----------- |
     * | 60GHz | 1 LSB = 3 * 400 * APLL_FREQ / 2^24 = ~28.610 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x4D) |
     * | 77GHz | 1 LSB = 4 * 400 * APLL_FREQ / 2^24 = ~38.147 kHz/us , signed number \n Valid range: -3495 to 3495 (+/- 100MHz/us (APLL_FREQ = 400MHz)) \n Recommended Default (APLL_FREQ = 400MHz): 2.2MHz/us (0x3A) |
     *
     */
    SINT16 xh_CalRfSlope;

    /*!
     * @brief TX channel power calibration TX enable mask.
     * Number of transmitters to turn on during TX power calibration.
     *  - During actual operation, if more than 1 TXs are enabled during the chirp, then enabling the same TXs during
     *      calibration will have better TX output power accuracy.
     * - If all TX channels have same back-off then use this mask to pick the right common
     * - @b NOTE: This mask is applicable even if the TX_CLPC_MODE is set to 1.
     *      This helps to selectively update Tx stage codes
     * calibrated bias code for all Tx channels.
     * | Byte Field   | Definition |
     * |--------------  |----------- |
     * | Byte[0] | TX channel 0 power calibration TX enable Mask. \n Bit[0] is TX0 control during calibration. If TX0 is disabled then, the TX1/ enabled calibration bias code is applied to TX0, in this case only one TX shall be enabled in the mask to pick the right bias code for TX0. The calibration is not performed for TX0 if this bit is 0. \n Bit[1] is TX1 control during calibration |
     * | Byte[1] | TX channel 1 power calibration TX Enable Mask. \n Bit[0] is TX0 control during calibration \n TX1 control during calibration. If TX1 is disabled then, the TX0/ enabled calibration bias code is applied to TX1, in this case only one TX shall be enabled in the mask to pick the right bias code for TX1. The calibration is not performed for TX1 if this bit is 0.|
     * | Byte[3:2] | Reserved |
     *
     * Below bit fields can be used to control the TXn during the calibration.
     * | Bit Field   | Definition |
     * |--------------  |----------- |
     * | Bit [0] | TX0 selection \n Value 0: Disable \n Value 1: Enable |
     * | Bit [1] | TX1 selection \n Value 0: Disable \n Value 1: Enable |
     * | Bits [7:2] | Reserved |
     *
     * Recommended Default: Byte[0] = 0x03 and Byte[1] = 0x01 \n
     *
     * Example: \n
     * TX0 back-off: 0dB \n TX1 back-off: 0dB \n Enable TX calibration only for TX0 and
     * use same calibrated codes in TX0 and TX1. \n
     * Byte[0] = 0x3 (Enabled both TX in TX0 calibration) \n
     * Byte[1] = 0x1 (Pick TX0 bias code for TX1 and TX1 calibration is disabled)
     *
     */
    UINT8 c_TxPwrCalTxEnaMask[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief Tx Code override values
     * Tx PA stage code override values for each Tx channel.
     * - @b NOTE: This field is valid if TX_CLPC_MODE bit is set to 1
     *
     * | Bytes     | Description        |
     * |-----------|--------------------|
     * | Byte[1:0] | TX_BIAS_CODES[TX0] |
     * | Byte[3:2] | TX_BIAS_CODES[TX1] |
     * | Byte[7:4] | Reserved           |
     *
    */
    UINT16    h_TxStgOverrideCodes[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief Reserved
     */
    UINT32 w_Reserved3[4U];
} T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD;


/*!
 * @brief mmWaveLink @ref rl_fecssRlRuntimeTxClpcCal API response data structure
 * FECSS RF runtime Tx close loop calibration API configuration contains Tx calibration results
 */
typedef struct
{
    /*!
     * @brief Calibration run status
     * FECSS Device TX CLPC calibration run status, 1 bit status per calibration.
     * This is a status of the current calibration command trigger and apply. @n
     *
     * Bit Status Description
     * | Bit Value | Description |
     * |-----------|-------------|
     * | 0         | Failure     |
     * | 1         | Success     |
     *
     * Bitwise status
     * | Bit | Description |
     * |-----|-------------|
     * | 0   | Tx CLPC calibration status |
    */
    UINT8   c_CalRunStatus;

    /*!
     * @brief Calibration temperature
     * Boot calibration temperature. The device temperature at which calibration is executed.
     * - 1LSB = 2 deg C
     * - Range : -127 to 127 (-254 deg C to 254 deg C)
     */
    SINT8   xc_CalibrationTemp;

    /*!
     * @brief  Reserved
     */
    UINT16  h_Reserved0;


    /*!
     * @brief Tx code values
     * Tx PA stage code values for each Tx channel.
     * - @b NOTE: This field is valid if TX_CLPC_MODE bit is set to 1
     *
     * | Bytes     | Description        |
     * |-----------|--------------------|
     * | Byte[1:0] | TX_BIAS_CODES[TX0] |
     * | Byte[3:2] | TX_BIAS_CODES[TX1] |
     * | Byte[7:4] | Reserved           |
     *
    */
    UINT16    h_TxStgCodes[M_DFP_MAX_TX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32  w_Reserved1[3U];
} T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfRuntimeCal API command Data structure. \n
 * FECSS RF runtime temperature compensation calibration configuration command data
 * structure contains the device calibration controls and input configurations.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS Runtime calibration control, 1 bit per channel.  \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF        |
     * | 0x1       | ON         |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]   | RESERVED |
     * | bits [1]   | VCO calibration ON/OFF control. \n Recommendation: \n Periodicity: TBD \n Change in temperature: TBD |
     * | bits [2]   | PD calibration ON/OFF control. \n Enable this calibration only if any PD measurement is done in runtime (TX power and ball break monitors, Tx CLPC calibration). \n Recommendation: \n Change in temperature: TBD |
     * | bits [3]   | LODIST calibration ON/OFF control. \n The new LODIST_BIAS_CODES are applied to HW based on CAL_TEMP_BIN_INDEX. \n Recommendation: \n Change in temperature: TBD |
     * | bits [4]   | RESERVED |
     * | bits [5] | RESERVED |
     * | bits [6] | RX Gain calibration ON/OFF control. \n The new RX_GAIN_CODES are applied to HW based on CAL_TEMP_BIN_INDEX. \n Recommendation: \n Change in temperature: TBD |
     * | bits [7] | TX power OLPC calibration ON/OFF control. \n The new TX_BIAS_CODES are applied to HW based on CAL_TEMP_BIN_INDEX. \n Recommendation: \n Change in temperature: TBD |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalCtrlBitMask;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved0;

    /*!
     * @brief  FECSS RF calibration temperature index, the application shall read
     * the sensor device temperature and provide the bin index during the calibration.  \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | LOW, The Device temperature is < 0deg C (Default Temperature: -20 C)       |
     * | 0x8       | MID, The Device temperature is >= 0deg C and < 85deg C (Default Temperature: 42 C)  |
     * | 0x10      | HIGH, The Device temperature is >= 85deg C  (Default Temperature: 105 C)   |
     *
     */
    UINT8 c_TempBinIndex;

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
    UINT32 w_Reserved3;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved5;

} T_RL_API_FECSS_RF_RUN_CAL_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssRfRuntimeCal API response Data structure. \n
 * FECSS RF runtime calibration result response data structure contains the
 * device calibration run and result validity status.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS Runtime calibration run status, 1 bit per calibration.  \n
     * This is a status of the current calibration command trigger.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | NU / FAIL - Not Updated, Calibration trigger is disabled. / Enabled calibration has failed.   |
     * | 0x1       | PASS - Enabled calibration has Passed.         |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | RESERVED |
     * | bits [1]    | VCO calibration status |
     * | bits [2]    | PD calibration status  |
     * | bits [3]    | LODIST calibration status  |
     * | bits [4]    | RESERVED |
     * | bits [5]    | RESERVED |
     * | bits [6]    | RX Gain calibration status |
     * | bits [7]    | TX power calibration status |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalRunStatus;

    /*!
     * @brief  FECSS RFS run-time calibration result validity status, 1 bit per calibration.  \n
     * This is the latest validity status of all the calibrations. \n
     * The previous validity status is retained if latest calibration trigger fails.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | INVALID - Calibration data is invalid.    |
     * | 0x1       | VALID - Calibration data is valid.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | APLL calibration validity status. \n Note: Updates status of APLL calibration. |
     * | bits [1]    | VCO calibration validity status |
     * | bits [2]    | PD calibration validity status  |
     * | bits [3]    | LODIST calibration validity status  |
     * | bits [4]    | RESERVED |
     * | bits [5]    | RX IFA calibration validity status. \n Note: Updates status of RX IFA calibration. |
     * | bits [6]    | RX Gain calibration validity status |
     * | bits [7]    | TX power calibration validity status |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalResStatus;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

} T_RL_API_FECSS_RF_RUN_CAL_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfClockBwCfg Command Data structure \n
 * FECSS clock Bandwidth configuration command data structure contains
 * the BW controls for APLL and Synth clock modules. \n
 * BW Table: \n
 * | Device   | SYNTH_ICP_TRIM | SYNTH_RZ_TRIM | APLL_ICP_TRIM | APLL_RZ_TRIM | APLL_VCO_RTRIM | SYNTH BW | APLL BW |
 * |---------|--------------- | ------------- |-------------- |------------- |--------------- |-------- |-------- |
 * | xWRL6432 | 0x5  | 0x4 | XTAL_38.4MHz, XTAL_40MHz - 0x1C, XTAL_25MHz, XTAL_26MHz - 0x12 | 0x11 | 0x0C | 1.2MHz | 250kHz |
 * | xWRL1432 | 0x1  | 0x4 | XTAL_38.4MHz, XTAL_40MHz - 0x1C, XTAL_25MHz, XTAL_26MHz - 0x12 | 0x11 | 0x12 | 900Hz | 250kHz |
 *
 * @note Reserved for Future use. This setting is not retained across deep sleep cycle.
 */
typedef struct
{
    /*!
     * @brief  FECSS Device RF SYNTH clock ICP Trim value. \n Note: Refer BW table for
     * BW settings info.
     */
    UINT8 c_SynthIcpTrim;

    /*!
     * @brief  FECSS Device RF SYNTH clock RZ Trim value. \n Note: Refer BW table for
     * BW settings info.
     */
    UINT8 c_SynthRzTrim;

    /*!
     * @brief  FECSS Device RF APLL clock ICP Trim value. \n Note: Refer BW table for
     * BW settings info.
     */
    UINT8 c_ApllIcpTrim;

    /*!
     * @brief  FECSS Device RF APLL clock RZ LPF Trim value. \n Note: Refer BW table for
     * BW settings info.
     */
    UINT8 c_ApllRzTrim;

    /*!
     * @brief  FECSS Device RF APLL clock VCO RTrim value. \n Note: Refer BW table for
     * BW settings info.
     */
    UINT8 c_ApllVcoRtrim;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved3;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved5;

} T_RL_API_FECSS_CLK_BW_CFG_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssGpadcMeasCfg Command Data structure \n
 * FECSS GPADC measurement configuration command data structure contains the channel controls
 * and input configurations. The GPADC measurement will be done on selected enabled channels. \n
 * This is a configuration only API, this configuration is retained in FECSS RAM during deep
 * sleep / warm reset. \n
 * GPADC Signals Config and Param Values:
 * | GPADC Signal  | Param Value | Config Value |
 * |---------------|-------------|--------------|
 * | External GPADC Signal 1 buffered | CHAN_CTRL = 124, NUM_MEAS = 4, SKIP_SAMPLS = 10, \n SIG_TYPE = 1, OFFSET = 0, ENABLE = 1 | 0x00200000 |
 * | External GPADC Signal 1 non-buffered| CHAN_CTRL = 124, NUM_MEAS = 4, SKIP_SAMPLS = 10, \n SIG_TYPE = 2, OFFSET = 0, ENABLE = 1 | 0x00008000 |
 * | External GPADC Signal 2 buffered | CHAN_CTRL = 124, NUM_MEAS = 4, SKIP_SAMPLS = 10, \n SIG_TYPE = 1, OFFSET = 0, ENABLE = 1 | 0x00010000 |
 * | External GPADC Signal 2 non-buffered | CHAN_CTRL = 124, NUM_MEAS = 4, SKIP_SAMPLS = 10, \n SIG_TYPE = 2, OFFSET = 0, ENABLE = 1 | 0x00004000 |
 *
 */
typedef struct
{
    /*!
     * @brief  FECSS GPADC channel param value, 32bit value per channel. \n
     * Max 8 GPADC signal measurements are supported. The param value is a unique
     * value includes, channel control , num of collect and skip samples for
     * internal or external signals to measure the corresponding signal in any GPADC channel.  \n
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [7:0]   | GPADC channel control value, 8bit value per channel.  \n Max 8 GPADC signals supported. \n The control value is a unique parameter value for a internal or external signal can be configured to measure the corresponding signal in any GPADC channel. |
     * | bits [15:8]  | GPADC number of measurement collect samples. \n 1LSB = 1 collect sample, i.e time period of Operating clock frequency / 16 \n Range: 0 to 255 |
     * | bits [22:16]   | GPADC number of skip samples before measurement, 7bit value. \n Total Skip Samples = Mantissa[3:0] * 2^(Exponent[6:4]) \n Bitfield definition \n Bits[3:0] : 1LSB = 1 skip sample (Mantissa) \n Range: 0 to 15 \n Skip samples: 0 to 15 \n Bits[6:4] : 1LSB = 2^N skip samples (Exponent) \n N Range: 0 to 7  \n Skip samples: 1 to 128 \n 1 skip sample period = Time period of Operating clock frequency      |
     * | bits [23]   | Reserved |
     * | bits [26:24]   | GPADC Signal type. 3 bits Value. \n The GPADC measurement signal type shall be appropriately selected to configure the analog and to get accurate measurement.  |
     * | bits [29:27]   | GPADC Buffer offset type index. 3 bits Value. \n The GPADC buffer offset index type shall be appropriately selected to add device process specific offset to the measurement.  |
     * | bits [30]   | Reserved |
     * | bits [31]   | GPADC channel ON/OFF control, 1 bit control.  |
     * The GPADC measurement channel enable control:
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF        |
     * | 0x1       | ON         |
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
     * The GPADC Signal types:
     * | Value  | Definition   |
     * |  ----  |  ---------------------- |
     * | 0      |  INTERNAL_BUFFERED_SIG_TYPE   |
     * | 1      |  EXTERNAL_BUFFERED_SIG_TYPE   |
     * | 2      |  EXTERNAL_NON_BUFFERED_SIG_TYPE     |
     * | 3      |  TEMP_SENS_DIFFERENTIAL_SIG_TYPE       |
     * | 4      |  TEMP_SENS_SINGLE_ENDED_SIG_TYPE       |
     * | 7-5    |  RESERVED      |
     * The GPADC Buffer offset index types:
     * | Value  | Definition   |
     * |  ----  |  ---------------------- |
     * | 0      |  NO_OFFSET (Default)   |
     * | 1      |  RX_OFFSET_TYPE   |
     * | 2      |  TX_OFFSET_TYPE     |
     * | 3      |  CLK_OFFSET_TYPE       |
     * | 4      |  PM_OFFSET_TYPE       |
     * | 7-5    |  RESERVED      |
     */
    UINT32 w_ParamVal[M_RL_FECSS_MAX_GPADC_CHANNELS];

   /*!
     * @brief  FECSS GPADC channel config value, 32bit value per channel.  \n
     * Max 8 GPADC signals supported. \n
     * The config value is a unique value for a internal or external signal can be configured
     * to measure the corresponding signal in any GPADC channel.
     */
    UINT32 w_ConfigVal[M_RL_FECSS_MAX_GPADC_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_GPADC_MEAS_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssGpadcMeasTrig Response Data structure \n
 * The command API does not have any input data. FECSS GPADC result response data structure
 * contains the measured GPADC values for all programmed signals.
 */
typedef struct
{
    /*!
     * @brief  FECSS GPADC channel Status, 1 bit per channel.  \n
     * Max 8 GPADC signal channels supproted. This is a status of the current GPADC
     * measurement command measurement trigger. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | NU / FAIL - Not Updated, GPADC channel is disabled. / Enabled GPADC channel has failed.   |
     * | 0x1       | PASS - Enabled GPADC channel has Passed.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | GPADC channel 0 status |
     * | bits [1]    | GPADC channel 1 status |
     * | bits [2]    | GPADC channel 2 status |
     * | bits [3]    | GPADC channel 3 status |
     * | bits [4]    | GPADC channel 4 status |
     * | bits [5]    | GPADC channel 5 status |
     * | bits [6]    | GPADC channel 6 status |
     * | bits [7]    | GPADC channel 7 status |
     */
    UINT8 c_GpadcChRunStatus;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  FECSS GPADC Measurement Avg Value, 1 LSB = 1.8V / 256
     * Max 8 GPADC channels supported
     */
    UINT8 c_GpadcMeasVal[M_RL_FECSS_MAX_GPADC_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved4;

} T_RL_API_FECSS_GPADC_MEAS_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssTempMeasCfg command Data structure \n
 * FECSS temperature sensor measurement configuration command data structure contains the sensor
 * enable controls. The temperature  measurement will be done on selected enabled sensors. \n
 * This is a configuration only API, this configuration is retained in FECSS RAM during
 * deep sleep / warm reset.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS Temperature measure control, 1 bit per channel. \n
     * Max 4 temperature sensors supported in this device. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF        |
     * | 0x1       | ON         |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------  |----------- |
     * | bits [0]       | RX temperature sensor ON/OFF control |
     * | bits [1]       | Reserved |
     * | bits [2]       | Reserved |
     * | bits [3]       | Reserved |
     * | bits [4]       | TX temperature sensor ON/OFF control |
     * | bits [5]       | Reserved |
     * | bits [6]       | Reserved |
     * | bits [7]       | Reserved |
     * | bits [8]       | PM temperature sensor ON/OFF control |
     * | bits [9]       | DIG temperature sensor ON/OFF control |
     * | bits [15:10]   | Reserved |
     */
    UINT16 h_TempCtrlBitMask;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_TEMP_MEAS_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssTempMeasTrig response Data structure \n
 * The command API does not have any input data. FECSS Temperature sensors result
 * response data structure contains the measured temperature values for all programmed channels.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS temperature measurement Status, 2 bit per channel.  \n
     * Max 4 temperature sensors supported in this device. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | NU / FAIL - Not Updated, Temperature sensor meas is disabled. / Enabled Temperature sensor meas has failed.    |
     * | 0x1       | PASS - Enabled Temperature sensor meas has Passed.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]     | RX temperature sensor status |
     * | bits [1]     | Reserved |
     * | bits [2]     | Reserved |
     * | bits [3]     | Reserved |
     * | bits [4]     | TX temperature sensor status |
     * | bits [5]     | Reserved |
     * | bits [6]     | Reserved |
     * | bits [7]     | Reserved |
     * | bits [8]     | PM temperature sensor status |
     * | bits [9]     | DIG temperature sensor status |
     * | bits [15:10] | Reserved |
     */
    UINT16 h_TempStsBitMask;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved1;

    /*!
     * @brief  FECSS RFS Temperature measured value, 1LSB = 1deg C signed.
     */
    SINT16 xh_TempValue[M_RL_FECSS_MAX_TEMP_SENSORS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_TEMP_MEAS_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfRxTxCalDataGet and @ref rl_fecssRfRxTxCalDataSet
 * command / response Data structure \n
 * This API is used to store the RX-TX factory calibration data to flash or external memory if
 * calibration is done in the factory, same API can be used to dynamically update the RX and TX
 * gain and bias codes to support multiple chirp profiles. \n
 * Use @ref rl_fecssRfRxTxCalDataGet and @ref rl_fecssRfRxTxCalDataSet API to update RX and
 * TX codes to support dynamic update of multiple chirp profiles in inter-frame time. \n
 * API can be used to store the RX and TX TI factory calibration data in external flash to avoid
 * running RX and TX calibration in each cold boot. It is recommended to run run-time calibration
 * RL_FECSS_RF_RUNTIME_CAL after this restore.
 */
typedef struct
{
   /*!
     * @brief  FECSS RFS RX-TX calibration result validity status, 1 bit per calibration.  \n
     * This is the latest validity status of all the calibrations. \n
     * The previous validity status is retained if latest calibration trigger fails.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | INVALID - Calibration data is invalid.    |
     * | 0x1       | VALID - Calibration data is valid.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | RX Gain calibration validity status |
     * | bits [1]    | TX power calibration validity status |
     * | bits [3:2] | Reserved |
     * | bits [4]    | RX Gain calibration validity status redundancy |
     * | bits [5]    | TX power calibration validity status redundancy |
     * | bits [7:6] | Reserved |
     */
    UINT8 c_CalRxTxResValidity;

   /*!
     * @brief  FECSS RFS RX calibrated codes for @ref c_CalRxGainSel. \n
     * RX_GAIN_CODES LUT contains calibrated RF_GAIN_CODE and IFA_GAIN_CODE for programmed
     * CAL_RX_GAIN_SEL. The computed RX_GAIN_CODES for three temperature bins are stored in this
     * structure. \n
     * | Byte Field  | Definition |
     * |---------    |----------- |
     * | Byte[0]     | RX_GAIN_CODES[LOW_TEMP_BIN]. \n RX gain code for low temperature Bin (< 0deg C).  |
     * | Byte[1]     | RX_GAIN_CODES[MID_TEMP_BIN]. \n RX gain code for Mid temperature Bin (>= 0deg C and < 85deg C).  |
     * | Byte[2]     | RX_GAIN_CODES[HIGH_TEMP_BIN]. \n RX gain code for High temperature Bin (>= 85deg C).  |
     * RX_GAIN_CODE (1 Byte) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [4:0]  | IFA_GAIN_CODE. \n Calibrated IFA gain code for given CAL_RX_GAIN_SEL.
Valid Rage: 0 to 14. \n IFA_GAIN (dB) = IFA_GAIN_CODE * 2 - 10dB |
     * | bits [7:5]  | RF_GAIN_CODE. \n Calibrated RF gain code . \n Valid Rage: 0 to 3. |
     */
    UINT8 c_CalRxGainCodes[M_RL_FECSS_CAL_MAX_TEMP_BINS];

    /*!
     * @brief Calibrated CAL_TX_BACK_OFF_SEL mapping for TX_BIAS_CODES. This back-off information
     * is needed by FECSS RFS FW. \n
     * The FEC analog is calibrated to this selected back-off setting and TX_BIAS_CODES are
     * computed for three temperature bins. \n
     * Byte definition: \n
     * | Byte Field   | Definition |
     * |------------  |----------- |
     * | 0            | TX0 output power back-off value  |
     * | 1            | TX1 output power back-off value  |
     * | Byte [3:2]   | Reserved  |
     * 1LSB: 0.5dB \n
     * Valid Range: even values (1dB step) from 0 (0dB) to 52 (26dB) \n
     */
    UINT8 c_CalTxBackOffMap[M_DFP_MAX_TX_CHANNELS];

   /*!
     * @brief  FECSS RFS TX calibrated codes for @ref h_CalTxBiasCodes. \n
     * TX_BIAS_CODES LUT contains calibrated TX_BIAS_CODES for programmed CAL_TX_BACK_OFF_SEL.
     * The computed TX_BIAS_CODES for three temperature bins are stored in this structure for
     * each TX. \n
     * | Byte Field  | Definition |
     * |---------    |----------- |
     * | Byte[1:0]     | TX_BIAS_CODES[TX0][LOW_TEMP_BIN]. TX0 Bias code for low temperature Bin (< 0deg C).  |
     * | Byte[3:2]     | TX_BIAS_CODES[TX0][MID_TEMP_BIN]. TX0 Bias code for mid temperature Bin (>= 0deg C and < 85deg C).  |
     * | Byte[5:4]     | TX_BIAS_CODES[TX0][HIGH_TEMP_BIN]. TX0 Bias code for high temperature Bin (>= 85deg C).  |
     * | Byte[7:6]     | TX_BIAS_CODES[TX1][LOW_TEMP_BIN]. TX1 Bias code for low temperature Bin (< 0deg C).  |
     * | Byte[9:8]     | TX_BIAS_CODES[TX1][MID_TEMP_BIN]. TX1 Bias code for mid temperature Bin (>= 0deg C and < 85deg C).  |
     * | Byte[11:10]     | TX_BIAS_CODES[TX1][HIGH_TEMP_BIN]. TX1 Bias code for high temperature Bin (>= 85deg C).  |
     * | Byte[23:12]     | Reserved  |
     * TX_BIAS_CODE (2 bytes) definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | Byte [0]  | TX_BIAS_CODE_1.  Calibrated TX Bias code 1 for given CAL_TX_BACK_OFF_SEL.  |
     * | Byte [1]  | TX_BIAS_CODE_2. Calibrated TX Bias code 2 for given CAL_TX_BACK_OFF_SEL.  |
     */
    UINT16 h_CalTxBiasCodes[M_DFP_MAX_TX_CHANNELS][M_RL_FECSS_CAL_MAX_TEMP_BINS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved;

} T_RL_API_FECSS_RXTX_CAL_DATA;

/*!
 * @brief mmWaveLink @ref rl_fecssRfFactoryCalDataGet and @ref rl_fecssRfFactoryCalDataSet
 * command / response Data structure \n
 * This API is used to store the calibration data to flash or external memory. This is an optional
 * API can be used only in advance use case, for normal operation please use
 * @ref rl_fecssRfRxTxCalDataGet API to store the RX and TX calibration data.
 * Optional API can be used to store the TI factory calibration data in external flash during
 * early evaluation, same API can be used by user if user wants to power off all FEC memory in
 * successive boots (repeated cold boot) and restore the latest calibration data to avoid the
 * rerun of all the factory and runtime calibrations including APLL.
 */
typedef struct
{

   /*!
     * @brief  FECSS RFS factory calibration result validity status, 1 bit per calibration.  \n
     * This is the latest validity status of all the calibrations. \n
     * The previous validity status is retained if latest calibration trigger fails.
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | INVALID - Calibration data is invalid.    |
     * | 0x1       | VALID - Calibration data is valid.        |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | APLL calibration validity status |
     * | bits [1]    | VCO calibration validity status |
     * | bits [2]    | PD calibration validity status  |
     * | bits [3]    | RESERVED  |
     * | bits [4]    | RESERVED |
     * | bits [5]    | RX IFA calibration validity status |
     * | bits [6]    | RX gain calibration validity status |
     * | bits [7]    | TX power calibration validity status |
     * | bits [15:8] | Reserved |
     */
    UINT16 h_CalResValidity;

   /*!
     * @brief  FECSS RFS factory calibration result validity status redundancy.
     * Copy of h_CalResValidity.
     */
    UINT16 h_CalResValRed;

   /*!
     * @brief  FECSS RFS APLL calibrated cap codes.
     */
    UINT32 w_ApllCapCodes;

   /*!
     * @brief  FECSS RFS SYNTH calibrated cap codes.
     */
    UINT16 h_SynthCapCodes;

   /*!
     * @brief  FECSS RFS IFA calibrated trim codes.
     */
    UINT16 h_IfaTrimCodes;

   /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved0;

   /*!
     * @brief  FECSS RFS PD factory cal temperature.
     */
    UINT16 h_PdFactCalTemp;

   /*!
     * @brief  FECSS RFS PD calibrated codes. Total 76 bytes
     */
    UINT8 c_PdCalCodes[76U];

   /*!
     * @brief  FECSS RFS RX-TX calibration data. Total 36 bytes
     */
    T_RL_API_FECSS_RXTX_CAL_DATA z_RxTxCalData;

} T_RL_API_FECSS_FACT_CAL_DATA;

/*!
 * @brief mmWaveLink @ref rl_fecssDevClockCtrl command Data structure \n
 * FECSS clock control command data structure contains the device Clock source selection setting.
 * The Application can switch the FECSS clock source using this API based on device low
 * power state.
 */
typedef struct
{
    /*!
     * @brief  FECSS run time clock source selection type. \n
     * The application can switch the clock source of FECSS based on low power state to
     * XTAL clock or Fast clock (ADPLL / APLL 100MHz) or can gate the FECSS system clock. \n
     * This API configures FECSS clock based on this setting.   \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | XTAL clock source, application can switch the XTAL clock (XTAL_FREQ) in short idle period when FECSS is in idle state. |
     * | 0x0A  | FClock clock source, Fast clock can be either (DIG_PLL_FREQ/2) ~80MHz clock set by application |
     * | 0x05  | Clock gate (OFF), the FECSS clock can be gated using this option. The application can gate FECSS clock in inter-frame idle time |
     * Typical Value: 0x00 (Short Idle time / RFS Boot), 0x0A (Data Acquisition Mode / Monitoring)
     * and 0x05 (Long Frame Idle time)
     */
    UINT8 c_DevClkCtrl;

    /*!
     * @brief FECSS frame timer (FT) clock gate option. The frame timer always runs on XTAL clock.
     * The application can gate the FT clock source based on low power state. \n
     * This API configures FECSS clock based on this setting. \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  |XTAL Clock source. Default operating frequency. \n Clock Frequency: 25MHz, 26MHz, 38.4MHz or 40MHz.  |
     * | 0x05 | Clock gate (OFF), the FECSS Frame Timer clock can be gated using this option. \n The application can gate FT clock in idle state and FT is not active.  |
     * Typical Value: 0x00 (Normal operating mode) and 0x05 (Idle time)
     */
    UINT8 c_FtClkCtrl;

    /*!
     * @brief  FECSS RF APLL clock block ON / OFF control. \n
     *
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | Turn OFF APLL clock   \n Turn OFF APLL before entering deep sleep or APLL functionality is not needed by HOST. \n Typical use case : After Data processing of a frame.  |
     * | 0x0A  | Turn ON APLL clock \n Use this ON mode in typical APLL ON after deep sleep exit or whenever APLL functionality is needed by HOST. \n Typical use case: After deep sleep exit or before Analog / sensor functionalities turn ON |
     * | 0xAA  | Calibrate and Turn ON APLL clock \n This ON mode shall be used in first time device boot and whenever APLL block needs calibrations. \n Recommendation: TBD \n This API return error code if calibration fails. |
     * The application shall enable / disable the APLL as per recommended sequence
     * for various functionalities. The APLL shall be ON before
     * - Calibration trigger API
     * - Functional frames
     * - Monitor trigger API
     * - CW mode trigger
     * - Test Burst trigger
     * - RDIF Data transfer
     * - HW accelerator processing (optional)
     */
    UINT8 c_ApllClkCtrl;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_DEV_CLK_CTRL_CMD;

/*!
 * @brief mmWaveLink @ref rl_fecssDevRdifCtrl command Data structure \n
 * FECSS device RDIF control command data structure contains the RDIF block configuration and
 * settings and enable / disable control. \n
 * The Application can use this API to collect the radar Data over CMOS interface pins using
 * this API. \n
 * The Max 3 channels supported in RDIF, they mapped to max 3 RX channels of the sensor,
 * it is possible to capture any one or two or all three of the DFE RX channels output.
 * The RX_CHAN_CTRL_MASK config in RL_FECSS_RF_PWR_ONOFF API is used to program the DFE
 * RX channel enable which is mapped to RDIF channels. If less than 3 channels are
 * enabled in DFE then only enabled channels will be mapped to lower RDIF channels
 * without any holes. \n
 * The max bits per sample supported is only 12 bits in RDIF, the number of bits and
 * samples per chirp and per channel shall match the DFE profile configuration. \n
 * The DDR clock is 50MHz (APLL_FREQ/8) and data is sampled at both raising and falling edge of
 * DDR clock. The max data rate of RDIF is 50M * 2 * 4 Data lines = 400Mbps \n
 * The one frame clock duration is 12 DDR clock edges corresponds to 48bits. \n
 * The below Figure shows the RDIF data format and data lane mapping. \n
 * TBD add figures.. \n \n
 * Sideband Data Format: \n
 * The 96 bits (8 samples) sideband data can be appended end of each chirp data
 * (API enable). The side band data consists of the following informations: \n
 * FRAME_CNT[11:0] - 12bits, 1st sample \n
 * BURST_CNT[11:0] - 12bits, 2nd sample \n
 * CHIRP_CNT[11:0] - 12bits, 3rd sample \n
 * SATURATION_CNT[35:0] - 12b per channel, there are three fixed slots named
 * SATURATION_CNT_CH3 (sample 4), SATURATION_CNT_CH2 (sample 5) and
 * SATURATION_CNT_CH1 (sample 6) - each 12b samples. In case of less number of
 * enabled DFE channels, lower slots will have valid saturation count and upper slot will be
 * padded with zeros. \n
 * CRC-16[23:0] - 16 bits CRC [15:0], MSB 8 bits are padded with zeros.
 * (CRC is calculated for Chirp data + Sideband data) - sample 7 (LSB) and 8 (MSB) \n \n
 * CW Mode: \n
 * In CW mode the DFE output samples will be sent out continuously for all the
 * enabled channels. The max data rate supported is 400Mbps, in case of max 3
 * enabled RX channels, the max DFE rate can be supported is 11.11Mbps. \n
 * The Sideband data is not supported in CW mode. \n
 * The below figure shows data format for various channel configurations in CW mode. \n
 * TBD add figure .. \n \n
 * Data Swizzling Mode: \n
 * The RDIF channel data and the side band sample 12 bits data can be swizzled as
 * per below configuration using RDIF_CONFIG API parameter. \n
 * TBD add figure..
 *
 */
typedef struct
{
    /*!
     * @brief  FECSS RDIF block ON / OFF control. This control shall be used to reset and
     * enable or disable the RDIF data transfer.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | RDIF block is turned OFF    |
     * | 0x0A  | RDIF block is turned ON |
     * The below configurations applicable in ON mode. \n
     * RDIF IP is always goes through reset in every ON command.
     */
    UINT8 c_RdifEnable;

    /*!
     * @brief FECSS RDIF configuration settings can be used to enable various modes of operation
     * of the RDIF IP. \n
     * This API configures RDIF based on below settings.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x0  |Disabled  |
     * | 0x1 | Enabled  |
     * Bit Definition:
     * | Bit Fields   | Definition |
     * |----------- |-----------   |
     * | Bit[0]     | RDIF Sideband data enable control. \n The 96 bits (8 samples) sideband data will be appended end of each chirp data when this mode is enabled.     |
     * | Bit[1]     | RDIF CW Mode enable control. \n In CW mode the DFE output samples will be sent out continuously for all the enabled channels at max rate 400Mbps. \n The Sideband data is not supported in CW mode.      |
     * | Bits[3:2]  | RDIF Data Swizzling Mode enable control. \n The RDIF channel data and the side band sample 12 bits data swizzling mode.  |
     * | Bit[4]     | RDIF Scrambler Mode enable control. \n Scrambler Polynomial: $1 = z^{-18} + z^{-23}$ |
     * | Bit[5]     | Enable RDIF lane rate update. \n By default per lane data rate is set to max 100Mbps. Max 400Mbs for 4 lanes. \n The lane rate can be updated if this bit is SET using RDIF_LANE_RATE. |
     * | Bits[7:6]  | Reserved |
     * RDIF Data Swizzling Mode control: \n
     * | Value   | Definition            |
     * |-------- |--------------------   |
     * | b00     | Pin0-bit0-Cycle1 Mode |
     * | b01     | Pin3-bit0-Cycle1 Mode |
     * | b10     | Pin0-bit0-Cycle3 Mode |
     * | b11     | Pin3-bit0-Cycle3 Mode |
     *
     * @note: Rx saturation monitor must be enabled to stream sideband data
     */
    UINT8 c_RdifCfg;

    /*!
     * @brief  FECSS RDIF Sample count. FECSS RDIF number of samples per channel per chirp.
     * This field should be exactly same as the NUM_ADC_SAMPLES configuration in
     * RL_SENSOR_CHIRP_PROFILE_COMM_CFG API. \n
     * The config value should be a multiple of 4. This field does not include samples from
     * sideband data. \n
     * Note: Variable chirp size is not supported. i.e. All the chirps will have the same size.
     * It is not possible to send 512 samples in first chirp and 1024 samples in next chirp. \n
     * Valid Range: 4 to 2048 \n
     */
    UINT16 h_RdifSampleCount;

    /*!
     * @brief FECSS Test Pattern block ON / OFF control. This control shall be used to enable or
     * disable the Test pattern in DFE. This test pattern can be used to validate the RDIF and
     * Data path buffer.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x0  |  TEST_PATTERN block is turned OFF  |
     * | 0xA |  TEST_PATTERN block is turned ON  |
     */
    UINT8 c_TestPatternEn;

    /*!
     * @brief  FECSS RDIF lane rate. The max lane rate supported 400Mbps (for 4 lanes,
     * each lane max 100Mbps).
     * | Value   | Combined Lane Rate |
     * |---------|----------- |
     * | 0       |  400Mbps  |
     * | 1       |  320Mbps  |
     * | 2       |  200Mbps  |
     * | 3       |  160Mbps  |
     * Note: This configuration will be updated only if c_RdifCfg[bit 5] is SET.
     * By default Lane rate is set to 400Mbps.
     * @[
     * RDIFLaneRate > (#RxChannels - (1152/#ADCSamples)) * 12-bits * ADCSamplingRate
     * @]
     *
     */
    UINT8 c_LaneRateCfg;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

    /*!
     * @brief  RX channel [3:0] Test pattern init code. \n The offset value code to be
     * used for the first sample for the test pattern data. \n
     * RX3 is reserved.
     */
    UINT16 h_TestPatrnInitCode[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief  RX channel [3:0] Test pattern increment code. \n The value to be added
     * for each successive sample for the test pattern data. \n
     * RX3 is reserved.
     */
    UINT16 h_TestPatrnIncrCode[M_DFP_MAX_RX_CHANNELS];

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved5;

} T_RL_API_FECSS_RDIF_CTRL_CMD;


/*!
 * @brief mmWaveLink  @ref rl_fecssDevStatusGet response Data structure \n
 * FECSS Device status response data structure contains the FECSS power and clock status  \n
 */
typedef struct
{
    /*!
     * @brief  Device XTAL Clock frequency used by FECSS if DEV_CLK_SRC_STATE = 0x00.
     * This value shall match the application programmed value.  \n
     * Unit - 1LSB = 1/256 MHz \n
     * Typical values are: 6400, 6656, 9830 and 10240 \n
     */
    UINT16 h_XtalClkFreq;

    /*!
     * @brief  FECSS Clock source state. The Valid configurations are as below, the value 0x05
     * is invalid as when clock is gated application can not read this status.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | XTAL Clock source \n Clock Frequency: 25MHz, 26MHz, 38.4MHz or 40MHz (XTAL_FREQ)   |
     * | 0x0A  | Fast Clock source \n Clock Frequency: 80MHz (DIG_PLL_FREQ/2).|
     */
    UINT8 c_DevClkSrcState;

    /*!
     * @brief  FECSS power up mode type.   \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | Cold Boot Mode. If POWER_MODE is 0x00 then, FECSS powered up state is assumed to be cold boot and the RFS data memory (.data, .bss section) is re-initialized during powerup. |
     * | 0x0A  |Warm Boot Mode.  If POWER_MODE is 0x0A then, FECSS powered up state is assumed to be warm boot and the RFS data memory (.data, .bss section) is retained, un-initialized during power up. \n In last power down state, the FECSS memory state is retained.  |
     */
    UINT8 c_PowerModeState;

    /*!
     * @brief  The Chirp timer module timing engine resolution type. The RF start
     * frequency resolution and timing related chirp parameters resolution type selected. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | Low Resolution Mode   |
     * | 0x1       | High Resolution Mode  |
     * Bit Definition: \n
     * | Bit Field | Definition |
     * |-----------|----------- |
     * | 0   | Resolution type of CHIRP_RF_FREQ_START. \n The Low Resolution Mode supports 16 bit CHIRP_RF_FREQ_START value and High Resolution Mode supports 24bit  CHIRP_RF_FREQ_START value. \n This resolution setting is applicable for RF_FREQ_START parameter in sensor PER_CHIRP_CFG and PROFILE_TIME_CFG APIs. |
     * | 1   | Resolution type of Timing Related chirp parameters. \n The Low Resolution Mode supports 1 LSB = 40/ APLL_FREQ (Typical 100ns with 40MHz XTAL) and High Resolution Mode supports 1 LSB = 48/ APLL_FREQ (Typical 20ns with 40MHz XTAL) \n This resolution setting is applicable for IDLE_TIME and RAMP_END_TIME parameters in sensor PROFILE_TIME_CFG API, BURST_PERIODICITY parameter in FRAME_CFG API and IDLE_TIME parameter in PER_CHIRP_CFG API. |
     */
    UINT8 c_ChirpTimerResType;

    /*!
     * @brief FECSS Frame timer Clock source type. The Valid configurations are as below. \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  |XTAL Clock source. Default operating frequency. \n Clock Frequency: 25MHz, 26MHz, 38.4MHz or 40MHz.  |
     * | 0x05 | Clock gate (OFF), the FECSS Frame Timer clock is gated.  |
     * Typical Value: 0x00 (Normal operating mode) and 0x05 (Idle time)
     */
    UINT8 c_FtClkState;

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
     * | Bit[1]  | RFS BIST FFT IP self test status  |
     * | Bit[2]  | RFS GPADC IP self test status  |
     * | Bit[7:3]  | Reserved  |
     */
    UINT8 c_RfsBootSelfTest;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  RFS FW State.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x0000  | Halt    |
     * | 0x4004  | Booting |
     * | 0x8008  | Idle    |
     * | 0xFFFF  | Fault   |
     * | Cmd Id  | API Execution |
     */
    UINT16 h_RfsFwState;

    /*!
     * @brief  RFS Patch Status in Lower nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | Disabled |
     * | 0xA     | Enabled |
     *         RFS SIL Status in Upper nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | Disabled |
     * | 0xA     | Enabled |
     */
    UINT8 c_PatchAndSilEna;

    /*!
     * @brief  RFS RF Type in Lower nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | 60GHz |
     * | 0xA     | 77GHz |
     *         RFS Sensor Type in Upper nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | SOC |
     * | 0xA     | FE |
     */
    UINT8 c_RfAndSensType;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved3;

} T_RL_API_FECSS_DEV_STS_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfStatusGet API response Data structure. \n
 * FECSS RF channels and IP modules power state data structure.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS RX channels control status, 1 bit per channel. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF State |
     * | 0x1       | Functional ON state      |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]    | Rx Channel 0 |
     * | bits [1]    | Rx Channel 1 |
     * | bits [2]    | Rx Channel 2 |
     * | bits [15:3] | Reserved |
     *
     */
    UINT16 h_RxChCtrlSts;

    /*!
     * @brief  FECSS RFS TX channels control status, 1 bit per channel. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | OFF State |
     * | 0x1       | Functional ON state      |
     * Bit field definition:
     * | Bit Field   | Definition |
     * |---------    |----------- |
     * | bits [0]  | Tx Channel 0 |
     * | bits [1]  | Tx Channel 1 |
     * | bits [15:2] | Reserved |
     *
     */
    UINT16 h_TxChCtrlSts;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved0;

    /*!
     * @brief  FECSS RFS FECSS Miscellaneous block / clock control status, 1 bit per control. \n
     * | Value     | Definition |
     * |---------  |----------- |
     * | 0x0       | Block Disable |
     * | 0x1       | Block Enable  |
     * Bit field definition:
     * | Bit Field     | Definition  |
     * |---------      |-----------  |
     * | bits [0]    | RDIF clock enable control status. \n The RDIF IP block clock shall be enabled using this control before configuring and enabling the RDIF. \n This clock is derived from APLL clock, APLL shall be ON while enabling this clock.  |
     * | Bit [1]     | RF ANA 1V LDO bypass control. \n By default the internal LDO is used to drive 1V supply. \n To bypass the internal LDO, SET this bit. \n Note: External 1V supply shall be connected in case of bypass |
     * | bits [7:2]  | Reserved    |
     *
     */
    UINT8 c_MiscCtrlSts;

    /*!
     * @brief  FECSS RF APLL clock block ON / OFF status. \n
     *
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x00  | APLL block is turned OFF state  |
     * | 0x0A  | APLL block is turned ON state |
     */
    UINT8 c_ApllClkCtrlSts;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved2;

} T_RL_API_FECSS_RF_STS_GET_RSP;

/*!
 * @brief mmWaveLink  @ref rl_fecssRfFaultStatusGet response Data structure \n
 * FECSS RFS CM3 fault status response data structure contains the CPU Firmware status. \n
 * Application shall read the fault status of the FECSS device when FECSS_RFS_FAULT interrupt is
 * generated at FEC_INTR1 (IRQ[3]) in application M4 core.
 */
typedef struct
{
    /*!
     * @brief  RFS FW State.  \n
     * | Value   | Definition |
     * |---------|----------- |
     * | 0x0000  | Halt    |
     * | 0x4004  | Booting |
     * | 0x8008  | Idle    |
     * | 0xFFFF  | Fault   |
     * | Cmd Id  | API Execution |
     */
    UINT16 h_RfsFwState;

    /*!
     * @brief  RFS Patch Status in Lower nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | Disabled |
     * | 0xA     | Enabled |
     *         RFS SIL Status in Upper nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | Disabled |
     * | 0xA     | Enabled |
     */
    UINT8 c_PatchAndSilEna;

    /*!
     * @brief  RFS RF Type in Lower nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | 60GHz |
     * | 0xA     | 77GHz |
     *         RFS Sensor Type in Upper nibble: \n
     * | Value   | Definition |
     * |---------|--------|
     * | 0x0     | SOC |
     * | 0xA     | FE |
     */
    UINT8 c_RfAndSensType;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

    /*!
     * @brief  RFS CPU Fault Type. \n
     * TBD update
     * | Value | Fault Type |
     * |------ |------------|
     * | 0x00  | No fault   |
     * | 0x81  | CPU Fault  |
     * | 0x82  | FW ASSERT  |
     *
     */
    UINT8 c_FaultType;

    /*!
     * @brief  RFS FW Fault Error codes. \n
     * TBD update
     * | Id   | Error Code |
     * |------| -----------|
     * | 0x00 | No Fault   |
     * | 0x81 | Stray MBOX Interrupt Error in FEC  |
     * | 0x82 | FEC Register Readback Error    |
     * | 0x83 | Functional Frame trigerred in CalMon duration  |
     * | 0x84 | Invalid data |
     * | 0x85 | Array index out of bound |
     * | 0x86 | Attempted to dereferecne null pointer |
     * | 0x87 | Resource busy |
     * | 0x88 | Timeout error |
     * | 0x89 | Stack overflow |
     * | 0x8A | Undefined error |
     *
     */
    UINT8 c_ErrorCode;

    /*!
     * @brief  RFS Fault line num
     */
    UINT16 h_LineNum;

    /*!
     * @brief RFS PC Value where fault occurred
     */
    UINT32 w_AbortPC;

    /*!
     * @brief RFS Fault calling function LR
     */
    UINT32 w_AbortLR;

    /*!
     * @brief  RFS Fault APSR
     */
    UINT32 w_AbortApsr;

    /*!
     * @brief  RFS Fault SP
     */
    UINT32 w_AbortSp;

    /*!
     * @brief  RFS Fault CFSR register content
     */
    UINT32 w_CfsrReg;

    /*!
     * @brief  RFS Fault HFSR register content
     */
    UINT32 w_HfsrReg;

    /*!
     * @brief  RFS Fault MemManage FAR register content
     */
    UINT32 w_MmFarReg;

    /*!
     * @brief  RFS Fault Bus FAR register content
     */
    UINT32 w_BFarReg;

    /*!
     * @brief  RFS SHCSR exception status register content
     */
    UINT32 w_ShcsrReg;

    /*!
     * @brief  RFS Exception count
     */
    UINT8 c_ExceptionCount;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved2;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved3;

} T_RL_API_RFS_FAULT_STS_GET_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssDieIdGet Response Data structure \n
 * FECSS sensor device DIE ID HEX value data structure.
 */
typedef struct
{
    /*!
     * @brief  Sensor Die ID Data Array.
     */
    UINT32     w_DieIdData[4U];

    /*!
     * @brief  Reserved.
     */
    UINT32     Reserved[4U];

} T_RL_API_SENSOR_DIEID_RSP;

/*!
 * @brief mmWaveLink @ref rl_fecssRfsDbgCfg Command Data structure \n
 * This API shall be used to configure FECSS RFS debug memory address, this is a optional
 * API can be used to log the RFS internal information.
 */
typedef struct
{
    /*!
     * @brief  FECSS RFS Debug log address.
     */
    UINT32 w_RfsDbgLogAddress;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved0;

    /*!
     * @brief  Reserved
     */
    UINT32 w_Reserved1;

} T_RL_API_FECSS_RFS_DBG_CFG_CMD;

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
 * @name mmWaveLink Library Device module API function calls
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
 * END OF dfp_device.h
 */


