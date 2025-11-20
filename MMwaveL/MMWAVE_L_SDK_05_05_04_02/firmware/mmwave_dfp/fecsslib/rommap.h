/*!*****************************************************************************
 * @file rommap.h
 *
 * @brief FECSSLib driver API functions ROM/PATCH mapping header file
 *
 * @b Description @n
 * This is a FECSSLib driver API functions ROM/PATCH mapping header file, provides function call
 * mapping info either to ROM or PATCH (RAM) functions. This mapping would help to build library
 * for ROM or PATCH.
 *
 * The FECSS drivers are open source software, TI is providing option either to use ROM/patch
 * library driver functions or RAM functions in library generation build options.
 *
 * - # RAM Lib build : The RAM or pre ROM build library (fecss_rom_m4.lib) maps all driver
 * function calls to '_rom' functions in RAM, application shall map these functions
 * to RAM area in linker.
 * - # ROM Lib build    : The ROM build library (fecss_tirom_m4.lib) maps all driver function
 * calls to '_rom' functions in ROM, application shall map these functions to ROM
 * area in linker.
 * - # Patch Lib build  : The PATCH build library (fecss_patch_m4.lib) maps all patched API
 * function calls to '_patch' functions in RAM, application shall map these functions
 * to RAM area in linker on top of ROM functions from fecss_tirom_m4.lib.
 *
 * @warning In ES1.0 mmwave silicon, the FECSSLib ROM is not available and user has to link
 * 'fecss_rom_m4.lib' static RAM library by default to RAM area. Refer \ref fecsslib_build_sec
 * for more details on ROM and patch library build options.
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
 * @addtogroup FECSSLIB_ROM_MAP FECSSLib ROM/PATCH functions mapping
 * @{
 *
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
#ifndef FECSS_ROMTABLE_H
#define FECSS_ROMTABLE_H

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
 * @name FECSS Library Device Module driver API ROM / PATCH functions
 * @{
 */
#define fe_fecssLibInit             fe_fecssLibInit_rom
#define fe_fecssLibDeInit           fe_fecssLibDeInit_rom
#define fe_fecssDevPwrOn            fe_fecssDevPwrOn_rom
#define fe_fecssDevPwrOff           fe_fecssDevPwrOff_rom
#define fe_fecssDevClockCtrl        fe_fecssDevClockCtrl_rom
#define fe_fecssDevClockCtrlCheck   fe_fecssDevClockCtrlCheck_rom
#define fe_fecssDevStatusGet        fe_fecssDevStatusGet_patch
#define fe_fecssDieIdGet            fe_fecssDieIdGet_rom
#define fe_fecssRfsFaultStatusGet   fe_fecssRfsFaultStatusGet_rom

/*! @} */

/*!
 * @name FECSS Library Sensor Module low level drivers ROM functions
 * @{
 */
#define fe_sensChirpProfComnCfg     fe_sensChirpProfComnCfg_rom
#define fe_sensChirpProfComnCfgGet  fe_sensChirpProfComnCfgGet_rom
#define fe_sensChirpProfTimeCfg     fe_sensChirpProfTimeCfg_rom
#define fe_sensChirpProfTimeCfgGet  fe_sensChirpProfTimeCfgGet_rom
#define fe_sensPerChirpCfg          fe_sensPerChirpCfg_rom
#define fe_sensPerChirpCfgGet       fe_sensPerChirpCfgGet_rom
#define fe_sensPerChirpCtrl         fe_sensPerChirpCtrl_rom
#define fe_sensPerChirpCtrlGet      fe_sensPerChirpCtrlGet_rom
#define fe_sensFrameCfg             fe_sensFrameCfg_rom
#define fe_sensFrameCfgGet          fe_sensFrameCfgGet_rom
#define fe_sensorStart              fe_sensorStart_rom
#define fe_sensorStop               fe_sensorStop_rom
#define fe_sensorStatusGet          fe_sensorStatusGet_rom

/*! @} */

/*!
 * @name FECSS Library RFS Module low level drivers ROM functions
 * @{
 */
#define rfs_cmdFwVerGet             rfs_cmdFwVerGet_rom
#define rfs_cmdCpuStsGet            rfs_cmdCpuStsGet_rom
#define rfs_cmdFactCalDataSet       rfs_cmdFactCalDataSet_rom
#define rfs_cmdFactCalDataGet       rfs_cmdFactCalDataGet_rom
#define rfs_cmdRxTxCalDataSet       rfs_cmdRxTxCalDataSet_rom
#define rfs_cmdRxTxCalDataGet       rfs_cmdRxTxCalDataGet_rom
#define rfs_cmdRfPwrOnOff           rfs_cmdRfPwrOnOff_rom
#define rfs_cmdRfFactCal            rfs_cmdRfFactCal_rom
#define rfs_cmdApllClkCtrl          rfs_cmdApllClkCtrl_rom
#define rfs_cmdRfCalibrate          rfs_cmdRfCalibrate_rom
#define rfs_cmdRfClockBwCfg         rfs_cmdRfClockBwCfg_rom
#define rfs_cmdGpadcCfg             rfs_cmdGpadcCfg_rom
#define rfs_cmdGpadcTrig            rfs_cmdGpadcTrig_rom
#define rfs_cmdTempCfg              rfs_cmdTempCfg_rom
#define rfs_cmdTempTrig             rfs_cmdTempTrig_rom
#define rfs_cmdSensLbCfg            rfs_cmdSensLbCfg_rom
#define rfs_cmdSensLbEna            rfs_cmdSensLbEna_rom
#define rfs_cmdFecssRdifCtrl        rfs_cmdFecssRdifCtrl_rom
#define rfs_cmdRfStatusGet          rfs_cmdRfStatusGet_rom
#define rfs_cmdDebugCfg             rfs_cmdDebugCfg_rom
#define rfs_cmdRfTxRuntimeClpcCal   rfs_cmdRfTxRuntimeClpcCal_rom

#define rfs_cmdChirpProfCfg         rfs_cmdChirpProfCfg_rom
#define rfs_cmdChirpProfCfgGet      rfs_cmdChirpProfCfgGet_rom
#define rfs_cmdCwSensorStart        rfs_cmdCwSensorStart_rom
#define rfs_cmdCwSensorStop         rfs_cmdCwSensorStop_rom
#define rfs_cmdSensDynPwrSaveDis    rfs_cmdSensDynPwrSaveDis_rom
#define rfs_cmdSensDynPwrSaveStsGet rfs_cmdSensDynPwrSaveStsGet_rom
#define rfs_cmdSensStartCfg         rfs_cmdSensStartCfg_rom

#define rfs_cmdMonEnableTrig        rfs_cmdMonEnableTrig_rom
#define rfs_cmdDbgPdMeas            rfs_cmdDbgPdMeas_rom
#define rfs_cmdDbgTxPwrMeas         rfs_cmdDbgTxPwrMeas_rom

#define rfs_cmdLiveSynthFreqCfg     rfs_cmdLiveSynthFreqCfg_rom
#define rfs_cmdLiveRxSaturCfg       rfs_cmdLiveRxSaturCfg_rom
#define rfs_cmdLiveGpadcCtmCfg      rfs_cmdLiveGpadcCtmCfg_rom
#define rfs_cmdPllCtrlVltMonCfg     rfs_cmdPllCtrlVltMonCfg_rom
#define rfs_cmdTxNRxLbMonCfg        rfs_cmdTxNRxLbMonCfg_rom
#define rfs_cmdTxNPwrMonCfg         rfs_cmdTxNPwrMonCfg_rom
#define rfs_cmdTxNBbMonCfg          rfs_cmdTxNBbMonCfg_rom
#define rfs_cmdTxNDcSigMonCfg       rfs_cmdTxNDcSigMonCfg_rom
#define rfs_cmdRxHpfDcSigMonCfg     rfs_cmdRxHpfDcSigMonCfg_rom
#define rfs_cmdPmClkMonCfg          rfs_cmdPmClkMonCfg_rom

/*! @} */

#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF rommap.h
 */


