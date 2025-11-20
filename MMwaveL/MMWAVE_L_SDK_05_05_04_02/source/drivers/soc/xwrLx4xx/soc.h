/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SOC_XWRL64XX_H_
#define SOC_XWRL64XX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup DRV_SOC_MODULE APIs for SOC Specific Functions
 *  \ingroup DRV_MODULE
 *
 * For more details and example usage, see \ref DRIVERS_SOC_PAGE
 *
 *  @{
 */

#include <kernel/dpl/SystemP.h>
#include <drivers/soc/xwrLx4xx/soc_rcm.h>
#include <drivers/hw_include/cslr_soc.h>

/**
 *  \anchor SOC_DomainId_t
 *  \name SOC Domain ID
 *  @{
 */
#define SOC_DOMAIN_ID_TOPSS_CTRL        (0U)
#define SOC_DOMAIN_ID_APP_RCM           (1U)
#define SOC_DOMAIN_ID_APP_CTRL          (2U)
#define SOC_DOMAIN_ID_TOP_IO_MUX        (3U)
#define SOC_DOMAIN_ID_TOP_PRCM          (4U)
/** @} */

/**
 * @brief SOC System Reset Reason
 */
typedef enum SOC_SysRstReason_e 
{
    /**
     * @brief   Value specifying Power ON Reset
     */
    SOC_SYS_RESET_REASON_PORZ = 0x01U,
    /**
     * @brief   Value specifying Warm reset due to soft register
     */
    SOC_SYS_RESET_REASON_SR = 0x02U,
    /**
     * @brief   Value specifying Warm reset due to WDG
     */
    SOC_SYS_RESET_REASON_WDG = 0x04U,

}SOC_SysRstReason ;

/**
 * @brief SOC Reset Reason
 */
typedef enum SOC_RstReason_e 
{
    /**
     * @brief   Value specifying Power ON Reset
     */
    SOC_RESET_REASON_PORZ = 0x01U,
    /**
     * @brief   Value specifying Warm Reset
     */               
    SOC_RESET_REASON_WARM,
    /**
     * @brief   Value specifying Deep Sleep Reset
     */     
    SOC_RESET_REASON_DEEPSLEEP,
    /**
     * @brief   Value specifying Soft Reset
     */
    SOC_RESET_REASON_SOFT,    
    /**
     * @brief   Value specifying STC or Warm Reset
     */
    SOC_RESET_REASON_STC_WARM, 
    /**
     * @brief   Value specifying STC or Power ON Reset
     */
    SOC_RESET_REASON_STC_PORZ, 

}SOC_RstReason ;



/**
 * \brief Enable clock to specified module
 *
 * \param moduleId [in] module ID's
 * \param enable [in] 1: enable clock to the module, 0: disable clock to the module
 *
 * \return SystemP_SUCCESS Module clock is enabled
 * \return SystemP_FAILURE Module clock could not be enabled
 */
int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable);

/**
 * \brief Enables ADPLL
 *
 * \return SystemP_SUCCESS Module clock is enabled
 * \return SystemP_FAILURE Module clock could not be enabled
 */
int32_t SOC_clocksEnable(void);

/**
 * \brief get clock frequency
 *
 * \return clock frequency in Hz
 */
uint64_t SOC_getSelfCpuClk(void);

/**
 * \brief Set module clock to specified frequency
 *
 * \param moduleId [in] module ID's
 * \param clkId [in] clocks associated with the specified module ID
 * \param clkRate [in] Frequency to set in Hz
 *
 * \return SystemP_SUCCESS Module clock is enabled
 * \return SystemP_FAILURE Module clock could not be enabled
 */
int32_t SOC_moduleSetClockFrequency(SOC_RcmPeripheralId moduleId, SOC_RcmPeripheralClockSource clkId, uint64_t clkRate);

/**
 * \brief Convert a core ID to a user readable name
 *
 * \param coreId    [in] see \ref CSL_CoreID
 *
 * \return name as a string
 */
const char *SOC_getCoreName(uint16_t coreId);

/**
 * \brief Lock control module partition to prevent writes into control MMRs
 *
 * \param domainId    [in] See SOC_DomainId_t
 * \param partition   [in] Partition number to unlock
 */
void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition);

/**
 * \brief Unlock control module partition to allow writes into control MMRs
 *
 * \param domainId    [in] See SOC_DomainId_t
 * \param partition   [in] Partition number to unlock
 */
void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition);

/**
 *  \brief Print's module clock info to the console
 */
void SOC_logAllClockHz(void);

/**
 *  \brief SOC Virtual (CPU) to Physical address translation function.
 *
 *  \param virtAddr [IN] Virtual/CPU address
 *
 *  \return Corresponding SOC physical address
 */
uint64_t SOC_virtToPhy(void *virtAddr);

/**
 *  \brief Initializes APPSS shared RAM0, RAM1 and HWASS Shared RAM
 */
void SOC_memoryInit(uint16_t flag);

/**
 *  \brief Enables MDLL Clock
 */
void SOC_enableMDLLClock(void);

/**
 *  \brief Software Warm reset request. Generates warm reset for entire device.
 */
void SOC_triggerWarmReset(void);

/**
 *  \brief Soft reset request.
 */
void SOC_triggerSoftReset(void);

/**
 *  \brief  Retrieves the reset reason.
 * 
 *  \return Reset reason
 */
SOC_RstReason SOC_getRstReason(void);

/**
 *  \brief  Retrieves the system reset reason from SYS_RST_CAUSE.
 *  All the reset registers will be cleared by the bootloader. 
 *  The reset register values will be stored in BOOT_INFO_REG0[23:0]
 *  The TOP_PRCM:SYS_RST_CAUSE[2:0] is stored in APP_CTRL:APPSS_BOOT_INFO_REG0[7:4]
 * 
 *  \return System reset reason
 */
SOC_SysRstReason SOC_getSysRstReason(void);

/**
 * \brief Enable or disable ePWM time base clock from Control MMR
 *
 * \param enable       [in] TRUE to enable and FALSE to disable
 */
void SOC_setEpwmTbClk(uint32_t enable);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
