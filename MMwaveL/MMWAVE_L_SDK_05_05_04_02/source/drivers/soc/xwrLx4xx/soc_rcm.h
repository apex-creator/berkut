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

#ifndef SOC_RCM_XWRL64XX_H
#define SOC_RCM_XWRL64XX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup DRV_SOC_RCM_MODULE APIs for SOC Reset and Clock Functions
 *  \ingroup DRV_SOC_MODULE
 *
 * For more details and example usage, see \ref DRIVERS_SOC_PAGE
 *
 *  @{
 */

#include <kernel/dpl/SystemP.h>

#define SOC_RCM_FREQ_HZ2MHZ(hz)     ((hz)/(1000000U))
#define SOC_RCM_FREQ_MHZ2HZ(mhz)    ((mhz)*(1000000U))

#define SOC_RCM_MEMINIT_APPSS_RAM1A_INIT      (1U << 0U)
#define SOC_RCM_MEMINIT_APPSS_RAM2A_INIT      (1U << 1U)
#define SOC_RCM_MEMINIT_APPSS_RAM3A_INIT      (1U << 2U)
#define SOC_RCM_MEMINIT_APPSS_ALL_INIT        (SOC_RCM_MEMINIT_APPSS_RAM1A_INIT | \
                                               SOC_RCM_MEMINIT_APPSS_RAM2A_INIT | \
                                               SOC_RCM_MEMINIT_APPSS_RAM3A_INIT)

#define SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT                 (0x1U)
#define SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT                 (0x1U << 1U)
#define SOC_RCM_MEMINIT_HWA_SHRAM_INIT                    (0x1U << 2U)
#define SOC_RCM_MEMINIT_FECSS_SHRAM_INIT                  (0x1U << 3U)
#define SOC_RCM_MEMINIT_TPCCA_INIT                        (0x1U << 8U)
#define SOC_RCM_MEMINIT_TPCCB_INIT                        (0x1U << 9U)





typedef enum  SOC_rcmM4ClockSrc_e {SOC_rcmM4ClockSrc_OSC_CLK = 0x000,
                                   SOC_rcmM4ClockSrc_SLOW_CLK = 0x111,
                                   SOC_rcmM4ClockSrc_MDLL_CLK = 0x222,
                                   SOC_rcmM4ClockSrc_FAST_CLK = 0x333} SOC_rcmM4ClockSrc;

/** @brief ROM version 2 devices */
#define SOC_RCM_EFUSEROM_VER_2          ((uint8_t)(0x02U))

/**
 * @brief Reset Causes
 */
typedef enum SOC_RcmResetCause_e
{
    /**
     * @brief   Value specifying Reset Cause Clear
     */
    SOC_RcmResetCause_POWER_CAUSE_CLEAR = 0x0U,
    /**
     * @brief   Value specifying Power ON Reset
     */
    SOC_RcmResetCause_POWER_ON_RESET = 0x1U,
    /**
     * @brief   Value specifying Warm Reset or Subsystem Reset
     */
    SOC_RcmResetCause_WARM_RESET = 0x2U,
    /**
     * @brief   Value specifying STC Reset
     */
	SOC_RcmResetCause_STC_RESET = 0x4U,
    /**
     * @brief   Value specifying M4 CPU Reset
     */
	SOC_RcmResetCause_CPU_ONLY_RESET = 0x10U,
    /**
     * @brief   Value specifying M4 Core Reset
     */
	SOC_RcmResetCause_CORE_RESET = 0x20U,
    /**
     * @brief max value
     */
    SOC_RcmResetCause_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmResetCause;

/**
 * @brief Peripheral IDs
 */
typedef enum SOC_RcmPeripheralId_e
{
    /**
     * @brief   Value specifying CAN
     */
    SOC_RcmPeripheralId_APPSS_MCAN,
    /**
     * @brief   Value specifying LIN
     */
    SOC_RcmPeripheralId_APPSS_LIN,
    /**
     * @brief   Value specifying QSPI (Quad SPI)
     */
    SOC_RcmPeripheralId_APPSS_QSPI,
    /**
     * @brief   Value specifying APPSS RTIA (Timer)
     */
    SOC_RcmPeripheralId_APPSS_RTI,
    /**
     * @brief   Value specifying APPSS WatchDog
     */
    SOC_RcmPeripheralId_APPSS_WDT,
    /**
     * @brief   Value specifying APPSS SPI-0
     */
    SOC_RcmPeripheralId_APPSS_MCSPIA,
    /**
     * @brief   Value specifying APPSS SPI-1
     */
    SOC_RcmPeripheralId_APPSS_MCSPIB,
    /**
     * @brief   Value specifying APPSS I2C
     */
    SOC_RcmPeripheralId_APPSS_I2C,
    /**
     * @brief   Value specifying APPSS SCI-A (UART)
     */
    SOC_RcmPeripheralId_APPSS_UART0,
    /**
     * @brief   Value specifying APPSS SCI-B (UART)
     */
    SOC_RcmPeripheralId_APPSS_UART1,
    /**
     * @brief   Value specifying APPSS ESM
     */
    SOC_RcmPeripheralId_APPSS_ESM,
    /**
     * @brief   Value specifying APPSS EDMA
     */
    SOC_RcmPeripheralId_APPSS_EDMA,
    /**
     * @brief   Value specifying APPSS CRC
     */
    SOC_RcmPeripheralId_APPSS_CRC,
    /**
     * @brief   Value specifying APPSS PWM
     */
    SOC_RcmPeripheralId_APPSS_PWM,
    /**
     * @brief   Value specifying APPSS PWM
     */
    SOC_RcmPeripheralId_APPSS_GIO,
    /**
     * @brief   Value specifying HWASS
     */
    SOC_RcmPeripheralId_HWASS,
    /**
     * @brief max value
     */
    SOC_RcmPeripheralId_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmPeripheralId;

/**
 * @brief Peripheral Clock Sources
 */
typedef enum SOC_RcmPeripheralClockSource_e
{
    /**
     * @brief   Value specifying Crystal Clock (40MHz)
     */
    SOC_RcmPeripheralClockSource_OSC_CLK,
    /**
     * @brief   Value specifying Slow Clock (33Khz)
     */
    SOC_RcmPeripheralClockSource_SLOW_CLK,
    /**
     * @brief   Value specifying MDLL Clock (160Mhz)
     */
    SOC_RcmPeripheralClockSource_MDLL_CLK,
    /**
     * @brief   Value specifying Fast Clock (160Mhz)
     */
    SOC_RcmPeripheralClockSource_FAST_CLK,
    /**
     * @brief   Value specifying XREF_IN Clock (40MHz)
     */
    SOC_RcmPeripheralClockSource_XREF_IN_CLK,
    /**
     * @brief   Value specifying OSC_CLKx2 Clock (80Mhz)
     */
    SOC_RcmPeripheralClockSource_OSC_CLKX2,
    /**
     * @brief   Value specifying RC_CLK_10M Clock (10Mhz)
     */
	SOC_RcmPeripheralClockSource_RC_CLK_10M,
    /**
     * @brief   Value specifying RCCLK32K Clock (32KHz)
     */
	SOC_RcmPeripheralClockSource_RCCLK32K,
    /**
     * @brief max value
     */
    SOC_RcmPeripheralClockSource_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmPeripheralClockSource;

/**
 * @brief M4 Clock Sources
 */
typedef enum SOC_RcmM4ClockSource_e
{
    SOC_RcmM4ClockSource_OSC_CLK,
    SOC_RcmM4ClockSource_SLOW_CLK,
    SOC_RcmM4ClockSource_MDLL_CLK,
    SOC_RcmM4ClockSource_FAST_CLK,
    SOC_RcmM4ClockSource_MAX_VALUE = 0xFFFFFFFFu,
} SOC_RcmM4ClockSource;

/**
 * @brief QSPI frequency values
 */
typedef enum SOC_RcmQspiClockFreqId_e {
    /**
     * @brief   Value specifying QSPI clock of 40 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_40MHZ = 0x0,
    /**
     * @brief   Value specifying QSPI clock of 60 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_60MHZ = 0x1,
    /**
     * @brief   Value specifying QSPI clock of 80 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_80MHZ = 0x2,
    /**
     * @brief max value
     */
    SOC_RcmQspiClockFreqId_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmQspiClockFreqId;

/**
 * @brief Peripheral Clock Gate Status
 */
typedef enum SOC_RcmPeripheralClockGate_e {
    /**
     * @brief Peripheral Clock Ungate
     */
    SOC_RcmPeripheralClockGateEnable,
    /**
     * @brief Peripheral Clock Gate
     */
    SOC_RcmPeripheralClockGateDisable

}SOC_RcmPeripheralClockGate;

/**
 *  \brief Enable ADPLL
 *
 */
void SOC_rcmEnableADPLLClock();

/**
 *  \brief Set M4 frequency
 *
 * \param m4FreqHz [in] M4 frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetM4Clock(uint32_t m4FreqHz);

/**
 *  \brief Set M4 Clock Source
 *
 * \param m4Src [in] Clock Source Enum
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetM4ClockSrc(SOC_rcmM4ClockSrc m4Src);

/**
 *  \brief Get M4 frequency
 *
 * \return M4 frequency, in Hz
 */
uint32_t SOC_rcmGetM4Clock(void);

/**
 *  \brief Set peripheral frequency
 *
 * \param periphId [in] Peripheral ID
 * \param clkSource [in] Peripheral clock source to use
 * \param freqHz [in] Peripheral frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId periphId, SOC_RcmPeripheralClockSource clkSource, uint32_t freqHz);

/**
 *  \brief Get peripheral frequency
 *
 * \param periphId [in] Peripheral ID
 *
 * \return Peripheral frequency, in Hz
 */
uint32_t SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId periphId);

/**
 *  \brief Get SOC reset cause
 *
 * \return SOC reset cause
 */
uint8_t SOC_rcmGetResetCause(void);

/**
 *  \brief Enable/Disable peripheral Clock gating
 *
 *      This API programs the IP clock gates
 *      for a specified peripheral Id.
 *
 * \param periphId [in] Peripheral ID
 * \param enable [in] ungate / gate clock
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmEnablePeripheralClock(SOC_RcmPeripheralId periphId, SOC_RcmPeripheralClockGate enable);

/**
 *  \brief Start memory initialization for APPSS Shared Memory RAM0, RAM1 and HWASS Shared RAM
 */
void SOC_rcmStartInitSharedRam(uint16_t flag);

/**
 *  \brief Wait memory initialization to complete APSS Shared Memory RAM0, RAM1 and HWA Shared RAM
 */
void SOC_rcmWaitMemInitSharedRam(uint16_t flag);

/**
 *  \brief Start memory initialization for TPCCA and TPCCB
 */
void SOC_rcmStartMemInitTpcc(uint16_t flag);

/**
 *  \brief Wait memory initialization to complete TPCCA and TPCCB
 */
void SOC_rcmWaitMemInitTpcc(uint16_t flag);

/**
 *  \brief Reads EFUSEROM_VER from TOP Efuse memory
 */
uint8_t SOC_getEfuseRomVersion(void);

/**
 *  \brief Reads SYNTH_TRIM_VALID field from TOP Efuse memory
 */
uint8_t SOC_rcmReadSynthTrimValid(void);

/**
 *  \brief Reads APLL_CALIB_TRIM_VALID field from TOP Efuse memory
 */
uint8_t SOC_rcmReadAPLLCalibTrimValid(void);

/**
 *  \brief Reads PG_VER field from TOP Efuse memory
 */
uint8_t SOC_getEfusePgVersion(void);

/**
 *  \brief Reads ANTENNA TYPE field from TOP Efuse memory
 */
uint8_t SOC_isDeviceAOP(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
