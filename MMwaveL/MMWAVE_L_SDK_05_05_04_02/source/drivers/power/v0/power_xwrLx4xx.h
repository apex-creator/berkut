/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \defgroup DRV_POWER_MODULE APIs for POWER Module
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the pinmux module.
 *
 *  @{
 */

/**
 *  \file  power_xwrLx4xx.h
 *
 *  \brief Power manager interface for the 64XX/14xx.
 *
 *  \brief This module defines the power resources, constraints, events, sleep
 *         states and transition latencies for 64XX.
 *
 *         A reference power policy is provided which can transition the MCU from the
 *         active state to one of two sleep states: Low-Power Deep Sleep (LPDS) or
 *         Sleep. The policy looks at the estimated idle time remaining, and the
 *         active constraints, and determine which sleep state to transition to. The
 *         policy will give first preference to choosing LPDS, but if that is not
 *         appropriate (e.g., not enough idle time), it will choose Sleep.
 */

#ifndef POWER_XWRLX4XX_H
#define POWER_XWRLX4XX_H

#include <stdint.h>
#include <drivers/utils/List.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/pinmux.h>
#include <drivers/power.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 *  @cond NODOC */
/* Power resources */
#define POWER_PERIPH_CAN        0
/*!< Resource ID: CAN */

#define POWER_PERIPH_SPI        1
/*!< Resource ID: SPI */

#define POWER_PERIPH_QSPI       2
/*!< Resource ID: QSPI */

#define POWER_PERIPH_TOPSS      3
/*!< Resource ID: TOPSS */

#define POWER_PERIPH_APP_RTI    4
/*!< Resource ID: APP_RTI */

#define POWER_PERIPH_WD         5
/*!< Resource ID: WD */

#define POWER_PERIPH_UART0      6
/*!< Resource ID: UART0 */

#define POWER_PERIPH_UART1      7
/*!< Resource ID: UART1 */

#define POWER_PERIPH_I2C        8
/*!< Resource ID: I2C */

#define POWER_PERIPH_LIN        9
/*!< Resource ID: LIN */

#define POWER_PERIPH_ESM        10
/*!< Resource ID: LIN */

#define POWER_PERIPH_EDMA        11
/*!< Resource ID: LIN */

#define POWER_PERIPH_CRC        12
/*!< Resource ID: LIN */

#define POWER_PERIPH_PWM        13
/*!< Resource ID: LIN */

#define POWER_PERIPH_GIO        14
/*!< Resource ID: LIN */

#define POWER_PERIPH_HWASS      15
/*!< Resource ID: LIN */

/* \cond */
#define POWER_NUMRESOURCES        16U /* Number of resources in database */
/* \endcond */


/*
 *  Power constraints on the 64XX device
 */
#define POWER_DISALLOW_LPDS       0
/*!< Constraint: Disallow entry to Low Power Deep Sleep (LPDS) */

#define POWER_DISALLOW_SLEEP   1
/*!< Constraint: Disallow entry to Sleep */

#define POWER_DISALLOW_IDLE   2
/*!< Constraint: Disallow entry to Idle */

/* \cond */
#define POWER_NUMCONSTRAINTS      3U   /*!< number of constraints */
/* \endcond */
/*! @endcond */
/*
 *  Power events on the 64XX device
 *
 *  Each event must be a power of two, and the event IDs must be sequential
 *  without any gaps.
 */
#define POWER_ENTERING_LPDS       0x1
/*!< Power event: The device is entering the LPDS sleep state */

#define POWER_ENTERING_SLEEP      0x2
/*!< Power event: The device is entering the device sleep state */

#define POWER_ENTERING_IDLE       0x4
/*!< Power event: The device is entering the idle state */

#define POWER_AWAKE_LPDS          0x8
/*!< Power event: The device is waking from the LPDS sleep state */

#define POWER_AWAKE_SLEEP         0x10
/*!< Power event: The device is waking from the device sleep state */

#define POWER_AWAKE_IDLE          0x20
/*!< Power event: The device is waking from the device idle state */

/* \cond */
#define POWER_NUMEVENTS           6    /*!< number of events */
/* \endcond */

/* \cond */
/* Use by NVIC Register structure */
#define POWER_numNVICSetEnableRegs    6U
#define POWER_numNVICIntPriority      49U
/* \endcond */

/* \cond */
/* Number of pins that can be parked in LPDS */
#define POWER_NUMPINS             24U
/* \endcond */

/*! @brief  Used to specify parking of a pin during LPDS */
typedef struct {
    uint32_t pin;
    /*!< The pin to be parked */
    uint32_t parkState;
    /*!< The state to park the pin (an enumerated Power_ParkState) */
} Power_ParkInfo;

/*! @brief  Source of Wake-up can be either GPIO (WU_REQIN) or SYNC_IN*/
typedef enum {
    POWER_GPIO_WAKEUP_LPDS,
    POWER_SYNCIN_IO_WAKEUP_LPDS
}Power_GPIO_SYNCIO_Wakeup_Enable;

/*! @brief Power global configuration */
typedef struct {
    /*! Initialization function for the power policy */
    Power_PolicyInitFxn policyInitFxn;
    /*! The power policy function */
    Power_PolicyFxn policyFxn;
    /*!
     *  @brief  Hook function called before entering LPDS
     *
     *  This function is called after any notifications are complete,
     *  and before any pins are parked, just before entry to LPDS.
     */
    void (*enterLPDSHookFxn)(void);
    /*!
     *  @brief  Hook function called when resuming from LPDS
     *
     *  This function is called early in the wake sequence, before any
     *  notification functions are run.
     */
    void (*resumeLPDSHookFxn)(void);
    /*!
     *  @brief  Hook function called before entering idle3
     *
     *  This function is called after any notifications are complete,
     *  and before any pins are parked, just before entry to LPDS.
     */
    void (*enteridle3HookFxn)(void);
    /*!
     *  @brief  Hook function called when resuming from idle3
     *
     *  This function is called early in the wake sequence, before any
     *  notification functions are run.
     */
    void (*resumeidle3HookFxn)(void);

    /*! Determines whether to run the power policy function */
    bool enablePolicy;

    /*! Enable GPIO (WU_REQIN)/SYNC-IN as a wakeup source for LPDS */
    bool enableGPIOSyncIOWakeupLPDS;

    /*! Enable sleep counter as a wakeup source for LPDS */
    bool enableSleepCounterWakeupLPDS;

    /*! Enable UART activity as a wakeup source for LPDS */
    bool enableUARTWakeupLPDS;

    /*! Enable SPI chip select as a wakeup source for LPDS */
    bool enableSPICSWakeupLPDS;

    /*! Enable RTC as a wakeup source for LPDS */
    bool enableRTCWakeupLPDS;

    /*! Enable FRC as a wakeup source for LPDS */
    bool enableFRCWakeupLPDS;

    /*! Select source of wakeup to be either GPIO (WU_REQIN) or SYNC_IN */
    Power_GPIO_SYNCIO_Wakeup_Enable selectGpioSyncIOLpds;
    /*!
     *  @brief  The GPIO (WU_REQIN) trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupGPIOEdgeLPDS;
    /*!
     *  @brief  The SYNC-IN trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupSyncIOEdgeLPDS;

    /*!
     *  @brief  The SPI CS (WU_REQIN) trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupSpiEdgeLPDS;

    /*!
     *  @brief  The UART RX (WU_REQIN) trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupUartEdgeLPDS;

    /*!
     *  @brief  SRAM retention mask for LPDS
     *
     *  Value can be a mask of the different Memory Clusters (defined in driverlib/prcm.h):
     */
    uint32_t ramRetentionMaskLPDS;

    /*!
     *  @brief  Pointer to an array of pins to be parked during LPDS
     *
     *  A value of NULL will disable parking of any pins during LPDS
     */
    Power_ParkInfo * pinParkDefs;
    /*!
     *  @brief  Number of pins to be parked during LPDS
     */
    uint32_t numPins;
    /*!
     *  @brief  Threshold for entry to LPDS in microseconds
     */
    uint32_t LPDSThreshold;
    /*!
     *  @brief  Latency for entry to and exit from LPDS in microseconds
     */
    uint32_t totalLatencyForLPDS;

    uint32_t resumeLatencyForLPDS;
    /*!
     *  @brief  Threshold for entry to Sleep in microseconds
     */
    uint32_t sleepThreshold;
    /*!
     *  @brief  Latency for entry to and exit from Sleep in microseconds
     */
    uint32_t totalLatencyForSleep;

    uint32_t resumeLatencyForSleep;
    /*!
     *  @brief  Threshold for entry to Idle in microseconds
     */
    uint32_t idleThreshold;
    /*!
     *  @brief  Latency for entry to and exit from Idle in microseconds
     */
    uint32_t totalLatencyForIdle;

    uint32_t resumeLatencyForIdle;

} Power_ConfigV1;

/*!
 *  @cond NODOC
 *  NVIC registers that need to be saved before entering LPDS.
 */
typedef struct {
    uint32_t vectorTable;
    uint32_t auxCtrl;
    uint32_t intCtrlState;
    uint32_t appInt;
    uint32_t sysCtrl;
    uint32_t configCtrl;
    uint32_t sysPri1;
    uint32_t sysPri2;
    uint32_t sysPri3;
    uint32_t sysHcrs;
    uint32_t systickCtrl;
    uint32_t systickReload;
    uint32_t systickCalib;
    uint32_t intSetEn[POWER_numNVICSetEnableRegs];
    uint32_t intPriority[POWER_numNVICIntPriority];
} Power_NVICRegisters;
/*! @endcond */

/*!
 *  @cond NODOC
 *  MCU core registers that need to be save before entering LPDS.
 */
typedef struct {
    uint32_t msp;
    uint32_t psp;
    uint32_t psr;
    uint32_t primask;
    uint32_t faultmask;
    uint32_t basepri;
    uint32_t control;
} Power_MCURegisters;
/*! @endcond */

/*!
 *  @cond NODOC
 *  Structure of context registers to save before entering LPDS.
 */
typedef struct {
    Power_MCURegisters m4Regs;
    Power_NVICRegisters nvicRegs;
} Power_SaveRegisters;
/*! @endcond */

/*! @brief Enumeration of states a pin can be parked in */
typedef enum {
    /*! No pull resistor, leave pin in a HIZ state */
    POWER_PARK,
    /*! Take no action; do not park the pin */
    POWER_DONT_PARK
} Power_ParkState;

/*! 
 *  @cond NODOC
 *  @brief Enumeration of pins that can be parked */
typedef enum {
    /*! PIN_PAD_AA */
    POWER_PIN_PAD_AA = PIN_PAD_AA,
    /*! PIN_PAD_AB */
    POWER_PIN_PAD_AB = PIN_PAD_AB,
    /*! PIN_PAD_AC */
    POWER_PIN_PAD_AC = PIN_PAD_AC,
    /*! PIN_PAD_AD */
    POWER_PIN_PAD_AD = PIN_PAD_AD,
    /*! PIN_PAD_AE */
    POWER_PIN_PAD_AE = PIN_PAD_AE,
    /*! PIN_PAD_AF */
    POWER_PIN_PAD_AF = PIN_PAD_AF,
    /*! PIN_PAD_AG */
    POWER_PIN_PAD_AG = PIN_PAD_AG,
    /*! PIN_PAD_AH */
    POWER_PIN_PAD_AH = PIN_PAD_AH,
    /*! PIN_PAD_AI */
    POWER_PIN_PAD_AI = PIN_PAD_AI,
    /*! PIN_PAD_AJ */
    POWER_PIN_PAD_AJ = PIN_PAD_AJ,
    /*! PIN_PAD_AK */
    POWER_PIN_PAD_AK = PIN_PAD_AK,
    /*! PIN_PAD_AL */
    POWER_PIN_PAD_AL = PIN_PAD_AL,
    /*! PIN_PAD_AM */
    POWER_PIN_PAD_AM = PIN_PAD_AM,
    /*! PIN_PAD_AN */
    POWER_PIN_PAD_AN = PIN_PAD_AN,
    /*! PIN_PAD_AO */
    POWER_PIN_PAD_AO = PIN_PAD_AO,
    /*! PIN_PAD_AP */
    POWER_PIN_PAD_AP = PIN_PAD_AP,
    /*! PIN_PAD_AQ */
    POWER_PIN_PAD_AQ = PIN_PAD_AQ,
    /*! PIN_PAD_AR */
    POWER_PIN_PAD_AR = PIN_PAD_AR,
    /*! PIN_PAD_AS */
    POWER_PIN_PAD_AS = PIN_PAD_AS,
    /*! PIN_PAD_AT */
    POWER_PIN_PAD_AT = PIN_PAD_AT,
    /*! PIN_PAD_AU */
    POWER_PIN_PAD_AU = PIN_PAD_AU,
    /*! PIN_PAD_AV */
    POWER_PIN_PAD_AV = PIN_PAD_AV,
    /*! PIN_PAD_AW */
    POWER_PIN_PAD_AW = PIN_PAD_AW,
    /*! PIN_PAD_AX */
    POWER_PIN_PAD_AX = PIN_PAD_AX,
} Power_Pin;
/*! @endcond */

/*!
 *  @brief  Specify the wakeup sources for LPDS and Shutdown
 *
 *  The wakeup sources for LPDS and Shutdown can be dynamically changed
 *  at runtime, via Power_configureWakeup().  The application
 *  should fill a structure of this type, and pass it as the parameter
 *  to Power_configureWakeup() to specify the new wakeup settings.
 */
typedef struct {

    /*! Enable GPIO or SYNC_IN as a wakeup source for LPDS */
    bool enableGPIOSyncIOWakeupLPDS;
    /*! Select between GPIO and SYNC_IN as a wakeup source for LPDS */
    Power_GPIO_SYNCIO_Wakeup_Enable selectGpioSyncIOLpds;
    /*! Enable sleep counter as a wakeup source for LPDS */
    bool enableSleepCounterWakeupLPDS;
    /*! Enable UART activity as a wakeup source for LPDS */
    bool enableUARTWakeupLPDS;
    /*! Enable SPI CS as a wakeup source for LPDS */
    bool enableSPICSWakeupLPDS;
    /*! Enable RTC as a wakeup source for LPDS */
    bool enableRTCWakeupLPDS;
    /*! Enable FRC as a wakeup source for LPDS */
    bool enableFRCWakeupLPDS;
    /*!
     *  @brief  The GPIO trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupGPIOEdgeLPDS;
    /*!
     *  @brief  The GPIO trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupSyncIOEdgeLPDS;
    /*!
     *  @brief  The SPI CS trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupSpiEdgeLPDS;
    /*!
     *  @brief  The UART Rx trigger type for wakeup from LPDS
     *
     *  Value can be one of the following (defined in driverlib/prcm.h):
     *  PRCM_LPDS_FALL_EDGE, PRCM_LPDS_RISE_EDGE
     */
    uint32_t wakeupUartEdgeLPDS;
} Power_Wakeup;

/*!
 *  @cond NODOC
 *  Internal structure defining Power module state.
 */
typedef struct {
    List_List notifyList;
    uint32_t constraintMask;
    uint32_t state;
    uint16_t dbRecords[POWER_NUMRESOURCES];
    bool enablePolicy;
    bool initialized;
    uint8_t refCount[POWER_NUMRESOURCES];
    uint8_t constraintCounts[POWER_NUMCONSTRAINTS];
    Power_PolicyFxn policyFxn;
    uint32_t pinMode[POWER_NUMPINS];
    uint32_t pinLockMask;
    Power_Wakeup wakeupConfig;
} Power_ModuleState;
/*! @endcond */

/*!
 *  @brief  Function configures wakeup for LPDS and shutdown
 *
 *  This function allows the app to configure the type for waking up from LPDS 
 *  This overwrites any previous wakeup settings.
 *
 *  @param[in]  wakeup      Settings applied to wakeup configuration
 */
void Power_configureWakeup(Power_Wakeup *wakeup);

/*! OS-specific power policy initialization function */
void Power_initPolicy(void);

/*!
 *  @brief  Function to get wakeup configuration settings
 *
 *  This function allows an app to query the current LPDS and shutdown
 *  wakeup configuration settings.
 *
 *  @param[in]  wakeup      A #Power_Wakeup structure to be written into
 */
void Power_getWakeup(Power_Wakeup *wakeup);

/*!
 *  @brief A reference power policy is provided which can transition the MCU
 *  from the active state to one of two sleep states: Low-Power Deep Sleep
 *  (LPDS) or Idle state. (Sleep will be supported in future releases)
 *
 *  The policy looks at the estimated idle time remaining, and the
 *  active constraints, and determine which sleep state to transition to. The
 *  policy will give first preference to choosing LPDS, but if that is not
 *  appropriate (e.g., not enough time), it will choose Idle.
 */
void Power_sleepPolicy(unsigned long long sleepTimeus);

/*!
 *  @cond NODOC
 *  @brief  Software reset of a resource
 *
 *  This function performs a software reset of a resource.
 *
 *  Resource identifiers are device specific, and defined in the
 *  device-specific Power include file.  For example, the resources for
 *  64XX are defined in Power.h.
 *
 *  @param[in]  resourceId      resource id
 *
 *  @retval  #Power_SOK on success,
 *  @retval  #Power_EINVALIDINPUT if the reseourceId is invalid.
 *
 */
int_fast16_t Power_reset(uint_fast16_t resourceId);
/*! @endcond */

 /*!
 *  @brief  Device Idle mode
 *
 *  This function brings the device to Idle3 mode. Idle3 is TI defined idle state where,
 *  - All Peripherals are clock gated
 *  - HWA and Frontend are powered Down
 *  - Core is kept in Deep Sleep State
 *  - TOPSS is driven by OSC/16 clock
 *  - Core is brought out of deep sleep using RTI timer interrupt (mapped to WIC)
 *
 */
 void Power_Idle3();

/*! @endcond */

 /*!
 *  @brief  Wakeup interrupt handler
 * 
 * @param args Wakeup interrupt arguments 
 *
 */
void Power_idleWicIsr(void *args);

/* \cond */
#define Power_getPerformanceLevel(void)   0
#define Power_setPerformanceLevel(level)  Power_EFAIL
/* \endcond */

#ifdef __cplusplus
}
#endif

#endif /* POWER_XWRLX4XX_H */

/** @} */
