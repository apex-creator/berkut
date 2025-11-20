/*
 * Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \defgroup DRV_PINMUX_MODULE APIs for PINMUX
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the pinmux module.
 *
 *  @{
 */

/**
 *  \file pinmux.h
 *
 *  \brief PINMUX Driver API/interface file.
 *
 * \brief  This file contains pad configure register offsets and bit-field
 *         value macros for different configurations,
 *
 *           BIT[21]        TXDISABLE       disable the pin's output driver
 *           BIT[18]        RXACTIVE        enable the pin's input buffer (typically kept enabled)
 *           BIT[17]        PULLTYPESEL     set the iternal resistor pull direction high or low (if enabled)
 *           BIT[16]        PULLUDEN        internal resistor disable (0 = enabled / 1 = disabled)
 *           BIT[3:0]       MUXMODE         select the desired function on the given pin
 */

#ifndef PINMUX_XWRL64XX_H_
#define PINMUX_XWRL64XX_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pinmux_DomainId_t
 *  \name Pinmux Domain ID
 *  @{
 */
/** \brief NOT USED on this SOC */
#define PINMUX_DOMAIN_ID_MAIN           (0U)
/** \brief NOT USED on this SOC */
#define PINMUX_DOMAIN_ID_MCU            (1U)
/** @} */

/** \brief Macro to mark end of pinmux config array */
#define PINMUX_END                      (0xFFFFFFFFU)
/** \brief Pin mode */
#define PIN_MODE(mode)                  (mode)
/** \brief Resistor disable */
#define PIN_PULL_DISABLE                (0x1U << 8U)
/** \brief Pull direction, 0: pull-down, 1: pull-up */
#define PIN_PULL_DIRECTION              (0x1U << 9U)

/**
 *  \anchor Pinmux_Offsets
 *  \name Pad config register offset in control module
 *  @{
 */
#define PIN_PAD_AA          (0x00000000U)
#define PIN_PAD_AB          (0x00000004U)
#define PIN_PAD_AC          (0x00000008U)
#define PIN_PAD_AD          (0x0000000CU)
#define PIN_PAD_AE          (0x00000010U)
#define PIN_PAD_AF          (0x00000014U)
#define PIN_PAD_AG          (0x00000018U)
#define PIN_PAD_AH          (0x0000001CU)
#define PIN_PAD_AI          (0x00000020U)
#define PIN_PAD_AJ          (0x00000024U)
#define PIN_PAD_AK          (0x00000028U)
#define PIN_PAD_AL          (0x0000002CU)
#define PIN_PAD_AM          (0x00000030U)
#define PIN_PAD_AN          (0x00000034U)
#define PIN_PAD_AO          (0x00000038U)
#define PIN_PAD_AP          (0x0000003CU)
#define PIN_PAD_AQ          (0x00000040U)
#define PIN_PAD_AR          (0x00000044U)
#define PIN_PAD_AS          (0x00000048U)
#define PIN_PAD_AT          (0x0000004CU)
#define PIN_PAD_AU          (0x00000050U)
#define PIN_PAD_AV          (0x00000054U)
#define PIN_PAD_AW          (0x00000058U)
#define PIN_PAD_AX          (0x0000005CU)


/* pin names */
/** \brief Pin Input Override*/
#define PIN_OVERRIDE_INPUT_ENABLE                 (0x1U << 5U)
#define PIN_OVERRIDE_INPUT_ENABLE_CTRL            (0x1U << 4U)
/** \brief Pin Output Override */
#define PIN_OVERRIDE_OUTPUT_DISABLE               (0x1U << 7U)
#define PIN_OVERRIDE_OUTPUT_DISABLE_CTRL          (0x1U << 6U)

/** \brief Slew Rate High */
#define PIN_SLEW_RATE_HIGH                        (0x0U << 10U)
/** \brief Slew Rate Low */
#define PIN_SLEW_RATE_LOW                         (0x1U << 10U)


/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief Structure defining the pin configuration parameters */
typedef struct Pinmux_PerCfg
{
    uint32_t    offset;
    /**< Register offset for configuring the pin.
     *   Refer \ref Pinmux_Offsets.
     *   Set this to #PINMUX_END to demark the end of configuration array */
    uint32_t    settings;
    /**< Value to be configured.
     *   Active mode configurations like mux mode, pull resistor and
     *   buffer mode.
     *
     *   To set a value use like   : "| PIN_PULL_DISABLE"
     *   To reset a value use like : "& (~PIN_PULL_DIRECTION)"
     *
     *   For example,
     *   PIN_MODE(7) | ((PIN_PULL_DISABLE | PIN_INPUT_ENABLE) & (~PIN_PULL_DIRECTION)
     */
} Pinmux_PerCfg_t;

typedef struct PadToPin
{
    uint32_t Pad;

    uint32_t gpioNum;

    uint32_t mode;
}PadToPin_t;

/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief  This API configures the pinmux based on the domain
 *
 *  \param  pinmuxCfg   Pointer to list of pinmux configuration array.
 *                      This parameter cannot be NULL and the last entry should
 *                      be initialized with #PINMUX_END so that this function
 *                      knows the end of configuration.
 *  \param  domainId    NOT USED in this SOC
 */
void Pinmux_config(const Pinmux_PerCfg_t *pinmuxCfg, uint32_t domainId);
uint32_t Pinmux_Read(uint32_t pinOffset);
uint32_t Pinmux_getMode(uint32_t pad);
uint32_t Pinmux_getPinNum(uint32_t pad);

/**
 *  \brief             This API is used to override input enable
 *
 *  \param  offset      Pinmux register offset value. Refer \ref Pinmux_Offsets
 *
 *  \param  state       Input Enable
 *                      This parameter is set to 1 to disable input
 *                      and 0 to disable
 */

void Pinmux_inputoverride(uint32_t offset, bool state);

/**
 * \brief               This API configures pin output override
 *
 * \param offset        Pinmux register offset value. Refer \ref Pinmux_Offsets
 *
 * \param state         Output Disable
 *                      This parameter is set to 1 to Disable output
 *                      and 0 to enable
 */

void Pinmux_outputoverride(uint32_t offset, bool state);
/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef PINMUX_AM64X_H_ */

/** @} */
