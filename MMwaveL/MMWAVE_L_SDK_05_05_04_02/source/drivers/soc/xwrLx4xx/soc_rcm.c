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

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/xwrL64xx/cslr_app_ctrl.h>
#include <drivers/prcm/v0/cslr_top_prcm.h>
#include <drivers/hw_include/xwrL64xx/cslr_soc_baseaddress.h>
#include <drivers/cslr_fec_ctrl.h>

#define SOC_RCM_EFUSE_FIELD_EXTRACT(efuseReg, fromNum, row, startBit, endBit)  SOC_rcmExtract8((efuseReg)->EFUSE##fromNum##_ROW_##row, endBit, startBit)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_XTAL_FREQ_GRP(efuseReg)          SOC_RCM_EFUSE_FIELD_EXTRACT(efuseReg, 0, 20, 0,  0)

#define SOC_RCM_XTAL_CLK_26MHZ                      (26000000U)
#define SOC_RCM_XTAL_CLK_40MHZ                      (40000000U)
#define SOC_RCM_XTAL_CLK_38p4MHZ                    (38400000U)
#define SOC_RCM_XTAL_CLK_25MHZ                      (25000000U)

#define SOC_RCM_UTILS_ARRAYSIZE(x)                  (sizeof(x)/sizeof(x[0]))

/* EFUSE0_ROW_15 Register Bit Fields */
#define SOC_RCM_ANT_TYPE_START_BIT                    (22U)
#define SOC_RCM_ANT_TYPE_END_BIT                      (24U)

/* EFUSE0_ROW_18 Register Bit Fields */
#define SOC_RCM_PG_VER_START_BIT                    (0U)
#define SOC_RCM_PG_VER_END_BIT                      (3U)
#define SOC_RCM_EFUSE_ROM_VER_START_BIT             (20U)
#define SOC_RCM_EFUSE_ROM_VER_END_BIT               (24U)
#define SOC_RCM_METAL_VER_START_BIT                 (7U)
#define SOC_RCM_METAL_VER_END_BIT                   (9U)

/* EFUSE0_ROW_41 Register Bit Fields */
#define SOC_RCM_SYNTH_TRIM_VALID_START_BIT          (25U)
#define SOC_RCM_SYNTH_TRIM_VALID_END_BIT            (25U)

/* EFUSE0_ROW_53 Register Bit Fields */
#define SOC_RCM_APLL_CALIB_TRIM_VALID_START_BIT     (19U)
#define SOC_RCM_APLL_CALIB_TRIM_VALID_END_BIT       (19U)

/* Status of PLL Lock */
#define  SOC_RCM_PLL_NOT_LOCKED                (0x0U)
#define  SOC_RCM_PLL_LOCKED                    (0x1U)

/* Status for current clock set to APLL/DPLL */
#define SOC_RCM_CURR_CLK_APLL_DPLL                  (0x8U)


typedef enum SOC_RcmXtalFreqId_e
{
    SOC_RcmXtalFreqId_CLK_26MHZ,
    SOC_RcmXtalFreqId_CLK_38p4MHZ,
    SOC_RcmXtalFreqId_CLK_40MHZ,
    SOC_RcmXtalFreqId_CLK_25MHZ
} SOC_RcmXtalFreqId;

typedef enum SOC_RcmFixedClockId_e
{
	SOC_RCM_FIXEDCLKID_SLOW_CLK,
    SOC_RCM_FIXEDCLKID_XREF_IN_CLK,
    SOC_RCM_FIXEDCLKID_RC_CLK_10M,
    SOC_RCM_FIXEDCLKID_RCCLK32K,
} SOC_RcmFixedClockId;

typedef enum SOC_RcmClockSrcId_e
{
    SOC_RCM_CLKSRCID_OSC_CLK,
    SOC_RCM_CLKSRCID_SLOW_CLK,
    SOC_RCM_CLKSRCID_MDLL_CLK,
    SOC_RCM_CLKSRCID_FAST_CLK,
    SOC_RCM_CLKSRCID_XREF_IN_CLK,
    SOC_RCM_CLKSRCID_OSC_CLKX2,
    SOC_RCM_CLKSRCID_RC_CLK_10M,
    SOC_RCM_CLKSRCID_RCCLK32K,
} SOC_RcmClockSrcId;

typedef enum SOC_RcmDpllIdType_e
{
    SOC_RCM_DPLL_CORE,
} SOC_RcmDpllId_e;

typedef enum SOC_RcmClockSrcType_e
{
    SOC_RCM_CLKSRCTYPE_XTAL,
    SOC_RCM_CLKSRCTYPE_FIXED,
    SOC_RCM_CLKSRCTYPE_DIGPLL,
} SOC_RcmClockSrcType;
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct SOC_RcmDpllInfo_s
{
    SOC_RcmDpllId_e dpllId;
} SOC_RcmDpllInfo;

typedef struct SOC_RcmFixedClkInfo_s
{
    SOC_RcmFixedClockId fixedClkId;
} SOC_RcmFixedClkInfo;

typedef struct SOC_RcmClkSrcInfo_s
{
    SOC_RcmClockSrcType clkSrcType;
    union{
        SOC_RcmFixedClkInfo  fixedClkInfo;
        SOC_RcmDpllInfo    dpllInfo;
    }u;
} SOC_RcmClkSrcInfo;

typedef struct SOC_RcmXTALInfo_s
{
    uint32_t Finp;
} SOC_RcmXTALInfo;

typedef struct SOC_RcmFixedClocks_s
{
    uint32_t       fOut;
} SOC_RcmFixedClocks_t;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const uint32_t gSocRcmXTALFreqTbl[4] =
{
    [SOC_RcmXtalFreqId_CLK_26MHZ]      = SOC_RCM_XTAL_CLK_26MHZ,
    [SOC_RcmXtalFreqId_CLK_38p4MHZ]      = SOC_RCM_XTAL_CLK_38p4MHZ,
    [SOC_RcmXtalFreqId_CLK_40MHZ]  = SOC_RCM_XTAL_CLK_40MHZ,
    [SOC_RcmXtalFreqId_CLK_25MHZ] = SOC_RCM_XTAL_CLK_25MHZ,
};

static const SOC_RcmClkSrcInfo gSocRcmClkSrcInfoMap[8] =
{
    [SOC_RCM_CLKSRCID_OSC_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_XTAL,
    },
    [SOC_RCM_CLKSRCID_SLOW_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
    },
    [SOC_RCM_CLKSRCID_MDLL_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DIGPLL,
    },
    [SOC_RCM_CLKSRCID_FAST_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DIGPLL,
    },
    [SOC_RCM_CLKSRCID_XREF_IN_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
    },
    [SOC_RCM_CLKSRCID_OSC_CLKX2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_XTAL,
    },
    [SOC_RCM_CLKSRCID_RC_CLK_10M] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
    },
    [SOC_RCM_CLKSRCID_RCCLK32K] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
    },
};

static const SOC_RcmFixedClocks_t gSocRcmFixedClocksTbl[4] =
{
    [SOC_RCM_FIXEDCLKID_SLOW_CLK] =
    {
        /* RCCLK32K	0.032	0.032	0.032	0.032 */
        .fOut = 32000U,
    },
    [SOC_RCM_FIXEDCLKID_XREF_IN_CLK] =
    {
        /* FE1_REF_CLK	40.000	40.000	40.000	40.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(40U),
    },
    [SOC_RCM_FIXEDCLKID_RC_CLK_10M] =
    {
        /* RC_CLK_10M	10.000	10.000	10.000	10.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(10U),
    },
    [SOC_RCM_FIXEDCLKID_RCCLK32K] =
    {
        /* RCCLK32K	0.032	0.032	0.032	0.032 */
        .fOut = 32000U,
    },
};

static const uint16_t gSocRcmM4ClkSrcValMap[4] =
{
    [SOC_RcmM4ClockSource_OSC_CLK]                       = 0x000U,
    [SOC_RcmM4ClockSource_SLOW_CLK]                      = 0x111U,
    [SOC_RcmM4ClockSource_MDLL_CLK]                      = 0x222U,
    [SOC_RcmM4ClockSource_FAST_CLK]                       = 0x333U,
};

static const SOC_RcmClockSrcId gSocRcmM4ClkSrcInfoMap[4] =
{
    [SOC_RcmM4ClockSource_OSC_CLK] = SOC_RCM_CLKSRCID_OSC_CLK,
    [SOC_RcmM4ClockSource_SLOW_CLK] = SOC_RCM_CLKSRCID_SLOW_CLK,
    [SOC_RcmM4ClockSource_MDLL_CLK] = SOC_RCM_CLKSRCID_MDLL_CLK,
    [SOC_RcmM4ClockSource_FAST_CLK] = SOC_RCM_CLKSRCID_FAST_CLK,
};

static const SOC_RcmClockSrcId gSocRcmPeripheralClkSrcInfoMap[8] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK] = SOC_RCM_CLKSRCID_OSC_CLK,
    [SOC_RcmPeripheralClockSource_SLOW_CLK] = SOC_RCM_CLKSRCID_SLOW_CLK,
    [SOC_RcmPeripheralClockSource_MDLL_CLK] = SOC_RCM_CLKSRCID_MDLL_CLK,
    [SOC_RcmPeripheralClockSource_FAST_CLK] = SOC_RCM_CLKSRCID_FAST_CLK,
    [SOC_RcmPeripheralClockSource_XREF_IN_CLK] = SOC_RCM_CLKSRCID_XREF_IN_CLK,
    [SOC_RcmPeripheralClockSource_OSC_CLKX2] = SOC_RCM_CLKSRCID_OSC_CLKX2,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmPeripheralClockSource_RCCLK32K] = SOC_RCM_CLKSRCID_RCCLK32K,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for MCAN
 */
static const uint16_t gSocRcmMcanClkSrcValMap[6] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK]     = 0x000U,
    [SOC_RcmPeripheralClockSource_OSC_CLKX2]   = 0x111U,
    [SOC_RcmPeripheralClockSource_MDLL_CLK]    = 0x222U,
    [SOC_RcmPeripheralClockSource_FAST_CLK]    = 0x333U,
    [SOC_RcmPeripheralClockSource_SLOW_CLK]    = 0x444U,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for QSPI
 */
static const uint16_t gSocRcmQspiClkSrcValMap[6] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK]     = 0x000U,
    [SOC_RcmPeripheralClockSource_OSC_CLKX2]   = 0x111U,
    [SOC_RcmPeripheralClockSource_MDLL_CLK]    = 0x222U,
    [SOC_RcmPeripheralClockSource_FAST_CLK]    = 0x333U,
    [SOC_RcmPeripheralClockSource_SLOW_CLK]    = 0x444U,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for RTI
 */
static const uint16_t gSocRcmRtiClkSrcValMap[5] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK]     = 0x000U,
    [SOC_RcmPeripheralClockSource_XREF_IN_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SLOW_CLK]    = 0x333U,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for RTI
 */
static const uint16_t gSocRcmWdtClkSrcValMap[5] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK]     = 0x000U,
    [SOC_RcmPeripheralClockSource_XREF_IN_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SLOW_CLK]    = 0x333U,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for SPI
 */
static const uint16_t gSocRcmSpiClkSrcValMap[6] =
{
    [SOC_RcmPeripheralClockSource_OSC_CLK]     = 0x000U,
    [SOC_RcmPeripheralClockSource_OSC_CLKX2]   = 0x111U,
    [SOC_RcmPeripheralClockSource_MDLL_CLK]    = 0x222U,
    [SOC_RcmPeripheralClockSource_FAST_CLK]    = 0x333U,
    [SOC_RcmPeripheralClockSource_SLOW_CLK]    = 0x444U,
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void SOC_rcmGetClkSrcAndDivReg (SOC_RcmPeripheralId periphId,
                                SOC_RcmPeripheralClockSource clkSource,
                                uint16_t *clkSrcVal,
                                volatile uint32_t **ClkCtlReg);
static int32_t SOC_rcmGetClkSrcAndDivValue (SOC_RcmPeripheralId periphId,
                                        uint32_t *clkSource,
                                        volatile uint32_t *clkDiv);
static SOC_RcmXtalFreqId SOC_rcmGetXTALFrequency(void);
static uint32_t SOC_rcmGetModuleClkDivFromRegVal(uint32_t moduleClkDivRegVal);
static uint32_t SOC_rcmGetPeripheralClockFrequency(uint32_t clkSource);
static uint32_t SOC_rcmGetFreqLeafNode(SOC_RcmClockSrcId clkSrcId);
static uint32_t SOC_rcmGetM4InClockFrequency(void);
static int32_t SOC_rcmGetFixedRootClkFout(SOC_RcmFixedClockId fixedClkId, uint32_t *fOut);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline uint32_t SOC_rcmMake8 (uint8_t msb, uint8_t lsb, uint8_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = (uint32_t)val & mask;

    return (newVal << lsb);
}

static inline uint32_t SOC_rcmMake16 (uint8_t msb, uint8_t lsb, uint16_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = (uint32_t)val & mask;

    return (newVal << lsb);
}

static inline uint32_t SOC_rcmMake32 (uint8_t msb, uint8_t lsb, uint32_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = val & mask;

    return (newVal << lsb);
}

static inline uint32_t SOC_rcmInsert8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint8_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake8(msb, lsb, val));
}

static inline uint32_t SOC_rcmInsert16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint16_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake16(msb, lsb, val));
}

static inline uint32_t SOC_rcmInsert32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint32_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake32(msb, lsb, val));
}

static inline uint8_t SOC_rcmExtract8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint8_t     value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint8_t)((reg >> lsb) & mask);
    return value;
}

static inline uint16_t SOC_rcmExtract16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint16_t    value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint16_t)((reg >> lsb) & mask);
    return value;
}

static inline uint32_t SOC_rcmExtract32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)/*GAP COMMENT ID:83 START*/
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (reg >> lsb) & mask;
    return value;
}/*GAP COMMENT ID:83 END*/



static inline CSL_fec_ctrlRegs* SOC_rcmGetBaseAddressFECSHAREDRAM (void)/*GAP COMMENT ID:83_1 START*/
{
    return (CSL_fec_ctrlRegs*)CSL_FEC_SHARED_RAM_U_BASE;
}/*GAP COMMENT ID:83_1 END*/


static inline CSL_topss_ctrlRegs* SOC_rcmGetBaseAddressTOPCTRL (void)/*GAP COMMENT ID:83_2 START*/
{
    return (CSL_topss_ctrlRegs*)CSL_TOPSS_CTRL_U_BASE;
}/*GAP COMMENT ID:83_2 END*/


static inline CSL_app_ctrlRegs* SOC_rcmGetBaseAddressAPPCTRL (void)/*GAP COMMENT ID:83_3 START*/
{
    return (CSL_app_ctrlRegs*) CSL_APP_CTRL_U_BASE;
}/*GAP COMMENT ID:83_3 END*/

static inline CSL_app_rcmRegs* SOC_rcmGetBaseAddressAPPRCM (void)
{
    return (CSL_app_rcmRegs*) CSL_APP_RCM_U_BASE;
}

static inline CSL_top_efuseRegs* SOC_rcmGetBaseAddressTOPEfuse (void)/*GAP COMMENT ID:83_5 START*/
{
    return (CSL_top_efuseRegs*) CSL_TOP_EFUSE_U_BASE;
}/*GAP COMMENT ID:83_5 START*/

static uint8_t SOC_rcmReadSynthTrimValidEfuse (const CSL_top_efuseRegs* ptrTopEfuseRegs)
{
    return (SOC_rcmExtract8 (ptrTopEfuseRegs->EFUSE0_ROW_41, \
                             SOC_RCM_SYNTH_TRIM_VALID_END_BIT, \
                             SOC_RCM_SYNTH_TRIM_VALID_START_BIT));
}

static uint8_t SOC_rcmReadApllCalibTrimValidEfuse (const CSL_top_efuseRegs* ptrTopEfuseRegs)
{
    return (SOC_rcmExtract8 (ptrTopEfuseRegs->EFUSE0_ROW_53, \
                             SOC_RCM_APLL_CALIB_TRIM_VALID_END_BIT, \
                             SOC_RCM_APLL_CALIB_TRIM_VALID_START_BIT));
}

void SOC_rcmEnableADPLLClock()
{
    uint8_t pllLockStatus;
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_STATUS);
    pllLockStatus = HW_GET_FIELD(regVal, CSL_PLLDIG_CTRL_PLLDIG_STATUS_PLLDIG_STATUS_PLLDIG_LOCKMON);
    if(pllLockStatus == SOC_RCM_PLL_NOT_LOCKED)/*GAP COMMENT ID: 84 START*/
    {
        /* Disable PLL DIG */
        HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN,  0x00000000);
        HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_MDIV_NDIV, 0x001400A0);
        /* Low frequency mode enable, High Frequency Mode enable */
        HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_MODE_EN,   0x00010001);
        /*
        CTRL[8] : 300M-400M VCO is enabled
        CTRL[12:13]: 300M-400M VCO clock goes to CLKOUT
        CTRL[15:14]: Lock time programmability after the frequency lock detection for phase lock set to 128 input ref clock cycles
        CTRL[19:18]: Programmability for the fast precharge of the vcontrol signal using fast lock gen block set to 8 input ref clock cycles
        */
        HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_CTRL,      0x00045100);
        /* Enable PLL DIG */
        HW_WR_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_EN,        0x00000007);
        /* Wait for Lock */
        do
        {
            regVal = HW_RD_REG32(CSL_PLLDIG_CTRL_U_BASE + CSL_PLLDIG_CTRL_PLLDIG_STATUS);
            pllLockStatus = HW_GET_FIELD(regVal, CSL_PLLDIG_CTRL_PLLDIG_STATUS_PLLDIG_STATUS_PLLDIG_LOCKMON);
        }while(pllLockStatus ==  SOC_RCM_PLL_NOT_LOCKED);
    }/*GAP COMMENT ID: 84 END*/
}

static uint32_t SOC_rcmGetClkSrcFromClkSelVal(const uint16_t *clkSelTbl, uint32_t numEntries, uint32_t clkSelMatchVal)
{
    uint32_t i;
    uint32_t clkSource;

    for (i = 0; i < numEntries; i++)/*GAP COMMENT ID: 85 START END*/
    {
        if (clkSelMatchVal == clkSelTbl[i])
        {
            break;
        }
    }
    if (i < numEntries)/*GAP COMMENT ID: 85_1 START END*/
    {
        clkSource = i;
    }
    else/*GAP COMMENT ID: 86 START*/
    {
        clkSource = ~0U;
    }/*GAP COMMENT ID: 86 END*/
    return clkSource;
}

static uint32_t SOC_rcmGetFreqLeafNode(SOC_RcmClockSrcId clkSrcId)
{
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t FOut = 1U;
    const SOC_RcmClkSrcInfo *clkSrcInfo;

    clkSrcInfo = &gSocRcmClkSrcInfoMap[clkSrcId];
    switch (clkSrcInfo->clkSrcType)
    {
        case SOC_RCM_CLKSRCTYPE_XTAL:
        {
            clkFreqId = SOC_rcmGetXTALFrequency();
            FOut = gSocRcmXTALFreqTbl[clkFreqId];
            break;
        }
        case SOC_RCM_CLKSRCTYPE_FIXED:
        {
            SOC_rcmGetFixedRootClkFout(clkSrcInfo->u.fixedClkInfo.fixedClkId ,&FOut);
            break;
        }
        case SOC_RCM_CLKSRCTYPE_DIGPLL:
        {
        	// TODO:
            FOut = 160000000;
            break;
        }
        default:
        {
            break;
        }
    }
    return (FOut);
}

static SOC_RcmXtalFreqId SOC_rcmGetXTALFrequency(void)
{

    uint32_t xtalFreqGrp, trimValid;
    SOC_RcmXtalFreqId freq = SOC_RcmXtalFreqId_CLK_40MHZ;

    xtalFreqGrp = 0;
    trimValid = 0;

    if(trimValid == 0U)/*GAP COMMENT ID: 87 START END*/
    {
        if(xtalFreqGrp == 0U)/*GAP COMMENT ID: 88 START END*/
        {
            freq = SOC_RcmXtalFreqId_CLK_40MHZ;
        }
        else if(xtalFreqGrp == 1U)/*GAP COMMENT ID: 86_1 START*/
        {
            freq = SOC_RcmXtalFreqId_CLK_26MHZ;
        }/*GAP COMMENT ID: 86_1 END*/
        else
        {
            /*MISRA-C*/
        }
    }
    else/*GAP COMMENT ID: 86_2 START*/
    {

    }/*GAP COMMENT ID: 86_2 END*/

    return (freq);
}

static int32_t SOC_rcmGetFixedRootClkFout(SOC_RcmFixedClockId fixedClkId, uint32_t *fOut)
{
    *fOut = gSocRcmFixedClocksTbl[fixedClkId].fOut;
    return SystemP_SUCCESS;
}

static void SOC_rcmGetClkSrcAndDivReg (SOC_RcmPeripheralId periphId,
                                SOC_RcmPeripheralClockSource clkSource,
                                uint16_t *clkSrcVal,
                                volatile uint32_t **ClkCtlReg)
{
	CSL_app_rcmRegs *ptrAPPRCMRegs;

    ptrAPPRCMRegs = SOC_rcmGetBaseAddressAPPRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_APPSS_MCAN:
         {
             *ClkCtlReg  = &(ptrAPPRCMRegs->APP_CAN_CLKCTL);
             *clkSrcVal = gSocRcmMcanClkSrcValMap[clkSource];
             break;
         }
        case SOC_RcmPeripheralId_APPSS_LIN:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_LIN_CLKCTL);
            *clkSrcVal = 0x000U; /* No SrcSel Field for LIN */
            break;
        }
        case SOC_RcmPeripheralId_APPSS_QSPI:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_QSPI_CLKCTL);
            *clkSrcVal = gSocRcmQspiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_APPSS_RTI:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_RTI_CLKCTL);
            *clkSrcVal = gSocRcmRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_APPSS_WDT:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_WD_CLKCTL);
            *clkSrcVal = gSocRcmWdtClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIA:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_SPI_CLKCTL);
            *clkSrcVal = gSocRcmSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIB:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_SPI_CLKCTL);
            *clkSrcVal = gSocRcmSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_APPSS_I2C:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_I2C_CLKCTL);
            *clkSrcVal = 0x000U; /* No SrcSel Field for I2C */
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART0:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_UART_0_CLKCTL);
            *clkSrcVal = 0x000U; /* No SrcSel Field for UART0 */
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART1:
        {
            *ClkCtlReg  = &(ptrAPPRCMRegs->APP_UART_1_CLKCTL);
            *clkSrcVal = 0x000U; /* No SrcSel Field for UART1 */
            break;
        }
        default:
        {
            *ClkCtlReg  = NULL;
            *clkSrcVal = 0x888U;
            break;
        }
    }
    return;
}

static int32_t SOC_rcmGetClkSrcAndDivValue (SOC_RcmPeripheralId periphId,
                                        uint32_t *clkSource,
                                        volatile uint32_t *clkDiv)
{

	CSL_app_rcmRegs *ptrAPPRCMRegs;
    uint32_t clkSrc;
    uint32_t clkSrcId;
    int32_t retVal = SystemP_SUCCESS;

    ptrAPPRCMRegs = SOC_rcmGetBaseAddressAPPRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_APPSS_MCAN:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_CAN_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_CAN_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmMcanClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmMcanClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_APPSS_LIN:
        {
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_LIN_CLKCTL, 27U, 16U);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_QSPI:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_QSPI_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_QSPI_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmQspiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmQspiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource =  clkSrcId;

            break;
        }
        case SOC_RcmPeripheralId_APPSS_RTI:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_RTI_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_RTI_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_APPSS_WDT:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_WD_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_WD_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmWdtClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmWdtClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIA:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_SPI_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_SPI_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSpiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSpiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIB:
        {
            clkSrc  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_SPI_CLKCTL, 15U, 4U);
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_SPI_CLKCTL, 27U, 16U);
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSpiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSpiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_APPSS_I2C:
        {
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_I2C_CLKCTL, 27U, 16U);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART0:
        {
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_UART_0_CLKCTL, 27U, 16U);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART1:
        {
            *clkDiv = SOC_rcmExtract16(ptrAPPRCMRegs->APP_UART_1_CLKCTL, 27U, 16U);
            break;
        }
        default:/*GAP COMMENT ID: 89 START*/
        {
            *clkDiv = 0U;
            retVal = SystemP_FAILURE;
            break;
        }/*GAP COMMENT ID: 89 END*/
    }
    return retVal;
}

static uint32_t SOC_rcmGetModuleClkDivVal(uint32_t inFreq, uint32_t outFreq)
{
    uint32_t moduleClkDivVal;

    DebugP_assert((inFreq % outFreq) == 0U);
    moduleClkDivVal = inFreq / outFreq;
    moduleClkDivVal--;
    return moduleClkDivVal;
}

static uint32_t SOC_rcmGetModuleClkDivRegVal(uint32_t moduleClkDivVal)/*GAP COMMENT ID:83_4 START*/
{
    uint32_t moduleClkDivRegVal;

    moduleClkDivRegVal = (moduleClkDivVal & 0xFU) | ((moduleClkDivVal & 0xFU) << 4) | ((moduleClkDivVal & 0xFU) << 8);
    return moduleClkDivRegVal;
}/*GAP COMMENT ID:83_4 END*/

static uint32_t SOC_rcmGetModuleClkDivFromRegVal(uint32_t moduleClkDivRegVal)
{
    uint32_t moduleClkDivVal;

    moduleClkDivVal = ((moduleClkDivRegVal & 0xFU) + 1U);
    return moduleClkDivVal;
}

static uint32_t SOC_rcmGetM4InClockFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClockSrcId rootClkSrcId;
    uint32_t m4ClkSrcId;
	CSL_app_rcmRegs *ptrAPPRCMRegs;
    uint32_t clkSrcVal;

    ptrAPPRCMRegs = SOC_rcmGetBaseAddressAPPRCM ();
    clkSrcVal  = SOC_rcmExtract16(ptrAPPRCMRegs->APP_CPU_CLKCTL, 15U, 4U);
    m4ClkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmM4ClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmM4ClkSrcValMap), clkSrcVal);
    DebugP_assert(m4ClkSrcId != (uint32_t)SOC_RcmM4ClockSource_MAX_VALUE);
    rootClkSrcId = gSocRcmM4ClkSrcInfoMap[m4ClkSrcId];
    clkFreq= SOC_rcmGetFreqLeafNode(rootClkSrcId);

    return (clkFreq);
}

int32_t SOC_rcmSetM4Clock(uint32_t m4FreqHz)
{
	CSL_app_rcmRegs *ptrAPPRCMRegs;
    uint32_t Finp;
    uint32_t moduleClkDivVal;
    uint32_t newCpuClockRate;

    Finp = SOC_rcmGetM4InClockFrequency();
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, m4FreqHz);

    ptrAPPRCMRegs = SOC_rcmGetBaseAddressAPPRCM ();

    newCpuClockRate = SOC_rcmInsert32(ptrAPPRCMRegs->APP_CPU_CLKCTL, 27U, 16U, moduleClkDivVal);
    ptrAPPRCMRegs->APP_CPU_CLKCTL=newCpuClockRate;
    return SystemP_SUCCESS;
}

int32_t SOC_rcmSetM4ClockSrc(SOC_rcmM4ClockSrc m4Src)
{
    // APP_RCM:APP_CPU_CLKSTAT register
    volatile uint32_t* clkStatusreg = (volatile uint32_t*) (CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKSTAT);
    volatile uint32_t stat = 0u; 
    CSL_REG32_FINS((CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL), APP_RCM_APP_CPU_CLKCTL_APP_CPU_CLKCTL_SRCSEL, m4Src);
    // Check the status if the Clock Source selection was successful
    do
    {
        stat = *clkStatusreg;
        // Check APP_CPU_CLKSTAT_CURRCLK field
        stat = ((stat & CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_MASK) >> CSL_APP_RCM_APP_CPU_CLKSTAT_APP_CPU_CLKSTAT_CURRCLK_SHIFT);
    }while(stat != SOC_RCM_CURR_CLK_APLL_DPLL); // Check if the source is PLL DIG

    return SystemP_SUCCESS;
}

uint32_t SOC_rcmGetM4Clock(void)
{
    uint32_t Finp;
    uint32_t moduleClkDivRegVal;
    uint32_t clkDivVal;
    CSL_app_rcmRegs *ptrAppRCMRegs;

    ptrAppRCMRegs = SOC_rcmGetBaseAddressAPPRCM();
    Finp = SOC_rcmGetM4InClockFrequency();
    moduleClkDivRegVal = SOC_rcmExtract16(ptrAppRCMRegs->APP_CPU_CLKCTL, 27U, 16U);
    clkDivVal = SOC_rcmGetModuleClkDivFromRegVal(moduleClkDivRegVal);
    return (Finp / clkDivVal);
}

int32_t SOC_rcmSetPeripheralClock (SOC_RcmPeripheralId periphID,
                                      SOC_RcmPeripheralClockSource clkSource,
                                      uint32_t freqHz)
{
    volatile uint32_t   *ptrClkCtlReg;
    uint16_t            clkSrcVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;

    Finp = SOC_rcmGetPeripheralClockFrequency((uint32_t)clkSource);
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetClkSrcAndDivReg (periphID, clkSource, &clkSrcVal, &ptrClkCtlReg);

    if ((ptrClkCtlReg != NULL) && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        if ((SOC_RcmPeripheralId_APPSS_LIN == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_I2C == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_UART0 == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_UART1 == periphID))
        {
            /* Write the Clock Control Selection Value */
            *ptrClkCtlReg = SOC_rcmInsert16 (*ptrClkCtlReg, 27U, 16U, clkDivVal);
        }
        else
        {
            /* Write the Clock Control Selection Value */
            *ptrClkCtlReg = SOC_rcmInsert16 (*ptrClkCtlReg, 27U, 16U, clkDivVal);

            /* Write the Clock Control Selection Value */
            *ptrClkCtlReg = SOC_rcmInsert16 (*ptrClkCtlReg, 15U, 4U, clkSrcVal);
        }

        retVal = SystemP_SUCCESS;
    }
    else
    {
        /* Error */
        retVal = SystemP_FAILURE;
    }

    return (retVal);
}


uint32_t SOC_rcmGetPeripheralClock (SOC_RcmPeripheralId periphID)
{
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;
    uint32_t            freqHz = 0;
    uint32_t clkSource;

    retVal = SOC_rcmGetClkSrcAndDivValue(periphID, &clkSource, &clkDivisorRegVal);
    DebugP_assert(retVal == SystemP_SUCCESS);
    if (SystemP_SUCCESS == retVal)
    {
        if ((SOC_RcmPeripheralId_APPSS_LIN == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_I2C == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_UART0 == periphID) || \
        	(SOC_RcmPeripheralId_APPSS_UART1 == periphID))
        {
            Finp = gSocRcmXTALFreqTbl[SOC_rcmGetXTALFrequency()];

            clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
            freqHz = Finp / clkDivisor;
    	}
    	else
    	{
            Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
            clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
            freqHz = Finp / clkDivisor;
    	}
    }
    return (freqHz);
}

uint8_t SOC_rcmGetResetCause (void)
{
	CSL_app_rcmRegs *ptrRCMRegs;
    uint8_t     resetCause = 0U;

    ptrRCMRegs = SOC_rcmGetBaseAddressAPPRCM();

    /* Read the Reset Cause Register bits */
    resetCause = SOC_rcmExtract8 (ptrRCMRegs->RST_CAUSE, 7U, 0U);

    /* Clear the Reset Cause Register */
    SOC_rcmInsert8 (ptrRCMRegs->RST_CAUSE, 7U, 0U, 0x00U);
    return  resetCause;
}

static uint32_t SOC_rcmGetPeripheralClockFrequency(uint32_t clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClockSrcId rootClkSrcId;

    rootClkSrcId = gSocRcmPeripheralClkSrcInfoMap[clkSource];
    clkFreq = SOC_rcmGetFreqLeafNode(rootClkSrcId);
    return (clkFreq);
}

static CSL_top_efuseRegs* SOC_rcmGetBaseAddressTOPEFUSE (void)
{
    return (CSL_top_efuseRegs*)CSL_TOP_EFUSE_U_BASE;
}

int32_t SOC_rcmEnablePeripheralClock(SOC_RcmPeripheralId periphId, SOC_RcmPeripheralClockGate enable)
{
    CSL_app_rcmRegs *ptrAPPRCMRegs;
    uint32_t regVal;
    int32_t retVal = SystemP_SUCCESS;

    ptrAPPRCMRegs = SOC_rcmGetBaseAddressAPPRCM ();

    /* Enable APP_CTRL region by default */
    regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CTRL ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
    HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

    switch (periphId)
    {
        case SOC_RcmPeripheralId_APPSS_MCAN:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CAN ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_CAN_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_CAN_CLKCTL_APP_CAN_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_CAN_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_LIN:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_LIN ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_LIN_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_LIN_CLKCTL_APP_LIN_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_LIN_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_QSPI:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_QSPI ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_QSPI_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_QSPI_CLKCTL_APP_QSPI_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_QSPI_CLKCTL,regVal);

            break;
        }
        case SOC_RcmPeripheralId_APPSS_RTI:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_RTI ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_RTI_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_RTI_CLKCTL_APP_RTI_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_RTI_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_WDT:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_WD ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_WD_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_WD_CLKCTL_APP_WD_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_WD_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIA:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_0 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_SPI_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_SPI_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_MCSPIB:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_SPI_1 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_SPI_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_SPI_CLKCTL_APP_SPI_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_SPI_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_I2C:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_I2C ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_I2C_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_I2C_CLKCTL_APP_I2C_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_I2C_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART0:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_0 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_UART_0_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_0_CLKCTL_APP_UART_0_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_UART_0_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_UART1:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_UART_1 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);

            regVal = HW_RD_REG32(&ptrAPPRCMRegs->APP_UART_1_CLKCTL);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_UART_1_CLKCTL_APP_UART_1_CLKCTL_GATE ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->APP_UART_1_CLKCTL,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_ESM:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_APP_ESM ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_EDMA:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPCC_A ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A0 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE0_IPCFGCLKGATE0_TPTC_A1 ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE0,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_CRC:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_CRC ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_PWM:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE1_IPCFGCLKGATE1_APP_PWM ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE1,regVal);
            break;
        }
        case SOC_RcmPeripheralId_APPSS_GIO:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE2);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_GIO ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE2,regVal);
            break;
        }
        case SOC_RcmPeripheralId_HWASS:
        {
            regVal = HW_RD_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE2);
            HW_SET_FIELD32(regVal, CSL_APP_RCM_IPCFGCLKGATE2_IPCFGCLKGATE2_HWASS ,(((uint32_t)enable == 0U) ? 0x0U : 0X7U));
            HW_WR_REG32(&ptrAPPRCMRegs->IPCFGCLKGATE2,regVal);
            break;
        }
        default:
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }
    return retVal;
}



void SOC_rcmStartInitSharedRam(uint16_t flag)
{

    CSL_app_ctrlRegs* ptrAppssCtrlRegs = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;
    CSL_app_hwa_adcbuf_ctrlRegs* ptrAppssAdcbufCtrlRegs = (CSL_app_hwa_adcbuf_ctrlRegs*)CSL_APP_HWA_ADCBUF_CTRL_U_BASE;
    CSL_fec_ctrlRegs* ptrFecssCtrlRegs = (CSL_fec_ctrlRegs*)CSL_FEC_CTRL_U_BASE;


    if((flag & SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT) == SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT)
    {
            
        /* APSS Shared RAM0 (128KB) initialization */
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT - APSS Shared Memory RAM0 - (128KB) */
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT_STATUS, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_STATUS_HWASS_SHRD_RAM0_MEM_INIT_STATUS_MEM_STATUS) != 0U){;}/*GAP COMMENT ID: 90 START END*/

        /* Clear MEMINIT DONE before initiating MEMINIT - APSS Shared Memory RAM0 - (128KB) */
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE, 1U);
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE) != 0U){;}/*GAP COMMENT ID: 90_1 START END*/

        /* Start Memory Init - APSS Shared Memory RAM0 - (128KB) */
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_HWASS_SHRD_RAM0_MEM_INIT_MEM_INIT, 1U);

        }

    if((flag & SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT) == SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT)
    {
        /* APSS Shared RAM1 (128KB) initialization */
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT - APSS Shared Memory RAM1 - (128KB) */
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT_STATUS, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_STATUS_HWASS_SHRD_RAM1_MEM_INIT_STATUS_MEM_STATUS) != 0U){;}/*GAP COMMENT ID: 90_2 START END*/

        /* Clear MEMINIT DONE before initiating MEMINIT - APSS Shared Memory RAM1 - (128KB) */
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE, 1U);
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE) != 0U){;}/*GAP COMMENT ID: 90_3 START END*/

        /* Start Memory Init - APSS Shared Memory RAM1 - (128KB) */
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_HWASS_SHRD_RAM1_MEM_INIT_MEM_INIT, 1U);
    }

    if((flag & SOC_RCM_MEMINIT_FECSS_SHRAM_INIT ) == SOC_RCM_MEMINIT_FECSS_SHRAM_INIT )
    { 
        /* FECSS SHRAM (96KB) - Check Memory Initialization Status */
        while (CSL_FEXT(ptrFecssCtrlRegs->FECSS_SHARED_MEM_STATUS, FEC_CTRL_FECSS_SHARED_MEM_STATUS_FECSS_SHARED_MEM_STATUS_MEM_INIT_STATUS) != 0U){;}/*GAP COMMENT ID: 90_4 START END*/

        /* Clear MEMINIT DONE before initiating MEMINIT - FECSS SHRAM (96KB)  */
        CSL_FINS(ptrFecssCtrlRegs->FECSS_SHARED_MEM_DONE,FEC_CTRL_FECSS_SHARED_MEM_DONE_FECSS_SHARED_MEM_DONE_MEM_INIT_DONE, 1U);
        while (CSL_FEXT(ptrFecssCtrlRegs->FECSS_SHARED_MEM_DONE, FEC_CTRL_FECSS_SHARED_MEM_DONE_FECSS_SHARED_MEM_DONE_MEM_INIT_DONE) != 0U){;}/*GAP COMMENT ID: 90_5 START END*/

        /* Start Memory Init - FECSS SHRAM (96KB)  */
        CSL_FINS(ptrFecssCtrlRegs->FECSS_SHARED_MEM_INIT,FEC_CTRL_FECSS_SHARED_MEM_INIT_FECSS_SHARED_MEM_INIT_MEM_INIT, 1U);
    }

    if((flag & SOC_RCM_MEMINIT_HWA_SHRAM_INIT) == SOC_RCM_MEMINIT_HWA_SHRAM_INIT)
    {
        /* HWASS Shared RAM (160KB) initialization */
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT - HWASS Shared Memory - (160KB) */
        while (CSL_FEXT(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_STATUS, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_STATUS_HWASS_SHRD_RAM_MEM_STATUS_MEM_INIT_STATUS) != 0U){;}/*GAP COMMENT ID: 90_6 START END*/

        /* Clear MEMINIT DONE before initiating MEMINIT - HWASS Shared Memory  - (160KB) */
        CSL_FINS(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_DONE, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_DONE_HWASS_SHRD_RAM_MEM_DONE_MEM_INIT_DONE, 1U);
        while (CSL_FEXT(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_DONE, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_DONE_HWASS_SHRD_RAM_MEM_DONE_MEM_INIT_DONE) != 0U){;}/*GAP COMMENT ID: 90_7 START END*/

            /* Start Memory Init - HWASS Shared Memory  - (160KB) */
        CSL_FINS(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_INIT, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_INIT_HWASS_SHRD_RAM_MEM_INIT_MEM_INIT, 1U);
    }

    return;
}

void SOC_rcmWaitMemInitSharedRam(uint16_t flag)
{
    
    CSL_app_ctrlRegs* ptrAppssCtrlRegs = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;
    CSL_fec_ctrlRegs* ptrFecssCtrlRegs = (CSL_fec_ctrlRegs*)CSL_FEC_CTRL_U_BASE;
    
    CSL_app_hwa_adcbuf_ctrlRegs* ptrAppssAdcbufCtrlRegs = (CSL_app_hwa_adcbuf_ctrlRegs*)CSL_APP_HWA_ADCBUF_CTRL_U_BASE;

    if((flag & SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT) == SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT)
    {
        /* APSS Shared RAM0 (128KB) - Check Memory Initialization Status */
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE) != 1U){;}
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM0_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM0_MEM_INIT_DONE_HWASS_SHRD_RAM0_MEM_INIT_DONE_MEM_INIT_DONE, 1U);
    }

    if((flag & SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT) == SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT)
    { 
        /* APSS Shared RAM1 (128KB) - Check Memory Initialization Status */
        while (CSL_FEXT(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE) != 1U){;}
        CSL_FINS(ptrAppssCtrlRegs->HWASS_SHRD_RAM1_MEM_INIT_DONE, APP_CTRL_HWASS_SHRD_RAM1_MEM_INIT_DONE_HWASS_SHRD_RAM1_MEM_INIT_DONE_MEM_INIT_DONE, 1U);
    }

    if((flag & SOC_RCM_MEMINIT_FECSS_SHRAM_INIT ) == SOC_RCM_MEMINIT_FECSS_SHRAM_INIT )
    { 
        /* FECSS SHRAM (96KB) - Check Memory Initialization Status */
        while (CSL_FEXT(ptrFecssCtrlRegs->FECSS_SHARED_MEM_DONE, FEC_CTRL_FECSS_SHARED_MEM_DONE_FECSS_SHARED_MEM_DONE_MEM_INIT_DONE) != 1U){;}
        CSL_FINS(ptrFecssCtrlRegs->FECSS_SHARED_MEM_DONE, FEC_CTRL_FECSS_SHARED_MEM_DONE_FECSS_SHARED_MEM_DONE_MEM_INIT_DONE, 1U);
    }

    if((flag & SOC_RCM_MEMINIT_HWA_SHRAM_INIT) == SOC_RCM_MEMINIT_HWA_SHRAM_INIT)
    {
        /* HWA Shared RAM0 (160KB) - Check Memory Initialization Status */
        while (CSL_FEXT(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_DONE, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_DONE_HWASS_SHRD_RAM_MEM_DONE_MEM_INIT_DONE) != 1U){;}
        CSL_FINS(ptrAppssAdcbufCtrlRegs->HWASS_SHRD_RAM_MEM_DONE, APP_HWA_ADCBUF_CTRL_HWASS_SHRD_RAM_MEM_DONE_HWASS_SHRD_RAM_MEM_DONE_MEM_INIT_DONE, 1U);
    }

    return;
}

void SOC_rcmStartMemInitTpcc(uint16_t flag)
{
    CSL_app_ctrlRegs* ptrAppssCtrlRegs = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;
    
    if ((flag & SOC_RCM_MEMINIT_TPCCA_INIT)==SOC_RCM_MEMINIT_TPCCA_INIT)
        {   
            /* Start Memory Init - TPCC A */
            CSL_FINS(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_START, APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_A_MEMINIT_START, 1U);
        }
    if ((flag & SOC_RCM_MEMINIT_TPCCB_INIT)==SOC_RCM_MEMINIT_TPCCB_INIT)
    {   
        /* Start Memory Init - TPCC B */
        CSL_FINS(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_START, APP_CTRL_APPSS_TPCC_MEMINIT_START_APPSS_TPCC_MEMINIT_START_TPCC_B_MEMINIT_START, 1U);
    }

    return;
}

void SOC_rcmWaitMemInitTpcc(uint16_t flag)
{
    CSL_app_ctrlRegs* ptrAppssCtrlRegs = (CSL_app_ctrlRegs*)CSL_APP_CTRL_U_BASE;

    if ((flag & SOC_RCM_MEMINIT_TPCCA_INIT)==SOC_RCM_MEMINIT_TPCCA_INIT)
        {   
            /* Wait for mem init complete - TPCC A */
            while (CSL_FEXT(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_DONE, APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE) != 1U){;}
        }

    if ((flag & SOC_RCM_MEMINIT_TPCCB_INIT)==SOC_RCM_MEMINIT_TPCCB_INIT)
    {
        /* Wait for mem init complete - TPCC B */
        while (CSL_FEXT(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_DONE, APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE) != 1U){;}
    }

    /* Clear Done bit - TPCC A */
    CSL_FINS(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_DONE, APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_A_MEMINIT_DONE, 1U);
    /* Clear Done bit - TPCC B */
    CSL_FINS(ptrAppssCtrlRegs->APPSS_TPCC_MEMINIT_DONE, APP_CTRL_APPSS_TPCC_MEMINIT_DONE_APPSS_TPCC_MEMINIT_DONE_TPCC_B_MEMINIT_DONE, 1U);
    
    
    return;
}

uint8_t SOC_getEfuseRomVersion(void)
{
    CSL_top_efuseRegs *ptrTopEfuseregs;
    uint8_t efuseRomVersion = 0U;

    ptrTopEfuseregs = SOC_rcmGetBaseAddressTOPEFUSE();

    efuseRomVersion = SOC_rcmExtract8 (ptrTopEfuseregs->EFUSE0_ROW_18,
                                  SOC_RCM_EFUSE_ROM_VER_END_BIT,
                                  SOC_RCM_EFUSE_ROM_VER_START_BIT);

    return efuseRomVersion;
}

uint8_t SOC_rcmReadSynthTrimValid(void)
{
    CSL_top_efuseRegs *ptrTopEfuseregs;

    ptrTopEfuseregs = SOC_rcmGetBaseAddressTOPEFUSE();

    return SOC_rcmReadSynthTrimValidEfuse(ptrTopEfuseregs);
}

uint8_t SOC_rcmReadAPLLCalibTrimValid(void)
{
    CSL_top_efuseRegs *ptrTopEfuseregs;

    ptrTopEfuseregs = SOC_rcmGetBaseAddressTOPEFUSE();

    return SOC_rcmReadApllCalibTrimValidEfuse(ptrTopEfuseregs);
}

uint8_t SOC_getEfusePgVersion(void)
{
    CSL_top_efuseRegs *ptrTopEfuseregs;
    uint8_t efusePgVersion = 0U;

    ptrTopEfuseregs = SOC_rcmGetBaseAddressTOPEFUSE();

    efusePgVersion = SOC_rcmExtract8 (ptrTopEfuseregs->EFUSE0_ROW_18,
                                  SOC_RCM_PG_VER_END_BIT,
                                  SOC_RCM_PG_VER_START_BIT);

    return efusePgVersion;
}

uint8_t SOC_isDeviceAOP(void)
{
    CSL_top_efuseRegs *ptrTopEfuseregs;
    uint8_t efuseAnttype = 0U;

    ptrTopEfuseregs = SOC_rcmGetBaseAddressTOPEFUSE();

    efuseAnttype = SOC_rcmExtract8 (ptrTopEfuseregs->EFUSE0_ROW_15,
                                  SOC_RCM_ANT_TYPE_END_BIT,
                                  SOC_RCM_ANT_TYPE_START_BIT);

    return efuseAnttype;
}