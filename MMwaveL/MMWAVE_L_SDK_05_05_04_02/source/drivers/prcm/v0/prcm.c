/*
 *  Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/ 
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
 *  
 */

//*****************************************************************************
//
//! \addtogroup PRCM_Power_Reset_Clock_Module_api
//! @{
//
//*****************************************************************************
#include <stdbool.h>
#include <drivers/hw_include/hw_types.h>
//#include "inc/hw_ints.h"
#include <drivers/hw_include/xwrL64xx/cslr_soc_baseaddress.h>
#include <drivers/hw_include/xwrL64xx/cslr_app_rcm.h>
#include <drivers/hw_include/cslr_prcm.h>
#include <drivers/prcm.h>
#include <drivers/pinmux.h>
//#include <kernel/dpl/Hwip.h>
#include <board/cpu.h>
#include <drivers/hw_include/cslr_soc.h>


//*****************************************************************************
// Macro definition
//*****************************************************************************
#define PRCM_SOFT_RESET           0x00000001
#define PRCM_ENABLE_STATUS        0x00000002


#define SYS_CLK_FREQ                   (80000000U)
#define XTAL_CLK_FREQ                  (40000000U)
#define XTAL_CLKX2_FREQ                (80000000U)
#define PLL_CLK_FREQ                   (160000000U)

/* Current Clock selected by GCM Clock Mux */
#define XTALCLK    (0x1U)
#define XTALCLKX2  (0x2U)
#define MDLL       (0x4U)
#define APLL_DPLL  (0x8U)
#define RCCLK      (0x10U)

/* PC_REGISTER2 separate SP & PC indication to RBL*/

#define CSL_TOP_PRCM_PC_REGISTER2_PC_REGISTER2_SEPARATE_SP_N_PC_MASK               (0x0E000000U)
#define CSL_TOP_PRCM_PC_REGISTER2_PC_REGISTER2_SEPARATE_SP_N_PC_SHIFT              (0x00000019U)

//*****************************************************************************
// Global Peripheral clock and rest Registers
//*****************************************************************************
static const PRCM_PeriphRegs_t PRCM_PeriphRegsList[] =
{
    { CSL_APP_RCM_APP_CAN_CLKCTL,0, CSL_APP_RCM_APP_CAN_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_SPI_CLKCTL,0, CSL_APP_RCM_APP_SPI_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_QSPI_CLKCTL,0, CSL_APP_RCM_APP_QSPI_CLKSTAT,0,0},
    { CSL_APP_RCM_TOPSS_CLKCTL,0, CSL_APP_RCM_TOPSS_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_RTI_CLKCTL,0, CSL_APP_RCM_APP_RTI_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_WD_CLKCTL,0, CSL_APP_RCM_APP_WD_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_UART_0_CLKCTL,0, CSL_APP_RCM_APP_UART_0_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_UART_1_CLKCTL,0, CSL_APP_RCM_APP_UART_1_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_I2C_CLKCTL,0, CSL_APP_RCM_APP_I2C_CLKSTAT,0,0},
    { CSL_APP_RCM_APP_LIN_CLKCTL,0, CSL_APP_RCM_APP_LIN_CLKSTAT,0,0}
};
//*****************************************************************************
void PRCM_asmWfi();
//*****************************************************************************
void PRCM_asmWfi()
{
  __asm("    wfi\n");
}
//*****************************************************************************
//
//! Performs a software reset of a MCU and associated peripherals
//!
//! \param bIncludeSubsystem is \b true to reset associated peripherals.
//!
//! This function performs a software reset of a MCU and associated peripherals.
//! To reset the associated peripheral, the parameter \e bIncludeSubsystem
//! should be set to \b true.
//!
//! \return None.
//
//*****************************************************************************
void PRCMMCUReset(Bool bIncludeSubsystem)
{
    uint32_t regVal;
    uint32_t loop = 1U;
  if((bIncludeSubsystem) != 0U)
  {
    //
    // Reset Apps processor and associated peripheral
    //
    regVal = HW_RD_REG32( CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RST_APP_PD_SOFT_RESET);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_RST_APP_PD_SOFT_RESET_RST_APP_PD_SOFT_RESET_APP_PD_WARM_RESET_REQN, 1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RST_APP_PD_SOFT_RESET, regVal);
  }
  else
  {
    //
    // Reset Apps processor only
    //
      regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RST_SOFT_APP_CORE_SYSRESET_REQ);
      HW_SET_FIELD32(regVal, CSL_TOP_PRCM_RST_SOFT_APP_CORE_SYSRESET_REQ_RST_SOFT_APP_CORE_SYSRESET_REQ_APP_PD_CORE_RESET_REQN, 1);
      HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RST_SOFT_APP_CORE_SYSRESET_REQ, regVal);
  }

  //
  // Wait for system to enter hibernate
  //
  PRCM_asmWfi();

  //
  // Infinite loop
  //
  while(loop != 0U)
  {

  }
}

//*****************************************************************************
//
//! Gets the reason for a reset.
//!
//! This function returns the reason(s) for a reset. The reset reason are:-
//! -\b PRCM_POWER_ON  - Device is powering up.
//! -\b PRCM_LPDS_EXIT - Device is exiting from LPDS.
//! -\b PRCM_CORE_RESET - Device is exiting soft core only reset
//! -\b PRCM_MCU_RESET - Device is exiting soft subsystem reset.
//! -\b PRCM_WDT_RESET - Device was reset by watchdog.
//! -\b PRCM_SOC_RESET - Device is exting SOC reset.
//! -\b PRCM_HIB_EXIT - Device is exiting hibernate.
//!
//! \return Returns one of the cause defined above.
//
//*****************************************************************************
unsigned long PRCMSysResetCauseGet()
{
    unsigned long regVal;
    unsigned long ulWakeupStatus;

  //
  // Read the Reset status
  //
  regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_RST_CAUSE);
  ulWakeupStatus = HW_GET_FIELD(regVal, CSL_APP_RCM_RST_CAUSE_RST_CAUSE_COMMON);

  //
  // Return status.
  //
  return ulWakeupStatus;
}

//*****************************************************************************
//
//! Enable clock(s) to peripheral.
//!
//! \param ulPeripheral is one of the valid peripherals
//! \param ulClkFlags are bitmask of clock(s) to be enabled.
//!
//! This function enables the clock for the specified peripheral. Peripherals
//! are by default clock gated (disabled) and generates a bus fault if
//! accessed.
//!
//! The parameter \e ulClkFlags can be logical OR of the following:
//! -\b PRCM_RUN_MODE_CLK - Ungates clock to the peripheral
//! -\b PRCM_SLP_MODE_CLK - Keeps the clocks ungated in sleep.
//!
//! \return None.
//
//*****************************************************************************
void
PRCMPeripheralClkEnable(unsigned long ulPeripheral, unsigned long ulClkFlags)
{
    uint32_t regVal;
  //
  // Enable the specified peripheral clocks, Nothing to be done for PRCM_ADC
  // as it is a dummy define for pinmux utility code generation
  //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + PRCM_PeriphRegsList[ulPeripheral].ulClkGateReg);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_PERIPHERAL_CLKCTL_GATE,ulClkFlags );
    HW_WR_REG32(CSL_APP_RCM_U_BASE + PRCM_PeriphRegsList[ulPeripheral].ulClkGateReg, regVal);
}

//*****************************************************************************
//
//! Disables clock(s) to peripheral.
//!
//! \param ulPeripheral is one of the valid peripherals
//! \param ulClkFlags are bitmask of clock(s) to be enabled.
//!
//! This function disable the clock for the specified peripheral. Peripherals
//! are by default clock gated (disabled) and generated a bus fault if
//! accessed.
//!
//! The parameter \e ulClkFlags can be logical OR bit fields as defined in
//! PRCMEnablePeripheral().
//!
//! \return None.
//
//*****************************************************************************
void
PRCMPeripheralClkDisable(unsigned long ulPeripheral, unsigned long ulClkFlags)
{
    uint32_t regVal;
  //
  // Disable the specified peripheral clocks
  //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + PRCM_PeriphRegsList[ulPeripheral].ulClkGateReg);
    HW_SET_FIELD32(regVal, CSL_APP_RCM_APP_PERIPHERAL_CLKCTL_GATE,ulClkFlags );
    HW_WR_REG32(CSL_APP_RCM_U_BASE + PRCM_PeriphRegsList[ulPeripheral].ulClkGateReg, regVal);
}

//*****************************************************************************
//
//! Gets the input clock for the specified peripheral.
//!
//! \param ulPeripheral is one of the valid peripherals.
//!
//! This function gets the input clock for the specified peripheral.
//!
//! The parameter \e ulPeripheral has the same definition as that in
//! PRCMPeripheralClkEnable();
//!
//! \return Returns input clock frequency for specified peripheral.
//
//*****************************************************************************
unsigned long
PRCMPeripheralClockGet(unsigned long ulPeripheral)
{

    uint32_t regVal;
    uint8_t currClkVal;
    uint8_t currDiv;
    uint32_t clockFreq;

  //
  // Extract the currclk value of the peripheral
  //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + PRCM_PeriphRegsList[ulPeripheral].ulClkStatReg);
    currClkVal = HW_GET_FIELD(regVal,CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRCLK);
    currDiv = HW_GET_FIELD(regVal,CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRDIVR);

    if(PRCM_APP_WD >= ulPeripheral)
    {
        if(XTALCLK == currClkVal)
        {
            clockFreq = (XTAL_CLK_FREQ/(currDiv + 1U));
        }
        else if(XTALCLKX2 == currClkVal)
        {
            clockFreq = (XTAL_CLKX2_FREQ/(currDiv + 1U));
        }
        else if(MDLL == currClkVal)
        {
            clockFreq = (PLL_CLK_FREQ/(currDiv + 1U));
        }
        else if(APLL_DPLL == currClkVal)
        {
            clockFreq = (PLL_CLK_FREQ/(currDiv + 1U));
        }
        else if(RCCLK == currClkVal)
        {
            clockFreq = (PLL_CLK_FREQ/(currDiv + 1U));
        }
        else
        {
            clockFreq = (XTAL_CLK_FREQ/(currDiv + 1U));
        }
    }
    else
    {
        clockFreq = (XTAL_CLK_FREQ/(currDiv + 1U));
    }

  //
  // Return the clock rate.
  //
  return clockFreq;
}

//*****************************************************************************
//
//! Performs a software reset of a peripheral.
//!
//! \param ulPeripheral is one of the valid peripheral.
//!
//! This function does soft reset of the specified peripheral
//!
//! \return None.
//
//*****************************************************************************
void
PRCMPeripheralReset(unsigned long ulPeripheral)
{
// TODO: BLOCKRESET Registers
}

//*****************************************************************************
//
//! Determines if a peripheral is ready.
//!
//! \param ulPeripheral is one of the valid modules
//!
//! This function determines if a particular peripheral is ready to be
//! accessed. The peripheral may be in a non-ready state if it is not enabled,
//! is being held in reset, or is in the process of becoming ready after being
//! enabled or taken out of reset.
//!
//! \return Returns \b true if the  peripheral is ready, \b false otherwise.
//
//*****************************************************************************
Bool
PRCMPeripheralStatusGet(unsigned long ulPeripheral)
{
    // TODO: BLOCKRESET Registers
  return (TRUE);
}

//*****************************************************************************
//
//! Sets the LPDS exit PC and SP restore vlaues.
//!
//! \param ulStackPtr is the SP restore value.
//! \param ulProgCntr is the PC restore value
//!
//! This function sets the LPDS exit PC and SP restore vlaues. Setting
//! \e ulProgCntr to a non-zero value, forces bootloader to jump to that
//! address with Stack Pointer initialized to \e ulStackPtr on LPDS exit,
//! otherwise the application's vector table entries are used.
//!
//! \return None.
//
//*****************************************************************************
void
PRCMLPDSRestoreInfoSet(unsigned long ulProgCntr)
{
    uint32_t regVal, parityVal;
    uint8_t bitIndex;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER2);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PC_REGISTER2_PC_REGISTER2_SEPARATE_SP_N_PC , 0x0U);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER2, regVal);

    //
    // Set The PC Value
    //
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER3, ulProgCntr);


/******************************************************************************/

    /* Parity for Image Length/ Program Counter - PCR3 */
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER3);
    parityVal = 0U;
    for (bitIndex = 0U; bitIndex < 32U; bitIndex = bitIndex + 4U)
    {
        parityVal = parityVal ^ (uint8_t)((regVal >> bitIndex) & 0xFU);
    }

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER1);
    regVal = regVal & ~(0xFU<<16U);
    regVal = regVal | (parityVal << 16U);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PC_REGISTER1, regVal);
}

//*****************************************************************************
//
//! Puts the system into Low Power Deep Sleep (LPDS) power mode.
//!
//! This function puts the system into Low Power Deep Sleep (LPDS) power mode.
//! A call to this function never returns and the execution starts from Reset.
//! \sa PRCMLPDSRestoreInfoSet().
//!
//! \return None.
//!
//! \note  External debugger will always disconnect whenever the system
//!  enters LPDS and debug interface is shutdown until next POR reset. In order
//!  to avoid this and allow for connecting back the debugger after waking up
//!  from LPDS \sa PRCMLPDSEnterKeepDebugIf().
//!
//
//*****************************************************************************
void
PRCMLPDSEnter()
{
    uint32_t regVal;
    uint32_t loop = 1U;

    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL, 0x000); // CPU CLK Switch to XTAL
    HW_WR_REG32(0x5B040004, 0x000); // Disable PLL DIG

    //
    // Disable Test PD
    //

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    //
    // Request Device LPDS Entry
    //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
    HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP, 0x1);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

    // ARM system control register
    regVal = HW_RD_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR);
    HW_SET_FIELD32(regVal,CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP, 0x1);
    HW_WR_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR, regVal);

    //
    // Wait for system to enter LPDS
    //
    PRCM_asmWfi();

    //
    // Infinite loop
    //
    while(loop != 0U)
    {

    }

}

//*****************************************************************************
//
//! Puts the system into Low Power Deep Sleep (LPDS) power mode.
//!
//! This function puts the system into Low Power Deep Sleep (LPDS) power mode.
//! A call to this function never returns and the execution starts from Reset.
//! \sa PRCMLPDSRestoreInfoSet().
//!
//! \return None.
//!
//! \note  External debugger will always disconnect whenever the system
//!  enters LPDS and debug interface is shutdown until next POR reset. In order
//!  to avoid this and allow for connecting back the debugger after waking up
//!  from LPDS \sa PRCMLPDSEnterKeepDebugIf().
//!
//
//*****************************************************************************
void
PRCMSleepEnter()
{
    uint32_t regVal;
    uint32_t loop = 1U;

    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL, 0x000); // CPU CLK Switch to XTAL
    HW_WR_REG32(0x5B040004, 0x000); // Disable PLL DIG

    //
    // Disable Test PD
    //

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_OV_TEST_DBG_PD_IS_SLEEP, 0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    //
    // Request Device LPDS Entry
    //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
    HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP, 0x1);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

    // ARM system control register
    regVal = HW_RD_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR);
    HW_SET_FIELD32(regVal,CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP, 0x1);
    HW_WR_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR, regVal);

    //
    // Wait for system to enter LPDS
    //
    PRCM_asmWfi();

    //
    // Infinite loop
    //
    while(loop != 0U)
    {

    }

}


//*****************************************************************************
//
//! Puts the system into Low Power Deep Sleep (LPDS) power mode keeping
//! debug interface alive.
//!
//! This function puts the system into Low Power Deep Sleep (LPDS) power mode
//! keeping debug interface alive. A call to this function never returns and the
//! execution starts from Reset \sa PRCMLPDSRestoreInfoSet().
//!
//! \return None.
//!
//! \note External debugger will always disconnect whenever the system
//!  enters LPDS, using this API will allow connecting back the debugger after
//!  waking up from LPDS. This API is recommended for development purposes
//!  only as it adds to the current consumption of the system.
//!
//
//*****************************************************************************
void
PRCMLPDSEnterKeepDebugIf()
{
    uint32_t regVal;
    uint32_t loop = 1U;

    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL, 0x000); // CPU CLK Switch to XTAL
    HW_WR_REG32(0x5B040004, 0x000); // Disable PLL DIG

    //
    // Keep Test PD Active
    // Shift control to Ice-Melter
    //
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    //
    // Request Device LPDS Entry
    //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
    HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_DEEPSLEEP, 0x1);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

    // ARM system control register
    regVal = HW_RD_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR);
    HW_SET_FIELD32(regVal,CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP, 0x1);
    HW_WR_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR, regVal);

    //
    // Wait for system to enter LPDS
    //
    PRCM_asmWfi();

    //
    // Infinite loop
    //
    while(loop != 0U)
    {

    }
}

//*****************************************************************************
//
//! Puts the system into Low Power Deep Sleep (LPDS) power mode keeping
//! debug interface alive.
//!
//! This function puts the system into Low Power Deep Sleep (LPDS) power mode
//! keeping debug interface alive. A call to this function never returns and the
//! execution starts from Reset \sa PRCMLPDSRestoreInfoSet().
//!
//! \return None.
//!
//! \note External debugger will always disconnect whenever the system
//!  enters LPDS, using this API will allow connecting back the debugger after
//!  waking up from LPDS. This API is recommended for development purposes
//!  only as it adds to the current consumption of the system.
//!
//
//*****************************************************************************

void
PRCMSleepEnterKeepDebugIf()
{
    uint32_t regVal;
    uint32_t loop = 1U;

    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_APP_CPU_CLKCTL, 0x000); // CPU CLK Switch to XTAL
    HW_WR_REG32(0x5B040004, 0x000); // Disable PLL DIG

    //
    // Keep Test PD Active
    // Shift control to Ice-Melter
    //
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN);
    HW_SET_FIELD32(regVal,CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN_PSCON_TEST_DBG_PD_EN_SEL_OV_TEST_DBG_PD_IS_SLEEP, 0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_TEST_DBG_PD_EN, regVal);

    //
    // Request Device LPDS Entry
    //
    regVal = HW_RD_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE);
    HW_SET_FIELD32(regVal,CSL_APP_RCM_POWERMODE_POWERMODE_SLEEP, 0x1);
    HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_POWERMODE, regVal);

    // ARM system control register
    regVal = HW_RD_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR);
    HW_SET_FIELD32(regVal,CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP, 0x1);
    HW_WR_REG32(CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR, regVal);

    //
    // Wait for system to enter LPDS
    //
    PRCM_asmWfi();

    //
    // Infinite loop
    //
    while(loop != 0U)
    {

    }
}

//*****************************************************************************
//
//! Enable the individual LPDS wakeup source(s).
//!
//! \param ulLpdsWakeupSrc is logical OR of wakeup sources.
//!
//! This function enable the individual LPDS wakeup source(s) and following
//! three wakeup sources (\e ulLpdsWakeupSrc ) are supported by the device.
//! -\b PRCM_LPDS_HOST_IRQ
//! -\b PRCM_LPDS_GPIO
//! -\b PRCM_LPDS_TIMER
//!
//! \return None.
//
//*****************************************************************************
void
PRCMLPDSWakeupSourceEnable(unsigned long ulLpdsWakeupSrc)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    regVal |= (ulLpdsWakeupSrc);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);
}

//*****************************************************************************
//
//! Disable the individual LPDS wakeup source(s).
//!
//! \param ulLpdsWakeupSrc is logical OR of wakeup sources.
//!
//! This function enable the individual LPDS wakeup source(s) and following
//! three wake up sources (\e ulLpdsWakeupSrc ) are supported by the device.
//! -\b PRCM_LPDS_HOST_IRQ
//! -\b PRCM_LPDS_GPIO
//! -\b PRCM_LPDS_TIMER
//!
//! \return None.
//
//*****************************************************************************
void
PRCMLPDSWakeupSourceDisable(unsigned long ulLpdsWakeupSrc)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    regVal &= ~(ulLpdsWakeupSrc);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);

}


//*****************************************************************************
//
//! Get LPDS wakeup cause
//!
//! This function gets LPDS wakeup caouse
//!
//! \return Returns values enumerated as described in
//! PRCMLPDSWakeupSourceEnable().
//
//*****************************************************************************
unsigned long
PRCMLPDSWakeupCauseGet()
{
    uint32_t regVal;
    uint32_t wkupCause;

   regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE+ CSL_TOP_PRCM_RADAR_WAKEUP_STATUS);
   wkupCause = HW_GET_FIELD(regVal, CSL_TOP_PRCM_RADAR_WAKEUP_STATUS_RADAR_WAKEUP_STATUS_WAKEUP_SOURCE);

   return (wkupCause);
}

//*****************************************************************************
//
//! Sets LPDS wakeup Timer
//!
//! \param ulTicks is number of 32.768 KHz clocks
//!
//! This function sets internal LPDS wakeup timer running at 32.768 KHz. The
//! timer is only configured if the parameter \e ulTicks is in valid range i.e.
//! from 21 to 2^32.
//!
//! \return Returns \b true on success, \b false otherwise.
//
//*****************************************************************************
void
PRCMLPDSIntervalSet(unsigned long ulTicks)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_SLEEP_COUNTER_END);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_SLEEP_COUNTER_END_SLEEP_COUNTER_END_SLEEP_COUNT_END,ulTicks);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_SLEEP_COUNTER_END, regVal);
}

//*****************************************************************************
//
//! Selects the GPIO for LPDS wakeup
//!
//! \param ulType is the wakeup trigger type.
//!
//! This function configures GPIO for LPDS wakeup.
//!
//! The parameter \e ulType sets the trigger type and can be one of the
//! following:
//! - \b PRCM_LPDS_FALL_EDGE
//! - \b PRCM_LPDS_RISE_EDGE
//!
//! \return None.
//
//*****************************************************************************
void
PRCMLPDSWakeUpGPIOSelect(unsigned long ulType)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL_WAKEUP_IO_MUX_SEL_WAKEUP_IO_MUX_SEL,0x0);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WU_SOURCE_EN_WU_SOURCE_EN_GPIO_INT_EDGE,ulType);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);
}

//*****************************************************************************
//
//! Selects the SYNC_IN for LPDS wakeup
//!
//! \param ulType is the wakeup trigger type.
//!
//! This function configures SYNC_IN for LPDS wakeup.
//!
//! The parameter \e ulType sets the trigger type and can be one of the
//! following:
//! - \b PRCM_LPDS_FALL_EDGE
//! - \b PRCM_LPDS_RISE_EDGE
//!
//! \return None.
//
//*****************************************************************************
void PRCMLPDSWakeUpSYNCIOSelect(unsigned long ulType)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL_WAKEUP_IO_MUX_SEL_WAKEUP_IO_MUX_SEL,0x1);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WAKEUP_IO_MUX_SEL, regVal);

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WU_SOURCE_EN_WU_SOURCE_EN_SYNCIN_IO_EDGE,ulType);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);
}

//*****************************************************************************
//
//! Selects the UART RX for LPDS wakeup
//!
//! \param ulType is the wakeup trigger type.
//!
//! This function configures UART RX for LPDS wakeup.
//!
//! The parameter \e ulType sets the trigger type and can be one of the
//! following:
//! - \b PRCM_LPDS_FALL_EDGE
//! - \b PRCM_LPDS_RISE_EDGE
//!
//! \return None.
//
//*****************************************************************************
void PRCMLPDSWakeUpUartRxSelect(unsigned long ulType)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WU_SOURCE_EN_WU_SOURCE_EN_UART_RX_EDGE,ulType);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);
}

//*****************************************************************************
//
//! Selects the SPI CS for LPDS wakeup
//!
//! \param ulType is the wakeup trigger type.
//!
//! This function configures SPI CS for LPDS wakeup.
//!
//! The parameter \e ulType sets the trigger type and can be one of the
//! following:
//! - \b PRCM_LPDS_FALL_EDGE
//! - \b PRCM_LPDS_RISE_EDGE
//!
//! \return None.
//
//*****************************************************************************
void PRCMLPDSWakeUpSPISelect(unsigned long ulType)
{
    uint32_t regVal;

    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN);
    HW_SET_FIELD32(regVal, CSL_TOP_PRCM_WU_SOURCE_EN_WU_SOURCE_EN_SPI_CS_EDGE,ulType);
    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_WU_SOURCE_EN, regVal);
}
//*****************************************************************************

//*****************************************************************************
//
//! Enable SRAM column retention during LPDS Power mode(s)
//!
//! \param ulSramColSel is bit mask of valid SRAM columns.
//! \param ulModeFlags is the bit mask of power modes.
//!
//! This functions enables the SRAM retention. The device supports configurable
//! SRAM column retention in Low Power Deep Sleep (LPDS). Each column is of
//! 64 KB size.
//!
//! The parameter \e ulSramColSel should be logical OR of the following:-
//! -\b PRCM_SRAM_COL_1
//! -\b PRCM_SRAM_COL_2
//! -\b PRCM_SRAM_COL_3
//! -\b PRCM_SRAM_COL_4
//!
//! The parameter \e ulModeFlags selects the power modes and sholud be logical
//! OR of one or more of the following
//! -\b PRCM_SRAM_LPDS_RET
//!
//! \return None.
//
//****************************************************************************
void
PRCMSetSRAMRetention(unsigned long ulSramColSel, unsigned long ulModeFlags)
{
    uint32_t regVal;
    int32_t mask;
    uint8_t value;
    if((ulModeFlags & PRCM_SRAM_LPDS_RET) != 0U)
    {
        // Control for RAM Memory Clusters 1,2,3 in APP PD
        mask = (PRCM_APP_PD_SRAM_CLUSTER_1 | PRCM_APP_PD_SRAM_CLUSTER_2 | PRCM_APP_PD_SRAM_CLUSTER_3);
        value = ulSramColSel & (unsigned long)mask;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_STATE);
        regVal = ((regVal & ~((unsigned long)mask)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_STATE, regVal);

        // Control for RAM Memory Clusters 4,5 in APP PD
        mask = (PRCM_APP_PD_SRAM_CLUSTER_4 | PRCM_APP_PD_SRAM_CLUSTER_5);
        value = (ulSramColSel & (unsigned long)mask) >> 3;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_GRP1_STATE);
        regVal = ((regVal & ~((unsigned long)mask >> 3)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_GRP1_STATE, regVal);

        // Control for RAM Memory Cluster 6 in APP PD
        mask = (PRCM_APP_PD_SRAM_CLUSTER_6);
        value = (ulSramColSel & (unsigned long)mask) >> 5;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_GRP2_STATE);
        regVal = ((regVal & ~((unsigned long)mask >> 5)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_APP_PD_RAM_GRP2_STATE, regVal);

        // Control for RAM Memory Cluster 1 in FEC PD
        mask = (PRCM_FEC_PD_SRAM_CLUSTER_1);
        value = (ulSramColSel & (unsigned long)mask) >> 6;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_STATE);
        regVal = ((regVal & ~((unsigned long)mask >> 6)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_STATE, regVal);

        // Control for RAM Memory Clusters 2,3 in FEC PD
        mask = (PRCM_FEC_PD_SRAM_CLUSTER_2 | PRCM_FEC_PD_SRAM_CLUSTER_3);
        value = (ulSramColSel & (unsigned long)mask) >> 7;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE);
        regVal = ((regVal & ~((unsigned long)mask >> 7)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_FEC_PD_RAM_GRP4_STATE, regVal);

        // Control for RAM Memory Clusters 1,2,3 in HWA PD
        mask = (PRCM_HWA_PD_SRAM_CLUSTER_1 | PRCM_HWA_PD_SRAM_CLUSTER_2 | PRCM_HWA_PD_SRAM_CLUSTER_3);
        value = (ulSramColSel & (unsigned long)mask) >> 9;
        regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_HWA_PD_RAM_GRP3_STATE);
        regVal = ((regVal & ~((unsigned long)mask >> 9)) | value);
        HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_HWA_PD_RAM_GRP3_STATE, regVal);
    }
}

//*****************************************************************************
//
//! Gets the current value of the internal slow clock counter
//!
//! This function latches and reads the internal RTC running at 32.768 Khz
//!
//! \return 64-bit current counter vlaue.
//
//*****************************************************************************
unsigned long long
PRCMSlowClkCtrGet()
{
  unsigned long long rtcVal;

  //
  // Read latched values as 2 32-bit vlaues
  //
  rtcVal  = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RTC_COUNT_MSB);
  rtcVal  = rtcVal << 32;
  rtcVal |= (unsigned long long)HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_RTC_COUNT_LSB);

  return rtcVal;
}

//*****************************************************************************
//
//! MCU Initialization Routine
//!
//! This function contains all the mandatory bug fixes, ECO enables,
//! initializations for 64XX.
//!
//! \note \b ###IMPORTANT### : This is a routine which should be one of the
//! first things to be executed after control comes to MCU Application code.
//!
//! \return None
//
//*****************************************************************************
void PRCMMCUInit()
{
    volatile int i=1;
    while(i != 0){;}
}

//*****************************************************************************
//!
//! This function shall configure PSCON_DFTRTA_OVERRIDE register for 
//! controlling dftrtagood and dftrtaon signal states of memories.
//!
//! \return None
//
//*****************************************************************************
void PRCMSetPSCONDFTRTAOverride(Bool dftrtagood_ovrd, Bool dftrtagood_ovrd_val, Bool dftrtaon_ovrd, Bool dftrtaon_ovrd_val)
{
    uint32_t regVal;
    regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE);
    regVal &= 0x0U;
    regVal = ((uint32_t)dftrtagood_ovrd << CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE_PSCON_DFTRTA_OVERRIDE_SEL_OV_DFTRTAGOOD_SHIFT) | \
             ((uint32_t)dftrtagood_ovrd_val << CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE_PSCON_DFTRTA_OVERRIDE_OV_DFTRTAGOOD_SHIFT) | \
             ((uint32_t)dftrtaon_ovrd << CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE_PSCON_DFTRTA_OVERRIDE_SEL_OV_DFTRTAON_SHIFT) | \
             ((uint32_t)dftrtaon_ovrd_val << CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE_PSCON_DFTRTA_OVERRIDE_OV_DFTRTAON_SHIFT);

    HW_WR_REG32(CSL_TOP_PRCM_U_BASE + CSL_TOP_PRCM_PSCON_DFTRTA_OVERRIDE, regVal);
}

//*****************************************************************************
//!
//! This function shall return true if Radar State is Idle, false otherwise
//
//*****************************************************************************
Bool PRCMIsRdrStateIdle()
{
    uint32_t regVal;
    Bool isIdle = FALSE;

   regVal = HW_RD_REG32(CSL_TOP_PRCM_U_BASE+ CSL_TOP_PRCM_RADAR_WAKEUP_STATUS);
   isIdle = (Bool) (HW_GET_FIELD(regVal, CSL_TOP_PRCM_RADAR_WAKEUP_STATUS_RADAR_WAKEUP_STATUS_RADAR_STATE_IS_IDLE));

   return (isIdle);
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
