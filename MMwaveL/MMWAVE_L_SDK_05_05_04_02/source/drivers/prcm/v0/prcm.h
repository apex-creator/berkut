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
//  prcm.h
//
//  Prototypes for the PRCM control driver.
//
//*****************************************************************************

#ifndef __PRCM_H__
#define __PRCM_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Peripheral clock and reset control registers
//
//*****************************************************************************
typedef struct _PRCM_PeripheralRegs_
{

unsigned long ulClkGateReg;
unsigned short ulClkGateRegMask;

unsigned long ulClkStatReg;

unsigned long ulClkRstReg;
unsigned short ulClkRstRegMask;


}PRCM_PeriphRegs_t;

//*****************************************************************************
// Values that can be passed to PRCMPeripheralEnable() and
// PRCMPeripheralDisable()
//*****************************************************************************
#define PRCM_GATE_CLK_ENABLE     0x00000000U
#define PRCM_GATE_CLK_DISABLE    0x00000007U

#define CSL_APP_RCM_APP_PERIPHERAL_CLKCTL_GATE_MASK            (0x0000000FU)
#define CSL_APP_RCM_APP_PERIPHERAL_CLKCTL_GATE_SHIFT           (0x00000000U)

#define CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRCLK_MASK        (0x00000FF0U)
#define CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRCLK_SHIFT       (0x00000004U)

#define CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRDIVR_MASK       (0x0000000FU)
#define CSL_APP_RCM_APP_PERIPHERAL_CLKSTAT_CURRDIVR_SHIFT      (0x00000000U)

#define CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_ADDR                (0xE000ED10U)
#define CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP_MASK      (0x00000004U)
#define CSL_ARM_M4_SYSTEM_CONTROL_REGISTER_SLEEPDEEP_SHIFT     (0x00000002U)

//*****************************************************************************
// Values that can be passed to PRCMSetSRAMRetention() and
// as ulSramColSel.
//*****************************************************************************
#define PRCM_APP_PD_SRAM_CLUSTER_1           (0x00000001U)
#define PRCM_APP_PD_SRAM_CLUSTER_2           (0x00000002U)
#define PRCM_APP_PD_SRAM_CLUSTER_3           (0x00000004U)
#define PRCM_APP_PD_SRAM_CLUSTER_4           (0x00000008U)
#define PRCM_APP_PD_SRAM_CLUSTER_5           (0x00000010U)
#define PRCM_APP_PD_SRAM_CLUSTER_6           (0x00000020U)

#define PRCM_FEC_PD_SRAM_CLUSTER_1           (0x00000040U)
#define PRCM_FEC_PD_SRAM_CLUSTER_2           (0x00000080U)
#define PRCM_FEC_PD_SRAM_CLUSTER_3           (0x00000100U)

#define PRCM_HWA_PD_SRAM_CLUSTER_1           (0x00000200U)
#define PRCM_HWA_PD_SRAM_CLUSTER_2           (0x00000400U)
#define PRCM_HWA_PD_SRAM_CLUSTER_3           (0x00000800U)


//*****************************************************************************
// Values that can be passed to PRCMSetSRAMRetention() and
// as ulModeFlags.
//*****************************************************************************
#define PRCM_SRAM_LPDS_RET        0x00000002U

//*****************************************************************************
// Values that can be passed to PRCMLPDSWakeupSourceEnable(),
// PRCMLPDSWakeupCauseGet() and PRCMLPDSWakeupSourceDisable().
//*****************************************************************************

/*
* It selects the wakeup source from deep sleep/sleep state of the device
* Bit 0 -> Sleep counter
* Bit 1 -> UART RX
* Bit 2 -> SPI CS
* Bit 3 -> GPIO or SYNCIN IO depends on the wakeup_io_mux_sel register
* Bit 4 -> RTC counter
* Bit 5 -> FRC frame start intr (Use only for device SLEEP wakeup)
*/

#define PRCM_LPDS_SLEEP_COUNTER           (0x00000001U)
#define PRCM_LPDS_UART_RX                 (0x00000002U)
#define PRCM_LPDS_SPI_CHIP_SELECT         (0x00000004U)
#define PRCM_LPDS_GPIO_SYNC_IO            (0x00000008U)
#define PRCM_LPDS_RTC_COUNTER             (0x00000010U)
#define PRCM_LPDS_FRC_FRAME_START_INT     (0x00000020U)
#define PRCM_LPDS_ALL_WAKEUPS             (0x0000003FU)

//*****************************************************************************
// Values that can be passed to PRCMLPDSWakeUpGPIOSelect() as Type
//*****************************************************************************
#define PRCM_LPDS_LOW_LEVEL       0x00000003
#define PRCM_LPDS_HIGH_LEVEL      0x00000004
#define PRCM_LPDS_FALL_EDGE       0x00000001
#define PRCM_LPDS_RISE_EDGE       0x00000000

//*****************************************************************************
// Values that can be passed to PRCMLPDSWakeUpGPIOSelect()
//*****************************************************************************
#define PRCM_LPDS_GPIO2           0x00000000
#define PRCM_LPDS_GPIO4           0x00000001
#define PRCM_LPDS_GPIO13          0x00000002
#define PRCM_LPDS_GPIO17          0x00000003
#define PRCM_LPDS_GPIO11          0x00000004
#define PRCM_LPDS_GPIO24          0x00000005
#define PRCM_LPDS_GPIO26          0x00000006

//*****************************************************************************
// Values that will be returned from PRCMSysResetCauseGet().
//*****************************************************************************
#define PRCM_POWER_ON             0x00000000
#define PRCM_LPDS_EXIT            0x00000001
#define PRCM_CORE_RESET           0x00000003
#define PRCM_MCU_RESET            0x00000004
#define PRCM_WDT_RESET            0x00000005
#define PRCM_SOC_RESET            0x00000006
#define PRCM_HIB_EXIT             0x00000007

//*****************************************************************************
// Values that can be passed to PRCMSEnableInterrupt
//*****************************************************************************
#define PRCM_INT_SLOW_CLK_CTR     0x00004000

//*****************************************************************************
// Values that can be passed to PRCMPeripheralClkEnable(),
// PRCMPeripheralClkDisable(), PRCMPeripheralReset()
//*****************************************************************************
#define PRCM_CAN                  0x00000000
#define PRCM_SPI                  0x00000001
#define PRCM_QSPI                 0x00000002
#define PRCM_TOPSS                0x00000003
#define PRCM_APP_RTI              0x00000004
#define PRCM_APP_WD               0x00000005U
#define PRCM_APP_UART0            0x00000006
#define PRCM_APP_UART1            0x00000007
#define PRCM_I2C                  0x00000008
#define PRCM_LIN                  0x00000009
#define PRCM_ESM                  0x0000000A
#define PRCM_EDMA                 0x0000000B
#define PRCM_CRC                  0x0000000C
#define PRCM_PWM                  0x0000000D
#define PRCM_GIO                  0x0000000E
#define PRCM_HWASS                0x0000000F

//*****************************************************************************
// Pre-defined helper macros
//*****************************************************************************


//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
extern void PRCMMCUReset(Bool bIncludeSubsystem);
extern unsigned long PRCMSysResetCauseGet(void);

extern void PRCMPeripheralClkEnable(unsigned long ulPeripheral,
                                    unsigned long ulClkFlags);
extern void PRCMPeripheralClkDisable(unsigned long ulPeripheral,
                                     unsigned long ulClkFlags);
extern void PRCMPeripheralReset(unsigned long ulPeripheral);
extern Bool PRCMPeripheralStatusGet(unsigned long ulPeripheral);

extern unsigned long PRCMPeripheralClockGet(unsigned long ulPeripheral);

extern void PRCMSleepEnter(void);

extern void PRCMSetSRAMRetention(unsigned long ulSramColSel,
                                    unsigned long ulFlags);
extern void PRCMLPDSRestoreInfoSet(unsigned long ulRestorePC);
extern void PRCMLPDSEnter(void);
extern void PRCMLPDSIntervalSet(unsigned long ulTicks);
extern void PRCMLPDSWakeupSourceEnable(unsigned long ulLpdsWakeupSrc);
extern unsigned long PRCMLPDSWakeupCauseGet(void);
extern void PRCMLPDSWakeUpGPIOSelect(unsigned long ulType);
extern void PRCMLPDSWakeUpSYNCIOSelect(unsigned long ulType);
extern void PRCMLPDSWakeUpUartRxSelect(unsigned long ulType);
extern void PRCMLPDSWakeUpSPISelect(unsigned long ulType);
extern void PRCMLPDSWakeupSourceDisable(unsigned long ulLpdsWakeupSrc);
extern unsigned long long PRCMSlowClkCtrGet(void);
extern void PRCMIntRegister(void (*pfnHandler)(void));
extern void PRCMIntUnregister(void);
extern void PRCMIntEnable(unsigned long ulIntFlags);
extern void PRCMIntDisable(unsigned long ulIntFlags);
extern unsigned long PRCMIntStatus(void);
extern void PRCMRTCInUseSet(void);
extern Bool PRCMRTCInUseGet(void);
extern void PRCMRTCSet(unsigned long ulSecs, unsigned short usMsec);
extern void PRCMRTCGet(unsigned long *ulSecs, unsigned short *usMsec);
extern void PRCMRTCMatchSet(unsigned long ulSecs, unsigned short usMsec);
extern void PRCMRTCMatchGet(unsigned long *ulSecs, unsigned short *usMsec);
extern void PRCMMCUInit(void);
extern void PRCMLPDSEnterKeepDebugIf(void);
extern void PRCMSleepEnterKeepDebugIf(void);
extern void PRCMSetPSCONDFTRTAOverride(Bool dftrtagood_ovrd, Bool dftrtagood_ovrd_val, Bool dftrtaon_ovrd,Bool dftrtaon_ovrd_val);
extern Bool PRCMIsRdrStateIdle(void);



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __PRCM_H__
