/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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
 *  Name        : cslr_top_efuse.h
*/
#ifndef CSLR_TOP_EFUSE_H_
#define CSLR_TOP_EFUSE_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PID;
    volatile uint32_t EFUSE_DIEID0;
    volatile uint32_t EFUSE_DIEID1;
    volatile uint32_t EFUSE_DIEID2;
    volatile uint32_t EFUSE_DIEID3;
    volatile uint32_t EFUSE_DEVICE_TYPE;
    volatile uint32_t EFUSE_UID0;
    volatile uint32_t EFUSE_UID1;
    volatile uint32_t EFUSE_UID2;
    volatile uint32_t EFUSE_UID3;
    volatile uint32_t EFUSE_UID4;
    volatile uint32_t EFUSE0_ROW_15;
    volatile uint32_t EFUSE0_ROW_16;
    volatile uint32_t EFUSE0_ROW_17;
    volatile uint32_t EFUSE0_ROW_18;
    volatile uint32_t EFUSE0_ROW_19;
    volatile uint32_t EFUSE0_ROW_20;
    volatile uint32_t EFUSE0_ROW_21;
    volatile uint32_t EFUSE0_ROW_22;
    volatile uint32_t EFUSE0_ROW_23;
    volatile uint32_t EFUSE0_ROW_24;
    volatile uint32_t EFUSE0_ROW_25;
    volatile uint32_t EFUSE0_ROW_26;
    volatile uint32_t EFUSE0_ROW_27;
    volatile uint32_t EFUSE0_ROW_28;
    volatile uint32_t EFUSE0_ROW_29;
    volatile uint32_t EFUSE0_ROW_30;
    volatile uint32_t EFUSE0_ROW_31;
    volatile uint32_t EFUSE0_ROW_32;
    volatile uint32_t EFUSE0_ROW_33;
    volatile uint32_t EFUSE0_ROW_34;
    volatile uint32_t EFUSE0_ROW_35;
    volatile uint32_t EFUSE0_ROW_36;
    volatile uint32_t EFUSE0_ROW_37;
    volatile uint32_t EFUSE0_ROW_38;
    volatile uint32_t EFUSE0_ROW_39;
    volatile uint32_t EFUSE0_ROW_40;
    volatile uint32_t EFUSE0_ROW_41;
    volatile uint32_t EFUSE0_ROW_42;
    volatile uint32_t EFUSE0_ROW_43;
    volatile uint32_t EFUSE0_ROW_44;
    volatile uint32_t EFUSE0_ROW_45;
    volatile uint32_t EFUSE0_ROW_46;
    volatile uint32_t EFUSE0_ROW_47;
    volatile uint32_t EFUSE0_ROW_48;
    volatile uint32_t EFUSE0_ROW_49;
    volatile uint32_t EFUSE0_ROW_50;
    volatile uint32_t EFUSE0_ROW_51;
    volatile uint32_t EFUSE0_ROW_52;
    volatile uint32_t EFUSE0_ROW_53;
    volatile uint32_t EFUSE0_ROW_54;
    volatile uint32_t EFUSE0_ROW_55;
    volatile uint32_t EFUSE0_ROW_56;
    volatile uint32_t EFUSE0_ROW_57;
    volatile uint32_t EFUSE0_ROW_58;
    volatile uint32_t EFUSE0_ROW_59;
    volatile uint32_t CUST_EFUSE0_ROW_0;
    volatile uint32_t CUST_EFUSE0_ROW_1;
    volatile uint32_t CUST_EFUSE0_ROW_2;
    volatile uint32_t CUST_EFUSE0_ROW_3;
    volatile uint32_t CUST_EFUSE0_ROW_4;
    volatile uint32_t CUST_EFUSE0_ROW_5;
    volatile uint32_t CUST_EFUSE0_ROW_6;
    volatile uint32_t CUST_EFUSE0_ROW_7;
    volatile uint32_t CUST_EFUSE0_ROW_8;
    volatile uint32_t CUST_EFUSE0_ROW_9;
    volatile uint32_t CUST_EFUSE0_ROW_10;
    volatile uint32_t CUST_EFUSE0_ROW_11;
    volatile uint32_t CUST_EFUSE0_ROW_12;
    volatile uint32_t CLR_CPFROM_EFUSE_SHIFT_REG;
    volatile uint32_t JTAG_DIS_OVERRIDE;
    volatile uint32_t RS232_DIS_OVERRIDE;
    volatile uint32_t TESTC_DIS_OVERRIDE;
    volatile uint32_t LVDS_DIS_OVERRIDE;
    volatile uint32_t JTAG_DIS_LOCK;
    volatile uint32_t RS232_DIS_LOCK;
    volatile uint32_t TESTC_DIS_LOCK;
    volatile uint32_t LVDS_DIS_LOCK;
    volatile uint32_t EFUSE_OVERRIDE_MEM_MARGINCTRL;
    volatile uint32_t EFUSE_OVERRIDE_SLICER_BIAS_RTRIM;
    volatile uint32_t EFUSE_OVERRIDE_RS232_CLKMODE;
    volatile uint32_t EFUSE_OVERRIDE_EN_VOL_MON_FUNC;
    volatile uint32_t EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE;
    volatile uint32_t EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE;
    volatile uint32_t EFUSE_OVERRIDE_IP1_BG1_RTRIM;
    volatile uint32_t EFUSE_OVERRIDE_IP1_BG1_SLOPE;
    volatile uint32_t EFUSE_OVERRIDE_IP1_BG1_MAG;
    volatile uint32_t EFUSE_OVERRIDE_SPARE_ANA;
    volatile uint32_t EFUSE_OVERRIDE_DS_V2I_RTRIM_30C;
    volatile uint32_t EFUSE_OVERRIDE_SLICER_LDO_VTRIM;
    volatile uint32_t EFUSE_FROM_CNTL;
    volatile uint32_t REFSYS_CTRL_REG_LOWV;
    volatile uint32_t REFSYS_SPARE_REG_LOWV;
    volatile uint32_t WU_CTRL_REG_LOWV;
    volatile uint32_t WU_CTRL_REG1_LOWV;
    volatile uint32_t WU_MODE_REG_LOWV;
    volatile uint32_t EFUSE_SPARE_REG;
    volatile uint32_t EFUSE_SPARE_OVERRIDE;
    volatile uint32_t FUSEFARM_ERR_STATUS;
    volatile uint32_t HW_REG0;
    volatile uint32_t HW_REG1;
    volatile uint32_t CFG_MMR_CLKSTOP_OVERRIDE;
    volatile uint32_t DEBUG_BUS_SEL;
    volatile uint32_t DEBUG_BUS_BIT_SEL;
    volatile uint32_t STICKY_STATUS_DEBUG;
    volatile uint32_t RESERVED0;
    volatile uint32_t ROM_LOCK;
    volatile uint32_t RAM_ECC_CFG;
    volatile uint32_t SPARE_REG1;
    volatile uint32_t SPARE_REG2;
    volatile uint32_t SPARE_REG3;
    volatile uint8_t  Resv_4104[3660];
    volatile uint32_t LOCK0_KICK0;
    volatile uint32_t LOCK0_KICK1;
    volatile uint32_t INTR_RAW_STATUS;
    volatile uint32_t INTR_ENABLED_STATUS_CLEAR;
    volatile uint32_t INTR_ENABLE;
    volatile uint32_t INTR_ENABLE_CLEAR;
    volatile uint32_t EOI;
    volatile uint32_t FAULT_ADDRESS;
    volatile uint32_t FAULT_TYPE_STATUS;
    volatile uint32_t FAULT_ATTR_STATUS;
    volatile uint32_t FAULT_CLEAR;
} CSL_top_efuseRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_TOP_EFUSE_PID                                                      (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID0                                             (0x00000004U)
#define CSL_TOP_EFUSE_EFUSE_DIEID1                                             (0x00000008U)
#define CSL_TOP_EFUSE_EFUSE_DIEID2                                             (0x0000000CU)
#define CSL_TOP_EFUSE_EFUSE_DIEID3                                             (0x00000010U)
#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE                                        (0x00000014U)
#define CSL_TOP_EFUSE_EFUSE_UID0                                               (0x00000018U)
#define CSL_TOP_EFUSE_EFUSE_UID1                                               (0x0000001CU)
#define CSL_TOP_EFUSE_EFUSE_UID2                                               (0x00000020U)
#define CSL_TOP_EFUSE_EFUSE_UID3                                               (0x00000024U)
#define CSL_TOP_EFUSE_EFUSE_UID4                                               (0x00000028U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_15                                            (0x0000002CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_16                                            (0x00000030U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_17                                            (0x00000034U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_18                                            (0x00000038U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_19                                            (0x0000003CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_20                                            (0x00000040U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_21                                            (0x00000044U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_22                                            (0x00000048U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_23                                            (0x0000004CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_24                                            (0x00000050U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_25                                            (0x00000054U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_26                                            (0x00000058U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_27                                            (0x0000005CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_28                                            (0x00000060U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_29                                            (0x00000064U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_30                                            (0x00000068U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_31                                            (0x0000006CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_32                                            (0x00000070U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_33                                            (0x00000074U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_34                                            (0x00000078U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_35                                            (0x0000007CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_36                                            (0x00000080U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_37                                            (0x00000084U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_38                                            (0x00000088U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_39                                            (0x0000008CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_40                                            (0x00000090U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_41                                            (0x00000094U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_42                                            (0x00000098U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_43                                            (0x0000009CU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_44                                            (0x000000A0U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_45                                            (0x000000A4U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_46                                            (0x000000A8U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_47                                            (0x000000ACU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_48                                            (0x000000B0U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_49                                            (0x000000B4U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_50                                            (0x000000B8U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_51                                            (0x000000BCU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_52                                            (0x000000C0U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_53                                            (0x000000C4U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_54                                            (0x000000C8U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_55                                            (0x000000CCU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_56                                            (0x000000D0U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_57                                            (0x000000D4U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_58                                            (0x000000D8U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_59                                            (0x000000DCU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0                                        (0x000000E0U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1                                        (0x000000E4U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2                                        (0x000000E8U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3                                        (0x000000ECU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4                                        (0x000000F0U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5                                        (0x000000F4U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6                                        (0x000000F8U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7                                        (0x000000FCU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8                                        (0x00000100U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9                                        (0x00000104U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10                                       (0x00000108U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11                                       (0x0000010CU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12                                       (0x00000110U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG                               (0x00000114U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE                                        (0x00000118U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE                                       (0x0000011CU)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE                                       (0x00000120U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE                                        (0x00000124U)
#define CSL_TOP_EFUSE_JTAG_DIS_LOCK                                            (0x00000128U)
#define CSL_TOP_EFUSE_RS232_DIS_LOCK                                           (0x0000012CU)
#define CSL_TOP_EFUSE_TESTC_DIS_LOCK                                           (0x00000130U)
#define CSL_TOP_EFUSE_LVDS_DIS_LOCK                                            (0x00000134U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL                            (0x00000138U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM                         (0x0000013CU)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE                             (0x00000140U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC                           (0x00000144U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE                        (0x00000148U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE                        (0x0000014CU)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM                             (0x00000150U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE                             (0x00000154U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG                               (0x00000158U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA                                 (0x0000015CU)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C                          (0x00000160U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM                          (0x00000164U)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL                                          (0x00000168U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV                                     (0x0000016CU)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV                                    (0x00000170U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV                                         (0x00000174U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV                                        (0x00000178U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV                                         (0x0000017CU)
#define CSL_TOP_EFUSE_EFUSE_SPARE_REG                                          (0x00000180U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE                                     (0x00000184U)
#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS                                      (0x00000188U)
#define CSL_TOP_EFUSE_HW_REG0                                                  (0x0000018CU)
#define CSL_TOP_EFUSE_HW_REG1                                                  (0x00000190U)
#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE                                 (0x00000194U)
#define CSL_TOP_EFUSE_DEBUG_BUS_SEL                                            (0x00000198U)
#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL                                        (0x0000019CU)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG                                      (0x000001A0U)
#define CSL_TOP_EFUSE_RESERVED0                                                (0x000001A4U)
#define CSL_TOP_EFUSE_ROM_LOCK                                                 (0x000001A8U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG                                              (0x000001ACU)
#define CSL_TOP_EFUSE_SPARE_REG1                                               (0x000001B0U)
#define CSL_TOP_EFUSE_SPARE_REG2                                               (0x000001B4U)
#define CSL_TOP_EFUSE_SPARE_REG3                                               (0x000001B8U)
#define CSL_TOP_EFUSE_LOCK0_KICK0                                              (0x00001008U)
#define CSL_TOP_EFUSE_LOCK0_KICK1                                              (0x0000100CU)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS                                          (0x00001010U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR                                (0x00001014U)
#define CSL_TOP_EFUSE_INTR_ENABLE                                              (0x00001018U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR                                        (0x0000101CU)
#define CSL_TOP_EFUSE_EOI                                                      (0x00001020U)
#define CSL_TOP_EFUSE_FAULT_ADDRESS                                            (0x00001024U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS                                        (0x00001028U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS                                        (0x0000102CU)
#define CSL_TOP_EFUSE_FAULT_CLEAR                                              (0x00001030U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_TOP_EFUSE_PID_PID_MINOR_MASK                                       (0x0000003FU)
#define CSL_TOP_EFUSE_PID_PID_MINOR_SHIFT                                      (0x00000000U)
#define CSL_TOP_EFUSE_PID_PID_MINOR_RESETVAL                                   (0x00000014U)
#define CSL_TOP_EFUSE_PID_PID_MINOR_MAX                                        (0x0000003FU)

#define CSL_TOP_EFUSE_PID_PID_CUSTOM_MASK                                      (0x000000C0U)
#define CSL_TOP_EFUSE_PID_PID_CUSTOM_SHIFT                                     (0x00000006U)
#define CSL_TOP_EFUSE_PID_PID_CUSTOM_RESETVAL                                  (0x00000000U)
#define CSL_TOP_EFUSE_PID_PID_CUSTOM_MAX                                       (0x00000003U)

#define CSL_TOP_EFUSE_PID_PID_MAJOR_MASK                                       (0x00000700U)
#define CSL_TOP_EFUSE_PID_PID_MAJOR_SHIFT                                      (0x00000008U)
#define CSL_TOP_EFUSE_PID_PID_MAJOR_RESETVAL                                   (0x00000002U)
#define CSL_TOP_EFUSE_PID_PID_MAJOR_MAX                                        (0x00000007U)

#define CSL_TOP_EFUSE_PID_PID_MISC_MASK                                        (0x0000F800U)
#define CSL_TOP_EFUSE_PID_PID_MISC_SHIFT                                       (0x0000000BU)
#define CSL_TOP_EFUSE_PID_PID_MISC_RESETVAL                                    (0x00000000U)
#define CSL_TOP_EFUSE_PID_PID_MISC_MAX                                         (0x0000001FU)

#define CSL_TOP_EFUSE_PID_PID_MSB16_MASK                                       (0xFFFF0000U)
#define CSL_TOP_EFUSE_PID_PID_MSB16_SHIFT                                      (0x00000010U)
#define CSL_TOP_EFUSE_PID_PID_MSB16_RESETVAL                                   (0x00006180U)
#define CSL_TOP_EFUSE_PID_PID_MSB16_MAX                                        (0x0000FFFFU)

#define CSL_TOP_EFUSE_PID_RESETVAL                                             (0x61800214U)

/* EFUSE_DIEID0 */

#define CSL_TOP_EFUSE_EFUSE_DIEID0_EFUSE_DIEID0_EFUSE_DIEID0_MASK              (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_DIEID0_EFUSE_DIEID0_EFUSE_DIEID0_SHIFT             (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID0_EFUSE_DIEID0_EFUSE_DIEID0_RESETVAL          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID0_EFUSE_DIEID0_EFUSE_DIEID0_MAX               (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_DIEID0_RESETVAL                                    (0x00000000U)

/* EFUSE_DIEID1 */

#define CSL_TOP_EFUSE_EFUSE_DIEID1_EFUSE_DIEID1_EFUSE_DIEID1_MASK              (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_DIEID1_EFUSE_DIEID1_EFUSE_DIEID1_SHIFT             (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID1_EFUSE_DIEID1_EFUSE_DIEID1_RESETVAL          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID1_EFUSE_DIEID1_EFUSE_DIEID1_MAX               (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_DIEID1_RESETVAL                                    (0x00000000U)

/* EFUSE_DIEID2 */

#define CSL_TOP_EFUSE_EFUSE_DIEID2_EFUSE_DIEID2_EFUSE_DIEID2_MASK              (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_DIEID2_EFUSE_DIEID2_EFUSE_DIEID2_SHIFT             (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID2_EFUSE_DIEID2_EFUSE_DIEID2_RESETVAL          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID2_EFUSE_DIEID2_EFUSE_DIEID2_MAX               (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_DIEID2_RESETVAL                                    (0x00000000U)

/* EFUSE_DIEID3 */

#define CSL_TOP_EFUSE_EFUSE_DIEID3_EFUSE_DIEID3_EFUSE_DIEID3_MASK              (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_DIEID3_EFUSE_DIEID3_EFUSE_DIEID3_SHIFT             (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID3_EFUSE_DIEID3_EFUSE_DIEID3_RESETVAL          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DIEID3_EFUSE_DIEID3_EFUSE_DIEID3_MAX               (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_DIEID3_RESETVAL                                    (0x00000000U)

/* EFUSE_DEVICE_TYPE */

#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_EFUSE_DEVICE_TYPE_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_DEVICE_TYPE_RESETVAL                               (0x00000000U)

/* EFUSE_UID0 */

#define CSL_TOP_EFUSE_EFUSE_UID0_EFUSE_UID0_EFUSE_UID0_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_UID0_EFUSE_UID0_EFUSE_UID0_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID0_EFUSE_UID0_EFUSE_UID0_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID0_EFUSE_UID0_EFUSE_UID0_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_UID0_RESETVAL                                      (0x00000000U)

/* EFUSE_UID1 */

#define CSL_TOP_EFUSE_EFUSE_UID1_EFUSE_UID1_EFUSE_UID1_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_UID1_EFUSE_UID1_EFUSE_UID1_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID1_EFUSE_UID1_EFUSE_UID1_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID1_EFUSE_UID1_EFUSE_UID1_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_UID1_RESETVAL                                      (0x00000000U)

/* EFUSE_UID2 */

#define CSL_TOP_EFUSE_EFUSE_UID2_EFUSE_UID2_EFUSE_UID2_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_UID2_EFUSE_UID2_EFUSE_UID2_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID2_EFUSE_UID2_EFUSE_UID2_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID2_EFUSE_UID2_EFUSE_UID2_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_UID2_RESETVAL                                      (0x00000000U)

/* EFUSE_UID3 */

#define CSL_TOP_EFUSE_EFUSE_UID3_EFUSE_UID3_EFUSE_UID3_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_UID3_EFUSE_UID3_EFUSE_UID3_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID3_EFUSE_UID3_EFUSE_UID3_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID3_EFUSE_UID3_EFUSE_UID3_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_UID3_RESETVAL                                      (0x00000000U)

/* EFUSE_UID4 */

#define CSL_TOP_EFUSE_EFUSE_UID4_EFUSE_UID4_EFUSE_UID4_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_UID4_EFUSE_UID4_EFUSE_UID4_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID4_EFUSE_UID4_EFUSE_UID4_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_UID4_EFUSE_UID4_EFUSE_UID4_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_UID4_RESETVAL                                      (0x00000000U)

/* EFUSE0_ROW_15 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_15_EFUSE0_ROW_15_EFUSE0_ROW_15_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_15_EFUSE0_ROW_15_EFUSE0_ROW_15_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_15_EFUSE0_ROW_15_EFUSE0_ROW_15_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_15_EFUSE0_ROW_15_EFUSE0_ROW_15_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_15_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_16 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_16_EFUSE0_ROW_16_EFUSE0_ROW_16_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_16_EFUSE0_ROW_16_EFUSE0_ROW_16_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_16_EFUSE0_ROW_16_EFUSE0_ROW_16_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_16_EFUSE0_ROW_16_EFUSE0_ROW_16_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_16_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_17 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_17_EFUSE0_ROW_17_EFUSE0_ROW_17_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_17_EFUSE0_ROW_17_EFUSE0_ROW_17_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_17_EFUSE0_ROW_17_EFUSE0_ROW_17_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_17_EFUSE0_ROW_17_EFUSE0_ROW_17_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_17_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_18 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_18_EFUSE0_ROW_18_EFUSE0_ROW_18_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_18_EFUSE0_ROW_18_EFUSE0_ROW_18_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_18_EFUSE0_ROW_18_EFUSE0_ROW_18_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_18_EFUSE0_ROW_18_EFUSE0_ROW_18_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_18_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_19 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_19_EFUSE0_ROW_19_EFUSE0_ROW_19_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_19_EFUSE0_ROW_19_EFUSE0_ROW_19_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_19_EFUSE0_ROW_19_EFUSE0_ROW_19_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_19_EFUSE0_ROW_19_EFUSE0_ROW_19_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_19_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_20 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_20_EFUSE0_ROW_20_EFUSE0_ROW_20_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_20_EFUSE0_ROW_20_EFUSE0_ROW_20_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_20_EFUSE0_ROW_20_EFUSE0_ROW_20_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_20_EFUSE0_ROW_20_EFUSE0_ROW_20_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_20_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_21 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_21_EFUSE0_ROW_21_EFUSE0_ROW_21_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_21_EFUSE0_ROW_21_EFUSE0_ROW_21_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_21_EFUSE0_ROW_21_EFUSE0_ROW_21_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_21_EFUSE0_ROW_21_EFUSE0_ROW_21_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_21_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_22 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_22_EFUSE0_ROW_22_EFUSE0_ROW_22_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_22_EFUSE0_ROW_22_EFUSE0_ROW_22_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_22_EFUSE0_ROW_22_EFUSE0_ROW_22_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_22_EFUSE0_ROW_22_EFUSE0_ROW_22_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_22_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_23 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_23_EFUSE0_ROW_23_EFUSE0_ROW_23_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_23_EFUSE0_ROW_23_EFUSE0_ROW_23_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_23_EFUSE0_ROW_23_EFUSE0_ROW_23_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_23_EFUSE0_ROW_23_EFUSE0_ROW_23_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_23_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_24 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_24_EFUSE0_ROW_24_EFUSE0_ROW_24_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_24_EFUSE0_ROW_24_EFUSE0_ROW_24_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_24_EFUSE0_ROW_24_EFUSE0_ROW_24_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_24_EFUSE0_ROW_24_EFUSE0_ROW_24_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_24_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_25 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_25_EFUSE0_ROW_25_EFUSE0_ROW_25_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_25_EFUSE0_ROW_25_EFUSE0_ROW_25_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_25_EFUSE0_ROW_25_EFUSE0_ROW_25_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_25_EFUSE0_ROW_25_EFUSE0_ROW_25_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_25_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_26 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_26_EFUSE0_ROW_26_EFUSE0_ROW_26_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_26_EFUSE0_ROW_26_EFUSE0_ROW_26_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_26_EFUSE0_ROW_26_EFUSE0_ROW_26_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_26_EFUSE0_ROW_26_EFUSE0_ROW_26_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_26_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_27 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_27_EFUSE0_ROW_27_EFUSE0_ROW_27_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_27_EFUSE0_ROW_27_EFUSE0_ROW_27_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_27_EFUSE0_ROW_27_EFUSE0_ROW_27_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_27_EFUSE0_ROW_27_EFUSE0_ROW_27_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_27_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_28 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_28_EFUSE0_ROW_28_EFUSE0_ROW_28_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_28_EFUSE0_ROW_28_EFUSE0_ROW_28_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_28_EFUSE0_ROW_28_EFUSE0_ROW_28_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_28_EFUSE0_ROW_28_EFUSE0_ROW_28_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_28_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_29 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_29_EFUSE0_ROW_29_EFUSE0_ROW_29_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_29_EFUSE0_ROW_29_EFUSE0_ROW_29_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_29_EFUSE0_ROW_29_EFUSE0_ROW_29_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_29_EFUSE0_ROW_29_EFUSE0_ROW_29_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_29_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_30 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_30_EFUSE0_ROW_30_EFUSE0_ROW_30_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_30_EFUSE0_ROW_30_EFUSE0_ROW_30_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_30_EFUSE0_ROW_30_EFUSE0_ROW_30_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_30_EFUSE0_ROW_30_EFUSE0_ROW_30_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_30_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_31 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_31_EFUSE0_ROW_31_EFUSE0_ROW_31_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_31_EFUSE0_ROW_31_EFUSE0_ROW_31_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_31_EFUSE0_ROW_31_EFUSE0_ROW_31_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_31_EFUSE0_ROW_31_EFUSE0_ROW_31_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_31_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_32 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_32_EFUSE0_ROW_32_EFUSE0_ROW_32_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_32_EFUSE0_ROW_32_EFUSE0_ROW_32_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_32_EFUSE0_ROW_32_EFUSE0_ROW_32_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_32_EFUSE0_ROW_32_EFUSE0_ROW_32_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_32_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_33 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_33_EFUSE0_ROW_33_EFUSE0_ROW_33_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_33_EFUSE0_ROW_33_EFUSE0_ROW_33_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_33_EFUSE0_ROW_33_EFUSE0_ROW_33_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_33_EFUSE0_ROW_33_EFUSE0_ROW_33_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_33_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_34 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_34_EFUSE0_ROW_34_EFUSE0_ROW_34_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_34_EFUSE0_ROW_34_EFUSE0_ROW_34_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_34_EFUSE0_ROW_34_EFUSE0_ROW_34_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_34_EFUSE0_ROW_34_EFUSE0_ROW_34_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_34_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_35 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_35_EFUSE0_ROW_35_EFUSE0_ROW_35_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_35_EFUSE0_ROW_35_EFUSE0_ROW_35_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_35_EFUSE0_ROW_35_EFUSE0_ROW_35_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_35_EFUSE0_ROW_35_EFUSE0_ROW_35_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_35_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_36 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_36_EFUSE0_ROW_36_EFUSE0_ROW_36_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_36_EFUSE0_ROW_36_EFUSE0_ROW_36_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_36_EFUSE0_ROW_36_EFUSE0_ROW_36_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_36_EFUSE0_ROW_36_EFUSE0_ROW_36_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_36_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_37 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_37_EFUSE0_ROW_37_EFUSE0_ROW_37_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_37_EFUSE0_ROW_37_EFUSE0_ROW_37_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_37_EFUSE0_ROW_37_EFUSE0_ROW_37_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_37_EFUSE0_ROW_37_EFUSE0_ROW_37_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_37_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_38 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_38_EFUSE0_ROW_38_EFUSE0_ROW_38_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_38_EFUSE0_ROW_38_EFUSE0_ROW_38_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_38_EFUSE0_ROW_38_EFUSE0_ROW_38_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_38_EFUSE0_ROW_38_EFUSE0_ROW_38_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_38_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_39 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_39_EFUSE0_ROW_39_EFUSE0_ROW_39_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_39_EFUSE0_ROW_39_EFUSE0_ROW_39_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_39_EFUSE0_ROW_39_EFUSE0_ROW_39_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_39_EFUSE0_ROW_39_EFUSE0_ROW_39_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_39_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_40 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_40_EFUSE0_ROW_40_EFUSE0_ROW_40_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_40_EFUSE0_ROW_40_EFUSE0_ROW_40_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_40_EFUSE0_ROW_40_EFUSE0_ROW_40_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_40_EFUSE0_ROW_40_EFUSE0_ROW_40_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_40_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_41 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_41_EFUSE0_ROW_41_EFUSE0_ROW_41_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_41_EFUSE0_ROW_41_EFUSE0_ROW_41_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_41_EFUSE0_ROW_41_EFUSE0_ROW_41_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_41_EFUSE0_ROW_41_EFUSE0_ROW_41_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_41_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_42 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_42_EFUSE0_ROW_42_EFUSE0_ROW_42_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_42_EFUSE0_ROW_42_EFUSE0_ROW_42_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_42_EFUSE0_ROW_42_EFUSE0_ROW_42_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_42_EFUSE0_ROW_42_EFUSE0_ROW_42_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_42_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_43 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_43_EFUSE0_ROW_43_EFUSE0_ROW_43_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_43_EFUSE0_ROW_43_EFUSE0_ROW_43_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_43_EFUSE0_ROW_43_EFUSE0_ROW_43_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_43_EFUSE0_ROW_43_EFUSE0_ROW_43_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_43_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_44 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_44_EFUSE0_ROW_44_EFUSE0_ROW_44_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_44_EFUSE0_ROW_44_EFUSE0_ROW_44_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_44_EFUSE0_ROW_44_EFUSE0_ROW_44_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_44_EFUSE0_ROW_44_EFUSE0_ROW_44_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_44_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_45 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_45_EFUSE0_ROW_45_EFUSE0_ROW_45_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_45_EFUSE0_ROW_45_EFUSE0_ROW_45_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_45_EFUSE0_ROW_45_EFUSE0_ROW_45_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_45_EFUSE0_ROW_45_EFUSE0_ROW_45_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_45_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_46 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_46_EFUSE0_ROW_46_EFUSE0_ROW_46_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_46_EFUSE0_ROW_46_EFUSE0_ROW_46_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_46_EFUSE0_ROW_46_EFUSE0_ROW_46_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_46_EFUSE0_ROW_46_EFUSE0_ROW_46_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_46_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_47 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_47_EFUSE0_ROW_47_EFUSE0_ROW_47_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_47_EFUSE0_ROW_47_EFUSE0_ROW_47_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_47_EFUSE0_ROW_47_EFUSE0_ROW_47_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_47_EFUSE0_ROW_47_EFUSE0_ROW_47_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_47_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_48 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_48_EFUSE0_ROW_48_EFUSE0_ROW_48_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_48_EFUSE0_ROW_48_EFUSE0_ROW_48_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_48_EFUSE0_ROW_48_EFUSE0_ROW_48_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_48_EFUSE0_ROW_48_EFUSE0_ROW_48_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_48_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_49 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_49_EFUSE0_ROW_49_EFUSE0_ROW_49_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_49_EFUSE0_ROW_49_EFUSE0_ROW_49_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_49_EFUSE0_ROW_49_EFUSE0_ROW_49_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_49_EFUSE0_ROW_49_EFUSE0_ROW_49_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_49_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_50 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_50_EFUSE0_ROW_50_EFUSE0_ROW_50_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_50_EFUSE0_ROW_50_EFUSE0_ROW_50_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_50_EFUSE0_ROW_50_EFUSE0_ROW_50_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_50_EFUSE0_ROW_50_EFUSE0_ROW_50_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_50_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_51 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_51_EFUSE0_ROW_51_EFUSE0_ROW_51_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_51_EFUSE0_ROW_51_EFUSE0_ROW_51_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_51_EFUSE0_ROW_51_EFUSE0_ROW_51_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_51_EFUSE0_ROW_51_EFUSE0_ROW_51_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_51_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_52 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_52_EFUSE0_ROW_52_EFUSE0_ROW_52_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_52_EFUSE0_ROW_52_EFUSE0_ROW_52_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_52_EFUSE0_ROW_52_EFUSE0_ROW_52_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_52_EFUSE0_ROW_52_EFUSE0_ROW_52_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_52_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_53 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_53_EFUSE0_ROW_53_EFUSE0_ROW_53_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_53_EFUSE0_ROW_53_EFUSE0_ROW_53_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_53_EFUSE0_ROW_53_EFUSE0_ROW_53_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_53_EFUSE0_ROW_53_EFUSE0_ROW_53_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_53_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_54 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_54_EFUSE0_ROW_54_EFUSE0_ROW_54_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_54_EFUSE0_ROW_54_EFUSE0_ROW_54_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_54_EFUSE0_ROW_54_EFUSE0_ROW_54_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_54_EFUSE0_ROW_54_EFUSE0_ROW_54_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_54_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_55 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_55_EFUSE0_ROW_55_EFUSE0_ROW_55_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_55_EFUSE0_ROW_55_EFUSE0_ROW_55_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_55_EFUSE0_ROW_55_EFUSE0_ROW_55_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_55_EFUSE0_ROW_55_EFUSE0_ROW_55_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_55_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_56 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_56_EFUSE0_ROW_56_EFUSE0_ROW_56_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_56_EFUSE0_ROW_56_EFUSE0_ROW_56_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_56_EFUSE0_ROW_56_EFUSE0_ROW_56_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_56_EFUSE0_ROW_56_EFUSE0_ROW_56_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_56_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_57 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_57_EFUSE0_ROW_57_EFUSE0_ROW_57_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_57_EFUSE0_ROW_57_EFUSE0_ROW_57_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_57_EFUSE0_ROW_57_EFUSE0_ROW_57_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_57_EFUSE0_ROW_57_EFUSE0_ROW_57_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_57_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_58 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_58_EFUSE0_ROW_58_EFUSE0_ROW_58_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_58_EFUSE0_ROW_58_EFUSE0_ROW_58_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_58_EFUSE0_ROW_58_EFUSE0_ROW_58_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_58_EFUSE0_ROW_58_EFUSE0_ROW_58_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_58_RESETVAL                                   (0x00000000U)

/* EFUSE0_ROW_59 */

#define CSL_TOP_EFUSE_EFUSE0_ROW_59_EFUSE0_ROW_59_EFUSE0_ROW_59_MASK           (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE0_ROW_59_EFUSE0_ROW_59_EFUSE0_ROW_59_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_59_EFUSE0_ROW_59_EFUSE0_ROW_59_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE0_ROW_59_EFUSE0_ROW_59_EFUSE0_ROW_59_MAX            (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE0_ROW_59_RESETVAL                                   (0x00000000U)

/* CUST_EFUSE0_ROW_0 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_CUST_EFUSE0_ROW_0_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_0_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_1 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_CUST_EFUSE0_ROW_1_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_1_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_2 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_CUST_EFUSE0_ROW_2_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_2_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_3 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_CUST_EFUSE0_ROW_3_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_3_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_4 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_CUST_EFUSE0_ROW_4_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_4_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_5 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_CUST_EFUSE0_ROW_5_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_5_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_6 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_CUST_EFUSE0_ROW_6_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_6_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_7 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_CUST_EFUSE0_ROW_7_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_7_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_8 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_CUST_EFUSE0_ROW_8_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_8_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_9 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_CUST_EFUSE0_ROW_9_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_9_RESETVAL                               (0x00000000U)

/* CUST_EFUSE0_ROW_10 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_CUST_EFUSE0_ROW_10_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_10_RESETVAL                              (0x00000000U)

/* CUST_EFUSE0_ROW_11 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_CUST_EFUSE0_ROW_11_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_11_RESETVAL                              (0x00000000U)

/* CUST_EFUSE0_ROW_12 */

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_MASK (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_CUST_EFUSE0_ROW_12_MAX (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_CUST_EFUSE0_ROW_12_RESETVAL                              (0x00000000U)

/* CLR_CPFROM_EFUSE_SHIFT_REG */

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_0_MASK (0x00000001U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_0_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_0_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_1_MASK (0x00000002U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_1_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_1_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_1_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_2_MASK (0x00000004U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_2_SHIFT (0x00000002U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_2_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_2_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_3_MASK (0x00000008U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_3_SHIFT (0x00000003U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_3_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_3_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_4_MASK (0x00000010U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_4_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_4_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_4_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_5_MASK (0x00000020U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_5_SHIFT (0x00000005U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_5_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_5_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_6_MASK (0x00000040U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_6_SHIFT (0x00000006U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_6_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_6_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_7_MASK (0x00000080U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_7_SHIFT (0x00000007U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_7_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_7_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_8_MASK (0x00000100U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_8_SHIFT (0x00000008U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_8_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_8_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_9_MASK (0x00000200U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_9_SHIFT (0x00000009U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_9_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_9_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_10_MASK (0x00000400U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_10_SHIFT (0x0000000AU)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_10_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_10_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_11_MASK (0x00000800U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_11_SHIFT (0x0000000BU)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_11_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_11_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_RESERVED_MASK (0xFFFFF000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_RESERVED_SHIFT (0x0000000CU)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_RESERVED_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_CLR_CPFROM_EFUSE_SHIFT_REG_RESERVED_MAX (0x000FFFFFU)

#define CSL_TOP_EFUSE_CLR_CPFROM_EFUSE_SHIFT_REG_RESETVAL                      (0x00000000U)

/* JTAG_DIS_OVERRIDE */

#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_VAL_MASK (0x00000002U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_VAL_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_JTAG_DIS_OVERRIDE_JTAG_DIS_SOFTWARE_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_JTAG_DIS_OVERRIDE_RESETVAL                               (0x00000000U)

/* RS232_DIS_OVERRIDE */

#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_VAL_MASK (0x00000002U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_VAL_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RS232_DIS_OVERRIDE_RS232_DIS_SOFTWARE_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_RS232_DIS_OVERRIDE_RESETVAL                              (0x00000000U)

/* TESTC_DIS_OVERRIDE */

#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_VAL_MASK (0x00000002U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_VAL_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_TESTC_DIS_OVERRIDE_TESTC_DIS_SOFTWARE_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_TESTC_DIS_OVERRIDE_RESETVAL                              (0x00000000U)

/* LVDS_DIS_OVERRIDE */

#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_VAL_MASK (0x00000002U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_VAL_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_LVDS_DIS_OVERRIDE_LVDS_DIS_SOFTWARE_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_LVDS_DIS_OVERRIDE_RESETVAL                               (0x00000000U)

/* JTAG_DIS_LOCK */

#define CSL_TOP_EFUSE_JTAG_DIS_LOCK_JTAG_DIS_LOCK_JTAG_DIS_LOCK_MASK           (0x00000001U)
#define CSL_TOP_EFUSE_JTAG_DIS_LOCK_JTAG_DIS_LOCK_JTAG_DIS_LOCK_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_JTAG_DIS_LOCK_JTAG_DIS_LOCK_JTAG_DIS_LOCK_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_JTAG_DIS_LOCK_JTAG_DIS_LOCK_JTAG_DIS_LOCK_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_JTAG_DIS_LOCK_RESETVAL                                   (0x00000000U)

/* RS232_DIS_LOCK */

#define CSL_TOP_EFUSE_RS232_DIS_LOCK_RS232_DIS_LOCK_RS232_DIS_LOCK_MASK        (0x00000001U)
#define CSL_TOP_EFUSE_RS232_DIS_LOCK_RS232_DIS_LOCK_RS232_DIS_LOCK_SHIFT       (0x00000000U)
#define CSL_TOP_EFUSE_RS232_DIS_LOCK_RS232_DIS_LOCK_RS232_DIS_LOCK_RESETVAL    (0x00000000U)
#define CSL_TOP_EFUSE_RS232_DIS_LOCK_RS232_DIS_LOCK_RS232_DIS_LOCK_MAX         (0x00000001U)

#define CSL_TOP_EFUSE_RS232_DIS_LOCK_RESETVAL                                  (0x00000000U)

/* TESTC_DIS_LOCK */

#define CSL_TOP_EFUSE_TESTC_DIS_LOCK_TESTC_DIS_LOCK_TESTC_DIS_LOCK_MASK        (0x00000001U)
#define CSL_TOP_EFUSE_TESTC_DIS_LOCK_TESTC_DIS_LOCK_TESTC_DIS_LOCK_SHIFT       (0x00000000U)
#define CSL_TOP_EFUSE_TESTC_DIS_LOCK_TESTC_DIS_LOCK_TESTC_DIS_LOCK_RESETVAL    (0x00000000U)
#define CSL_TOP_EFUSE_TESTC_DIS_LOCK_TESTC_DIS_LOCK_TESTC_DIS_LOCK_MAX         (0x00000001U)

#define CSL_TOP_EFUSE_TESTC_DIS_LOCK_RESETVAL                                  (0x00000000U)

/* LVDS_DIS_LOCK */

#define CSL_TOP_EFUSE_LVDS_DIS_LOCK_LVDS_DIS_LOCK_LVDS_DIS_LOCK_MASK           (0x00000001U)
#define CSL_TOP_EFUSE_LVDS_DIS_LOCK_LVDS_DIS_LOCK_LVDS_DIS_LOCK_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_LVDS_DIS_LOCK_LVDS_DIS_LOCK_LVDS_DIS_LOCK_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_LVDS_DIS_LOCK_LVDS_DIS_LOCK_LVDS_DIS_LOCK_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_LVDS_DIS_LOCK_RESETVAL                                   (0x00000000U)

/* EFUSE_OVERRIDE_MEM_MARGINCTRL */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_MASK (0x00000030U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GLG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_MASK (0x00000100U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_SHIFT (0x00000008U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_MASK (0x0000F000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_SHIFT (0x0000000CU)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_GWG_MARGIN_MAX (0x0000000FU)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_MASK (0x00010000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_SHIFT (0x00000010U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_MASK (0x00300000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_SHIFT (0x00000014U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BYG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_MASK (0x01000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_SHIFT (0x00000018U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_MASK (0x30000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_SHIFT (0x0000001CU)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_EFUSE_OVERRIDE_MEM_MARGINCTRL_BRG_MARGIN_MAX (0x00000003U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_MEM_MARGINCTRL_RESETVAL                   (0x00000000U)

/* EFUSE_OVERRIDE_SLICER_BIAS_RTRIM */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_BIAS_RTRIM_RESETVAL                (0x00000000U)

/* EFUSE_OVERRIDE_RS232_CLKMODE */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_VAL_MASK (0x00000010U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_VAL_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_VAL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_EFUSE_OVERRIDE_RS232_CLKMODE_OVERRIDE_VAL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RS232_CLKMODE_RESETVAL                    (0x00000000U)

/* EFUSE_OVERRIDE_EN_VOL_MON_FUNC */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_EN_VOL_MON_FUNC_RESETVAL                  (0x00000000U)

/* EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC10M_TRIM_CODE_RESETVAL               (0x00000000U)

/* EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_RCOSC32K_TRIM_CODE_RESETVAL               (0x00000000U)

/* EFUSE_OVERRIDE_IP1_BG1_RTRIM */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM_EFUSE_OVERRIDE_IP1_BG1_RTRIM_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM_EFUSE_OVERRIDE_IP1_BG1_RTRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM_EFUSE_OVERRIDE_IP1_BG1_RTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM_EFUSE_OVERRIDE_IP1_BG1_RTRIM_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_RTRIM_RESETVAL                    (0x00000000U)

/* EFUSE_OVERRIDE_IP1_BG1_SLOPE */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_SLOPE_RESETVAL                    (0x00000000U)

/* EFUSE_OVERRIDE_IP1_BG1_MAG */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG_EFUSE_OVERRIDE_IP1_BG1_MAG_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG_EFUSE_OVERRIDE_IP1_BG1_MAG_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG_EFUSE_OVERRIDE_IP1_BG1_MAG_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG_EFUSE_OVERRIDE_IP1_BG1_MAG_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_IP1_BG1_MAG_RESETVAL                      (0x00000000U)

/* EFUSE_OVERRIDE_SPARE_ANA */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA_EFUSE_OVERRIDE_SPARE_ANA_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA_EFUSE_OVERRIDE_SPARE_ANA_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA_EFUSE_OVERRIDE_SPARE_ANA_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA_EFUSE_OVERRIDE_SPARE_ANA_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SPARE_ANA_RESETVAL                        (0x00000000U)

/* EFUSE_OVERRIDE_DS_V2I_RTRIM_30C */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_DS_V2I_RTRIM_30C_RESETVAL                 (0x00000000U)

/* EFUSE_OVERRIDE_SLICER_LDO_VTRIM */

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_OVERRIDE_SLICER_LDO_VTRIM_RESETVAL                 (0x00000000U)

/* EFUSE_FROM_CNTL */

#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_RD_ENABLE_BIT_MASK (0x0000000FU)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_RD_ENABLE_BIT_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_RD_ENABLE_BIT_RESETVAL (0x0000000AU)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_RD_ENABLE_BIT_MAX (0x0000000FU)

#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_WR_ENABLE_BIT_MASK (0x000F0000U)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_WR_ENABLE_BIT_SHIFT (0x00000010U)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_WR_ENABLE_BIT_RESETVAL (0x0000000AU)
#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_EFUSE_FROM_CNTL_SECURITY_EFUSE_FROM_WR_ENABLE_BIT_MAX (0x0000000FU)

#define CSL_TOP_EFUSE_EFUSE_FROM_CNTL_RESETVAL                                 (0x000A000AU)

/* REFSYS_CTRL_REG_LOWV */

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED0_MASK (0x00000001U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED0_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED0_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED1_MASK (0x00000002U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED1_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED1_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED1_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED2_MASK (0x00000004U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED2_SHIFT (0x00000002U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED2_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED2_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED3_MASK (0x00000008U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED3_SHIFT (0x00000003U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED3_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RESERVED3_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_MASK (0x000001F0U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_RESETVAL (0x0000000DU)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_SLOPE_TRIM_4_0_MAX (0x0000001FU)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_MASK (0x00003E00U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_SHIFT (0x00000009U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_MAG_TRIM_4_0_MAX (0x0000001FU)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_MASK (0x0007C000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_SHIFT (0x0000000EU)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_RESETVAL (0x00000002U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IREF_TRIM_4_0_MAX (0x0000001FU)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_BGAP_ISW_MASK  (0x00080000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_BGAP_ISW_SHIFT (0x00000013U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_BGAP_ISW_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_BGAP_ISW_MAX   (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_MASK (0x00100000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_SHIFT (0x00000014U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_V2I_STARTUP_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_MASK (0x00200000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_SHIFT (0x00000015U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_CLKTOP_IBIAS_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_LODIST_IBIAS_EN_MASK (0x00400000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_LODIST_IBIAS_EN_SHIFT (0x00000016U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_LODIST_IBIAS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_LODIST_IBIAS_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_TX_TOP_IBIAS_EN_MASK (0x00800000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_TX_TOP_IBIAS_EN_SHIFT (0x00000017U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_TX_TOP_IBIAS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_TX_TOP_IBIAS_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_BYPASS_EN_MASK (0x01000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_BYPASS_EN_SHIFT (0x00000018U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_BYPASS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_V2I_BYPASS_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IDIODE_EN_MASK (0x02000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IDIODE_EN_SHIFT (0x00000019U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IDIODE_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_IDIODE_EN_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RX_TOP_IBIAS_EN_MASK (0x04000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RX_TOP_IBIAS_EN_SHIFT (0x0000001AU)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RX_TOP_IBIAS_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_RX_TOP_IBIAS_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_MASK (0x78000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_SHIFT (0x0000001BU)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_FTRIM_3_0_MAX  (0x0000000FU)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_TEST_EN_MASK (0x80000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_TEST_EN_SHIFT (0x0000001FU)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_TEST_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_REFSYS_CTRL_REG_LOWV_REFSYS_TEST_EN_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_CTRL_REG_LOWV_RESETVAL                            (0x002080DCU)

/* REFSYS_SPARE_REG_LOWV */

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_MASK (0x00000003U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL_MAX (0x00000003U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_MASK (0x00000004U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_SHIFT (0x00000002U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_MASK (0x00000008U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_SHIFT (0x00000003U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_MASK (0x00000030U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL_MAX (0x00000003U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_MASK (0x000000C0U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_SHIFT (0x00000006U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL_MAX (0x00000003U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_MASK (0x00000100U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_SHIFT (0x00000008U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_UV_RSET_MASK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_MASK (0x00000200U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_SHIFT (0x00000009U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_OV_RSET_MASK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_MASK (0x00000C00U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_SHIFT (0x0000000AU)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL_MAX (0x00000003U)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_RESERVED0_MASK (0xFFFFF000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_RESERVED0_SHIFT (0x0000000CU)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_RESERVED0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_REFSYS_SPARE_REG_LOWV_RESERVED0_MAX (0x000FFFFFU)

#define CSL_TOP_EFUSE_REFSYS_SPARE_REG_LOWV_RESETVAL                           (0x00000000U)

/* WU_CTRL_REG_LOWV */

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE0_MASK            (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE0_SHIFT           (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE0_RESETVAL        (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE0_MAX             (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_STOP_CLK_MASK          (0x00000002U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_STOP_CLK_SHIFT         (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_STOP_CLK_RESETVAL      (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_STOP_CLK_MAX           (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_SW_SEL_MASK        (0x00000004U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_SW_SEL_SHIFT       (0x00000002U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_SW_SEL_RESETVAL    (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_SW_SEL_MAX         (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_CLK_TRIM_MASK      (0x000003F8U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_CLK_TRIM_SHIFT     (0x00000003U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_CLK_TRIM_RESETVAL  (0x0000004BU)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_INT_CLK_TRIM_MAX       (0x0000007FU)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE1_MASK            (0x00000400U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE1_SHIFT           (0x0000000AU)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE1_RESETVAL        (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE1_MAX             (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_VMON_MASKING_MASK      (0x00007800U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_VMON_MASKING_SHIFT     (0x0000000BU)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_VMON_MASKING_RESETVAL  (0x00000008U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_VMON_MASKING_MAX       (0x0000000FU)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SCALED_1P2_EN_MASK     (0x00008000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SCALED_1P2_EN_SHIFT    (0x0000000FU)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SCALED_1P2_EN_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SCALED_1P2_EN_MAX      (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE2_MASK            (0x000F0000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE2_SHIFT           (0x00000010U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE2_RESETVAL        (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SPARE2_MAX             (0x0000000FU)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SUPPLY_MONITOR_EN_MASK (0x00100000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SUPPLY_MONITOR_EN_SHIFT (0x00000014U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SUPPLY_MONITOR_EN_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_SUPPLY_MONITOR_EN_MAX  (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_UV_VMON_EN_MASK        (0x00200000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_UV_VMON_EN_SHIFT       (0x00000015U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_UV_VMON_EN_RESETVAL    (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_UV_VMON_EN_MAX         (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_SUP_DET_CTRL_MASK   (0x00400000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_SUP_DET_CTRL_SHIFT  (0x00000016U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_SUP_DET_CTRL_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_SUP_DET_CTRL_MAX    (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_OSC_PWR_SW_MASK     (0x7F800000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_OSC_PWR_SW_SHIFT    (0x00000017U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_OSC_PWR_SW_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_WU_OSC_PWR_SW_MAX      (0x000000FFU)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_OV_DET_EN_MASK         (0x80000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_OV_DET_EN_SHIFT        (0x0000001FU)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_OV_DET_EN_RESETVAL     (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_WU_CTRL_REG_LOWV_OV_DET_EN_MAX          (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG_LOWV_RESETVAL                                (0x0050425CU)

/* WU_CTRL_REG1_LOWV */

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_RES_TRIM_MASK   (0x00000007U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_RES_TRIM_SHIFT  (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_RES_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_RES_TRIM_MAX    (0x00000007U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_FREQ_TRIM_MASK  (0x00000038U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_FREQ_TRIM_SHIFT (0x00000003U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_FREQ_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_FREQ_TRIM_MAX   (0x00000007U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_LOW_FREQ_TRIM_MASK (0x00000040U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_LOW_FREQ_TRIM_SHIFT (0x00000006U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_LOW_FREQ_TRIM_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SCLK_LOW_FREQ_TRIM_MAX (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SPARE_MASK           (0x00000180U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SPARE_SHIFT          (0x00000007U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SPARE_RESETVAL       (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SPARE_MAX            (0x00000003U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_EN_MN_REF_MASK       (0x00000200U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_EN_MN_REF_SHIFT      (0x00000009U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_EN_MN_REF_RESETVAL   (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_EN_MN_REF_MAX        (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_DIS_WU_REF_MASK      (0x00000400U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_DIS_WU_REF_SHIFT     (0x0000000AU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_DIS_WU_REF_RESETVAL  (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_DIS_WU_REF_MAX       (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_OSC_EN_MASK          (0x00000800U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_OSC_EN_SHIFT         (0x0000000BU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_OSC_EN_RESETVAL      (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_OSC_EN_MAX           (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SUBREG_EN_MASK       (0x00001000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SUBREG_EN_SHIFT      (0x0000000CU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SUBREG_EN_RESETVAL   (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_SUBREG_EN_MAX        (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_PWR_DET_EN_MASK      (0x00002000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_PWR_DET_EN_SHIFT     (0x0000000DU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_PWR_DET_EN_RESETVAL  (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_PWR_DET_EN_MAX       (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_WUP_REF_CTRL_MASK    (0x00004000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_WUP_REF_CTRL_SHIFT   (0x0000000EU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_WUP_REF_CTRL_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_WUP_REF_CTRL_MAX     (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_MREG_CTRL_MASK       (0x00008000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_MREG_CTRL_SHIFT      (0x0000000FU)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_MREG_CTRL_RESETVAL   (0x00000001U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_MREG_CTRL_MAX        (0x00000001U)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_RESERVED0_MASK       (0xFFFF0000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_RESERVED0_SHIFT      (0x00000010U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_RESERVED0_RESETVAL   (0x00000000U)
#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_WU_CTRL_REG1_LOWV_RESERVED0_MAX        (0x0000FFFFU)

#define CSL_TOP_EFUSE_WU_CTRL_REG1_LOWV_RESETVAL                               (0x0000FE00U)

/* WU_MODE_REG_LOWV */

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_MASK (0x00000001U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_FUNC_TEST_DET_SYNC_MAX (0x00000001U)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_MASK (0x00000002U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_TEST_MODE_DET_SYNC_MAX (0x00000001U)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_MASK  (0x0000007CU)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_SHIFT (0x00000002U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SOP_MODE_LAT_4_0_MAX   (0x0000001FU)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_INT_CLK_LAT_OK_MASK    (0x00000080U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_INT_CLK_LAT_OK_SHIFT   (0x00000007U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_INT_CLK_LAT_OK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_INT_CLK_LAT_OK_MAX     (0x00000001U)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SPARE0_MASK            (0x00000100U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SPARE0_SHIFT           (0x00000008U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SPARE0_RESETVAL        (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_SPARE0_MAX             (0x00000001U)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_RESERVED0_MASK         (0xFFFFFE00U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_RESERVED0_SHIFT        (0x00000009U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_RESERVED0_RESETVAL     (0x00000000U)
#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_WU_MODE_REG_LOWV_RESERVED0_MAX          (0x007FFFFFU)

#define CSL_TOP_EFUSE_WU_MODE_REG_LOWV_RESETVAL                                (0x00000000U)

/* EFUSE_SPARE_REG */

#define CSL_TOP_EFUSE_EFUSE_SPARE_REG_EFUSE_SPARE_REG_EFUSE_SPARE_REG_MASK     (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_EFUSE_SPARE_REG_EFUSE_SPARE_REG_EFUSE_SPARE_REG_SHIFT    (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_REG_EFUSE_SPARE_REG_EFUSE_SPARE_REG_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_REG_EFUSE_SPARE_REG_EFUSE_SPARE_REG_MAX      (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_EFUSE_SPARE_REG_RESETVAL                                 (0x00000000U)

/* EFUSE_SPARE_OVERRIDE */

#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_MASK (0x00000001U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_EFUSE_SPARE_OVERRIDE_MAX (0x00000001U)

#define CSL_TOP_EFUSE_EFUSE_SPARE_OVERRIDE_RESETVAL                            (0x00000000U)

/* FUSEFARM_ERR_STATUS */

#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS_FUSEFARM_ERR_STATUS_STATUS_MASK      (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS_FUSEFARM_ERR_STATUS_STATUS_SHIFT     (0x00000000U)
#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS_FUSEFARM_ERR_STATUS_STATUS_RESETVAL  (0x00000000U)
#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS_FUSEFARM_ERR_STATUS_STATUS_MAX       (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_FUSEFARM_ERR_STATUS_RESETVAL                             (0x00000000U)

/* HW_REG0 */

#define CSL_TOP_EFUSE_HW_REG0_HW_REG0_HW_REG0_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_HW_REG0_HW_REG0_HW_REG0_SHIFT                            (0x00000000U)
#define CSL_TOP_EFUSE_HW_REG0_HW_REG0_HW_REG0_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_HW_REG0_HW_REG0_HW_REG0_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_HW_REG0_RESETVAL                                         (0x00000000U)

/* HW_REG1 */

#define CSL_TOP_EFUSE_HW_REG1_HW_REG1_HW_REG1_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_HW_REG1_HW_REG1_HW_REG1_SHIFT                            (0x00000000U)
#define CSL_TOP_EFUSE_HW_REG1_HW_REG1_HW_REG1_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_HW_REG1_HW_REG1_HW_REG1_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_HW_REG1_RESETVAL                                         (0x00000000U)

/* CFG_MMR_CLKSTOP_OVERRIDE */

#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE_CFG_MMR_CLKSTOP_OVERRIDE_SEL_MASK (0x00000001U)
#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE_CFG_MMR_CLKSTOP_OVERRIDE_SEL_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE_CFG_MMR_CLKSTOP_OVERRIDE_SEL_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE_CFG_MMR_CLKSTOP_OVERRIDE_SEL_MAX (0x00000001U)

#define CSL_TOP_EFUSE_CFG_MMR_CLKSTOP_OVERRIDE_RESETVAL                        (0x00000001U)

/* DEBUG_BUS_SEL */

#define CSL_TOP_EFUSE_DEBUG_BUS_SEL_DEBUG_BUS_SEL_SEL_MASK                     (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_DEBUG_BUS_SEL_DEBUG_BUS_SEL_SEL_SHIFT                    (0x00000000U)
#define CSL_TOP_EFUSE_DEBUG_BUS_SEL_DEBUG_BUS_SEL_SEL_RESETVAL                 (0x00000000U)
#define CSL_TOP_EFUSE_DEBUG_BUS_SEL_DEBUG_BUS_SEL_SEL_MAX                      (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_DEBUG_BUS_SEL_RESETVAL                                   (0x00000000U)

/* DEBUG_BUS_BIT_SEL */

#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL_DEBUG_BUS_BIT_SEL_BIT_SEL_MASK         (0x000000FFU)
#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL_DEBUG_BUS_BIT_SEL_BIT_SEL_SHIFT        (0x00000000U)
#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL_DEBUG_BUS_BIT_SEL_BIT_SEL_RESETVAL     (0x00000000U)
#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL_DEBUG_BUS_BIT_SEL_BIT_SEL_MAX          (0x000000FFU)

#define CSL_TOP_EFUSE_DEBUG_BUS_BIT_SEL_RESETVAL                               (0x00000000U)

/* STICKY_STATUS_DEBUG */

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_JTAG_DIS_LOCK_MASK (0x00000001U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_JTAG_DIS_LOCK_SHIFT (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_JTAG_DIS_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_JTAG_DIS_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_RS232_DIS_LOCK_MASK (0x00000002U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_RS232_DIS_LOCK_SHIFT (0x00000001U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_RS232_DIS_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_RS232_DIS_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_TESTC_DIS_LOCK_MASK (0x00000004U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_TESTC_DIS_LOCK_SHIFT (0x00000002U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_TESTC_DIS_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_TESTC_DIS_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_LVDS_DIS_LOCK_MASK (0x00000008U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_LVDS_DIS_LOCK_SHIFT (0x00000003U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_LVDS_DIS_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_LVDS_DIS_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_RD_LOCK_MASK (0x00000010U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_RD_LOCK_SHIFT (0x00000004U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_RD_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_RD_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_WR_LOCK_MASK (0x00000020U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_WR_LOCK_SHIFT (0x00000005U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_WR_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_FROM_WR_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_ROM_LOCK_MASK (0x00000040U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_ROM_LOCK_SHIFT (0x00000006U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_ROM_LOCK_RESETVAL (0x00000000U)
#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_STICKY_STATUS_DEBUG_STICKY_SET_ROM_LOCK_MAX (0x00000001U)

#define CSL_TOP_EFUSE_STICKY_STATUS_DEBUG_RESETVAL                             (0x00000000U)

/* RESERVED0 */

#define CSL_TOP_EFUSE_RESERVED0_RESERVED0_RESERVED0_MASK                       (0x0000FFFFU)
#define CSL_TOP_EFUSE_RESERVED0_RESERVED0_RESERVED0_SHIFT                      (0x00000000U)
#define CSL_TOP_EFUSE_RESERVED0_RESERVED0_RESERVED0_RESETVAL                   (0x00000000U)
#define CSL_TOP_EFUSE_RESERVED0_RESERVED0_RESERVED0_MAX                        (0x0000FFFFU)

#define CSL_TOP_EFUSE_RESERVED0_RESETVAL                                       (0x00000000U)

/* ROM_LOCK */

#define CSL_TOP_EFUSE_ROM_LOCK_ROM_LOCK_ROM_LOCK_MASK                          (0x00000001U)
#define CSL_TOP_EFUSE_ROM_LOCK_ROM_LOCK_ROM_LOCK_SHIFT                         (0x00000000U)
#define CSL_TOP_EFUSE_ROM_LOCK_ROM_LOCK_ROM_LOCK_RESETVAL                      (0x00000000U)
#define CSL_TOP_EFUSE_ROM_LOCK_ROM_LOCK_ROM_LOCK_MAX                           (0x00000001U)

#define CSL_TOP_EFUSE_ROM_LOCK_RESETVAL                                        (0x00000000U)

/* RAM_ECC_CFG */

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM1_ECC_EN_MASK           (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM1_ECC_EN_SHIFT          (0x00000000U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM1_ECC_EN_RESETVAL       (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM1_ECC_EN_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM2_ECC_EN_MASK           (0x00000002U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM2_ECC_EN_SHIFT          (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM2_ECC_EN_RESETVAL       (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM2_ECC_EN_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM3_ECC_EN_MASK           (0x00000004U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM3_ECC_EN_SHIFT          (0x00000002U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM3_ECC_EN_RESETVAL       (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_RAM3_ECC_EN_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM0_ECC_EN_MASK     (0x00000008U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM0_ECC_EN_SHIFT    (0x00000003U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM0_ECC_EN_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM0_ECC_EN_MAX      (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM1_ECC_EN_MASK     (0x00000010U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM1_ECC_EN_SHIFT    (0x00000004U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM1_ECC_EN_RESETVAL (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_APPSS_SHAREDMEM1_ECC_EN_MAX      (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_RAM1_ECC_EN_MASK           (0x00000100U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_RAM1_ECC_EN_SHIFT          (0x00000008U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_RAM1_ECC_EN_RESETVAL       (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_RAM1_ECC_EN_MAX            (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_SHAREDMEM_ECC_EN_MASK      (0x00000200U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_SHAREDMEM_ECC_EN_SHIFT     (0x00000009U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_SHAREDMEM_ECC_EN_RESETVAL  (0x00000001U)
#define CSL_TOP_EFUSE_RAM_ECC_CFG_RAM_ECC_CFG_FECSS_SHAREDMEM_ECC_EN_MAX       (0x00000001U)

#define CSL_TOP_EFUSE_RAM_ECC_CFG_RESETVAL                                     (0x0000031FU)

/* SPARE_REG1 */

#define CSL_TOP_EFUSE_SPARE_REG1_SPARE_REG1_READ_WRITE_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_SPARE_REG1_SPARE_REG1_READ_WRITE_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG1_SPARE_REG1_READ_WRITE_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG1_SPARE_REG1_READ_WRITE_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_SPARE_REG1_RESETVAL                                      (0x00000000U)

/* SPARE_REG2 */

#define CSL_TOP_EFUSE_SPARE_REG2_SPARE_REG2_READ_WRITE_MASK                    (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_SPARE_REG2_SPARE_REG2_READ_WRITE_SHIFT                   (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG2_SPARE_REG2_READ_WRITE_RESETVAL                (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG2_SPARE_REG2_READ_WRITE_MAX                     (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_SPARE_REG2_RESETVAL                                      (0x00000000U)

/* SPARE_REG3 */

#define CSL_TOP_EFUSE_SPARE_REG3_SPARE_REG3_READ_ONLY_MASK                     (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_SPARE_REG3_SPARE_REG3_READ_ONLY_SHIFT                    (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG3_SPARE_REG3_READ_ONLY_RESETVAL                 (0x00000000U)
#define CSL_TOP_EFUSE_SPARE_REG3_SPARE_REG3_READ_ONLY_MAX                      (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_SPARE_REG3_RESETVAL                                      (0x00000000U)

/* LOCK0_KICK0 */

#define CSL_TOP_EFUSE_LOCK0_KICK0_LOCK0_KICK0_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_LOCK0_KICK0_LOCK0_KICK0_SHIFT                            (0x00000000U)
#define CSL_TOP_EFUSE_LOCK0_KICK0_LOCK0_KICK0_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_LOCK0_KICK0_LOCK0_KICK0_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_LOCK0_KICK0_RESETVAL                                     (0x00000000U)

/* LOCK0_KICK1 */

#define CSL_TOP_EFUSE_LOCK0_KICK1_LOCK0_KICK1_MASK                             (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_LOCK0_KICK1_LOCK0_KICK1_SHIFT                            (0x00000000U)
#define CSL_TOP_EFUSE_LOCK0_KICK1_LOCK0_KICK1_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_LOCK0_KICK1_LOCK0_KICK1_MAX                              (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_LOCK0_KICK1_RESETVAL                                     (0x00000000U)

/* INTR_RAW_STATUS */

#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROT_ERR_MASK                            (0x00000001U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROT_ERR_SHIFT                           (0x00000000U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROT_ERR_RESETVAL                        (0x00000000U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROT_ERR_MAX                             (0x00000001U)

#define CSL_TOP_EFUSE_INTR_RAW_STATUS_ADDR_ERR_MASK                            (0x00000002U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_ADDR_ERR_SHIFT                           (0x00000001U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_ADDR_ERR_RESETVAL                        (0x00000000U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_ADDR_ERR_MAX                             (0x00000001U)

#define CSL_TOP_EFUSE_INTR_RAW_STATUS_KICK_ERR_MASK                            (0x00000004U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_KICK_ERR_SHIFT                           (0x00000002U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_KICK_ERR_RESETVAL                        (0x00000000U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_KICK_ERR_MAX                             (0x00000001U)

#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROXY_ERR_MASK                           (0x00000008U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROXY_ERR_SHIFT                          (0x00000003U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROXY_ERR_RESETVAL                       (0x00000000U)
#define CSL_TOP_EFUSE_INTR_RAW_STATUS_PROXY_ERR_MAX                            (0x00000001U)

#define CSL_TOP_EFUSE_INTR_RAW_STATUS_RESETVAL                                 (0x00000000U)

/* INTR_ENABLED_STATUS_CLEAR */

#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MASK          (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_SHIFT         (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_RESETVAL      (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROT_ERR_MAX           (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MASK          (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_SHIFT         (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_RESETVAL      (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_ADDR_ERR_MAX           (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MASK          (0x00000004U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_SHIFT         (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_RESETVAL      (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_KICK_ERR_MAX           (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MASK         (0x00000008U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_SHIFT        (0x00000003U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_RESETVAL     (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_ENABLED_PROXY_ERR_MAX          (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLED_STATUS_CLEAR_RESETVAL                       (0x00000000U)

/* INTR_ENABLE */

#define CSL_TOP_EFUSE_INTR_ENABLE_PROT_ERR_EN_MASK                             (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROT_ERR_EN_SHIFT                            (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROT_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROT_ERR_EN_MAX                              (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_ADDR_ERR_EN_MASK                             (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLE_ADDR_ERR_EN_SHIFT                            (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLE_ADDR_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_ADDR_ERR_EN_MAX                              (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_KICK_ERR_EN_MASK                             (0x00000004U)
#define CSL_TOP_EFUSE_INTR_ENABLE_KICK_ERR_EN_SHIFT                            (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLE_KICK_ERR_EN_RESETVAL                         (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_KICK_ERR_EN_MAX                              (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_PROXY_ERR_EN_MASK                            (0x00000008U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROXY_ERR_EN_SHIFT                           (0x00000003U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROXY_ERR_EN_RESETVAL                        (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_PROXY_ERR_EN_MAX                             (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_RESETVAL                                     (0x00000000U)

/* INTR_ENABLE_CLEAR */

#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MASK                   (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_SHIFT                  (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROT_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MASK                   (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_SHIFT                  (0x00000001U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_ADDR_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MASK                   (0x00000004U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_SHIFT                  (0x00000002U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_RESETVAL               (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_KICK_ERR_EN_CLR_MAX                    (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MASK                  (0x00000008U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_SHIFT                 (0x00000003U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_RESETVAL              (0x00000000U)
#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_PROXY_ERR_EN_CLR_MAX                   (0x00000001U)

#define CSL_TOP_EFUSE_INTR_ENABLE_CLEAR_RESETVAL                               (0x00000000U)

/* EOI */

#define CSL_TOP_EFUSE_EOI_EOI_VECTOR_MASK                                      (0x000000FFU)
#define CSL_TOP_EFUSE_EOI_EOI_VECTOR_SHIFT                                     (0x00000000U)
#define CSL_TOP_EFUSE_EOI_EOI_VECTOR_RESETVAL                                  (0x00000000U)
#define CSL_TOP_EFUSE_EOI_EOI_VECTOR_MAX                                       (0x000000FFU)

#define CSL_TOP_EFUSE_EOI_RESETVAL                                             (0x00000000U)

/* FAULT_ADDRESS */

#define CSL_TOP_EFUSE_FAULT_ADDRESS_FAULT_ADDR_MASK                            (0xFFFFFFFFU)
#define CSL_TOP_EFUSE_FAULT_ADDRESS_FAULT_ADDR_SHIFT                           (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ADDRESS_FAULT_ADDR_RESETVAL                        (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ADDRESS_FAULT_ADDR_MAX                             (0xFFFFFFFFU)

#define CSL_TOP_EFUSE_FAULT_ADDRESS_RESETVAL                                   (0x00000000U)

/* FAULT_TYPE_STATUS */

#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_TYPE_MASK                        (0x0000003FU)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_TYPE_SHIFT                       (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_TYPE_RESETVAL                    (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_TYPE_MAX                         (0x0000003FU)

#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_NS_MASK                          (0x00000040U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_NS_SHIFT                         (0x00000006U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_NS_RESETVAL                      (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_FAULT_NS_MAX                           (0x00000001U)

#define CSL_TOP_EFUSE_FAULT_TYPE_STATUS_RESETVAL                               (0x00000000U)

/* FAULT_ATTR_STATUS */

#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_PRIVID_MASK                      (0x000000FFU)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_PRIVID_SHIFT                     (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_PRIVID_RESETVAL                  (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_PRIVID_MAX                       (0x000000FFU)

#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_ROUTEID_MASK                     (0x000FFF00U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_ROUTEID_SHIFT                    (0x00000008U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_ROUTEID_RESETVAL                 (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_ROUTEID_MAX                      (0x00000FFFU)

#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_XID_MASK                         (0xFFF00000U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_XID_SHIFT                        (0x00000014U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_XID_RESETVAL                     (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_FAULT_XID_MAX                          (0x00000FFFU)

#define CSL_TOP_EFUSE_FAULT_ATTR_STATUS_RESETVAL                               (0x00000000U)

/* FAULT_CLEAR */

#define CSL_TOP_EFUSE_FAULT_CLEAR_FAULT_CLR_MASK                               (0x00000001U)
#define CSL_TOP_EFUSE_FAULT_CLEAR_FAULT_CLR_SHIFT                              (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_CLEAR_FAULT_CLR_RESETVAL                           (0x00000000U)
#define CSL_TOP_EFUSE_FAULT_CLEAR_FAULT_CLR_MAX                                (0x00000001U)

#define CSL_TOP_EFUSE_FAULT_CLEAR_RESETVAL                                     (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
