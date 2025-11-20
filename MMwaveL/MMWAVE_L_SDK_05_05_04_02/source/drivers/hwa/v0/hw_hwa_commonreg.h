/**
 *  @file    hw_hwa_commonreg.h
 *
 *  @brief
 *    This file gives register definitions of DSS_HW_ACC module.
 *
 *  This file is auto-generated.
 *
 */

 /*
 *   (C) Copyright 2016, Texas Instruments Incorporated. - TI web address www.ti.com
 *---------------------------------------------------------------------------------------
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *    Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  OWNER OR CONTRIBUTORS
 *  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT,  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HW_HWA_COMMONREG_H_
#define HW_HWA_COMMONREG_H_

 /****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


 /** Definition for field ACCENABLE in Register HWACCREG1**/
#define HWACCREG1_ACCENABLE_BIT_START   0U
#define HWACCREG1_ACCENABLE_BIT_END     2U
/** Definition for field ACCCLKEN in Register HWACCREG1**/
#define HWACCREG1_ACCCLKEN_BIT_START   3U
#define HWACCREG1_ACCCLKEN_BIT_END     5U
/** Definition for field ACCRESET in Register HWACCREG1**/
#define HWACCREG1_ACCRESET_BIT_START   6U
#define HWACCREG1_ACCRESET_BIT_END     8U
/** Definition for field NU1 in Register HWACCREG1**/
#define HWACCREG1_NU1_BIT_START   9U
#define HWACCREG1_NU1_BIT_END     9U
/** Definition for field FFT1DEN in Register HWACCREG1**/
#define HWACCREG1_FFT1DEN_BIT_START   10U
#define HWACCREG1_FFT1DEN_BIT_END     10U
/** Definition for field ACCDYNCLKEN in Register HWACCREG1**/
#define HWACCREG1_ACCDYNCLKEN_BIT_START   11U
#define HWACCREG1_ACCDYNCLKEN_BIT_END     11U
/** Definition for field NU2 in Register HWACCREG1**/
#define HWACCREG1_NU2_BIT_START   12U
#define HWACCREG1_NU2_BIT_END     31U
/** Definition for field DMA2ACCTRIG in Register HWACCREG2**/
#define HWACCREG2_DMA2ACCTRIG_BIT_START   0U
#define HWACCREG2_DMA2ACCTRIG_BIT_END     15U
/** Definition for field NU in Register HWACCREG2**/
#define HWACCREG2_NU_BIT_START   16U
#define HWACCREG2_NU_BIT_END     31U
/** Definition for field CM42ACCTRIG in Register HWACCREG3**/
#define HWACCREG3_CM42ACCTRIG_BIT_START   0U
#define HWACCREG3_CM42ACCTRIG_BIT_END     0U
/** Definition for field NU in Register HWACCREG3**/
#define HWACCREG3_NU_BIT_START   1U
#define HWACCREG3_NU_BIT_END     15U
/** Definition for field CM42DMATRIG in Register HWACCREG3**/
#define HWACCREG3_CM42DMATRIG_BIT_START   16U
#define HWACCREG3_CM42DMATRIG_BIT_END     31U
/** Definition for field SPARE in Register HWACCREG4**/
#define HWACCREG4_SPARE_BIT_START   0U
#define HWACCREG4_SPARE_BIT_END     15U
/** Definition for field BPMPATTERNMSB in Register HWACCREG5**/
#define HWACCREG5_BPMPATTERNMSB_BIT_START   0U
#define HWACCREG5_BPMPATTERNMSB_BIT_END     31U
/** Definition for field BPMPATTERNLSB in Register HWACCREG6**/
#define HWACCREG6_BPMPATTERNLSB_BIT_START   0U
#define HWACCREG6_BPMPATTERNLSB_BIT_END     31U
/** Definition for field BPMRATE in Register HWACCREG7**/
#define HWACCREG7_BPMRATE_BIT_START   0U
#define HWACCREG7_BPMRATE_BIT_END     9U
/** Definition for field NU1 in Register HWACCREG7**/
#define HWACCREG7_NU1_BIT_START   10U
#define HWACCREG7_NU1_BIT_END     15U
/** Definition for field DITHERTWIDEN in Register HWACCREG7**/
#define HWACCREG7_DITHERTWIDEN_BIT_START   16U
#define HWACCREG7_DITHERTWIDEN_BIT_END     16U
/** Definition for field NU2 in Register HWACCREG7**/
#define HWACCREG7_NU2_BIT_START   17U
#define HWACCREG7_NU2_BIT_END     23U
/** Definition for field STG1LUTSELWR in Register HWACCREG7**/
#define HWACCREG7_STG1LUTSELWR_BIT_START   24U
#define HWACCREG7_STG1LUTSELWR_BIT_END     24U
/** Definition for field NU3 in Register HWACCREG7**/
#define HWACCREG7_NU3_BIT_START   25U
#define HWACCREG7_NU3_BIT_END     31U
/** Definition for field NU1 in Register HWACCREG8**/
#define HWACCREG8_NU1_BIT_START   0U
#define HWACCREG8_NU1_BIT_END     23U
/** Definition for field FFTSUMDIV in Register HWACCREG8**/
#define HWACCREG8_FFTSUMDIV_BIT_START   24U
#define HWACCREG8_FFTSUMDIV_BIT_END     28U
/** Definition for field NU2 in Register HWACCREG8**/
#define HWACCREG8_NU2_BIT_START   29U
#define HWACCREG8_NU2_BIT_END     31U
/** Definition for field CMULTSCALE1I in Register CMULTSCALE1I**/
#define CMULTSCALE1I_CMULTSCALE1I_BIT_START   0U
#define CMULTSCALE1I_CMULTSCALE1I_BIT_END     20U
/** Definition for field NU in Register CMULTSCALE1I**/
#define CMULTSCALE1I_NU_BIT_START   21U
#define CMULTSCALE1I_NU_BIT_END     31U
/** Definition for field CMULTSCALE1Q in Register CMULTSCALE1Q**/
#define CMULTSCALE1Q_CMULTSCALE1Q_BIT_START   0U
#define CMULTSCALE1Q_CMULTSCALE1Q_BIT_END     20U
/** Definition for field NU in Register CMULTSCALE1Q**/
#define CMULTSCALE1Q_NU_BIT_START   21U
#define CMULTSCALE1Q_NU_BIT_END     31U
/** Definition for field LFSRSEED in Register HWACCREG11**/
#define HWACCREG11_LFSRSEED_BIT_START   0U
#define HWACCREG11_LFSRSEED_BIT_END     28U
/** Definition for field NU in Register HWACCREG11**/
#define HWACCREG11_NU_BIT_START   29U
#define HWACCREG11_NU_BIT_END     30U
/** Definition for field LFSRLOAD in Register HWACCREG11**/
#define HWACCREG11_LFSRLOAD_BIT_START   31U
#define HWACCREG11_LFSRLOAD_BIT_END     31U
/** Definition for field ACC_TRIGGER_IN_STAT in Register HWACCREG12**/
#define HWACCREG12_ACC_TRIGGER_IN_STAT_BIT_START   0U
#define HWACCREG12_ACC_TRIGGER_IN_STAT_BIT_END     18U
/** Definition for field NU1 in Register HWACCREG12**/
#define HWACCREG12_NU1_BIT_START   19U
#define HWACCREG12_NU1_BIT_END     23U
/** Definition for field ACC_TRIGGER_IN_CLR in Register HWACCREG12**/
#define HWACCREG12_ACC_TRIGGER_IN_CLR_BIT_START   24U
#define HWACCREG12_ACC_TRIGGER_IN_CLR_BIT_END     24U
/** Definition for field NU2 in Register HWACCREG12**/
#define HWACCREG12_NU2_BIT_START   25U
#define HWACCREG12_NU2_BIT_END     31U
/** Definition for field CFAR_THRESH in Register HWACCREG13**/
#define HWACCREG13_CFAR_THRESH_BIT_START   0U
#define HWACCREG13_CFAR_THRESH_BIT_END     17U
/** Definition for field NU in Register HWACCREG13**/
#define HWACCREG13_NU_BIT_START   18U
#define HWACCREG13_NU_BIT_END     31U
/** Definition for field PARAMDONESTAT in Register HWACCREG14**/
#define HWACCREG14_PARAMDONESTAT_BIT_START   0U
#define HWACCREG14_PARAMDONESTAT_BIT_END     31U
/** Definition for field PARAMDONECLR in Register HWACCREG15**/
#define HWACCREG15_PARAMDONECLR_BIT_START   0U
#define HWACCREG15_PARAMDONECLR_BIT_END     31U
/** Definition for field MAX1VALUE in Register MAX1VALUE**/
#define MAX1VALUE_MAX1VALUE_BIT_START   0U
#define MAX1VALUE_MAX1VALUE_BIT_END     23U
/** Definition for field NU in Register MAX1VALUE**/
#define MAX1VALUE_NU_BIT_START   24U
#define MAX1VALUE_NU_BIT_END     31U
/** Definition for field MAX1INDEX in Register MAX1INDEX**/
#define MAX1INDEX_MAX1INDEX_BIT_START   0U
#define MAX1INDEX_MAX1INDEX_BIT_END     11U
/** Definition for field NU in Register MAX1INDEX**/
#define MAX1INDEX_NU_BIT_START   12U
#define MAX1INDEX_NU_BIT_END     31U
/** Definition for field ISUM1LSB in Register ISUM1LSB**/
#define ISUM1LSB_ISUM1LSB_BIT_START   0U
#define ISUM1LSB_ISUM1LSB_BIT_END     31U
/** Definition for field ISUM1MSB in Register ISUM1MSB**/
#define ISUM1MSB_ISUM1MSB_BIT_START   0U
#define ISUM1MSB_ISUM1MSB_BIT_END     3U
/** Definition for field NU in Register ISUM1MSB**/
#define ISUM1MSB_NU_BIT_START   4U
#define ISUM1MSB_NU_BIT_END     31U
/** Definition for field QSUM1LSB in Register QSUM1LSB**/
#define QSUM1LSB_QSUM1LSB_BIT_START   0U
#define QSUM1LSB_QSUM1LSB_BIT_END     31U
/** Definition for field QSUM1MSB in Register QSUM1MSB**/
#define QSUM1MSB_QSUM1MSB_BIT_START   0U
#define QSUM1MSB_QSUM1MSB_BIT_END     3U
/** Definition for field NU in Register QSUM1MSB**/
#define QSUM1MSB_NU_BIT_START   4U
#define QSUM1MSB_NU_BIT_END     31U
/** Definition for field MAX2VALUE in Register MAX2VALUE**/
#define MAX2VALUE_MAX2VALUE_BIT_START   0U
#define MAX2VALUE_MAX2VALUE_BIT_END     23U
/** Definition for field NU in Register MAX2VALUE**/
#define MAX2VALUE_NU_BIT_START   24U
#define MAX2VALUE_NU_BIT_END     31U
/** Definition for field MAX2INDEX in Register MAX2INDEX**/
#define MAX2INDEX_MAX2INDEX_BIT_START   0U
#define MAX2INDEX_MAX2INDEX_BIT_END     11U
/** Definition for field NU in Register MAX2INDEX**/
#define MAX2INDEX_NU_BIT_START   12U
#define MAX2INDEX_NU_BIT_END     31U
/** Definition for field ISUM2LSB in Register ISUM2LSB**/
#define ISUM2LSB_ISUM2LSB_BIT_START   0U
#define ISUM2LSB_ISUM2LSB_BIT_END     31U
/** Definition for field ISUM2MSB in Register ISUM2MSB**/
#define ISUM2MSB_ISUM2MSB_BIT_START   0U
#define ISUM2MSB_ISUM2MSB_BIT_END     3U
/** Definition for field NU in Register ISUM2MSB**/
#define ISUM2MSB_NU_BIT_START   4U
#define ISUM2MSB_NU_BIT_END     31U
/** Definition for field QSUM2LSB in Register QSUM2LSB**/
#define QSUM2LSB_QSUM2LSB_BIT_START   0U
#define QSUM2LSB_QSUM2LSB_BIT_END     31U
/** Definition for field QSUM2MSB in Register QSUM2MSB**/
#define QSUM2MSB_QSUM2MSB_BIT_START   0U
#define QSUM2MSB_QSUM2MSB_BIT_END     3U
/** Definition for field NU in Register QSUM2MSB**/
#define QSUM2MSB_NU_BIT_START   4U
#define QSUM2MSB_NU_BIT_END     31U
/** Definition for field MAX3VALUE in Register MAX3VALUE**/
#define MAX3VALUE_MAX3VALUE_BIT_START   0U
#define MAX3VALUE_MAX3VALUE_BIT_END     23U
/** Definition for field NU in Register MAX3VALUE**/
#define MAX3VALUE_NU_BIT_START   24U
#define MAX3VALUE_NU_BIT_END     31U
/** Definition for field MAX3INDEX in Register MAX3INDEX**/
#define MAX3INDEX_MAX3INDEX_BIT_START   0U
#define MAX3INDEX_MAX3INDEX_BIT_END     11U
/** Definition for field NU in Register MAX3INDEX**/
#define MAX3INDEX_NU_BIT_START   12U
#define MAX3INDEX_NU_BIT_END     31U
/** Definition for field ISUM3LSB in Register ISUM3LSB**/
#define ISUM3LSB_ISUM3LSB_BIT_START   0U
#define ISUM3LSB_ISUM3LSB_BIT_END     31U
/** Definition for field ISUM3MSB in Register ISUM3MSB**/
#define ISUM3MSB_ISUM3MSB_BIT_START   0U
#define ISUM3MSB_ISUM3MSB_BIT_END     3U
/** Definition for field NU in Register ISUM3MSB**/
#define ISUM3MSB_NU_BIT_START   4U
#define ISUM3MSB_NU_BIT_END     31U
/** Definition for field QSUM3LSB in Register QSUM3LSB**/
#define QSUM3LSB_QSUM3LSB_BIT_START   0U
#define QSUM3LSB_QSUM3LSB_BIT_END     31U
/** Definition for field QSUM3MSB in Register QSUM3MSB**/
#define QSUM3MSB_QSUM3MSB_BIT_START   0U
#define QSUM3MSB_QSUM3MSB_BIT_END     3U
/** Definition for field NU in Register QSUM3MSB**/
#define QSUM3MSB_NU_BIT_START   4U
#define QSUM3MSB_NU_BIT_END     31U
/** Definition for field MAX4VALUE in Register MAX4VALUE**/
#define MAX4VALUE_MAX4VALUE_BIT_START   0U
#define MAX4VALUE_MAX4VALUE_BIT_END     23U
/** Definition for field NU in Register MAX4VALUE**/
#define MAX4VALUE_NU_BIT_START   24U
#define MAX4VALUE_NU_BIT_END     31U
/** Definition for field MAX4INDEX in Register MAX4INDEX**/
#define MAX4INDEX_MAX4INDEX_BIT_START   0U
#define MAX4INDEX_MAX4INDEX_BIT_END     11U
/** Definition for field NU in Register MAX4INDEX**/
#define MAX4INDEX_NU_BIT_START   12U
#define MAX4INDEX_NU_BIT_END     31U
/** Definition for field ISUM4LSB in Register ISUM4LSB**/
#define ISUM4LSB_ISUM4LSB_BIT_START   0U
#define ISUM4LSB_ISUM4LSB_BIT_END     31U
/** Definition for field ISUM4MSB in Register ISUM4MSB**/
#define ISUM4MSB_ISUM4MSB_BIT_START   0U
#define ISUM4MSB_ISUM4MSB_BIT_END     3U
/** Definition for field NU in Register ISUM4MSB**/
#define ISUM4MSB_NU_BIT_START   4U
#define ISUM4MSB_NU_BIT_END     31U
/** Definition for field QSUM4LSB in Register QSUM4LSB**/
#define QSUM4LSB_QSUM4LSB_BIT_START   0U
#define QSUM4LSB_QSUM4LSB_BIT_END     31U
/** Definition for field QSUM4MSB in Register QSUM4MSB**/
#define QSUM4MSB_QSUM4MSB_BIT_START   0U
#define QSUM4MSB_QSUM4MSB_BIT_END     3U
/** Definition for field NU in Register QSUM4MSB**/
#define QSUM4MSB_NU_BIT_START   4U
#define QSUM4MSB_NU_BIT_END     31U
/** Definition for field PARAMADDR in Register RDSTATUS**/
#define RDSTATUS_PARAMADDR_BIT_START   0U
#define RDSTATUS_PARAMADDR_BIT_END     4U
/** Definition for field LOOPCNT in Register RDSTATUS**/
#define RDSTATUS_LOOPCNT_BIT_START   5U
#define RDSTATUS_LOOPCNT_BIT_END     16U
/** Definition for field NU in Register RDSTATUS**/
#define RDSTATUS_NU_BIT_START   17U
#define RDSTATUS_NU_BIT_END     31U
/** Definition for field SIGDMACH1DONE in Register SIGDMACH1DONE**/
#define SIGDMACH1DONE_SIGDMACH1DONE_BIT_START   0U
#define SIGDMACH1DONE_SIGDMACH1DONE_BIT_END     31U
/** Definition for field SIGDMACH2DONE in Register SIGDMACH2DONE**/
#define SIGDMACH2DONE_SIGDMACH2DONE_BIT_START   0U
#define SIGDMACH2DONE_SIGDMACH2DONE_BIT_END     31U
/** Definition for field SIGDMACH3DONE in Register SIGDMACH3DONE**/
#define SIGDMACH3DONE_SIGDMACH3DONE_BIT_START   0U
#define SIGDMACH3DONE_SIGDMACH3DONE_BIT_END     31U
/** Definition for field SIGDMACH4DONE in Register SIGDMACH4DONE**/
#define SIGDMACH4DONE_SIGDMACH4DONE_BIT_START   0U
#define SIGDMACH4DONE_SIGDMACH4DONE_BIT_END     31U
/** Definition for field SIGDMACH5DONE in Register SIGDMACH5DONE**/
#define SIGDMACH5DONE_SIGDMACH5DONE_BIT_START   0U
#define SIGDMACH5DONE_SIGDMACH5DONE_BIT_END     31U
/** Definition for field SIGDMACH6DONE in Register SIGDMACH6DONE**/
#define SIGDMACH6DONE_SIGDMACH6DONE_BIT_START   0U
#define SIGDMACH6DONE_SIGDMACH6DONE_BIT_END     31U
/** Definition for field SIGDMACH7DONE in Register SIGDMACH7DONE**/
#define SIGDMACH7DONE_SIGDMACH7DONE_BIT_START   0U
#define SIGDMACH7DONE_SIGDMACH7DONE_BIT_END     31U
/** Definition for field SIGDMACH8DONE in Register SIGDMACH8DONE**/
#define SIGDMACH8DONE_SIGDMACH8DONE_BIT_START   0U
#define SIGDMACH8DONE_SIGDMACH8DONE_BIT_END     31U
/** Definition for field SIGDMACH9DONE in Register SIGDMACH9DONE**/
#define SIGDMACH9DONE_SIGDMACH9DONE_BIT_START   0U
#define SIGDMACH9DONE_SIGDMACH9DONE_BIT_END     31U
/** Definition for field SIGDMACH10DONE in Register SIGDMACH10DONE**/
#define SIGDMACH10DONE_SIGDMACH10DONE_BIT_START   0U
#define SIGDMACH10DONE_SIGDMACH10DONE_BIT_END     31U
/** Definition for field SIGDMACH11DONE in Register SIGDMACH11DONE**/
#define SIGDMACH11DONE_SIGDMACH11DONE_BIT_START   0U
#define SIGDMACH11DONE_SIGDMACH11DONE_BIT_END     31U
/** Definition for field SIGDMACH12DONE in Register SIGDMACH12DONE**/
#define SIGDMACH12DONE_SIGDMACH12DONE_BIT_START   0U
#define SIGDMACH12DONE_SIGDMACH12DONE_BIT_END     31U
/** Definition for field SIGDMACH13DONE in Register SIGDMACH13DONE**/
#define SIGDMACH13DONE_SIGDMACH13DONE_BIT_START   0U
#define SIGDMACH13DONE_SIGDMACH13DONE_BIT_END     31U
/** Definition for field SIGDMACH14DONE in Register SIGDMACH14DONE**/
#define SIGDMACH14DONE_SIGDMACH14DONE_BIT_START   0U
#define SIGDMACH14DONE_SIGDMACH14DONE_BIT_END     31U
/** Definition for field SIGDMACH15DONE in Register SIGDMACH15DONE**/
#define SIGDMACH15DONE_SIGDMACH15DONE_BIT_START   0U
#define SIGDMACH15DONE_SIGDMACH15DONE_BIT_END     31U
/** Definition for field SIGDMACH16DONE in Register SIGDMACH16DONE**/
#define SIGDMACH16DONE_SIGDMACH16DONE_BIT_START   0U
#define SIGDMACH16DONE_SIGDMACH16DONE_BIT_END     31U
/** Definition for field FFTCLCIPSTAT in Register FFTCLIP**/
#define FFTCLIP_FFTCLCIPSTAT_BIT_START   0U
#define FFTCLIP_FFTCLCIPSTAT_BIT_END     9U
/** Definition for field NU1 in Register FFTCLIP**/
#define FFTCLIP_NU1_BIT_START   10U
#define FFTCLIP_NU1_BIT_END     15U
/** Definition for field CLRFFTCLIPSTAT in Register FFTCLIP**/
#define FFTCLIP_CLRFFTCLIPSTAT_BIT_START   16U
#define FFTCLIP_CLRFFTCLIPSTAT_BIT_END     16U
/** Definition for field NU2 in Register FFTCLIP**/
#define FFTCLIP_NU2_BIT_START   17U
#define FFTCLIP_NU2_BIT_END     31U
/** Definition for field FFTPEAKCNT in Register FFTPEAKCNT**/
#define FFTPEAKCNT_FFTPEAKCNT_BIT_START   0U
#define FFTPEAKCNT_FFTPEAKCNT_BIT_END     11U
/** Definition for field NU in Register FFTPEAKCNT**/
#define FFTPEAKCNT_NU_BIT_START   12U
#define FFTPEAKCNT_NU_BIT_END     31U
/** Definition for field CMP_EGE_K0 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_CMP_EGE_K0_BIT_START   0U
#define CMP_EGE_K0123_CMP_EGE_K0_BIT_END     4U
/** Definition for field NU1 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_NU1_BIT_START   5U
#define CMP_EGE_K0123_NU1_BIT_END     7U
/** Definition for field CMP_EGE_K1 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_CMP_EGE_K1_BIT_START   8U
#define CMP_EGE_K0123_CMP_EGE_K1_BIT_END     12U
/** Definition for field NU2 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_NU2_BIT_START   13U
#define CMP_EGE_K0123_NU2_BIT_END     15U
/** Definition for field CMP_EGE_K2 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_CMP_EGE_K2_BIT_START   16U
#define CMP_EGE_K0123_CMP_EGE_K2_BIT_END     20U
/** Definition for field NU3 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_NU3_BIT_START   21U
#define CMP_EGE_K0123_NU3_BIT_END     23U
/** Definition for field CMP_EGE_K3 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_CMP_EGE_K3_BIT_START   24U
#define CMP_EGE_K0123_CMP_EGE_K3_BIT_END     28U
/** Definition for field NU4 in Register CMP_EGE_K0123**/
#define CMP_EGE_K0123_NU4_BIT_START   29U
#define CMP_EGE_K0123_NU4_BIT_END     31U
/** Definition for field CMP_EGE_K4 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_CMP_EGE_K4_BIT_START   0U
#define CMP_EGE_K4567_CMP_EGE_K4_BIT_END     4U
/** Definition for field NU1 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_NU1_BIT_START   5U
#define CMP_EGE_K4567_NU1_BIT_END     7U
/** Definition for field CMP_EGE_K5 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_CMP_EGE_K5_BIT_START   8U
#define CMP_EGE_K4567_CMP_EGE_K5_BIT_END     12U
/** Definition for field NU2 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_NU2_BIT_START   13U
#define CMP_EGE_K4567_NU2_BIT_END     15U
/** Definition for field CMP_EGE_K6 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_CMP_EGE_K6_BIT_START   16U
#define CMP_EGE_K4567_CMP_EGE_K6_BIT_END     20U
/** Definition for field NU3 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_NU3_BIT_START   21U
#define CMP_EGE_K4567_NU3_BIT_END     23U
/** Definition for field CMP_EGE_K7 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_CMP_EGE_K7_BIT_START   24U
#define CMP_EGE_K4567_CMP_EGE_K7_BIT_END     28U
/** Definition for field NU4 in Register CMP_EGE_K4567**/
#define CMP_EGE_K4567_NU4_BIT_START   29U
#define CMP_EGE_K4567_NU4_BIT_END     31U
/** Definition for field WIN_RAM_PARITY_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_WIN_RAM_PARITY_EN_BIT_START   0U
#define HWA_SAFETY_ENABLE_WIN_RAM_PARITY_EN_BIT_END     0U
/** Definition for field PARAM_ECC_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_PARAM_ECC_EN_BIT_START   1U
#define HWA_SAFETY_ENABLE_PARAM_ECC_EN_BIT_END     1U
/** Definition for field NU1 in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_NU1_BIT_START   2U
#define HWA_SAFETY_ENABLE_NU1_BIT_END     11U
/** Definition for field IPING_PARITY_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_IPING_PARITY_EN_BIT_START   12U
#define HWA_SAFETY_ENABLE_IPING_PARITY_EN_BIT_END     12U
/** Definition for field IPONG_PARITY_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_IPONG_PARITY_EN_BIT_START   13U
#define HWA_SAFETY_ENABLE_IPONG_PARITY_EN_BIT_END     13U
/** Definition for field OPING_PARITY_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_OPING_PARITY_EN_BIT_START   14U
#define HWA_SAFETY_ENABLE_OPING_PARITY_EN_BIT_END     14U
/** Definition for field OPONG_PARITY_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_OPONG_PARITY_EN_BIT_START   15U
#define HWA_SAFETY_ENABLE_OPONG_PARITY_EN_BIT_END     15U
/** Definition for field FSM_LOCKSTEP_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_FSM_LOCKSTEP_EN_BIT_START   16U
#define HWA_SAFETY_ENABLE_FSM_LOCKSTEP_EN_BIT_END     16U
/** Definition for field FSM_LOCKSTEP_SELFTEST_EN in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_FSM_LOCKSTEP_SELFTEST_EN_BIT_START   17U
#define HWA_SAFETY_ENABLE_FSM_LOCKSTEP_SELFTEST_EN_BIT_END     17U
/** Definition for field NU2 in Register HWA_SAFETY_ENABLE**/
#define HWA_SAFETY_ENABLE_NU2_BIT_START   18U
#define HWA_SAFETY_ENABLE_NU2_BIT_END     31U
/** Definition for field WIN_RAM_INIT in Register MEMINIT**/
#define MEMINIT_WIN_RAM_INIT_BIT_START   0U
#define MEMINIT_WIN_RAM_INIT_BIT_END     0U
/** Definition for field PARAM_INIT in Register MEMINIT**/
#define MEMINIT_PARAM_INIT_BIT_START   1U
#define MEMINIT_PARAM_INIT_BIT_END     1U
/** Definition for field IPING_INIT in Register MEMINIT**/
#define MEMINIT_IPING_INIT_BIT_START   2U
#define MEMINIT_IPING_INIT_BIT_END     2U
/** Definition for field IPONG_INIT in Register MEMINIT**/
#define MEMINIT_IPONG_INIT_BIT_START   3U
#define MEMINIT_IPONG_INIT_BIT_END     3U
/** Definition for field OPING_INIT in Register MEMINIT**/
#define MEMINIT_OPING_INIT_BIT_START   4U
#define MEMINIT_OPING_INIT_BIT_END     4U
/** Definition for field OPONG_INIT in Register MEMINIT**/
#define MEMINIT_OPONG_INIT_BIT_START   5U
#define MEMINIT_OPONG_INIT_BIT_END     5U
/** Definition for field MC_EVEN_INIT in Register MEMINIT**/
#define MEMINIT_MC_EVEN_INIT_BIT_START   6U
#define MEMINIT_MC_EVEN_INIT_BIT_END     6U
/** Definition for field MC_ODD_INIT in Register MEMINIT**/
#define MEMINIT_MC_ODD_INIT_BIT_START   7U
#define MEMINIT_MC_ODD_INIT_BIT_END     7U
/** Definition for field NU in Register MEMINIT**/
#define MEMINIT_NU_BIT_START   8U
#define MEMINIT_NU_BIT_END     31U
/** Definition for field WIN_RAM_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_WIN_RAM_INITDONE_BIT_START   0U
#define MEMINITDONE_WIN_RAM_INITDONE_BIT_END     0U
/** Definition for field PARAM_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_PARAM_INITDONE_BIT_START   1U
#define MEMINITDONE_PARAM_INITDONE_BIT_END     1U
/** Definition for field IPING_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_IPING_INITDONE_BIT_START   2U
#define MEMINITDONE_IPING_INITDONE_BIT_END     2U
/** Definition for field IPONG_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_IPONG_INITDONE_BIT_START   3U
#define MEMINITDONE_IPONG_INITDONE_BIT_END     3U
/** Definition for field OPING_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_OPING_INITDONE_BIT_START   4U
#define MEMINITDONE_OPING_INITDONE_BIT_END     4U
/** Definition for field OPONG_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_OPONG_INITDONE_BIT_START   5U
#define MEMINITDONE_OPONG_INITDONE_BIT_END     5U
/** Definition for field MC_EVEN_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_MC_EVEN_INITDONE_BIT_START   6U
#define MEMINITDONE_MC_EVEN_INITDONE_BIT_END     6U
/** Definition for field MC_ODD_INITDONE in Register MEMINITDONE**/
#define MEMINITDONE_MC_ODD_INITDONE_BIT_START   7U
#define MEMINITDONE_MC_ODD_INITDONE_BIT_END     7U
/** Definition for field NU in Register MEMINITDONE**/
#define MEMINITDONE_NU_BIT_START   8U
#define MEMINITDONE_NU_BIT_END     31U
/** Definition for field HWA_SAFETY_WIN_RAM_ERR_ADDR in Register HWA_SAFETY_WIN_RAM_ERR_LOC**/
#define HWA_SAFETY_WIN_RAM_ERR_LOC_HWA_SAFETY_WIN_RAM_ERR_ADDR_BIT_START   0U
#define HWA_SAFETY_WIN_RAM_ERR_LOC_HWA_SAFETY_WIN_RAM_ERR_ADDR_BIT_END     15U
/** Definition for field NU in Register HWA_SAFETY_WIN_RAM_ERR_LOC**/
#define HWA_SAFETY_WIN_RAM_ERR_LOC_NU_BIT_START   16U
#define HWA_SAFETY_WIN_RAM_ERR_LOC_NU_BIT_END     31U
/** Definition for field NU1 in Register HWA_SAFETY_PARAM_RAM_ERR_LOC**/
#define HWA_SAFETY_PARAM_RAM_ERR_LOC_NU1_BIT_START   0U
#define HWA_SAFETY_PARAM_RAM_ERR_LOC_NU1_BIT_END     15U
/** Definition for field NU2 in Register HWA_SAFETY_PARAM_RAM_ERR_LOC**/
#define HWA_SAFETY_PARAM_RAM_ERR_LOC_NU2_BIT_START   16U
#define HWA_SAFETY_PARAM_RAM_ERR_LOC_NU2_BIT_END     31U
/** Definition for field HWA_SAFETY_IPING_ERR_ADDR in Register HWA_SAFETY_IPING_ERR_LOC**/
#define HWA_SAFETY_IPING_ERR_LOC_HWA_SAFETY_IPING_ERR_ADDR_BIT_START   0U
#define HWA_SAFETY_IPING_ERR_LOC_HWA_SAFETY_IPING_ERR_ADDR_BIT_END     15U
/** Definition for field NU in Register HWA_SAFETY_IPING_ERR_LOC**/
#define HWA_SAFETY_IPING_ERR_LOC_NU_BIT_START   16U
#define HWA_SAFETY_IPING_ERR_LOC_NU_BIT_END     31U
/** Definition for field HWA_SAFETY_IPONG_ERR_ADDR in Register HWA_SAFETY_IPONG_ERR_LOC**/
#define HWA_SAFETY_IPONG_ERR_LOC_HWA_SAFETY_IPONG_ERR_ADDR_BIT_START   0U
#define HWA_SAFETY_IPONG_ERR_LOC_HWA_SAFETY_IPONG_ERR_ADDR_BIT_END     15U
/** Definition for field NU in Register HWA_SAFETY_IPONG_ERR_LOC**/
#define HWA_SAFETY_IPONG_ERR_LOC_NU_BIT_START   16U
#define HWA_SAFETY_IPONG_ERR_LOC_NU_BIT_END     31U
/** Definition for field HWA_SAFETY_OPING_ERR_ADDR in Register HWA_SAFETY_OPING_ERR_LOC      **/
#define HWA_SAFETY_OPING_ERR_LOC_HWA_SAFETY_OPING_ERR_ADDR_BIT_START   0U
#define HWA_SAFETY_OPING_ERR_LOC_HWA_SAFETY_OPING_ERR_ADDR_BIT_END     15U
/** Definition for field NU in Register HWA_SAFETY_OPING_ERR_LOC      **/
#define HWA_SAFETY_OPING_ERR_LOC_NU_BIT_START   16U
#define HWA_SAFETY_OPING_ERR_LOC_NU_BIT_END     31U
/** Definition for field HWA_SAFETY_OPONG_ERR_ADDR in Register HWA_SAFETY_OPONG_ERR_LOC      **/
#define HWA_SAFETY_OPONG_ERR_LOC_HWA_SAFETY_OPONG_ERR_ADDR_BIT_START   0U
#define HWA_SAFETY_OPONG_ERR_LOC_HWA_SAFETY_OPONG_ERR_ADDR_BIT_END     15U
/** Definition for field NU in Register HWA_SAFETY_OPONG_ERR_LOC      **/
#define HWA_SAFETY_OPONG_ERR_LOC_NU_BIT_START   16U
#define HWA_SAFETY_OPONG_ERR_LOC_NU_BIT_END     31U
/** Definition for field NLOOPS in Register HWACCREG16**/
#define HWACCREG16_NLOOPS_BIT_START   0U
#define HWACCREG16_NLOOPS_BIT_END     11U
/** Definition for field PARAMSTART in Register HWACCREG16**/
#define HWACCREG16_PARAMSTART_BIT_START   12U
#define HWACCREG16_PARAMSTART_BIT_END     16U
/** Definition for field PARAMSTOP in Register HWACCREG16**/
#define HWACCREG16_PARAMSTOP_BIT_START   17U
#define HWACCREG16_PARAMSTOP_BIT_END     21U
/** Definition for field NU1 in Register HWACCREG16**/
#define HWACCREG16_NU1_BIT_START   22U
#define HWACCREG16_NU1_BIT_END     31U
/** Definition for field DCEST1I_SW in Register DCEST1I_SW**/
#define DCEST1I_SW_DCEST1I_SW_BIT_START   0U
#define DCEST1I_SW_DCEST1I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST1I_SW**/
#define DCEST1I_SW_NU1_BIT_START   24U
#define DCEST1I_SW_NU1_BIT_END     31U
/** Definition for field DCEST2I_SW in Register DCEST2I_SW**/
#define DCEST2I_SW_DCEST2I_SW_BIT_START   0U
#define DCEST2I_SW_DCEST2I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST2I_SW**/
#define DCEST2I_SW_NU1_BIT_START   24U
#define DCEST2I_SW_NU1_BIT_END     31U
/** Definition for field DCEST3I_SW in Register DCEST3I_SW**/
#define DCEST3I_SW_DCEST3I_SW_BIT_START   0U
#define DCEST3I_SW_DCEST3I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST3I_SW**/
#define DCEST3I_SW_NU1_BIT_START   24U
#define DCEST3I_SW_NU1_BIT_END     31U
/** Definition for field DCEST4I_SW in Register DCEST4I_SW**/
#define DCEST4I_SW_DCEST4I_SW_BIT_START   0U
#define DCEST4I_SW_DCEST4I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST4I_SW**/
#define DCEST4I_SW_NU1_BIT_START   24U
#define DCEST4I_SW_NU1_BIT_END     31U
/** Definition for field DCEST5I_SW in Register DCEST5I_SW**/
#define DCEST5I_SW_DCEST5I_SW_BIT_START   0U
#define DCEST5I_SW_DCEST5I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST5I_SW**/
#define DCEST5I_SW_NU1_BIT_START   24U
#define DCEST5I_SW_NU1_BIT_END     31U
/** Definition for field DCEST6I_SW in Register DCEST6I_SW**/
#define DCEST6I_SW_DCEST6I_SW_BIT_START   0U
#define DCEST6I_SW_DCEST6I_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST6I_SW**/
#define DCEST6I_SW_NU1_BIT_START   24U
#define DCEST6I_SW_NU1_BIT_END     31U
/** Definition for field DCEST1I in Register DCEST1I**/
#define DCEST1I_DCEST1I_BIT_START   0U
#define DCEST1I_DCEST1I_BIT_END     23U
/** Definition for field NU1 in Register DCEST1I**/
#define DCEST1I_NU1_BIT_START   24U
#define DCEST1I_NU1_BIT_END     31U
/** Definition for field DCEST2I in Register DCEST2I**/
#define DCEST2I_DCEST2I_BIT_START   0U
#define DCEST2I_DCEST2I_BIT_END     23U
/** Definition for field NU1 in Register DCEST2I**/
#define DCEST2I_NU1_BIT_START   24U
#define DCEST2I_NU1_BIT_END     31U
/** Definition for field DCEST3I in Register DCEST3I**/
#define DCEST3I_DCEST3I_BIT_START   0U
#define DCEST3I_DCEST3I_BIT_END     23U
/** Definition for field NU1 in Register DCEST3I**/
#define DCEST3I_NU1_BIT_START   24U
#define DCEST3I_NU1_BIT_END     31U
/** Definition for field DCEST4I in Register DCEST4I**/
#define DCEST4I_DCEST4I_BIT_START   0U
#define DCEST4I_DCEST4I_BIT_END     23U
/** Definition for field NU1 in Register DCEST4I**/
#define DCEST4I_NU1_BIT_START   24U
#define DCEST4I_NU1_BIT_END     31U
/** Definition for field DCEST5I in Register DCEST5I**/
#define DCEST5I_DCEST5I_BIT_START   0U
#define DCEST5I_DCEST5I_BIT_END     23U
/** Definition for field NU1 in Register DCEST5I**/
#define DCEST5I_NU1_BIT_START   24U
#define DCEST5I_NU1_BIT_END     31U
/** Definition for field DCEST6I in Register DCEST6I**/
#define DCEST6I_DCEST6I_BIT_START   0U
#define DCEST6I_DCEST6I_BIT_END     23U
/** Definition for field NU1 in Register DCEST6I**/
#define DCEST6I_NU1_BIT_START   24U
#define DCEST6I_NU1_BIT_END     31U
/** Definition for field DC_ACC1I_LSB in Register DC_ACC1I_LSB**/
#define DC_ACC1I_LSB_DC_ACC1I_LSB_BIT_START   0U
#define DC_ACC1I_LSB_DC_ACC1I_LSB_BIT_END     31U
/** Definition for field DC_ACC1I_MSB in Register DC_ACC1I_MSB**/
#define DC_ACC1I_MSB_DC_ACC1I_MSB_BIT_START   0U
#define DC_ACC1I_MSB_DC_ACC1I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC1I_MSB**/
#define DC_ACC1I_MSB_NU1_BIT_START   4U
#define DC_ACC1I_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC2I_LSB in Register DC_ACC2I_LSB**/
#define DC_ACC2I_LSB_DC_ACC2I_LSB_BIT_START   0U
#define DC_ACC2I_LSB_DC_ACC2I_LSB_BIT_END     31U
/** Definition for field DC_ACC2I_MSB in Register DC_ACC2I_MSB**/
#define DC_ACC2I_MSB_DC_ACC2I_MSB_BIT_START   0U
#define DC_ACC2I_MSB_DC_ACC2I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC2I_MSB**/
#define DC_ACC2I_MSB_NU1_BIT_START   4U
#define DC_ACC2I_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC3I_LSB in Register DC_ACC3I_LSB**/
#define DC_ACC3I_LSB_DC_ACC3I_LSB_BIT_START   0U
#define DC_ACC3I_LSB_DC_ACC3I_LSB_BIT_END     31U
/** Definition for field DC_ACC3I_MSB in Register DC_ACC3I_MSB**/
#define DC_ACC3I_MSB_DC_ACC3I_MSB_BIT_START   0U
#define DC_ACC3I_MSB_DC_ACC3I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC3I_MSB**/
#define DC_ACC3I_MSB_NU1_BIT_START   4U
#define DC_ACC3I_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC4I_LSB in Register DC_ACC4I_LSB**/
#define DC_ACC4I_LSB_DC_ACC4I_LSB_BIT_START   0U
#define DC_ACC4I_LSB_DC_ACC4I_LSB_BIT_END     31U
/** Definition for field DC_ACC4I_MSB in Register DC_ACC4I_MSB**/
#define DC_ACC4I_MSB_DC_ACC4I_MSB_BIT_START   0U
#define DC_ACC4I_MSB_DC_ACC4I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC4I_MSB**/
#define DC_ACC4I_MSB_NU1_BIT_START   4U
#define DC_ACC4I_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC5I_LSB in Register DC_ACC5I_LSB**/
#define DC_ACC5I_LSB_DC_ACC5I_LSB_BIT_START   0U
#define DC_ACC5I_LSB_DC_ACC5I_LSB_BIT_END     31U
/** Definition for field DC_ACC5I_MSB in Register DC_ACC5I_MSB**/
#define DC_ACC5I_MSB_DC_ACC5I_MSB_BIT_START   0U
#define DC_ACC5I_MSB_DC_ACC5I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC5I_MSB**/
#define DC_ACC5I_MSB_NU1_BIT_START   4U
#define DC_ACC5I_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC6I_LSB in Register DC_ACC6I_LSB**/
#define DC_ACC6I_LSB_DC_ACC6I_LSB_BIT_START   0U
#define DC_ACC6I_LSB_DC_ACC6I_LSB_BIT_END     31U
/** Definition for field DC_ACC6I_MSB in Register DC_ACC6I_MSB**/
#define DC_ACC6I_MSB_DC_ACC6I_MSB_BIT_START   0U
#define DC_ACC6I_MSB_DC_ACC6I_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC6I_MSB**/
#define DC_ACC6I_MSB_NU1_BIT_START   4U
#define DC_ACC6I_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH1_SW in Register INTF_MAGTHRESH1_SW**/
#define INTF_MAGTHRESH1_SW_INTF_MAGTHRESH1_SW_BIT_START   0U
#define INTF_MAGTHRESH1_SW_INTF_MAGTHRESH1_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH1_SW**/
#define INTF_MAGTHRESH1_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH1_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH2_SW in Register INTF_MAGTHRESH2_SW**/
#define INTF_MAGTHRESH2_SW_INTF_MAGTHRESH2_SW_BIT_START   0U
#define INTF_MAGTHRESH2_SW_INTF_MAGTHRESH2_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH2_SW**/
#define INTF_MAGTHRESH2_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH2_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH3_SW in Register INTF_MAGTHRESH3_SW**/
#define INTF_MAGTHRESH3_SW_INTF_MAGTHRESH3_SW_BIT_START   0U
#define INTF_MAGTHRESH3_SW_INTF_MAGTHRESH3_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH3_SW**/
#define INTF_MAGTHRESH3_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH3_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH4_SW in Register INTF_MAGTHRESH4_SW**/
#define INTF_MAGTHRESH4_SW_INTF_MAGTHRESH4_SW_BIT_START   0U
#define INTF_MAGTHRESH4_SW_INTF_MAGTHRESH4_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH4_SW**/
#define INTF_MAGTHRESH4_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH4_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH5_SW in Register INTF_MAGTHRESH5_SW**/
#define INTF_MAGTHRESH5_SW_INTF_MAGTHRESH5_SW_BIT_START   0U
#define INTF_MAGTHRESH5_SW_INTF_MAGTHRESH5_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH5_SW**/
#define INTF_MAGTHRESH5_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH5_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH6_SW in Register INTF_MAGTHRESH6_SW**/
#define INTF_MAGTHRESH6_SW_INTF_MAGTHRESH6_SW_BIT_START   0U
#define INTF_MAGTHRESH6_SW_INTF_MAGTHRESH6_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH6_SW**/
#define INTF_MAGTHRESH6_SW_NU1_BIT_START   24U
#define INTF_MAGTHRESH6_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH1_SW in Register INTF_MAGDIFFTHRESH1_SW**/
#define INTF_MAGDIFFTHRESH1_SW_INTF_MAGDIFFTHRESH1_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH1_SW_INTF_MAGDIFFTHRESH1_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH1_SW**/
#define INTF_MAGDIFFTHRESH1_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH1_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH2_SW in Register INTF_MAGDIFFTHRESH2_SW**/
#define INTF_MAGDIFFTHRESH2_SW_INTF_MAGDIFFTHRESH2_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH2_SW_INTF_MAGDIFFTHRESH2_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH2_SW**/
#define INTF_MAGDIFFTHRESH2_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH2_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH3_SW in Register INTF_MAGDIFFTHRESH3_SW**/
#define INTF_MAGDIFFTHRESH3_SW_INTF_MAGDIFFTHRESH3_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH3_SW_INTF_MAGDIFFTHRESH3_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH3_SW**/
#define INTF_MAGDIFFTHRESH3_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH3_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH4_SW in Register INTF_MAGDIFFTHRESH4_SW**/
#define INTF_MAGDIFFTHRESH4_SW_INTF_MAGDIFFTHRESH4_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH4_SW_INTF_MAGDIFFTHRESH4_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH4_SW**/
#define INTF_MAGDIFFTHRESH4_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH4_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH5_SW in Register INTF_MAGDIFFTHRESH5_SW**/
#define INTF_MAGDIFFTHRESH5_SW_INTF_MAGDIFFTHRESH5_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH5_SW_INTF_MAGDIFFTHRESH5_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH5_SW**/
#define INTF_MAGDIFFTHRESH5_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH5_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH6_SW in Register INTF_MAGDIFFTHRESH6_SW**/
#define INTF_MAGDIFFTHRESH6_SW_INTF_MAGDIFFTHRESH6_SW_BIT_START   0U
#define INTF_MAGDIFFTHRESH6_SW_INTF_MAGDIFFTHRESH6_SW_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH6_SW**/
#define INTF_MAGDIFFTHRESH6_SW_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH6_SW_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC1_LSB in Register INTF_MAGACC1_LSB**/
#define INTF_MAGACC1_LSB_INTF_MAGACC1_LSB_BIT_START   0U
#define INTF_MAGACC1_LSB_INTF_MAGACC1_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC1_MSB in Register INTF_MAGACC1_MSB**/
#define INTF_MAGACC1_MSB_INTF_MAGACC1_MSB_BIT_START   0U
#define INTF_MAGACC1_MSB_INTF_MAGACC1_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC1_MSB**/
#define INTF_MAGACC1_MSB_NU1_BIT_START   4U
#define INTF_MAGACC1_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC2_LSB in Register INTF_MAGACC2_LSB**/
#define INTF_MAGACC2_LSB_INTF_MAGACC2_LSB_BIT_START   0U
#define INTF_MAGACC2_LSB_INTF_MAGACC2_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC2_MSB in Register INTF_MAGACC2_MSB**/
#define INTF_MAGACC2_MSB_INTF_MAGACC2_MSB_BIT_START   0U
#define INTF_MAGACC2_MSB_INTF_MAGACC2_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC2_MSB**/
#define INTF_MAGACC2_MSB_NU1_BIT_START   4U
#define INTF_MAGACC2_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC3_LSB in Register INTF_MAGACC3_LSB**/
#define INTF_MAGACC3_LSB_INTF_MAGACC3_LSB_BIT_START   0U
#define INTF_MAGACC3_LSB_INTF_MAGACC3_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC3_MSB in Register INTF_MAGACC3_MSB**/
#define INTF_MAGACC3_MSB_INTF_MAGACC3_MSB_BIT_START   0U
#define INTF_MAGACC3_MSB_INTF_MAGACC3_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC3_MSB**/
#define INTF_MAGACC3_MSB_NU1_BIT_START   4U
#define INTF_MAGACC3_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC4_LSB in Register INTF_MAGACC4_LSB**/
#define INTF_MAGACC4_LSB_INTF_MAGACC4_LSB_BIT_START   0U
#define INTF_MAGACC4_LSB_INTF_MAGACC4_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC4_MSB in Register INTF_MAGACC4_MSB**/
#define INTF_MAGACC4_MSB_INTF_MAGACC4_MSB_BIT_START   0U
#define INTF_MAGACC4_MSB_INTF_MAGACC4_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC4_MSB**/
#define INTF_MAGACC4_MSB_NU1_BIT_START   4U
#define INTF_MAGACC4_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC5_LSB in Register INTF_MAGACC5_LSB**/
#define INTF_MAGACC5_LSB_INTF_MAGACC5_LSB_BIT_START   0U
#define INTF_MAGACC5_LSB_INTF_MAGACC5_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC5_MSB in Register INTF_MAGACC5_MSB**/
#define INTF_MAGACC5_MSB_INTF_MAGACC5_MSB_BIT_START   0U
#define INTF_MAGACC5_MSB_INTF_MAGACC5_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC5_MSB**/
#define INTF_MAGACC5_MSB_NU1_BIT_START   4U
#define INTF_MAGACC5_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC6_LSB in Register INTF_MAGACC6_LSB**/
#define INTF_MAGACC6_LSB_INTF_MAGACC6_LSB_BIT_START   0U
#define INTF_MAGACC6_LSB_INTF_MAGACC6_LSB_BIT_END     31U
/** Definition for field INTF_MAGACC6_MSB in Register INTF_MAGACC6_MSB**/
#define INTF_MAGACC6_MSB_INTF_MAGACC6_MSB_BIT_START   0U
#define INTF_MAGACC6_MSB_INTF_MAGACC6_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGACC6_MSB**/
#define INTF_MAGACC6_MSB_NU1_BIT_START   4U
#define INTF_MAGACC6_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC1_LSB in Register INTF_MAGDIFFACC1_LSB**/
#define INTF_MAGDIFFACC1_LSB_INTF_MAGDIFFACC1_LSB_BIT_START   0U
#define INTF_MAGDIFFACC1_LSB_INTF_MAGDIFFACC1_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC1_MSB in Register INTF_MAGDIFFACC1_MSB**/
#define INTF_MAGDIFFACC1_MSB_INTF_MAGDIFFACC1_MSB_BIT_START   0U
#define INTF_MAGDIFFACC1_MSB_INTF_MAGDIFFACC1_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC1_MSB**/
#define INTF_MAGDIFFACC1_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC1_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC2_LSB in Register INTF_MAGDIFFACC2_LSB**/
#define INTF_MAGDIFFACC2_LSB_INTF_MAGDIFFACC2_LSB_BIT_START   0U
#define INTF_MAGDIFFACC2_LSB_INTF_MAGDIFFACC2_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC2_MSB in Register INTF_MAGDIFFACC2_MSB**/
#define INTF_MAGDIFFACC2_MSB_INTF_MAGDIFFACC2_MSB_BIT_START   0U
#define INTF_MAGDIFFACC2_MSB_INTF_MAGDIFFACC2_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC2_MSB**/
#define INTF_MAGDIFFACC2_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC2_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC3_LSB in Register INTF_MAGDIFFACC3_LSB**/
#define INTF_MAGDIFFACC3_LSB_INTF_MAGDIFFACC3_LSB_BIT_START   0U
#define INTF_MAGDIFFACC3_LSB_INTF_MAGDIFFACC3_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC3_MSB in Register INTF_MAGDIFFACC3_MSB**/
#define INTF_MAGDIFFACC3_MSB_INTF_MAGDIFFACC3_MSB_BIT_START   0U
#define INTF_MAGDIFFACC3_MSB_INTF_MAGDIFFACC3_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC3_MSB**/
#define INTF_MAGDIFFACC3_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC3_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC4_LSB in Register INTF_MAGDIFFACC4_LSB**/
#define INTF_MAGDIFFACC4_LSB_INTF_MAGDIFFACC4_LSB_BIT_START   0U
#define INTF_MAGDIFFACC4_LSB_INTF_MAGDIFFACC4_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC4_MSB in Register INTF_MAGDIFFACC4_MSB**/
#define INTF_MAGDIFFACC4_MSB_INTF_MAGDIFFACC4_MSB_BIT_START   0U
#define INTF_MAGDIFFACC4_MSB_INTF_MAGDIFFACC4_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC4_MSB**/
#define INTF_MAGDIFFACC4_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC4_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC5_LSB in Register INTF_MAGDIFFACC5_LSB**/
#define INTF_MAGDIFFACC5_LSB_INTF_MAGDIFFACC5_LSB_BIT_START   0U
#define INTF_MAGDIFFACC5_LSB_INTF_MAGDIFFACC5_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC5_MSB in Register INTF_MAGDIFFACC5_MSB**/
#define INTF_MAGDIFFACC5_MSB_INTF_MAGDIFFACC5_MSB_BIT_START   0U
#define INTF_MAGDIFFACC5_MSB_INTF_MAGDIFFACC5_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC5_MSB**/
#define INTF_MAGDIFFACC5_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC5_MSB_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC6_LSB in Register INTF_MAGDIFFACC6_LSB**/
#define INTF_MAGDIFFACC6_LSB_INTF_MAGDIFFACC6_LSB_BIT_START   0U
#define INTF_MAGDIFFACC6_LSB_INTF_MAGDIFFACC6_LSB_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC6_MSB in Register INTF_MAGDIFFACC6_MSB**/
#define INTF_MAGDIFFACC6_MSB_INTF_MAGDIFFACC6_MSB_BIT_START   0U
#define INTF_MAGDIFFACC6_MSB_INTF_MAGDIFFACC6_MSB_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFFACC6_MSB**/
#define INTF_MAGDIFFACC6_MSB_NU1_BIT_START   4U
#define INTF_MAGDIFFACC6_MSB_NU1_BIT_END     31U
/** Definition for field DCEST1_CLIP in Register DCEST1_CLIP**/
#define DCEST1_CLIP_DCEST1_CLIP_BIT_START   0U
#define DCEST1_CLIP_DCEST1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST1_CLIP**/
#define DCEST1_CLIP_NU1_BIT_START   1U
#define DCEST1_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST2_CLIP in Register DCEST2_CLIP**/
#define DCEST2_CLIP_DCEST2_CLIP_BIT_START   0U
#define DCEST2_CLIP_DCEST2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST2_CLIP**/
#define DCEST2_CLIP_NU1_BIT_START   1U
#define DCEST2_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST3_CLIP in Register DCEST3_CLIP**/
#define DCEST3_CLIP_DCEST3_CLIP_BIT_START   0U
#define DCEST3_CLIP_DCEST3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST3_CLIP**/
#define DCEST3_CLIP_NU1_BIT_START   1U
#define DCEST3_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST4_CLIP in Register DCEST4_CLIP**/
#define DCEST4_CLIP_DCEST4_CLIP_BIT_START   0U
#define DCEST4_CLIP_DCEST4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST4_CLIP**/
#define DCEST4_CLIP_NU1_BIT_START   1U
#define DCEST4_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST5_CLIP in Register DCEST5_CLIP**/
#define DCEST5_CLIP_DCEST5_CLIP_BIT_START   0U
#define DCEST5_CLIP_DCEST5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST5_CLIP**/
#define DCEST5_CLIP_NU1_BIT_START   1U
#define DCEST5_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST6_CLIP in Register DCEST6_CLIP**/
#define DCEST6_CLIP_DCEST6_CLIP_BIT_START   0U
#define DCEST6_CLIP_DCEST6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCEST6_CLIP**/
#define DCEST6_CLIP_NU1_BIT_START   1U
#define DCEST6_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC1_CLIP in Register DCACC1_CLIP**/
#define DCACC1_CLIP_DCACC1_CLIP_BIT_START   0U
#define DCACC1_CLIP_DCACC1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC1_CLIP**/
#define DCACC1_CLIP_NU1_BIT_START   1U
#define DCACC1_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC2_CLIP in Register DCACC2_CLIP**/
#define DCACC2_CLIP_DCACC2_CLIP_BIT_START   0U
#define DCACC2_CLIP_DCACC2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC2_CLIP**/
#define DCACC2_CLIP_NU1_BIT_START   1U
#define DCACC2_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC3_CLIP in Register DCACC3_CLIP**/
#define DCACC3_CLIP_DCACC3_CLIP_BIT_START   0U
#define DCACC3_CLIP_DCACC3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC3_CLIP**/
#define DCACC3_CLIP_NU1_BIT_START   1U
#define DCACC3_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC4_CLIP in Register DCACC4_CLIP**/
#define DCACC4_CLIP_DCACC4_CLIP_BIT_START   0U
#define DCACC4_CLIP_DCACC4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC4_CLIP**/
#define DCACC4_CLIP_NU1_BIT_START   1U
#define DCACC4_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC5_CLIP in Register DCACC5_CLIP**/
#define DCACC5_CLIP_DCACC5_CLIP_BIT_START   0U
#define DCACC5_CLIP_DCACC5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC5_CLIP**/
#define DCACC5_CLIP_NU1_BIT_START   1U
#define DCACC5_CLIP_NU1_BIT_END     31U
/** Definition for field DCACC6_CLIP in Register DCACC6_CLIP**/
#define DCACC6_CLIP_DCACC6_CLIP_BIT_START   0U
#define DCACC6_CLIP_DCACC6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCACC6_CLIP**/
#define DCACC6_CLIP_NU1_BIT_START   1U
#define DCACC6_CLIP_NU1_BIT_END     31U
/** Definition for field DCEST_SHIFT in Register DCEST_SHIFT**/
#define DCEST_SHIFT_DCEST_SHIFT_BIT_START   0U
#define DCEST_SHIFT_DCEST_SHIFT_BIT_END     3U
/** Definition for field NU1 in Register DCEST_SHIFT**/
#define DCEST_SHIFT_NU1_BIT_START   4U
#define DCEST_SHIFT_NU1_BIT_END     31U
/** Definition for field DCEST_SCALE in Register DCEST_SCALE**/
#define DCEST_SCALE_DCEST_SCALE_BIT_START   0U
#define DCEST_SCALE_DCEST_SCALE_BIT_END     8U
/** Definition for field NU1 in Register DCEST_SCALE**/
#define DCEST_SCALE_NU1_BIT_START   9U
#define DCEST_SCALE_NU1_BIT_END     31U
/** Definition for field DCSUB1_CLIP in Register DCSUB1_CLIP**/
#define DCSUB1_CLIP_DCSUB1_CLIP_BIT_START   0U
#define DCSUB1_CLIP_DCSUB1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB1_CLIP**/
#define DCSUB1_CLIP_NU1_BIT_START   1U
#define DCSUB1_CLIP_NU1_BIT_END     31U
/** Definition for field DCSUB2_CLIP in Register DCSUB2_CLIP**/
#define DCSUB2_CLIP_DCSUB2_CLIP_BIT_START   0U
#define DCSUB2_CLIP_DCSUB2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB2_CLIP**/
#define DCSUB2_CLIP_NU1_BIT_START   1U
#define DCSUB2_CLIP_NU1_BIT_END     31U
/** Definition for field DCSUB3_CLIP in Register DCSUB3_CLIP**/
#define DCSUB3_CLIP_DCSUB3_CLIP_BIT_START   0U
#define DCSUB3_CLIP_DCSUB3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB3_CLIP**/
#define DCSUB3_CLIP_NU1_BIT_START   1U
#define DCSUB3_CLIP_NU1_BIT_END     31U
/** Definition for field DCSUB4_CLIP in Register DCSUB4_CLIP**/
#define DCSUB4_CLIP_DCSUB4_CLIP_BIT_START   0U
#define DCSUB4_CLIP_DCSUB4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB4_CLIP**/
#define DCSUB4_CLIP_NU1_BIT_START   1U
#define DCSUB4_CLIP_NU1_BIT_END     31U
/** Definition for field DCSUB5_CLIP in Register DCSUB5_CLIP**/
#define DCSUB5_CLIP_DCSUB5_CLIP_BIT_START   0U
#define DCSUB5_CLIP_DCSUB5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB5_CLIP**/
#define DCSUB5_CLIP_NU1_BIT_START   1U
#define DCSUB5_CLIP_NU1_BIT_END     31U
/** Definition for field DCSUB6_CLIP in Register DCSUB6_CLIP**/
#define DCSUB6_CLIP_DCSUB6_CLIP_BIT_START   0U
#define DCSUB6_CLIP_DCSUB6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register DCSUB6_CLIP**/
#define DCSUB6_CLIP_NU1_BIT_START   1U
#define DCSUB6_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAG_SCALE in Register INTF_MAG_SCALE**/
#define INTF_MAG_SCALE_INTF_MAG_SCALE_BIT_START   0U
#define INTF_MAG_SCALE_INTF_MAG_SCALE_BIT_END     7U
/** Definition for field NU1 in Register INTF_MAG_SCALE**/
#define INTF_MAG_SCALE_NU1_BIT_START   8U
#define INTF_MAG_SCALE_NU1_BIT_END     31U
/** Definition for field INTF_MAG_SHIFT in Register INTF_MAG_SHIFT**/
#define INTF_MAG_SHIFT_INTF_MAG_SHIFT_BIT_START   0U
#define INTF_MAG_SHIFT_INTF_MAG_SHIFT_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAG_SHIFT**/
#define INTF_MAG_SHIFT_NU1_BIT_START   4U
#define INTF_MAG_SHIFT_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFF_SCALE in Register INTF_MAGDIFF_SCALE**/
#define INTF_MAGDIFF_SCALE_INTF_MAGDIFF_SCALE_BIT_START   0U
#define INTF_MAGDIFF_SCALE_INTF_MAGDIFF_SCALE_BIT_END     7U
/** Definition for field NU1 in Register INTF_MAGDIFF_SCALE**/
#define INTF_MAGDIFF_SCALE_NU1_BIT_START   8U
#define INTF_MAGDIFF_SCALE_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFF_SHIFT in Register INTF_MAGDIFF_SHIFT**/
#define INTF_MAGDIFF_SHIFT_INTF_MAGDIFF_SHIFT_BIT_START   0U
#define INTF_MAGDIFF_SHIFT_INTF_MAGDIFF_SHIFT_BIT_END     3U
/** Definition for field NU1 in Register INTF_MAGDIFF_SHIFT**/
#define INTF_MAGDIFF_SHIFT_NU1_BIT_START   4U
#define INTF_MAGDIFF_SHIFT_NU1_BIT_END     31U
/** Definition for field CLR_MISC_CLIP in Register CLR_MISC_CLIP**/
#define CLR_MISC_CLIP_CLR_MISC_CLIP_BIT_START   0U
#define CLR_MISC_CLIP_CLR_MISC_CLIP_BIT_END     0U
/** Definition for field NU1 in Register CLR_MISC_CLIP**/
#define CLR_MISC_CLIP_NU1_BIT_START   1U
#define CLR_MISC_CLIP_NU1_BIT_END     31U
/** Definition for field CMULTSCALE2I in Register CMULTSCALE2I**/
#define CMULTSCALE2I_CMULTSCALE2I_BIT_START   0U
#define CMULTSCALE2I_CMULTSCALE2I_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE2I**/
#define CMULTSCALE2I_NU1_BIT_START   21U
#define CMULTSCALE2I_NU1_BIT_END     31U
/** Definition for field CMULTSCALE3I in Register CMULTSCALE3I**/
#define CMULTSCALE3I_CMULTSCALE3I_BIT_START   0U
#define CMULTSCALE3I_CMULTSCALE3I_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE3I**/
#define CMULTSCALE3I_NU1_BIT_START   21U
#define CMULTSCALE3I_NU1_BIT_END     31U
/** Definition for field CMULTSCALE4I in Register CMULTSCALE4I**/
#define CMULTSCALE4I_CMULTSCALE4I_BIT_START   0U
#define CMULTSCALE4I_CMULTSCALE4I_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE4I**/
#define CMULTSCALE4I_NU1_BIT_START   21U
#define CMULTSCALE4I_NU1_BIT_END     31U
/** Definition for field CMULTSCALE5I in Register CMULTSCALE5I**/
#define CMULTSCALE5I_CMULTSCALE5I_BIT_START   0U
#define CMULTSCALE5I_CMULTSCALE5I_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE5I**/
#define CMULTSCALE5I_NU1_BIT_START   21U
#define CMULTSCALE5I_NU1_BIT_END     31U
/** Definition for field CMULTSCALE6I in Register CMULTSCALE6I**/
#define CMULTSCALE6I_CMULTSCALE6I_BIT_START   0U
#define CMULTSCALE6I_CMULTSCALE6I_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE6I**/
#define CMULTSCALE6I_NU1_BIT_START   21U
#define CMULTSCALE6I_NU1_BIT_END     31U
/** Definition for field CMULTSCALE2Q in Register CMULTSCALE2Q**/
#define CMULTSCALE2Q_CMULTSCALE2Q_BIT_START   0U
#define CMULTSCALE2Q_CMULTSCALE2Q_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE2Q**/
#define CMULTSCALE2Q_NU1_BIT_START   21U
#define CMULTSCALE2Q_NU1_BIT_END     31U
/** Definition for field CMULTSCALE3Q in Register CMULTSCALE3Q**/
#define CMULTSCALE3Q_CMULTSCALE3Q_BIT_START   0U
#define CMULTSCALE3Q_CMULTSCALE3Q_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE3Q**/
#define CMULTSCALE3Q_NU1_BIT_START   21U
#define CMULTSCALE3Q_NU1_BIT_END     31U
/** Definition for field CMULTSCALE4Q in Register CMULTSCALE4Q**/
#define CMULTSCALE4Q_CMULTSCALE4Q_BIT_START   0U
#define CMULTSCALE4Q_CMULTSCALE4Q_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE4Q**/
#define CMULTSCALE4Q_NU1_BIT_START   21U
#define CMULTSCALE4Q_NU1_BIT_END     31U
/** Definition for field CMULTSCALE5Q in Register CMULTSCALE5Q**/
#define CMULTSCALE5Q_CMULTSCALE5Q_BIT_START   0U
#define CMULTSCALE5Q_CMULTSCALE5Q_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE5Q**/
#define CMULTSCALE5Q_NU1_BIT_START   21U
#define CMULTSCALE5Q_NU1_BIT_END     31U
/** Definition for field CMULTSCALE6Q in Register CMULTSCALE6Q**/
#define CMULTSCALE6Q_CMULTSCALE6Q_BIT_START   0U
#define CMULTSCALE6Q_CMULTSCALE6Q_BIT_END     20U
/** Definition for field NU1 in Register CMULTSCALE6Q**/
#define CMULTSCALE6Q_NU1_BIT_START   21U
#define CMULTSCALE6Q_NU1_BIT_END     31U
/** Definition for field DCEST1Q_SW in Register DCEST1Q_SW**/
#define DCEST1Q_SW_DCEST1Q_SW_BIT_START   0U
#define DCEST1Q_SW_DCEST1Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST1Q_SW**/
#define DCEST1Q_SW_NU1_BIT_START   24U
#define DCEST1Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST2Q_SW in Register DCEST2Q_SW**/
#define DCEST2Q_SW_DCEST2Q_SW_BIT_START   0U
#define DCEST2Q_SW_DCEST2Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST2Q_SW**/
#define DCEST2Q_SW_NU1_BIT_START   24U
#define DCEST2Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST3Q_SW in Register DCEST3Q_SW**/
#define DCEST3Q_SW_DCEST3Q_SW_BIT_START   0U
#define DCEST3Q_SW_DCEST3Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST3Q_SW**/
#define DCEST3Q_SW_NU1_BIT_START   24U
#define DCEST3Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST4Q_SW in Register DCEST4Q_SW**/
#define DCEST4Q_SW_DCEST4Q_SW_BIT_START   0U
#define DCEST4Q_SW_DCEST4Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST4Q_SW**/
#define DCEST4Q_SW_NU1_BIT_START   24U
#define DCEST4Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST5Q_SW in Register DCEST5Q_SW**/
#define DCEST5Q_SW_DCEST5Q_SW_BIT_START   0U
#define DCEST5Q_SW_DCEST5Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST5Q_SW**/
#define DCEST5Q_SW_NU1_BIT_START   24U
#define DCEST5Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST6Q_SW in Register DCEST6Q_SW**/
#define DCEST6Q_SW_DCEST6Q_SW_BIT_START   0U
#define DCEST6Q_SW_DCEST6Q_SW_BIT_END     23U
/** Definition for field NU1 in Register DCEST6Q_SW**/
#define DCEST6Q_SW_NU1_BIT_START   24U
#define DCEST6Q_SW_NU1_BIT_END     31U
/** Definition for field DCEST1Q in Register DCEST1Q**/
#define DCEST1Q_DCEST1Q_BIT_START   0U
#define DCEST1Q_DCEST1Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST1Q**/
#define DCEST1Q_NU1_BIT_START   24U
#define DCEST1Q_NU1_BIT_END     31U
/** Definition for field DCEST2Q in Register DCEST2Q**/
#define DCEST2Q_DCEST2Q_BIT_START   0U
#define DCEST2Q_DCEST2Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST2Q**/
#define DCEST2Q_NU1_BIT_START   24U
#define DCEST2Q_NU1_BIT_END     31U
/** Definition for field DCEST3Q in Register DCEST3Q**/
#define DCEST3Q_DCEST3Q_BIT_START   0U
#define DCEST3Q_DCEST3Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST3Q**/
#define DCEST3Q_NU1_BIT_START   24U
#define DCEST3Q_NU1_BIT_END     31U
/** Definition for field DCEST4Q in Register DCEST4Q**/
#define DCEST4Q_DCEST4Q_BIT_START   0U
#define DCEST4Q_DCEST4Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST4Q**/
#define DCEST4Q_NU1_BIT_START   24U
#define DCEST4Q_NU1_BIT_END     31U
/** Definition for field DCEST5Q in Register DCEST5Q**/
#define DCEST5Q_DCEST5Q_BIT_START   0U
#define DCEST5Q_DCEST5Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST5Q**/
#define DCEST5Q_NU1_BIT_START   24U
#define DCEST5Q_NU1_BIT_END     31U
/** Definition for field DCEST6Q in Register DCEST6Q**/
#define DCEST6Q_DCEST6Q_BIT_START   0U
#define DCEST6Q_DCEST6Q_BIT_END     23U
/** Definition for field NU1 in Register DCEST6Q**/
#define DCEST6Q_NU1_BIT_START   24U
#define DCEST6Q_NU1_BIT_END     31U
/** Definition for field DC_ACC1Q_LSB in Register DC_ACC1Q_LSB**/
#define DC_ACC1Q_LSB_DC_ACC1Q_LSB_BIT_START   0U
#define DC_ACC1Q_LSB_DC_ACC1Q_LSB_BIT_END     31U
/** Definition for field DC_ACC1Q_MSB in Register DC_ACC1Q_MSB**/
#define DC_ACC1Q_MSB_DC_ACC1Q_MSB_BIT_START   0U
#define DC_ACC1Q_MSB_DC_ACC1Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC1Q_MSB**/
#define DC_ACC1Q_MSB_NU1_BIT_START   4U
#define DC_ACC1Q_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC2Q_LSB in Register DC_ACC2Q_LSB**/
#define DC_ACC2Q_LSB_DC_ACC2Q_LSB_BIT_START   0U
#define DC_ACC2Q_LSB_DC_ACC2Q_LSB_BIT_END     31U
/** Definition for field DC_ACC2Q_MSB in Register DC_ACC2Q_MSB**/
#define DC_ACC2Q_MSB_DC_ACC2Q_MSB_BIT_START   0U
#define DC_ACC2Q_MSB_DC_ACC2Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC2Q_MSB**/
#define DC_ACC2Q_MSB_NU1_BIT_START   4U
#define DC_ACC2Q_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC3Q_LSB in Register DC_ACC3Q_LSB**/
#define DC_ACC3Q_LSB_DC_ACC3Q_LSB_BIT_START   0U
#define DC_ACC3Q_LSB_DC_ACC3Q_LSB_BIT_END     31U
/** Definition for field DC_ACC3Q_MSB in Register DC_ACC3Q_MSB**/
#define DC_ACC3Q_MSB_DC_ACC3Q_MSB_BIT_START   0U
#define DC_ACC3Q_MSB_DC_ACC3Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC3Q_MSB**/
#define DC_ACC3Q_MSB_NU1_BIT_START   4U
#define DC_ACC3Q_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC4Q_LSB in Register DC_ACC4Q_LSB**/
#define DC_ACC4Q_LSB_DC_ACC4Q_LSB_BIT_START   0U
#define DC_ACC4Q_LSB_DC_ACC4Q_LSB_BIT_END     31U
/** Definition for field DC_ACC4Q_MSB in Register DC_ACC4Q_MSB**/
#define DC_ACC4Q_MSB_DC_ACC4Q_MSB_BIT_START   0U
#define DC_ACC4Q_MSB_DC_ACC4Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC4Q_MSB**/
#define DC_ACC4Q_MSB_NU1_BIT_START   4U
#define DC_ACC4Q_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC5Q_LSB in Register DC_ACC5Q_LSB**/
#define DC_ACC5Q_LSB_DC_ACC5Q_LSB_BIT_START   0U
#define DC_ACC5Q_LSB_DC_ACC5Q_LSB_BIT_END     31U
/** Definition for field DC_ACC5Q_MSB in Register DC_ACC5Q_MSB**/
#define DC_ACC5Q_MSB_DC_ACC5Q_MSB_BIT_START   0U
#define DC_ACC5Q_MSB_DC_ACC5Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC5Q_MSB**/
#define DC_ACC5Q_MSB_NU1_BIT_START   4U
#define DC_ACC5Q_MSB_NU1_BIT_END     31U
/** Definition for field DC_ACC6Q_LSB in Register DC_ACC6Q_LSB**/
#define DC_ACC6Q_LSB_DC_ACC6Q_LSB_BIT_START   0U
#define DC_ACC6Q_LSB_DC_ACC6Q_LSB_BIT_END     31U
/** Definition for field DC_ACC6Q_MSB in Register DC_ACC6Q_MSB**/
#define DC_ACC6Q_MSB_DC_ACC6Q_MSB_BIT_START   0U
#define DC_ACC6Q_MSB_DC_ACC6Q_MSB_BIT_END     3U
/** Definition for field NU1 in Register DC_ACC6Q_MSB**/
#define DC_ACC6Q_MSB_NU1_BIT_START   4U
#define DC_ACC6Q_MSB_NU1_BIT_END     31U
/** Definition for field INTF_FRAME_ZEROCOUNT in Register INTF_FRAME_ZEROCOUNT**/
#define INTF_FRAME_ZEROCOUNT_INTF_FRAME_ZEROCOUNT_BIT_START   0U
#define INTF_FRAME_ZEROCOUNT_INTF_FRAME_ZEROCOUNT_BIT_END     19U
/** Definition for field NU1 in Register INTF_FRAME_ZEROCOUNT**/
#define INTF_FRAME_ZEROCOUNT_NU1_BIT_START   20U
#define INTF_FRAME_ZEROCOUNT_NU1_BIT_END     31U
/** Definition for field INTF_CHIRP_ZEROCOUNT in Register INTF_CHIRP_ZEROCOUNT**/
#define INTF_CHIRP_ZEROCOUNT_INTF_CHIRP_ZEROCOUNT_BIT_START   0U
#define INTF_CHIRP_ZEROCOUNT_INTF_CHIRP_ZEROCOUNT_BIT_END     11U
/** Definition for field NU1 in Register INTF_CHIRP_ZEROCOUNT**/
#define INTF_CHIRP_ZEROCOUNT_NU1_BIT_START   12U
#define INTF_CHIRP_ZEROCOUNT_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC1_CLIP in Register INTF_MAGACC1_CLIP**/
#define INTF_MAGACC1_CLIP_INTF_MAGACC1_CLIP_BIT_START   0U
#define INTF_MAGACC1_CLIP_INTF_MAGACC1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC1_CLIP**/
#define INTF_MAGACC1_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC1_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC2_CLIP in Register INTF_MAGACC2_CLIP**/
#define INTF_MAGACC2_CLIP_INTF_MAGACC2_CLIP_BIT_START   0U
#define INTF_MAGACC2_CLIP_INTF_MAGACC2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC2_CLIP**/
#define INTF_MAGACC2_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC2_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC3_CLIP in Register INTF_MAGACC3_CLIP**/
#define INTF_MAGACC3_CLIP_INTF_MAGACC3_CLIP_BIT_START   0U
#define INTF_MAGACC3_CLIP_INTF_MAGACC3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC3_CLIP**/
#define INTF_MAGACC3_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC3_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC4_CLIP in Register INTF_MAGACC4_CLIP**/
#define INTF_MAGACC4_CLIP_INTF_MAGACC4_CLIP_BIT_START   0U
#define INTF_MAGACC4_CLIP_INTF_MAGACC4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC4_CLIP**/
#define INTF_MAGACC4_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC4_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC5_CLIP in Register INTF_MAGACC5_CLIP**/
#define INTF_MAGACC5_CLIP_INTF_MAGACC5_CLIP_BIT_START   0U
#define INTF_MAGACC5_CLIP_INTF_MAGACC5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC5_CLIP**/
#define INTF_MAGACC5_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC5_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGACC6_CLIP in Register INTF_MAGACC6_CLIP**/
#define INTF_MAGACC6_CLIP_INTF_MAGACC6_CLIP_BIT_START   0U
#define INTF_MAGACC6_CLIP_INTF_MAGACC6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGACC6_CLIP**/
#define INTF_MAGACC6_CLIP_NU1_BIT_START   1U
#define INTF_MAGACC6_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC1_CLIP in Register INTF_MAGDIFFACC1_CLIP**/
#define INTF_MAGDIFFACC1_CLIP_INTF_MAGDIFFACC1_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC1_CLIP_INTF_MAGDIFFACC1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC1_CLIP**/
#define INTF_MAGDIFFACC1_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC1_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC2_CLIP in Register INTF_MAGDIFFACC2_CLIP**/
#define INTF_MAGDIFFACC2_CLIP_INTF_MAGDIFFACC2_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC2_CLIP_INTF_MAGDIFFACC2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC2_CLIP**/
#define INTF_MAGDIFFACC2_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC2_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC3_CLIP in Register INTF_MAGDIFFACC3_CLIP**/
#define INTF_MAGDIFFACC3_CLIP_INTF_MAGDIFFACC3_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC3_CLIP_INTF_MAGDIFFACC3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC3_CLIP**/
#define INTF_MAGDIFFACC3_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC3_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC4_CLIP in Register INTF_MAGDIFFACC4_CLIP**/
#define INTF_MAGDIFFACC4_CLIP_INTF_MAGDIFFACC4_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC4_CLIP_INTF_MAGDIFFACC4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC4_CLIP**/
#define INTF_MAGDIFFACC4_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC4_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC5_CLIP in Register INTF_MAGDIFFACC5_CLIP**/
#define INTF_MAGDIFFACC5_CLIP_INTF_MAGDIFFACC5_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC5_CLIP_INTF_MAGDIFFACC5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC5_CLIP**/
#define INTF_MAGDIFFACC5_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC5_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFACC6_CLIP in Register INTF_MAGDIFFACC6_CLIP**/
#define INTF_MAGDIFFACC6_CLIP_INTF_MAGDIFFACC6_CLIP_BIT_START   0U
#define INTF_MAGDIFFACC6_CLIP_INTF_MAGDIFFACC6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFACC6_CLIP**/
#define INTF_MAGDIFFACC6_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFACC6_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH1 in Register INTF_MAGTHRESH1 **/
#define INTF_MAGTHRESH1_INTF_MAGTHRESH1_BIT_START   0U
#define INTF_MAGTHRESH1_INTF_MAGTHRESH1_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH1 **/
#define INTF_MAGTHRESH1_NU1_BIT_START   24U
#define INTF_MAGTHRESH1_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH2 in Register INTF_MAGTHRESH2**/
#define INTF_MAGTHRESH2_INTF_MAGTHRESH2_BIT_START   0U
#define INTF_MAGTHRESH2_INTF_MAGTHRESH2_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH2**/
#define INTF_MAGTHRESH2_NU1_BIT_START   24U
#define INTF_MAGTHRESH2_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH3 in Register INTF_MAGTHRESH3**/
#define INTF_MAGTHRESH3_INTF_MAGTHRESH3_BIT_START   0U
#define INTF_MAGTHRESH3_INTF_MAGTHRESH3_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH3**/
#define INTF_MAGTHRESH3_NU1_BIT_START   24U
#define INTF_MAGTHRESH3_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH4 in Register INTF_MAGTHRESH4 **/
#define INTF_MAGTHRESH4_INTF_MAGTHRESH4_BIT_START   0U
#define INTF_MAGTHRESH4_INTF_MAGTHRESH4_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH4 **/
#define INTF_MAGTHRESH4_NU1_BIT_START   24U
#define INTF_MAGTHRESH4_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH5 in Register INTF_MAGTHRESH5 **/
#define INTF_MAGTHRESH5_INTF_MAGTHRESH5_BIT_START   0U
#define INTF_MAGTHRESH5_INTF_MAGTHRESH5_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH5 **/
#define INTF_MAGTHRESH5_NU1_BIT_START   24U
#define INTF_MAGTHRESH5_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH6 in Register INTF_MAGTHRESH6 **/
#define INTF_MAGTHRESH6_INTF_MAGTHRESH6_BIT_START   0U
#define INTF_MAGTHRESH6_INTF_MAGTHRESH6_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGTHRESH6 **/
#define INTF_MAGTHRESH6_NU1_BIT_START   24U
#define INTF_MAGTHRESH6_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH1 in Register INTF_MAGDIFFTHRESH1 **/
#define INTF_MAGDIFFTHRESH1_INTF_MAGDIFFTHRESH1_BIT_START   0U
#define INTF_MAGDIFFTHRESH1_INTF_MAGDIFFTHRESH1_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH1 **/
#define INTF_MAGDIFFTHRESH1_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH1_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH2 in Register INTF_MAGDIFFTHRESH2 **/
#define INTF_MAGDIFFTHRESH2_INTF_MAGDIFFTHRESH2_BIT_START   0U
#define INTF_MAGDIFFTHRESH2_INTF_MAGDIFFTHRESH2_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH2 **/
#define INTF_MAGDIFFTHRESH2_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH2_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH3 in Register INTF_MAGDIFFTHRESH3  **/
#define INTF_MAGDIFFTHRESH3_INTF_MAGDIFFTHRESH3_BIT_START   0U
#define INTF_MAGDIFFTHRESH3_INTF_MAGDIFFTHRESH3_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH3  **/
#define INTF_MAGDIFFTHRESH3_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH3_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH4 in Register INTF_MAGDIFFTHRESH4 **/
#define INTF_MAGDIFFTHRESH4_INTF_MAGDIFFTHRESH4_BIT_START   0U
#define INTF_MAGDIFFTHRESH4_INTF_MAGDIFFTHRESH4_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH4 **/
#define INTF_MAGDIFFTHRESH4_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH4_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH5 in Register INTF_MAGDIFFTHRESH5 **/
#define INTF_MAGDIFFTHRESH5_INTF_MAGDIFFTHRESH5_BIT_START   0U
#define INTF_MAGDIFFTHRESH5_INTF_MAGDIFFTHRESH5_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH5 **/
#define INTF_MAGDIFFTHRESH5_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH5_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH6 in Register INTF_MAGDIFFTHRESH6 **/
#define INTF_MAGDIFFTHRESH6_INTF_MAGDIFFTHRESH6_BIT_START   0U
#define INTF_MAGDIFFTHRESH6_INTF_MAGDIFFTHRESH6_BIT_END     23U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH6 **/
#define INTF_MAGDIFFTHRESH6_NU1_BIT_START   24U
#define INTF_MAGDIFFTHRESH6_NU1_BIT_END     31U
/** Definition for field INTF_SUMMAGTHRESH in Register INTF_SUMMAGTHRESH**/
#define INTF_SUMMAGTHRESH_INTF_SUMMAGTHRESH_BIT_START   0U
#define INTF_SUMMAGTHRESH_INTF_SUMMAGTHRESH_BIT_END     23U
/** Definition for field NU1 in Register INTF_SUMMAGTHRESH**/
#define INTF_SUMMAGTHRESH_NU1_BIT_START   24U
#define INTF_SUMMAGTHRESH_NU1_BIT_END     31U
/** Definition for field INTF_SUMMAGDIFFTHRESH in Register INTF_SUMMAGDIFFTHRESH**/
#define INTF_SUMMAGDIFFTHRESH_INTF_SUMMAGDIFFTHRESH_BIT_START   0U
#define INTF_SUMMAGDIFFTHRESH_INTF_SUMMAGDIFFTHRESH_BIT_END     23U
/** Definition for field NU1 in Register INTF_SUMMAGDIFFTHRESH**/
#define INTF_SUMMAGDIFFTHRESH_NU1_BIT_START   24U
#define INTF_SUMMAGDIFFTHRESH_NU1_BIT_END     31U
/** Definition for field INTF_SUMMAGTHRESH_CLIP in Register INTF_SUMMAGTHRESH_CLIP**/
#define INTF_SUMMAGTHRESH_CLIP_INTF_SUMMAGTHRESH_CLIP_BIT_START   0U
#define INTF_SUMMAGTHRESH_CLIP_INTF_SUMMAGTHRESH_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_SUMMAGTHRESH_CLIP**/
#define INTF_SUMMAGTHRESH_CLIP_NU1_BIT_START   1U
#define INTF_SUMMAGTHRESH_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_SUMMAGDIFFTHRESH_CLIP in Register INTF_SUMMAGDIFFTHRESH_CLIP**/
#define INTF_SUMMAGDIFFTHRESH_CLIP_INTF_SUMMAGDIFFTHRESH_CLIP_BIT_START   0U
#define INTF_SUMMAGDIFFTHRESH_CLIP_INTF_SUMMAGDIFFTHRESH_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_SUMMAGDIFFTHRESH_CLIP**/
#define INTF_SUMMAGDIFFTHRESH_CLIP_NU1_BIT_START   1U
#define INTF_SUMMAGDIFFTHRESH_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_STATS_RESET_SW in Register INTF_STATS_RESET_SW**/
#define INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_BIT_START   0U
#define INTF_STATS_RESET_SW_INTF_STATS_RESET_SW_BIT_END     0U
/** Definition for field NU1 in Register INTF_STATS_RESET_SW**/
#define INTF_STATS_RESET_SW_NU1_BIT_START   1U
#define INTF_STATS_RESET_SW_NU1_BIT_END     31U
/** Definition for field DCEST_RESET_SW in Register DCEST_RESET_SW**/
#define DCEST_RESET_SW_DCEST_RESET_SW_BIT_START   0U
#define DCEST_RESET_SW_DCEST_RESET_SW_BIT_END     0U
/** Definition for field NU1 in Register DCEST_RESET_SW**/
#define DCEST_RESET_SW_NU1_BIT_START   1U
#define DCEST_RESET_SW_NU1_BIT_END     31U
/** Definition for field IP_FORMATTER_CLIP_STATUS in Register IP_OP_FORMATTER_CLIP_STATUS**/
#define IP_OP_FORMATTER_CLIP_STATUS_IP_FORMATTER_CLIP_STATUS_BIT_START   0U
#define IP_OP_FORMATTER_CLIP_STATUS_IP_FORMATTER_CLIP_STATUS_BIT_END     0U
/** Definition for field NU1 in Register IP_OP_FORMATTER_CLIP_STATUS**/
#define IP_OP_FORMATTER_CLIP_STATUS_NU1_BIT_START   1U
#define IP_OP_FORMATTER_CLIP_STATUS_NU1_BIT_END     15U
/** Definition for field OP_FORMATTER_CLIP_STATUS in Register IP_OP_FORMATTER_CLIP_STATUS**/
#define IP_OP_FORMATTER_CLIP_STATUS_OP_FORMATTER_CLIP_STATUS_BIT_START   16U
#define IP_OP_FORMATTER_CLIP_STATUS_OP_FORMATTER_CLIP_STATUS_BIT_END     16U
/** Definition for field NU2 in Register IP_OP_FORMATTER_CLIP_STATUS**/
#define IP_OP_FORMATTER_CLIP_STATUS_NU2_BIT_START   17U
#define IP_OP_FORMATTER_CLIP_STATUS_NU2_BIT_END     31U
/** Definition for field INTF_MAGTHRESH1_CLIP in Register INTF_MAGTHRESH1_CLIP**/
#define INTF_MAGTHRESH1_CLIP_INTF_MAGTHRESH1_CLIP_BIT_START   0U
#define INTF_MAGTHRESH1_CLIP_INTF_MAGTHRESH1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH1_CLIP**/
#define INTF_MAGTHRESH1_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH1_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH2_CLIP in Register INTF_MAGTHRESH2_CLIP**/
#define INTF_MAGTHRESH2_CLIP_INTF_MAGTHRESH2_CLIP_BIT_START   0U
#define INTF_MAGTHRESH2_CLIP_INTF_MAGTHRESH2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH2_CLIP**/
#define INTF_MAGTHRESH2_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH2_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH3_CLIP in Register INTF_MAGTHRESH3_CLIP**/
#define INTF_MAGTHRESH3_CLIP_INTF_MAGTHRESH3_CLIP_BIT_START   0U
#define INTF_MAGTHRESH3_CLIP_INTF_MAGTHRESH3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH3_CLIP**/
#define INTF_MAGTHRESH3_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH3_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH4_CLIP in Register INTF_MAGTHRESH4_CLIP**/
#define INTF_MAGTHRESH4_CLIP_INTF_MAGTHRESH4_CLIP_BIT_START   0U
#define INTF_MAGTHRESH4_CLIP_INTF_MAGTHRESH4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH4_CLIP**/
#define INTF_MAGTHRESH4_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH4_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH5_CLIP in Register INTF_MAGTHRESH5_CLIP**/
#define INTF_MAGTHRESH5_CLIP_INTF_MAGTHRESH5_CLIP_BIT_START   0U
#define INTF_MAGTHRESH5_CLIP_INTF_MAGTHRESH5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH5_CLIP**/
#define INTF_MAGTHRESH5_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH5_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGTHRESH6_CLIP in Register INTF_MAGTHRESH6_CLIP**/
#define INTF_MAGTHRESH6_CLIP_INTF_MAGTHRESH6_CLIP_BIT_START   0U
#define INTF_MAGTHRESH6_CLIP_INTF_MAGTHRESH6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGTHRESH6_CLIP**/
#define INTF_MAGTHRESH6_CLIP_NU1_BIT_START   1U
#define INTF_MAGTHRESH6_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH1_CLIP in Register INTF_MAGDIFFTHRESH1_CLIP**/
#define INTF_MAGDIFFTHRESH1_CLIP_INTF_MAGDIFFTHRESH1_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH1_CLIP_INTF_MAGDIFFTHRESH1_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH1_CLIP**/
#define INTF_MAGDIFFTHRESH1_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH1_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH2_CLIP in Register INTF_MAGDIFFTHRESH2_CLIP**/
#define INTF_MAGDIFFTHRESH2_CLIP_INTF_MAGDIFFTHRESH2_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH2_CLIP_INTF_MAGDIFFTHRESH2_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH2_CLIP**/
#define INTF_MAGDIFFTHRESH2_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH2_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH3_CLIP in Register INTF_MAGDIFFTHRESH3_CLIP**/
#define INTF_MAGDIFFTHRESH3_CLIP_INTF_MAGDIFFTHRESH3_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH3_CLIP_INTF_MAGDIFFTHRESH3_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH3_CLIP**/
#define INTF_MAGDIFFTHRESH3_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH3_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH4_CLIP in Register INTF_MAGDIFFTHRESH4_CLIP**/
#define INTF_MAGDIFFTHRESH4_CLIP_INTF_MAGDIFFTHRESH4_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH4_CLIP_INTF_MAGDIFFTHRESH4_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH4_CLIP**/
#define INTF_MAGDIFFTHRESH4_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH4_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH5_CLIP in Register INTF_MAGDIFFTHRESH5_CLIP**/
#define INTF_MAGDIFFTHRESH5_CLIP_INTF_MAGDIFFTHRESH5_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH5_CLIP_INTF_MAGDIFFTHRESH5_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH5_CLIP**/
#define INTF_MAGDIFFTHRESH5_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH5_CLIP_NU1_BIT_END     31U
/** Definition for field INTF_MAGDIFFTHRESH6_CLIP in Register INTF_MAGDIFFTHRESH6_CLIP**/
#define INTF_MAGDIFFTHRESH6_CLIP_INTF_MAGDIFFTHRESH6_CLIP_BIT_START   0U
#define INTF_MAGDIFFTHRESH6_CLIP_INTF_MAGDIFFTHRESH6_CLIP_BIT_END     0U
/** Definition for field NU1 in Register INTF_MAGDIFFTHRESH6_CLIP**/
#define INTF_MAGDIFFTHRESH6_CLIP_NU1_BIT_START   1U
#define INTF_MAGDIFFTHRESH6_CLIP_NU1_BIT_END     31U
/** Definition for field HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_BIT_START   0U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_BIT_END     0U
/** Definition for field HWA_SAFETY_PARITY_ERR_MASK_WINDOW_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_WINDOW_RAM_BIT_START   1U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_WINDOW_RAM_BIT_END     1U
/** Definition for field HWA_SAFETY_PARITY_ERR_MASK_IPING_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_IPING_RAM_BIT_START   2U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_IPING_RAM_BIT_END     2U
/** Definition for field HWA_SAFETY_PARITY_ERR_MASK_IPONG_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_IPONG_RAM_BIT_START   3U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_IPONG_RAM_BIT_END     3U
/** Definition for field HWA_SAFETY_PARITY_ERR_MASK_OPING_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_OPING_RAM_BIT_START   4U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_OPING_RAM_BIT_END     4U
/** Definition for field HWA_SAFETY_PARITY_ERR_MASK_OPONG_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_OPONG_RAM_BIT_START   5U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_PARITY_ERR_MASK_OPONG_RAM_BIT_END     5U
/** Definition for field HWA_SAFETY_ACCESS_ERR_MASK_IPING_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_IPING_RAM_BIT_START   6U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_IPING_RAM_BIT_END     6U
/** Definition for field HWA_SAFETY_ACCESS_ERR_MASK_IPONG_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_IPONG_RAM_BIT_START   7U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_IPONG_RAM_BIT_END     7U
/** Definition for field HWA_SAFETY_ACCESS_ERR_MASK_OPING_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_OPING_RAM_BIT_START   8U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_OPING_RAM_BIT_END     8U
/** Definition for field HWA_SAFETY_ACCESS_ERR_MASK_OPONG_RAM in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_OPONG_RAM_BIT_START   9U
#define HWA_SAFETY_ERR_MASK_HWA_SAFETY_ACCESS_ERR_MASK_OPONG_RAM_BIT_END     9U
/** Definition for field NU1 in Register HWA_SAFETY_ERR_MASK**/
#define HWA_SAFETY_ERR_MASK_NU1_BIT_START   10U
#define HWA_SAFETY_ERR_MASK_NU1_BIT_END     31U
/** Definition for field HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_BIT_START   0U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP_BIT_END     0U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_WINDOW_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_WINDOW_RAM_BIT_START   1U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_WINDOW_RAM_BIT_END     1U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_IPING_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_IPING_RAM_BIT_START   2U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_IPING_RAM_BIT_END     2U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_IPONG_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_IPONG_RAM_BIT_START   3U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_IPONG_RAM_BIT_END     3U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_OPING_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_OPING_RAM_BIT_START   4U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_OPING_RAM_BIT_END     4U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_OPONG_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_OPONG_RAM_BIT_START   5U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_PARITY_ERR_STATUS_OPONG_RAM_BIT_END     5U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_IPING_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_IPING_RAM_BIT_START   6U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_IPING_RAM_BIT_END     6U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_IPONG_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_IPONG_RAM_BIT_START   7U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_IPONG_RAM_BIT_END     7U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_OPING_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_OPING_RAM_BIT_START   8U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_OPING_RAM_BIT_END     8U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_OPONG_RAM in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_OPONG_RAM_BIT_START   9U
#define HWA_SAFETY_ERR_STATUS_HWA_SAFETY_ACCESS_ERR_STATUS_OPONG_RAM_BIT_END     9U
/** Definition for field NU1 in Register HWA_SAFETY_ERR_STATUS**/
#define HWA_SAFETY_ERR_STATUS_NU1_BIT_START   10U
#define HWA_SAFETY_ERR_STATUS_NU1_BIT_END     31U
/** Definition for field HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP_BIT_START   0U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ERR_STATUS_RAW_FSM_LOCKSTEP_BIT_END     0U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_RAW_WINDOW_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_WINDOW_RAM_BIT_START   1U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_WINDOW_RAM_BIT_END     1U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPING_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPING_RAM_BIT_START   2U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPING_RAM_BIT_END     2U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPONG_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPONG_RAM_BIT_START   3U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_IPONG_RAM_BIT_END     3U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPING_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPING_RAM_BIT_START   4U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPING_RAM_BIT_END     4U
/** Definition for field HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPONG_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPONG_RAM_BIT_START   5U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_PARITY_ERR_STATUS_RAW_OPONG_RAM_BIT_END     5U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPING_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPING_RAM_BIT_START   6U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPING_RAM_BIT_END     6U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPONG_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPONG_RAM_BIT_START   7U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_IPONG_RAM_BIT_END     7U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPING_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPING_RAM_BIT_START   8U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPING_RAM_BIT_END     8U
/** Definition for field HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPONG_RAM in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPONG_RAM_BIT_START   9U
#define HWA_SAFETY_ERR_STATUS_RAW_HWA_SAFETY_ACCESS_ERR_STATUS_RAW_OPONG_RAM_BIT_END     9U
/** Definition for field NU1 in Register HWA_SAFETY_ERR_STATUS_RAW**/
#define HWA_SAFETY_ERR_STATUS_RAW_NU1_BIT_START   10U
#define HWA_SAFETY_ERR_STATUS_RAW_NU1_BIT_END     31U

/** @struct hwaMaxStats
*   @brief  Hardware accelerator stats Register Definition
*
*   This structure is used to access the Hardware accelerator stats registers within the common space.
*/
/** @typedef hwaMaxStats_t
*   @brief  Hardware accelerator stats Register Definition
*
*   This structure is used to access the Hardware accelerator stats registers within the common space.
*/
typedef volatile struct hwaMaxStats
{
    uint32_t    MAXnVALUE       ;        /* AddrOffset = 0x000 */
    uint32_t    MAXnINDEX       ;        /* AddrOffset = 0x004 */
    uint32_t    ISUMnLSB        ;        /* AddrOffset = 0x008 */
    uint32_t    ISUMnMSB        ;        /* AddrOffset = 0x00C */
    uint32_t    QSUMnLSB        ;        /* AddrOffset = 0x010 */
    uint32_t    QSUMnMSB        ;        /* AddrOffset = 0x014 */
} hwaMaxStats_t;

/** @struct hwaAccVal
*   @brief  HWA accumulator register data structure
*
*/
typedef volatile struct hwaAccVal_t
{
    uint32_t    accValLSB;        /* AddrOffset = 0x000 */
    uint32_t    accValMSB;        /* AddrOffset = 0x004 */
} hwaAccVal;

/**
 * @struct DSSHWACCRegs_t
 * @brief
 *   Module DSS_HW_ACC Register Definition
 * @details
 *   This structure is used to access the DSS_HW_ACC module registers.
 */
/**
 * @typedef DSSHWACCRegs
 * @brief
 *   Module DSS_HW_ACC Register Frame type Definition
 * @details
 *   This type is used to access the DSS_HW_ACC module registers.
 */
typedef volatile struct DSSHWACCRegs_t
{
    uint32_t  HWACCREG1   ;  /** Offset = 0x0 */
    uint32_t  HWACCREG2   ;  /** Offset = 0x4 */
    uint32_t  HWACCREG3   ;  /** Offset = 0x8 */
    uint32_t  HWACCREG4   ;  /** Offset = 0xc */
    uint32_t  HWACCREG5   ;  /** Offset = 0X10 */
    uint32_t  HWACCREG6   ;  /** Offset = 0x14 */
    uint32_t  HWACCREG7   ;  /** Offset = 0x18 */
    uint32_t  HWACCREG8   ;  /** Offset = 0x1C */
    uint32_t  HWACCREG11   ;  /** Offset = 0x20  */
    uint32_t  HWACCREG12   ;  /** Offset = 0x24  */
    uint32_t  HWACCREG13   ;  /** Offset = 0x28 */
    uint32_t  HWACCREG14   ;  /** Offset = 0x2C */
    uint32_t  HWACCREG15   ;  /** Offset = 0x30  */
    uint32_t  CFAR_DET_THR ;  /** Offset = 0x34  */

#if 0
    uint32_t  MAX1VALUE   ;  /** Offset = 0x38  */
    uint32_t  MAX1INDEX   ;  /** Offset = 0x3C  */
    uint32_t  ISUM1LSB   ;  /** Offset = 0x40  */
    uint32_t  ISUM1MSB   ;  /** Offset = 0x44 */
    uint32_t  QSUM1LSB   ;  /** Offset = 0x48 */
    uint32_t  QSUM1MSB   ;  /** Offset = 0x4C  */
    uint32_t  MAX2VALUE   ;  /** Offset = 0x50  */
    uint32_t  MAX2INDEX   ;  /** Offset = 0x54  */
    uint32_t  ISUM2LSB   ;  /** Offset = 0x58  */
    uint32_t  ISUM2MSB   ;  /** Offset = 0x5C  */
    uint32_t  QSUM2LSB   ;  /** Offset = 0x60  */
    uint32_t  QSUM2MSB   ;  /** Offset = 0x64  */
    uint32_t  MAX3VALUE   ;  /** Offset = 0x68  */
    uint32_t  MAX3INDEX   ;  /** Offset = 0x6C  */
    uint32_t  ISUM3LSB   ;  /** Offset = 0x70  */
    uint32_t  ISUM3MSB   ;  /** Offset = 0x74  */
    uint32_t  QSUM3LSB   ;  /** Offset = 0x78  */
    uint32_t  QSUM3MSB   ;  /** Offset = 0x7C */
    uint32_t  MAX4VALUE   ;  /** Offset = 0x80 */
    uint32_t  MAX4INDEX   ;  /** Offset = 0x84 */
    uint32_t  ISUM4LSB   ;  /** Offset = 0x88  */
    uint32_t  ISUM4MSB   ;  /** Offset = 0x8C */
    uint32_t  QSUM4LSB   ;  /** Offset = 0x90  */
    uint32_t  QSUM4MSB   ;  /** Offset = 0x94 */
#else
    hwaMaxStats_t HWACCSTATSREG[4]; /** Offset = 0x38 - 0x94 */
#endif

    uint32_t  Reserved2   ;  /** Offset = 0x98 */
    uint32_t  RDSTATUS   ;  /** Offset = 0x9C */

#if 0
    uint32_t  SIGDMACH1DONE   ;  /** Offset = 0xA0 */
    uint32_t  SIGDMACH2DONE   ;  /** Offset = 0xA4 */
    uint32_t  SIGDMACH3DONE   ;  /** Offset = 0xA8 */
    uint32_t  SIGDMACH4DONE   ;  /** Offset = 0xAC */
    uint32_t  SIGDMACH5DONE   ;  /** Offset = 0xB0 */
    uint32_t  SIGDMACH6DONE   ;  /** Offset = 0xB4 */
    uint32_t  SIGDMACH7DONE   ;  /** Offset = 0xB8 */
    uint32_t  SIGDMACH8DONE   ;  /** Offset = 0xBC */
    uint32_t  SIGDMACH9DONE   ;  /** Offset = 0xC0 */
    uint32_t  SIGDMACH10DONE   ;  /** Offset = 0xC4 */
    uint32_t  SIGDMACH11DONE   ;  /** Offset = 0xC8 */
    uint32_t  SIGDMACH12DONE   ;  /** Offset = 0xCC */
    uint32_t  SIGDMACH13DONE   ;  /** Offset = 0xD0 */
    uint32_t  SIGDMACH14DONE   ;  /** Offset = 0xD4 */
    uint32_t  SIGDMACH15DONE   ;  /** Offset = 0xD8 */
    uint32_t  SIGDMACH16DONE   ;  /** Offset = 0xDC */
#else
    uint32_t    SIGDMACHxDONE[16];  /** Offset = 0xA0 - 0xDC */
#endif    

    uint32_t  Reserved3   ;  /** Offset = 0xE0 */
    uint32_t  FFTCLIP   ;  /** Offset = 0xE4 */
    uint32_t  FFTPEAKCNT   ;  /** Offset = 0xE8 */
    uint32_t  Reserved4[3]   ;  /** Offset = 0xEC - 0xF4 */
    uint32_t  CMP_EGE_K0123   ;  /** Offset = 0xF8 */
    uint32_t  CMP_EGE_K4567   ;  /** Offset = 0xFC */
    uint32_t  HWA_SAFETY_ENABLE   ;  /** Offset = 0x100 */
    uint32_t  MEMINIT   ;  /** Offset = 0x104 */
    uint32_t  MEMINITDONE   ;  /** Offset = 0x108 */
    uint32_t  HWA_SAFETY_WIN_RAM_ERR_LOC   ;  /** Offset = 0x10C */
    uint32_t  HWA_SAFETY_PARAM_RAM_ERR_LOC   ;  /** Offset = 0x110 */
    uint32_t  HWA_SAFETY_IPING_ERR_LOC   ;  /** Offset = 0x114 */
    uint32_t  HWA_SAFETY_IPONG_ERR_LOC   ;  /** Offset = 0x118 */
    uint32_t  HWA_SAFETY_OPING_ERR_LOC   ;  /** Offset = 0x11C */
    uint32_t  HWA_SAFETY_OPONG_ERR_LOC   ;  /** Offset = 0x120 */
    uint32_t  Reserved5[2]   ;  /** Offset = 0x124 - 0x128 */
    uint32_t  HWACCREG16   ;  /** Offset = 0x12C */
    uint32_t  DCEST_I_SW[6]  ;  /** Offset = 0x130 - 0x144 */
    uint32_t  DCEST_I_VAL[6] ;  /** Offset = 0x148 - 0x15C */
    hwaAccVal DC_ACC_I_VAL[6];  /* offset = 0x0160 - 0x018C */
    uint32_t  DCEST_Q_SW[6]  ;  /** Offset = 0x190 - 0x1A4 */
    uint32_t  DCEST_Q_VAL[6] ;  /** Offset = 0x1A8 - 0x1BC */
    hwaAccVal DC_ACC_Q_VAL[6];  /* offset = 0x01C0 - 0x01EC */
    uint32_t  DCACC_CLIP[6]  ;  /** Offset = 0x1F0 - 0x204 */
    uint32_t  DCEST_CLIP[6]  ;  /** Offset = 0x208 - 0x21C*/
    uint32_t  DCSUB_CLIP[6]  ;  /** Offset = 0x220 - 0x234*/
    uint32_t  DCEST_SHIFT   ;  /** Offset = 0x238 */
    uint32_t  DCEST_SCALE   ;  /** Offset = 0x23C */
    uint32_t  INTF_MAG_SCALE   ;  /** Offset = 0x240 */
    uint32_t  INTF_MAG_SHIFT   ;  /** Offset = 0x244 */
    uint32_t  INTF_MAGDIFF_SCALE   ;  /** Offset = 0x248 */
    uint32_t  INTF_MAGDIFF_SHIFT   ;  /** Offset = 0x24C */
    uint32_t  INTF_FRAME_ZEROCOUNT   ;  /** Offset = 0x250 */
    uint32_t  INTF_CHIRP_ZEROCOUNT   ;  /** Offset = 0x254 */
    uint32_t  INTF_MAGTHRESH1_SW   ;  /** Offset = 0x258 */
    uint32_t  INTF_MAGTHRESH2_SW   ;  /** Offset = 0x25C */
    uint32_t  INTF_MAGTHRESH3_SW   ;  /** Offset = 0x260 */
    uint32_t  INTF_MAGTHRESH4_SW   ;  /** Offset = 0x264 */
    uint32_t  INTF_MAGTHRESH5_SW   ;  /** Offset = 0x268 */
    uint32_t  INTF_MAGTHRESH6_SW   ;  /** Offset = 0x26C */
    uint32_t  INTF_MAGDIFFTHRESH1_SW   ;  /** Offset = 0x270 */
    uint32_t  INTF_MAGDIFFTHRESH2_SW   ;  /** Offset = 0x274 */
    uint32_t  INTF_MAGDIFFTHRESH3_SW   ;  /** Offset = 0x278 */
    uint32_t  INTF_MAGDIFFTHRESH4_SW   ;  /** Offset = 0x27C */
    uint32_t  INTF_MAGDIFFTHRESH5_SW   ;  /** Offset = 0x280 */
    uint32_t  INTF_MAGDIFFTHRESH6_SW   ;  /** Offset = 0x284 */
    uint32_t  INTF_MAGACC1_LSB   ;  /** Offset = 0x288 */
    uint32_t  INTF_MAGACC1_MSB   ;  /** Offset = 0x28C */
    uint32_t  INTF_MAGACC2_LSB   ;  /** Offset = 0x290 */
    uint32_t  INTF_MAGACC2_MSB   ;  /** Offset = 0x294 */
    uint32_t  INTF_MAGACC3_LSB   ;  /** Offset = 0x298 */
    uint32_t  INTF_MAGACC3_MSB   ;  /** Offset = 0x29C */
    uint32_t  INTF_MAGACC4_LSB   ;  /** Offset = 0x2A0 */
    uint32_t  INTF_MAGACC4_MSB   ;  /** Offset = 0x2A4 */
    uint32_t  INTF_MAGACC5_LSB   ;  /** Offset = 0x2A8 */
    uint32_t  INTF_MAGACC5_MSB   ;  /** Offset = 0x2AC */
    uint32_t  INTF_MAGACC6_LSB   ;  /** Offset = 0x2B0 */
    uint32_t  INTF_MAGACC6_MSB   ;  /** Offset = 0x2B4 */
    uint32_t  INTF_MAGDIFFACC1_LSB   ;  /** Offset = 0x2B8 */
    uint32_t  INTF_MAGDIFFACC1_MSB   ;  /** Offset = 0x2BC */
    uint32_t  INTF_MAGDIFFACC2_LSB   ;  /** Offset = 0x2C0 */
    uint32_t  INTF_MAGDIFFACC2_MSB   ;  /** Offset = 0x2C4 */
    uint32_t  INTF_MAGDIFFACC3_LSB   ;  /** Offset = 0x2C8 */
    uint32_t  INTF_MAGDIFFACC3_MSB   ;  /** Offset = 0x2CC */
    uint32_t  INTF_MAGDIFFACC4_LSB   ;  /** Offset = 0x2D0 */
    uint32_t  INTF_MAGDIFFACC4_MSB   ;  /** Offset = 0x2D4 */
    uint32_t  INTF_MAGDIFFACC5_LSB   ;  /** Offset = 0x2D8 */
    uint32_t  INTF_MAGDIFFACC5_MSB   ;  /** Offset = 0x2DC */
    uint32_t  INTF_MAGDIFFACC6_LSB   ;  /** Offset = 0x2E0 */
    uint32_t  INTF_MAGDIFFACC6_MSB   ;  /** Offset = 0x2E4 */
    uint32_t  INTF_MAGACC1_CLIP   ;  /** Offset = 0x2E8 */
    uint32_t  INTF_MAGACC2_CLIP   ;  /** Offset = 0x2EC */
    uint32_t  INTF_MAGACC3_CLIP   ;  /** Offset = 0x2F0 */
    uint32_t  INTF_MAGACC4_CLIP   ;  /** Offset = 0x2F4 */
    uint32_t  INTF_MAGACC5_CLIP   ;  /** Offset = 0x2F8 */
    uint32_t  INTF_MAGACC6_CLIP   ;  /** Offset = 0x2FC */
    uint32_t  INTF_MAGDIFFACC1_CLIP   ;  /** Offset = 0x300 */
    uint32_t  INTF_MAGDIFFACC2_CLIP   ;  /** Offset = 0x304 */
    uint32_t  INTF_MAGDIFFACC3_CLIP   ;  /** Offset = 0x308 */
    uint32_t  INTF_MAGDIFFACC4_CLIP   ;  /** Offset = 0x30C */
    uint32_t  INTF_MAGDIFFACC5_CLIP   ;  /** Offset = 0x310 */
    uint32_t  INTF_MAGDIFFACC6_CLIP   ;  /** Offset = 0x314 */
    uint32_t  INTF_MAGTHRESH1   ;  /** Offset = 0x318 */
    uint32_t  INTF_MAGTHRESH2   ;  /** Offset = 0x31C */
    uint32_t  INTF_MAGTHRESH3   ;  /** Offset = 0x320 */
    uint32_t  INTF_MAGTHRESH4   ;  /** Offset = 0x324 */
    uint32_t  INTF_MAGTHRESH5   ;  /** Offset = 0x328 */
    uint32_t  INTF_MAGTHRESH6   ;  /** Offset = 0x32C */
    uint32_t  INTF_MAGDIFFTHRESH1   ;  /** Offset = 0x330 */
    uint32_t  INTF_MAGDIFFTHRESH2   ;  /** Offset = 0x334 */
    uint32_t  INTF_MAGDIFFTHRESH3   ;  /** Offset = 0x338 */
    uint32_t  INTF_MAGDIFFTHRESH4   ;  /** Offset = 0x33C */
    uint32_t  INTF_MAGDIFFTHRESH5   ;  /** Offset = 0x340 */
    uint32_t  INTF_MAGDIFFTHRESH6   ;  /** Offset = 0x344 */
    uint32_t  INTF_SUMMAGTHRESH   ;  /** Offset = 0x348 */
    uint32_t  INTF_SUMMAGDIFFTHRESH   ;  /** Offset = 0x34C */
    uint32_t  INTF_SUMMAGTHRESH_CLIP   ;  /** Offset = 0x350 */
    uint32_t  INTF_SUMMAGDIFFTHRESH_CLIP   ;  /** Offset = 0x354 */
    uint32_t  CMULTSCALE1I   ;  /** Offset = 0x358 */
    uint32_t  CMULTSCALE2I   ;  /** Offset = 0x35C */
    uint32_t  CMULTSCALE3I   ;  /** Offset = 0x360 */
    uint32_t  CMULTSCALE4I   ;  /** Offset = 0x364 */
    uint32_t  CMULTSCALE5I   ;  /** Offset = 0x368 */
    uint32_t  CMULTSCALE6I   ;  /** Offset = 0x36C */
    uint32_t  CMULTSCALE1Q   ;  /** Offset = 0x370 */
    uint32_t  CMULTSCALE2Q   ;  /** Offset = 0x374 */
    uint32_t  CMULTSCALE3Q   ;  /** Offset = 0x378 */
    uint32_t  CMULTSCALE4Q   ;  /** Offset = 0x37C */
    uint32_t  CMULTSCALE5Q   ;  /** Offset = 0x380 */
    uint32_t  CMULTSCALE6Q   ;  /** Offset = 0x384 */
    uint32_t  CLR_MISC_CLIP   ;  /** Offset = 0x388 */
    uint32_t  Reserved6   ;  /** Offset = 0x38C */
    uint32_t  INTF_STATS_RESET_SW   ;  /** Offset = 0x390 */
    uint32_t  DCEST_RESET_SW   ;  /** Offset = 0x394 */
    uint32_t  IP_OP_FORMATTER_CLIP_STATUS   ;  /** Offset = 0x398 */
    uint32_t  INTF_MAGTHRESH1_CLIP   ;  /** Offset = 0x39C */
    uint32_t  INTF_MAGTHRESH2_CLIP   ;  /** Offset = 0x3A0 */
    uint32_t  INTF_MAGTHRESH3_CLIP   ;  /** Offset = 0x3A4 */
    uint32_t  INTF_MAGTHRESH4_CLIP   ;  /** Offset = 0x3A8 */
    uint32_t  INTF_MAGTHRESH5_CLIP   ;  /** Offset = 0x3AC */
    uint32_t  INTF_MAGTHRESH6_CLIP   ;  /** Offset = 0x3B0 */
    uint32_t  INTF_MAGDIFFTHRESH1_CLIP   ;  /** Offset = 0x3B4 */
    uint32_t  INTF_MAGDIFFTHRESH2_CLIP   ;  /** Offset = 0x3B8 */
    uint32_t  INTF_MAGDIFFTHRESH3_CLIP   ;  /** Offset = 0x3BC */
    uint32_t  INTF_MAGDIFFTHRESH4_CLIP   ;  /** Offset = 0x3C0 */
    uint32_t  INTF_MAGDIFFTHRESH5_CLIP   ;  /** Offset = 0x3C4 */
    uint32_t  INTF_MAGDIFFTHRESH6_CLIP   ;  /** Offset = 0x3C8 */
    uint32_t  HWA_SAFETY_ERR_MASK   ;  /** Offset = 0x3CC */
    uint32_t  HWA_SAFETY_ERR_STATUS   ;  /** Offset = 0x3D0 */
    uint32_t  HWA_SAFETY_ERR_STATUS_RAW   ;  /** Offset = 0x3D4 */
} DSSHWACCRegs;

#ifdef __cplusplus
}
#endif

#endif /* HW_HWA_COMMONREG_H_ */
