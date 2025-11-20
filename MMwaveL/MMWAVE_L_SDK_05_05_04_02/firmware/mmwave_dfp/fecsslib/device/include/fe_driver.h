/*!*****************************************************************************
 * @file fe_driver.h
 *
 * @brief FECSSLib driver header file.
 *
 * @b Description @n
 * This FECSS library driver header file defines FECSS interface and driver data structure,
 * MACROs and functions.
 *
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
 * @addtogroup FECSSLIB_DEVICE_DRV_MODULE Device Interface and Driver API functions
 *  @{
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Revision History   :
 *------------------------------------------------------------------------------
 * Version Date        Author  Ticket No           Description
 *------------------------------------------------------------------------------
 * 0.1     14Jan2021   TI      NA                  Initial Version
 *******************************************************************************
 */

/*******************************************************************************
 * FILE INCLUSION PROTECTION
 *******************************************************************************
 */
#ifndef FE_DRIVER_H
#define FE_DRIVER_H

/*******************************************************************************
 * INCLUDE FILES
 *******************************************************************************
 */
#ifdef __cplusplus
extern "C" {
#endif

#include <fecsslib/common/reg_topprcm.h>
#include <fecsslib/common/reg_fecrcm.h>
#include <fecsslib/common/reg_fecctrl.h>

/*******************************************************************************
 * MACRO DEFINITIONS
 *******************************************************************************
 */
/*!
 * @brief base address for EFUSE DIEID10 register
 */
#define M_FE_TOP_DIEID0_REG_ADDR                ((REG32 *)0x5A020004)
/*!
 * @name FECSS Library Driver module MACROs
 * @{
 */
/*!
 * @brief FECSS MB response interrupt index
 */
#define M_FE_INTR0_RFS_MB_RSP                     (0U)

/*!
 * @brief Num of interrupts used in FECSS
 */
#define M_FE_NUM_OF_INTERRUPTS                    (1U)

/*!
 * @brief The FECSS TOP PRCM register space start address
 */
#define M_FE_TOP_PRCM_REG_START_ADDRESS             ((T_TOPPRCM_REGS*)0x5A040000U)

/*!
 * @brief The FECSS FECRCM register space start address
 */
#define M_FE_FECSS_RCM_REG_START_ADDRESS            ((T_FECRCM_REGS*)0x52020000U)

/*!
 * @brief The FECSS FECCTRL register space start address
 */
#define M_FE_FECSS_CTRL_REG_START_ADDRESS           ((T_FECCTRL_REGS*)0x52000000U)

/*!
 * @brief The FE driver data address pointer holds the z_FeDriverData start address.
 * In SOC device the PC_REGISTER8 AON register holds FECSS z_FeDriverData start address.
 * In FE device a global variable w_FeDrvDataAdd holds FECSS z_FeDriverData start address.
 * This pointer links  dynamic FECSS driver data structure data RAM address with ROM.
 * This dynamic address will be generated when mmwavelink library is
 * linked with application.
 */
#if ((M_DFP_DEVICE_PLATFORM_TYPE) == (M_DFP_DEVICE_PLATFORM_FE))
extern REG32 w_FeDrvDataAdd;
#define M_FE_DRV_DATA_ADDRESS_PTR          ((REG32*)(&w_FeDrvDataAdd))
#else
#define M_FE_DRV_DATA_ADDRESS_PTR          \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_PcRegister8.b32_Reg))
#endif

/*!
 * @brief The TOPRCM FEC power domain control register address
 */
#define M_FE_TOPRCM_FEC_PD_CTRL_REG_ADDRESS         \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_FecPwrReqParam.b32_Reg))

/*!
 * @brief The TOPRCM FEC PD M3 core reset control register address
 */
#define M_FE_TOPRCM_FEC_PD_CM3_RESET_REG_ADDRESS    \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_FecCoreSysresetParam.b32_Reg))

/*!
 * @brief The TOPRCM FEC PD cluster1 memory retention register address, the
 * The FEC data ram 16kB Bank1 and per chirp ram part of this cfg
 */
#define M_FE_TOPRCM_FEC_PD_CLUST1_RET_REG_ADDRESS    \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_PsconFecPdRamState.b32_Reg))

/*!
 * @brief The TOPRCM FEC PD cluster2 and 3 memory retention register address, the
 * The FEC data ram 16kB Bank2 (patch), GPADC and shared ram 96kB part of this cfg
 */
#define M_FE_TOPRCM_FEC_PD_CLUST23_RET_REG_ADDRESS    \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_PsconFecPdRamGrp4State.b32_Reg))

/*!
 * @brief The TOPRCM FEC PD status register address
 */
#define M_FE_TOPRCM_FEC_PD_STATUS_REG_ADDRESS       \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_PsconFecPdEn.b32_Reg))

/*!
 * @brief The TOPRCM FEC Frame Timer clock gate register address
 */
#define M_FE_TOPRCM_FEC_FT_CLK_GATE_REG_ADDRESS     \
    ((REG32*)&(M_FE_TOP_PRCM_REG_START_ADDRESS->r_FrcOscClkGate.b32_Reg))

/*!
 * @brief The APPSS FEC root clock gate register address
 */
#define M_FE_APPSS_FEC_CLK_GATE_REG_ADDRESS         ((REG32*)0x56060394U)


/*****************************FECSS driver Control config values *********************************/
/*!
 * @brief The TOPRCM FEC PD power up control value in manual mode
 */
#define M_FE_TOPRCM_FEC_PD_CTRL_PWR_UP_VAL          ((UINT32)0x000217FFU)

/*!
 * @brief The TOPRCM FEC PD power down control value in manual mode
 */
#define M_FE_TOPRCM_FEC_PD_CTRL_PWR_DOWN_VAL        ((UINT32)0x000207FFU)

/*!
 * @brief The TOPRCM FEC PD sys reset value in manual mode (except mem logic)
 */
#define M_FE_TOPRCM_FEC_PD_CM3_RESET_VAL            ((UINT32)0x000207FFU)

/*!
 * @brief The TOPRCM FEC PD cluster1 memory retention value in deep sleep (power down)
 *  - Bit 0, Bit 16 - Bank1 (Data memory) + Per chirp RAM (retained)
 */
#define M_FE_TOPRCM_FEC_PD_CLUST1_RAM_STATE_ON_VAL      ((UINT32)0x00010001U)

/*!
 * @brief The TOPRCM FEC PD memory logic OFF value in deep sleep (power down)
 *  - Bit 0, Bit 16 - Bank1 (Data memory) + Per chirp RAM (reset)
 */
#define M_FE_TOPRCM_FEC_PD_CLUST1_RAM_STATE_OFF_VAL     ((UINT32)0x00010000U)

/*!
 * @brief The TOPRCM FEC PD cluster2 memory retention value in deep sleep (power down)
 *  - Bit 0 - Bank2 (Patch code) + GPADC memory (retained)
 *  - Bit 1 - Bank3 Shared ram (retained)
 */
#define M_FE_TOPRCM_FEC_PD_CLUST23_RAM_STATE_ON_VAL     ((UINT32)0x00030003U)

/*!
 * @brief The TOPRCM FEC PD reset and power up Status mask value, in powerup both status
 * has to be indicating powerup. \n
 *  - Bit 9 - value 0 powered up (reset released), value 1 powered down (in reset state)
 *  - Bit 8 - value 1 powered up (ON), value 0 powered down (OFF)
 */
#define M_FE_TOPRCM_FEC_PD_ON_STATUS_MASK           ((UINT32)0x00000300U)

/*!
 * @brief The TOPRCM FEC PD reset and power down Status mask value, in power down, just
 * poll for reset status as power status is controlled by HWASS (shared ram access)
 *  - Bit 9 - value 0 powered up (reset released), value 1 powered down (in reset state)
 */
#define M_FE_TOPRCM_FEC_PD_OFF_STATUS_MASK           ((UINT32)0x00000200U)

/*!
 * @brief The TOPRCM FEC PD & reset Status power up value
 *  - Bit 9 - value 0 powered up (reset released)
 *  - Bit 8 - value 1 powered up (ON)
 */
#define M_FE_TOPRCM_FEC_PD_STATUS_PWR_UP            ((UINT32)0x00000100U)

/*!
 * @brief The TOPRCM FEC PD & reset Status power down value
 *  - Bit 9 - value 1 powered down (in reset state)
 */
#define M_FE_TOPRCM_FEC_PD_STATUS_PWR_DOWN          ((UINT32)0x00000200U)

/*!
 * @brief The TOPRCM FEC FT clock gate value
 */
#define M_FE_TOPRCM_FEC_FT_CLK_GATE_VALUE           ((UINT32)0x7U)

/*!
 * @brief The APPSS FEC SYS clock gate value
 */
#define M_FE_APPSS_FEC_SYS_CLK_GATE_VALUE           ((UINT32)0x7U)

/*!
 * @brief The FECSS FEC RCM space default clock ungate value
 * Ungate DFE and chirp timer
 */
#define M_FE_FECSS_RCM_SPACE_CLK_UNGATE_VALUE       ((UINT32)0x381C0038U)

/*!
 * @brief The FECSS FEC Fast clock (160MHz) source divider value, multibit
 */
#define M_FE_FECSS_CLK_SRC_FAST_DIV_VALUE           ((UINT32)0x111U)

/*!
 * @brief The FECSS FEC XTAL clock (40MHz) source divider value, multibit
 */
#define M_FE_FECSS_CLK_SRC_XTAL_DIV_VALUE           ((UINT32)0x000U)

/*!
 * @brief The FECSS FEC XTAL clock source select value, div = 0
 */
#define M_FE_FECSS_CLK_SRC_XTAL_SEL_VALUE           ((UINT32)0x0000U)

/*!
 * @brief The FECSS FEC Fast clock (160MHz) source select value, div = 1
 */
#define M_FE_FECSS_CLK_SRC_FAST_SEL_VALUE           ((UINT32)0x0333U)

/*!
 * @brief The FECSS M3 core config, sleep hold dis, wic enable
 */
#define M_FE_FECSS_M3_CORE_CFG_VALUE                ((UINT32)0x6U)

/*!
 * @brief The FECSS M3 core unhalt manual mode (remove reset)
 */
#define M_FE_TOPRCM_FEC_PD_CM3_UNHALT_VAL           ((UINT32)0x000217FFU)

/*!
 * @brief FECSS RFS(M3) DATA, and CHIRP RAM mem init trigger / status
 */
#define M_FE_RFS_RAM_MEM_INIT                       ((UINT32)0x1U)

/*!
 * @brief FECSS RFS(M3) DATA bank1 mem init slice select
 */
#define M_FE_RFS_RAM_MEM_INIT_BANK1_SEL             ((UINT32)0x1U)

/*!
 * @brief The FECSS shared RAM clock enable config value - FEC clock
 */
#define M_FE_FECSS_SHARED_RAM_CLK_CFG_VALUE         ((UINT32)0x2U)


/*! @} */
/*******************************************************************************
 * TYPE-DEFINE STRUCT/ENUM/UNION DEFINITIONS
 *******************************************************************************
 */
/*!
 * @name FECSS Library Driver module Type defines
 * @{
 */

/*!
 * @brief FECSS Device specific data structure
 */
typedef struct
{
    /*!
     * @brief  Communication Interface Handle
     */
    T_DFP_COMIF_HDL         p_ComIfHdl;

    /*!
     * @brief  Platform HWI Handle
     */
    T_DFP_PLT_HWI_HDL       p_PltHwiHdl[M_FE_NUM_OF_INTERRUPTS];

    /*!
     * @brief  OSI Semph Handle
     */
    T_DFP_OSI_SEM_HDL       p_SemHdl;

    /*!
     * @brief  OSI Mutex Handle
     */
    T_DFP_OSI_MUTEX_HDL     p_MutexHdl;

} T_FE_DEVICE_DATA;

/*!
 * @brief FECSS Power and clock (PRCM) specific data structure
 */
typedef struct
{
    /*!
     * @brief  FECSS Device Clock Source type, XTAL or FClock (100MHz APLL/DPLL) or OFF (Gate)
     */
    UINT8 c_DevClkCtrl;

    /*!
     * @brief  FECSS Frame timer Clock Source type, XTAL or OFF (Gate)
     */
    UINT8 c_FtClkCtrl;

    /*!
     * @brief  Reserved
     */
    UINT16 h_XtalClkFreq;

    /*!
     * @brief  FECSS APLL Clock control type, ON, ON_CAL or OFF
     */
    UINT8 c_ApllClkCtrl;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved1;

    /*!
     * @brief  Reserved
     */
    UINT16 h_Reserved2;

} T_FE_FECSS_DATA;

/*!
 * @brief FECSS RFS Driver specific data structure
 */
typedef struct
{
    /*!
     * @brief  RFS boot Status, 0 - UnInit, 0xA - Pass, 0x5 -Fail
     */
    UINT8 c_BootStatus;

    /*!
     * @brief  RFS IPC MB Status, if any failure in IPC recorded here
     * 0 - UnInit, 0xA - Pass, 0x5 - IPC failure
     */
    UINT8 c_MbStatus;

    /*!
     * @brief  RFS IPC last command sent
     */
    UINT16 h_MbLastCmd;

    /*!
     * @brief  RFS IPC last response received
     */
    UINT16 h_MbLastRes;

    /*!
     * @brief  Reserved
     */
    UINT8 c_RfsRespSemaSigFail;

    /*!
     * @brief  Reserved
     */
    UINT8 c_Reserved;

    /*!
     * @brief  RFS boot time
     */
    UINT32 w_BootTime;

} T_FE_RFS_DATA;

/*!
 * @brief FECSS driver handler global data structure
 */
typedef struct
{
    /*!
     * @brief  Runtime Active Device map init info, FECSS device SW framework init status
     */
    UINT32                  w_ActiveDevInit;

    /*!
     * @brief  Runtime Active Device map powerup info, FECSS device HW powerup status
     */
    UINT32                  w_ActiveDevPwrUp;

    /*!
     * @brief  FECSS client call back info data structure, this is a interface data
     * should be commonly used by all devices, application shall maintain this data
     * identical across device instances thread.
     */
    T_DFP_CLIENT_CB_DATA    z_ClientCbData;

    /*!
     * @brief  FECSS device SW data
     */
    T_FE_DEVICE_DATA        z_DevData[M_DFP_MAX_DEVICES];

    /*!
     * @brief  FECSS PRCM data
     */
    T_FE_FECSS_DATA         z_FecssData[M_DFP_MAX_DEVICES];

    /*!
     * @brief  FECSS RFS data
     */
    T_FE_RFS_DATA           z_RfsData[M_DFP_MAX_DEVICES];

} T_FE_DRIVER_DATA;

/*! @} */
/*******************************************************************************
 * EXTERN GLOBAL VARIABLES/DATA-TYPES DECLARATIONS
 *******************************************************************************
 */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 *******************************************************************************
 */
T_FE_DRIVER_DATA* fe_DrvGetHandle(void);
T_DFP_PRINT_FUNCP fe_getDbgLogFptr(void);
T_RETURNTYPE fe_hwMemclr(UINT32 * const p_srcAdd, UINT16 h_numOfWords);
T_RETURNTYPE fe_hwMemcopy(UINT32 *p_destAdd, const UINT32 *p_sourceAdd, UINT16 h_numOfWords);
T_RETURNTYPE fe_validateDevice(UINT8 c_devIndex);
T_RETURNTYPE fe_fecssOpen(UINT8 c_devIndex, UINT8 c_clkSrcSel, \
                UINT8 c_powerMode, const T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE fe_fecssClose(UINT8 c_devIndex, UINT8 c_powerMode, \
                    const T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE fe_fecssMemInit(UINT8 c_devIndex, const T_FE_DRIVER_DATA* p_feDrvData);
T_RETURNTYPE fe_fecssClockSwitch(UINT8 c_devIndex, UINT8 c_clkSrcSel);
T_RETURNTYPE fe_fecssClockGate(UINT8 c_devIndex, UINT8 c_clkSrcSel);
T_RETURNTYPE fe_fecssFtClockGate(UINT8 c_devIndex, UINT8 c_clkSrcSel);
T_RETURNTYPE fe_fecssDevclkSrcFtClkStsGet(UINT8 c_devIndex, UINT8 *p_devClkSrc, \
                                      UINT8 *p_ftClkSrc);
T_RETURNTYPE fe_fecssDieIdDataGet(UINT8 c_devIndex, \
                            T_RL_API_SENSOR_DIEID_RSP *p_resData);

#ifdef __cplusplus
}
#endif

/* End of group */
/*! @} */
#endif
/*
 * END OF fe_driver.h
 */


