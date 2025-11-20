/*!*****************************************************************************
 * @file fe_device.c
 *
 * @brief FECSSLib Device Module API functions
 *
 * @b Description @n
 * This file provides FECSS library Device Module API function services to mmWaveLink.
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
 * @addtogroup FECSSLIB_DEVICE_API FECSSLib Device Control driver API functions
 *  @{
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
 * INCLUDE FILES
 *******************************************************************************
 */
#include <fecsslib/fecsslib.h>
#include <fecsslib/common/fe_registers.h>
#include <fecsslib/rfscripter/include/rfs_driver.h>
#include <fecsslib/sensor/include/fe_sensdriver.h>

/*******************************************************************************
 * GLOBAL VARIABLES/DATA-TYPES DEFINITIONS
 *******************************************************************************
 */


/*******************************************************************************
 * FUNCTION DEFINITIONS
 *******************************************************************************
 */

/****************************FECSSLib Static Functions*************************/
/*!
 * @name FECSS Library Device module API Driver static function calls
 * @{
 */

/*! @} */

/******************************FECSSLib Device APIs****************************/

/*!*****************************************************************************
 * @brief FECSS device power ON API driver function
 *
 * @b Description @n
 * This FECSS API driver function configures the FECSS device power ON and clock settings.
 * The RFS M3 core is powered up part of this API.
 *
 * @b Assumption: The mmWaveLink Init is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_PWR_ON_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-399, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevPwrOn_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_PWR_ON_CMD *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    UINT32 w_devMap = (UINT32)1U << c_devIndex;
    UINT32 w_devPwrStateMap;
    T_FE_RFS_BOOT_INFO z_rfsBootInfo  = {0};
    T_FE_RFS_BOOT_STS z_rfsBootStatus = {0};
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    UINT8 c_rfsApiCtrl,
          c_xtalType;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if ((M_RL_FECSS_PWR_ON_MODE_WARM != p_cmdData->c_PowerMode) && \
            (M_RL_FECSS_PWR_ON_MODE_COLD != p_cmdData->c_PowerMode))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_DEV_PWR_MODE;
        }
        else if ((M_RL_FECSS_DEV_CLK_SRC_XTAL != p_cmdData->c_ClkSourceSel) && \
            (M_RL_FECSS_DEV_CLK_SRC_FCLK != p_cmdData->c_ClkSourceSel))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_DEV_CLK_SRC;
        }
        else if ((M_RL_FECSS_XTAL_CLK_FREQ_25M != p_cmdData->h_XtalClkFreq) && \
            (M_RL_FECSS_XTAL_CLK_FREQ_26M != p_cmdData->h_XtalClkFreq) && \
            (M_RL_FECSS_XTAL_CLK_FREQ_38P4M != p_cmdData->h_XtalClkFreq) && \
            (M_RL_FECSS_XTAL_CLK_FREQ_40M != p_cmdData->h_XtalClkFreq) && \
            (M_RL_FECSS_XTAL_CLK_FREQ_10M != p_cmdData->h_XtalClkFreq))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_XTAL_CLK_SRC;
        }
        else if (p_cmdData->c_ChirpTimerResol > M_RL_FECSS_CHIRP_TIMER_RES_11)
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_CT_RESOL;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then FECSS power up / down */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Device power state map  */
        w_devPwrStateMap = (p_feDrvData->w_ActiveDevPwrUp & w_devMap);

        /*!
        * -# if device is not already powered up then enter power ON sequence
        */
        if ((UINT32)0U == w_devPwrStateMap)
        {
            /*! -# Power up the FECSS power domain and config clock */
            xw_return = fe_fecssOpen(c_devIndex, p_cmdData->c_ClkSourceSel, \
                                p_cmdData->c_PowerMode, p_feDrvData);

            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*! -# Set w_ActiveDevPwrUp flag for the selected device */
                p_feDrvData->w_ActiveDevPwrUp |= w_devMap;

                /*! -# Update FECSS clock source type */
                p_feDrvData->z_FecssData[c_devIndex].c_DevClkCtrl = p_cmdData->c_ClkSourceSel;

                /*! -# Update FECSS XTAL clock freq */
                p_feDrvData->z_FecssData[c_devIndex].h_XtalClkFreq = p_cmdData->h_XtalClkFreq;

                /*! -# Configure Chirp timer resolution global setting  */
                xw_return = sens_configCtResolution(c_devIndex, p_cmdData->c_ChirpTimerResol);
            }

            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*!
                *  -# Open RFS (un halt cortex M3) using rfs_rfsOpen function
                *    - Populate z_rfsBootInfo data structure
                *    - Read z_rfsBootStatus info
                */
                /*! -# Select Fastclock freq M_RL_FECSS_FCLK_CLK_FREQ_80M if c_ClkSourceSel is
                 * M_RL_FECSS_DEV_CLK_SRC_FCLK
                 */
                if (M_RL_FECSS_DEV_CLK_SRC_FCLK == p_cmdData->c_ClkSourceSel)
                {
                    z_rfsBootInfo.h_ClkFreq   = M_RL_FECSS_FCLK_CLK_FREQ_80M;
                }
                /*! -# else Select XTAL clock frequency h_XtalClkFreq from API
                 */
                else
                {
                    z_rfsBootInfo.h_ClkFreq   = p_cmdData->h_XtalClkFreq;
                }

                /*! -# Derive XTAL clock type index from XTAL freq */
                if (M_RL_FECSS_XTAL_CLK_FREQ_25M == p_cmdData->h_XtalClkFreq)
                {
                    c_xtalType = M_FE_RFS_XTAL_CLK_FREQ_25M_INDX;
                }
                else if (M_RL_FECSS_XTAL_CLK_FREQ_26M == p_cmdData->h_XtalClkFreq)
                {
                    c_xtalType = M_FE_RFS_XTAL_CLK_FREQ_26M_INDX;
                }
                else if (M_RL_FECSS_XTAL_CLK_FREQ_38P4M == p_cmdData->h_XtalClkFreq)
                {
                    c_xtalType = M_FE_RFS_XTAL_CLK_FREQ_38P4M_INDX;
                }
                else if (M_RL_FECSS_XTAL_CLK_FREQ_40M == p_cmdData->h_XtalClkFreq)
                {
                    c_xtalType = M_FE_RFS_XTAL_CLK_FREQ_40M_INDX;
                }
                else
                {
                    c_xtalType = M_FE_RFS_XTAL_CLK_FREQ_10M_INDX;
                }

                /*! -# Update RFS boot Power mode and XTAL clock type */
                z_rfsBootInfo.c_PwrModeAndXtalType = p_cmdData->c_PowerMode | \
                            (UINT8)(c_xtalType << M_FE_RFS_XTAL_CLK_TYPE_OFFSET);

                /*! -# Update RFS API control as per mmwaveLink input callback control */
                c_rfsApiCtrl = (p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis & (UINT8)0x1U);
                c_rfsApiCtrl |= (UINT8)((p_feDrvData->z_ClientCbData.c_ApiDbglogEn & (UINT8)0x1U) << 1U);
                z_rfsBootInfo.c_RfsApiCtrl = c_rfsApiCtrl;

                /*! -# Update RFS Boot cfg control */
                z_rfsBootInfo.c_FecBootCfg = p_cmdData->c_FecBootCfg;

                /*! -# Call rfs_rfsOpen function */
                xw_return = rfs_rfsOpen(c_devIndex, z_rfsBootInfo, &z_rfsBootStatus, p_feDrvData);
            }
        }
        else
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVALID_INPUT;
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_fecssDevPwrOn API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS device power OFF API driver function
 *
 * @b Description @n
 * This FECSS API driver function configures the FECSS device in power OFF state based
 * on power mode. The RFS M3 core is powered down part of this API.
 *
 * @b Assumption: The FECSS powerup is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_PWR_OFF_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-400, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevPwrOff_rom(UINT8 c_devIndex, \
                            const T_RL_API_FECSS_DEV_PWR_OFF_CMD *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;
    UINT32 w_devMap = (UINT32)1U << c_devIndex;
    UINT32 w_devPwrStateMap;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis)
    {
        if ((M_RL_FECSS_PWR_DOWN_RET_ON != p_cmdData->c_RetentionMode) && \
            (M_RL_FECSS_PWR_DOWN_RET_OFF != p_cmdData->c_RetentionMode))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_DEV_PWR_MODE;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }
    else
    {
        xw_return = M_DFP_RET_CODE_OK;
    }

    /*! -# If error check is success then FECSS power up / down */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Device power state map  */
        w_devPwrStateMap = (p_feDrvData->w_ActiveDevPwrUp & w_devMap);

        /*!
        * -# if device is already powered up then enter power OFF sequence
        */
        if ((UINT32)0U != w_devPwrStateMap)
        {
            /*! -# Close RFS (Reset and halt M3)  */
            xw_return = rfs_rfsClose(c_devIndex, p_feDrvData);

            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*! -# Clear w_ActiveDevPwrUp flag for the selected device */
                p_feDrvData->w_ActiveDevPwrUp &= ~(w_devMap);

                /*! -# Power down the FECSS power domain and retain memory setting */
                xw_return = fe_fecssClose(c_devIndex, p_cmdData->c_RetentionMode, p_feDrvData);

                /*! -# Set APLL status to off before turning-off FECSS */
                p_feDrvData->z_FecssData[c_devIndex].c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_OFF;
            }
        }
        else
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVALID_INPUT;
        }
    }

    /*! -# Print Debug Info */
    M_DFP_LOG_INFO_ARG2("FECSS fe_fecssDevPwrOff API:%d:%d\n", c_devIndex, xw_return);

    /*! -# Return the status xw_return */
    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS device clock control API error check function
 *
 * @b Description @n
 * This FECSS API driver function checks the API inputs for FECSS device clock
 *
 * @b Assumption: The FECSS Init is done before issuing this API.
 *
 * @param[in] c_disableErrorCheck  - Disable error check
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_CLK_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-401, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevClockCtrlCheck_rom(UINT8 c_disableErrorCheck, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData)
{
    T_RETURNTYPE xw_return = M_DFP_RET_CODE_OK;

    /*! -# If error check is enabled then perform the checks */
    if (M_FALSE == c_disableErrorCheck)
    {
        if ((M_RL_FECSS_DEV_CLK_SRC_XTAL != p_cmdData->c_DevClkCtrl) && \
            (M_RL_FECSS_DEV_CLK_SRC_FCLK != p_cmdData->c_DevClkCtrl) && \
            (M_RL_FECSS_DEV_CLK_SRC_OFF != p_cmdData->c_DevClkCtrl))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_DEV_CLK_SRC;
        }
        else if ((M_RL_FECSS_FT_CLK_SRC_XTAL != p_cmdData->c_FtClkCtrl) && \
            (M_RL_FECSS_FT_CLK_SRC_OFF != p_cmdData->c_FtClkCtrl))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_FT_CLK_SRC;
        }
        else if ((M_RL_FECSS_APLL_CTRL_OFF != p_cmdData->c_ApllClkCtrl) && \
            (M_RL_FECSS_APLL_CTRL_ON != p_cmdData->c_ApllClkCtrl) && \
            (M_RL_FECSS_APLL_CTRL_ON_CAL != p_cmdData->c_ApllClkCtrl))
        {
            /*! -# Return Invalid input error code */
            xw_return = M_DFP_RET_CODE_INVAL_APLL_CLK_CTRL;
        }
        else
        {
            xw_return = M_DFP_RET_CODE_OK;
        }
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS device clock control API function
 *
 * @b Description @n
 * This FECSS API driver function gates and configure the FECSS device and
 * frame timer clock source as per the command. The entire FECSS clock can be gated
 * (enter sleep) using this driver function.
 *
 * @b Assumption: The FECSS Init is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[in] p_cmdData   - pointer to command data @ref T_RL_API_FECSS_DEV_CLK_CTRL_CMD
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-402, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevClockCtrl_rom(UINT8 c_devIndex, \
                const T_RL_API_FECSS_DEV_CLK_CTRL_CMD *p_cmdData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return,
                 xw_return1;
    T_FE_DRIVER_DATA* p_feDrvData = fe_DrvGetHandle();
    UINT16 h_clkFreq;

    /** -# Perform API error check */
    xw_return = fe_fecssDevClockCtrlCheck(p_feDrvData->z_ClientCbData.c_ApiErrorCheckDis, p_cmdData);

    /*! -# If error check is success then control FECSS & FRC clock */
    if (M_DFP_RET_CODE_OK == xw_return)
    {
        /*! -# Update FECSS device clock as per input and the new state shall be different from
         * previous state
         */
        if (p_feDrvData->z_FecssData[c_devIndex].c_DevClkCtrl != p_cmdData->c_DevClkCtrl)
        {
            /*! -# UnGate/Gate FECSS clock before clock switch based on the input */
            xw_return = fe_fecssClockGate(c_devIndex, p_cmdData->c_DevClkCtrl);

            /*!
            * -# Set FECSS clock as per c_clkSrcSel
            *    - Fast clock DIG_PLL or APLL (160MHz) / DIV (2) = 80MHz
            *    - XTAL clock is 40MHz (typical) / DIV 1 = 40MHz
            */
            if (M_RL_FECSS_DEV_CLK_SRC_OFF != p_cmdData->c_DevClkCtrl)
            {
                xw_return1 = fe_fecssClockSwitch(c_devIndex, p_cmdData->c_DevClkCtrl);
                xw_return += xw_return1;

                /*! -# if the source clock selected is Fast clock then set RFS clock 80MHz */
                if (M_RL_FECSS_DEV_CLK_SRC_FCLK == p_cmdData->c_DevClkCtrl)
                {
                    h_clkFreq = M_RL_FECSS_FCLK_CLK_FREQ_80M;
                }
                /*! -# if the source clock selected is XTAL then set RFS clock XTAL clock */
                else
                {
                    h_clkFreq = p_feDrvData->z_FecssData[c_devIndex].h_XtalClkFreq;
                }

                xw_return1 = rfs_rfsClkCfg(c_devIndex, h_clkFreq);
                xw_return += xw_return1;
            }

            /*! If all configurations are successful then */
            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*! -# Update FECSS clock source type */
                p_feDrvData->z_FecssData[c_devIndex].c_DevClkCtrl = p_cmdData->c_DevClkCtrl;
            }
        }

        /*! -# Update FECSS Frame timer (FT) clock as per input and the new state shall
         * be different from previous state
         */
        if ((p_feDrvData->z_FecssData[c_devIndex].c_FtClkCtrl != p_cmdData->c_FtClkCtrl) && \
            (M_DFP_RET_CODE_OK == xw_return))
        {
            /*! Gate / Ungate the FT clock based on input */
            xw_return = fe_fecssFtClockGate(c_devIndex, p_cmdData->c_FtClkCtrl);

            /*! If all configurations are successful then */
            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*! -# Update FECSS Frame Timer clock source type */
                p_feDrvData->z_FecssData[c_devIndex].c_FtClkCtrl = p_cmdData->c_FtClkCtrl;
            }
        }

        /*! -# Update FECSS APLL clock as per input and the new state shall
         * be different from previous state
         */
        if ((p_feDrvData->z_FecssData[c_devIndex].c_ApllClkCtrl != p_cmdData->c_ApllClkCtrl) && \
            (M_DFP_RET_CODE_OK == xw_return))
        {
            /*! Control APLL clock based on input */
            xw_return = rfs_cmdApllClkCtrl(c_devIndex, p_cmdData);

            /*! If all configurations are successful then */
            if (M_DFP_RET_CODE_OK == xw_return)
            {
                /*! -# Update FECSS Frame Timer clock source type */
                p_feDrvData->z_FecssData[c_devIndex].c_ApllClkCtrl = p_cmdData->c_ApllClkCtrl;

                /*! -# If APLL is not OFF then update the state to M_RL_FECSS_APLL_CTRL_ON */
                if (M_RL_FECSS_APLL_CTRL_OFF != p_feDrvData->z_FecssData[c_devIndex].c_ApllClkCtrl)
                {
                    p_feDrvData->z_FecssData[c_devIndex].c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_ON;
                }
            }
        }
    }

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS device status GET
 *
 * @b Description @n
 * This FECSS API driver function Get the FECSS Device Status
 *
 * @b Assumption: The FECSS Init is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_rspData   - pointer to response data @ref T_RL_API_FECSS_DEV_STS_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-403, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDevStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_FECSS_DEV_STS_RSP *p_rspData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return, xw_return1;
    T_FE_RFS_BOOT_STS   z_rfsBootSts = {0};
    T_FE_RFS_BOOT_INFO  z_rfsBootInfo = {0};
    T_RL_API_RFS_FAULT_STS_GET_RSP z_rfsCpuFaultSts = {0};

    /*! -# Get RFS CPU status  */
    xw_return = rfs_cmdCpuStsGet(c_devIndex, &z_rfsCpuFaultSts);
    p_rspData->h_RfsFwState     = z_rfsCpuFaultSts.h_RfsFwState;
    p_rspData->c_PatchAndSilEna = z_rfsCpuFaultSts.c_PatchAndSilEna;
    p_rspData->c_RfAndSensType  = z_rfsCpuFaultSts.c_RfAndSensType;

    /*! -# Get RFS Boot self test status  */
    xw_return1 = rfs_rfsBootStatusGet(c_devIndex, &z_rfsBootSts);
    xw_return += xw_return1;
    p_rspData->c_RfsBootSelfTest = z_rfsBootSts.c_RfsBootSelfTest;

    /*! -# Get RFS Boot info status
     *     - Xtal Freq
     *     - Power Mode
     */
    xw_return1 = rfs_rfsBootInfoGet(c_devIndex, &z_rfsBootInfo);
    xw_return += xw_return1;
    p_rspData->h_XtalClkFreq = z_rfsBootInfo.h_ClkFreq;
    p_rspData->c_PowerModeState =  \
            (z_rfsBootInfo.c_PwrModeAndXtalType & (UINT8)0x0FU);

    /*! -# Get CT resolution */
    xw_return1 = sens_getCtResolution(c_devIndex, &p_rspData->c_ChirpTimerResType);
    xw_return += xw_return1;

    /*! Get Clock Status */
    xw_return1 = fe_fecssDevclkSrcFtClkStsGet(c_devIndex, \
                    &p_rspData->c_DevClkSrcState, \
                    &p_rspData->c_FtClkState);
    xw_return += xw_return1;

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS device RFS CPU Fault status GET
 *
 * @b Description @n
 * This FECSS API driver function Get the FECSS Device CPU Fault Status
 *
 * @b Assumption: The FECSS Init is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_rspData   - pointer to response data @ref T_RL_API_RFS_FAULT_STS_GET_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-404, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssRfsFaultStatusGet_rom(UINT8 c_devIndex, \
                            T_RL_API_RFS_FAULT_STS_GET_RSP *p_rspData)
{
    /*! -# T_RETURNTYPE type id SINT32 value */
    T_RETURNTYPE xw_return;

    /*! -# Get RFS CPU fault status  */
    xw_return = rfs_cmdCpuStsGet(c_devIndex, p_rspData);

    return xw_return;
}

/*!*****************************************************************************
 * @brief FECSS DIEID GET API driver
 *
 * @b Description @n
 * This FECSS API driver function to Get the DIE ID
 *
 * @b Assumption: The FECSS Device ON is done before issuing this API.
 *
 * @param[in] c_devIndex  - Device Index
 * @param[out] p_resData  - pointer to response data @ref T_RL_API_SENSOR_DIEID_RSP
 *
 * @retval T_RETURNTYPE - Signed Return status, Success - 0, Failure - Error code < 0
 *
 * -----------------------------------------------------------------------------
 * <B> Traceability Information </B> @n
 *
 * <table>
 *
 * <tr> <th> @b Design-Ids <td> MMWAVE_DFP_REQ-405, MMWAVE_DFP_REQ-472
 *
 * </table>
 *
 * -----------------------------------------------------------------------------
 *
 * @b Documentation @n
 *
 *******************************************************************************
 */
T_RETURNTYPE fe_fecssDieIdGet_rom(UINT8 c_devIndex, \
                            T_RL_API_SENSOR_DIEID_RSP *p_resData)
{
    T_RETURNTYPE xw_return;

    xw_return = fe_fecssDieIdDataGet(c_devIndex, p_resData);

    return xw_return;
}

/* End of group */
/*! @} */

/*
 * END OF fe_sensor.c
 */
