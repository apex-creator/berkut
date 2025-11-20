/*
 * Copyright (C) 2022 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
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
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/*Include Files. */
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include <drivers/mcspi.h>

/* Local Include Files*/
#include "ripple_link.h"
#include "./ripple_flash/ripple_flash.h"
#include "./ripple_cli/ripple_cli.h"
#include "./ripple.h"
#include "ti_drivers_open_close.h"
#include "ti_drivers_config.h"
/*dfp Include Files*/
#include <drivers/hw_include/cslr_adcbuf.h>
#include <mmwavelink/include/rl_device.h>
#include <mmwavelink/include/rl_sensor.h>


/*External Global variables*/
extern MmwDemo_MSS_MCB gMmwMssMCB;
extern uint8_t gATECalibDataStorage[132U];



/* Global mmWave Control MCB: */
MMWave_MCB           gMMWave_MCB;
SemaphoreP_Object gMmwOsalMutex;
SemaphoreP_Object gMmwOsalSem;

/* APPSS Debug Data */
uint32_t AppDebugData[16];
uint8_t gCalibDataStorage[(ATE_CALIB_DATA_LENGTH + 4)] __attribute__((aligned(8))) = {0};
T_SensPerChirpLut* sensPerChirpLuTable = (T_SensPerChirpLut*)(0x21880000U);
uint32_t MMWave_ISRHandle = 18U;
int32_t             transferOK;

static uint8_t gMmwHeapMem[MMWAVE_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));
static HeapP_Object gMmwHeapObj;
static bool isHeapMemAllocated = false;

static uint32_t MMWave_TimeStamp(void)
{
    return 0;
}

static int32_t MMWave_SysDelay(uint32_t delayMicroSec)
{
    ClockP_usleep(delayMicroSec/10);
    return 0;
}
static uint64_t MMWave_EnterCriticalRegion(void)
{
    HwiP_disable();
    return 0;
}

static void MMWave_ExitCriticalRegion(uint64_t key)
{
    HwiP_enable();
}
static void Mmwave_EnChannelSetOffset
(
    CSL_app_hwa_adcbuf_ctrlRegs *ptrAdcBufCtrlRegs,
    uint8_t channel,
    uint16_t offset
)
{
    switch(channel)
    {
        case 0U:
            /* Enable the channel */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN,
                    1);

            /* Setup the offset */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG2,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0,
                    (offset >> 4));
            break;
        case 1U:
            /* Enable the channel */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN,
                    1);

            /* Setup the offset */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG2,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1,
                    (offset >> 4));
            break;
        case 2U:
            /* Enable the channel */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN,
                    1);

            /* Setup the offset */
            CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG3,
                    APP_HWA_ADCBUF_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2,
                    (offset >> 4));
            break;

        default:
            /* Not  supported channels, code should not end up here */
            DebugP_assert(0);
            break;
    }
}

/**
 *  @b Description
 *  @n
 *      The function is called to concatenate the src list to the end of the
 *      destination list.
 *
 *  @param[in]  ptr_dst
 *      This is the head of the destination list.
 *  @param[in]  ptr_src
 *      This is the head of the source list.
 *
 *  @retval
 *      Not Applicable
 */
void MMWave_listCat (MMWave_ListNode** ptr_dst, MMWave_ListNode** ptr_src)
{
    MMWave_ListNode*    ptr_node;
    MMWave_ListNode*    ptr_prev;

    /* Is the source list empty? */
    if (*ptr_src != NULL)
    {
        /* NO: Is the destination list empty? */
        if (*ptr_dst == NULL)
        {
            /* YES: Make the source now as the destination. */
            *ptr_dst = *ptr_src;
        }
        else
        {
            /* NO: Both the lists are not empty. */
            ptr_node = *ptr_dst;
            ptr_prev = NULL;

            /* Reach the end of the list. */
            while (ptr_node != NULL)
            {
                ptr_prev = ptr_node;
                ptr_node = ptr_node->p_next;
            }

            /* Link the last element to the source list. */
            ptr_prev->p_next = *ptr_src;
            (*ptr_src)->p_prev = ptr_prev;
        }
    }
    return;
}

static void MmwDemo_ADCBufConfig
(
    uint16_t rxChannelEn,
    uint32_t chanDataSize
)
{
    CSL_app_hwa_adcbuf_ctrlRegs *ptrAdcBufCtrlRegs = (CSL_app_hwa_adcbuf_ctrlRegs *)CSL_APP_HWA_ADCBUF_CTRL_U_BASE;
    uint8_t channel = 0;
    uint16_t offset = 0;
    uint32_t chanDataSizeAligned16 = 0;

    chanDataSizeAligned16 = ((chanDataSize + 15) / 16) * 16;

    /* Disable all ADCBuf channels */
    CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1, APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN, 0);
    CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1, APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN, 0);
    CSL_FINS(ptrAdcBufCtrlRegs->ADCBUFCFG1, APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN, 0);

    /* Enable Rx Channels */
    for (channel = 0; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(rxChannelEn & (0x1U << channel))
        {
            /* Enable Channel and configure offset. */
            Mmwave_EnChannelSetOffset(ptrAdcBufCtrlRegs, channel, offset);

            /* Calculate offset for the next channel */
            offset  += chanDataSizeAligned16;
        }
    }

    return;
}

/**************************************************************************
 ************************* Common Test Functions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Utility function which populates the profile configuration with
 *      well defined defaults.
 *
 *  @param[out]  ptrProfileCfg
 *      Pointer to the populated profile configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultProfileCfg (T_RL_API_SENS_CHIRP_PROF_COMN_CFG* ptrProfileCfg, T_RL_API_SENS_CHIRP_PROF_TIME_CFG* ptrProfileTimeCfg)
{
    /* Initialize the profile configuration: */
    memset ((void*)ptrProfileCfg, 0, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));

    /* Populate the *default* profile configuration: */
    ptrProfileCfg->c_DigOutputSampRate = gMmwMssMCB.profileComCfg.c_DigOutputSampRate; //23; //8; //M_RL_SENS_DIG_OUT_SAMP_RATE_MAX_12P5M;
    ptrProfileCfg->c_DigOutputBitsSel = gMmwMssMCB.profileComCfg.c_DigOutputBitsSel; //0; //M_RL_SENS_DIG_OUT_12BITS_4LSB_ROUND;
    ptrProfileCfg->c_DfeFirSel = gMmwMssMCB.profileComCfg.c_DfeFirSel; //0; //M_RL_SENS_DFE_FIR_LONG_FILT;
    ptrProfileCfg->c_VcoMultiChipMode = 0; //M_RL_SENS_VCO_MULT_CHIP_SINGLE;
    ptrProfileCfg->h_NumOfAdcSamples = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples; //128; //256U; /* 2.56us */
    ptrProfileCfg->c_ChirpTxMimoPatSel = gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel; //4; //0; //M_RL_SENS_TX_MIMO_PATRN_DIS;

    ptrProfileCfg->c_MiscSettings = gMmwMssMCB.profileComCfg.c_MiscSettings; //0U; /* HPF FINIT, CRD ena, PA blank dis */
    ptrProfileCfg->c_HpfFastInitDuration = gMmwMssMCB.profileComCfg.c_HpfFastInitDuration; //15U; /* 1.5us */
    ptrProfileCfg->h_CrdNSlopeMag = gMmwMssMCB.profileComCfg.h_CrdNSlopeMag; //0; //0x800U; /* default slope */

    //ptrProfileCfg->h_ChirpRampEndTime = 200U; /* 4us high res */
    ptrProfileCfg->h_ChirpRampEndTime = gMmwMssMCB.profileComCfg.h_ChirpRampEndTime; //361; //600; //250U; /* 25us low res */
    ptrProfileCfg->c_ChirpRxHpfSel = gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel; //1; //M_RL_SENS_RX_HPF_SEL_350KHZ;

    /*ptrProfileCfg->c_ChirpRxGainSel = (36U | \
                    (M_RL_SENS_RF_GAIN_TARG_1 << M_RL_SENS_RF_GAIN_OFFSET));
    ptrProfileCfg->c_ChirpTxBackOffSel[0] = 0U;
    ptrProfileCfg->c_ChirpTxBackOffSel[1] = 0U;*/

    /* Initialize the profile time configuration: */
    memset ((void*)ptrProfileTimeCfg, 0, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    //ptrProfileTimeCfg->h_ChirpIdleTime = 325U; /* 6.5us high res */
    ptrProfileTimeCfg->h_ChirpIdleTime = gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime; //80; //400; //65U; /* 6.5us low res */
    ptrProfileTimeCfg->h_ChirpAdcStartTime = gMmwMssMCB.profileTimeCfg.h_ChirpAdcStartTime; //300; //30770;
    /*((UINT16)25U | \
            ((UINT16)5U << M_RL_SENS_CHIRP_ADC_SKIP_SAMP_OFFSET)); 0.5us Fract + 0.4us skip */
    ptrProfileTimeCfg->xh_ChirpTxStartTime = gMmwMssMCB.profileTimeCfg.xh_ChirpTxStartTime; //0; //-10; /* -0.2us */
    ptrProfileTimeCfg->xh_ChirpRfFreqSlope = gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope; //419; //699; //3495; /* 100MHz/us , 77G - 2621 */
    //ptrProfileTimeCfg->w_ChirpRfFreqStart  = M_RL_SENS_CHIRP_RFFREQ_HR_57G; /* 57GHz / 76GHz High */
    ptrProfileTimeCfg->w_ChirpRfFreqStart  = gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart; //51200; //50347; //M_RL_SENS_CHIRP_RFFREQ_LR_57G; /* 57GHz / 76GHz low */
    ptrProfileTimeCfg->h_ChirpTxEnSel = gMmwMssMCB.profileTimeCfg.h_ChirpTxEnSel;//0x3U; /* 2 TX enable in chirp */
    ptrProfileTimeCfg->h_ChirpTxBpmEnSel = gMmwMssMCB.profileTimeCfg.h_ChirpTxBpmEnSel; //0x3U; //0; //0x2U; /* TX1 BPM enable in chirp */
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to unlock the mutex
 *
 *  @param[in]  mutexHandle
 *      Handle to the mutex object
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalMutexUnlock(T_DFP_OSI_MUTEX_HDL mutexHandle)
{
    /* Post the semaphore */
    SemaphoreP_post((SemaphoreP_Object *)(mutexHandle));
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to create a semaphore
 *
 *  @param[out]  semHandle
 *      Handle to the semaphore object which is to be created
 *  @param[in]   name
 *      Name of the semaphore object
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalSemCreate(T_DFP_OSI_SEM_HDL* semHandle, uint8_t* name)
{
    int32_t              retVal;

    retVal = SemaphoreP_constructBinary(&gMmwOsalSem, 0);

    if(retVal == SystemP_FAILURE)
    {
        /* Error: Unable to create the semaphore */
        retVal       = MINUS_ONE;
        *semHandle = NULL;
    }
    else
    {
        /* Successfully created the semaphore */
        retVal    = 0;
        *semHandle = (T_DFP_OSI_SEM_HDL*)&gMmwOsalSem;
    }

    return (int32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      Utility function which populates the chirp configuration with
 *      well defined defaults.
 *
 *  @param[out]  ptrChirpCfg
 *      Pointer to the populated chirp configuration
 *
 *  @retval
 *      Not applicable
 */
static void Mmwave_populateDefaultChirpCfg (T_RL_API_SENS_PER_CHIRP_CFG* ptrChirpCfg, T_RL_API_SENS_PER_CHIRP_CTRL* ptrChirpCtrl)
{
    /* Initialize the chirp configuration: */
    memset ((void*)ptrChirpCfg, 0, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));

    /* Populate the chirp configuration: */
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_START] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_IDLE_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_START_TIME] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_TX_ENABLE] = 4;
    ptrChirpCfg->h_ParamArrayLen[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = 4;

    /* repeat count is not applicable for acc chirps, so new chirp param is picked after 2*2 chirps */
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_START] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_IDLE_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_START_TIME] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_TX_ENABLE] = 1;
    ptrChirpCfg->h_ParamRptCount[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = 1;

    /* Initialize the chirp control configuration: */
    memset ((void*)ptrChirpCtrl, 0, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));

    /* Sensor per chirp control api */
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_START] = \
        ((UINT32)(&sensPerChirpLuTable->StartFreqLowRes[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_FREQ_SLOPE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpSlope[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_IDLE_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpIdleTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_ADC_START_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpAdcStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_START_TIME] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpTxStartTime[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_TX_ENABLE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpTxEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);
    ptrChirpCtrl->h_ParamArrayStartAdd[M_RL_SENS_PER_CHIRP_BPM_ENABLE] = \
        ((UINT32)(&sensPerChirpLuTable->ChirpBpmEn[0]) & M_RL_SENS_PER_CHIRP_LUT_ADD_MASK);

    ptrChirpCtrl->h_PerChirpParamCtrl = M_RL_SENS_PER_CHIRP_CTRL_MAX;
}

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\r\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      The function is called to add a node to the list.
 *
 *  @param[in]  ptr_list
 *      This is the list to which the node is to be added.
 *  @param[in]  ptr_node
 *      This is the node which is to be added.
 *
 *  @retval
 *      Not Applicable
 */
void MMWave_listAdd (MMWave_ListNode** ptr_list, MMWave_ListNode* ptr_node)
{
    MMWave_ListNode*    ptr_head;

    /* Check if the list is empty ? */
    if (*ptr_list == NULL)
    {
        /* YES the list is empty. Initialize the links */
        ptr_node->p_next = NULL;
        ptr_node->p_prev = NULL;

        /* Initialize the LIST */
        *ptr_list = ptr_node;
    }
    else
    {
        /* No the list was NOT empty. Add the node to the beginning of list.
         * Get the current head of the list. */
        ptr_head = *ptr_list;

        /* Initialize the new head of the list. */
        ptr_node->p_next  = ptr_head;
        ptr_node->p_prev = NULL;

        /* Update the old head to point to the new head */
        ptr_head->p_prev = ptr_node;

        /* Update the pointer to the head of the list. */
        *ptr_list = ptr_node;
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      The function is called to remove the head node from the list.
 *
 *  @param[in]  ptr_list
 *      This is the pointer to the list from where nodes will be removed.
 *
 *  @retval
 *      Pointer to the head of the list.
 */
MMWave_ListNode* MMWave_listRemove (MMWave_ListNode** ptr_list)
{
    MMWave_ListNode*    ptr_head;
    MMWave_ListNode*    ptr_node = NULL;

    /* Check if the list is empty ? */
    if (*ptr_list != NULL)
    {
        /* NO: Get the head of the list. */
        ptr_node = *ptr_list;

        /* Move the head to the next element in the list. */
        ptr_head = ptr_node->p_next;
        *ptr_list = ptr_head;

        /* Did we remove the last element?*/
        if (ptr_head != NULL)
        {
            /* No; in that case update the pointers for the new head. */
            ptr_head->p_prev = NULL;
        }

        /* Kill the links before returning the OLD head. */
        ptr_node->p_next = NULL;
        ptr_node->p_prev = NULL;
    }
    return ptr_node;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the head of the specific list
 *
 *  @param[in]  ptr_list
 *      This is the pointer to the list.
 *
 *  @retval
 *      Head of the list (could be NULL if the list is empty)
 */
MMWave_ListNode* MMWave_listGetHead (MMWave_ListNode** ptr_list)
{
    return *ptr_list;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the next element in the list.
 *
 *  @param[in]  ptr_list
 *      This is the pointer to the node in the list.
 *
 *  @retval
 *      Next element in the list. (could be NULL if this is the last element)
 */
MMWave_ListNode* MMWave_listGetNext (MMWave_ListNode* ptr_list)
{
    return ptr_list->p_next;
}
/**
 *  @b Description
 *  @n
 *      The function is used to encode the error code. This encoded error
 *      will be returned back to the application.
 *
 *  @param[in]  errorLevel
 *      Error level to be encoded
 *  @param[in]  mmWaveError
 *      Error code for the mmWave
 *  @param[in]  subSysError
 *      Subsystem error code
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Encoded error code
 */
int32_t MMWave_encodeError
(
    MMWave_ErrorLevel   errorLevel,
    int32_t             mmWaveError,
    int32_t             subSysError
)
{
    int32_t     encodedErrorCode;
    uint32_t    tmpEncodedErrorCode;

    /* Determine the error level */
    if (errorLevel == MMWave_ErrorLevel_SUCCESS)
    {
        /* No error: */
        encodedErrorCode = 0;
    }
    else
    {
        /* Warning or Error: */
        tmpEncodedErrorCode = CSL_FMKR (31U, 16U, (uint32_t)mmWaveError);
        tmpEncodedErrorCode = tmpEncodedErrorCode | CSL_FMKR (15U, 2U,  (uint32_t)subSysError);
        tmpEncodedErrorCode = tmpEncodedErrorCode | CSL_FMKR (1U,  0U,  (uint32_t)errorLevel);

        /* Convert the error code into an integer. */
        encodedErrorCode = (int32_t)tmpEncodedErrorCode;
    }
    return encodedErrorCode;
}


/**
 *  @b Description
 *  @n
 *      This is a utility function which given the error code will return
 *      the corresponding error level.
 *
 *  @param[in]  errCode
 *      Encoded error code retreived from the mmWave API. This is the error
 *      code which will be decoded.
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Decoded Error Level
 */
MMWave_ErrorLevel MMWave_decodeErrorLevel (int32_t errCode)
{
    return (MMWave_ErrorLevel)CSL_FEXTR ((uint32_t)errCode, 1U, 0U);
}


/**
 *  @b Description
 *  @n
 *      Application can use the API to decode the error. The mmWave API
 *      can return errors because of either the mmWave or the underlying
 *      mmWave Link API.
 *
 *  @param[in]  errCode
 *      Encoded error code retreived from the mmWave API. This is the error
 *      code which will be decoded.
 *  @param[out] errorLevel
 *      Error level populated by the API.
 *  @param[out] mmWaveError
 *      Error code from the mmWave module. This could be set to 0
 *      to indicate that the mmWave module did not return an error.
 *  @param[out] subSysError
 *      Subsystem error code. This could be set to 0 to indicate that there
 *      was no error at the subsystem level.
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void MMWave_decodeError
(
    int32_t             errCode,
    MMWave_ErrorLevel*  errorLevel,
    int16_t*            mmWaveError,
    int16_t*            subSysError
)
{
    uint16_t    tmpSubSysError;

    /* Get the error level: */
    *errorLevel = (MMWave_ErrorLevel)CSL_FEXTR ((uint32_t)errCode, 1U, 0U);

    /* Determine the error level */
    if (*errorLevel == MMWave_ErrorLevel_SUCCESS)
    {
        /* No error: */
        *mmWaveError = 0;
        *subSysError = 0;
    }
    else
    {
        /* Warning or Error: */
        *mmWaveError = (int16_t)CSL_FEXTR ((uint32_t)errCode, 31U, 16U);

        /* Get the subsystem error: Handle the sign extension correctly. */
        tmpSubSysError = CSL_FEXTR ((uint32_t)errCode, 15U, 2U) << 2;
        tmpSubSysError = tmpSubSysError >> 2;
        *subSysError   = (int16_t)tmpSubSysError;
    }
    return;
}


/**
 *  @b Description
 *  @n
 *      The function is used to add the profile with the specific
 *      profile configuration.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  ptrProfileCfg
 *      Pointer to the profile configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the profile
 *  @retval
 *      Error   -   NULL
 */
MMWave_ProfileHandle MMWave_addProfile
(
    MMWave_Handle           mmWaveHandle,
    const T_RL_API_SENS_CHIRP_PROF_COMN_CFG*   ptrProfileCfg,
    const T_RL_API_SENS_CHIRP_PROF_TIME_CFG*   ptrProfileTimeCfg,
    int32_t*                errCode
)
{
    MMWave_MCB*             ptrMMWaveMCB;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_ProfileHandle    retHandle = NULL;

    /* Initialize the error code: */
    *errCode = 0;

    if(!isHeapMemAllocated)
    {
        /* create heap for profile, chirp and BPM config. */
        HeapP_construct(&gMmwHeapObj, gMmwHeapMem, MMWAVE_HEAP_MEM_SIZE);
    }

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (ptrProfileCfg == NULL) || (ptrProfileTimeCfg == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the mmWave MCB: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Allocate memory for the Profile: */
    ptrMMWaveProfile = HeapP_alloc(&gMmwHeapObj, (sizeof(MMWave_Profile)));
    if (ptrMMWaveProfile == NULL)
    {
        /* Error: Out of memory */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOMEM, 0);
        goto exit;
    }

    /* Initialize the allocated memory: */
    memset ((void *)ptrMMWaveProfile, 0, sizeof(MMWave_Profile));

    /* Populate the profile: */
    memcpy ((void*)&ptrMMWaveProfile->profileCfg, (const void*)ptrProfileCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));
    memcpy ((void*)&ptrMMWaveProfile->profileTimeCfg, (const void*)ptrProfileTimeCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    ptrMMWaveProfile->ptrMMWaveMCB = ptrMMWaveMCB;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Add the profile to the Profile List  */
    MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList, (MMWave_ListNode*)ptrMMWaveProfile);

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Profile has been successfully registered */
    retHandle = (MMWave_ProfileHandle)ptrMMWaveProfile;

exit:
    return retHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in] ptrCalibrationData
 *      Optional pointer to the calibration data which needs to be
 *      restored.
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_openLink
(
    MMWave_MCB*                 ptrMMWaveMCB,
    int32_t*                    errCode
)
{
    int32_t                     retVal = MINUS_ONE;
    uint8_t                     tempBinIndex = 8U; /* Default Value*/

    /****************************************************************************************
     * Setup the RF Calibration Time unit:
     ****************************************************************************************/



    /* Are we supporting runtime calibration or not? */
    if(ptrMMWaveMCB->openCfg.useRunTimeCalib != 0)
    {
        /* Are we supporting custom calibration or not? */
        if (ptrMMWaveMCB->openCfg.useCustomCalibration == true)
        {
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = ptrMMWaveMCB->openCfg.customCalibrationEnableMask;
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = tempBinIndex;
            retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
        }
        else
        {
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 0xCAU;
            ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = tempBinIndex;
            retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
        }

        /* Check if run-time calibrtaion return status. */
        if ((retVal != M_DFP_RET_CODE_OK) || 
        (ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp.h_CalRunStatus != ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask) ||
        (ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp.h_CalResStatus != 
        (ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask | RFS_FACTORY_CAL_VALIDITY_STATUS)))
        {
            /* Error: Unable to set the calibration time unit */
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALPERIOD, retVal);
            retVal   = MINUS_ONE;
            goto exit;
        }
    }

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to create a mutex.
 *
 *  @param[out]  mutexHandle
 *      Handle to the mutex object which is to be created
 *  @param[in]   name
 *      Name of the mutex object
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalMutexCreate(T_DFP_OSI_MUTEX_HDL* mutexHandle, uint8_t* name)
{
    int32_t              retVal;

    retVal = SemaphoreP_constructMutex(&gMmwOsalMutex);

    if(retVal == SystemP_FAILURE)
    {
        /* Error: Unable to create the semaphore */
        retVal       = MINUS_ONE;
        *mutexHandle = NULL;
    }
    else
    {
        /* Successfully created the semaphore */
        retVal    = 0;
        *mutexHandle = (T_DFP_OSI_MUTEX_HDL*)&gMmwOsalMutex;
    }

    return (int32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the version of the various components
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deviceGetVersion(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t retVal = MINUS_ONE;

    retVal = rl_mmWaveDfpVerGet(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->version);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to get the device version */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        DebugP_logError("Error MMWave_deviceGetVersion\n");
        retVal   = MINUS_ONE;
    }

    DebugP_logInfo ("RF MMWave LIB Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_GenVerNum, ptrMMWaveMCB->version.z_MmwlLibVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_MmwlLibVersion.c_BuildVerNum);
    DebugP_logInfo ("RF MMWave LIB Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_MmwlLibVersion.c_Date, ptrMMWaveMCB->version.z_MmwlLibVersion.c_Month, ptrMMWaveMCB->version.z_MmwlLibVersion.c_Year);

    DebugP_logInfo ("RF FECSS LIB Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_FecssLibVersion.c_GenVerNum, ptrMMWaveMCB->version.z_FecssLibVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_FecssLibVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_FecssLibVersion.c_BuildVerNum);
    DebugP_logInfo ("RF FECSS LIB Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_FecssLibVersion.c_Date, ptrMMWaveMCB->version.z_FecssLibVersion.c_Month, ptrMMWaveMCB->version.z_FecssLibVersion.c_Year);

    DebugP_logInfo ("RFS ROM Version Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsRomVersion.c_GenVerNum, ptrMMWaveMCB->version.z_RfsRomVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_RfsRomVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_RfsRomVersion.c_BuildVerNum);
    DebugP_logInfo ("RFS ROM Version Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsRomVersion.c_Date, ptrMMWaveMCB->version.z_RfsRomVersion.c_Month, ptrMMWaveMCB->version.z_RfsRomVersion.c_Year);

    DebugP_logInfo ("RF RFS Patch Version    : %02d.%02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_GenVerNum, ptrMMWaveMCB->version.z_RfsPatchVersion.c_MajorVerNum,
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_MinorVerNum, ptrMMWaveMCB->version.z_RfsPatchVersion.c_BuildVerNum);
    DebugP_logInfo ("RF RFS Patch Version Date    : %02d.%02d.%02d\n",
            ptrMMWaveMCB->version.z_RfsPatchVersion.c_Date, ptrMMWaveMCB->version.z_RfsPatchVersion.c_Month, ptrMMWaveMCB->version.z_RfsPatchVersion.c_Year);

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to lock the mutex
 *
 *  @param[in]  mutexHandle
 *      Handle to the mutex object
 *  @param[in]  timeout
 *      Maximum timeout to wait for the mutex
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalMutexLock(T_DFP_OSI_MUTEX_HDL mutexHandle, uint32_t timeout)
{
    uint32_t   semTimeout;

    /* Translate the timeout from mmWave link format to the semaphore OSAL module */
    if (timeout == M_DFP_OSI_WAIT_FOREVER)
    {
        /* Semaphore timeout is set to wait forever */
        semTimeout = SystemP_WAIT_FOREVER;
    }
    else
    {
        /* Set the semaphore timeout. */
        semTimeout = timeout;
    }

    /* Pend on the semaphore: */
    SemaphoreP_pend((SemaphoreP_Object*)(mutexHandle), semTimeout);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to wait on the semaphore object.
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore
 *  @param[in]  timeout
 *      Maximum timeout to wait for the semaphore
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalSemWait(T_DFP_OSI_SEM_HDL semHandle, uint32_t timeout)
{
    uint32_t   semTimeout;

    /* Translate the timeout from mmWave link format to the semaphore OSAL module */
    if (timeout == M_DFP_OSI_WAIT_FOREVER)
    {
        /* Semaphore timeout is set to wait forever */
        semTimeout = SystemP_WAIT_FOREVER;
    }
    else
    {
        /* Set the semaphore timeout. */
        semTimeout = timeout;
    }

    /* Pend on the semaphore: */
    SemaphoreP_pend((SemaphoreP_Object*)(semHandle), semTimeout);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to signal the semaphore object
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalSemSignal(T_DFP_OSI_SEM_HDL semHandle)
{
    /* Post the semaphore */
    SemaphoreP_post((SemaphoreP_Object*)(semHandle));
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to delete the semaphore object
 *
 *  @param[in]  semHandle
 *      Handle to the semaphore
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -  <0
 */
int32_t MMWave_osalSemDelete(T_DFP_OSI_SEM_HDL semHandle)
{
    int32_t retVal = 0;

    /* Delete the semaphore: */
    SemaphoreP_destruct((SemaphoreP_Object*)(semHandle));

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered OSAL function which is used
 *      to delete the mutex
 *
 *  @param[in]  mutexHandle
 *      Handle to the mutex object
 *
 *  \ingroup MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      always returns zero
 *
 */
int32_t MMWave_osalMutexDelete(T_DFP_OSI_MUTEX_HDL mutexHandle)
{
    int32_t retVal = 0;

    /* Delete the semaphore: */
    SemaphoreP_destruct((SemaphoreP_Object*)(mutexHandle));

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to register the
 *      interrupt handler. In the case of the Mailbox the driver is responsible
 *      for the interrupt management. This function is a dummy stub
 *
 *  @param[in]  deviceIndex
 *      Device for which the interrupt is to be registered
 *  @param[in]  pHandler
 *      ISR Handler
 *  @param[in]  pValue
 *      Argument to the ISR
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Always returns 0
 */
static int32_t MMWave_registerInterruptHandler
(uint8_t deviceIndex, T_DFP_PLT_ISR_HANDLER isrHandler, \
                    void* value, T_DFP_PLT_HWI_HDL* hwiHdl)
{
    HwiP_Object HwiObj;
    HwiP_Params HwiParams;

    HwiP_Params_init(&HwiParams);
    HwiParams.intNum = (uint32_t)MMWave_ISRHandle;
    HwiParams.callback = (HwiP_FxnCallback) isrHandler;
    HwiParams.isPulse = 0;
    HwiP_construct(&HwiObj, &HwiParams);

    gMMWave_MCB.mmwavelinkInterruptFunc = isrHandler;
    *hwiHdl = (T_DFP_PLT_HWI_HDL *) &MMWave_ISRHandle;
    return 0;
}

static int32_t MMWave_deregisterInterruptHandler
(T_DFP_PLT_HWI_HDL hwiHdl)
{
    uint32_t intNum = (uint32_t) hwiHdl;
    HwiP_disableInt(intNum);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to mask the
 *      interrupts. Since the mmWave control module is using the Mailbox
 *      communication interface the driver is handling all the interrupt
 *      management. This function is a dummy stub.
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static int32_t MMWave_maskHostIRQ(T_DFP_PLT_HWI_HDL fd)
{
    uint32_t intNum = (uint32_t) fd;
    HwiP_enableInt(intNum);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the mmWave link registered callback function to unmask the
 *      interrupts to indicate that the message has been successfully handled
 *
 *  @param[in]  fd
 *      Handle to the communication interface
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static int32_t MMWave_unmaskHostIRQ(T_DFP_PLT_HWI_HDL fd)
{
    /* The Mailbox driver indicates to the remote endpoint that the message
     * have been processed. The mmWave is now capable of receiving another message */
    uint32_t intNum = (uint32_t) fd;
    HwiP_disableInt(intNum);

    return 0;
}


int32_t MMWave_factoryCalibConfig (
    MMWave_Handle    mmWaveHandle,
    MMWave_calibCfg* ptrfactoryCalibCfg, 
    int32_t*         errCode)
{
    MMWave_MCB*         ptrMMWaveMCB = NULL;
    int32_t             retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Sanity Check: Validate the arguments */
    if ((ptrfactoryCalibCfg == NULL) || (ptrMMWaveMCB == NULL))
    {
        /* Error: Invalid argument detected */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Populate the control MCB */
    memcpy ((void *)&gMMWave_MCB.factoryCalCfg, (void *)ptrfactoryCalibCfg, sizeof(MMWave_calibCfg));

    /* Do we need to restore factory calibration data? */
    if(ptrMMWaveMCB->factoryCalCfg.ptrFactoryCalibData != NULL)
    {
        retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->factoryCalCfg.ptrFactoryCalibData));
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }
    }

    if (TRUE == ptrMMWaveMCB->factoryCalCfg.isFactoryCalEnabled)
    {
        retVal = rl_fecssRfFactoryCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd, &ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp);
        if((retVal != M_DFP_RET_CODE_OK) || 
           (ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp.h_CalRunStatus != ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask) || 
           (ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalRsp.h_CalResStatus != 
           ( ptrMMWaveMCB->factoryCalCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask | RFS_FACTORY_CAL_VALIDITY_STATUS)))
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFSBOOTCAL, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }

    return retVal;
}


/**************************************************************************
 ************************ mmWave Link Functions ***************************
 **************************************************************************/

int32_t MMWave_LinkInitCommon(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal = MINUS_ONE;
    
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.h_XtalClkFreq = M_RL_FECSS_XTAL_CLK_FREQ_40M;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_ClkSourceSel = M_RL_FECSS_DEV_CLK_SRC_FCLK;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_PowerMode = M_RL_FECSS_PWR_ON_MODE_COLD;
        /* For RPMF*/
    if(ptrMMWaveMCB->initCfg.iswarmstart)
    {
      ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_PowerMode = M_RL_FECSS_PWR_ON_MODE_WARM;
    }
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_FecBootCfg = 0U;
    ptrMMWaveMCB->initCfg.fecDevPwrOnCmd.c_ChirpTimerResol = M_RL_FECSS_CHIRP_TIMER_RES_00; /* Low res */
    /* Power on the Device: */
   
    retVal = (int32_t)rl_fecssDevPwrOn(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevPwrOnCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to initialize and power on the BSS. Encode the error code to account
         * for the subsystem error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Get the version information: */
    
    retVal = MMWave_deviceGetVersion(ptrMMWaveMCB, errCode);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }

    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_DevClkCtrl = M_RL_FECSS_DEV_CLK_SRC_FCLK;
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_FtClkCtrl = M_RL_FECSS_FT_CLK_SRC_XTAL;
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_ON_CAL;

    /* FECSS Device Clock Control */
 
    retVal = rl_fecssDevClockCtrl(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }

    /*ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd.h_RxChCtrlBitMask = 0x7U;
    ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd.h_TxChCtrlBitMask = 0x03U;
    ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd.c_MiscCtrl = 1U << M_RL_RF_MISC_CTRL_RDIF_CLK; //0U;

    retVal = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }*/

#if 0
    //TODO: RFStatusGet function pointer isn't available yet on RFS
    retVal = rl_fecssRfStatusGet(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecRFStsGetRsp);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
        goto exit;
    }
#endif

#if 0
    /* Do we need to restore calibration data? */
    if(ptrMMWaveMCB->initCfg.ptrFactoryCalibData != NULL)
    {
        retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->initCfg.ptrFactoryCalibData));
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }
    }

    if (TRUE == ptrMMWaveMCB->initCfg.isFactoryCalEnabled)
    {
        retVal = rl_fecssRfFactoryCalDataSet(M_DFP_DEVICE_INDEX_0, (T_RL_API_FECSS_FACT_CAL_DATA *)(ptrMMWaveMCB->initCfg.ptrAteCalibration));
        if (retVal < 0)
        {
            /* Error: Unable to start the link; error code is already setup */
            goto exit;
        }

        retVal = rl_fecssRfFactoryCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd, &ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp);
        if((retVal != M_DFP_RET_CODE_OK) || 
           (ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp.h_CalRunStatus != ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask) || 
           (ptrMMWaveMCB->initCfg.fecRFFactoryCalRsp.h_CalResStatus != 
           ( ptrMMWaveMCB->initCfg.fecRFFactoryCalCmd.h_CalCtrlBitMask | RFS_FACTORY_CAL_VALIDITY_STATUS)))
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ERFSBOOTCAL, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }
#endif

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_initLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t                 retVal = 0;
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uint32_t                index;

    /* Initialize and setup the spawn lists */
    for (index = 0U; index < MMWAVE_MAX_NUM_SPAWN_LIST; index++)
    {
        /* Get the pointer to the spawn node */
        ptrSpawnFxnNode = &ptrMMWaveMCB->spawnTable[index];

        /* Initialize the spawn node */
        memset ((void*)ptrSpawnFxnNode, 0, sizeof(MMWave_SpawnFxnNode));

        /* Add the node to the free list: */
        MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);
    }

    /* Reset the active list: */
    ptrMMWaveMCB->ptrSpawnFxnActiveList = NULL;
    /* YES: Setup the mmWave Link */
    
    retVal = MINUS_ONE;
    /* Reset the client context: */
    memset ((void *)&ptrMMWaveMCB->fecClientCbData, 0, sizeof(T_DFP_CLIENT_CB_DATA));

    ptrMMWaveMCB->deviceMap = M_DFP_DEVICE_MAP_DEVICE_0;

    /* Device Platform related initialization */
    ptrMMWaveMCB->fecClientCbData.c_PlatformType     = M_DFP_DEVICE_PLATFORM_TYPE;
#ifdef SOC_XWRL64XX
    ptrMMWaveMCB->fecClientCbData.c_DeviceType       = M_DFP_DEVICETYPE_6432;
#else
    ptrMMWaveMCB->fecClientCbData.c_DeviceType       = M_DFP_DEVICETYPE_1432;
#endif
    ptrMMWaveMCB->fecClientCbData.c_ApiErrorCheckDis = M_FALSE;
    ptrMMWaveMCB->fecClientCbData.h_ApiRespTimeoutMs = (UINT16)2U; /* DKS update later */
    ptrMMWaveMCB->fecClientCbData.c_ApiDbglogEn      = 1U;

    /* OSI related initialization */
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexCreate = &MMWave_osalMutexCreate;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexLock   = &MMWave_osalMutexLock;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexUnLock = &MMWave_osalMutexUnlock;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Mutex.p_OsiMutexDelete = &MMWave_osalMutexDelete;

    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemCreate = &MMWave_osalSemCreate;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemWait   = &MMWave_osalSemWait;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemSignal = &MMWave_osalSemSignal;
    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Semp.p_OsiSemDelete = &MMWave_osalSemDelete;

    ptrMMWaveMCB->fecClientCbData.z_OsiCb.z_Queue.p_OsiSpawn = (void *)NULL;

    /* Platform related initialization */
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_Delay                = &MMWave_SysDelay;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_TimeStamp            = &MMWave_TimeStamp;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.w_TimeStampCounterMask = FECSS_TIMESTAMP_COUNTER_MASK;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_RegisterIsr          = &MMWave_registerInterruptHandler;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_DeRegisterIsr        = &MMWave_deregisterInterruptHandler;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_MaskIsr              = &MMWave_maskHostIRQ;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_UnMaskIsr            = &MMWave_unmaskHostIRQ;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_EnterCriticalRegion  = &MMWave_EnterCriticalRegion;
    ptrMMWaveMCB->fecClientCbData.z_PltfCb.p_ExitCriticalRegion   = &MMWave_ExitCriticalRegion;


    /* Debug related initialization */
    ptrMMWaveMCB->fecClientCbData.z_DbgCb.p_RfsdbgData  = &AppDebugData[0];
    ptrMMWaveMCB->fecClientCbData.z_DbgCb.p_Print  = NULL;

    /* Communication Interface related initialization */
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfOpen   = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfRead   = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfWrite  = M_NULL;
    ptrMMWaveMCB->fecClientCbData.z_ComIfCb.p_ComIfClose  = M_NULL;


    retVal = (int32_t)rl_mmWaveLinkInit(ptrMMWaveMCB->deviceMap, ptrMMWaveMCB->fecClientCbData);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to initialize and power on the BSS. Encode the error code to account
         * for the subsystem error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINIT, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    retVal = (int32_t)MMWave_LinkInitCommon(ptrMMWaveMCB, errCode);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        retVal   = MINUS_ONE;
        goto exit;
    }
    /* Link has been setup successfully. */
    retVal = 0;
    
exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to initialize the mmWave control module. This
 *      needs to be invoked before any other control API is invoked.
 *
 *  @param[in]  ptrCtrlInitCfg
 *      Pointer to the control init configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the control module
 *  @retval
 *      Error   -   NULL
 */
MMWave_Handle mmw_linkControlinit (MMWave_InitCfg* ptrCtrlInitCfg, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB = NULL;
    int32_t             retVal;
    int32_t             status;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if (ptrCtrlInitCfg == NULL)
    {
        /* Error: Invalid argument detected */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Initialize the memory block */
    memset ((void *)&gMMWave_MCB, 0, sizeof(MMWave_MCB));

    /* Populate the devicemap based on the device type*/
    gMMWave_MCB.deviceMap = M_DFP_DEVICE_MAP_DEVICE_0;

    /* Populate the control MCB */
    memcpy ((void *)&gMMWave_MCB.initCfg, (void *)ptrCtrlInitCfg, sizeof(MMWave_InitCfg));


    status = SemaphoreP_constructMutex(&gMMWave_MCB.cfgSemHandle);
    DebugP_assert(SystemP_SUCCESS == status);

    status = SemaphoreP_constructBinary(&(gMMWave_MCB.linkSemHandle), 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Initialize the mmWave Link module: */
    retVal = MMWave_initLink (&gMMWave_MCB, errCode);
    if (retVal < 0)
    {
        /* Error: Failure to open the mmWave Link; error code is already setup. */
        goto exit;
    }


exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        ptrMMWaveMCB = &gMMWave_MCB;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        ptrMMWaveMCB = NULL;
    }
    return (MMWave_Handle)ptrMMWaveMCB;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the chirp configuration given
 *      the chirp handle
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[out] ptrProfileCfg
 *      Pointer to the profile configuration populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getProfileCfg
(
    MMWave_ProfileHandle    profileHandle,
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG*         ptrProfileCfg,
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG*         ptrProfileTimeCfg,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (ptrProfileCfg == NULL) || (ptrProfileTimeCfg == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Copy over the configuration: */
    memcpy ((void*)ptrProfileCfg, (void*)&ptrMMWaveProfile->profileCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG));
    memcpy ((void*)ptrProfileTimeCfg, (void*)&ptrMMWaveProfile->profileTimeCfg, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of chirps attached to a profile
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[out] numChirps
 *      Number of chirps attached to a profile
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getNumChirps
(
    MMWave_ProfileHandle    profileHandle,
    uint32_t*               numChirps,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (numChirps == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Get the number of chirps: */
    *numChirps = ptrMMWaveProfile->numChirps;

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      The function can be used by the application to get the chirp handle
 *      at the specified index. If the index exceeds the number of chirps
 *      configured the function will fail with the error code.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile
 *  @param[in]  chirpIndex
 *      Chirp Index for which the handle is needed. Set to 1 to get the
 *      first chirp index etc
 *  @param[out] chirpHandle
 *      Populated chirp handle
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpHandle
(
    MMWave_ProfileHandle    profileHandle,
    uint32_t                chirpIndex,
    MMWave_ChirpHandle*     chirpHandle,
    int32_t*                errCode
)
{
    MMWave_Profile*     ptrMMWaveProfile;
    MMWave_Chirp*       ptrMMWaveChirp;
    uint32_t            index  = 1U;
    int32_t             retVal = MINUS_ONE;
    int32_t             endProcessing = 0;

    /* Initialize the error code: */
    *errCode     = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (chirpHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Profile: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Profiles are always linked to the control module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Initialize the chirp handle */
    *chirpHandle = NULL;

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Get the head of the chirp list: */
    ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    while (endProcessing == 0)
    {
        /* Have we reached the end of the list? */
        if (ptrMMWaveChirp == NULL)
        {
            /* YES: Control comes here indicates that the chirp index specified exceeds the
             * configured number of chirps. We are done with the processing */
            *errCode      = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
            endProcessing = 1;
        }
        else
        {
            /* Is this what we are looking for? */
            if (index == chirpIndex)
            {
                /* YES: Setup the chirp handle. */
                *chirpHandle  = (MMWave_ChirpHandle)ptrMMWaveChirp;
                retVal        = 0;
                endProcessing = 1;
            }

            /* Get the next element: */
            index = index + 1U;
            ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveChirp);
        }
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle));

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the chirp configuration given
 *      the chirp handle
 *
 *  @param[in]  chirpHandle
 *      Handle to the chirp
 *  @param[out] ptrChirpCfg
 *      Pointer to the chirp configuration populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getChirpCfg
(
    MMWave_ChirpHandle  chirpHandle,
    T_RL_API_SENS_PER_CHIRP_CFG*       ptrChirpCfg,
    T_RL_API_SENS_PER_CHIRP_CTRL*      ptrChirpCtrl,
    int32_t*            errCode
)
{
    MMWave_Chirp*   ptrMMWaveChirp;
    int32_t         retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((chirpHandle == NULL) || (ptrChirpCfg == NULL) || (ptrChirpCtrl == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the Chirp: */
    ptrMMWaveChirp = (MMWave_Chirp*)chirpHandle;

    /* Chirps are always linked to profiles and profiles to the mmWave control module. */
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile != NULL);
    DebugP_assert (ptrMMWaveChirp->ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Copy over the configuration: */
    memcpy ((void*)ptrChirpCfg, (void*)&ptrMMWaveChirp->chirpCfg, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));
    memcpy ((void*)ptrChirpCtrl, (void*)&ptrMMWaveChirp->chirpCtrl, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the specified profile
 *      and corresponding chirp configuration. The following order is preserved in
 *      the function:
 *          - Profile configuration
 *          - Chirp configuration
 *
 *  @param[in]  ptrControlCfg
 *      Pointer to the control config
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_configureProfileChirp(MMWave_CtrlCfg * ptrControlCfg, int32_t* errCode)
{

    MMWave_ProfileHandle  * ptrProfileHandle;
    int32_t                 retVal;
    int32_t                 index;
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfg;
    MMWave_ChirpHandle      chirpHandle;
    T_RL_API_SENS_PER_CHIRP_CFG            chirpCfg;
    T_RL_API_SENS_PER_CHIRP_CTRL           chirpCtrl;
    uint32_t                numChirps;
    uint32_t                chirpIndex;
    uint32_t                u32DevIdx;

    /* Loop across all devices to send profile and chirp configurations */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {

        /* Get the first profile handler of the selected device */
        ptrProfileHandle = &ptrControlCfg->frameCfg[u32DevIdx].profileHandle[0];

        /* Cycle through all the profile(s) which have been specified. */
        for (index = 0; index < MMWAVE_MAX_PROFILE; index++)
        {
            /* Do we have a valid profile? */
            if (ptrProfileHandle[index] == NULL)
            {
                /* NO: Skip to the next profile */
                continue;
            }

            /* YES: Get the profile configuration */
            if (MMWave_getProfileCfg(ptrProfileHandle[index], &profileCfg, &profileTimeCfg, errCode) < 0)
            {
                /* Error: Unable to get the profile configuration. Setup the return value */
                retVal = MINUS_ONE;
                goto end;
            }

            /* Configure the profile using the mmWave Link API */
            retVal = rl_sensChirpProfComnCfg(u32DevIdx, &profileCfg);

            if (retVal != M_DFP_RET_CODE_OK)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }

            /* Configure the profile using the mmWave Link API */
            retVal = rl_sensChirpProfTimeCfg(u32DevIdx, &profileTimeCfg);

            if (retVal != M_DFP_RET_CODE_OK)
            {
                *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EPROFILECFG, retVal);
                retVal   = MINUS_ONE;
                goto end;
            }

            /* Get the number of chirps configured and attached to the profile: */
            if (MMWave_getNumChirps(ptrProfileHandle[index], &numChirps, errCode) < 0)
            {
                /* Error: Unable to get the number of chirps. Error code is already setup */
                retVal = MINUS_ONE;
                goto end;
            }

            /* For the profile; Cycle through all the chirps and configure them. */
            for (chirpIndex = 1U; chirpIndex <= numChirps; chirpIndex++)
            {
                /* Get the Chirp Handle associated at the specified index */
                if (MMWave_getChirpHandle(ptrProfileHandle[index], chirpIndex, &chirpHandle, errCode) < 0)
                {
                    /* Error: Unable to get the chirp handle. Error code is already setup */
                    retVal = MINUS_ONE;
                    goto end;
                }

                /* Get the chirp configuration: */
                if (MMWave_getChirpCfg(chirpHandle, &chirpCfg, &chirpCtrl, errCode) < 0)
                {
                    /* Error: Unable to get the chirp configuration. Error code is already setup */
                    retVal = MINUS_ONE;
                    goto end;
                }

            }
        }
    }

    /* Control comes here implies that the profile & chirp was configured successfully */
    retVal = 0;

end:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the specified frame
 *      configuration.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control config
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_configureFrame(MMWave_MCB * ptrMMWaveMCB, MMWave_CtrlCfg * ptrControlCfg, int32_t * errCode)
{
    MMWave_ProfileHandle  * ptrProfileHandle;
    T_RL_API_SENS_FRAME_CFG          * pstFrameCfg;
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          stProfileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          stProfileTimeCfg;
    int32_t  retVal;
    uint32_t u32ProfIdx;
    uint32_t u32DevIdx;

    /* Get the number of ADC samples (it is expected that all profiles across
     *   all devices have the same number) */
    ptrProfileHandle = &ptrControlCfg->frameCfg[0].profileHandle[0];

    for(u32ProfIdx = 0U; u32ProfIdx < MMWAVE_MAX_PROFILE; u32ProfIdx++)
    {
        /* Check if this index holds a profile handle */
        if(ptrProfileHandle[u32ProfIdx] != NULL)
        {
            /* Get the profile configuration */
            MMWave_getProfileCfg(ptrProfileHandle[u32ProfIdx], &stProfileCfg, &stProfileTimeCfg, errCode);

            break;
        }
    }

    /* Loop across all devices to send profile and chirp configurations */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {
        /* Get the frame configuration of the device */
        pstFrameCfg = &ptrControlCfg->frameCfg[u32DevIdx].frameCfg;

        retVal = rl_sensFrameCfg(u32DevIdx, pstFrameCfg);
        if (retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError(MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
            retVal   = MINUS_ONE;
            break;
        }
    }

   return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the mmWave link with the supplied
 *      configuration
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_configLink
(
    MMWave_MCB*         ptrMMWaveMCB,
    MMWave_CtrlCfg*     ptrControlCfg,
    int32_t*            errCode
)
{
    int32_t retVal;

    /**************************************************************************
     * Frame Mode:
     * Order of operations as specified by the mmWave Link are
     *  - Profile configuration
     *  - Chirp configuration
     *  - Frame configuration
     **************************************************************************/
    retVal = MMWave_configureProfileChirp(ptrControlCfg, errCode);

    if(retVal < 0)
    {
        goto end;
    }

    /* Set the frame configuration */
    retVal = MMWave_configureFrame(ptrMMWaveMCB, ptrControlCfg, errCode);
    if(retVal < 0)
    {
        goto end;
    }

    /* Set the frame configuration: */
    DebugP_logInfo ("rlSetFrameConfig...\n");
    retVal = rl_sensFrameCfg(M_DFP_DEVICE_INDEX_0, &ptrControlCfg->frameCfg[0].frameCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rlSetFrameConfig error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

//#ifdef DFP_LOOPBACK_ENABLED
#if 0
    T_RL_API_SENS_LOOPBACK_CFG sensLoopbackCfg;
    sensLoopbackCfg.c_LbFreqSel = 20U; /* 5MHz */
    sensLoopbackCfg.c_IfaLbGainIndex = 0xB2U; /* 0dB */
    sensLoopbackCfg.c_LoPaLbCmnGainIndex = 48U; /* 0dB */
    sensLoopbackCfg.c_LoLbGainIndex = 96U; /* 0dB */
    sensLoopbackCfg.c_PaLbGainIndex = 48U; /* 0dB */
    sensLoopbackCfg.h_ExtLbTxBpmEnSel = 3U; /* External LB settings */
    retVal = rl_sensLoopBackCfg(M_DFP_DEVICE_INDEX_0, &sensLoopbackCfg);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rl_loopbackCfg error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

    T_RL_API_SENS_LOOPBACK_ENA   sensLoopbackEna;
    /* RFS RFS sens LB ena api - DIG  LB */
    sensLoopbackEna.c_LbEnable = 0x1U;
    retVal = rl_sensLoopBackEna(M_DFP_DEVICE_INDEX_0, &sensLoopbackEna);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Setting the frame configuration failed */
        DebugP_logError("rl_loopbackEna error %d\n",retVal);

        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EFRAMECFG, retVal);
        retVal   = MINUS_ONE;
        goto end;
    }

#endif

    /* Set the return value to be success. */
    retVal = 0;

end:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is the exported API which is used to configure the mmWave.
 *      This is only applicable in the full configuration mode.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrControlCfg
 *      Pointer to the control configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_config (MMWave_Handle mmWaveHandle, MMWave_CtrlCfg* ptrControlCfg, int32_t* errCode)
{
    MMWave_MCB* ptrMMWaveMCB;
    int32_t     retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrControlCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/

    /* Full Configuration Mode: Ensure that the application has opened the mmWave module. */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == 0U)
    {
        /* Error: Invalid usage the module should be synchronized before it can be started. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Configure the link: */
    if (MMWave_configLink (ptrMMWaveMCB, ptrControlCfg, errCode) < 0)
    {
        /* Error: Unable to configure the link; error code is already setup. */
        goto exit;
    }

    /* The module has been configured successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_CONFIGURED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      This function Initialize and setup the mmWave Control module
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t mmw_link_Initialization(bool iswarmstrt)
{
    int32_t             errCode;
    int32_t             retVal = SystemP_SUCCESS;
    MMWave_InitCfg      initCfg;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Is Warm Start? */
    initCfg.iswarmstart = iswarmstrt;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = mmw_linkControlinit (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        CLI_write ("Error: mmWave Control Initialization failed [Error code %d] [errorLevel %d] [mmWaveErrorCode %d] [subsysErrorCode %d]\n", errCode, errorLevel, mmWaveErrorCode, subsysErrorCode);
        retVal = SystemP_FAILURE;
    }
    /* FECSS RF Power ON*/
    if(initCfg.iswarmstart)
    {
        errCode = rl_fecssRfPwrOnOff(M_DFP_DEVICE_INDEX_0, &gMmwMssMCB.channelCfg);
        if(errCode != M_DFP_RET_CODE_OK)
        {
            CLI_write ("Error: FECSS RF PowerON failed [Error code %d] \r\n", errCode);
            retVal = SystemP_FAILURE;
        }   
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to open the mmWave module.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *  @param[in] ptrCalibrationData
 *      This is the pointer to the calibration data which needs to be passed
 *      to the RadarSS to bypass calibration and restore it to a previously saved
 *      value. Set this to NULL and the mmWave module will request the RadarSS to
 *      perform the calibration. Note this structure consists of multiple sub-calibration
 *      structures.
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_open
(
    MMWave_Handle               mmWaveHandle,
    const MMWave_OpenCfg*       ptrOpenCfg,
    int32_t*                    errCode
)
{
    int32_t         retVal;
    MMWave_MCB*     ptrMMWaveMCB;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    if (ptrOpenCfg->fecRDIFCtrlCmd.c_RdifEnable == M_RL_FECSS_RDIF_ENA)
    {
        retVal = rl_fecssDevRdifCtrl(M_DFP_DEVICE_INDEX_0, &ptrOpenCfg->fecRDIFCtrlCmd);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
            retVal = MINUS_ONE;
            goto exit;
        }
    }

    /* Sanity Check: Validate the arguments */
    if ((ptrMMWaveMCB == NULL) || (ptrOpenCfg == NULL))
    {
        /* Error: Invalid arguments. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    /* Sanity Check: Ensure that the module is not already open */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == MMWAVE_STATUS_OPENED)
    {
        /* Error: Invalid usage the module should be closed before it can be opened */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Copy over the configuration: */
    memcpy ((void *)&ptrMMWaveMCB->openCfg, (const void*)ptrOpenCfg, sizeof(MMWave_OpenCfg));
    ptrMMWaveMCB->openCfg.useRunTimeCalib = true;
    ptrMMWaveMCB->openCfg.useCustomCalibration = false;
    ptrMMWaveMCB->openCfg.customCalibrationEnableMask = 0U;
    ptrMMWaveMCB->openCfg.fecRDIFCtrlCmd.c_RdifEnable = M_RL_FECSS_RDIF_DIS;
    ptrMMWaveMCB->openCfg.fecRDIFCtrlCmd.h_RdifSampleCount = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples;
    /* Open the mmWave Link: */
    retVal = MMWave_openLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: mmWave Open failed; error code is already setup  */
        goto exit;
    }

    /* The module has been opened successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_OPENED;

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization.
 *      User need to fill gMmwMssMCB.mmwOpenCfg before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_openSensor(void)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;

    /**********************************************************
     **********************************************************/
    /* Open mmWave module, this is only done once */

    /* Open the mmWave module: */
    if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmwOpenCfg, &errCode) < 0)
    {
        /* Error: decode and Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        DebugP_log ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                        mmWaveErrorCode, subsysErrorCode);
        return -1;
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      The function is used to close the mmWave Link module.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_closeLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal = MINUS_ONE;
    uint32_t u32DevIdx;

    /* Link is not operational: */
    for(u32DevIdx = 0U; u32DevIdx < MMWAVE_NBR_DEVICES; u32DevIdx++)
    {
        //TODO:
#if 0
        ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_OFF;
        retVal = rl_fecssRfPwrOnOff(u32DevIdx, &ptrMMWaveMCB->initCfg.fecRFPwrCfgCmd);
        if(retVal != M_DFP_RET_CODE_OK)
        {
            *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
            retVal = MINUS_ONE;
        }
#endif
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the mmWave Link
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MMWave_deinitMMWaveLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t     retVal;
    T_RL_API_FECSS_DEV_PWR_OFF_CMD devPwrOffCmd;

    /* For RPMF*/
    ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd.c_ApllClkCtrl = M_RL_FECSS_APLL_CTRL_OFF;
    retVal = rl_fecssDevClockCtrl(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->initCfg.fecDevClkCtrlCmd);
    if(retVal != M_DFP_RET_CODE_OK)
    {
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EVERSION, retVal);
        retVal = MINUS_ONE;
    }

    /* Power off the Device: */
    devPwrOffCmd.c_RetentionMode = M_RL_FECSS_PWR_DOWN_RET_ON;
    retVal = (int32_t)rl_fecssDevPwrOff(M_DFP_DEVICE_INDEX_0, &devPwrOffCmd);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to power off the BSS */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EDEINIT, retVal);
        retVal   = MINUS_ONE;
    }

    retVal = rl_mmWaveLinkDeInit(ptrMMWaveMCB->deviceMap);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to power off the BSS */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EDEINIT, retVal);
        retVal   = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deinitLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t                 retVal = 0;
    MMWave_SpawnFxnNode*    ptrSpawnFxnNode;
    uintptr_t               key;

    /* Deinitialize the mmWave Link: */
    retVal = MMWave_deinitMMWaveLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to deinitialize the mmWave link; error code is already setup */
        goto exit;
    }

    /* Cycle through and cleanup the active spawn lists: There might be some entries in
     * the Active list which still need to be handled but because we are shutting down
     * the module we simply flush out the entries. */
    key = HwiP_disable();
    ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnActiveList);
    while (ptrSpawnFxnNode != NULL)
    {
        /* Add this back to the free list: */
        MMWave_listAdd ((MMWave_ListNode**)&ptrMMWaveMCB->ptrSpawnFxnFreeList, (MMWave_ListNode*)ptrSpawnFxnNode);

        /* Get the next entry from the active list: */
        ptrSpawnFxnNode = (MMWave_SpawnFxnNode*)MMWave_listRemove ((MMWave_ListNode**)&gMMWave_MCB.ptrSpawnFxnActiveList);
    }
    HwiP_restore (key);

    /* Control comes here implies that the deinitialization of the module was successful. */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to deinitialize and shutdown the mmWave module
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_deinit (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Deinitialize the link: */
    retVal = MMWave_deinitLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Link deinitialization failed. Error code is already setup */
        goto exit;
    }

    /* SOC was deinitialized: Delete the semaphore handle */
    SemaphoreP_destruct(&(ptrMMWaveMCB->linkSemHandle));

    /* Delete the configuration semaphore handle (if available) */
    SemaphoreP_destruct(&(ptrMMWaveMCB->cfgSemHandle));

    /* Reset the memory: */
    memset ((void *)ptrMMWaveMCB, 0, sizeof(MMWave_MCB));

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      The function is used to close the mmWave module.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_close (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    int32_t         retVal;
    MMWave_MCB*     ptrMMWaveMCB;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the mmWave module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Sanity Check: Validate the arguments */
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid arguments. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Sanity Check: Ensure that the module is not already closed */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED) == 0U)
    {
        /* Error: Invalid usage the module should be opened before it can be closed */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }


    /* The module has been closed successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status & (~(uint32_t)MMWAVE_STATUS_OPENED | MMWAVE_STATUS_CONFIGURED);

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the number of profiles which have been added.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[out] numProfiles
 *      Number of added profiles populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getNumProfiles(MMWave_Handle mmWaveHandle, uint32_t* numProfiles, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB;
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (numProfiles == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the pointer to the mmWave control module: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Initialize the number of profiles */
    *numProfiles = 0U;

    /* Cycle through the profile list */
    ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    while (ptrMMWaveProfile != NULL)
    {
        /* Increment the number of profiles */
        *numProfiles = *numProfiles + 1U;
        ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveProfile);
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to add the chirp to an existing profile.
 *
 *  @param[in]  profileHandle
 *      Handle to the profile to which the chirp is to be added
 *  @param[in]  ptrChirpCfg
 *      Pointer to the chirp configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   Handle to the chirp
 *  @retval
 *      Error   -   NULL
 */
MMWave_ChirpHandle MMWave_addChirp
(
    MMWave_ProfileHandle    profileHandle,
    const T_RL_API_SENS_PER_CHIRP_CFG*     ptrChirpCfg,
    const T_RL_API_SENS_PER_CHIRP_CTRL*    ptrChirpCtrl,
    int32_t*                errCode
)
{
    MMWave_Chirp*           ptrMMWaveChirp;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_ChirpHandle      retHandle = NULL;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((profileHandle == NULL) || (ptrChirpCfg == NULL) || (ptrChirpCtrl == NULL))
    {
        /* Error: Invalid argument */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Get the profile handle: */
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Sanity Check: Each profile is linked to the mmWave module */
    DebugP_assert (ptrMMWaveProfile->ptrMMWaveMCB != NULL);

    /* Allocate memory for the chirp: */
    ptrMMWaveChirp = (MMWave_Chirp*) HeapP_alloc(&gMmwHeapObj, (sizeof(MMWave_Chirp)));
    if (ptrMMWaveChirp == NULL)
    {
        /* Error: Out of memory */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOMEM, 0);
        goto exit;
    }

    /* Initialize the allocated memory for the chirp: */
    memset ((void*)ptrMMWaveChirp, 0, sizeof(MMWave_Chirp));

    /* Populate the Chirp: */
    memcpy ((void *)&ptrMMWaveChirp->chirpCfg, (const void*)ptrChirpCfg, sizeof(T_RL_API_SENS_PER_CHIRP_CFG));
    memcpy ((void *)&ptrMMWaveChirp->chirpCtrl, (const void*)ptrChirpCtrl, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL));
    ptrMMWaveChirp->ptrMMWaveProfile = ptrMMWaveProfile;

    /* Critical Section Enter: Protect the 'Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Add the chirp to the profile list */
    MMWave_listCat ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList, (MMWave_ListNode**)&ptrMMWaveChirp);

    /* Increment the number of chirps which are linked to the profile: */
    ptrMMWaveProfile->numChirps++;

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveProfile->ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return handle: */
    retHandle = (MMWave_ChirpHandle)ptrMMWaveChirp;

exit:
    return retHandle;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the profile handle for the specific profile
 *      identifier.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  profileId
 *      Profile Id
 *  @param[out] profileHandle
 *      Handle to the profile populated by the API
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_getProfileHandle
(
    MMWave_Handle           mmWaveHandle,
    uint8_t                 profileId,
    MMWave_ProfileHandle*   profileHandle,
    int32_t*                errCode
)
{
    MMWave_MCB*         ptrMMWaveMCB;
    MMWave_Profile*     ptrMMWaveProfile;
    int32_t             retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (profileHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Setup the pointers: */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;

    /* Critical Section Enter: Protect the 'Profile List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Cycle through the profile list */
    ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList);
    while (ptrMMWaveProfile != NULL)
    {
        /* Is this what we are looking for? */
        if (profileId == ptrMMWaveProfile->profileId)
        {
            /* YES: Setup the profile handle */
            *profileHandle = (MMWave_ProfileHandle)ptrMMWaveProfile;
            retVal = 0;
            break;
        }

        /* Get the next element: */
        ptrMMWaveProfile = (MMWave_Profile*)MMWave_listGetNext ((MMWave_ListNode*)ptrMMWaveProfile);
    }

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Did we find a match? */
    if (ptrMMWaveProfile == NULL)
    {
        /* Error: No matching profile identifier found. Setup the error code. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ENOTFOUND, 0);
    }

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is used to populate the default control configuration
 *      in chirp configuration mode
 *
 *  @param[out]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void Mmwave_populateDefaultChirpControlCfg (MMWave_CtrlCfg* ptrCtrlCfg)
{
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG      profileCfg;
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG      profileTimeCfg;
    T_RL_API_SENS_PER_CHIRP_CFG        chirpCfg;
    T_RL_API_SENS_PER_CHIRP_CTRL       chirpCtrl;
    int32_t             errCode;
    MMWave_ChirpHandle  chirpHandle;

    MmwDemo_ADCBufConfig(gMmwMssMCB.channelCfg.h_RxChCtrlBitMask, (gMmwMssMCB.profileComCfg.h_NumOfAdcSamples *2));

#if 0
    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX0EN, 1U);
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1, regVal);

    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX1EN, 1U);
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1, regVal);

    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1_ADCBUFCFG1_RX2EN, 1U);
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG1, regVal);

    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX0, 0x000U); //TODO: ADC RX0 OFFSET
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2, regVal);

    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2_ADCBUFCFG2_ADCBUFADDRX1, (256/16)); //TODO: ADC RX1 OFFSET
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG2, regVal);

    regVal = HW_RD_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG3);
    HW_SET_FIELD32(regVal, CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG3_ADCBUFCFG3_ADCBUFADDRX2, (512/16)); //TODO: ADC RX2 OFFSET
    HW_WR_REG32(CSL_APP_HWA_ADCBUF_CTRL_U_BASE + CSL_APP_HWA_ADCBUF_CTRL_ADCBUFCFG3, regVal);
#endif

    /* Initialize the control configuration: */
    memset ((void*)ptrCtrlCfg, 0, sizeof(MMWave_CtrlCfg));

    /* Populate the profile configuration: */
    Mmwave_populateDefaultProfileCfg (&profileCfg, &profileTimeCfg);

    /* Create the profile: */
    ptrCtrlCfg->frameCfg[0].profileHandle[0] = MMWave_addProfile (gMmwMssMCB.ctrlHandle, &profileCfg, &profileTimeCfg, &errCode);
    if (ptrCtrlCfg->frameCfg[0].profileHandle[0] == NULL)
    {
        DebugP_logError ("Error: Unable to add the profile [Error code %d]\n", errCode);
        DebugP_log ("MMWave Add Profile Error\r\n");
        return;
    }

    /**************************************************************************************************
     * Unit Test: Verify the Full Configuration Profile API
     **************************************************************************************************/
    {
        T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileCfgTmp;
        T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfgTmp;
        uint32_t                numProfiles;
        MMWave_ProfileHandle    tmpProfileHandle;

        /* Verify the number of profiles */
        if (MMWave_getNumProfiles (gMmwMssMCB.ctrlHandle, &numProfiles, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the number of profiles [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Number Profile Error\r\n");
            return;
        }
        if (numProfiles != 1U)
        {
            DebugP_logError ("Error: Invalid number of profiles detected [%d]\r\n", numProfiles);
            DebugP_log ("MMWave Get Number Profile Error\r\n");
        }

        /* Get the profile handle: */
        if (MMWave_getProfileHandle (gMmwMssMCB.ctrlHandle, 0U, &tmpProfileHandle, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile handle [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Profile Handle Error\r\n");
        }
        if (tmpProfileHandle != ptrCtrlCfg->frameCfg[0].profileHandle[0])
        {
            DebugP_logError ("Error: Invalid profile handle detected\r\n");
            DebugP_log ("MMWave Get Profile Handle Error\r\n");
        }

        /* Get the profile configuration */
        if (MMWave_getProfileCfg (ptrCtrlCfg->frameCfg[0].profileHandle[0], &profileCfgTmp, &profileTimeCfgTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile configuration [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Profile Error\r\n");
            return;
        }
        if (memcmp ((void*)&profileCfg, (void*)&profileCfgTmp, sizeof(T_RL_API_SENS_CHIRP_PROF_COMN_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid profile configuration detected\r\n");
            DebugP_log ("MMWave Get Profile Error\r\n");
            return;
        }

        if (memcmp ((void*)&profileTimeCfg, (void*)&profileTimeCfgTmp, sizeof(T_RL_API_SENS_CHIRP_PROF_TIME_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid profile time configuration detected\r\n");
            DebugP_log ("MMWave Get Profile Error\r\n");
            return;
        }
    }

    /* Populate the default chirp configuration */
    Mmwave_populateDefaultChirpCfg (&chirpCfg, &chirpCtrl);

    /* Add the chirp to the profile: */
    chirpHandle = MMWave_addChirp (ptrCtrlCfg->frameCfg[0].profileHandle[0], &chirpCfg, &chirpCtrl, &errCode);
    if (chirpHandle == NULL)
    {
        DebugP_logError ("Error: Unable to add the chirp [Error code %d]\r\n", errCode);
        DebugP_log ("MMWave Add Chirp Error\r\n");
        return;
    }


    /**************************************************************************************************
     * Unit Test: Verify the Full Configuration Chirp API
     **************************************************************************************************/
    {
        T_RL_API_SENS_PER_CHIRP_CFG        chirpCfgTmp;
        T_RL_API_SENS_PER_CHIRP_CTRL       chirpCtrlTmp;
        uint32_t            numChirps;
        MMWave_ChirpHandle  chirpHandleTmp;

        /* Get the number of chirps attached to the profile */
        if (MMWave_getNumChirps (ptrCtrlCfg->frameCfg[0].profileHandle[0], &numChirps, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the number of chirps [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Number of Chirps Error\r\n");
            return;
        }
        if (numChirps != 1U)
        {
            DebugP_logError ("Error: Invalid number of chirps detected [%d]\r\n", numChirps);
            DebugP_log ("MMWave Get Number of Chirps Error\r\n");
            return;
        }

        /* Get the Chirp Handle */
        if (MMWave_getChirpHandle (ptrCtrlCfg->frameCfg[0].profileHandle[0], 1U, &chirpHandleTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the chirp handle [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Chirp Handle Error\r\n");
            return;
        }
        if (chirpHandleTmp != chirpHandle)
        {
            DebugP_logError ("Error: Chirp handle validation failed [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Chirp Handle Error\r\n");
            return;
        }

        /* Get the chirp configuration */
        if (MMWave_getChirpCfg (chirpHandle, &chirpCfgTmp, &chirpCtrlTmp, &errCode) < 0)
        {
            DebugP_logError ("Error: Unable to get the profile configuration [Error code %d]\r\n", errCode);
            DebugP_log ("MMWave Get Chirp Error\r\n");
            return;
        }
        if (memcmp ((void*)&chirpCfg, (void*)&chirpCfgTmp, sizeof(T_RL_API_SENS_PER_CHIRP_CFG)) != 0)
        {
            DebugP_logError ("Error: Invalid chirp configuration detected\r\n");
            DebugP_log ("MMWave Get Chirp Configuration Error\r\n");
            return;
        }

        if (memcmp ((void*)&chirpCtrl, (void*)&chirpCtrlTmp, sizeof(T_RL_API_SENS_PER_CHIRP_CTRL)) != 0)
        {
            DebugP_logError ("Error: Invalid chirp configuration detected\r\n");
            DebugP_log ("MMWave Get Chirp Configuration Error\r\n");
            return;
        }
    }

    /* Populate the frame configuration: */
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfChirpsInBurst = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst; //2; //10; //2U;
    ptrCtrlCfg->frameCfg[0].frameCfg.c_NumOfChirpsAccum = gMmwMssMCB.frameCfg.c_NumOfChirpsAccum; //0U;
   //ptrCtrlCfg->frameCfg[0].frameCfg.w_BurstPeriodicity = 2500U; /* 8 chirps = 40us + 10us idle */
    ptrCtrlCfg->frameCfg[0].frameCfg.w_BurstPeriodicity = gMmwMssMCB.frameCfg.w_BurstPeriodicity; //1698; //12000; //3480U; /* 4 chirps = 148us + 200us idle , (12 + 25) chirp*/
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfBurstsInFrame = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame; //64; //2; //32U;
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FramePeriodicity = gMmwMssMCB.frameCfg.w_FramePeriodicity; //10000000; //120000; //29440U; /* 2 bursts = 696us + 40us idle, 40M XTAL */
    ptrCtrlCfg->frameCfg[0].frameCfg.h_NumOfFrames = gMmwMssMCB.frameCfg.h_NumOfFrames; //0; //10; //50U;
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FrameEvent0TimeCfg = 0;

    /*\
        (ptrCtrlCfg->frameCfg[0].frameCfg.w_FramePeriodicity - 240U); frame period - 6us */
    ptrCtrlCfg->frameCfg[0].frameCfg.w_FrameEvent1TimeCfg = 0;

    return;
}

/**
 *  @b Description
 *  @n
 *      MMW demo helper Function to configure sensor. User need to fill gMmwMssMCB.mmwCtrlCfg and
 *      add profiles/chirp to mmWave before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_configSensor(void)
{
    int32_t     errCode = 0;

    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmwCtrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        DebugP_log ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\r\n",
                        mmWaveErrorCode, subsysErrorCode);
        goto exit;
    }
exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave link. This function can only be
 *      invoked once the configuration has been completed successfully.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_startLink (MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{

    int32_t  retVal;
    T_RL_API_SENSOR_START_CMD sensStartCmd;

    ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 206U;
    ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = 8U; /* mid temp bin DKS */

    retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to set the calibration time unit */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALPERIOD, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }

    /* Sensor frame trigger cfg api */
    sensStartCmd.c_FrameTrigMode = M_RL_SENS_FRAME_SW_TRIG; /* SW trigger */
    sensStartCmd.c_ChirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS;
    sensStartCmd.c_FrameLivMonEn = 0x3U; /* SYNTH, RX_SAT enabled - DKS GPADC pending */
    sensStartCmd.w_FrameTrigTimerVal = 0U; /* Read timer and write DKS */
    retVal = rl_sensSensorStart(M_DFP_DEVICE_INDEX_0, &sensStartCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Starting the sensor failed */
        * errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ESENSOR, retVal);

        retVal = MINUS_ONE;
        goto exit;
    }

    /* Control comes here indicates that the sensor has been started successfully */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the mmWave link.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the control MCB
 *  @param[out] errCode
 *      Error code populated by the API on an error
 *
 *  \ingroup  MMWAVE_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_stopLink (const MMWave_MCB* ptrMMWaveMCB, int32_t* errCode)
{
    int32_t         retVal;
    T_RL_API_SENSOR_STOP_CMD    sensStopCmd;
    /* Are we operating in Chirp or Continuous mode? */
    /******************************************************************************
     * CHIRP or ADVANCED: Stop the sensor
     ******************************************************************************/
    /* Stop Master device */
    sensStopCmd.c_FrameStopMode = M_RL_SENS_STOP_FRAME;
    retVal = rl_sensSensorStop(M_DFP_DEVICE_INDEX_0, &sensStopCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* All other are treated as FATAL error */
        * errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ESENSOR, retVal);
        retVal    = MINUS_ONE;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to start the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_init
 *  @pre
 *      MMWave_sync
 *  @pre
 *      MMWave_open
 *  @pre
 *      MMWave_config (Only in full configuration mode)
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_start (MMWave_Handle mmWaveHandle, const MMWave_CalibrationCfg* ptrCalibrationCfg, int32_t* errCode)
{
    MMWave_MCB*     ptrMMWaveMCB;
    int32_t         retVal =1;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if ((ptrMMWaveMCB == NULL) || (ptrCalibrationCfg == NULL))
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    /* Full Configuration Mode: Ensure that the application has configured the mmWave module
     * Only then can we start the module. */
    if (((ptrMMWaveMCB->status & MMWAVE_STATUS_OPENED)       == 0U)    ||
            ((ptrMMWaveMCB->status & MMWAVE_STATUS_CONFIGURED)   == 0U))
    {
        /* Error: Invalid usage the module should be synchronized before it can be started. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Sanity Check: Ensure that the module has not already been started */
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == MMWAVE_STATUS_STARTED)
    {
        /* Error: Invalid usage the module should be stopped before it can be started again. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Start the mmWave link: */
    T_RL_API_SENSOR_START_CMD sensStartCmd;
    ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.h_CalCtrlBitMask = 206U;
    ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd.c_TempBinIndex = 8U;
    retVal = rl_fecssRfRuntimeCal(M_DFP_DEVICE_INDEX_0, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalCmd, &ptrMMWaveMCB->openCfg.fecRFRunTimeCalRsp);
    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Unable to set the calibration time unit */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ECALPERIOD, retVal);
        retVal   = MINUS_ONE;
        goto exit;
    }
    
    /* Sensor frame trigger cfg api */
    sensStartCmd.c_FrameTrigMode = M_RL_SENS_FRAME_SW_TRIG; /* SW trigger */
    sensStartCmd.c_ChirpStartSigLbEn = M_RL_SENS_CT_START_SIG_LB_DIS;
    sensStartCmd.c_FrameLivMonEn = 0x3U; /* SYNTH, RX_SAT enabled - DKS GPADC pending */
    sensStartCmd.w_FrameTrigTimerVal = 0U; /* Read timer and write DKS */
    retVal = rl_sensSensorStart(M_DFP_DEVICE_INDEX_0, &sensStartCmd);

    if (retVal != M_DFP_RET_CODE_OK)
    {
        /* Error: Starting the sensor failed */
        * errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_ESENSOR, retVal);

        retVal = MINUS_ONE;
        goto exit;
    }

    /* Control comes here indicates that the sensor has been started successfully */
    retVal = 0;

    /* The module has been started successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status | MMWAVE_STATUS_STARTED;

    exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to stop the mmWave control module after the
 *      configuration has been applied.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave control module
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @pre
 *      MMWave_start
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_stop (MMWave_Handle mmWaveHandle, int32_t* errCode)
{
    MMWave_MCB*         ptrMMWaveMCB;
    int32_t             retVal;

    /* Initialize the error code: */
    *errCode = 0;

    /* Get the pointer to the control module */
    ptrMMWaveMCB = (MMWave_MCB*)mmWaveHandle;
    if (ptrMMWaveMCB == NULL)
    {
        /* Error: Invalid argument. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /****************************************************************************************
     * Sanity Check:
     *  - Validate the prerequisites
     ****************************************************************************************/
    if ((ptrMMWaveMCB->status & MMWAVE_STATUS_STARTED) == 0U)
    {
        /* Error: Invalid usage the module should be started before it can be stopped. */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Stop the mmWave link */
    retVal = MMWave_stopLink (ptrMMWaveMCB, errCode);
    if (retVal < 0)
    {
        /* Error: Unable to stop the link; error code is setup already */
        goto exit;
    }

    /* The module has been stopped successfully: */
    ptrMMWaveMCB->status = ptrMMWaveMCB->status & (~(uint32_t)MMWAVE_STATUS_STARTED);

exit:
    /* Determing the error level from the error code? */
    if (MMWave_decodeErrorLevel (*errCode) == MMWave_ErrorLevel_SUCCESS)
    {
        /* Success: Setup the return value */
        retVal = 0;
    }
    else
    {
        /* Informational/Error: Setup the return value */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/* Enable Continuous wave mode */
#define CONTINUOS_WAVE_MODE_ENABLE   0
/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t ripple_link_startSensor(void)
{
#if !(CONTINUOS_WAVE_MODE_ENABLE) /* suppress warning */
    int32_t     errCode;
#endif
    MMWave_CalibrationCfg   calibrationCfg;


    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
    calibrationCfg.chirpCalibrationCfg.enableCalibration    = false;
    calibrationCfg.chirpCalibrationCfg.enablePeriodicity    = false;
    calibrationCfg.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    DebugP_logInfo("App: MMWave_start Issued\n");
#if !(CONTINUOS_WAVE_MODE_ENABLE) /* disable mmWave_start for continousMode CW */
    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to start the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        DebugP_log ("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
        /* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
           assert here and re-start from the beginning. For now, choosing the latter path */
        MmwDemo_debugAssert(0);
        return -1;
    }
#endif
    return 0;
}


/* This function is called to transfer RAW ADC data via SPI*/
void mmw_SPIWrite (SPI_Handle handle,uint8_t *payload,uint32_t payloadLength)
{   
    
    MCSPI_Transaction trans;
    
    MCSPI_Transaction_init(&trans);
    
    trans.channel  = gConfigMcspi0ChCfg[0].chNum;
    trans.dataSize = 32;
    trans.csDisable = TRUE;
    trans.count    = payloadLength/ (trans.dataSize/8);
    trans.txBuf    = (void *)payload;
    trans.args     = NULL;
    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &trans);
   
}



/**
 *  @b Description
 *  @n
 *      The function is called to remove the specified node from the list.
 *
 *  @param[in]  ptr_list
 *      This is the pointer to the list from where node will be removed.
 *  @param[in]  ptr_remove
 *      This is the node which is to be removed.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   -1
 */
int32_t MMWave_listRemoveNode (MMWave_ListNode** ptr_list, MMWave_ListNode* ptr_remove)
{
    MMWave_ListNode*    ptr_next;
    MMWave_ListNode*    ptr_prev;
    int32_t         retVal = 0;

    /* Are there any nodes in the list? */
    if (*ptr_list != NULL)
    {
        /* YES: Are we removing the head? */
        if (ptr_remove == *ptr_list)
        {
            /* Use the other API to acheive the needful. */
            MMWave_listRemove(ptr_list);
        }
        else
        {
            /* OK; we are trying to remove a non head element; so lets get the
             * previous and next pointer of the elements that needs to be removed. */
            ptr_prev = ptr_remove->p_prev;
            ptr_next = ptr_remove->p_next;

            /* Kill the Links for element that is being removed. */
            ptr_remove->p_prev = NULL;
            ptr_remove->p_next = NULL;

            /* Are we removing the last element */
            if (ptr_next == NULL)
            {
                /* The last element points to nothing. */
                ptr_prev->p_next = NULL;
            }
            else
            {
                /* We are trying to remove an element in the middle of the list. */
                ptr_prev->p_next = ptr_next;
                ptr_next->p_prev = ptr_prev;
            }
        }
    }
    else
    {
        /* No: The list is empty. */
        retVal = MINUS_ONE;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This is an *internal* function which is used to delete the profile
 *      The function does not hold the critical section.
 *
 *  @param[in]  ptrMMWaveMCB
 *      Pointer to the mmWave control module
 *  @param[in]  ptrMMWaveProfile
 *      Pointer to the profile to be deleted
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_internalDelProfile (MMWave_MCB* ptrMMWaveMCB, MMWave_Profile* ptrMMWaveProfile)
{
    /* Remove the profile from the Profile List  */
    MMWave_listRemoveNode ((MMWave_ListNode**)&ptrMMWaveMCB->ptrProfileList, (MMWave_ListNode*)ptrMMWaveProfile);

    /* Cleanup the profile memory: */
    HeapP_free(&gMmwHeapObj, (void *)ptrMMWaveProfile);
    return;
}
/**
 *  @b Description
 *  @n
 *      This is an *internal* function which is used to delete the chirp
 *      associated with a profile. The function does not hold the critical
 *      section.
 *
 *  @param[in]  ptrMMWaveProfile
 *      Pointer to the profile
 *  @param[in]  ptrMMWaveChirp
 *      Pointer to the chirp to be deleted
 *
 *  \ingroup MMWAVE_CTRL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void MMWave_internalDelChirp
(
    MMWave_Profile* ptrMMWaveProfile,
    MMWave_Chirp*   ptrMMWaveChirp
)
{
    /* Remove the chirp from the profile. */
    MMWave_listRemoveNode ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList, (MMWave_ListNode*)ptrMMWaveChirp);

    /* Decrement the number of chirps which are linked to the profile: */
    ptrMMWaveProfile->numChirps--;

    /* Cleanup the chirp memory: */
    HeapP_free(&gMmwHeapObj, (void *)ptrMMWaveChirp);
}

/**
 *  @b Description
 *  @n
 *      The function is used to delete the profile. This will also delete all
 *      the chirps which are still attached to the profile.
 *
 *  @param[in]  mmWaveHandle
 *      Handle to the mmWave module
 *  @param[in]  profileHandle
 *      Handle to the profile to be deleted
 *  @param[out] errCode
 *      Encoded Error code populated by the API on an error
 *
 *  \ingroup MMWAVE_CTRL_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MMWave_delProfile
(
    MMWave_Handle           mmWaveHandle,
    MMWave_ProfileHandle    profileHandle,
    int32_t*                errCode
)
{
    MMWave_MCB*             ptrMMWaveMCB;
    MMWave_Profile*         ptrMMWaveProfile;
    MMWave_Chirp*           ptrMMWaveChirp;
    int32_t                 retVal = MINUS_ONE;

    /* Initialize the error code: */
    *errCode = 0;

    /* Sanity Check: Validate the arguments */
    if ((mmWaveHandle == NULL) || (profileHandle == NULL))
    {
        /* Error: Invalid arguments */
        *errCode = MMWave_encodeError (MMWave_ErrorLevel_ERROR, MMWAVE_EINVAL, 0);
        goto exit;
    }

    /* Setup the pointers: */
    ptrMMWaveMCB     = (MMWave_MCB*)mmWaveHandle;
    ptrMMWaveProfile = (MMWave_Profile*)profileHandle;

    /* Critical Section Enter: Protect the 'Profile & Chirp List' */
    SemaphoreP_pend (&(ptrMMWaveMCB->cfgSemHandle), SystemP_WAIT_FOREVER);

    /* Cycle through all the registered chirps: */
    ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    while (ptrMMWaveChirp != NULL)
    {
        /* Delete the chirp: Use the internal API since the semaphore is already held. */
        MMWave_internalDelChirp (ptrMMWaveProfile, ptrMMWaveChirp);

        /* Cycle through the list again and get the new head. */
        ptrMMWaveChirp = (MMWave_Chirp*)MMWave_listGetHead ((MMWave_ListNode**)&ptrMMWaveProfile->ptrChirpList);
    }

    /* Delete the profile: */
    MMWave_internalDelProfile (ptrMMWaveMCB, ptrMMWaveProfile);

    /* Critical Section Exit: */
    SemaphoreP_post (&(ptrMMWaveMCB->cfgSemHandle));

    /* Setup the return value: */
    retVal = 0;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *    Utility function that frees DMA resource
 *
 */
void DPEDMA_freeEDMAChannel(EDMA_Handle handle,
    uint32_t *dmaCh,
    uint32_t *tcc,
    uint32_t *param,
    uint32_t *shadowParam
)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;

    baseAddr = EDMA_getBaseAddr(handle);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(handle);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        *dmaCh, EDMA_TRIG_MODE_MANUAL, *tcc, 0);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(handle, dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    *dmaCh  = 0;

    //testStatus = EDMA_freeTcc(handle, tcc);
    //DebugP_assert(testStatus == SystemP_SUCCESS);
    //*tcc  = 0;

    testStatus = EDMA_freeParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    *param  = 0;

    testStatus = EDMA_freeParam(handle, shadowParam);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    *shadowParam  = 0;

    return;
}
// Free all the allocated EDMA channels
void mmwDemo_freeDmaChannels(EDMA_Handle edmaHandle)
{
    uint32_t   index;
    uint32_t  dmaCh, tcc, pram, shadow;
    for(index = 0; index < 64; index++)
    {
        dmaCh = index;
        tcc = index;
        pram = index;
        shadow = index;
        DPEDMA_freeEDMAChannel(edmaHandle, &dmaCh, &tcc, &pram, &shadow);
    }
    for(index = 0; index < 128; index++)
    {
        shadow = index;
        DebugP_assert(EDMA_freeParam(edmaHandle, &shadow) == SystemP_SUCCESS);
    }
    return;
}