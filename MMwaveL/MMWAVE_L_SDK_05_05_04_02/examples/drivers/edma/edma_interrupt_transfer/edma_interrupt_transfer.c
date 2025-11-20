/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/**
 *  This example performs EDMA transfer test using interrupt mode.
 *
 * The source memory is initialized with sample data and destination memory
 * is initialized with zeroes for validation. Cache write back is done to
 * ensure that initialized data from cache is written to memory.
 *
 * The PaRAM set is initialized with proper configuration and assigned to a
 * channel for transfer. The transfer done is A Synchronized and so, BCNT*CCNT
 * triggers are needed for complete transfer.
 *
 * Intermediate and final transfer interrupts are enabled and the 
 * transfer completion interrupt status is polled to determine 
 * if the status is set before giving next trigger.
 *
 * Data validation is performed by comparing source and destination memory. 
 * If the equality test is successful, the test was successful.
 *
 */

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/edma.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Value for A count*/
#define EDMA_TEST_A_COUNT           (16U)
/* Value for B count */
#define EDMA_TEST_B_COUNT           (4U)
/* Value for C count */
#define EDMA_TEST_C_COUNT           (2U)
/* Event queue to be used  */
#define EDMA_TEST_EVT_QUEUE_NO      (0U)
/**
 * Buffers (src and dest) are needed for mem-2-mem data transfers.
 * This define is for the MAXIMUM size and hence the maximum data
 * which could be transferred using the sample test cases below.
 */
#define EDMA_TEST_BUFFER_SIZE             (EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT)

// -- void EnableInterrupts(void);
void ungate_shared_mem(void); /*TO DO: cleanup Move this fn to soc.c */

/* TODO: cleanup */
/* The source buffer used for transfer */
//static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
//static uint8_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Object gEdmaTestDoneSem;

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args);

void edma_interrupt_transfer(void *args)
{
    uint32_t            baseAddr, regionId;
    int32_t             status = SystemP_SUCCESS;
    uint32_t            loopCnt = 0;
    /* TODO: cleanup */
    uint8_t            *srcBuffPtr, *dstBuffPtr;
    EDMACCPaRAMEntry   edmaParam;
    Edma_IntrObject     intrObj;
    uint32_t            dmaCh, tcc, param;

    /* Open drivers to open the UART driver for console */
    Drivers_open(); //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();

    DebugP_log("[EDMA] Interrupt Transfer Test Started...\r\n");
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(status == SystemP_SUCCESS);

    tcc = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);

    param = EDMA_RESOURCE_ALLOC_ANY;
    status = EDMA_allocParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    ungate_shared_mem(); /* TODO: cleanup Move this fn to soc.c */
    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
     /* TODO: cleanup */
    srcBuffPtr = (uint8_t *) (CSL_APP_RAM_U_BASE + 0x20000);
    dstBuffPtr = (uint8_t *) (CSL_APP_RAM_U_BASE + 0x21000);
    for(loopCnt = 0U; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
       srcBuffPtr[loopCnt] = (uint8_t)loopCnt;
       dstBuffPtr[loopCnt] = 0;
    }
    CacheP_wb((void *)srcBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    CacheP_wb((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, EDMA_TEST_EVT_QUEUE_NO);

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam);
    edmaParam.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam.linkAddr      = (0xFFFFU);
    edmaParam.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMASetPaRAM(baseAddr, param, &edmaParam);

    status = SemaphoreP_constructBinary(&gEdmaTestDoneSem, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Register interrupt */
    intrObj.tccNum = tcc;
    intrObj.cbFxn  = &EDMA_regionIsrFxn;
    intrObj.appData = (void *) &gEdmaTestDoneSem;
    status = EDMA_registerIntr(gEdmaHandle[0], &intrObj);
    DebugP_assert(status == SystemP_SUCCESS);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMAEnableTransferRegion(
            baseAddr, regionId, dmaCh, EDMA_TRIG_MODE_MANUAL);

        SemaphoreP_pend(&gEdmaTestDoneSem, SystemP_WAIT_FOREVER);
    }

    /* Invalidate destination buffer and compare with src buffer */
    /* TODO: cleanup */
    //CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);
    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
       if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
       {
           DebugP_log("Error matching value at src and dst address %d\r\n", loopCnt);
           status = SystemP_FAILURE;
           break;
       }
    }
    DebugP_assert(status == SystemP_SUCCESS);

    status = EDMA_unregisterIntr(gEdmaHandle[0], &intrObj);
    SemaphoreP_destruct(&gEdmaTestDoneSem);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Free channel */
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
        dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    status = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeTcc(gEdmaHandle[0], &tcc);
    DebugP_assert(status == SystemP_SUCCESS);
    status = EDMA_freeParam(gEdmaHandle[0], &param);
    DebugP_assert(status == SystemP_SUCCESS);

    if(status == SystemP_SUCCESS)
    {
        DebugP_log("[EDMA] Interrupt Transfer Test Completed!!\r\n");
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
    return;
}

static void EDMA_regionIsrFxn(Edma_IntrHandle intrHandle, void *args)
{
    SemaphoreP_Object *semObjPtr = (SemaphoreP_Object *)args;
    DebugP_assert(semObjPtr != NULL);
    SemaphoreP_post(semObjPtr);
}

/*TO DO: cleanup Move this fn to soc.c */
void ungate_shared_mem(void)
{
//    APPSS_CTRL_REG->APPSS_SHARED_MEM_CLK_GATE = 0x0;
    HW_WR_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE, (uint32_t) 0x0U);
   // APPSS_CTRL->APPSS_SHARED_MEM_CLK_GATE.APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE = 0x0;
   // APPSS_CTRL->APPSS_SHARED_MEM_CLK_GATE.APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE = 0x0;
    HW_WR_REG32((0x5200002C), 0x0); // FEC_CTRL:FECSS_SHARED_MEM_CLK_GATE
    HW_WR_REG32((0x5A040500), 0x707); //TOP_PRCM:HWA_PD_MEM_SHARE_REG - extend CM3, CM4 code memory
    HW_WR_REG32(CSL_APP_CTRL_U_BASE + CSL_APP_CTRL_APPSS_SHARED_MEM_CLK_GATE, (uint32_t) 0x3U);
   // APPSS_CTRL->APPSS_SHARED_MEM_CLK_GATE.APPSS_SHARED_MEM_CLK_GATE_MEM0_APP_ENABLE = 0x1;
   // APPSS_CTRL->APPSS_SHARED_MEM_CLK_GATE.APPSS_SHARED_MEM_CLK_GATE_MEM0_HWA_ENABLE = 0x1;
    HW_WR_REG32((0x5200002C), 0x3);
}
