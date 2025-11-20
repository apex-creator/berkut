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
 * This example performs EDMA transfer test using chain mode.
 *
 * In chain transfer mode completion of one transfer automatically sets the event for another channel.
 * The PaRAM set is initialized with two channels and provides a synchronization
 * event (or trigger) to the chained DMA channel, as soon as the transfer (final or intermediate)
 * completes on the main DMA/QDMA channel.This example uses AB Synchronized transfer mode.
 *
 * The source memory is initialized with sample data and destination memory
 * is initialized with zeroes for validation.
 *
 * When a chained completion code is detected, the value of which is dictated by the 
 * transfer completion code of the PaRAM set associated with the channel,
 * Intermediate and final transfer interrupts are enabled and the transfer
 * completion interrupt status is polled to be set before giving next trigger.
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

void ungate_shared_mem(void); /*TO DO: cleanup Move this fn to soc.c */


uint8_t            *srcBuffPtr, *dstBuffPtr;
EDMACCPaRAMEntry   edmaParam1, edmaParam2;

/* The source buffer used for transfer */
//static uint8_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));
/* The destination buffer used for transfer */
//static uint8_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));

void edma_chain_transfer(void *args)
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            dmaCh0, tcc0, param0, dmaCh1, tcc1, param1;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();
	
	DebugP_log("[EDMA] Chain Transfer Test Started...\r\n");

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[0]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    dmaCh0 = EDMA_RESOURCE_ALLOC_ANY;
    //dmaCh0 = (1U);
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc0 = EDMA_RESOURCE_ALLOC_ANY;
    //tcc0 = (1U);
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param0 = EDMA_RESOURCE_ALLOC_ANY;
    //param0 = (1U);
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    dmaCh1 = EDMA_RESOURCE_ALLOC_ANY;
    //dmaCh1 = (1U);
    testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    tcc1 = EDMA_RESOURCE_ALLOC_ANY;
    //tcc1 = (1U);
    testStatus = EDMA_allocTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    param1 = EDMA_RESOURCE_ALLOC_ANY;
    //param1 = (1U);
    testStatus = EDMA_allocParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    ungate_shared_mem(); /* TODO: cleanup Move this fn to soc.c */

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    //srcBuffPtr = (uint8_t *) gEdmaTestSrcBuff;
    //dstBuffPtr = (uint8_t *) gEdmaTestDstBuff;

    uint32_t chainOptions = (EDMA_OPT_TCCHEN_MASK |
                             EDMA_OPT_ITCCHEN_MASK |
                             EDMA_OPT_TCINTEN_MASK |
                             EDMA_OPT_ITCINTEN_MASK);

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
         dmaCh0, tcc0, param0, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMADisableEvtIntrRegion(baseAddr, regionId, dmaCh0);

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) (srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) (dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT / 2;
    edmaParam1.bCntReload    = (uint16_t) 0;
    edmaParam1.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc0) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMASetPaRAM(baseAddr, param0, &edmaParam1);

    /* Request channel */
    EDMAConfigureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh1, tcc1, param1, EDMA_TEST_EVT_QUEUE_NO);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMADisableEvtIntrRegion(baseAddr, regionId, dmaCh1);

    /* Program Param Set */
    EDMACCPaRAMEntry_init(&edmaParam2);
    edmaParam2.srcAddr       = (uint32_t) (srcBuffPtr)+(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT);
    edmaParam2.destAddr      = (uint32_t) (dstBuffPtr)+(EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT);
    edmaParam2.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam2.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam2.cCnt          = (uint16_t) EDMA_TEST_C_COUNT / 2;
    edmaParam2.bCntReload    = (uint16_t) 0;
    edmaParam2.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam2.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam2.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.destCIdx      = (int16_t) EDMA_TEST_A_COUNT * EDMA_TEST_B_COUNT;
    edmaParam2.linkAddr      = 0xFFFFU;
    edmaParam2.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK | EDMA_OPT_SYNCDIM_MASK |
         ((((uint32_t)tcc1) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMASetPaRAM(baseAddr, param1, &edmaParam2);

    /*Set params for chain transfer. Chain two channels*/
    EDMAChainChannel(baseAddr, param0, dmaCh1, chainOptions);

    /*
     * Transfer is done in AB sync mode
     */
    EDMAEnableTransferRegion(baseAddr, regionId, dmaCh0,
            EDMA_TRIG_MODE_MANUAL);

    while(EDMAReadIntrStatusRegion(baseAddr, regionId, tcc1) != 1);

    EDMAClrIntrRegion(baseAddr, regionId, tcc1);

    /* Free channel */
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh0, EDMA_TRIG_MODE_MANUAL, tcc0, EDMA_TEST_EVT_QUEUE_NO);

    /* Free channel */
    EDMAFreeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh1, EDMA_TRIG_MODE_EVENT, tcc1, EDMA_TEST_EVT_QUEUE_NO);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param0);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[0], &dmaCh1);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[0], &tcc1);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[0], &param1);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /* Invalidate destination buffer and compare with src buffer */
    CacheP_inv((void *)dstBuffPtr, EDMA_TEST_BUFFER_SIZE, CacheP_TYPE_ALL);

    for(loopCnt = 0; loopCnt < EDMA_TEST_BUFFER_SIZE; loopCnt++)
    {
        if(srcBuffPtr[loopCnt] != dstBuffPtr[loopCnt])
        {
            testStatus = SystemP_FAILURE;
            if(testStatus==SystemP_SUCCESS)
            break;
        }
    }
	
    if(testStatus == SystemP_SUCCESS)
    {
        DebugP_log("[EDMA] Chain Transfer Test Completed!!\r\n");
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