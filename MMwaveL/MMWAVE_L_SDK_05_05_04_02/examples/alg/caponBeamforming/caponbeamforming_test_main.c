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

/**
 *  This example performs 1D Capon Beamforming tests for 6 instances one after the other.
 *  There are 2 test files:
 *      1. caponDataGenerated6ant128snaps32angles6inst_50deg.bin
 *      2. caponDataGenerated4ant64snaps64angles6inst_-30deg.bin
 *  Both the files have 6 intances of Capon Beamforming
 * 
 * The data is read in the L3 memory from a the input files.
 * The data is read in the M4 memory from the input files.
 * The Capon Spectrum Output is stored in the structure: 
 * caponBeamformingCfg->hwRes.caponSpectrum
 * 
 * After the FFT is completed, data validation is performed by comparing
 * After the algorithm is complete, data validation is performed by comparing
 * source and destination memory, against the reference output. If the 
 * equality test is successful, the test was successful.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>
#include <drivers/edma.h>
#include <drivers/hwa.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <datapath/dpedma/v0/dpedmahwa.h>
#include <datapath/dpedma/v0/dpedma.h>
#include <alg/caponBeamforming/caponBeamforming.h>
#include <utils/mathutils/mathutils.h>
#include <drivers/hw_include/cslr_adcbuf.h>
#include <kernel/dpl/CycleCounterP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
#define MAX_NUM_SNAPSHOTS           (128U)

#define DPC_OBJDET_EDMA_SHADOW_BASE                                             SOC_EDMA_NUM_DMACH

#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_CH_CORRELATION                   EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ10
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_CH_CAPONSPECTRUM                 EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SHADOW_CORRELATION               (DPC_OBJDET_EDMA_SHADOW_BASE + 0)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SHADOW_CAPONSPECTRUM             (DPC_OBJDET_EDMA_SHADOW_BASE + 1)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_EVENT_QUE_CORRELATION            0
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_EVENT_QUE_CAPONSPECTRUM          0

#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_CH_CORRELATION               EDMA_APPSS_TPCC_B_EVT_FREE_0
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_SHADOW_CORRELATION           (DPC_OBJDET_EDMA_SHADOW_BASE + 2)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_CH_CAPONSPECTRUM             EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_SHADOW_CAPONSPECTRUM         (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_EVENT_QUE_CORRELATION        0
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_EVENT_QUE_CAPONSPECTRUM      0

#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_CH                  EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ0
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_SHADOW              (DPC_OBJDET_EDMA_SHADOW_BASE + 4)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_EVENT_QUE           0

#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_CH                EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_SHADOW            (DPC_OBJDET_EDMA_SHADOW_BASE + 5)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_EVENT_QUE         0

#define MMW_DEMO_TEST_CAPON_INPUT_SIZE                                          MAX_NUM_SNAPSHOTS * MAX_NUM_TX_ANTENNA * MAX_NUM_RX_ANTENNA

/*! Input data to the Capon DPU */
int16_t caponInputData[2*MMW_DEMO_TEST_CAPON_INPUT_SIZE] __attribute__((section(".data"), aligned(32)));

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the 1D Capon Beamforming DPU configurations
*/
void Test_setProfile(CaponBeamformingHWA_Config *caponBeamformingCfg, Edma_IntrObject *intrObjCaponSpectrum, Edma_IntrObject *intrObjCorrelationMatrix, int16_t *dpuConfigurations)
{   
    /* HWA configurations, not related to per test, common to all test
       Setting up the start indices for the two uses of HWA */
    caponBeamformingCfg->hwRes.hwaCfg.corrMatParamSetIdx = 0;

    caponBeamformingCfg->hwRes.hwaCfg.caponSpectrumParamSetStartIdx = 11;

    /* edma configuration */
    caponBeamformingCfg->hwRes.edmaCfg.edmaHandle  = gEdmaHandle[0];  

    /* input data to Capon Beamforming */
    caponBeamformingCfg->hwRes.caponBeamformingInputData = caponInputData;    

    /* Set inputs parameters to the Capon Beamforming Algorithm */
    caponBeamformingCfg->staticCfg.numTxAntennas        = dpuConfigurations[0];
    caponBeamformingCfg->staticCfg.numRxAntennas        = dpuConfigurations[1];
    caponBeamformingCfg->staticCfg.numSnapshots         = dpuConfigurations[2];
    caponBeamformingCfg->staticCfg.numAnglesToSample    = dpuConfigurations[3];
    DebugP_assert(caponBeamformingCfg->staticCfg.numAnglesToSample == 16 || caponBeamformingCfg->staticCfg.numAnglesToSample == 32 || caponBeamformingCfg->staticCfg.numAnglesToSample == 64);
    
    caponBeamformingCfg->staticCfg.numVirtualAntennas = caponBeamformingCfg->staticCfg.numRxAntennas * caponBeamformingCfg->staticCfg.numTxAntennas;
    caponBeamformingCfg->staticCfg.internalRAMAddress = 0x55018000;

    /* Data Input EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.correlationMatrixChannel.channel          = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_CH_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.channel              = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_CH_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.correlationMatrixChannel.channelShadow    = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SHADOW_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.channelShadow        = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SHADOW_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.correlationMatrixChannel.eventQueue       = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_EVENT_QUE_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.eventQueue           = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_EVENT_QUE_CAPONSPECTRUM;

    /* Hot signature EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.correlationMatrixChannel.channel          = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_CH_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.channel              = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_CH_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.correlationMatrixChannel.channelShadow    = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_SHADOW_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.channelShadow        = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_SHADOW_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.correlationMatrixChannel.eventQueue       = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_EVENT_QUE_CORRELATION;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.eventQueue           = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAIN_SIG_EVENT_QUE_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.intrObjCorrelationMatrix                                     = intrObjCorrelationMatrix;
    caponBeamformingCfg->hwRes.intrObjCaponSpectrum                                         = intrObjCaponSpectrum;

    /* Data Output EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.correlationMatrixChannel.channel         = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_CH;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channel             = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_CH;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.correlationMatrixChannel.channelShadow   = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_SHADOW;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channelShadow       = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_SHADOW;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.correlationMatrixChannel.eventQueue      = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CORRELATION_EVENT_QUE;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.eventQueue          = DPC_OBJDET_DPU_CAPONBEAMFORMING_EDMAOUT_CAPONSPECTRUM_EVENT_QUE;
}

/**
*  @b Description
*  @n
*    Function to initialize the HWA used in this test
*/
HWA_Handle caponBeamformingTest_hwaInit()
{
    HWA_Handle gHwaHandle = NULL;
    int32_t status = SystemP_SUCCESS;
    
    /* Open HWA driver */
    gHwaHandle = HWA_open(0, NULL, &status);
    if(gHwaHandle == NULL)
    {
        DebugP_log("Error: Unable to open HWA instance. Error: %d\n", status);
        DebugP_assert(gHwaHandle == NULL);
    }
    return gHwaHandle;
}

/**
*  @b Description
*  @n
*    Function to initialize Capon Beamforming
*/
CaponBeamformingHWA_Handle caponBeamformingTest_Init(HWA_Handle gHwaHandle)
{
    int32_t  errorCode = 0;
    CaponBeamformingHWA_InitParams initParams;
    CaponBeamformingHWA_Handle  caponBeamformingHandle = NULL;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    caponBeamformingHandle =  CaponBeamformingHWA_init(&initParams, &errorCode);
    if (caponBeamformingHandle == NULL)
    {
        DebugP_log ("Debug: Capon Beamforming initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return caponBeamformingHandle;
    }
    return caponBeamformingHandle;
}

/**
*  @b Description
*  @n
*    main function from which Capon Beamforming test-case begins
*/
void caponbeamforming_test_main(void *args)
{
    FILE                            *fileId;
    int32_t                         retVal = 0;
    uint16_t                        counter = 0;
    CaponBeamformingHWA_Handle      caponBeamformingHandle = NULL;
    CaponBeamformingHWA_Config      caponBeamformingCfg;
    HWA_Handle                      gHwaHandle = NULL;
    Edma_IntrObject                 intrObjCaponSpectrum, intrObjCorrelationMatrix;
    volatile uint32_t               startCycle, cycleCount;
    int16_t                         dpuConfigurations[6];   //[N_tx N_rx N_snaps N_anglesToSample N_instances test_angle]
    int16_t                         i;
    
    Drivers_open();         //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_TPCCA_INIT|SOC_RCM_MEMINIT_TPCCB_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);

    /* Initialize the HWA for Capon Beamforming unit-testing */
    gHwaHandle = caponBeamformingTest_hwaInit();
    /* Initialize the Capon Beamforming for unit-testing */
    caponBeamformingHandle = caponBeamformingTest_Init(gHwaHandle);

    /* RUNNING THE FIRST INPUT FILE */
    /* Input file */
    fileId = fopen("..\\..\\..\\testbench\\caponDataGenerated4ant64snaps64angles6inst_-30deg.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open the .bin file!\n");
        exit(0);
    }   

    fread((int16_t *)&dpuConfigurations[0], sizeof(int16_t), 3*sizeof(int16_t), fileId);

    /* set the current Capon Beamforming parameters */
    Test_setProfile(&caponBeamformingCfg, &intrObjCaponSpectrum, &intrObjCorrelationMatrix, &dpuConfigurations[0]);
    
    /* Configure the Capon Beamforming DPU according to the parameters set */
    retVal = CaponBeamformingHWA_config(caponBeamformingHandle, &caponBeamformingCfg);
    if (retVal !=0)
    {
        DebugP_log("Could not configure Capon Beamforming. Stopping");
        return;
    }

    for(i=0;i<dpuConfigurations[4];i++)
    {
        uint8_t j;
        FILE    *fileIDAns;
        char    fileName[100];
        int32_t caponSpectrumTruth[64];
        
        counter = 0;
        //Read the data
        for(j=0;j<caponBeamformingCfg.staticCfg.numVirtualAntennas/2;j++)
        {
            fread(((int16_t *)&caponInputData[0]+counter*2*caponBeamformingCfg.staticCfg.numSnapshots*sizeof(int16_t)), sizeof(int16_t), 2*caponBeamformingCfg.staticCfg.numSnapshots*sizeof(int16_t), fileId);
            counter++;
            DebugP_log("Reading Data From File.. %d percent\n", (counter) * 100 / (caponBeamformingCfg.staticCfg.numVirtualAntennas/2));
        }

        startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
        /* Call the compute function */
        retVal = CaponBeamformingHWA_process(caponBeamformingHandle, &caponBeamformingCfg);
        if(retVal < 0)
        {
            DebugP_log("DEBUG: Capon Beamforming DPU process return error:%d \n", retVal);
            DebugP_assert(0);
        }
        cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
        DebugP_log("Measured processing time (Capon Beamforming) = CPU cycles = %d !!!\r\n", cycleCount);

        sprintf(fileName, "..\\..\\..\\testbench\\caponSpectrum%dant%dsnaps%dangles%dinst_%ddeg.bin", dpuConfigurations[0]*dpuConfigurations[1], dpuConfigurations[2], dpuConfigurations[3], i+1, dpuConfigurations[5]);
        fileIDAns = fopen(fileName, "rb");
        if (fileIDAns == NULL)
        {
            printf("Error:  Cannot open the .bin file!\n");
            exit(0);
        } 
        fread((int32_t *)&caponSpectrumTruth[0], sizeof(int32_t), caponBeamformingCfg.staticCfg.numAnglesToSample, fileIDAns);
        fclose(fileIDAns);

        for(j=0;j<caponBeamformingCfg.staticCfg.numAnglesToSample;j++)
        {
            if(caponSpectrumTruth[j] != caponBeamformingCfg.hwRes.caponSpectrum[j])
            {
                DebugP_log("Test case %d failed!\n\r", i+1);
                goto exit;
            }
        }
        DebugP_log("Test case %d passed!\n\r", i + 1);
    }
    DebugP_log("All test cases passed!\n\r");

    exit:
        fclose(fileId);
        Board_driversClose();
        Drivers_close();
        return;
}
