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
 *  This example performs 2D Capon Beamforming tests for 64 instances one after the other.
 * 
 * The data is read in the M4 memory from the input files.
 * The Capon Spectrum Output is stored in the array: 
 * caponSpectrum
 * 
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
#include <alg/caponBeamforming2D/caponBeamforming2D.h>
#include <utils/mathutils/mathutils.h>
#include <drivers/hw_include/cslr_adcbuf.h>
#include <kernel/dpl/CycleCounterP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
#define MAX_NUM_SNAPSHOTS           (36U)
#define MAX_RANGE_BINS_SKIP         (2U)

#define DPC_OBJDET_EDMA_SHADOW_BASE                                             SOC_EDMA_NUM_DMACH

#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_CH_CAPONSPECTRUM                 EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ11
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SHADOW_CAPONSPECTRUM             (DPC_OBJDET_EDMA_SHADOW_BASE + 1)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_EVENT_QUE_CAPONSPECTRUM          0

#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_CH_CAPONSPECTRUM             EDMA_APPSS_TPCC_B_EVT_FREE_1
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_SHADOW_CAPONSPECTRUM         (DPC_OBJDET_EDMA_SHADOW_BASE + 3)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_EVENT_QUE_CAPONSPECTRUM      0

#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_CH                EDMA_APPSS_TPCC_B_EVT_HWA_DMA_REQ1
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_SHADOW            (DPC_OBJDET_EDMA_SHADOW_BASE + 5)
#define DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_EVENT_QUE         0

#define MMW_DEMO_TEST_CAPON_INPUT_SIZE                                          MAX_NUM_SNAPSHOTS * MAX_NUM_TX_ANTENNA * MAX_NUM_RX_ANTENNA

/*! Input data to the Capon DPU */
int32_t caponInputData[2*MMW_DEMO_TEST_CAPON_INPUT_SIZE] __attribute__((section(".data"), aligned(32)));
float   caponSpectrum[MAX_ANGLES_AZIMUTH*MAX_ANGLES_ELEVATION*MAX_RANGE_BINS_SKIP] __attribute__((section(".data"), aligned(32)));

/**
*  @b Description
*  @n
*    Based on the test configuration, set up the 2D Capon Beamforming DPU configurations
*/
void Test_setProfile(CaponBeamforming2DHWA_Config *caponBeamformingCfg, int32_t *dpuConfigurations)
{   
    /* HWA configurations, not related to per test, common to all test */
    
    /* Setting up the paramset start index for the use of HWA */
    caponBeamformingCfg->hwRes.hwaCfg.caponBeamforming2DParamSetStartIdx = 11;

    /* Setting up the DMA channel which will trigger the HWA on DMA trigger (value <= 15) */
    caponBeamformingCfg->hwRes.hwaDmaTriggerSourceCaponSpectrum = 11;
    DebugP_assert(caponBeamformingCfg->hwRes.hwaDmaTriggerSourceCaponSpectrum < 16);

    /* Setting up the bCnt value to skip arrays in between successive elevation bins */
    caponBeamformingCfg->hwRes.bCnt = 2;

    /* edma configuration */
    caponBeamformingCfg->hwRes.edmaCfg.edmaHandle  = gEdmaHandle[0];  

    /* input data to Capon Beamforming 2D */
    caponBeamformingCfg->hwRes.caponBeamforming2DInputData = caponInputData;    
    
    caponBeamformingCfg->hwRes.caponSpectrum = caponSpectrum;    

    /* Set inputs parameters to the Capon Beamforming 2D Algorithm */
    caponBeamformingCfg->staticCfg.numVirtualAntennas           = dpuConfigurations[0];
    caponBeamformingCfg->staticCfg.numSnapshots                 = dpuConfigurations[1];
    caponBeamformingCfg->staticCfg.numAnglesToSampleAzimuth     = dpuConfigurations[2];
    caponBeamformingCfg->staticCfg.numAnglesToSampleElevation   = dpuConfigurations[3];
    DebugP_assert(caponBeamformingCfg->staticCfg.numAnglesToSampleAzimuth == 16 || caponBeamformingCfg->staticCfg.numAnglesToSampleAzimuth == 32 || caponBeamformingCfg->staticCfg.numAnglesToSampleAzimuth == 64);
    DebugP_assert(caponBeamformingCfg->staticCfg.numAnglesToSampleElevation == 8 || caponBeamformingCfg->staticCfg.numAnglesToSampleElevation == 16 || caponBeamformingCfg->staticCfg.numAnglesToSampleElevation == 32);
    for(uint8_t i=0;i<caponBeamformingCfg->staticCfg.numVirtualAntennas;i++)
    {
        caponBeamformingCfg->staticCfg.antennaCoordinates[2*i]= dpuConfigurations[5+2*i+1];
        caponBeamformingCfg->staticCfg.antennaCoordinates[2*i+1]= dpuConfigurations[5+2*i];
    }    

    /* Data Input EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.channel              = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_CH_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.channelShadow        = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SHADOW_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaIn.caponSpectrumChannel.eventQueue           = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_EVENT_QUE_CAPONSPECTRUM;

    /* Hot signature EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.channel              = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_CH_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.channelShadow        = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_SHADOW_CAPONSPECTRUM;
    caponBeamformingCfg->hwRes.edmaCfg.edmaHotSig.caponSpectrumChannel.eventQueue           = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAIN_SIG_EVENT_QUE_CAPONSPECTRUM;

    /* Data Output EDMA */
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channel             = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_CH;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.channelShadow       = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_SHADOW;
    caponBeamformingCfg->hwRes.edmaCfg.edmaOut.caponSpectrumChannel.eventQueue          = DPC_OBJDET_DPU_CAPONBEAMFORMING2D_EDMAOUT_CAPONSPECTRUM_EVENT_QUE;
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
*    Function to initialize Capon Beamforming 2D
*/
CaponBeamforming2DHWA_Handle caponBeamformingTest_Init(HWA_Handle gHwaHandle)
{
    int32_t  errorCode = 0;
    CaponBeamforming2DHWA_InitParams initParams;
    CaponBeamforming2DHWA_Handle  caponBeamformingHandle = NULL;
    initParams.hwaHandle =  gHwaHandle;
    /* generate the dpu handler*/
    caponBeamformingHandle =  CaponBeamforming2DHWA_init(&initParams, &errorCode);
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
void caponbeamforming2D_test_main(void *args)
{
    FILE                            *fileId;
    FILE                            *fileIDAns;
    int32_t                         retVal = 0;
    CaponBeamforming2DHWA_Handle      caponBeamformingHandle = NULL;
    CaponBeamforming2DHWA_Config      caponBeamformingCfg;
    HWA_Handle                      gHwaHandle = NULL;
    volatile uint32_t               startCycle, cycleCount;
    int32_t                         dpuConfigurations[17];   //[N_ant N_snaps N_anglesToSampleAzimuth N_anglesToSampleElevation N_instances test_angle_azimuth test_angle_elevation (antenna_coordinate_xi antenna_coordinate_yi)*N_ant]
    int16_t                         i;
    
    Drivers_open();         //calls EDMA_open and constructs tpcc intr
    Board_driversOpen();
    
    DebugP_log("DPU Test Started \r\n");
    
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_TPCCA_INIT|SOC_RCM_MEMINIT_TPCCB_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT|SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);

    /* Initialize the HWA for Capon Beamforming unit-testing */
    gHwaHandle = caponBeamformingTest_hwaInit();
    /* Initialize the Capon Beamforming for unit-testing */
    caponBeamformingHandle = caponBeamformingTest_Init(gHwaHandle);

    /* RUNNING THE INPUT CONFUGURATION FILE */
    /* Input file */
    fileId = fopen("..\\..\\..\\testbench\\caponData2DCfg.bin", "rb");
    // fileId = fopen("C:\\Users\\a0491005\\Downloads\\caponData2DCfg.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open the configuration .bin file!\n");
        exit(0);
    }   

    fread((int32_t *)&dpuConfigurations[0], sizeof(int32_t), 17*sizeof(int32_t), fileId);
    fclose(fileId);

    /* set the current Capon Beamforming parameters */
    Test_setProfile(&caponBeamformingCfg, &dpuConfigurations[0]);
    
    /* Configure the Capon Beamforming DPU according to the parameters set */
    retVal = CaponBeamforming2DHWA_config(caponBeamformingHandle, &caponBeamformingCfg);
    if (retVal !=0)
    {
        DebugP_log("Could not configure Capon Beamforming. Stopping");
        return;
    }

    fileId = fopen("..\\..\\..\\testbench\\caponData2D.bin", "rb");
    if (fileId == NULL)
    {
        printf("Error:  Cannot open the configuration .bin file!\n");
        exit(0);
    }

    fileIDAns = fopen("..\\..\\..\\testbench\\caponSpectrumTruth.bin", "rb");
    if (fileIDAns == NULL)
    {
        printf("Error:  Cannot open the configuration .bin file!\n");
        exit(0);
    }

    for(i=0;i<dpuConfigurations[4];i++)
    {
        uint8_t j;
        float caponSpectrumTruth[1024];
        
        /* RUNNING THE INPUT FILE */
        DebugP_log("Reading testcase %d\n", i+1);
        fread((int32_t *)&caponInputData[0], sizeof(int32_t), caponBeamformingCfg.staticCfg.numSnapshots/2*caponBeamformingCfg.staticCfg.numVirtualAntennas*sizeof(int32_t), fileId);

        startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
        /* Call the compute function */
        retVal = CaponBeamforming2DHWA_process(caponBeamformingHandle, &caponBeamformingCfg, caponSpectrum);
        if(retVal < 0)
        {
            DebugP_log("DEBUG: Capon Beamforming DPU process return error:%d \n", retVal);
            DebugP_assert(0);
            goto exit;
        }
        cycleCount = CycleCounterP_getCount32()-startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */
        DebugP_log("Measured processing time (Capon Beamforming) = CPU cycles = %d !!!\r\n", cycleCount);

        fread(caponSpectrumTruth,sizeof(float),caponBeamformingCfg.staticCfg.numAnglesToSampleAzimuth*caponBeamformingCfg.hwRes.bCnt*caponBeamformingCfg.staticCfg.numAnglesToSampleElevation,fileIDAns);

        for(j=0;j<caponBeamformingCfg.staticCfg.numAnglesToSampleAzimuth;j++)
        {
            if(caponSpectrumTruth[j] != caponSpectrum[j])
            {
                DebugP_log("Test case %d failed!\n\r", i+1);
                goto exit;
            }
        }
        DebugP_log("Test case %d passed!\n\r", i + 1);
    }
    DebugP_log("All test cases passed!\n\r");
    fclose(fileId);
    fclose(fileIDAns);

    exit:
        fclose(fileId);
        Board_driversClose();
        Drivers_close();
        return;
}
