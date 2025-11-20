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

/* Standard Include Files. */
#include <stdio.h>

/* MCU Plus Include Files. */
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>

/* Feature extraction library Include Files*/
#include <alg/featExtract/rangeDopplerBased/featExtract.h>

#define BIN_FILENAME_MAX_LEN 256

#define NUM_DOPPLER_BINS_ZERO_OUT 5

#define NUM_FRAMES 100

/* User defined heap memory and handle */
#define MY_HEAP_MEM_SIZE  (100*1024u)
static uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/* Feature extraction variables */
HeapP_Object gFeatExtractHeapObj;
static bool printForDebug = true;
static float fltEpsilon = 0.001f;

/* Clock and time measurement APIs variables */
static uint32_t startCycle, cycleCount;

void *featExtract_malloc(uint32_t sizeInBytes)
{
    return HeapP_alloc(&gFeatExtractHeapObj, sizeInBytes);
}
void featExtract_free(void *pFree, uint32_t sizeInBytes)
{
    HeapP_free(&gFeatExtractHeapObj, pFree);
}

void featextract_demo_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Feature Extraction Test Started!\r\n");

    /* create heap */
    HeapP_construct(&gFeatExtractHeapObj, gMyHeapMem, MY_HEAP_MEM_SIZE);
    DebugP_log("Benchmark data generation started: heap free size = %d bytes\r\n",
            (uint32_t)HeapP_getFreeHeapSize(&gFeatExtractHeapObj)
            );

    FeatExtract_moduleConfig config={0};
    FeatExtract_featOutput featOut;

    /* Number of features */
    uint32_t numFeatures;
    numFeatures = (uint32_t)(sizeof(FeatExtract_featOutput) / sizeof(float));
    DebugP_log("Number of available features: %d\n", numFeatures);

    void *handle;
	int32_t errCode;
    uint32_t *detMatrix;
    uint32_t *maxDetMatrix;
    int16_t *elev_idx;
    int16_t *azim_idx;

	FILE *fpCfg, *fpDetMat, *fpElAz, *fpMaxDet;
    FILE *fpFeatOutRef, *fpCycle, *fpFeatOut;
	char basicSpec[BIN_FILENAME_MAX_LEN];
    char detmatrices[BIN_FILENAME_MAX_LEN];
    char elev_azim_indices[BIN_FILENAME_MAX_LEN];
    char featOutRef[BIN_FILENAME_MAX_LEN];
    char maxDetmatrix[BIN_FILENAME_MAX_LEN];
    char featOutAppln[BIN_FILENAME_MAX_LEN];
    char cycleCounts[BIN_FILENAME_MAX_LEN];
    sprintf(basicSpec, "../../../testBench/basicSpec.bin");
    sprintf(detmatrices, "../../../testBench/detmatrices.bin");
    sprintf(elev_azim_indices, "../../../testBench/elev_azim_indices.bin");
    sprintf(featOutRef, "../../../testBench/featOutputRef.bin");
    sprintf(maxDetmatrix, "../../../testBench/maxDetmatrix.bin");
    sprintf(featOutAppln, "../../../testBench/featOut.bin");
    sprintf(cycleCounts, "../../../testBench/cycleOut.bin");

    /* Open the basic spec file */
    uint32_t i, j;
    fpCfg = fopen(basicSpec, "rb");
    DebugP_assert(fpCfg!=NULL);
    
    /* Open the detmatrix file */
    fpDetMat = fopen(detmatrices, "rb");
    DebugP_assert(fpDetMat!=NULL);

    /* Open the file containing azimuth elevation indices */
    fpElAz = fopen(elev_azim_indices, "rb");
    DebugP_assert(fpElAz!=NULL);

    /* Open the file containing max values of detection matrix */
    fpMaxDet = fopen(maxDetmatrix, "rb");
    DebugP_assert(fpMaxDet!=NULL);

    /* Open the file containing reference output for the feature extraction  */
    fpFeatOutRef = fopen(featOutRef, "rb");
    DebugP_assert(fpFeatOutRef!=NULL);

    /* Create the output features file */
    fpFeatOut = fopen(featOutAppln, "wb");
    DebugP_assert(fpFeatOut!=NULL);

    /* Create the cycle count file */
    fpCycle = fopen(cycleCounts, "wb");
    DebugP_assert(fpCycle!=NULL);

    /* Enable and reset CPU cycle counter */
    CycleCounterP_reset();

    /* Read in the basic config spec which remains constant throughout the test case */ 
    fread(&config.numFrames, sizeof(uint16_t), 1, fpCfg);
    fread(&config.noOfPts, sizeof(uint16_t), 1, fpCfg);
    fread(&config.azimuthFFTSize, sizeof(uint16_t), 1, fpCfg);
    fread(&config.elevationFFTSize, sizeof(uint16_t), 1, fpCfg);
    fread(&config.numRangeBins, sizeof(uint16_t), 1, fpCfg);
    fread(&config.numDopplerBins, sizeof(uint16_t), 1, fpCfg);
    fread(&config.maxNumRangeBins, sizeof(uint16_t), 1, fpCfg);
    fread(&config.minNumRangeBins, sizeof(uint16_t), 1, fpCfg);

    /* (Optional) print the config parameters */
    if (printForDebug == true) {
        DebugP_log("numRangeBins: %d\n", config.numRangeBins);
        DebugP_log("numDopplerBins: %d\n", config.numDopplerBins);
        DebugP_log("maxNumRangeBins: %d\n", config.maxNumRangeBins);
        DebugP_log("minNumRangeBins: %d\n", config.minNumRangeBins);
        DebugP_log("numFrames for correlation: %d\n", config.numFrames);
        DebugP_log("noOfPts for which angle estimation happens: %d\n", config.noOfPts);
        DebugP_log("azimuthFFTSize: %d\n", config.azimuthFFTSize);
        DebugP_log("elevationFFTSize: %d\n", config.elevationFFTSize);
    }

    /* Heap Allocation for files to be read and used in this demo */
    detMatrix = (uint32_t *)HeapP_alloc(&gFeatExtractHeapObj, config.numDopplerBins* config.numRangeBins * sizeof(uint32_t));
    maxDetMatrix = (uint32_t *)HeapP_alloc(&gFeatExtractHeapObj, config.noOfPts * sizeof(uint32_t));
    elev_idx = (int16_t *)HeapP_alloc(&gFeatExtractHeapObj, config.noOfPts * sizeof(int16_t));
    azim_idx = (int16_t *)HeapP_alloc(&gFeatExtractHeapObj, config.noOfPts * sizeof(int16_t));

    config.detMatrixMax = maxDetMatrix;
    config.azim_idx = azim_idx;
    config.elev_idx = elev_idx;

    config.minNumDopplerBins = NUM_DOPPLER_BINS_ZERO_OUT;
    config.maxNumDopplerBins = config.numDopplerBins-NUM_DOPPLER_BINS_ZERO_OUT-1;

    /* Call the create function (once only before start of 1st frame) */
    handle = featExtract_create(&config, &errCode);

    /* Read the parameters from a file for frame*/
    for (i = 0; i < NUM_FRAMES; i++) 
    {
        /* Read the range-Doppler heatmap */
        for(j=0;j<config.numRangeBins;j++)
        {
            fread(detMatrix+j*config.numDopplerBins, config.numDopplerBins * sizeof(uint32_t) , 1, fpDetMat);
            if(j%10 == 0)
            {
                DebugP_log("Reading Detection Matrix From File.. %d percent\n", j * 100 / config.numRangeBins);
            }
        }
        /* Read the maxDetMatrix file */
        fread(maxDetMatrix, config.noOfPts * sizeof(uint32_t) , 1, fpMaxDet);

        /* Read the elev_idx file */
        fread(elev_idx, config.noOfPts * sizeof(int16_t) , 1, fpElAz);

        /* Read the azim_idx file */
        fread(azim_idx, config.noOfPts * sizeof(int16_t) , 1, fpElAz);

        /* Call the compute function */
        startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
        featExtract_compute(handle, detMatrix, i, &featOut);
        cycleCount = CycleCounterP_getCount32() - startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */

        /* (Optional) print the cycle */
        if (printForDebug == true) {
            DebugP_log("Measured processing time (featExtract_compute) = CPU cycles = %d !!!\r\n", cycleCount);
        }

        /* Write the cycle */
        fwrite(&cycleCount, sizeof(uint32_t), 1, fpCycle);

        /* Write the output */
        fwrite(&featOut, sizeof(FeatExtract_featOutput), 1, fpFeatOut);

        /* Status (every frame) */
        DebugP_log("Frame %d/%d is processed...\n", i+1, NUM_FRAMES);
    }

    /* Call the delete function */           
    featExtract_delete(handle);

    HeapP_free(&gFeatExtractHeapObj, detMatrix);
    HeapP_free(&gFeatExtractHeapObj, maxDetMatrix);
    HeapP_free(&gFeatExtractHeapObj, elev_idx);
    HeapP_free(&gFeatExtractHeapObj, azim_idx);

    fclose(fpCfg);
    fclose(fpDetMat);
    fclose(fpFeatOutRef);
    fclose(fpMaxDet);
    fclose(fpElAz);
    fclose(fpFeatOut);
    fclose(fpCycle);

    DebugP_log("Benchmark data generation ended: heap free size = %d bytes\r\n",
            (uint32_t)HeapP_getFreeHeapSize(&gFeatExtractHeapObj)
            );
    HeapP_destruct(&gFeatExtractHeapObj);

    DebugP_log("Benchmark validation started!\r\n");

    /* Open the reference and output features file */
    fpFeatOut = fopen(featOutAppln, "rb");
    DebugP_assert(fpFeatOut!=NULL);
    fpFeatOutRef = fopen(featOutRef, "rb");
    DebugP_assert(fpFeatOutRef!=NULL);

    float feat, featRef, featError;
    int32_t errCount = 0;
    for (i = 0; i < NUM_FRAMES*numFeatures; i++) {
        fread(&feat, sizeof(float), 1, fpFeatOut);
        fread(&featRef, sizeof(float), 1, fpFeatOutRef);
        featError = feat - featRef;
        if (fabs(featError) > fltEpsilon) {
            errCount++;
        }
    }
    
    DebugP_log("Total features compared: %d!!\r\n", numFeatures*NUM_FRAMES);
    DebugP_log("Total features failed: %d!!\r\n", errCount);
    DebugP_log("Overal test performance: %%%.2f!!\r\n", (float)(numFeatures*NUM_FRAMES-errCount) / (float)(numFeatures*NUM_FRAMES) * 100.0f);

    DebugP_log("Benchmark validation ended!\r\n");

    DebugP_log("Feature Extraction Test Ended!\r\n");

    Board_driversClose();
    Drivers_close();
}

