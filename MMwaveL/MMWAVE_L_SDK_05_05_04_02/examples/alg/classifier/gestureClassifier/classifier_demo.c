/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

/* Classifier library Include Files*/
#include <alg/classifier/gestureClassifier/classifier.h>

#define BIN_FILENAME_MAX_LEN 256

/* User defined heap memory and handle */
#define MY_HEAP_MEM_SIZE  (16*1024u)
static uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/* Classifier variables */
HeapP_Object gClassifierHeapObj;
static bool internalMalloc = true;
static bool printForDebug = false;
static float fltEpsilon = 0.005f;

/* Clock and time measurement APIs variables */
static uint32_t startCycle, cycleCount;

/* Constant layer definitions */
#define CLASSIFIER_NUM_FRAMES           15
#define CLASSIFIER_NUM_FEATURES         6
#define CLASSIFIER_NUM_CLASSES          8
#define CLASSIFIER_DENSE1_NUM_NODES     10
#define CLASSIFIER_DENSE2_NUM_NODES     20
//#define CLASSIFIER_THREE_LAYERS
#ifdef CLASSIFIER_THREE_LAYERS
    #define CLASSIFIER_DENSE3_NUM_NODES    40
#endif

void *classifier_malloc(uint32_t sizeInBytes)
{
    return HeapP_alloc(&gClassifierHeapObj, sizeInBytes);
}
void classifier_free(void *pFree, uint32_t sizeInBytes)
{
    HeapP_free(&gClassifierHeapObj, pFree);
}

void classifier_demo_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    DebugP_log("Classifier Test Started!\r\n");

    /* create heap */
    HeapP_construct(&gClassifierHeapObj, gMyHeapMem, MY_HEAP_MEM_SIZE);
    DebugP_log("Benchmark data generation started: heap free size = %d bytes\r\n",
            (uint32_t)HeapP_getFreeHeapSize(&gClassifierHeapObj)
            );

    Classifier_moduleConfig config;

    void *handle;
	int32_t errCode;

    /* Features and Predictions  */
    float features[CLASSIFIER_NUM_FEATURES*CLASSIFIER_NUM_FRAMES] = { 0 };
    float predictions[CLASSIFIER_NUM_CLASSES] = { 0 };

	FILE *fpCfg, *fpFeat, *fpPredRef, *fpPred, *fpCycle;
    char fileNameCfg[BIN_FILENAME_MAX_LEN];
    char fileNameFeat[BIN_FILENAME_MAX_LEN];
    char fileNamePredRef[BIN_FILENAME_MAX_LEN];
    char fileNamePred[BIN_FILENAME_MAX_LEN];
    char fileNameCycle[BIN_FILENAME_MAX_LEN];
    sprintf(fileNameCfg, "../../../testBench/fileCfgIn.bin");
    sprintf(fileNameFeat, "../../../testBench/fileFeatIn.bin");
    #ifdef CLASSIFIER_THREE_LAYERS
        sprintf(fileNamePredRef, "../../../testBench/filePredRef_3.bin");
    #else
        sprintf(fileNamePredRef, "../../../testBench/filePredRef.bin");
    #endif
    sprintf(fileNamePred, "../../../testBench/filePredOut.bin");
    sprintf(fileNameCycle, "../../../testBench/fileCycleOut.bin");
    
    /* Open the input file */
    uint32_t i, n, num_observations;
    fpCfg = fopen(fileNameCfg, "rb");
    DebugP_assert(fpCfg!=NULL);

    /* Read the number of observations from the file */
    fread(&num_observations, sizeof(uint32_t), 1, fpCfg);
    DebugP_log("num_observations: %d\n", num_observations);
    
    /* Read the other parameters from the file */
    fread(&config.num_frames, sizeof(int32_t), 1, fpCfg);
    fread(&config.num_features, sizeof(int32_t), 1, fpCfg);
    fread(&config.num_classes, sizeof(int32_t), 1, fpCfg);
    DebugP_assert(CLASSIFIER_NUM_FRAMES == config.num_frames);
    DebugP_assert(CLASSIFIER_NUM_FEATURES == config.num_features);
    DebugP_assert(CLASSIFIER_NUM_CLASSES == config.num_classes);
    DebugP_log("num_frames: %d\n", config.num_frames);
    DebugP_log("num_features: %d\n", config.num_features);
    DebugP_log("num_classes: %d\n", config.num_classes);

    /* Allocate internal memory or not */
    if (internalMalloc == true) {
        config.scratchBuffer = NULL;
        config.scratchBufferSizeInBytes = 0;
    }
    else {
        #ifdef CLASSIFIER_THREE_LAYERS
            uint32_t bufferSizeNeeded = CLASSIFIER_DENSE1_NUM_NODES + CLASSIFIER_DENSE2_NUM_NODES + CLASSIFIER_DENSE3_NUM_NODES +CLASSIFIER_NUM_CLASSES;
        #else
            uint32_t bufferSizeNeeded = CLASSIFIER_DENSE1_NUM_NODES + CLASSIFIER_DENSE2_NUM_NODES + CLASSIFIER_NUM_CLASSES;
        #endif
        config.scratchBuffer = (float*)HeapP_alloc(&gClassifierHeapObj, bufferSizeNeeded * sizeof(float));
        config.scratchBufferSizeInBytes = bufferSizeNeeded * sizeof(float);
        DebugP_assert(config.scratchBuffer!=NULL);       
    }

    /* Open the features file */
    fpFeat = fopen(fileNameFeat, "rb");
    DebugP_assert(fpFeat!=NULL);

    /* Open the predictions file */
    fpPred = fopen(fileNamePred, "wb");
    DebugP_assert(fpPred!=NULL);

    /* Create the cycle count file */
    fpCycle = fopen(fileNameCycle, "wb");
    DebugP_assert(fpCycle!=NULL);

    /* Call the create function */
    handle = classifier_create(&config, &errCode);
    DebugP_assert(handle!=NULL);
    DebugP_assert(errCode == 0);

    /* Enable and reset CPU cycle counter */
    CycleCounterP_reset();

    /* Process the observations */
    for (i = 0; i < num_observations; i++) {
        /* Read the features file */
        fread(features, CLASSIFIER_NUM_FEATURES * CLASSIFIER_NUM_FRAMES * sizeof(float), 1, fpFeat);      
        
        /* (Optional) print the features */
        if (printForDebug == true) {
            DebugP_log("Features: ");
            for (n = 0; n < CLASSIFIER_NUM_FEATURES * CLASSIFIER_NUM_FRAMES; n++) {
                DebugP_log("%f ", features[n]);
            }
            DebugP_log("\n");
        }

        /* Call the predict function */
        startCycle = CycleCounterP_getCount32(); /* get CPU cycle count */
        classifier_predict(handle, features, predictions);
        cycleCount = CycleCounterP_getCount32() - startCycle; /* get CPU cycle count and calculate diff, we dont expect any overflow for this short duration */

        /* (Optional) print the cycle */
        if (printForDebug == true) {
            DebugP_log("Measured processing time (classifier_predict) = CPU cycles = %d !!!\r\n", cycleCount);
        }

        /* Write the cycle */
        fwrite(&cycleCount, sizeof(uint32_t), 1, fpCycle);

        /* Write the output */
        fwrite(predictions, CLASSIFIER_NUM_CLASSES * sizeof(float), 1, fpPred);

        // (Optional) print the predictions
        if (printForDebug == true) {
            DebugP_log("Predictions: ");
            for (n = 0; n < CLASSIFIER_NUM_CLASSES; n++) {
                DebugP_log("%f ", predictions[n]);
            }
            DebugP_log("\n");
        }

        /* Status (every 10th test) */
        if ((i%10) == 0) {
            DebugP_log("Observation %d/%d is processed...\n", i+1, num_observations);
        }
    }

    /* Call the delete function */           
    classifier_delete(handle);

    /* Free the memory */
    if (internalMalloc == false)
        HeapP_free(&gClassifierHeapObj, config.scratchBuffer); 

    fclose(fpCfg);
    fclose(fpFeat);
    fclose(fpPred);
    fclose(fpCycle);

    DebugP_log("Benchmark data generation ended: heap free size = %d bytes\r\n",
            (uint32_t)HeapP_getFreeHeapSize(&gClassifierHeapObj)
            );
    HeapP_destruct(&gClassifierHeapObj);

    DebugP_log("Benchmark validation started!\r\n");

    /* Open the reference and output predictions file */
    fpPred = fopen(fileNamePred, "rb");
    DebugP_assert(fpPred!=NULL);
    fpPredRef = fopen(fileNamePredRef, "rb");
    DebugP_assert(fpPredRef!=NULL);

    float pred, predRef, predError;
    int32_t errCount = 0, testCount = 0;
    for (i = 0; i < num_observations*CLASSIFIER_NUM_CLASSES; i++) {
        fread(&pred, sizeof(float), 1, fpPred);
        fread(&predRef, sizeof(float), 1, fpPredRef);
        predError = pred - predRef;
        if (fabs(predError) > fltEpsilon) {
            errCount++;
        }
        testCount++;
    }
    
    DebugP_log("Total predictions compared: %d!!\r\n", testCount);
    DebugP_log("Total predictions failed: %d!!\r\n", errCount);
    DebugP_log("Overall test performance: %%%.2f!!\r\n", (float)(testCount-errCount) / (float)testCount * 100.0f);

    DebugP_log("Benchmark validation ended!\r\n");

    DebugP_log("Classifier Test Ended!\r\n");

    Board_driversClose();
    Drivers_close();
}

