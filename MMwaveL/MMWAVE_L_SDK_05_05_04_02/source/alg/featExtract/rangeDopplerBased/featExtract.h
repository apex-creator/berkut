/*! 
 *  \file   featExtract.h
 *
 *  \brief  Header file for feature extraction module
 *
 * Copyright (C) 2023 Texas Instruments Incorporated 
 * 
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
 *
*/

#ifndef FEATEXTRACT_H
#define FEATEXTRACT_H


#ifdef __cplusplus
extern "C" {
#endif


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <float.h>


/* OSAL definition for MEX code generation. Comment out if needed (i.e., when the PC MEX is built) */
// #define OSAL_MEX

#ifndef OSAL_MEX
#include <kernel/dpl/HeapP.h>
#endif

// #define LOWPOWER_DEMO
/* Feature Extraction Handle */
typedef void* FeatExtract_Handle;

/* Algorithm Error codes */
#define FEXTRACT_ERRNO_BASE		(-8000)                     /* Base error code for FEXTRACT algorithm */
#define FEXTRACT_EOK			(0)                         /* Error code: No errors */
#define FEXTRACT_EINVAL			(FEXTRACT_ERRNO_BASE-1)     /* Error Code: Invalid argument */
#define FEXTRACT_EINUSE			(FEXTRACT_ERRNO_BASE-2)     /* Error Code: Operation cannot be implemented because a previous operation is still not complete */
#define FEXTRACT_ENOTIMPL		(FEXTRACT_ERRNO_BASE-3)     /* Error Code: Operation is not implemented */
#define FEXTRACT_ENOMEM			(FEXTRACT_ERRNO_BASE-4)     /* Error Code: Out of memory */


/* Maximum supported configurations */
#define NUM_DOPPLER_BINS_MAX		(256U)  /* Defines maximum number of doppler bins the algorithm will accept at configuration time */
#define NUM_RANGE_BINS_MAX		    (128U)  /* Defines maximum number of range bins the algorithm will accept at configuration time */
#define THRESH_NUM_POINTS           2000    /* Threshold above which num points is to be calculated */


/* Benchmarking results. During runtime execution, feature extraction step function can optionally return cycle counts for the sub-functions defined below
 * Each count is 32bit unsigned integer value representing a timestamp of free runing clock */
#define FEXTRACT_BENCHMARK_COMPUTE   (0U)    /* Cycle count after compute function */

/* Module config */
typedef struct {
	uint32_t numRangeBins;   /* Number of Range bins */
    int32_t numDopplerBins; /* Number of Doppler bins */
    uint32_t maxNumRangeBins;/* Maximum number of range bins to be considered - varies from application to application for (K2O its usually set to 40 while for hand gesture recognition this is set to 8 (for near-field and larger values for far-field) */
    uint32_t minNumRangeBins;/* Usually the first few range bins are left behind for some application to avoid leakage */
    uint32_t maxNumDopplerBins;/* Few doppler bins initially and few of them at last are zeroed/left out to take into account the spectral leakage */
    uint32_t minNumDopplerBins;/* Few doppler bins initially and few of them at last are zeroed/left out to take into account the spectral leakage */
    uint32_t numFrames;        /* Number of frames after which the hybrid metrics computation must start */
    uint32_t *detMatrixMax;    /* Pointer where the max of detection Matrix is stored */
    uint16_t noOfPts;          /* Pointer which shows the number of max points to be taken from the detection matrix */
    int16_t  *elev_idx;        /* Pointer to where elevation indices are stored */
    int16_t  *azim_idx;        /* Pointer to where azimuth indices are stored */
    uint16_t azimuthFFTSize;   /* Azimuth FFT size */
    uint16_t elevationFFTSize; /* Elevation FFT size */
} FeatExtract_moduleConfig;


/* Feature output structure */
typedef struct
{
    float rangeAvg;     /* Range Average across Range-Doppler heatmap */
    float dopplerAvg;   /* Doppler Average across Range-Doppler heatmap */
    float dopplerAvgPos;/* Doppler Positive Average across Range-Doppler heatmap */
    float dopplerAvgNeg;/* Doppler Negative Average across Range-Doppler heatmap */
    float numPoints;    /* Number of points above a certain threshold in Range-Doppler heatmap*/
    float azimWtMean;   /* Azimuth Weighted Mean across Range-Doppler heatmap */
    float elevWtMean;   /* Elevation Weighted Mean across Range-Doppler heatmap */
    float azimWtDisp;   /* Azimuth Weighted Displacement across Range-Doppler heatmap */
    float elevWtDisp;   /* Elevation Weighted Displacement across Range-Doppler heatmap */

    float dopAzimCorr;  /* Hybrid feature which is a correlation between Doppler and Azimuth for numFrames  */
    float dopElevCorr;  /* Hybrid feature which is a correlation between Doppler and Elevation for numFrames  */
    float dopPosNegCorr;/* Hybrid feature which is a correlation between Doppler Positive and Azimuth for numFrames  */
    float dopPosElevCorr;/* Hybrid feature which is a correlation between Doppler Positive and Elevation for numFrames  */
    float dopPosAzimCorr;/* Hybrid feature which is a correlation between Doppler Positive and Azimuth for numFrames  */
    float dopNegElevCorr;/* Hybrid feature which is a correlation between Doppler Negative and Elevation for numFrames  */
    float dopNegAzimCorr;/* Hybrid feature which is a correlation between Doppler Negative and Azimuth for numFrames  */

    float multnumPointsAzim;
    float multnumPointsElev;
} FeatExtract_featOutput;


/* Feature extraction module instance structure */
typedef struct {
	uint32_t numRangeBins;   /* Number of Range bins */
    int32_t numDopplerBins; /* Number of Doppler bins */
    uint32_t maxNumRangeBins;/* Maximum number of range bins to be considered - varies from application to application for (K2O its usually set to 40 while for hand gesture recognition this is set to 8 (for near-field and larger values for far-field) */
    uint32_t minNumRangeBins;/* Usually the first few range bins are left behind for some application to avoid leakage */
    uint32_t maxNumDopplerBins;/* Few doppler bins initially and few of them at last are zeroed/left out to take into account the spectral leakage */
    uint32_t minNumDopplerBins;/* Few doppler bins initially and few of them at last are zeroed/left out to take into account the spectral leakage */
    uint32_t numFrames;        /* Number of frames after which the hybrid metrics computation must start */
    uint32_t *detMatrixMax;   /* Pointer where the max of detection Matrix is stored */
    uint16_t noOfPts;        /* Pointer which shows the number of max points to be taken from the detection matrix */
    int16_t  *elev_idx;        /* Pointer to where elevation indices are stored */
    int16_t  *azim_idx;        /* Pointer to where azimuth indices are stored */
    uint16_t azimuthFFTSize;   /* Azimuth FFT size */
    uint16_t elevationFFTSize; /* Elevation FFT size */
    uint16_t frameCntr;        /* Frame Counter */
    float *dop;          /* Doppler Average pointer for hybrid metrics computation */
    float *dopPos;       /* Doppler Positive Average pointer for hybrid metrics computation */
    float *dopNeg;       /* Doppler Negative Average pointer for hybrid metrics computation */
    float *azWts;        /* Azimuth Weighted Mean pointer for hybrid metrics computation */
    float *elWts;        /* Elevation Weighted Mean pointer for hybrid metrics computation */
} FeatExtract_moduleInstance;


/* Compute Spectrum Features Function Declaration */
extern void *featExtract_create(FeatExtract_moduleConfig *config, int32_t *errCode);
extern void featExtract_compute(void *handle, uint32_t *detMatrix, uint16_t frame_no, FeatExtract_featOutput *featOut);
extern void featExtract_delete(void *handle);

/* Declarations of functions which are involved in feature computation process specific for rangeDoppler, elevAzim and hybrid based */
void computeFeatures_rangeDoppler(uint32_t *detMatrix, FeatExtract_featOutput *featOut, FeatExtract_moduleInstance *inst);
void computeFeatures_azimElev(uint32_t *detMatrix, FeatExtract_featOutput *featOut, FeatExtract_moduleInstance *inst);
void computeFeatures_hybrid(FeatExtract_featOutput *featOut, FeatExtract_moduleInstance *inst);

/* Declarations of memory allocation functions. Call these functions to allocate or free memory */
void *featExtract_malloc(uint32_t sizeInBytes);
void featExtract_free(void *pFree, uint32_t sizeInBytes);

#ifdef __cplusplus
}
#endif

#endif
