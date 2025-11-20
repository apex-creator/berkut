/*! 
 *  \file   featExtract.h
 *
 *  \brief  Header file for feature extraction module
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/ 
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
#define FEXTRACT_FREQ_BINS_MAX		(1024U)  /* Defines maximum number of spectrum bins the algorithm will accept at configuration time */


/* Benchmarking results. During runtime execution, feature extraction step function can optionally return cycle counts for the sub-functions defined below
 * Each count is 32bit unsigned integer value representing a timestamp of free runing clock */
#define FEXTRACT_BENCHMARK_COMPUTE   (0U)    /* Cycle count after compute function */


/* Power threshold to extract lower and upper envelopes */
typedef struct {
	float low; /* Lower power interception (in percentage) */
	float up;  /* Upper power interception (in percentage) */
} FeatExtract_powerThreshold;


/* Module config */
typedef struct {
	uint32_t numFreqBins;   /* Number of frequency bins */
    float    freqStep;      /* Conversion factor from index to frequency */
    FeatExtract_powerThreshold pwrThre; /* Power threshold */
    uint32_t specShiftMode; /* 0 (no shift), 1 (shift around mean), 2 (shift around median) */

    float *scratchBuffer;   /* Scratch memory needed for computations. Keep NULL to request internal allocation */
    uint32_t scratchBufferSizeInBytes; /* Module needs ((numFreqBins + 1) * sizeof(float)) bytes */
} FeatExtract_moduleConfig;


/* Feature output structure */
typedef struct
{
    float fLow;     /* Lower frequency border of the occupied bandwidth */
    float fUp;      /* Upper frequency border of the occupied bandwidth */
    float bwPwr;    /* Power within the occupied bandwidth */
    float meanFreq; /* Mean frequency of the power spectral density estimate */
    float medFreq;  /* Median frequency of the power spectral density estimate */
    float sEntropy; /* Spectral entropy of the power spectral density estimate */
} FeatExtract_featOutput;


/* Feature extraction module instance structure */
typedef struct {
    uint32_t numFreqBins;   /* Number of frequency bins */
    float    freqStep;      /* Conversion factor from index to frequency */
    FeatExtract_powerThreshold pwrThre; /* Power threshold */
    uint32_t specShiftMode; /* 0 (no shift), 1 (shift around mean), 2 (shift around median) */

    bool internalMalloc;    /* Memory is allocated internally */
    float *cumPwr;          /* Cumulative power */
} FeatExtract_moduleInstance;


/* Compute Spectrum Features Function Declaration */
extern void *featExtract_create(FeatExtract_moduleConfig *config, int32_t *errCode);
extern void featExtract_compute(void *handle, float *spec, FeatExtract_featOutput *featOut);
extern void featExtract_delete(void *handle);


/* Declarations of utility functions */
float featExtract_interpFreq(const float *cumPwr, float freqStep, uint32_t length, float pwrThresh);
float featExtract_computeSpectralEntropy(const float *spec, float *buffer, uint32_t length);
float featExtract_computeWeightedPower(const float *spec, uint32_t length, float freqStep);
uint32_t featExtract_freqToIndex(float freq, uint32_t numFreqBins, float freqStep);
void featExtract_specShiftAroundFreq(float *spec, uint32_t numFreqBins, float freqStep, float shiftFreq);


/* Declarations of math functions */
void featExtract_cumulativeSum(const float *x, uint32_t length, float *y);
void featExtract_cumulativeWeightedSum(const float *x, uint32_t length, float c, float *y);
void featExtract_vectorScalarMul(const float *x, uint32_t length, float c, float *y);
float featExtract_linearInterpolate(float ypre, float ypost, float xpre, float xpost, float x);
float featExtract_vectorSum(const float *x, uint32_t length);
float featExtract_vectorInnerProduct(const float *x, const float *y, uint32_t length);
float featExtract_vectorEntropy(const float *x, uint32_t length);
void featExtract_reverseVector(float *x, uint32_t start, uint32_t end);
void featExtract_circularRotateVector(float *x, uint32_t length, int32_t numShifts);
void featExtract_fftShiftVector(float *x, uint32_t length);
void featExtract_vectorNormalize(const float *x, uint32_t length, float *y);
void featExtract_vectorMaxMin(const float *x, uint32_t length, float *xMax, float *xMin);


/* Declarations of memory allocation functions. Call these functions to allocate or free memory */
void *featExtract_malloc(uint32_t sizeInBytes);
void featExtract_free(void *pFree, uint32_t sizeInBytes);


#ifdef __cplusplus
}
#endif

#endif
