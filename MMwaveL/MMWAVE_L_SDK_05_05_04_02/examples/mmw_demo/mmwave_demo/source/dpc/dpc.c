/*
 * Copyright (C) 2022-24 Texas Instruments Incorporated
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "FreeRTOS.h"
#include "task.h"
/* mmwave SDK files */
#include <control/mmwave/mmwave.h>
#include "source/mmw_cli.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "drivers/power.h"
#include <drivers/prcm.h>
#include <utils/mathutils/mathutils.h>

#include "source/mmwave_demo.h"
#include "source/mmw_res.h"
#include "source/dpc/dpc.h"
#include "source/mmwave_control/interrupts.h"
#include "source/calibrations/range_phase_bias_measurement.h"
#include "source/utils/mmw_demo_utils.h"

#define HWA_MAX_NUM_DMA_TRIG_CHANNELS 16
#define MAX_NUM_DETECTIONS          (MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE)

#define LOW_PWR_MODE_DISABLE (0)
#define LOW_PWR_MODE_ENABLE (1)
#define LOW_PWR_TEST_MODE (2)

#define MMW_DEMO_MAJOR_MODE 0
#define MMW_DEMO_MINOR_MODE 1

#define FRAME_REF_TIMER_CLOCK_MHZ  40

/* Max Frame Size for FTDI chip is 64KB */
#define MAXSPISIZEFTDI               (65536U)

#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING
#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL   0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL   1
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 17

#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3e8)

#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING

#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 17

#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET 0
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_BLACKMAN
#define DPC_OBJDET_QFORMAT_RANGE_FFT 17
#define MMW_DEMO_TEST_ADC_BUFF_SIZE 1024  //maximum 128 real samples (int16_t), 3 Rx channels

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern HWA_Handle hwaHandle;

#define L3_MEM_SIZE (0x40000 + 160*1024)
extern uint8_t gMmwL3[L3_MEM_SIZE]  __attribute((section(".l3")));
/*! Local RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE ((8U+6U+4U+2U+8U) * 1024U)
extern uint8_t gMmwCoreLocMem[MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE];
/*! Local RAM buffer for tracker */
#define MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE (25U * 1024U)
uint8_t gMmwCoreLocMem2[MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE];
/* User defined heap memory and handle */
#define MMWDEMO_OBJDET_CORE_LOCAL_MEM3_SIZE  (2*1024u)
extern uint8_t gMmwCoreLocMem3[MMWDEMO_OBJDET_CORE_LOCAL_MEM3_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

extern uint8_t pgVersion;

// LED config
extern uint32_t gpioBaseAddrLed, pinNumLed;
extern MMWave_temperatureStats  tempStats;

extern DPU_DoaProc_HW_Resources  *hwRes;

volatile unsigned long long test;
void mmwDemo_dpcTask();

DPU_Aoa2dProc_Config aoa2dProcDpuCfg;
DPU_CFARProcHWA_Config cfarProcDpuCfg;
DPU_DopplerProcHWA_Config dopplerProcDpuCfg;
DPU_RangeProcHWA_Config rangeProcDpuCfg;

/*! @brief     EDMA interrupt objects for DPUs */
Edma_IntrObject intrObj_aoa2dProc;
Edma_IntrObject intrObj_cfarProc;
Edma_IntrObject intrObj_dopplerProc;
Edma_IntrObject     intrObj_rangeProc[2];

/**
 *  @b Description
 *  @n
 *      The function allocates HWA DMA source channel from the pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      channel Allocated HWA trigger source channel
 */
uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool)
{
    uint8_t channel = 0xFF;
    if(pool->dmaTrigSrcNextChan < HWA_MAX_NUM_DMA_TRIG_CHANNELS)
    {
        channel = pool->dmaTrigSrcNextChan;
        pool->dmaTrigSrcNextChan++;
    }
    return channel;
}

/**
 *  @b Description
 *  @n
 *      The function resets HWA DMA source channel pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      none
 */
void DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool)
{
    pool->dmaTrigSrcNextChan = 0;
}

/**
 *  @b Description
 *  @n
 *      The function allocates memory in HWA RAM memory pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      startSampleIndex sample index in the HWA RAM memory
 */
int16_t DPC_ObjDet_HwaWinRamMemoryPoolAlloc(HwaWinRamMemoryPoolObj *pool, uint16_t numSamples)
{
    int16_t startSampleIndex = -1;
    if((pool->memStartSampleIndex + numSamples) < (CSL_APP_HWA_WINDOW_RAM_U_SIZE/sizeof(uint32_t)))
    {
        startSampleIndex = pool->memStartSampleIndex;
        pool->memStartSampleIndex += numSamples;
    }
    return startSampleIndex;
}

/**
 *  @b Description
 *  @n
 *      The function resets HWA DMA source channel pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      none
 */
void DPC_ObjDet_HwaWinRamMemoryPoolReset(HwaWinRamMemoryPoolObj *pool)
{
    pool->memStartSampleIndex = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
void DPC_ObjDet_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}


/**
 *  @b Description
 *  @n
 *      Utility function for setting memory pool to desired address in the pool.
 *      Helps to rewind for example.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  addr Address to assign to the pool's current address.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
void DPC_ObjDet_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr = (uintptr_t)addr;
    pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting memory pool current address.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to current address of the pool (from which next allocation will
 *      allocate to the desired alignment).
 */
void *DPC_ObjDet_MemPoolGet(MemPoolObj *pool)
{
    return((void *)pool->currAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
uint32_t DPC_ObjDet_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
void *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool,
                              uint32_t size,
                              uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function to do a parabolic/quadratic fit on 3 input points
 *      and return the coordinates of the peak. This is used to accurately estimate
 *      range bias.
 *
 *  @param[in]  x Pointer to array of 3 elements representing the x-coordinate
 *              of the points to fit
 *  @param[in]  y Pointer to array of 3 elements representing the y-coordinate
 *              of the points to fit
 *  @param[out] xv Pointer to output x-coordinate of the peak value
 *  @param[out] yv Pointer to output y-coordinate of the peak value
 *
 *  @retval   None
 *
 */
void rangeBiasRxChPhaseMeasure_quadfit(float *x, float*y, float *xv, float *yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];

    denom = (x0 - x1)*(x0 - x2)*(x1 - x2);
    if (denom != 0.)
    {
        a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
        b = (x2*x2 * (y0 - y1) + x1*x1 * (y2 - y0) + x0*x0 * (y1 - y2)) / denom;
        c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;
    }
    else
    {
        *xv = x[1];
        *yv = y[1];
        return;
    }
    if (a != 0.)
    {
        *xv = -b/(2*a);
        *yv = c - b*b/(4*a);
    }
    else
    {
        *xv = x[1];
        *yv = y[1];
    }
}

/**
*  @b Description
*  @n
*    Function to construct feature extract heap
*/
void featExtract_heapConstruct()
{
    HeapP_construct(&gMmwMssMCB.CoreLocalFeatExtractHeapObj, (void *) gMmwCoreLocMem3, MMWDEMO_OBJDET_CORE_LOCAL_MEM3_SIZE);
}

/**
*  @b Description
*  @n
*    Function to allocate memory for feature extract heap
*/
void *featExtract_malloc(uint32_t sizeInBytes)
{
    return HeapP_alloc(&gMmwMssMCB.CoreLocalFeatExtractHeapObj, sizeInBytes);
}

/**
*  @b Description
*  @n
*    Function to free memory from feature extract heap
*/
void featExtract_free(void *pFree, uint32_t sizeInBytes)
{
    HeapP_free(&gMmwMssMCB.CoreLocalFeatExtractHeapObj, pFree);
}

/**
*  @b Description
*  @n
*    Function to get memory usage stats of feature extract heap object
*/
uint32_t featExtract_memUsage()
{
    uint32_t usedMemSizeInBytes;
    HeapP_MemStats heapStats;

    HeapP_getHeapStats(&gMmwMssMCB.CoreLocalFeatExtractHeapObj, &heapStats);
    usedMemSizeInBytes = sizeof(gMmwCoreLocMem3) - heapStats.availableHeapSpaceInBytes;

    return usedMemSizeInBytes;
}

/**
*  @b Description
*  @n
*    Select coordinates of active virtual antennas and calculate the size of the 2D virtual antenna pattern,
*    i.e. number of antenna rows and number of antenna columns.
*/
void MmwDemo_calcActiveAntennaGeometry()
{
    int32_t txInd, rxInd, ind;
    int32_t rowMax, colMax;
    int32_t rowMin, colMin;
    /* Select only active antennas */
    ind = 0;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind] = gMmwMssMCB.antennaGeometryCfg.ant[gMmwMssMCB.rxAntOrder[rxInd] + (txInd * SYS_COMMON_NUM_RX_CHANNEL)];
            ind++;
        }
    }

    /* Calculate virtual antenna 2D array size */
    ind = 0;
    rowMax = 0;
    colMax = 0;
    rowMin = 127;
    colMin = 127;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row > rowMax)
            {
                rowMax = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col > colMax)
            {
                colMax = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row < rowMin)
            {
                rowMin = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
            }
            if (gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col < colMin)
            {
                colMin = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
            }
            ind++;
        }
    }
    ind = 0;
    for (txInd = 0; txInd < gMmwMssMCB.numTxAntennas; txInd++)
    {
        for (rxInd = 0; rxInd < gMmwMssMCB.numRxAntennas; rxInd++)
        {
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row -= rowMin;
            gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col -= colMin;
            ind++;
        }
    }
    gMmwMssMCB.numAntRow = rowMax - rowMin + 1;
    gMmwMssMCB.numAntCol = colMax - colMin + 1;
}

/**
*  @b Description
*  @n
*    Based on the activeAntennaGeometryCfg configures the table which used to configure
*    Doppler FFT HWA param sets in DoA DPU. THese param sets perform Doppler FFT and
*    at the same time mapping of input antennas into 2D row-column antenna array where columns
*    are in  azimuth dimension, and rows in elevation dimension.
*    It also calculates the size of 2D antenna array, ie. number of rows and number of columns.
*/
int32_t MmwDemo_cfgDopplerParamMapping(DPU_Aoa2dProc_HWA_Option_Cfg *dopplerParamCfg, uint32_t mappingOption)
{
    int32_t ind, indNext, indNextPrev;
    int32_t row, col;
    int32_t dopParamInd;
    int32_t state;
    int16_t BT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t DT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t SCAL[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int8_t  DONE[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int32_t retVal = 0;
    int32_t rowOffset;

    if (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol > DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS)
    {
        retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
        goto exit;
    }

    if (mappingOption == DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL)
    {
        /*For AOA DPU, Output is */
        rowOffset =  gMmwMssMCB.numAntCol;
    }
    else if (mappingOption == DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL)
    {
        rowOffset =  gMmwMssMCB.numDopplerBins * gMmwMssMCB.numAntCol;
    }
    else
    {
        retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
        goto exit;
    }

    /* Initialize tables */
    for (ind = 0; ind < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); ind++)
    {
        BT[ind] = 0;
        SCAL[ind] = 0;
        DONE[ind] = 0;
    }

    for (ind = 0; ind < (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas); ind++)
    {
        row = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
        col = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
        BT[row * gMmwMssMCB.numAntCol + col] = ind;
        SCAL[row * gMmwMssMCB.numAntCol + col] = 1;
    }
    for (row = 0; row < gMmwMssMCB.numAntRow; row++)
    {
        for (col = 0; col < gMmwMssMCB.numAntCol; col++)
        {
            ind = row * gMmwMssMCB.numAntCol + col;
            DT[ind] = row * rowOffset + col;
        }
    }


    /* Configure Doppler HWA mapping params for antenna mapping */
    dopParamInd = 0;
    dopplerParamCfg->numDopFftParams = 0;
    for (ind = 0; ind < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); ind++)
    {
        if (!DONE[ind])
        {
            if(dopParamInd < DPU_DOA_PROC_MAX_NUM_DOP_FFFT_PARAMS)
            {
                DONE[ind] = 1;
                dopplerParamCfg->numDopFftParams++;
                dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt = 1;
                dopplerParamCfg->dopFftCfg[dopParamInd].scale = SCAL[ind];
                if (dopplerParamCfg->dopFftCfg[dopParamInd].scale == 0)
                {
                    dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = 0;
                }
                else
                {
                    dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = BT[ind];
                }
                dopplerParamCfg->dopFftCfg[dopParamInd].dstAddrOffset = DT[ind];
                state = 1;//STATE_SECOND:
                for (indNext = ind+1; indNext < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); indNext++)
                {

                    if (!DONE[indNext] && (dopplerParamCfg->dopFftCfg[dopParamInd].scale == SCAL[indNext]))
                    {
                        switch (state)
                        {
                            case 1://STATE_SECOND:
                                dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                DONE[indNext] = 1;
                                if (SCAL[indNext] == 1)
                                {
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = BT[indNext] - dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset;
                                }
                                else
                                {
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = 0;
                                }
                                dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx = DT[indNext] - DT[ind];
                                indNextPrev = indNext;
                                state = 2;//STATE_NEXT:
                                break;
                            case 2://STATE_NEXT:
                                if (SCAL[indNext] == 1)
                                {
                                    if ((dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx == (BT[indNext] - BT[indNextPrev])) &&
                                        (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev])))
                                    {
                                        DONE[indNext] = 1;
                                        dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                        indNextPrev = indNext;
                                    }
                                }
                                else
                                {
                                    if (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev]))
                                    {
                                        DONE[indNext] = 1;
                                        dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                        indNextPrev = indNext;
                                    }
                                }
                                break;
                        }
                    }
                }
                dopParamInd++;
            }
            else
            {
                retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
                goto exit;
            }
        }
    }

    dopplerParamCfg->numDopFftParams = dopParamInd;

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *     Compress point cloud list which is transferred to the Host via UART.
 *     Floating point values are converted to int16
 *
 * @param[out] pointCloudOut        Compressed point cloud list
 * @param[in]  pointCloudUintRecip  Scales used for conversion from float values to integer value
 * @param[in]  pointCloudIn         Input point cloud list, generated by CFAR DPU
 * @param[in]  numPoints            Number of points in the point cloud list
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_compressPointCloudList(MmwDemo_output_message_UARTpointCloud *pointCloudOut,
                                    MmwDemo_output_message_point_unit *pointCloudUintRecip,
                                    DPIF_PointCloudCartesianExt *pointCloudIn,
                                    uint32_t numPoints)
{
    uint32_t i;
    float xyzUnitScale = pointCloudUintRecip->xyzUnit;
    float dopplerScale = pointCloudUintRecip->dopplerUnit;
    float snrScale = pointCloudUintRecip->snrUint;
    float noiseScale = pointCloudUintRecip->noiseUint;
    uint32_t tempVal;

    for (i = 0; i < numPoints; i++)
    {
        pointCloudOut->point[i].x = (int16_t) roundf(pointCloudIn[i].x * xyzUnitScale); //saturate the values
        pointCloudOut->point[i].y = (int16_t) roundf(pointCloudIn[i].y * xyzUnitScale);
        pointCloudOut->point[i].z = (int16_t) roundf(pointCloudIn[i].z * xyzUnitScale);
        pointCloudOut->point[i].doppler = (int16_t) roundf(pointCloudIn[i].velocity * dopplerScale);
        tempVal = (uint32_t) roundf(pointCloudIn[i].snr * snrScale);
        if (tempVal > 255)
        {
            tempVal = 255;
        }
        pointCloudOut->point[i].snr = (uint8_t) tempVal;
        tempVal = (uint32_t) roundf(pointCloudIn[i].noise * noiseScale);
        if (tempVal > 255)
        {
            tempVal = 255;
        }
        pointCloudOut->point[i].noise = (uint8_t) tempVal;
    }
}

/*  @b Description
*  @n
*    Range processing DPU Initialization
*/
void rangeProc_dpuInit()
{
    int32_t errorCode = 0;
    DPU_RangeProcHWA_InitParams initParams;
    initParams.hwaHandle = hwaHandle;

    /* generate the dpu handler*/
    gMmwMssMCB.rangeProcDpuHandle = DPU_RangeProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.rangeProcDpuHandle == NULL)
    {
        CLI_write("Error: RangeProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert(0);
        return;
    }
}

/*  @b Description
*  @n
*    Doppler processing DPU Initialization
*/
void dopplerProc_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_DopplerProcHWA_InitParams initParams;
    initParams.hwaHandle =  hwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.dopplerProcDpuHandle =  DPU_DopplerProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.dopplerProcDpuHandle == NULL)
    {
        CLI_write ("Error: Doppler Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    CFAR DPU Initialization
*/
void cfarProc_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_CFARProcHWA_InitParams initParams;
    initParams.hwaHandle =  hwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.cfarProcDpuHandle =  DPU_CFARProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.cfarProcDpuHandle == NULL)
    {
        CLI_write ("Error: CFAR Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    AOA2D DPU Initialization
*/
void aoa2dProc_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_Aoa2dProc_InitParams initParams;
    initParams.hwaHandle =  hwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.aoa2dProcDpuHandle =  DPU_Aoa2dProc_init(&initParams, &errorCode);
    if (gMmwMssMCB.aoa2dProcDpuHandle == NULL)
    {
        CLI_write ("Error: AOA2D Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the range processing DPU configurations
*/
int32_t RangeProc_configParser()
{

    int32_t retVal = 0;
    DPU_RangeProcHWA_HW_Resources *pHwConfig = &rangeProcDpuCfg.hwRes;
    DPU_RangeProcHWA_StaticConfig  * params;
    uint32_t index;
    uint32_t bytesPerRxChan;

    /* Rangeproc DPU */
    pHwConfig = &rangeProcDpuCfg.hwRes;
    params = &rangeProcDpuCfg.staticCfg;

    memset((void *)&rangeProcDpuCfg, 0, sizeof(DPU_RangeProcHWA_Config));

    params->enableMajorMotion = gMmwMssMCB.enableMajorMotion;
    params->enableMinorMotion = gMmwMssMCB.enableMinorMotion;

    params->numFramesPerMinorMotProc = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc;
    params->numMinorMotionChirpsPerFrame = gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame;
    params->frmCntrModNumFramesPerMinorMot = gMmwMssMCB.frmCntrModNumFramesPerMinorMot;
    params->lowPowerMode = gMmwMssMCB.lowPowerMode;

    gMmwMssMCB.frmCntrModNumFramesPerMinorMot++;
    if(gMmwMssMCB.frmCntrModNumFramesPerMinorMot == gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc)
    {
        gMmwMssMCB.frmCntrModNumFramesPerMinorMot = 0;
    }

    /* hwi configuration */
    pHwConfig = &rangeProcDpuCfg.hwRes;

    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;
    //pHwConfig->hwaCfg.numParamSet = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS; //This is calculated in the configuration API
    pHwConfig->hwaCfg.hwaWinRamOffset  = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                               mathUtils_pow2roundup(gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/2);//DPC_OBJDET_HWA_RANGE_WINDOW_RAM_OFFSET;
    pHwConfig->hwaCfg.hwaWinSym = HWA_FFT_WINDOW_SYMMETRIC; //we store only lower half
    pHwConfig->hwaCfg.dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;
    pHwConfig->hwaCfg.dmaTrigSrcChan[0] = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);
    pHwConfig->hwaCfg.dmaTrigSrcChan[1] = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);


    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];
    /* edma configuration depends on the interleave or non-interleave */

    /* windowing buffer is fixed, size will change*/
    params->windowSize = sizeof(uint32_t) * ((gMmwMssMCB.profileComCfg.h_NumOfAdcSamples +1 ) / 2); //symmetric window, for real samples
    params->window =  (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         params->windowSize,
                                                         sizeof(uint32_t));
    if (params->window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }

    /* adc buffer buffer, format fixed, interleave, size will change */
    params->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;
    params->ADCBufData.dataProperty.adcBits = 2U; // 12-bit only
    params->ADCBufData.dataProperty.numChirpsPerChirpEvent = 1U;

    #if (CLI_REMOVAL == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        params->ADCBufData.data = (void *)CSL_APP_HWA_ADCBUF_RD_U_BASE;
    }
    else
    {
        gMmwMssMCB.adcTestBuff  = (uint8_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                            MMW_DEMO_TEST_ADC_BUFF_SIZE,
                                                                            sizeof(uint32_t));
        if(gMmwMssMCB.adcTestBuff == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_ADC_TEST_BUFF;
            goto exit;
        }
        params->ADCBufData.data = (void *)gMmwMssMCB.adcTestBuff;

    }
    #else
    params->ADCBufData.data = (void *)CSL_APP_HWA_ADCBUF_RD_U_BASE;
    #endif

    params->numTxAntennas = (uint8_t) gMmwMssMCB.numTxAntennas;
    params->numVirtualAntennas = (uint8_t) (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numChirpsPerFrame = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
    params->numDopplerChirpsPerFrame = params->numChirpsPerFrame/params->numTxAntennas;

    if ((params->numTxAntennas == 1) && (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame !=1))
    {
        retVal = DPC_OBJECTDETECTION_EINVAL_CFG;
        goto exit;
    }
    if ((params->numTxAntennas == 1) && (gMmwMssMCB.isBpmEnabled))
    {
        retVal = DPC_OBJECTDETECTION_EINVAL_CFG;
        goto exit;
    }

    if (params->enableMajorMotion)
    {
        params->numDopplerChirpsPerProc = params->numDopplerChirpsPerFrame;
    }
    else
    {
        params->numDopplerChirpsPerProc = params->numFramesPerMinorMotProc * params->numMinorMotionChirpsPerFrame;
    }

    params->isBpmEnabled = 0;//gMmwMssMCB.isBpmEnabled;
    /* windowing */
    params->ADCBufData.dataProperty.numRxAntennas = (uint8_t) gMmwMssMCB.numRxAntennas;
    params->ADCBufData.dataProperty.numAdcSamples = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples;

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)params->window,
                            (uint32_t) params->ADCBufData.dataProperty.numAdcSamples,
                            params->windowSize/sizeof(uint32_t),
                            DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_RANGE_FFT);
    }
    params->rangeFFTtuning.fftOutputDivShift = 2;
    params->rangeFFTtuning.numLastButterflyStagesToScale = 0; /* no scaling needed as ADC is 16-bit and we have 8 bits to grow */

    params->rangeFftSize = mathUtils_pow2roundup(params->ADCBufData.dataProperty.numAdcSamples);

    bytesPerRxChan = params->ADCBufData.dataProperty.numAdcSamples * sizeof(uint16_t);
    bytesPerRxChan = (bytesPerRxChan + 15) / 16 * 16;

    for (index = 0; index < SYS_COMMON_NUM_RX_CHANNEL; index++)
    {
        params->ADCBufData.dataProperty.rxChanOffset[index] = index * bytesPerRxChan;
    }

    params->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    /* Data Input EDMA */
    pHwConfig->edmaInCfg.dataIn.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    pHwConfig->edmaInCfg.dataIn.channelShadow[0]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PING;
    pHwConfig->edmaInCfg.dataIn.channelShadow[1]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PONG;
    pHwConfig->edmaInCfg.dataIn.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    pHwConfig->edmaInCfg.dataInSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    pHwConfig->edmaInCfg.dataInSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaInCfg.dataInSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = intrObj_rangeProc;

    /* Data Output EDMA */
    pHwConfig->edmaOutCfg.path[0].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_CH;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_0;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_1;
    pHwConfig->edmaOutCfg.path[0].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_0;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_1;
    pHwConfig->edmaOutCfg.path[1].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_EVENT_QUE;

    /* Radar cube compression */
    params->isCompressionEnabled = gMmwMssMCB.cliCompressionCfg.enabled;
    if (params->isCompressionEnabled)
    {
        params->compressCfg.compressionFactor = (uint8_t) (1./gMmwMssMCB.cliCompressionCfg.compressionRatio);
        params->compressCfg.numComplexElements = 2; //2 complex16 range bins
    }

    /* Radar cube Major Motion*/
    if (params->enableMajorMotion)
    {
        gMmwMssMCB.radarCube[0].dataSize = params->numRangeBins * params->numVirtualAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirpsPerProc;
        if (params->isCompressionEnabled)
        {
            gMmwMssMCB.radarCube[0].dataSize = gMmwMssMCB.radarCube[0].dataSize / params->compressCfg.compressionFactor;
        }
        gMmwMssMCB.radarCube[0].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                               gMmwMssMCB.radarCube[0].dataSize,
                                                                               sizeof(uint32_t));
        if(gMmwMssMCB.radarCube[0].data == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }


    }
    else
    {
        gMmwMssMCB.radarCube[0].data  = NULL;
        gMmwMssMCB.radarCube[0].dataSize = 0;
    }
    gMmwMssMCB.radarCube[0].datafmt = DPIF_RADARCUBE_FORMAT_6;
    rangeProcDpuCfg.hwRes.radarCube = gMmwMssMCB.radarCube[0];

    /* Radar cube Minor Motion*/
    if (params->enableMinorMotion)
    {
        gMmwMssMCB.radarCube[1].dataSize = params->numRangeBins * params->numVirtualAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirpsPerProc;
        if (params->isCompressionEnabled)
        {
            gMmwMssMCB.radarCube[1].dataSize = gMmwMssMCB.radarCube[1].dataSize / params->compressCfg.compressionFactor;
        }
        gMmwMssMCB.radarCube[1].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                     gMmwMssMCB.radarCube[1].dataSize,
                                                                                     sizeof(uint32_t));
        if(gMmwMssMCB.radarCube[1].data == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }

    }
    else
    {
        gMmwMssMCB.radarCube[1].data  = NULL;
        gMmwMssMCB.radarCube[1].dataSize = 0;
    }
    gMmwMssMCB.radarCube[1].datafmt = DPIF_RADARCUBE_FORMAT_6;
    rangeProcDpuCfg.hwRes.radarCubeMinMot = gMmwMssMCB.radarCube[1];

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the Doppler processing DPU configurations
*/
int32_t DopplerProc_configParser()
{   
    uint32_t retVal = 0;
    DPU_DopplerProcHWA_HW_Resources  *pHwConfig;
    DPU_DopplerProcHWA_StaticConfig  *dopStaticConfig;
    
    uint32_t sizeElementInBytes;

    pHwConfig = &dopplerProcDpuCfg.hwRes; 
    dopStaticConfig = &dopplerProcDpuCfg.staticCfg; 

    memset((void *)&dopplerProcDpuCfg, 0, sizeof(DPU_DopplerProcHWA_Config));
    
    /* overwrite the common params with the test configuration*/
    /* edma configuration */
    pHwConfig->edmaCfg.edmaHandle  = gEdmaHandle[0];
    dopStaticConfig->numTxAntennas = gMmwMssMCB.numTxAntennas;
    dopStaticConfig->numRxAntennas = gMmwMssMCB.numRxAntennas;
    dopStaticConfig->numVirtualAntennas = gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas;
    dopStaticConfig->numRangeBins = gMmwMssMCB.numRangeBins;
    dopStaticConfig->numDopplerChirps = (gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst*gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame)/gMmwMssMCB.numTxAntennas;
    dopStaticConfig->numDopplerBins = mathUtils_pow2roundup(dopStaticConfig->numDopplerChirps);
    dopStaticConfig->log2NumDopplerBins = mathUtils_ceilLog2(dopStaticConfig->numDopplerBins);
    dopStaticConfig->doppFFT_is16b = 0; //Takes into account of 16-bit complex Doppler FFT
    dopStaticConfig->isDetMatrixLogScale   = false;

    /* windowing */
    pHwConfig->hwaCfg.winRamOffset = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                           dopStaticConfig->numDopplerChirps/2); //dopStaticConfig->numRangeBins;
    pHwConfig->hwaCfg.winSym = HWA_FFT_WINDOW_SYMMETRIC;
    pHwConfig->hwaCfg.windowSize = sizeof(uint32_t) * ((dopStaticConfig->numDopplerChirps + 1) / 2); //symmetric window, for real samples
    pHwConfig->hwaCfg.window =  (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         pHwConfig->hwaCfg.windowSize,
                                                         sizeof(uint32_t));
    if (pHwConfig->hwaCfg.window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW;
        goto exit;
    }
    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)pHwConfig->hwaCfg.window,
                            (uint32_t) dopStaticConfig->numDopplerChirps,
                            pHwConfig->hwaCfg.windowSize/sizeof(uint32_t),
                            DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_DOPPLER_FFT);
    }

    /* Radar cube compression */
    dopStaticConfig->isCompressionEnabled = gMmwMssMCB.cliCompressionCfg.enabled;
    if (dopStaticConfig->isCompressionEnabled)
    {
        dopStaticConfig->compressCfg.compressionFactor = (uint8_t) (1./gMmwMssMCB.cliCompressionCfg.compressionRatio);
        dopStaticConfig->compressCfg.numComplexElements = 2; //2 complex16 range bins
    }

    /* 1D-FFT radar cube */
    pHwConfig->radar_1D_FFT_Cube.dataSize = dopStaticConfig->numRangeBins * dopStaticConfig->numTxAntennas * dopStaticConfig->numRxAntennas * sizeof(cmplx16ReIm_t) * dopStaticConfig->numDopplerChirps;
    if (dopStaticConfig->isCompressionEnabled)
    {
        pHwConfig->radar_1D_FFT_Cube.dataSize /= dopStaticConfig->compressCfg.compressionFactor;
    }
    pHwConfig->radar_1D_FFT_Cube.datafmt = DPIF_RADARCUBE_FORMAT_6;
    pHwConfig->radar_1D_FFT_Cube.data = gMmwMssMCB.radarCube[0].data;
    
    if (dopStaticConfig->isDetMatrixLogScale)
    {
        sizeElementInBytes = sizeof(uint16_t);
    }
    else
    {
        sizeElementInBytes = sizeof(uint32_t);
    }

    /* detection Matrix after 2D-FFT and non-coherent addition*/
    pHwConfig->detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;
    pHwConfig->detMatrix.dataSize = dopStaticConfig->numRangeBins * dopStaticConfig->numDopplerBins * sizeElementInBytes;
    pHwConfig->detMatrix.data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                        pHwConfig->detMatrix.dataSize,
                                                        sizeof(uint32_t));
    if (pHwConfig->detMatrix.data == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
        goto exit;
    }

    /* Data Input EDMA */
    pHwConfig->edmaCfg.edmaIn.ping.channel        = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PING;
    pHwConfig->edmaCfg.edmaIn.pong.channel        = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_CH_PONG;
    pHwConfig->edmaCfg.edmaIn.ping.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PING;
    pHwConfig->edmaCfg.edmaIn.pong.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaIn.ping.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaIn.pong.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_EVENT_QUE_PONG;

    /* Hot signature EDMA */
    pHwConfig->edmaCfg.edmaHotSig.ping.channel    = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.channel    = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_CH_PONG;
    pHwConfig->edmaCfg.edmaHotSig.ping.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.channelShadow  = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_SHADOW_PONG;
    pHwConfig->edmaCfg.edmaHotSig.ping.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PING;
    pHwConfig->edmaCfg.edmaHotSig.pong.eventQueue     = DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_SIG_EVENT_QUE_PING;
    pHwConfig->intrObj = &intrObj_dopplerProc;

    /* Data Output EDMA */
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.channel = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_CH;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.channel = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_CH;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.channelShadow = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_SHADOW;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.channelShadow = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_SHADOW;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.ping.eventQueue = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PING_EVENT_QUE;
    pHwConfig->edmaCfg.edmaOutDetectionMatrix.pong.eventQueue = DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_DETMATRIX_PONG_EVENT_QUE;

    gMmwMssMCB.detMatrix.data = pHwConfig->detMatrix.data;
    gMmwMssMCB.detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;
    gMmwMssMCB.detMatrix.dataSize = pHwConfig->detMatrix.dataSize;
    gMmwMssMCB.numDopplerBins = dopStaticConfig->numDopplerBins;
    
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;
    pHwConfig->hwaCfg.dmaTrigSrcPingChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);
    pHwConfig->hwaCfg.dmaTrigSrcPongChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the CFAR detection processing DPU configurations
*/
int32_t CfarProc_configParser()
{
    int32_t retVal = 0;
    float adcStart, startFreq, slope, bandwidth, centerFreq;
    DPU_CFARProcHWA_HW_Resources *pHwConfig;
    DPU_CFARProcHWA_StaticConfig  *params;

    /* CFARproc DPU based on Range/Doppler heatmap */
    pHwConfig = &cfarProcDpuCfg.res;
    params = &cfarProcDpuCfg.staticCfg;

    memset((void *)&cfarProcDpuCfg, 0, sizeof(DPU_CFARProcHWA_Config));

    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;
    pHwConfig->hwaCfg.dmaTrigSrcChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);

    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];

    /* Data Input EDMA */
    pHwConfig->edmaHwaIn.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH;
    pHwConfig->edmaHwaIn.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SHADOW;
    pHwConfig->edmaHwaIn.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_EVENT_QUE;
    pHwConfig->edmaHwaInSignature.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH;
    pHwConfig->edmaHwaInSignature.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaHwaInSignature.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = &intrObj_cfarProc;

    /* Data Output EDMA */
    pHwConfig->edmaHwaOut.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_CH;
    pHwConfig->edmaHwaOut.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_SHADOW;
    pHwConfig->edmaHwaOut.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_EVENT_QUE;

    /* Give M0 and M1 memory banks for detection matrix scratch. */
    pHwConfig->hwaMemInp = (uint16_t *) CSL_APP_HWA_DMA0_RAM_BANK0_BASE;
    pHwConfig->hwaMemInpSize = (CSL_APP_HWA_BANK_SIZE * 2) / sizeof(uint16_t);

    /* M2 bank: for CFAR detection list */
    pHwConfig->hwaMemOutDetList = (DPU_CFARProcHWA_CfarDetOutput *) CSL_APP_HWA_DMA0_RAM_BANK2_BASE;
    pHwConfig->hwaMemOutDetListSize = CSL_APP_HWA_BANK_SIZE /
                                sizeof(DPU_CFARProcHWA_CfarDetOutput);

    /* M3 bank: for maximum azimuth values per range bin  (range profile) */
    pHwConfig->hwaMemOutRangeProfile = (DPU_CFARProcHWA_HwaMaxOutput *) CSL_APP_HWA_DMA0_RAM_BANK3_BASE;

    /* dynamic config */
    cfarProcDpuCfg.dynCfg.cfarCfg   = &gMmwMssMCB.cfarCfg;
    cfarProcDpuCfg.dynCfg.cfarScndPassCfg   = &gMmwMssMCB.cfarScndPassCfg;
    cfarProcDpuCfg.dynCfg.fovRange  = &gMmwMssMCB.rangeSelCfg;
    cfarProcDpuCfg.dynCfg.fovAoaCfg    = &gMmwMssMCB.fovCfg;

    /* DPU Static config */
    params->detectionHeatmapType = DPU_CFAR_RANGE_DOPPLER_HEATMAP;
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numDopplerBins     = gMmwMssMCB.numDopplerBins;
    params->log2NumDopplerBins = mathUtils_ceilLog2(params->numDopplerBins);

    params->angleDimension        = gMmwMssMCB.angleDimension;
    params->isDetMatrixLogScale   = false;
    params->azimuthFftSize        = gMmwMssMCB.sigProcChainCfg.azimuthFftSize;
    params->elevationFftSize      = gMmwMssMCB.sigProcChainCfg.elevationFftSize;
    params->isStaticClutterRemovalEnabled = gMmwMssMCB.staticClutterRemovalEnable;

    if (params->isDetMatrixLogScale)
    {
        gMmwMssMCB.cfarScndPassCfg.thresholdScale    = (uint32_t) lroundf(gMmwMssMCB.cfarScndPassCfg.threshold_dB * 2048.0 / (log10(2)*20));
        gMmwMssMCB.cfarCfg.thresholdScale    = (uint32_t) lroundf(gMmwMssMCB.cfarCfg.threshold_dB * 2048.0 / (log10(2)*20));
    }
    else
    {
        gMmwMssMCB.cfarScndPassCfg.thresholdScale = (uint32_t) lroundf(pow(10, gMmwMssMCB.cfarScndPassCfg.threshold_dB/20.0) * 16.0);
        gMmwMssMCB.cfarCfg.thresholdScale = (uint32_t) lroundf(pow(10, gMmwMssMCB.cfarCfg.threshold_dB/20.0) * 16.0);
    }

    /* Allocate range profile */
    gMmwMssMCB.rangeProfile = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                      params->numRangeBins * sizeof(uint32_t),
                                                      sizeof(uint32_t));
    if (gMmwMssMCB.rangeProfile == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_PROFILE;
        goto exit;
    }

    pHwConfig->rangeDopplerDetPointBitMap =  (uint32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                                                sizeof(uint32_t) * ((params->numRangeBins * params->numDopplerBins + 31)>>5),
                                                                                sizeof(uint32_t));

    adcStart                        =   (gMmwMssMCB.adcStartTime * 1.e-6);
    startFreq                       =   (float)(gMmwMssMCB.startFreq * 1.e9);
    slope                           =   (float)(gMmwMssMCB.chirpSlope * 1.e12);
    bandwidth                       =   (slope * gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/(gMmwMssMCB.adcSamplingRate * 1.e6);
    centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;

    params->rangeStep            =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                        (2.f * slope * (2*params->numRangeBins));

    if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame > 1)
    {
        /* Burst mode: h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = 2 */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * params->numDopplerBins *
                                            centerFreq * (gMmwMssMCB.burstPeriod * 1e-6));
    }
    else
    {
        /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= 2 */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst *
                                            centerFreq * ((gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime + gMmwMssMCB.profileComCfg.h_ChirpRampEndTime) * 1e-1 * 1e-6));
        if(gMmwMssMCB.frameCfg.c_NumOfChirpsAccum != 0)
        {
            /* When numOfChirpsAccum is greater than zero, the chirping window will increase acccording to numOfChirpsAccum selected. */ 
            params->dopplerStep = params->dopplerStep/gMmwMssMCB.frameCfg.c_NumOfChirpsAccum;
        }
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceXdim == 0.)
    {
        params->lambdaOverDistX = 2.0;
    }
    else
    {
        params->lambdaOverDistX = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceXdim);
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceZdim == 0.)
    {
        params->lambdaOverDistZ = 2.0;
    }
    else
    {
        params->lambdaOverDistZ = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceZdim);
    }

    /* Range bias (m) */
    params->rangeBias = gMmwMssMCB.compRxChannelBiasCfg.rangeBias;

    #if (CLI_REMOVAL == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        //ADC data from file, populate point cloud list with target indices (range/azimuth/elevation/doppler)
        params->enableCfarPointCloudListWithIndices = true;
    }
    else
    {
        params->enableCfarPointCloudListWithIndices = false;
    }
    #else
    params->enableCfarPointCloudListWithIndices = false;
    #endif

    /* hwres config - Copy these structures */
    pHwConfig->dopplerIndexMatrix.data = NULL;
    pHwConfig->dopplerIndexMatrix.dataSize = 0;
    pHwConfig->elevationIndexMatrix.data = NULL;
    pHwConfig->elevationIndexMatrix.dataSize = 0;

    /* For point cloud compression over UART: set the cloud point units and reciprocal values */
    gMmwMssMCB.pointCloudToUart.pointUint.xyzUnit = (params->rangeStep * params->numRangeBins) / 32768.0;
    gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit = (params->dopplerStep * params->numDopplerBins/2) / 32768.0;
    gMmwMssMCB.pointCloudToUart.pointUint.snrUint = 0.25;
    gMmwMssMCB.pointCloudToUart.pointUint.noiseUint = 1.0;
    gMmwMssMCB.pointCloudUintRecip.xyzUnit = 1. / gMmwMssMCB.pointCloudToUart.pointUint.xyzUnit;
    gMmwMssMCB.pointCloudUintRecip.dopplerUnit = 1. / gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit;
    gMmwMssMCB.pointCloudUintRecip.snrUint = 1. / gMmwMssMCB.pointCloudToUart.pointUint.snrUint;  //scale 0.1 since in the CFAR DPU structure it is in 0.1 dB
    gMmwMssMCB.pointCloudUintRecip.noiseUint = 1. / gMmwMssMCB.pointCloudToUart.pointUint.noiseUint;  //scale 0.1 since in the CFAR DPU structure it is in 0.1 dB

exit:
    return retVal;
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the aoa2d processing DPU configurations
*/
int32_t Aoa2dProc_configParser()
{
    /* Aoa2dproc DPU */
    DPU_Aoa2dProc_EdmaCfg *edmaCfg;
    DPU_Aoa2dProc_HwaCfg *hwaCfg;
    int32_t winGenLen, i;
    int32_t retVal = 0;
    DPU_Aoa2dProc_StaticConfig  *aoa2dStaticCfg;
    float adcStart, startFreq, slope, bandwidth, centerFreq;
    DPU_Aoa2dProc_HW_Resources  *hwRes;

    memset((void *)&aoa2dProcDpuCfg, 0, sizeof(DPU_Aoa2dProc_Config));

    hwRes = &aoa2dProcDpuCfg.hwRes;
    aoa2dStaticCfg = &aoa2dProcDpuCfg.staticCfg;
    edmaCfg = &hwRes->edmaCfg;
    hwaCfg = &hwRes->hwaCfg;


    aoa2dStaticCfg->numAntRow = gMmwMssMCB.numAntRow;
    aoa2dStaticCfg->numAntCol = gMmwMssMCB.numAntCol;

    /* FOV configuration */
    aoa2dStaticCfg->fovAoaCfg = &gMmwMssMCB.fovAoaCfg;

    /* From CFAR configuration */
    aoa2dStaticCfg->sideLobeThresholdScaleQ8 = gMmwMssMCB.cfarCfg.sideLobeThresholdScaleQ8;
    /* Always true: */
    aoa2dStaticCfg->enableInterpAzimuthDom = true;

    aoa2dStaticCfg->isDetMatrixLogScale = false;

    /* Radar cube compression */
    aoa2dStaticCfg->isCompressionEnabled = gMmwMssMCB.cliCompressionCfg.enabled;
    if (aoa2dStaticCfg->isCompressionEnabled)
    {
        aoa2dStaticCfg->compressCfg.compressionFactor = (uint8_t) (1./gMmwMssMCB.cliCompressionCfg.compressionRatio);
        aoa2dStaticCfg->compressCfg.numComplexElements = 2; //2 complex16 range bins
    }

    adcStart                        =   (gMmwMssMCB.adcStartTime * 1.e-6);
    startFreq                       =   (float)(gMmwMssMCB.startFreq * 1.e9);
    slope                           =   (float)(gMmwMssMCB.chirpSlope * 1.e12);
    bandwidth                       =   (slope * gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/(gMmwMssMCB.adcSamplingRate * 1.e6);
    centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;


    if (gMmwMssMCB.antennaGeometryCfg.antDistanceXdim == 0.)
    {
        aoa2dStaticCfg->lambdaOverDistX = 2.0;
    }
    else
    {
        aoa2dStaticCfg->lambdaOverDistX = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceXdim);
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceZdim == 0.)
    {
        aoa2dStaticCfg->lambdaOverDistZ = 2.0;
    }
    else
    {
        aoa2dStaticCfg->lambdaOverDistZ = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceZdim);
    }

    aoa2dStaticCfg->numTxAntennas = (uint8_t) gMmwMssMCB.numTxAntennas;
    aoa2dStaticCfg->numRxAntennas = (uint8_t) gMmwMssMCB.numRxAntennas;
    aoa2dStaticCfg->numVirtualAntennas = (uint8_t) (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    aoa2dStaticCfg->numRangeBins = gMmwMssMCB.numRangeBins;


    if (aoa2dStaticCfg->numTxAntennas > 1)
    {
        if ((gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame > 1) && (gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst == 2))
        {
            /* Burst mode: h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = 2 */
            aoa2dStaticCfg->numDopplerChirps   = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
        }
        else if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame == 1)
        {
            /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= aoa2dStaticCfg->numTxAntennas */
            aoa2dStaticCfg->numDopplerChirps   = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst / aoa2dStaticCfg->numTxAntennas;
        }
        else
        {
            retVal = DPU_DOAPROC_EINVAL;
            goto exit;
        }
    }
    else
    {
        /* 1Tx antenna */
        if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame != 1)
        {
            retVal = DPU_DOAPROC_EINVAL;
            goto exit;
        }
        aoa2dStaticCfg->numDopplerChirps = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
    }


    aoa2dStaticCfg->numDopplerBins     = mathUtils_pow2roundup(aoa2dStaticCfg->numDopplerChirps);
    aoa2dStaticCfg->log2NumDopplerBins = mathUtils_ceilLog2(aoa2dStaticCfg->numDopplerBins);

    aoa2dStaticCfg->numBpmDopplerChirps = aoa2dStaticCfg->numTxAntennas * aoa2dStaticCfg->numDopplerChirps;
    aoa2dStaticCfg->bpmDopplerFftSize = aoa2dStaticCfg->numTxAntennas * aoa2dStaticCfg->numDopplerBins;
    aoa2dStaticCfg->log2BpmDopplerFftSize = mathUtils_ceilLog2(aoa2dStaticCfg->bpmDopplerFftSize);

    gMmwMssMCB.numDopplerBins = aoa2dStaticCfg->numDopplerBins; //We save this for later, it will be used by CFAR.

    aoa2dStaticCfg->angleDimension        = gMmwMssMCB.angleDimension;
    //aoa2dStaticCfg->isDetMatrixLogScale   = false;
    aoa2dStaticCfg->azimuthFftSize        = gMmwMssMCB.sigProcChainCfg.azimuthFftSize;
    aoa2dStaticCfg->elevationFftSize      = gMmwMssMCB.sigProcChainCfg.elevationFftSize;
    aoa2dStaticCfg->isStaticClutterRemovalEnabled = gMmwMssMCB.staticClutterRemovalEnable;

    /* Configure Doppler fft param sets for mapping antennas into 1D/2D virtual antenna array */
    retVal = MmwDemo_cfgDopplerParamMapping(&hwaCfg->aoa2dRngGateCfg, DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL);
    if (retVal < 0)
    {
        goto exit;
    }

    /* hwRes - copy these structures */
    hwRes->radarCube = gMmwMssMCB.radarCube[0];

    /* hwRes - edmaCfg */
    edmaCfg->edmaHandle = gEdmaHandle[0];

    /* edmaIn */
    edmaCfg->edmaIn.channel =            DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_CH;
    edmaCfg->edmaIn.channelShadow =      DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_SHADOW;
    edmaCfg->edmaIn.eventQueue =         DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_EVENT_QUE;

    /* edmaHotSig */
    edmaCfg->edmaHotSig.channel =             DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_SIG_CH;
    edmaCfg->edmaHotSig.channelShadow =       DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_SIG_SHADOW;
    edmaCfg->edmaHotSig.eventQueue =          DPC_OBJDET_DPU_AOA2D_PROC_EDMAIN_SIG_EVENT_QUE;

    edmaCfg->intrObj = &intrObj_aoa2dProc;

    /* hwaCfg */
    hwaCfg->hwaMemInpAddr = CSL_APP_HWA_DMA0_RAM_BANK0_BASE;
    hwaCfg->hwaMemOutAddr = CSL_APP_HWA_DMA0_RAM_BANK1_BASE;
    hwaCfg->numParamSets = 0;  //The number depends on the configuration, will be populated by DPU_Aoa2dProc_config()
    hwaCfg->paramSetStartIdx = gMmwMssMCB.numUsedHwaParamSets;

    hwaCfg->dmaTrigSrcChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);

    /* hwaCfg - window */
    /* Doppler FFT window */
    hwaCfg->dopplerWindow.winRamOffset = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                               aoa2dStaticCfg->numBpmDopplerChirps / 2);//aoa2dStaticCfg->numRangeBins + (aoa2dStaticCfg->numDopplerBins/2) ;
    hwaCfg->dopplerWindow.winSym = HWA_FFT_WINDOW_SYMMETRIC; // Symmetric window, store only half
    hwaCfg->dopplerWindow.windowSize = sizeof(uint32_t) * aoa2dStaticCfg->numBpmDopplerChirps / 2; //symmetric window, for real samples (size in bytes)
    hwaCfg->dopplerWindow.window =  (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                                       hwaCfg->dopplerWindow.windowSize,
                                                                       sizeof(uint32_t));
    if (hwaCfg->dopplerWindow.window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA2D_HWA_WINDOW;
        goto exit;
    }
    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)hwaCfg->dopplerWindow.window,
                            (uint32_t) aoa2dStaticCfg->numBpmDopplerChirps,
                            hwaCfg->dopplerWindow.windowSize/sizeof(uint32_t),
                            DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_DOPPLER_FFT);
    }



    /* Azimuth and elevation FFT window */
    //Share FFT window between azimuth and elevation FFT, window = [+1 -1 +1 -1 ... ]
    winGenLen = (aoa2dStaticCfg->azimuthFftSize > aoa2dStaticCfg->elevationFftSize) ? aoa2dStaticCfg->azimuthFftSize : aoa2dStaticCfg->elevationFftSize;
    hwaCfg->angleWindow.windowSize = winGenLen * sizeof(int32_t);

    hwaCfg->angleWindow.window = (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                                    hwaCfg->angleWindow.windowSize,
                                                                    sizeof(uint32_t));
    if (hwaCfg->angleWindow.window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA2D_HWA_WINDOW;
        goto exit;
    }
    /*Alternate 1,-1,...*/
    for (i=0; i<winGenLen; i++)
    {
        hwaCfg->angleWindow.window[i] = (1 - 2 * (i & 0x1)) * ((1<<17) - 1);
    }

    hwaCfg->angleWindow.winRamOffset = DPC_ObjDet_HwaWinRamMemoryPoolAlloc(&gMmwMssMCB.HwaWinRamMemoryPoolObj,
                                                                             winGenLen);//aoa2dStaticCfg->numRangeBins + (aoa2dStaticCfg->numDopplerBins/2) + (aoa2dStaticCfg->bpmDopplerFftSize/2);
    hwaCfg->angleWindow.winSym = HWA_FFT_WINDOW_NONSYMMETRIC;

    /* Rx compensation coefficients */
    {
        int32_t rxInd, txInd;
        int32_t ind = 0;
        aoa2dStaticCfg->compRxChanCfg.rangeBias = gMmwMssMCB.compRxChannelBiasCfg.rangeBias;
        for (txInd = 0; txInd < aoa2dStaticCfg->numTxAntennas; txInd++)
        {
            for (rxInd = 0; rxInd < aoa2dStaticCfg->numRxAntennas; rxInd++)
            {
                aoa2dStaticCfg->compRxChanCfg.rxChPhaseComp[ind++] = gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[gMmwMssMCB.rxAntOrder[rxInd] + (txInd * SYS_COMMON_NUM_RX_CHANNEL)]; 
            }
        }
    }

exit:
return retVal;
}

/**
*  @b Description
*  @n
*        Function configuring range processing DPU
*/
void mmwDemo_rangeProcConfig()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = RangeProc_configParser();
    if (retVal < 0)
    {
        CLI_write("Error in setting up range profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_RangeProcHWA_config(gMmwMssMCB.rangeProcDpuHandle, &rangeProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("Error: RANGE DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    retVal = DPU_RangeProcHWA_GetNumUsedHwaParamSets(gMmwMssMCB.rangeProcDpuHandle, &numUsedHwaParamSets);
    if (retVal < 0)
    {
        CLI_write("Error: RANGE DPU return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring dopplerproc
*/
void mmwDemo_dopplerProcConfig()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = DopplerProc_configParser();
    if (retVal < 0)
    {
        CLI_write("Error: Error in setting up doppler profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_DopplerProcHWA_config (gMmwMssMCB.dopplerProcDpuHandle, &dopplerProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("Doppler DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    retVal = DPU_DopplerProcHWA_GetNumUsedHwaParamSets(gMmwMssMCB.dopplerProcDpuHandle, &numUsedHwaParamSets);
    if (retVal < 0)
    {
        CLI_write("Doppler DPU return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring CFAR DPU
*/
void mmwDemo_cfarProcConfig()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = CfarProc_configParser();
    if (retVal < 0)
    {
        CLI_write("Error in setting up CFAR profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_CFARProcHWA_config(gMmwMssMCB.cfarProcDpuHandle, &cfarProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("CFAR DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    retVal = DPU_CFARProcHWA_GetNumUsedHwaParamSets(gMmwMssMCB.cfarProcDpuHandle, &numUsedHwaParamSets);
    if (retVal < 0)
    {
        CLI_write("CFAR DPU return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function configuring AOA2D DPU
*/
void mmwDemo_aoa2dProcConfig()
{
    int32_t retVal = 0;
    uint8_t numUsedHwaParamSets;

    retVal = Aoa2dProc_configParser();
    if (retVal < 0)
    {
        CLI_write("Error: Error in setting up aoa2d profile:%d \n", retVal);
        DebugP_assert(0);
    }

    retVal = DPU_Aoa2dProc_config (gMmwMssMCB.aoa2dProcDpuHandle, &aoa2dProcDpuCfg);
    if (retVal < 0)
    {
        CLI_write("AOA2D DPU config return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Get number of used HWA param sets by this DPU */
    retVal = DPU_Aoa2dProc_GetNumUsedHwaParamSets(gMmwMssMCB.aoa2dProcDpuHandle, &numUsedHwaParamSets);
    if (retVal < 0)
    {
        CLI_write("AOA2D DPU return error:%d \n", retVal);
        DebugP_assert(0);
    }

    /* Update number of used HWA param sets */
    gMmwMssMCB.numUsedHwaParamSets += numUsedHwaParamSets;
}

/**
*  @b Description
*  @n
*        Function initiliazing all indvidual DPUs
*/
void DPC_Init()
{
    /* hwa, edma, and DPU initialization*/

    /* Register Frame Start Interrupt */
    if(mmwDemo_registerFrameStartInterrupt() != 0){
        CLI_write("Error: Failed to register frame start interrupts\n");
        DebugP_assert(0);
    }
/* For debugging purposes*/
#if 0
    if(mmwDemo_registerChirpAvailableInterrupts() != 0){
        CLI_write("Failed to register chirp available interrupts\n");
        DebugP_assert(0);
    }
    mmwDemo_registerChirpInterrupt();
    mmwDemo_registerBurstInterrupt();
#endif
    int32_t status = SystemP_SUCCESS;

    /* Shared memory pool */
    gMmwMssMCB.L3RamObj.cfg.addr = (void *)&gMmwL3[0];
    gMmwMssMCB.L3RamObj.cfg.size = sizeof(gMmwL3);

    /* Local memory pool */
    gMmwMssMCB.CoreLocalRamObj.cfg.addr = (void *)&gMmwCoreLocMem[0];
    gMmwMssMCB.CoreLocalRamObj.cfg.size = sizeof(gMmwCoreLocMem);
/* For debugging purposes*/
#if 0
    /* Memory pool for the tracker */
    HeapP_construct(&gMmwMssMCB.CoreLocalTrackerHeapObj, (void *) gMmwCoreLocMem2, MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE);

    /* Memory pool for the feature extraction */
    featExtract_heapConstruct();
#endif

    hwaHandle = HWA_open(0, NULL, &status);
    if (hwaHandle == NULL)
    {
        CLI_write("Error: Unable to open the HWA Instance err:%d\n", status);
        DebugP_assert(0);
    }

    rangeProc_dpuInit();
    dopplerProc_dpuInit();
    cfarProc_dpuInit();
    aoa2dProc_dpuInit();
}


/**
*  @b Description
*  @n

*        Function configuring all DPUs
*/
void DPC_Config()
{

    int32_t retVal;

    /*TODO Cleanup: MMWLPSDK-237*/
    
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.L3RamObj);
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.CoreLocalRamObj);
    DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(&gMmwMssMCB.HwaDmaChanPoolObj);
    DPC_ObjDet_HwaWinRamMemoryPoolReset(&gMmwMssMCB.HwaWinRamMemoryPoolObj);

    /*Allocate memory for point cloud */
    gMmwMssMCB.cfarDetObjOut = (DPIF_PointCloudCartesianExt *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudCartesianExt),
                                                                                       sizeof(uint32_t));
    if (gMmwMssMCB.cfarDetObjOut == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    /*Allocate memory for CFAR Range/Doppler detection output*/
    gMmwMssMCB.detRngDopList = (DPIF_CFARRngDopDetListElement *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_CFARRngDopDetListElement),
                                                                                       sizeof(uint32_t));
    if (gMmwMssMCB.detRngDopList == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    /*Allocate memory for CFAR Range/Doppler detection output*/
    gMmwMssMCB.detectedRangeGates.array = (DPIF_DetectedRangeGate *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_DetectedRangeGate),
                                                                                       sizeof(uint32_t));
    if (gMmwMssMCB.detectedRangeGates.array == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }
    gMmwMssMCB.detectedRangeGates.numRangeGates = 0;
    gMmwMssMCB.detectedRangeGates.maxNumRangeGates = MAX_NUM_DETECTIONS;


    gMmwMssMCB.dpcObjOut = (DPIF_PointCloudCartesian *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudCartesian),
                                                                                       sizeof(uint32_t));
    if (gMmwMssMCB.dpcObjOut == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    gMmwMssMCB.dpcObjSideInfo = (DPIF_PointCloudSideInfo *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                       MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudSideInfo),
                                                                                       sizeof(uint32_t));
    if (gMmwMssMCB.dpcObjSideInfo == NULL)
    {
        CLI_write("DPC configuration: memory allocation failed\n");
        DebugP_assert(0);
    }

    #if (CLI_REMOVAL == 0)
    if (gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        gMmwMssMCB.dpcObjIndOut = (DPIF_PointCloudRngAzimElevDopInd *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                           MAX_NUM_DETECTIONS * sizeof(DPIF_PointCloudRngAzimElevDopInd),
                                                                                           sizeof(uint32_t));
        if (gMmwMssMCB.dpcObjIndOut == NULL)
        {
            CLI_write("DPC configuration: memory allocation failed\n");
            DebugP_assert(0);
        }
    }
    #endif

    /* Select active antennas from available antennas and calculate number of antennas rows and columns */
    MmwDemo_calcActiveAntennaGeometry();

    /* Angle dimension */
    if ((gMmwMssMCB.numAntRow > 1) && (gMmwMssMCB.numAntCol > 1))
    {
        gMmwMssMCB.angleDimension = 2;
    }
    else if ((gMmwMssMCB.numAntRow == 1) && (gMmwMssMCB.numAntCol > 1))
    {
        gMmwMssMCB.angleDimension = 1;
    }
    else
    {
        gMmwMssMCB.angleDimension = 0;
    }

    /* Configure DPUs */
    gMmwMssMCB.numUsedHwaParamSets = 0;
    mmwDemo_rangeProcConfig();
    mmwDemo_dopplerProcConfig();
    mmwDemo_cfarProcConfig();
    mmwDemo_aoa2dProcConfig();

    if(gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
    {
        retVal = mmwDemo_rangeBiasRxChPhaseMeasureConfig();
        if (retVal != 0)
        {
            CLI_write("DPC configuration: Invalid Rx channel compensation procedure configuration \n");
            DebugP_assert(0);
        }
    }

    if (!gMmwMssMCB.oneTimeConfigDone)
    {

        /* Report RAM usage */
        gMmwMssMCB.memUsage.CoreLocalRamUsage = DPC_ObjDet_MemPoolGetMaxUsage(&gMmwMssMCB.CoreLocalRamObj);
        gMmwMssMCB.memUsage.L3RamUsage = DPC_ObjDet_MemPoolGetMaxUsage(&gMmwMssMCB.L3RamObj);
        HeapP_getHeapStats(&gMmwMssMCB.CoreLocalTrackerHeapObj, &gMmwMssMCB.memUsage.trackerHeapStats);

        gMmwMssMCB.memUsage.L3RamTotal = gMmwMssMCB.L3RamObj.cfg.size;
        gMmwMssMCB.memUsage.CoreLocalRamTotal = gMmwMssMCB.CoreLocalRamObj.cfg.size;
    
        if(gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_DISABLE)
        {
            DebugP_log(" ========== Memory Stats ==========\n");
            DebugP_log("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");

            DebugP_log("%20s %12d %12d %12d\n", "L3",
                      sizeof(gMmwL3),
                      gMmwMssMCB.memUsage.L3RamUsage,
                      sizeof(gMmwL3) - gMmwMssMCB.memUsage.L3RamUsage);

            DebugP_log("%20s %12d %12d %12d\n", "Local",
                      sizeof(gMmwCoreLocMem),
                      gMmwMssMCB.memUsage.CoreLocalRamUsage,
                      sizeof(gMmwCoreLocMem) - gMmwMssMCB.memUsage.CoreLocalRamUsage);
        }
    }

}

/**
 *  @b Description
 *  @n  DPC processing chain execute function.
 *
 */
void DPC_Execute(){
    int32_t retVal;
    int32_t errCode = 0;
    int32_t i;
    DPU_RangeProcHWA_OutParams outParms;
    DPU_DopplerProcHWA_OutParams outParmsDoppler;
    DPU_CFARProcHWA_OutParams outParmsCfar;
    DPU_Aoa2dProc_OutParams outParmsAoa2d;
    #if (SPI_ADC_DATA_STREAMING==1)
    MCSPI_Transaction   spiTransaction;
    int32_t             transferOK;
    uint32_t totalSizeToTfr,tempSize;
    uint8_t count;
    #endif
    uint8_t enableMajorMotion;
    uint8_t enableMinorMotion;
    DPC_ObjectDetection_ExecuteResult *result = &gMmwMssMCB.dpcResult;
    uint32_t numDetectedPoints; //numDetectedPoints[0] - Number of points in major motion detection, numDetectedPoints[0] - Number of points in minor motion detection,
    /* give initial trigger for the first frame */
    errCode = DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle,
                 DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if(errCode < 0)
    {
        CLI_write("Error: Range control execution failed [Error code %d]\n", errCode);
    }

    if (gMmwMssMCB.sigProcChainCfg.motDetMode == 1)
    {
        enableMajorMotion = 1;
        enableMinorMotion = 0;
    }
    else if (gMmwMssMCB.sigProcChainCfg.motDetMode == 3)
    {
        enableMajorMotion = 1;
        enableMinorMotion = 1;
    }
    else
    {
        enableMajorMotion = 0;
        enableMinorMotion = 1;
    }
    if (enableMajorMotion)
    {
        result->rngAzHeatMap[MMW_DEMO_MAJOR_MODE] = (uint32_t *) gMmwMssMCB.detMatrix.data;
    }
    else
    {
        result->rngAzHeatMap[MMW_DEMO_MAJOR_MODE] = NULL;
    }
    if (enableMinorMotion)
    {
        result->rngAzHeatMap[MMW_DEMO_MINOR_MODE] = (uint32_t *) gMmwMssMCB.detMatrix.data;
    }
    else
    {
        result->rngAzHeatMap[MMW_DEMO_MINOR_MODE] = NULL;
    }

    result->objOut = gMmwMssMCB.dpcObjOut;
    result->objOutSideInfo = gMmwMssMCB.dpcObjSideInfo;
    result->rngDopplerHeatMap = (uint32_t *) gMmwMssMCB.detMatrix.data;

    /* Send signal to CLI task that this is ready */
    SemaphoreP_post(&gMmwMssMCB.dpcTaskConfigDoneSemHandle);

    while(true){

        memset((void *)&outParms, 0, sizeof(DPU_RangeProcHWA_OutParams));
        retVal = DPU_RangeProcHWA_process(gMmwMssMCB.rangeProcDpuHandle, &outParms);
        if(retVal != 0){
            CLI_write("DPU_RangeProcHWA_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        /***************************ADC Streaming Via SPI***********************************************************/

        #if (SPI_ADC_DATA_STREAMING==1)

            if( gMmwMssMCB.spiADCStream == 1)
            {
                uint32_t* adc_data = (uint32_t*)adcbuffer;
                totalSizeToTfr = adcDataPerFrame;
                tempSize = adcDataPerFrame;
                count = 0;
                while(totalSizeToTfr > 0)
                {
                    if(totalSizeToTfr > MAXSPISIZEFTDI)
                    {
                        tempSize=MAXSPISIZEFTDI;
                    }
                    else
                    {
                        tempSize = totalSizeToTfr;
                    }
                    
                    MCSPI_Transaction_init(&spiTransaction);
                    spiTransaction.channel  = gConfigMcspi0ChCfg[0].chNum;
                    spiTransaction.dataSize = 32;
                    spiTransaction.csDisable = TRUE;
                    spiTransaction.count    = tempSize/4;
                    spiTransaction.txBuf    = (void *)(&adc_data[(MAXSPISIZEFTDI/4)*count]);
                    spiTransaction.rxBuf    = NULL;
                    spiTransaction.args     = NULL;

                    GPIO_pinWriteLow(gpioBaseAddrLed, pinNumLed);
                
                    transferOK = MCSPI_transfer(gMcspiHandle[CONFIG_MCSPI0], &spiTransaction);
                    
                    if(transferOK != 0)
                    {
                        CLI_write("SPI Raw Data Transfer Failed\r\n");
                    }
                    GPIO_pinWriteHigh(gpioBaseAddrLed, pinNumLed);
                    totalSizeToTfr  =   totalSizeToTfr  -   tempSize;
                    count++;
                }

            }
        #endif
    /********************************************************************/
        #if (ENABLE_GPADC==1U)
        // Read the GPADC Data
        MMWave_readGPADC(&gMmwMssMCB.GPADCVal[0],&gMmwMssMCB.GPADCVal[1]);
       //CLI_write("\r\n GPADC 1 Reading: %f\r\n",gMmwMssMCB.GPADCVal[0]);
       //CLI_write("\r\n GPADC 2 Reading: %f\r\n",gMmwMssMCB.GPADCVal[1]);
       #endif

        // Read the temperature
        MMWave_getTemperatureReport(&tempStats);
        
        #if (ENABLE_MONITORS==1)
        // If atleast one monitor is enabled.
        if(gMmwMssMCB.rfMonEnbl != 0)
        {
            // Enable Monitors configured (They have to be enabled only during frame idle time)
            MMWave_enableMonitors(gMmwMssMCB.ctrlHandle);
        }
        /* If Synth Frequency Monitor is enabled read the value*/
        if((gMmwMssMCB.sensorStart.frameLivMonEn & 0x1) == 0x1)
        {
            gMmwMssMCB.rfMonRes.synthFreqres = MMWaveMon_getSynthFreqMonres();
            #if (PRINT_MON_RES == 1)
            CLI_write("Synth Frequency monitor: %x \r\n",gMmwMssMCB.rfMonRes.synthFreqres.status);
            #endif
        }

        /* If Rx Sat Live Monitor is enabled read the value*/
        if((gMmwMssMCB.sensorStart.frameLivMonEn & 0x2) == 0x2)
        {
            /*Base Address of Rx Saturation Live Monitor Results */
            uint32_t *baseAddrRxSatLive;
            gMmwMssMCB.rfMonRes.rxSatLiveres.rxSatLivePtr = MMWaveMon_getRxSatLiveMonres(); 
            /*Each chirp will have 1 byte data for each RX, max 4 RX is supported (4th Rx is reserved)*/
            for (int i=0; i<(gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame);i+=1U)
            {
                /* Incrementing Address by 32 bits */
                baseAddrRxSatLive = gMmwMssMCB.rfMonRes.rxSatLiveres.rxSatLivePtr + i;
                /*Checking Rx Saturation Live Monitor status*/
                if(*baseAddrRxSatLive==0)
                {
                    gMmwMssMCB.rfMonRes.status_rxSatLive=0x1; // No saturation
                }
                else
                {
                    gMmwMssMCB.rfMonRes.status_rxSatLive=0x0;
                    break;
                }
            }
            #if (PRINT_MON_RES == 1)
            if(gMmwMssMCB.rfMonRes.status_rxSatLive == 0)
            {
                CLI_write("Rx Saturation Live monitor: Saturation is occurring \r\n");
            }
            else
            {
                CLI_write("Rx Saturation Live monitor: No Saturation  \r\n");
            }
            #endif
        }
        #endif
        
        /* Chirping finished start interframe processing */
        gMmwMssMCB.stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();
        gMmwMssMCB.stats.chirpingTime_us = (gMmwMssMCB.stats.interFrameStartTimeStamp - gMmwMssMCB.stats.frameStartTimeStamp)/FRAME_REF_TIMER_CLOCK_MHZ;

        if((gMmwMssMCB.lowPowerMode == LOW_PWR_MODE_ENABLE) || (gMmwMssMCB.lowPowerMode == LOW_PWR_TEST_MODE))
        {
            //Shutdown the FECSS after chirping
            // Retain FECSS Code Memory
            int32_t err;

            if(pgVersion==1)
            {
                PRCMSetSRAMRetention((PRCM_FEC_PD_SRAM_CLUSTER_2 | PRCM_FEC_PD_SRAM_CLUSTER_3), PRCM_SRAM_LPDS_RET);
            }
            else
            {
                PRCMSetSRAMRetention((PRCM_FEC_PD_SRAM_CLUSTER_1), PRCM_SRAM_LPDS_RET);
            }
            
            //Reset The FrameTimer for next frame
            HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_BLOCKRESET2, 0x1c0);
            for(int i =0;i<10;i++)
            {
                test = PRCMSlowClkCtrGet();
            }
            HW_WR_REG32(CSL_APP_RCM_U_BASE + CSL_APP_RCM_BLOCKRESET2, 0x0);

            /* Delay to account for RFS Processing time before shutting it down */
            ClockP_usleep(RFS_PROC_END_TIME);
            
            #if (ENABLE_MONITORS==1)
            // If atleast one monitor is enabled ,wait till monitors are complete
            if(gMmwMssMCB.rfMonEnbl != 0)
            {
                SemaphoreP_pend(&gMmwMssMCB.rfmonSemHandle, SystemP_WAIT_FOREVER);
            }
            #endif

            // MMW Closure in preparation for Low power state
            MMWave_stop(gMmwMssMCB.ctrlHandle,&err);
            MMWave_close(gMmwMssMCB.ctrlHandle,&err);
            MMWave_deinit(gMmwMssMCB.ctrlHandle,&err);
            /* As the Frame timer is reset, Capture the Inter Frame Start Time again */
            gMmwMssMCB.stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();
        }
        
         /* Procedure for range bias measurement and Rx channels gain/phase offset measurement */
        if(gMmwMssMCB.measureRxChannelBiasCliCfg.enabled)
        {
                mmwDemo_rangeBiasRxChPhaseMeasure();
        }
        
        /* Doppler DPU */
        memset((void *)&outParmsDoppler, 0, sizeof(DPU_DopplerProcHWA_OutParams));
        retVal = DPU_DopplerProcHWA_process(gMmwMssMCB.dopplerProcDpuHandle, &outParmsDoppler);
        if(retVal != 0){
            CLI_write("DPU_DopplerProc_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        /* CFAR DPU */
        numDetectedPoints = 0;
        memset((void *)&outParmsCfar, 0, sizeof(DPU_CFARProcHWA_OutParams));
        outParmsCfar.rangeProfile = gMmwMssMCB.rangeProfile;
        outParmsCfar.detObjOut = gMmwMssMCB.cfarDetObjOut;
        outParmsCfar.detObjIndOut = gMmwMssMCB.dpcObjIndOut;
        outParmsCfar.detObjOutMaxSize = MAX_NUM_DETECTIONS;
        outParmsCfar.detRngDopList = gMmwMssMCB.detRngDopList;

        retVal = DPU_CFARProcHWA_process(gMmwMssMCB.cfarProcDpuHandle,
                                         &gMmwMssMCB.detMatrix,
                                         &gMmwMssMCB.cfarRunTimeInputParams,
                                         &outParmsCfar);
        numDetectedPoints = outParmsCfar.numCfarDetectedPoints;

        result->numObjOut = numDetectedPoints;
        if(retVal != 0){
            CLI_write("DPU_CFARProcHWA_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        /* Prepare range gates for AoA */
        memset((void *)&outParmsAoa2d, 0, sizeof(DPU_Aoa2dProc_OutParams));
        outParmsAoa2d.detObjOut = gMmwMssMCB.cfarDetObjOut;
        outParmsAoa2d.detObjOutMaxSize = MAX_NUM_DETECTIONS;

        retVal = DPU_Aoa2dProc_process(gMmwMssMCB.aoa2dProcDpuHandle,
                                       &gMmwMssMCB.radarCube[0],
                                       gMmwMssMCB.detRngDopList,
                                       numDetectedPoints,
                                       &gMmwMssMCB.detectedRangeGates,
                                       &gMmwMssMCB.detMatrix,
                                       &outParmsAoa2d);
        if(retVal != 0){
            CLI_write("DPU_Aoa2dProc_process failed with error code %d", retVal);
            DebugP_assert(0);
        }

        result->numObjOut = outParmsAoa2d.numDetectedPoints;

#if 1
        if(gMmwMssMCB.guiMonSel.pointCloud == 1)
        {
            for(i=0; i < result->numObjOut; i++)
            {
                result->objOut[i].x = gMmwMssMCB.cfarDetObjOut[i].x;
                result->objOut[i].y = gMmwMssMCB.cfarDetObjOut[i].y;
                result->objOut[i].z = gMmwMssMCB.cfarDetObjOut[i].z;
                result->objOut[i].velocity = gMmwMssMCB.cfarDetObjOut[i].velocity;
                result->objOutSideInfo[i].snr = (int16_t) (10. * gMmwMssMCB.cfarDetObjOut[i].snr); //steps of 0.1dB
                result->objOutSideInfo[i].noise = (int16_t) (10. * gMmwMssMCB.cfarDetObjOut[i].noise); //steps of 0.1dB
            }
        }

        if(gMmwMssMCB.guiMonSel.pointCloud == 2)
        {
            /* Compress point cloud list (data converted from floating point to fix point) */
            MmwDemo_compressPointCloudList(&gMmwMssMCB.pointCloudToUart,
                                           &gMmwMssMCB.pointCloudUintRecip,
                                           gMmwMssMCB.cfarDetObjOut,
                                           result->numObjOut);
            gMmwMssMCB.pointCloudToUart.pointUint.numDetectedPoints[0] = result->numObjOut; //Number of points in major motion mode
            gMmwMssMCB.pointCloudToUart.pointUint.numDetectedPoints[1] = 0; //Number of points in minor motion mode
        }
#endif
        #if (CLI_REMOVAL == 0)
        if (gMmwMssMCB.adcDataSourceCfg.source == 2)
        {
            ClockP_sleep(1);
        }
        #endif
        
        /* Give initial trigger for the next frame */
        retVal = DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle,
                    DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
        if(retVal < 0)
        {
            CLI_write("Error: DPU_RangeProcHWA_control failed with error code %d", retVal);
            DebugP_assert(0);
        }


        /* Interframe processing finished */
        gMmwMssMCB.stats.interFrameEndTimeStamp = Cycleprofiler_getTimeStamp();
        gMmwMssMCB.outStats.interFrameProcessingTime = (gMmwMssMCB.stats.interFrameEndTimeStamp - gMmwMssMCB.stats.interFrameStartTimeStamp)/FRAME_REF_TIMER_CLOCK_MHZ;

        /* Trigger UART task to send TLVs to host */
        SemaphoreP_post(&gMmwMssMCB.tlvSemHandle);
    }
}