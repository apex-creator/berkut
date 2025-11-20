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
#ifndef MMWAVE_DEMO_H
#define MMWAVE_DEMO_H

#include <drivers/uart.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/HeapP.h>
#include "mmw_cli.h"
#include "FreeRTOS.h"
#include "task.h"

#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>
#include <datapath/dpu/cfarproc/v0/cfarprochwa.h>
#include <datapath/dpu/dopplerproc/v0/dopplerprochwa.h>
#include <datapath/dpu/aoa2dproc/v0/aoa2dproc.h>

#include <common/mmwave_error.h>
#include <common/syscommon.h>
#include <control/mmwave/mmwave.h>
#include <datapath/dpif/dpif_pointcloud.h>
#include <datapath/dpif/dpif_chcomp.h>
#include "ti_cli_mmwave_demo_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*To enable printing of all monitor values*/
#define PRINT_MON_RES 0

#define DPC_TASK_PRI (5U)
#define ADC_FILEREAD_TASK_PRI (4U)
#define TLV_TASK_PRI (3U)
#define CLI_TASK_PRIORITY (1U)

#define DPC_TASK_STACK_SIZE 8192
#define ADC_FILEREAD_TASK_STACK_SIZE 1024
#define TLV_TASK_STACK_SIZE 2048

#define DPC_ADC_FILENAME_MAX_LEN 256
/*Enabling GPADC*/
#define ENABLE_GPADC                (0U)
#define MAX_NUM_TX_ANTENNA          (2U)
#define MAX_NUM_RX_ANTENNA          (3U)
/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/** @brief Output Point cloud list size in number of list elements */
#define MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE 100

/**
 * @brief   Error Code: Out of L3 RAM during radar cube allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE            (DP_ERRNO_OBJECTDETECTION_BASE-1)

/**
 * @brief   Error Code: Out of L3 RAM during detection matrix allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX            (DP_ERRNO_OBJECTDETECTION_BASE-2)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing range DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-3)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-4)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-5)

/**
 * @brief   Error Code: Out of Core Local RAM for range profile
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_PROFILE    (DP_ERRNO_OBJECTDETECTION_BASE-6)

/**
 * @brief   Error Code: Out of L3 RAM during ADC test buffer allocation allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_ADC_TEST_BUFF            (DP_ERRNO_OBJECTDETECTION_BASE-7)

/**
 * @brief   Error Code: Invalid configuration
 */
#define DPC_OBJECTDETECTION_EINVAL_CFG                              (DP_ERRNO_OBJECTDETECTION_BASE-8)

/**
 * @brief   Error Code: Antenna geometry configuration failed
 */
#define DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED            (DP_ERRNO_OBJECTDETECTION_BASE-9)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing doppler DPU Config.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_AOA2D_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-10)

/**
 * @brief   Error Code: Out of Core Local RAM.
 */
#define DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_NEXTCANC_HWA_WINDOW    (DP_ERRNO_OBJECTDETECTION_BASE-11)
/**
 * @brief   Error Code: Out of Core Local RAM.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_NEXTCANC_NEAR_END_SIGNATURE_DATA    (DP_ERRNO_OBJECTDETECTION_BASE-12)

/**
 * @brief
 *  ADC Data Sorce Cfg
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  ADC Data Source to the DPC.
 */
typedef struct ADC_Data_Source_Cfg_t
{
    /*! @brief      Source for ADC Data */
    uint8_t source;

    /*! @brief      ADC filename with full path */
    char fileName[DPC_ADC_FILENAME_MAX_LEN];

}ADC_Data_Source_Cfg;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 */
typedef struct DPC_ObjectDetection_compRxChannelBiasFloatCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q20 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    float rxChPhaseComp[2 * SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPC_ObjectDetection_compRxChannelBiasFloatCfg;


typedef struct MmwDemo_calibCfg_t
{
    /*! @brief      Size of Calibraton data size includng header */
    uint32_t 		sizeOfCalibDataStorage;

    /*! @brief      Enable/Disable calibration restore process  */
    uint32_t 		restoreEnable;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t 		flashOffset;
} MmwDemo_ateCalibCfg;

/**
 * @brief Range Bias and rx channel gain/phase measurement configuration.
 *
 */
typedef struct DPC_ObjectDetection_MeasureRxChannelBiasCliCfg_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Target distance during measurement (in meters) */
    float targetDistance;

    /*! @brief  Search window size (in meters), the search is done in range
     *          [-searchWinSize/2 + targetDistance, targetDistance + searchWinSize/2] */
    float searchWinSize;

} DPC_ObjectDetection_MeasureRxChannelBiasCliCfg;

/**
 * @brief CLI Compression configuration.
 *
 */
typedef struct DPC_ObjectDetection_CliCompressionCfg_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Compression ratio */
    float compressionRatio;

} DPC_ObjectDetection_CliCompressionCfg;

/**
 * @brief Parameters for rx channel compensation procedure
 *
 */
typedef struct DPC_ObjDet_rangeBiasRxChPhaseMeasureParams_t
{

    /*! @brief  range step (meters/bin) */
    float rangeStep;

    /*! @brief  one over range step (meters/bin) */
    float oneOverRangeStep;

    /*! @brief  target distance in range bins */
    float trueBinPosition;
    
    /*! @brief  Range peak search left index  */
    int16_t   rngSearchLeftIdx;

    /*! @brief  Range peak search right index */
    int16_t   rngSearchRightIdx;

} DPC_ObjDet_rangeBiasRxChPhaseMeasureParams;


typedef struct DPC_memUsage_t
{
    /*! @brief   Indicates number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief   Indicates number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t L3RamUsage;

    /*! @brief   Indicates number of bytes of Core Local memory allocated to be used by DPC */
    uint32_t CoreLocalRamTotal;

    /*! @brief   Indicates number of bytes of Core Local memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t CoreLocalRamUsage;

    /*! @brief   Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief   Indicates number of bytes of system heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief   Indicates number of bytes of system heap used by DCP at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;

    /*! @brief   Tracker heap stats */
    HeapP_MemStats trackerHeapStats;
} DPC_memUsage;

/*
 * @brief Memory Configuration used during init API
 */
typedef struct DPC_ObjectDetection_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} DPC_ObjectDetection_MemCfg;

/*
 * @brief Memory pool object to manage memory based on @ref DPC_ObjectDetection_MemCfg_t.
 */
typedef struct MemPoolObj_t
{
    /*! @brief Memory configuration */
    DPC_ObjectDetection_MemCfg cfg;

    /*! @brief   Pool running adress.*/
    uintptr_t currAddr;

    /*! @brief   Pool max address. This pool allows setting address to desired
     *           (e.g for rewinding purposes), so having a running maximum
     *           helps in finding max pool usage
     */
    uintptr_t maxCurrAddr;
} MemPoolObj;

/*
 * @brief This is the result structure reported from DPC's registered processing function
 *        to the application through the DPM_Buffer structure. The DPM_Buffer's
 *        first fields will be populated as follows:
 *        pointer[0] = pointer to this structure.
 *        size[0] = size of this structure i.e sizeof(DPC_ObjectDetection_Result)
 *
 *        pointer[1..3] = NULL and size[1..3] = 0.
 */
typedef struct DPC_ObjectDetection_ExecuteResult_t
{
    /*! @brief      Number of detected objects - Major motion */
    uint16_t        numObjOutMajor;

    /*! @brief      Number of detected objects - Minor motion */
    uint16_t        numObjOutMinor;
    
    /*! @brief      Total Number of detected objects */
    uint32_t        numObjOut;

    /*! @brief      Detected objects output list of @ref numObjOut elements */
    DPIF_PointCloudCartesian *objOut;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              of @ref numObjOut elements */
    DPIF_PointCloudSideInfo *objOutSideInfo;

    /*! @brief      Range Azimuth Heatmap [0] - for Major and [1] - for Minor Motion*/
    uint32_t        *rngAzHeatMap[2];

    /*! @brief      Range Doppler Heatmap */
    uint32_t        *rngDopplerHeatMap;

} DPC_ObjectDetection_ExecuteResult;

/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path.
 */
typedef struct MmwDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

} MmwDemo_output_message_tl;

/*!
 * @brief
 * Structure holds the message body for the  Point Cloud units
 *
 * @details
 * Reporting units for range, azimuth, and doppler
 */
typedef struct MmwDemo_output_message_point_uint_t
{
    /*! @brief x/y/z coordinates reporting unit, in m */
    float       xyzUnit;

    /*! @brief Doppler  reporting unit, in m/s */
    float       dopplerUnit;

    /*! @brief SNR  reporting unit, dB */
    float       snrUint;

    /*! @brief Noise reporting unit, dB */
    float       noiseUint;

    /*! @brief number of detected points in major motion mode */
    uint16_t  numDetectedPoints[2];

} MmwDemo_output_message_point_unit;

/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report x,y,z, Doppler, SNR and detection mode
 */
typedef struct MmwDemo_output_message_UARTpoint_t
{
    /*! @brief Detected point x, in number of x units */
    int16_t      x;
    /*! @brief Detected point y, in number of y units */
    int16_t      y;
    /*! @brief Detected point z, in number of z units */
    int16_t      z;
    /*! @brief Detected point doppler, in number of dopplerUnit */
    int16_t      doppler;
    /*! @brief Range detection SNR, in number of snrUnits */
    uint8_t      snr;
    /*! @brief Detected point noise value in in number of noiseUnits */
    uint8_t      noise;
} MmwDemo_output_message_UARTpoint;

/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report x,y,z, Doppler, SNR and detection mode
 */
typedef struct MmwDemo_output_message_UARTpointCloud_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_point_unit pointUint;
    MmwDemo_output_message_UARTpoint    point[MMWDEMO_OUTPUT_POINT_CLOUD_LIST_MAX_SIZE];
} MmwDemo_output_message_UARTpointCloud;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   Current Power Sensor Readings for 1.8V, 3.3V, 1.2V and 1.2V RF rails respectively expressed in 100 uW (1LSB = 100 uW) */
    uint16_t     powerMeasured[4];

    /*! @brief   Temperature Readings: Rx, Tx, PM, DIG. in C degrees, 1LSB = 1 deg C, signed */
    int16_t      tempReading[4];

} MmwDemo_output_message_stats;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibData_t
{
    /*! @brief      Magic word for calibration data */
    uint32_t        magic;

    /*! @brief      RX TX Calibration data */
    T_RL_API_FECSS_RXTX_CAL_DATA  calibData;
} MmwDemo_calibData;

/*
 * @brief structure holds calibration save configuration.
 */
typedef struct MmwDemo_factoryCalibCfg_t
{
    /*! @brief      Enable/Disable calibration save process  */
    uint8_t         saveEnable;

    /*! @brief      Enable/Disable calibration restore process  */
    uint8_t         restoreEnable;

    /*! @brief      RX channels gain setting for factory calibration */
    uint8_t         rxGain;

    /*! @brief      TX channels power back-off setting for calibration (in 0.5 dB Resolution). */
    uint8_t         txBackoffSel;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t        flashOffset;

    /*! @brief      Is RF Trimmed? */
    bool        atecalibinEfuse;
} MmwDemo_factoryCalibCfg;


/*
 * @brief Stats structure to convey to Application timing and related information.
 */
typedef struct DPC_ObjectDetection_Stats_t
{
    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStamp;

    /*! @brief   Frame period in usec */
    uint32_t      framePeriod_us;

    /*! @brief   Chirping time in usec */
    uint32_t      chirpingTime_us;

    /*! @brief   Total active time in usec */
    uint32_t      totalActiveTime_us;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      interFrameStartTimeStamp;

    /*! @brief   Inter-frame end CPU time stamp */
    uint32_t      interFrameEndTimeStamp;

    /*! @brief   UART data transfer end CPU time stamp */
    uint32_t      uartTransferEndTimeStamp;

    /*! @brief   Tracker processing time */
    uint32_t      trackerTime_us;

    /*! @brief   Micro Doppler DPU processing time  */
    uint32_t      microDopplerDpuTime_us;

    /*! @brief   micro doppler calculation plus feature extraction time  */
    uint32_t      featureExtractionTime_us;

    /*! @brief   Classifier processing time  */
    uint32_t      classifierTime_us;

} DPC_ObjectDetection_Stats;
#if (ENABLE_MONITORS==1)
/*!
 * @brief
 * Structure to hold RF monitor results
 *
 */
typedef struct MmwDemo_Mon_Result_t
{
    /*! @brief Status of Monitors Enabled, 0 - Failed/ Not enabled, 1- Passed */
    uint64_t monStat;

    MMWave_MonTxRxLb_res txRxLb[MAX_NUM_TX_ANTENNA];

    /*! @brief Loopback status Pass/Fail 
     * Pass - 1 , Fail -0
     *     | Bit Field   | Definition  |
     *     |-------------|-----------  |
     *     | Bit[0]      | Tx 0 Loopback Noise Rx 0|
     *     | Bit[1]      | Tx 0 Loopback Noise Rx 1|
     *     | Bit[2]      | Tx 0 Loopback Noise Rx 2| 
     *     | Bit[3]      | Tx 0 Loopback BPM Noise Rx 0|
     *     | Bit[4]      | Tx 0 Loopback BPM Noise Rx 1| 
     *     | Bit[5]      | Tx 0 Loopback BPM Noise Rx 2|
     *     | Bit[6]      | Tx 0 Loopback BPM Gain Error Rx 0|
     *     | Bit[7]      | Tx 0 Loopback BPM Gain Error Rx 1|
     *     | Bit[8]      | Tx 0 Loopback BPM Gain Error Rx 2|
     *     | Bit[9]      | Tx 0 Loopback BPM Phase Error Rx 0|
     *     | Bit[10]     | Tx 0 Loopback BPM Phase Error Rx 1|
     *     | Bit[11]     | Tx 0 Loopback BPM Phase Error Rx 2|
     *     | Bit[12]     | RX LoopBack Gain Mismatch Variation 1 Tx0|
     *     | Bit[13]     | RX LoopBack Gain Mismatch Variation 2 Tx0|
     *     | Bit[14]     | RX LoopBack Phase Mismatch Variation 1 Tx0|
     *     | Bit[15]     | RX LoopBack Phase Mismatch Variation 2 Tx0|
     *     | Bit[16]     | TX0 LoopBack Rx Gain|
     *     | Bit[24:17]  | Reserved|
     *     | Bit[25]     | Tx 1 Loopback Noise Rx 0|
     *     | Bit[26]     | Tx 1 Loopback Noise Rx 1|
     *     | Bit[27]     | Tx 1 Loopback Noise Rx 2| 
     *     | Bit[28]     | Tx 1 Loopback BPM Noise Rx 0|
     *     | Bit[29]     | Tx 1 Loopback BPM Noise Rx 1| 
     *     | Bit[30]     | Tx 1 Loopback BPM Noise Rx 2|
     *     | Bit[31]     | Tx 1 Loopback BPM Gain Error Rx 0|
     *     | Bit[32]     | Tx 1 Loopback BPM Gain Error Rx 1|
     *     | Bit[33]     | Tx 1 Loopback BPM Gain Error Rx 2|
     *     | Bit[34]     | Tx 1 Loopback BPM Phase Error Rx 0|
     *     | Bit[35]     | Tx 1 Loopback BPM Phase Error Rx 1|
     *     | Bit[36]     | Tx 1 Loopback BPM Phase Error Rx 2|
     *     | Bit[37]     | RX LoopBack Gain Mismatch Variation 1 Tx1|
     *     | Bit[38]     | RX LoopBack Gain Mismatch Variation 2 Tx1|
     *     | Bit[39]     | RX LoopBack Phase Mismatch Variation 1 Tx1|
     *     | Bit[40]     | RX LoopBack Phase Mismatch Variation 2 Tx1|
     *     | Bit[41]     | TX1 LoopBack Rx Gain|
     *     | Bit[49:42]  | Reserved|
     *     | Bit[50]     | TX LoopBack Gain Mismatch Variation 1 Rx|
     *     | Bit[51]     | TX LoopBack Gain Mismatch Variation 2 Rx|
     *     | Bit[52]     | TX LoopBack Gain Mismatch Variation 3 Rx|
     *     | Bit[53]     | TX LoopBack Phase Mismatch Variation 1 Rx|
     *     | Bit[54]     | TX LoopBack Phase Mismatch Variation 2 Rx|
     *     | Bit[55]     | TX LoopBack Phase Mismatch Variation 3 Rx|
     *     | Bit[63:56]  | Reserved|
    */
    uint64_t status_txRxLb;

    /*! @brief TX gain mismatch before zeroing in dBm*/
    float txlbGainMismatch[MAX_NUM_RX_ANTENNA];

    /*! @brief TX phase mismatch before zeroing*/
    float txlbPhaseMismatch[MAX_NUM_RX_ANTENNA];

    /*! @brief TX gain before zeroing after zeroing time-0 mismatch in dBm*/
    float txlbGainMismatchVar[MAX_NUM_RX_ANTENNA];

    /*! @brief TX phase before zeroing after zeroing time-0 mismatch*/
    float txlbPhaseMismatchVar[MAX_NUM_RX_ANTENNA];

    /*! @brief TX Loop Back RX gain mismatch before zeroing in dBm*/
    float rxlbGaintx0MismatchVar[MAX_NUM_TX_ANTENNA];

    /*! @brief TX Loop Back RX phase mismatch before zeroing*/
    float rxlbPhasetx0MismatchVar[MAX_NUM_TX_ANTENNA];

    /*! @brief TX Loop Back RX gain mismatch after zeroing time-0 mismatch in dBm*/
    float rxlbGaintx1MismatchVar[MAX_NUM_TX_ANTENNA];

    /*! @brief TX Loop Back RX phase mismatch after zeroing time-0 mismatch*/
    float rxlbPhasetx1MismatchVar[MAX_NUM_TX_ANTENNA];

    /*! @brief TX power in dBm*/
    float txPower[MAX_NUM_TX_ANTENNA];
    
    /*! @brief Power monitor status Pass/Fail 
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | TX0 Power Monitor      |
    *     | Bit[1]      | TX1 Power Monitor      |
    *     | Bit[7:2]  | Reserved     | 
    */
    uint8_t status_txPower;

    /*! @brief TX loopback Rx gain in dBm*/
    float txlbRxgain[MAX_NUM_TX_ANTENNA];

    /*! @brief TX power ball break return loss mismatch*/
    float txPowerBBretlossMismatch[MAX_NUM_TX_ANTENNA];

    /*! @brief TX power ball break */
    MMWave_MonTxnPowBB_res txPowerBB[MAX_NUM_TX_ANTENNA];

    /*! @brief Ball break status Pass/Fail 
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | TX0 Ball Break Monitor      |
    *     | Bit[1]      | TX1 Ball Break Monitor      |
    *     | Bit[7:2]  | Reserved     | 
    */
    uint8_t status_txPowerBB;

    /*! @brief TX DC Signal Result 0 - Fail 1 - PASS
     *     | Bit Field   | Definition  |
     *     |-------------|-----------  |
     *     | Bit[0]      | TXn_VDDA_SUPPLY signal Status      |
     *     | Bit[1]      | TXn_1V_CLUSTER_SUPPLY signal Status     |
     *     | Bit[15:2]  | Reserved     | */
    uint16_t txDcSig[MAX_NUM_TX_ANTENNA];

    /*! @brief DC Signal status Pass/Fail 
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | TX0 DC SIGNAL Monitor      |
    *     | Bit[1]      | TX1 DC SIGNAL Monitor      |
    *     | Bit[7:2]  | Reserved     | 
    */
    uint8_t status_txDcSig;

    MMWave_MonPllVolRes_res pllvolres;
    /*! @brief PM CLK status Pass/Fail
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | APLL      |
    *     | Bit[1]      | SynthMin     | 
    *     | Bit[2]      | SynthMax     | 
    *     | Bit[7:3]    | Reserved   | 
    */
    uint8_t status_pllvolres;

    MMWave_MonRxHpfDcSig_res rxHpfDcsigres;
    /*! @brief Rx HPF status Pass/Fail
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition                |
    *     |-------------|---------------------------|
    *     | Bit[0]      | Rx HPF 0 Attenuation      |
    *     | Bit[1]      | Rx HPF 1 Attenuation      | 
    *     | Bit[2]      | Rx HPF 2 Attenuation      | 
    *     | Bit[3]      | Rx HPF 0 In Band Power    | 
    *     | Bit[4]      | Rx HPF 1 In Band Power    |
    *     | Bit[5]      | Rx HPF 1 In Band Power    |
    *     | Bit[7:6]    | Reserved                  | 
    */
    uint8_t status_rxHpfDCsigres;

    MMWave_MonPmClkDcSig_res pmClkDcSigres;

    /*! @brief PM CLK status Pass/Fail
    * Pass - 1 , Fail -0
    *     | Bit Field   | Definition  |
    *     |-------------|-----------  |
    *     | Bit[0]      | PM CLK DC SIG Monitor      |
    *     | Bit[7:1]    | Reserved     | 
    */
    uint8_t status_pmClkDcSig;

    MMWave_MonSynthFreqRes_res synthFreqres;

    MMWave_MonRxSatLive_res rxSatLiveres; 

    uint8_t status_rxSatLive;

} MmwDemo_Mon_Result;
#endif
typedef struct MmwDemo_antennaGeometryAnt_t
{

    /*! @brief  row index in steps of lambda/2 */
    int8_t row;

    /*! @brief  row index in steps of lambda/2 */
    int8_t col;

} MmwDemo_antennaGeometryAnt;

typedef struct MmwDemo_antennaGeometryCfg_t
{
    /*! @brief  Distance between antennas in x dimension (in meters) */
    float antDistanceXdim;
    /*! @brief  Distance between antennas in z dimension (in meters)*/
    float antDistanceZdim;
    /*! @brief  virtual antenna positions */
    MmwDemo_antennaGeometryAnt ant[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
} MmwDemo_antennaGeometryCfg;

typedef struct HwaDmaTrigChanPoolObj_t
{
    uint16_t dmaTrigSrcNextChan;
} HwaDmaTrigChanPoolObj;

typedef struct HwaWinRamMemoryPoolObj_t
{
    uint16_t memStartSampleIndex;
} HwaWinRamMemoryPoolObj;
/**
 * @brief
 *  Spec Values for Monitors
 *
 * @details
 *  The structure specifies the SPEC Values for Monitors.
 */
typedef struct MMWave_MonSpecVal_t
{
    /*Spec Values for APLL*/
    float APLLVSpecMin;
    float APLLVSpecMax;
    /*Spec Values for SynthMin*/
    float SynthMinVSpecMin;
    /*Spec Values for SynthMax*/
    float SynthMaxVSpecMax;
    /*Spec Values for LoopBack*/
    float RxGainSpecMin;
    float RxGainSpecMax;
    float lbNoiseSpecMax;
    float lbBPMNoiseSpecMax;
    float lbBPMGainErrSpecMin;
    float lbBPMGainErrSpecMax;
    float lbBPMPhaseErrSpecMin;
    float lbBPMPhaseErrSpecMax;
    float RxlbGainMisVarSpecMin;
    float RxlbGainMisVarSpecMax;
    float RxlbPhaseMisVarSpecMin;
    float RxlbPhaseMisVarSpecMax;
    float TxlbGainMisVarSpecMin;
    float TxlbGainMisVarSpecMax;
    float TxlbPhaseMisVarSpecMin;
    float TxlbPhaseMisVarSpecMax;
    /*Spec Values for Power Monitor*/
    float TxPowSpecMin[MAX_NUM_TX_ANTENNA];
    /*Spec Values for Ball Break Monitor*/
    float TxBBRetLossSpec;
    float TxBBRetLossVarSpec;
    /*Spec Values for DC Signal Monitor*/
    uint8_t TxDCSigResSpec;
    /*Spec Values for RX HPF Monitor*/
    float RxHPFAttnSpecMin;
    float RxHPFAttnSpecMax;
    float RxHPFInBandPwrMindBm;
    float RxHPFInBandPwrMaxdBm;
    /*Spec Values for PM CLK DC Signal Monitor*/
    uint32_t PMClkDCSigStatSpec;

}MMWave_MonSpecVal;


/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo.
 */
typedef struct MmwDemo_MSS_MCB_t
{

    /*! @brief      UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                  edmaHandle;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    /*! @brief   Rf frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
    double                      rfFreqScaleFactor;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            demoInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            cliInitTaskCompleteSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            TestSemHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            tlvSemHandle;
#if (ENABLE_MONITORS==1)
    /*! @brief   Semaphore Object to pend Front end shutdown when RF monitors are running in Low power mode*/
    SemaphoreP_Object            rfmonSemHandle;
#endif
    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            adcFileTaskSemHandle;

    /*! @brief   Semaphore Object  */
    SemaphoreP_Object            dpcTaskConfigDoneSemHandle;
    /*! @brief   Semaphore Object  */
    SemaphoreP_Object            uartTaskConfigDoneSemHandle;



    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /**
     * @brief MMWave configuration tracked by the module.
     */
    T_RL_API_SENS_FRAME_CFG                    frameCfg;

    /**
     * @brief sensor stop command data structure
     *
     */
    T_RL_API_SENSOR_STOP_CMD                   sensorStop;

    /**
     * @brief RF power ON/OFF config command data structure
     *
     */
    T_RL_API_FECSS_RF_PWR_CFG_CMD              channelCfg;

    /**
     * @brief Sensor chirp profile common config command data structure
     *
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileComCfg;

    /**
     * @brief Sensor chirp profile common config command data structure
     *
     */
    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfg;

    /**
     * @brief Front End Run Time TX CLPC calibration command
     *
     */
    T_RL_API_FECSS_RUNTIME_TX_CLPC_CAL_CMD     fecTxclpcCalCmd;

    #if (CLI_REMOVAL == 0)
    /**
     * @brief ADC read from file data structure
     *
     */
    ADC_Data_Source_Cfg                        adcDataSourceCfg;
    #endif

    /**
     * @brief Raw ADC data capture configuration
     *
     */
    CLI_adcLoggingCfg                           adcLogging;
    
    /**
     * @brief sensor start command data structure
     *
     */
    MMWave_StrtCfg                   sensorStart;

    /**
     * @brief Gui Monitor Sel
     *
     */
    CLI_GuiMonSel                              guiMonSel;

    /**
     * @brief Signal Chain CFG
     *
     */
    CLI_sigProcChainCfg                        sigProcChainCfg;

    /**
     * @brief Cfar Cfg
     *
     */
    DPU_CFARProc_CfarCfg                       cfarCfg;

    /**
     * @brief Cfar second pass Cfg
     *
     */
    DPU_CFARProc_CfarScndPassCfg                       cfarScndPassCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_CFARProc_AoaFovCfg                     fovCfg;

    /**
     * @brief Fov Cfg
     *
     */
    DPU_Aoa2dProc_AoaFovCfg                     fovAoaCfg;

     /* @brief static clutter removal flag - Note: not used in this demo */
    uint32_t staticClutterRemovalEnable;

    /**
     *  @brief   Range Bias and rx channel gain/phase measurement configuration
     *
     */
    DPC_ObjectDetection_MeasureRxChannelBiasCliCfg measureRxChannelBiasCliCfg;

    /**
     *  @brief   Parameters for range bias rx channel compensation procedure
     *
     */
    DPC_ObjDet_rangeBiasRxChPhaseMeasureParams measureRxChannelBiasParams;

    /**
     * @brief Rx channel compensation coefficients, specified by CLI command
     *
     */
    DPIF_compRxChannelBiasFloatCfg compRxChannelBiasCfgMeasureOut;

    /**
     * @brief Range Select Cfg
     *
     */
    DPU_CFARProc_RangeFovCfg rangeSelCfg;

     /* @brief Rx channel compensation coefficients, specified by CLI command
     *
     */
    DPIF_compRxChannelBiasCfg    compRxChannelBiasCfg;

    /*! @brief  Configuration to setup DFP */
    MMWave_CtrlCfg                      mmwCtrlCfg;

    /*! @brief  Configuration to open DFP */
    MMWave_OpenCfg                      mmwOpenCfg;

    /*! @brief  Rx Channel offsets in ADC buffer - not used */
    uint16_t                            rxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

    /*! @brief  Range processing DPU handle */
    DPU_RangeProcHWA_Handle             rangeProcDpuHandle;

    /*! @brief  Doppler processing DPU handle */
    DPU_DopplerProcHWA_Handle           dopplerProcDpuHandle;

    /*! @brief  CFAR DPU handle */
    DPU_CFARProcHWA_Handle              cfarProcDpuHandle;

    /*! @brief  AOA2D DPU handle */
    DPU_Aoa2dProc_Handle                aoa2dProcDpuHandle;

    /* DPC configs from CLI */
    /*! @brief Number of enabled Tx antennas */
    uint16_t                            numTxAntennas;

    /*! @brief Number of enabled Rx antennas */
    uint16_t                            numRxAntennas;
    uint8_t                             rxAntOrder[SYS_COMMON_NUM_RX_CHANNEL];
    float                               adcStartTime;

    /*! @brief Start frequency in GHz */
    float                               startFreq;

    /*! @brief Chirp slope in MHz/usec */
    float                               chirpSlope;

    /*! @brief ADC sampling rate in MHz */
    float                               adcSamplingRate;

    /*! @brief Burst period in msec */
    float                               burstPeriod;

    /*! @brief Number of range bins */
    uint32_t                            numRangeBins;

    /*! @brief Number of Doppler bins */
    uint32_t                            numDopplerBins;

    /*! @brief  Angle dimension:
     * 0-1Tx antenna-1Rx antenna,
     * 1-virtual antenna array only in azimuth dimension,
     * 2-two-dimensional virtual antenna array */
    uint8_t                             angleDimension;

    MmwDemo_antennaGeometryCfg          antennaGeometryCfg;
    MmwDemo_antennaGeometryCfg          activeAntennaGeometryCfg;
    uint16_t                            numAntRow;
    uint16_t                            numAntCol;

    /*! @brief Flag: 0-Low power mode disabled, 1-Low Power mode enabled, 2-Used for testing low power mode */
    uint8_t                             lowPowerMode;

    /*! @brief Flag to control in low power mode some configuration parts to be executed only once, and not to be repeated from frame to frame */
    uint8_t                             oneTimeConfigDone;

    MmwDemo_ateCalibCfg                 mmwAteCalibCfg;

    /*! @brief Flag set to 1 if Major Motion Detection mode is enabled  */
    uint8_t enableMajorMotion;

    /*! @brief Flag set to 1 if Minor Motion Detection mode is enabled  */
    uint8_t enableMinorMotion;

    /*! @brief  MIMO mode selection: 0-TDM-MIMO, 1-BPM-MIMO  */
    uint8_t isBpmEnabled;

    /*! @brief  Motion-presence DPU enable flag */
    uint8_t isMotionPresenceDpuEnabled;

    /*! @brief L3 ram memory pool object */
    MemPoolObj    L3RamObj;

    /*! @brief Core Local ram memory pool object */
    MemPoolObj    CoreLocalRamObj;

    /*! @brief Tracker memory pool object */
    HeapP_Object CoreLocalTrackerHeapObj;

    /* Feature extraction variables memory pool object */
    HeapP_Object CoreLocalFeatExtractHeapObj;

    /*! @brief Memory Usage */
    DPC_memUsage    memUsage;

    /*! @brief ADC buffer allocated in testing mode only */
    uint8_t         *adcTestBuff;

    /*! @brief      Radar cube data interface for [0]-Major Motion Detection [1]-Minor Motion Detection*/
    DPIF_RadarCube  radarCube[2];

   /*! @brief      Detection matrix */
    DPIF_DetMatrix  detMatrix;

    /*! @brief      Range profiles to be exported to GUI */
    uint32_t        *rangeProfile;

    /*! @brief      Run time input parameters to CFAR DPU, [0] - for major motion, [1] - for minor motion */
    DPU_CFARProcHWA_InputParams cfarRunTimeInputParams;


    /*! @brief      Doppler index matrix, type uint8, size = number range bins x number azimuth bins * number of elevation bins */
    DPIF_DetMatrix dopplerIndexMatrix;

    /*! @brief      Elevation index matrix, type uint8, size = number range bins x number azimuth bins*/
    DPIF_DetMatrix elevationIndexMatrix;

    /*! @brief      Pointers to DPC output data */
    DPC_ObjectDetection_ExecuteResult dpcResult;

    /*! @brief Point cloud structure with int16 type coordinates sent to Host via UART. Includes TLV header and uints structure */
    MmwDemo_output_message_UARTpointCloud pointCloudToUart;

    /*! @brief Structure with inverse unit scales for coordinate conversion from float type to int16 type */
    MmwDemo_output_message_point_unit pointCloudUintRecip;

    /*! @brief  Point cloud list generated by CFAR DPU */
    DPIF_PointCloudCartesianExt *cfarDetObjOut;

    /*! @brief  CFAR rnage/doppler detection list */
    DPIF_CFARRngDopDetListElement *detRngDopList;

    DPIF_DetectedRangeGates detectedRangeGates;

    /*! @brief  Point cloud list (floating point) sent over UART if enabled */
    DPIF_PointCloudCartesian *dpcObjOut;

    /*! @brief  Point cloud side information list  sent over UART if enabled */
    DPIF_PointCloudSideInfo *dpcObjSideInfo;

    /*! @brief  Point cloud list range/azimuth/elevation/doppler indices per point */
    DPIF_PointCloudRngAzimElevDopInd *dpcObjIndOut;

    /*! @brief  Presence detect state of each zone*/
    uint8_t *dpcZoneState;

    /*! @brief      DPC stats structure */
    DPC_ObjectDetection_Stats stats;

    /*! @brief      DPC reported output stats structure */
    MmwDemo_output_message_stats outStats;

    /*! @brief Token is checked in the frame start ISR, asserted to have zero value, and incremented. At the end of UART task, it is decremented */
    uint32_t interSubFrameProcToken;
    /*! @brief Counts frames not completed in time */
    uint32_t interSubFrameProcOverflowCntr;

    /*! @brief  This is introduced to support minor motion detection in the power saving mode */
    uint32_t frmCntrModNumFramesPerMinorMot;

    /*! @brief  This is used in testing the low power mode */
    uint64_t frameStartTimeStampSlowClk;
#if (ENABLE_MONITORS==1)
    /*! @brief  List of RF monitors to be enabled */
    uint64_t rfMonEnbl;

    /*! @brief  List of PLL control voltage monitors to be enabled
     * | Value   | Definition  |
     * |---------|-----------  |
     * | 0      | Monitor Disabled    |
     * | 1      | Monitor Enabled    |
     * Bit Field Definition:
     * | BitField   | Definition  |
     * |---------|-----------  |
     * | Bit[0]  | APLL_CTRL_VOLT_MON_EN. APLL control voltage monitor enable   |
     * | Bit[1]  | SYNTH_CTRL_VOLT_MON_EN. SYNTH VCO control voltage monitor enable   |
     * | Bit[7:2] | RESERVED |*/
    uint8_t monPllVolEnaMask;
    
    /*! @brief  Are TX RX Loop Back monitors enabled */
    uint8_t ismonTxRxLbCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  TX RX Loob Back monitors configuration */
    MMWave_MonTxRxLb_cfg monTxRxLbCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  Are TX Power RF monitors enabled */
    uint8_t ismonTxpwrCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  TX Power monitors configuration */
    MMWave_MonTxnPow_cfg monTxpwrCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  Are TX Power BB RF monitors enabled */
    uint8_t ismonTxpwrBBCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  TX Power Ball Break monitors configuration */
    MMWave_MonTxnPow_cfg monTxpwrBBCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  Are TX Dc Signal RF monitors enabled */
    uint8_t ismonTxDcSigCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  Monitor Spec Values */
    MMWave_MonSpecVal SpecVal;

    /*! @brief  TX DC Singal monitors configuration */
    MMWave_MonTxnDcSig_cfg monTxDcSigCfg[MAX_NUM_TX_ANTENNA];

    /*! @brief  RX HPF DC Singal monitors configuration */
    MMWave_MonRxHpfDcSig_cfg monRxHpfDcSigCfg;

    /*! @brief  PM Clock DC Signal monitor configuration */
    float monPmClkDcStFreqGhz;

    /*! @brief  RF Monitor result */
    MmwDemo_Mon_Result rfMonRes;

    /*! @brief  RF Monitor LB result during factory calibration */
    MmwDemo_Mon_Result rfMonResFactCal;
#endif
    /*! @brief  Factory calibration cofiguration for save/restore */
    MmwDemo_factoryCalibCfg    factoryCalCfg;

    /*! @brief HWA DMA trigger source channel pool */
    HwaDmaTrigChanPoolObj  HwaDmaChanPoolObj;

    /*! @brief HWA Window RAM memory pool */
    HwaWinRamMemoryPoolObj  HwaWinRamMemoryPoolObj;

    /*! @brief Number of used HWA param sets */
    uint8_t numUsedHwaParamSets;

    /*! @brief CLI Compression configuration */
    DPC_ObjectDetection_CliCompressionCfg cliCompressionCfg;

    /*! @brief GPADC Volatage values (V)*/
    float GPADCVal[2];

    /*! @brief Enable ADC data streaming via SPI*/
    uint8_t spiADCStream;

    /*! @brief Is the device Antenna On Package */
    uint8_t isDevAOP;

} MmwDemo_MSS_MCB;

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, on the XWRLx4xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,

    /*! @brief   Samples to calculate static azimuth/elevation heatmap, (all virtual antennas exported) */
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,

    /*! @brief   temperature stats from Radar front end */
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,

    MMWDEMO_OUTPUT_MSG_MAX,

    MMWDEMO_OUTPUT_EXT_MSG_START = 300,

    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_EXT_MSG_DETECTED_POINTS,

    /*! @brief   Range profile - Major Motion */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR,

    /*! @brief   Range-azimuth  heatmap - Major motion */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR,

    /*! @brief   Range-azimuth  heatmap - Minor motion  */
    MMWDEMO_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR,

    /*! @brief   Stats information, (timing, temperature, power) */
    MMWDEMO_OUTPUT_EXT_MSG_STATS,

    /*! @brief   Presence information */
    MMWDEMO_OUTPUT_EXT_MSG_PRESENCE_INFO,

    /*! @brief   Target List - Array of detected targets (position, velocity, error covariance) */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_LIST,

    /*! @brief   Target List - Array of target indices */
    MMWDEMO_OUTPUT_EXT_MSG_TARGET_INDEX,

    /*! @brief   Micro doppler raw data */
    MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA,

    /*! @brief   Micro doppler features */
    MMWDEMO_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES,

    /*! @brief   Radar Cube - Major motion */
    MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MAJOR,

    /*! @brief   Radar Cube - Minor motion  */
    MMWDEMO_OUTPUT_EXT_MSG_RADAR_CUBE_MINOR,

    /*! @brief   Point cloud range azimuth indices */
    MMWDEMO_OUTPUT_EXT_MSG_POINT_CLOUD_INDICES,

    /*! @brief   Presence detected in a region of interest - major, minor, unoccupied  */
    MMWDEMO_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION,

    /*! @brief   ADC samples */
    MMWDEMO_OUTPUT_EXT_MSG_ADC_SAMPLES,

    /*! @brief   Classifier info */
    MMWDEMO_OUTPUT_EXT_MSG_CLASSIFIER_INFO,

    /*! @brief   Rx Channel compensation info */
    MMWDEMO_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO,

    MMWDEMO_OUTPUT_EXT_MSG_MAX

} MmwDemo_output_message_type;

#define MMWDEMO_OUTPUT_ALL_MSG_MAX (MMWDEMO_OUTPUT_MSG_MAX + MMWDEMO_OUTPUT_EXT_MSG_MAX - MMWDEMO_OUTPUT_EXT_MSG_START - 1)

/*!
 * @brief
 *  Message header for reporting detection information from data path.
 *
 * @details
 *  The structure defines the message header.
 */
typedef struct MmwDemo_output_message_header_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t     version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;

    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   Time in CPU cycles when the message was created. For XWR16xx/XWR18xx: DSP CPU cycles, for XWR14xx: R4F CPU cycles */
    uint32_t    timeCpuCycles;

    /*! @brief   Number of detected objects */
    uint32_t    numDetectedObj;

    /*! @brief   Number of TLVs */
    uint32_t    numTLVs;

    uint32_t    subFrameNumber;

} MmwDemo_output_message_header;


// void configINASensors(void);

/* Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

#ifdef __cplusplus
}
#endif

#endif /* MMWAVE_DEMO_H */

