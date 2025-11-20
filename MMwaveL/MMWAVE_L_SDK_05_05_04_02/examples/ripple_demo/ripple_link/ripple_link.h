/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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
#ifndef RIPPLE_LINK_H
#define RIPPLE_LINK_H
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>

#include <mmwavelink/mmwavelink.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/hw_include/soc_config.h>
#include <drivers/mcspi/v0/mcspi.h>

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ATE_CALIB_DATA_LENGTH  (uint8_t)(128U)
#define SYS_COMMON_NUM_RX_CHANNEL                   3U
/* Boot Calibration Macros */
#define M_RL_SENS_CHIRP_RFFREQ_LR_60P5G  ((UINT32)0x0000C9AAU) /* 60.5GHz*/
#define M_RL_SENS_CHIRP_RFFREQ_LR_78P5G  ((UINT32)0x0000C440U) /* 78.5GHz*/
/**
 * @brief
 *  mmWave Supported max profiles which can be configured on the BSS
 */
#define MMWAVE_MAX_PROFILE                (4U)
#define MMWAVE_NBR_DEVICES                (1U)
#define MMWAVE_ERRNO_BASE                 (-3100)
/**
 * @brief   Error Code: Invalid argument
 */
#define MMWAVE_EINVAL                   (MMWAVE_ERRNO_BASE-1)
/**
 * @brief   Status flag to indicate the open state
 */
#define MMWAVE_STATUS_OPENED                        1U
/**
 * @brief   Status flag to indicate the configured state
 */
#define MMWAVE_STATUS_CONFIGURED                    2U
/**
 * @brief
 *  This defines the maximum number of entries in the spawn list.
 */
#define MMWAVE_MAX_NUM_SPAWN_LIST                   4U
/**
 * @brief   Status flag to indicate the started state
 */
#define MMWAVE_STATUS_STARTED                       8U
/**
 * @brief   Error Code: mmWave link profile configuration failed
 */
#define MMWAVE_EPROFILECFG              (MMWAVE_ERRNO_BASE-8)
/**
 * @brief   Error Code: mmWave Link version error
 */
#define MMWAVE_EVERSION                 (MMWAVE_ERRNO_BASE-14)
/**
 * @brief   Error Code: mmWave link frame configuration failed
 */
#define MMWAVE_EFRAMECFG                (MMWAVE_ERRNO_BASE-10)
/**
 * @brief   Error Code: mmWave link sensor failed
 */
#define MMWAVE_ESENSOR                  (MMWAVE_ERRNO_BASE-12)
/**
 * @brief   Error Code: mmWave link initialization failed
 */
#define MMWAVE_EINIT                    (MMWAVE_ERRNO_BASE-2)
/**
 * @brief   Error Code: mmWave deinitialization error
 */
#define MMWAVE_EDEINIT                  (MMWAVE_ERRNO_BASE-16)
#define     MINUS_ONE           -((int32_t)1)
/**
 * @brief   Error Code: mmWave link BSS calibration periodicity failed
 */
#define MMWAVE_ECALPERIOD               (MMWAVE_ERRNO_BASE-21)

/**
 * @brief   Error Code: Out of memory error
 */
#define MMWAVE_ENOMEM                   (MMWAVE_ERRNO_BASE-23)
/**
 * @brief   Error Code: Not found
 */
#define MMWAVE_ENOTFOUND                (MMWAVE_ERRNO_BASE-24)


 /**
 * @brief   Error Code: RFS Boot Calibration Failure
 */
#define MMWAVE_ERFSBOOTCAL                    (MMWAVE_ERRNO_BASE-41)
#define FECSS_TIMESTAMP_COUNTER_MASK    (0x0FFFFFU)
/* User defined heap memory and handle */
#define MMWAVE_HEAP_MEM_SIZE  (16*1024u)
/** \brief RFS Factory CAL Validity Status Macro */
#define RFS_FACTORY_CAL_VALIDITY_STATUS       ((uint16_t) 0x2FU)

typedef void *SPI_Handle;


/**
 * @brief
 *  mmWave Handle
 */
typedef void*   MMWave_Handle;

/**
 * @brief
 *  mmWave Profile Handle
 */
typedef void*   MMWave_ProfileHandle;

/**
 * @brief
 *  mmWave Chirp Handle
 */
typedef void*   MMWave_ChirpHandle;
/**
 * @brief
 *  Frame mode configuration
 *
 * @details
 *  The structure specifies the configuration which is required to configure
 *  the mmWave link to operate in frame mode
 */
typedef struct MMWave_FrameCfg_t
{
    /**
     * @brief   List of all the active profile handles which can be configured.
     * Setting to NULL indicates that the profile is skipped.
     */
    MMWave_ProfileHandle    profileHandle[MMWAVE_MAX_PROFILE];

    /**
     * @brief   Configuration which is used to setup Frame
     */
    T_RL_API_SENS_FRAME_CFG            frameCfg;

    /**
     * @brief   Configuration which is used to setup Temperature measurements
     */
    T_RL_API_FECSS_TEMP_MEAS_CMD       tempCfg;

}MMWave_FrameCfg;


/**
 * @brief
 *  Chirp Calibration configuration
 *
 * @details
 *  The structure is used to hold the information which is required
 *  to setup the calibration of the RF while operating in the Chirp
 *  mode
 */
typedef struct MMWave_ChirpCalibrationCfg_t
{
    /**
     * @brief   Flag which determines if calibration is to be enabled or
     * not.
     */
    bool            enableCalibration;

    /**
     * @brief   Flag which determines if periodic calibration is to be enabled or
     * not. The mmWave will always do one time calibration.
     */
    bool            enablePeriodicity;

    /**
     * @brief   This is valid only if periodicity is enabled and is the time in frames
     * when a calibration report is received by the application through an asynchrous
     * event.
     */
    uint16_t        periodicTimeInFrames;
}MMWave_ChirpCalibrationCfg;


/**
 * @brief
 *  Calibration configuration
 *
 * @details
 *  The structure is used to hold the information which is required
 *  to setup the calibration of the RF
 */
typedef struct MMWave_CalibrationCfg_t
{
    /**
     * @brief   This should be configured if the DFE Data output mode
     * is configured to operate in frame or advanced frame mode
     */
    MMWave_ChirpCalibrationCfg  chirpCalibrationCfg;
}MMWave_CalibrationCfg;
/**
 * @brief
 *  Control configuration
 *
 * @details
 *  The structure specifies the configuration which is required to configure
 *  and setup the BSS.
 */
typedef struct MMWave_CtrlCfg_t
{

    /**
     * @brief   Chirp configuration to be used: This is only applicable
     * if the data output mode is set to Chirp
     */
    MMWave_FrameCfg         frameCfg[MMWAVE_NBR_DEVICES];
}MMWave_CtrlCfg;

/**
 * @brief
 *  Open Configuration
 *
 * @details
 *  The structure specifies the configuration which is required to open the
 *  MMWave module. Once the MMWave module has been opened the mmWave link
 *  to the BSS is operational.
 */
typedef struct MMWave_OpenCfg_t
{
    T_RL_API_FECSS_RF_RUN_CAL_CMD    fecRFRunTimeCalCmd;

    T_RL_API_FECSS_RF_RUN_CAL_RSP    fecRFRunTimeCalRsp;

    bool                             useRunTimeCalib;

    bool                        useCustomCalibration;

    uint16_t             customCalibrationEnableMask;

    T_RL_API_FECSS_RDIF_CTRL_CMD      fecRDIFCtrlCmd;

}MMWave_OpenCfg;

typedef struct ATE_CalibData_s
{
    /* Flag to indicate the calibration data is valid */
    uint32_t            validityFlag;

    /* Factory Calibration data */
    uint8_t            factoryCalData[ATE_CALIB_DATA_LENGTH];

} ATE_CalibData;
/*!
 * @brief  Sensor Perchirp LUT, total 64 bytes used, 4 values per params
 */
typedef struct
{
    uint32_t StartFreqHighRes[4]; /* LUT address 0 */
    uint32_t StartFreqLowRes[4]; /* LUT address 16 */
    int16_t ChirpSlope[4]; /* LUT address 32 */
    uint16_t ChirpIdleTime[4]; /* LUT address 40 */
    uint16_t ChirpAdcStartTime[4]; /* LUT address 48 */
    int16_t ChirpTxStartTime[4]; /* LUT address 56 */
    uint8_t ChirpTxEn[4]; /* LUT address 64 */
    uint8_t ChirpBpmEn[4]; /* LUT address 68 */
} T_SensPerChirpLut;

/**
 * @brief
 *  Generic List Node structure
 *
 * @details
 *  The structure defines a LIST NODE structure that contains links to the
 *  previous and next element in the list.
 */
typedef struct MMWave_ListNode_t
{
    /**
     * @brief   Pointer to the next element in the list.
     */
    void*   p_next;

    /**
     * @brief   Pointer to the prev element in the list.
     */
    void*   p_prev;
}MMWave_ListNode;
/**
 * @brief
 *  mmWave Spawn function entry
 *
 * @details
 *  The structure is used to keep track of the spawn functions which
 *  need to be executed by the control module.
 */
typedef struct MMWave_SpawnFxnNode_t
{
    /**
     * @brief   Links to the other elements in the list
     */
    MMWave_ListNode         links;

    /**
     * @brief   Spawn function entry
     */
    T_DFP_OSI_SPAWN_HANDLER    spawnEntry;

    /**
     * @brief   Argument passed to the spawn function
     */
    const void*             arg;
}MMWave_SpawnFxnNode;
/**
 * @brief
 *  Initial Configuration
 *
 * @details
 *  The structure specifies the configuration which is required to initialize
 *  and setup the mmWave module.
 */
typedef struct MMWave_InitCfg_t
{
    T_RL_API_FECSS_DEV_PWR_ON_CMD     fecDevPwrOnCmd;

    T_RL_API_FECSS_DEV_CLK_CTRL_CMD   fecDevClkCtrlCmd;

    T_RL_API_FECSS_RF_PWR_CFG_CMD     fecRFPwrCfgCmd;

    T_RL_API_FECSS_RF_STS_GET_RSP     fecRFStsGetRsp;

    bool                              useBootCalib;

    T_RL_API_FECSS_RF_FACT_CAL_CMD    fecRFFactoryCalCmd;

    T_RL_API_FECSS_RF_FACT_CAL_RSP    fecRFFactoryCalRsp;

    uint8_t                           *ptrAteCalibration;

    bool                              iswarmstart; /* For RPMF*/
}MMWave_InitCfg;

/**
 * @brief
 *  Error Level
 *
 * @details
 *  The mmWave module API can return different error levels. The enumeration
 *  describes different error levels. Please refer to the MMWave error decode
 *  function on the interpretation of this error level.
 *
 *  @sa
 *      MMWave_decodeError
 */
typedef enum MMWave_ErrorLevel_e
{
    /**
     * @brief   The mmWave API was successful. There were no errors detected.
     * There is no reason to perform any error decode here.
     */
    MMWave_ErrorLevel_SUCCESS   = 0,

    /**
     * @brief   The mmWave API reported a warning. Application can either ignore this
     * error message *OR* can perform the error decoding to get more information
     * on the actual reason.
     */
    MMWave_ErrorLevel_WARNING,

    /**
     * @brief   The mmWave API reported an error and applications should perform
     * error decoding to get the exact reason for the failure.
     */
    MMWave_ErrorLevel_ERROR
}MMWave_ErrorLevel;

typedef struct MMWave_calibCfg_t
{
    T_RL_API_FECSS_RF_FACT_CAL_CMD    fecRFFactoryCalCmd;

    T_RL_API_FECSS_RF_FACT_CAL_RSP    fecRFFactoryCalRsp;

    T_RL_API_FECSS_FACT_CAL_DATA      *ptrFactoryCalibData;

    uint8_t                           *ptrAteCalibration;

    bool                              isFactoryCalEnabled;

    bool                              isATECalibEfused;

}MMWave_calibCfg;

typedef struct MmwDemo_calibCfg_t
{
    /*! @brief      Size of Calibraton data size includng header */
    uint32_t 		sizeOfCalibDataStorage;

    /*! @brief      Enable/Disable calibration restore process  */
    uint32_t 		restoreEnable;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t 		flashOffset;
} MmwDemo_ateCalibCfg;
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
    uint32_t 	    magic;

    /*! @brief      Calibration data */
    T_RL_API_FECSS_FACT_CAL_DATA  calibData;
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

    /*! @brief      TX channels power back-off setting for calibration. */
    uint8_t         txBackoffSel;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t        flashOffset;

    /*! @brief      Is RF Trimmed? */
    bool        atecalibinEfuse;
} MmwDemo_factoryCalibCfg;

typedef struct Frame_Stats_t
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
} Frame_Stats;

typedef struct MmwDemo_MSS_MCB_t
{

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;
    
    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 loggingUartHandle;

    SPI_Handle                  loggingSpiHandle;
    
    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            demoInitTaskCompleteSemHandle;

    
    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object            cliInitTaskCompleteSemHandle;

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
     * @brief sensor start command data structure
     *
     */
    T_RL_API_SENSOR_START_CMD                  sensorStart;

    /*! @brief  MIMO mode selection: 0-TDM-MIMO, 1-BPM-MIMO  */
    uint8_t isBpmEnabled;
    /* DPC configs from CLI */
    float                               adcStartTime;
    float                               startFreq;
    float                               chirpSlope;
    float                               adcSamplingRate;
    float                               burstPeriod;
    uint32_t                            numRangeBins;
    uint32_t                            numDopplerBins;
    uint8_t                             angleDimension;
    uint32_t                            sensorStopCount;
    MmwDemo_ateCalibCfg                 mmwAteCalibCfg;

        /*! @brief      Frame stats structure */
    Frame_Stats                         stats;

    MMWave_CtrlCfg                      mmwCtrlCfg;
    MMWave_OpenCfg                      mmwOpenCfg;
        /*! @brief Flag to control in low power mode some configuration parts to be executed only once, and not to be repeated from frame to frame */
    uint8_t                             oneTimeConfigDone;
    /*! @brief Flag: 0-Low power mode disabled, 1-Low Power mode enabled, 2-Used for testing low power mode */
    uint8_t                             lowPowerMode;
    uint32_t                            chirpsize;
    uint32_t                            rxnum;
    /*! @brief  This is introduced to support minor motion detection in the power saving mode */
    uint32_t                            frmCntrModNumFramesPerMinorMot;

    /*! @brief  This is used in testing the low power mode */
    uint64_t                            frameStartTimeStampSlowClk;
    /*! @brief Token is checked in the frame start ISR, asserted to have zero value, and incremented. At the end of UART task, it is decremented */
    uint32_t                            interSubFrameProcToken;
    /*! @brief  Factory calibration cofiguration for save/restore */
    MmwDemo_factoryCalibCfg             factoryCalCfg;
} MmwDemo_MSS_MCB;

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
    char fileName[500];

}ADC_Data_Source_Cfg;


/**
 * @brief
 *  mmWave Chirp configuration
 *
 * @details
 *  The structure is used to keep track of the chirps which
 *  have been configured in the system.
 */
typedef struct MMWave_Chirp_t
{
    /**
     * @brief   Links to the other elements in the list
     */
    MMWave_ListNode             links;

    /**
     * @brief   Chirp configuration
     */
    T_RL_API_SENS_PER_CHIRP_CFG                chirpCfg;

    /**
     * @brief   Chirp control configuration
     */
    T_RL_API_SENS_PER_CHIRP_CTRL               chirpCtrl;

    /**
     * @brief   Profile for which the chirp is configured
     */
    struct MMWave_Profile_t*    ptrMMWaveProfile;
}MMWave_Chirp;

/**
 * @brief
 *  mmWave Profile configuration
 *
 * @details
 *  The structure is used to keep track of the profiles
 */
typedef struct MMWave_Profile_t
{
    /**
     * @brief   Links to the other elements in the list
     */
    MMWave_ListNode         links;

    /**
     * @brief  Profile index (0-3)
     */
    uint16_t profileId;

    /**
     * @brief   Profile configuration
     */
    T_RL_API_SENS_CHIRP_PROF_COMN_CFG          profileCfg;

    T_RL_API_SENS_CHIRP_PROF_TIME_CFG          profileTimeCfg;

    /**
     * @brief   Link to the mmWave
     */
    struct MMWave_MCB_t*    ptrMMWaveMCB;

    /**
     * @brief   Number of chirps
     */
    uint32_t                numChirps;

    /**
     * @brief   List of all the chirps associated with the profile
     */
    MMWave_Chirp*           ptrChirpList;
}MMWave_Profile;


/**
 * @brief
 *  mmWave Control MCB
 *
 * @details
 *  The structure is used to store all the relevant information required
 *  to execute the mmWave control module.
 */
typedef struct MMWave_MCB_t
{
    T_DFP_CLIENT_CB_DATA        fecClientCbData;
    /**
     * @brief   Initialization configuration which was used to initialize the
     * control module.
     */
    MMWave_InitCfg              initCfg;
    /**
     * @brief   Factory Calibration configuration which was used to perform the
     * factory calibration of Sensor.
     */
    MMWave_calibCfg             factoryCalCfg;

    /**
     * @brief   Configuration which is used to open the control module.
     */
    MMWave_OpenCfg              openCfg;

    /**
     * @brief   This is the semaphore handle which is used to handle the messages exchanged
     * between the MSS/DSS and BSS.
     */
    SemaphoreP_Object           linkSemHandle;

    /**
     * @brief   This is the semaphore handle which is used to protect the chirp/profile
     * configuration while operating in FULL configuration mode.
     */
    SemaphoreP_Object           cfgSemHandle;

    /**
     * @brief   This is the spawn table used by the free & active lists.
     */
    MMWave_SpawnFxnNode         spawnTable[MMWAVE_MAX_NUM_SPAWN_LIST];

    /**
     * @brief   List which tracks all the nodes in the spawn free list.
     */
    MMWave_SpawnFxnNode*        ptrSpawnFxnFreeList;

    /**
     * @brief   List which tracks all the nodes in the spawn list which are
     * to be executed.
     */
    MMWave_SpawnFxnNode*        ptrSpawnFxnActiveList;

    /**
     * @brief   List which tracks all the profiles which have been created
     */
    MMWave_Profile*             ptrProfileList;
    
    /**
     * @brief   Status of the mmWave control module:
     */
    uint32_t                    status;

    /**
     * @brief   Version Information which is stored once the link is operational
     */
    T_RL_API_DFP_FW_VER_GET_RSP                 version;
        
       
    /**
     * @brief   mmWaveLink Event Handler callback
     */
    T_DFP_PLT_ISR_HANDLER          mmwavelinkInterruptFunc;
    
    /**
     * @brief   Bitmap of devices to send the message. Check the mmwavelink documentation.
     */
    uint8_t                   deviceMap;

}MMWave_MCB;
/**
 * @brief
 *  Temperature measurements
 *
 * @details
 *  The structure is used to hold all the relevant temperature
 *  measurements - Rx, Tx, PM, DIG.
 */
typedef struct MMWave_temperatureStats_t
{
    /*! @brief      Temp meas status returned by the meas API */
    uint16_t       tempStatus;

    /*! @brief      Temp meas values returned by the meas API */
    int16_t        tempValue[4];
} MMWave_temperatureStats;

void MMWave_decodeError
(
    int32_t             errCode,
    MMWave_ErrorLevel*  errorLevel,
    int16_t*            mmWaveError,
    int16_t*            subSysError
);
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
int32_t mmw_link_Initialization(bool iswarmstrt);
int32_t MMWave_LinkInitCommon(MMWave_MCB* ptrMMWaveMCB, int32_t* errCode);
int32_t MMWave_stop (MMWave_Handle mmWaveHandle, int32_t* errCode);
int32_t MMWave_close (MMWave_Handle mmWaveHandle, int32_t* errCode);
int32_t MMWave_deinit (MMWave_Handle mmWaveHandle, int32_t* errCode);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }
void mmw_SPIWrite (SPI_Handle handle, uint8_t *payload, uint32_t payloadLength);
void mmwDemo_freeDmaChannels(EDMA_Handle edmaHandle);
int32_t MMWave_delProfile
(
    MMWave_Handle           mmWaveHandle,
    MMWave_ProfileHandle    profileHandle,
    int32_t*                errCode
);
int32_t MMWave_factoryCalibConfig (MMWave_Handle mmWaveHandle, MMWave_calibCfg* ptrfactoryCalibCfg, int32_t* errCode);
#ifdef __cplusplus
}
#endif

#endif /* LINK_H */