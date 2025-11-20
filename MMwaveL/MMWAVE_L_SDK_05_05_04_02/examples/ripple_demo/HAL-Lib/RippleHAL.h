////////////////////////////////////////////////////////////////////////////////////////////////////


/*
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

/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <stdint.h>


  
struct ChannelCfg{
  int NumofRx;
  int rxChannelEn;
  int txChannelEn;
  int cascading;
  int NumtxEnabled;
};

struct ProfileCfg{
  int dfeDataOutputMode;
  int profileId;
  int startFreq;
  //endFreq is a additionally added profile Config 
  float endFreq;
  float IdleTime;
  //ChirpPeriod is a additionally added profile Config
  int ChirpPeriod;
  float adcStartTime;
  //maxTxOutpower in DB;
  int DevicemaxTxOutpower;
  float rampEndTime;
  int txOutPower;
  int txOutPower_backoff;
  int txPhaseShifter;
  float freqSlopeConst;
  int txStartTime;
  int numAdcSamples;
  float digOutSampleRate;
  int hpfCornerFreq1;
  int hpfCornerFreq2;
  int rxGain;
  float freqDiff;
};

struct FrameCfg {
  int chirpStartIndex;
  int chirEndIndex;
  int NumofChirps;
  int BytesPerChirp;
  int NumofLoops;
  int NumofFrames;
  int framePeriodicity;
  int triggerselect;
  int frametriggerdelay;

};

struct adcCfg
{
  /* data */
  int NumADCBits;
  int NumADCBitsCfg;
  int adcOutputFmt;
  int adcOutputFmtCfg;
  int adcBufCfg;
};


struct lowPowerCfg{
  int interchirpmode;
  int interburstmode;
  int mode; 
};
typedef struct {
  int fd;
  int fd_l;
  int numConfigslots;
  
}RadarHandle;

struct SensorInfo {
 const char name;
 const char vendor;
 uint32_t device_id;
//  Version driver_version;
//  Version api_version;
};
//the radar burst format and shall be provided with every burst
typedef struct {
uint32_t sequence_number;
uint32_t max_sample_value;
uint8_t bits_per_sample;
uint16_t samples_per_chirp;
uint8_t channels_count;
uint8_t chirps_per_burst;
uint8_t config_id;
union {
 struct{
      uint16_t is_channels_interleaved: 1;
      uint16_t is_big_endian: 1;
      uint16_t reserved: 14;
      };
      uint16_t flags;
      };
      uint32_t burst_data_crc;
      uint32_t timestamp_ms;
}RadarBurstFormat;

typedef int timespec_t ;

//-------------------------------------
////----- Enums --------------------------
//--------------------------------------

// A list of possible status/return codes that API can report back.
typedef enum {
  // A default undefined value that should be used at initialization.
  RC_UNDEFINED = 0,
  // Operation completed successfully.
  RC_OK,
  // Operation failed and no more information can be provided.
  RC_ERROR,
  // Input parameters are invalid or out of supported range.
  RC_BAD_INPUT,
  // Operation timed out.
  RC_TIMEOUT,
  // Operation cannot be performed at the current state.
  RC_BAD_STATE,
  // Operation failed due to limited resources (memory, timers, mutexes, etc).
  RC_RES_LIMIT,
  // Operation is not supported.
  RC_UNSUPPORTED,
  // An internal system error that should never happen.
  RC_OOPS
} RadarReturnCode;

// A list of possible power mode states for radar sensors.
typedef enum {
  // A default undefined value that should be used at initialization.
  RSTATE_UNDEFINED = 0,
  // Active state when radar is emitting/collecting data started.
  RSTATE_ACTIVE,
  // Idle state when the radar is neither ACTIVE nor SLEEP nor OFF.
  RSTATE_IDLE,
  // Sleep state when configuration persists but power consumption reduced.
  RSTATE_SLEEP,
  // When radar is currently turned off and configuration is reset.
  RSTATE_OFF
} RadarState;


//--------------------------------------
//----- Params -------------------------
//--------------------------------------

// A list of radar sensor parameters that define main characteristics.
// A configuration slot can hold only 1 value for each MainParam.
typedef enum {
  // A default undefined value that should be used at initialization.
  RADAR_PARAM_UNDEFINED = 0,
  // Power mode for after the burst period.
  RADAR_PARAM_AFTERBURST_POWER_MODE,
  // Power mode for the period between chirps.
  RADAR_PARAM_INTERCHIRP_POWER_MODE,
  // Duration between the start times of two consecutive bursts.
  RADAR_PARAM_BURST_PERIOD_US,
  // Duration between the start times of two consecutive chirps.
  RADAR_PARAM_CHIRP_PERIOD_US,
  // Amount of chirps within the burst.
  RADAR_PARAM_CHIRPS_PER_BURST,
  // The number of ADC sample values captured for each chirp.
  RADAR_PARAM_SAMPLES_PER_CHIRP,
  // The lower frequency at what TX antenna starts emitting the signal.
  RADAR_PARAM_LOWER_FREQ_MHZ,
  // The upper frequency at what TX antenna stops emitting the signal.
  RADAR_PARAM_UPPER_FREQ_MHZ,
  // Bit mask for enabled TX antennas.
  RADAR_PARAM_TX_ANTENNA_MASK,
  // Bit mask for enabled RX antennas.
  RADAR_PARAM_RX_ANTENNA_MASK,
  // Unused Parameter.
  RADAR_PARAM_UNUSED_0,
  // ADC sampling frequency.
  RADAR_PARAM_ADC_SAMPLING_HZ
} RadarMainParam;

// A Tx specific list of parameters.
typedef enum {
  // A default undefined value that should be used at initialization.
  TX_PARAM_UNDEFINED = 0,
  // TX antenna's emitting power in dB.
  TX_PARAM_POWER_DB
} RadarTxParam;

// A Rx Specific liist of parameters.
typedef enum{
  // A default undefined value that should be used at initialization.
  RX_PARAM_UNDEFINED =0,
  // Rx Variable Gain Amplifiers(VGA) in dB
  RX_PARAM_VGA_DB,
  //High Pass(HP) filter  Gain in dB
  RX_PARAM_HP_GAIN_DB,
  //High Pass(HP) cutoff frequency in kHz
  RX_PARAM_HP_CUTOFF_KHZ
} RadarRxParam;

// Forward declaration for a list of vensor specific parameters.
typedef enum {
  // A default undefined value that should be used at initialization.
  RADAR_VENDOR_PARAM_UNDEFINED = 0,
  // Chirp Idle Time ( Time period Between Ramp End and Ramp Start time)(in us)(set to 2200us for default)
  RADAR_VENDOR_PARAM_CHIRP_IDLE_TIME,
  // Set Number of bursts(valid range is 0 to 65535, 0 means infinite)
  RADAR_VENDOR_PARAM_NUM_OF_BURSTS
} RadarVendorParam;



typedef enum {
  // A default undefined value that should be used at initialization.
  RLOG_UNDEFINED = 0,
  // None of log messages are requested.
  RLOG_OFF,
  // Provide only log messages about occurred errors.
  RLOG_ERR,
  // Provide log messages same as for RLOG_ERR and warnings.
  RLOG_WRN,
  // Provide log messages same as for RLOG_WRN and informative changes.
  RLOG_INF,
  // Provide log messages same as for RLOG_INF and debugging info details.
  RLOG_DBG
} RadarLogLevel;

// /**
//  * @brief
//  * Handle
//  */
// typedef struct RadarHandle;


typedef struct user_data_struct{
    RadarHandle* handle;
    RadarBurstFormat format;
    uint32_t* read_bytes;
    uint8_t read_buffer[1000000];
    timespec_t timeout;
 } UserData;

typedef void (*RadarBurstReadyCB)(void* user_data);
RadarReturnCode radarSetBurstReadyCb(RadarHandle* handle, RadarBurstReadyCB cb,void* user_data);
RadarHandle* radarCreate(int32_t id);
RadarReturnCode radarTurnOff(RadarHandle* handle);
RadarReturnCode radarTurnOn(RadarHandle* handle);
RadarReturnCode radarSetMainParam(RadarHandle* handle, uint32_t slot_id,RadarMainParam id, uint32_t value);
RadarReturnCode radarActivateConfig(RadarHandle* handle, int8_t slot_id);
RadarReturnCode radarSetVendorParam(RadarHandle* handle, uint32_t slot_id, RadarVendorParam id, uint32_t value);
RadarReturnCode radarSetTxParam(RadarHandle* handle, uint8_t slotid, uint32_t antenna_mask, RadarTxParam id ,uint32_t value);
RadarReturnCode radarSetRxParam(RadarHandle* handle, uint8_t slotid, uint32_t antenna_mask, RadarRxParam id ,uint32_t value);
RadarReturnCode radarStartDataStreaming(RadarHandle* handle);
RadarReturnCode radarGetVendorParam(RadarHandle* handle, uint32_t slot_id,RadarVendorParam id, uint32_t* value);
RadarReturnCode radarGetMainParam(RadarHandle* handle, uint32_t slot_id,RadarMainParam id, uint32_t* value);
RadarReturnCode radarReadBurst(RadarHandle* handle, RadarBurstFormat* format,uint8_t* buffer, uint32_t* read_bytes,timespec_t timeout);
void OnBurstReady(void* userData);
RadarReturnCode radarStopDataStreaming(RadarHandle* handle);
