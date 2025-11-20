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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include "CLI_Write.h"
#include "RippleHAL.h"


#define _NCCS 19

#ifndef __cplusplus
typedef unsigned char bool;
static const bool False = 0;
static const bool True = 1;
#endif

void *userdataClass = NULL;
RadarBurstReadyCB burstreadyCB;

struct ProfileCfg profileCfg_ins;
struct FrameCfg FrameCfg_ins;
struct ChannelCfg channelCfg_ins;
struct adcCfg adcCfg_ins;
struct lowPowerCfg lowPowerCfg_ins;

//--------------------------------------
//----- API ----------------------------
//--------------------------------------

// Lifecycle.

/*
 * @brief Initialize a radar module.
 *
 * @note This function should be called the most first of all radar API.
 */
RadarReturnCode radarInit(void){
	return RC_UNSUPPORTED;
};
/*
 * @brief De-initialize a radar module.
 *
 * @note This function should be called the most last of all radar API.
 */
RadarReturnCode radarDeinit(void){
	return RC_UNSUPPORTED;
};

/*
 * @brief Create a radar module instance.
 *
 * @param id a unique identifier of the radar chip.
 *        Can be used to differentiate if multiple radar available at the
 *        same time.
 *
 * @return A handler for a newly created radar instance.
 */
RadarHandle* radarCreate(int32_t id){

	int fd_a;
	char cmdString[256],dataPortString2[12],dataPortString[12];
	struct termios options; 

	printf("Enter Data Port 1:(Example: /dev/ttyACM0)");
	scanf("%s",dataPortString);
		
	/* Serial port 1 setting */
	fd_a = open(dataPortString, O_RDWR);
	if (fd_a < 0) {
		perror("Error opening serial port");
		// return -1;
	}

	/* Set up serial port */
	/*New settings*/
	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag &= ~(CSIZE | PARENB);
	options.c_cflag |= CS8;
	options.c_cc[VTIME]=10;
	options.c_cc[VMIN]=255;
	cfsetspeed(&options, B115200);

	/* Apply the settings */
	tcflush(fd_a, TCIOFLUSH);
	/* saving termios settings andalso checking for error*/
	if(tcsetattr(fd_a, TCSANOW, &options)!= 0){
		printf("Error %i from tcsetattr: %s\n",errno,strerror(errno));
	}
	
	clean_buffer(fd_a);
	//returning the Pointer to Handle
	RadarHandle handle;
	RadarHandle *ptrhandle = (RadarHandle *)malloc(sizeof(handle));
	ptrhandle->fd=fd_a;
	
	//setting the Default Vendor Params 
	profileCfg_ins.IdleTime=150;	//(us)
	profileCfg_ins.adcStartTime=3;
	profileCfg_ins.DevicemaxTxOutpower =26;
	profileCfg_ins.dfeDataOutputMode=1;
	adcCfg_ins.NumADCBitsCfg=2;
	adcCfg_ins.NumADCBits=16;
	adcCfg_ins.adcOutputFmtCfg=1;
	adcCfg_ins.adcOutputFmt=2;
	adcCfg_ins.adcBufCfg=0;
	FrameCfg_ins.NumofFrames=100;
	
	
	return ptrhandle;
};

/*
 * @brief Destroy a radar module instance.
 *
 * @param handle the handle of the radar instance to be destroyed.
 */
RadarReturnCode radarDestroy(RadarHandle* handle){
	close(handle->fd);
	close(handle->fd_l);
	return RC_OK;
};

// Power management.

/*
 * @brief Get the current power state.
 *
 * @param handle a handler for the radar instance to use.
 * @param state a pointer to the state that will be set.
 */
RadarReturnCode radarGetState(RadarHandle* handle, RadarState* state){
	return RC_UNSUPPORTED;
};

/*
 * @brief Turn on the radar.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarTurnOn(RadarHandle* handle){
	CLI_Just_Write(handle->fd,"sensorStart 0 0 0 0\n\r");
	return 1;
};

/*
 * @brief Turn off the radar.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarTurnOff(RadarHandle* handle){
	tcflush(handle->fd, TCIFLUSH);
	tcflush(handle->fd, TCOFLUSH);
    if(CLI_Write_UART(handle->fd,"sensorStop 0\n\r")==1){
		return RC_OK;
	};
};

/*
 * @brief Put the radar to sleep and preserve configuration.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarGoSleep(RadarHandle* handle){
	return RC_UNSUPPORTED;
};

/*
 * @brief Wake up the radar.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarWakeUp(RadarHandle* handle){
	return RC_UNSUPPORTED;
};

// Configuration.
/*
 * Get the total available configuration slots.
 *
 * @param handle a handler for the radar instance to use.
 * @param num_slots a pointer to where the number of config slots to write.
 */
RadarReturnCode radarGetNumConfigSlots(RadarHandle* handle, int8_t* num_slots){
	return RC_UNSUPPORTED;
};

/*
 * @brief Activate a specified configuration slot. Does not start the radar.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID to activate.
 *
 * @note This function will perform the final configuration check for
 *       compatibility before activating.
 */
RadarReturnCode radarActivateConfig(RadarHandle* handle, int8_t slot_id){

	//Commands Library
	char defaultCliCmd[28][100] = {
				"channelCfg ",
				"chirpComnCfg ",
				"chirpTimingCfg ",
				"frameCfg ",
				"lowPowerCfg ",
				"factoryCalibCfg ",
				"sensorStart " };
	
	char cliStr[100] = { 0 };


	//NecessaryCalculations
	channelCfg_ins.NumofRx= rxMaptoRxNum(channelCfg_ins.rxChannelEn);
	channelCfg_ins.NumtxEnabled=rxMaptoRxNum(channelCfg_ins.txChannelEn);
	profileCfg_ins.rampEndTime=(profileCfg_ins.ChirpPeriod-profileCfg_ins.IdleTime);
	profileCfg_ins.freqDiff=profileCfg_ins.endFreq-profileCfg_ins.startFreq;
	profileCfg_ins.txOutPower_backoff =(profileCfg_ins.txOutPower-profileCfg_ins.DevicemaxTxOutpower);
	if(profileCfg_ins.txOutPower_backoff <0){
		profileCfg_ins.txOutPower_backoff = (-1)*(profileCfg_ins.txOutPower_backoff);
	}
	profileCfg_ins.freqSlopeConst=(profileCfg_ins.freqDiff/profileCfg_ins.rampEndTime);
	if(lowPowerCfg_ins.interchirpmode==1 || lowPowerCfg_ins.interburstmode==1){lowPowerCfg_ins.mode=1;}
	FrameCfg_ins.BytesPerChirp=channelCfg_ins.NumofRx * adcCfg_ins.NumADCBitsCfg * profileCfg_ins.numAdcSamples;
	sprintf(cliStr,"%s""%d"" ""%d"" ""%s""\n\r",defaultCliCmd[0],channelCfg_ins.rxChannelEn,channelCfg_ins.txChannelEn,"0");
	CLI_Write_UART(handle->fd,cliStr); //channelCfg

	sprintf(cliStr,"%s""%.0f""%s""%d""%s"" ""%.0f"" 0""\n\r",
	defaultCliCmd[1],
	profileCfg_ins.digOutSampleRate,
	" 0 0 ",
	profileCfg_ins.numAdcSamples,
	" 0",
	profileCfg_ins.rampEndTime);
	CLI_Write_UART(handle->fd,cliStr);	//chirpComnCfg
	
	sprintf(cliStr,"%s""%.0f"" ""%s"" ""%.0f"" ""%d""\n\r",
	defaultCliCmd[2],
	profileCfg_ins.IdleTime,
	"24 0",
	profileCfg_ins.freqSlopeConst,
	(profileCfg_ins.startFreq/1000));
	CLI_Write_UART(handle->fd,cliStr);	//chirpTimingCfg

	sprintf(cliStr,"%s""%d""%s""%d""%s""%d"" ""%d""\n\r",
	defaultCliCmd[3],
	FrameCfg_ins.NumofLoops,
	" 0 ",
	FrameCfg_ins.framePeriodicity,
	" 1 ",
	FrameCfg_ins.framePeriodicity,
	FrameCfg_ins.NumofFrames);
	CLI_Write_UART(handle->fd,cliStr);	//frameCfg

	sprintf(cliStr,"%s""%d""\n\r",
	defaultCliCmd[4],
	lowPowerCfg_ins.mode);
	CLI_Write_UART(handle->fd,cliStr);	//lowPowerCfg

	sprintf(cliStr,"%s""%s""%d"" ""%d""%s""\n\r",
	defaultCliCmd[5],
	"1 0 ",
	profileCfg_ins.rxGain,
	profileCfg_ins.txOutPower_backoff,
	" 0x1ff000"
	);
	CLI_Write_UART(handle->fd,cliStr);	//factoryCalibCfg
	return 1;
};

/*
 * @brief Deactivate a specified configuration slot.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID to deactivate.
 */
RadarReturnCode radarDeactivateConfig(RadarHandle* handle, int8_t slot_id){
	return RC_UNSUPPORTED;
};

/*
 * @brief Check if the configuration slot is active.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID to check if active.
 * @param is_active a pointer to where the result will be written into.
 */
RadarReturnCode radarIsActiveConfig(RadarHandle* handle, int8_t slot_id, bool* is_active){
	return RC_UNSUPPORTED;
};

/*
 * @brief Get a main radar parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to read the parameter value.
 * @param id a parameter ID to be read.
 * @param value a pointer to where a parameter value will be written into.
 */

RadarReturnCode radarGetMainParam(RadarHandle* handle, uint32_t slot_id,
                             RadarMainParam id, uint32_t* value){
	
	switch(id){
	case RADAR_PARAM_UNDEFINED:
	break;
	// Power mode for after the burst period.
	case RADAR_PARAM_AFTERBURST_POWER_MODE:
	*value=lowPowerCfg_ins.interburstmode;
	break;
	// Power mode for the period between chirps.
	case RADAR_PARAM_INTERCHIRP_POWER_MODE:
	break;
	// Duration between the start times of two consecutive bursts.
	case RADAR_PARAM_BURST_PERIOD_US:
	*value= FrameCfg_ins.framePeriodicity;
	// printf("%u",FrameCfg_ins.framePeriodicity);
	break;
	// Duration between the start times of two consecutive chirps.
	case RADAR_PARAM_CHIRP_PERIOD_US:
	*value=profileCfg_ins.ChirpPeriod;
	break;
	// Amount of chirps within the burst.
	case RADAR_PARAM_CHIRPS_PER_BURST:
	*value=FrameCfg_ins.NumofLoops;
	break;
	// The number of ADC sample values captured for each chirp.
	case RADAR_PARAM_SAMPLES_PER_CHIRP:
	*value=profileCfg_ins.numAdcSamples;
	break;
	// The lower frequency at what TX antenna starts emitting the signal.
	case RADAR_PARAM_LOWER_FREQ_MHZ:
	*value=profileCfg_ins.startFreq;
	break;
	// The upper frequency at what TX antenna stops emitting the signal.
	case RADAR_PARAM_UPPER_FREQ_MHZ:
	// value=(void *)profileCfg_ins.endFreq;
	break;
	// Bit mask for enabled TX antennas.
	case RADAR_PARAM_TX_ANTENNA_MASK:
	*value=channelCfg_ins.txChannelEn;
	break;
	// Bit mask for enabled RX antennas.
	case RADAR_PARAM_RX_ANTENNA_MASK:
		*value=channelCfg_ins.rxChannelEn;

	break;
	// Unused Parameter.
	case RADAR_PARAM_UNUSED_0:
	break;
	// ADC sampling frequency.
	case RADAR_PARAM_ADC_SAMPLING_HZ:
	// value=(void *)profileCfg_ins.digOutSampleRate;
	break;
	};
	}
/*
 * @brief Set a main radar parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to set a new parameter value.
 * @param id a parameter ID to be set.
 * @param value a new value for the parameter.
 */
RadarReturnCode radarSetMainParam(RadarHandle* handle, uint32_t slot_id,RadarMainParam id, uint32_t value){
	switch(id){
	case RADAR_PARAM_UNDEFINED:
	break;
	// Power mode for after the burst period.
	case RADAR_PARAM_AFTERBURST_POWER_MODE:
	lowPowerCfg_ins.interburstmode=value;
	break;
	// Power mode for the period between chirps.
	case RADAR_PARAM_INTERCHIRP_POWER_MODE:
	lowPowerCfg_ins.interchirpmode=value;
	break;
	// Duration between the start times of two consecutive bursts.
	case RADAR_PARAM_BURST_PERIOD_US:
	FrameCfg_ins.framePeriodicity=value;
	break;
	// Duration between the start times of two consecutive chirps.
	case RADAR_PARAM_CHIRP_PERIOD_US:
	profileCfg_ins.ChirpPeriod=value;
	break;
	// Amount of chirps within the burst.
	case RADAR_PARAM_CHIRPS_PER_BURST:
	FrameCfg_ins.NumofLoops=value;
	break;
	// The number of ADC sample values captured for each chirp.
	case RADAR_PARAM_SAMPLES_PER_CHIRP:
	profileCfg_ins.numAdcSamples=value;
	break;
	// The lower frequency at what TX antenna starts emitting the signal.
	case RADAR_PARAM_LOWER_FREQ_MHZ:
	profileCfg_ins.startFreq=value;
	break;
	// The upper frequency at what TX antenna stops emitting the signal.
	case RADAR_PARAM_UPPER_FREQ_MHZ:
	profileCfg_ins.endFreq=value;
	break;
	// Bit mask for enabled TX antennas.
	case RADAR_PARAM_TX_ANTENNA_MASK:
	channelCfg_ins.txChannelEn=value;
	break;
	// Bit mask for enabled RX antennas.
	case RADAR_PARAM_RX_ANTENNA_MASK:
		channelCfg_ins.rxChannelEn=value;
		//default cascading =0 as Soc cascading, not applicable, set to 0
		channelCfg_ins.cascading=0;
	break;
	// Unused Parameter.
	case RADAR_PARAM_UNUSED_0:
	break;
	// ADC sampling frequency.
	case RADAR_PARAM_ADC_SAMPLING_HZ:
	profileCfg_ins.digOutSampleRate= (100000000/value); ///profileCfg_ins.digOutSampleRate= (100/(value/1000000);
	break;

	}
	return 1;
};

/*
 * @brief Get a main radar parameter range of acceptable values.
 *
 * @param handle a handler for the radar instance to use.
 * @param id a parameter ID which range of values to read.
 * @param min_value a pointer where a minimum parameter value will be set.
 * @param max_value a pointer where a maximum parameter value will be set.
 */
RadarReturnCode radarGetMainParamRange(RadarHandle* handle, RadarMainParam id,uint32_t* min_value, uint32_t* max_value){
};


/*
 * @brief Set a TX antenna specific operating radar parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to set a new parameter value.
 * @param id a parameter ID to be set.
 * @param value a new value for the parameter.
 */
RadarReturnCode radarSetTxParam(RadarHandle* handle, uint8_t slotid, uint32_t antenna_mask, RadarTxParam id ,uint32_t value){
	if (id==0)
	{/* code */}
	if (id==1)
	{	 // TX antenna's emitting power in dB.
		 profileCfg_ins.txOutPower = value;
	}
	return 1;
};
/*
 * @brief Set a RX antenna specific operating radar parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to set a new parameter value.
 * @param id a parameter ID to be set.
 * @param value a new value for the parameter.
 */
RadarReturnCode radarSetRxParam(RadarHandle* handle, uint8_t slotid, uint32_t antenna_mask, RadarRxParam id ,uint32_t value){
	if (id==0)
	{
		/* code */
	}
	if (id==1)
	{
		 // TX antenna's emitting power in dB.
		 profileCfg_ins.rxGain=value;
	}
	//hpfCornerFrequencies
	if (id==3){
		int arr[]={175,235,350,700};
		profileCfg_ins.hpfCornerFreq1=findClosestFreq(arr, value);
		int arr2[]={350,700,1400,2800};
		profileCfg_ins.hpfCornerFreq2=findClosestFreq(arr2,value);
	}
	return 1;
};
/*
 * @brief Get a vendor specific parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to read a parameter value.
 * @param id a vendor specific parameter ID to read.
 * @param value a pointer to where a parameter value will be written into.
 */
RadarReturnCode radarGetVendorParam(RadarHandle* handle, uint32_t slot_id,RadarVendorParam id, uint32_t* value){
	switch(id){
		case RADAR_VENDOR_PARAM_UNDEFINED:
		break;
		case RADAR_VENDOR_PARAM_CHIRP_IDLE_TIME:
		*value= profileCfg_ins.IdleTime;
		break;
		case RADAR_VENDOR_PARAM_NUM_OF_BURSTS:
		*value=FrameCfg_ins.NumofFrames;
		break;
	}
	return 1;
};

/*
 * @brief Set a vendor specific parameter.
 *
 * @param handle a handler for the radar instance to use.
 * @param slot_id a configuration slot ID where to set a new parameter value.
 * @param id a parameter ID to set.
 * @param value a new value for the parameter.
 */
RadarReturnCode radarSetVendorParam(RadarHandle* handle, uint32_t slot_id, RadarVendorParam id, uint32_t value){ 
	switch(id){
		case RADAR_VENDOR_PARAM_UNDEFINED:
		break;
		case RADAR_VENDOR_PARAM_CHIRP_IDLE_TIME:
		profileCfg_ins.IdleTime=value;
		break;
		case RADAR_VENDOR_PARAM_NUM_OF_BURSTS:
		FrameCfg_ins.NumofFrames=value;
		break;
	}
	return 1;
};


// Running.

/*
 * @brief Start running the radar with active configuration.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarStartDataStreaming(RadarHandle* handle){
	CLI_Just_Write(handle->fd,"sensorStart 0 0 0 0\n\r");
	return 1;
	};


/*
 * @brief Stop running the radar.
 *
 * @param handle a handler for the radar instance to use.
 */
RadarReturnCode radarStopDataStreaming(RadarHandle* handle){
    CLI_Write_UART(handle->fd,"sensorStop 0\n\r");
		return RC_OK;
};

/*
 * @brief Check if the radar has a new burst ready to read.
 *
 * @param handle a handler for the radar instance to use.
 * @param is_ready a pointer where the result will be set.
 */
RadarReturnCode radarIsBurstReady(RadarHandle* handle, bool* is_ready){
	return RC_UNSUPPORTED;
};

/*
 * @brief Initiate reading a new burst.
 *
 * @param handle a handler for the radar instance to use.
 * @param format a pointer where a new burst format will be written into.
 * @param buffer a pointer where a burst data to write.
 * @param read_bytes a pointer where the maximum buffer size is set.
 *        When function finishes, the pointer will have the amount of bytes
 *        have been read.
 * @param timeout the maximum time to wait if the burst frame is not ready.
 */
RadarReturnCode radarReadBurst(RadarHandle* handle, RadarBurstFormat* format,
                          uint8_t* buffer, uint32_t* read_bytes,
                          timespec_t timeout){
	int fd_B= handle->fd_l;
	int chirpLoopcount=FrameCfg_ins.NumofLoops ;
	uint32_t bytes_read=0,printCnt=0,readtrials=0,totalbytesreceived=0;
	uint8_t * storeaddress= buffer;
	uint32_t chirpsize=FrameCfg_ins.BytesPerChirp;
	while(chirpLoopcount){

	uint32_t rdcnt=0;
	while(rdcnt<chirpsize && readtrials<500){
			bytes_read = read(fd_B,storeaddress,chirpsize); /* Read the data */
			storeaddress+=bytes_read;
			rdcnt+=bytes_read;
			readtrials++;
			}
		
	chirpLoopcount--;
	}
	format->chirps_per_burst=FrameCfg_ins.NumofLoops;
	format->bits_per_sample=adcCfg_ins.NumADCBits * adcCfg_ins.adcOutputFmt;
	format->samples_per_chirp=profileCfg_ins.numAdcSamples;
	format->channels_count=channelCfg_ins.NumofRx;
	return 1;
};



// // Feedback.

/*
 * @brief Set a callback that will be invoked when a new burst is ready to read.
 *
 * @param handle a handler for the radar iTest.cnstance to use.
 * @param cb a callback function.
 * @param user_data a pointer to user_data that will be passed to the callback.
 */
RadarReturnCode radarSetBurstReadyCb(RadarHandle* handle, RadarBurstReadyCB cb,
                                void* user_data){
 cb(((UserData *)user_data));
 return 1; 
 };

void OnBurstReady(void* userData) {
    radarReadBurst(((UserData *)userData)->handle,&(((UserData *)userData)->format),&(((UserData *)userData)->read_buffer)[0],((UserData *)userData)->read_bytes,((UserData *)userData)->timeout);
}
/*
 * @biref Set a callback that will be invoked with a log message from
 *        the radar API impl.
 *
 * @param handle a handler for the radar instance to use.
 * @param cb a callback function.
 * @param user_data a pointer to user_data that will be passed to the callback.
 */
// RadarReturnCode radarSetLogCb(RadarHandle* handle, RadarLogCB cb, void* user_data){

// };

/*
 * @brief Set a callback that will be invoked when a radar’s register is set.
 *
 * @param handle a handler for the radar instance to use.
 * @param cb a callback function.
 * @param user_data a pointer to user_data that will be passed to the callback.
 */
// RadarReturnCode radarSetRegisterSetCb(RadarHandle* handle, RadarRegisterSetCB cb,
//                                  void* user_data){

//                                  };

// Miscellaneous.

/*
 * @brief Set country code. If local regulations do not allow current sensor
 *        to operate, it should be turned off or faile to turn on.
 *
 * @param handle a handler for the radar instance to use.
 * @param country_code a ISO 3166-1 alpha-2 country code.
 */
// RadarReturnCode radarSetCountryCode(RadarHandle* handle, const char* country_code){};

// /*
//  * @brief Get radar sensor info.
//  *
//  * @param handle a handler for the radar instance to use.
//  * @param info a pointer to the sensor info to be filled in.
//  */
// RadarReturnCode radarGetSensorInfo(RadarHandle* handle, SensorInfo* info){};

// /*
//  * @brief Set a run time log level for radar API impl.
//  *
//  * @param handle a handler for the radar instance to use.
//  * @param level new log level.
//  */
// RadarReturnCode radarSetLogLevel(RadarHandle* handle, RadarLogLevel level){};

// /*
//  * @brief Get all register values from the radar sensor.
//  *
//  * @param handle a handler for the radar instance to use.
//  * @param addresses a pointer where register addresses will be written into.
//  * @param values a pointer where register values will be written into.
//  * @param count a pointer where the maximum amount of elemnets for register
//  *        addresses and values can set. When function returns, the value
//  *        under the pointer will have the number of address-value pairs
//  *        have been set.
//  */
// RadarReturnCode radarGetAllRegisters(RadarHandle* handle, uint32_t* addresses,
//                                 uint32_t* values, uint32_t* count){};

// /*
//  * @brief Get a register value directly from the sensor.
//  *
//  * @param handle a handler for the radar instance to use.
//  * @param address an address to read.
//  * @param value a pointer where the register's value will be written into.
//  */
// RadarReturnCode radarGetRegister(RadarHandle* handle, uint32_t address,
//                             uint32_t* value){

//                             };

// /*
//  * @brief Set a register value directly to the sensor.
//  *
//  * @param handle a handler for the radar instance to use.
//  * @param address an address of the register to set.
//  * @param value a new value to set.
//  */
// RadarReturnCode radarSetRegister(RadarHandle* handle, uint32_t address, uint32_t value){

// };


