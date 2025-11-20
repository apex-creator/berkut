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
#include "../HAL-Lib/CLI_Write.h"
#include "../HAL-Lib/RippleHAL.h"

int main(){
	int32_t a = 12345678;

	printf("radarCreate(a) Ripple API is being called\n");
	//Create a handle, initialize the UART Port and store the file descriptor in "Handle"
	RadarHandle* handle2 = radarCreate(a);

	printf("radarStopDataStreaming(handle2) Ripple API is being called\n");
	// radarStopDataStreaming will write sensorStop to Cli 
	radarStopDataStreaming(handle2);	

	// setting rx Antenna mask
	radarSetMainParam(handle2,1,10,1);
	printf("radarSetMainParam(handle2,1,10,1); Ripple API is being called\n");	
	
	// setting tx antenna mask
	radarSetMainParam(handle2,1,9,1);
	printf("radarSetMainParam(handle2,1,9,1); Ripple API is being called\n");

 	// set DigOutSampleRate (in Hz)
 	radarSetMainParam(handle2,1,12,12500000);
 	printf("radarSetMainParam(handle2,1,12,12500000); Ripple API is being called\n");
 
	// setNum AdcSamples
	radarSetMainParam(handle2,1,6,128);
	printf("radarSetMainParam(handle2,1,6,128); Ripple API is being called\n");

	// set chirp Period (should be kept more than 150us(vendor specific Idle time) for SPI interface to work)
 	radarSetMainParam(handle2,1,4,186);
 	printf("radarSetMainParam(handle2,1,4,2986); Ripple API is being called\n");
 	
	// set up startFreq
	radarSetMainParam(handle2,1,7,60000);
	printf("radarSetMainParam(handle2,1,7,60000); Ripple API is being Called\n");

 	// set Rx Gain
 	radarSetRxParam(handle2,1,1,1,40);
 	printf("radarSetRxParam(handle2,1,1,1,40); Ripple API is being Called\n");

	// set up EndFreq
	radarSetMainParam(handle2,1,8,64000);
	printf("radarSetMainParam(handle2,1,8,64000); Ripple API is being Called\n");

	// set up reQuired TxOutPower ((in dB )  (antenna mask 1)
 	radarSetTxParam(handle2,1,1,1,26); 
 	printf("radarSetTxParam(handle2,1,1,1,26);  Ripple API is being Called\n");
	
	// set up CHIRPS PER BURST(NUM of Loops)(1,32) 
	radarSetMainParam(handle2,1,5,32);
	printf("radarSetMainParam(handle2,1,5,32); Ripple API is being Called\n");
	
	/*Setting After Burst Power Mode*/
	radarSetMainParam(handle2,1,1,0);

	// set BURST PERIODICITY
	radarSetMainParam(handle2,1,3,250);
	printf("radarSetMainParam(handle2,1,3,250); Ripple API is being called\n");
	
	//number of Bursts
	radarSetVendorParam(handle2,1,2,40);
	printf("radarSetVendorParam(handle2,1,2,40); Ripple API is being called\n");

	printf("radarActivateConfig(handle2,1); Ripple API is being called\n");
	//Activating the above configs
	radarActivateConfig(handle2,1);

	UserData Data;
	Data.handle=handle2;
	void *userData = &Data;
	FILE *dumpfPtr;
	/*Open File and write to it, it will also create file if it doesn't exist and write contents*/
	dumpfPtr = fopen("rawADCHex.txt", "w");	
	uint32_t Numofbursts,rxAntennaMask,NumberofADCSamples,chirpsperburst;
	uint32_t *ptrNumofbursts=&Numofbursts;
	uint32_t *ptrxAntennaMask=&rxAntennaMask;
	uint32_t *ptrNumberofADCSamples=&NumberofADCSamples;
	uint32_t *ptrchirpsperburst=&chirpsperburst;
	uint32_t burstsize,i=0,printCnt=0;	

	radarGetVendorParam(handle2,1,2,ptrNumofbursts);
	radarGetMainParam(handle2,1,10,ptrxAntennaMask);
	radarGetMainParam(handle2,1,6,ptrNumberofADCSamples);
	radarGetMainParam(handle2,1,5,ptrchirpsperburst);
	uint32_t NumberOfBursts, NumberOfBurstsToPrint =*ptrNumofbursts;
	burstsize=rxMaptoRxNum(*ptrxAntennaMask) * (2) * (*ptrNumberofADCSamples) * (*ptrchirpsperburst);

	printf("radarStartDataStreaming(handle2) Ripple API is being called ");
	// printf("and radarSetBurstReadyCb(handle2,OnBurstReady,userData) callback is being set\n");	
	
		
	//start the sensor
	radarStartDataStreaming(handle2);
	printf("\r\nSensor Started");
	
	
	/*while(NumberOfBurstsToPrint){		
		printCnt=0;		//Set the OnBurstReady CallBack		
		radarSetBurstReadyCb(handle2,OnBurstReady,userData);				
		for(i=0;i<burstsize;i++){				
			fprintf(dumpfPtr,"%02x,",*((((UserData *)userData)->read_buffer)+printCnt));				
			printCnt+=1;}		
		NumberOfBurstsToPrint--;
	}

 printf("Chirps per burst=%d",((UserData *)userData)->format.chirps_per_burst);
 printf(", Bits per Sample=%d",((UserData *)userData)->format.bits_per_sample);
 printf(", Samples per Chirp=%d",((UserData *)userData)->format.samples_per_chirp);
 printf(", Channels Count=%d\n",((UserData *)userData)->format.channels_count);*/
 
}
