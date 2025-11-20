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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "CLI_Write.h"

typedef uint32_t DWORD;
/* Helper functions for running the Ripple APIs */
int findWordInSentence(char *word, char *sentence)
{
	int n = 0;
	int m = 0;
	int foundLoc = 0;
	int len = strlen(word); 
	int retVal = -1;

	while (sentence[n] != '\0') {
		/* if first character of search string matches */
		if (sentence[n] == word[m]) {    
			/* keep on searching till end */
			while (sentence[n] == word[m] && sentence[n] != '\0') {
				n++;
				m++;
			}

			/* if we sequence of characters matching with the length of searched string */
			if (m == len)
			{
				/* we find our search string */
				foundLoc++;
				retVal = n - strlen(word) +1;
				break;
			}
		}

		n++;
		/* reset the counter to start from first character of the search string. */
		m = 0;  
	}
    return retVal;
}
 // Just Write to Serial Port Without Checking For Response
int CLI_Just_Write(int fd,char *line){
	/* Write to serial port */	

	// Buffer Containing Rxed Data
	char  SerialBuffer[257] = { 0 };
	DWORD NoBytesRead;
	int i = 0, j = 0, found_result = 100, rdStrLen = 0, retVal = 0;
	// No of bytes to write into the port
	DWORD  dNoOFBytestoWrite = 0;              
    DWORD  dNoOfBytesWritten = 0;
	char cliCmdStr[257] = { 0 };
	DWORD dwBytesRead = 1024, dwErrorFlags;

	/* clear the command string buffer */
	memset(&cliCmdStr[0], 0, sizeof(cliCmdStr));
	retVal=strlen(line);
	strcpy(cliCmdStr,line);
	dNoOFBytestoWrite = strlen(cliCmdStr);
	/* write the CLI CMD over serial port */

	if (!(dNoOfBytesWritten= write(fd, &cliCmdStr[0], dNoOFBytestoWrite))){
		printf("Error writing text to Serial Port\n");
	}
	else{
		printf("[WR]%s", cliCmdStr);
	}
	return 1;
}


int clean_buffer(int fd_i){
	/* Write to serial port */	
	char *line="cleanbuffr 0\n\r";
	// Buffer Containing Rxed Data
	char  SerialBuffer[257] = { 0 };
	DWORD NoBytesRead;
	int i = 0, j = 0, found_result = 100, rdStrLen = 0, retVal = 0,fd_w=fd_i;
	// No of bytes to write into the port
	DWORD  dNoOFBytestoWrite = 0;              
    DWORD  dNoOfBytesWritten = 0;
	char cliCmdStr[257] = { 0 };
	DWORD dwBytesRead = 1024, dwErrorFlags;

	/* clear the command string buffer */
	memset(&cliCmdStr[0], 0, sizeof(cliCmdStr));
	retVal=strlen(line);
	strcpy(cliCmdStr,line);
	dNoOFBytestoWrite = strlen(cliCmdStr);
	/* write the CLI CMD over serial port */

	if (!(dNoOfBytesWritten= write(fd_w, &cliCmdStr[0], dNoOFBytestoWrite))){
		printf("Error writing text to Serial Port\n");
	}

	/* reading the COM port contains the lastly written data as well,
	* we read those length and extra bytes */
	rdStrLen = dNoOFBytestoWrite + 49;
	/* Reset the read buffer */
	memset(&SerialBuffer[0], 0, 257);
	int rdCnt = 0;
	int errorRsp, doneRspLoc, mmwavedemo_text_count;
	char rdCharCnt = 0;
	/* loop till we are able to read response from device */
	while (found_result)
	{
		doneRspLoc = -1;
		errorRsp = -1;
		mmwavedemo_text_count=-1;
		/********* Read COM port data ******************/
		NoBytesRead =0;

		NoBytesRead=read(fd_w, &SerialBuffer[rdCnt],rdStrLen);

		if (NoBytesRead==0)
		{
			printf("wrong character");
			retVal = -1;
			goto EXIT;
		}
		rdCnt += NoBytesRead;
		if (rdCnt > sizeof(SerialBuffer)){
			retVal = -1;
			goto EXIT;
		}
		/* read 10 bytes at a time in next iteration */
		rdStrLen = 10;

		/* Search "Done" or "Error" in the received Text */
		doneRspLoc = findWordInSentence("Done", &SerialBuffer[0]);
		errorRsp = findWordInSentence("Error", &SerialBuffer[0]);
		mmwavedemo_text_count=findWordInSentence("Demo", &SerialBuffer[0]);
		if ((doneRspLoc > 0) || (errorRsp > 0)){
			char errCodeStr[10] = { 0 };
			int  strCnt = 0;
			rdCharCnt = (doneRspLoc > 0) ? (doneRspLoc +strlen("Done")): \
						(errorRsp + strlen("Error"));
			/* read till '\n' after "Done" or "Error" location from COM Port */
			
			while(findWordInSentence("Demo:/>", &SerialBuffer[rdCharCnt-1])<0){
				NoBytesRead=read(fd_w, &SerialBuffer[rdCnt], 16);
				rdCnt += NoBytesRead;	
			}
			/* if error string recieved */
			if (errorRsp > 0){
				strncpy(&errCodeStr[0], &SerialBuffer[rdCharCnt], (rdCnt - rdCharCnt));
				retVal = atoi(errCodeStr);
				/* If this is error then print the error value */
				printf("[ERROR] mmwave device returns [%d]\n", atoi(errCodeStr));
				goto EXIT;
			}

			break;
		}
		if(mmwavedemo_text_count>0)break;
		/* decrement the retry count */
		found_result--;
		/* it is in last retry to read Done/Error msg and not yet got RSP */
		if (found_result == 0)
		{
			retVal = -1;
			printf("[ERROR] mmwave device NO RSP for CLI CMD\n");
			goto EXIT;
		}

	}
	/* set retry count to max defined */
	found_result = 100;
   
EXIT:
	return(1);
}

//Write to Serial Port and Checks and waits for the Response
int CLI_Write_UART(int fd_i,char *line) {
	
	/* Write to serial port */	

	// Buffer Containing Rxed Data
	char  SerialBuffer[257] = { 0 };
	DWORD NoBytesRead;
	int i = 0, j = 0, found_result = 100, rdStrLen = 0, retVal = 0,fd_w=fd_i;
	// No of bytes to write into the port
	DWORD  dNoOFBytestoWrite = 0;              
    DWORD  dNoOfBytesWritten = 0;
	char cliCmdStr[257] = { 0 };
	DWORD dwBytesRead = 1024, dwErrorFlags;

	/* clear the command string buffer */
	memset(&cliCmdStr[0], 0, sizeof(cliCmdStr));
	retVal=strlen(line);
	strcpy(cliCmdStr,line);
	dNoOFBytestoWrite = strlen(cliCmdStr);
	/* write the CLI CMD over serial port */

	if (!(dNoOfBytesWritten= write(fd_w, &cliCmdStr[0], dNoOFBytestoWrite))){
		printf("Error writing text to Serial Port\n");
	}
	else{
		printf("[WR]%s", cliCmdStr);
	}

	/* reading the COM port contains the lastly written data as well,
	* we read those length and extra bytes */
	rdStrLen = dNoOFBytestoWrite + 36;
	/* Reset the read buffer */
	memset(&SerialBuffer[0], 0, 257);
	int rdCnt = 0;
	int errorRsp, doneRspLoc, mmwavedemo_text_count;
	char rdCharCnt = 0;
	/* loop till we are able to read response from device */
	while (found_result)
	{
		doneRspLoc = -1;
		errorRsp = -1;
		mmwavedemo_text_count=-1;
		/********* Read COM port data ******************/
		NoBytesRead =0;

		NoBytesRead=read(fd_w, &SerialBuffer[rdCnt],rdStrLen);

		if (NoBytesRead==0)
		{
			printf("wrong character");
			retVal = -1;
			goto EXIT;
		}
		rdCnt += NoBytesRead;
		if (rdCnt > sizeof(SerialBuffer)){
			retVal = -1;
			goto EXIT;
		}
		/* read 10 bytes at a time in next iteration */
		rdStrLen = 10;
		printf("[RD]%s\n", SerialBuffer);
		


		/* Search "Done" or "Error" in the received Text */
		doneRspLoc = findWordInSentence("Done", &SerialBuffer[0]);
		errorRsp = findWordInSentence("Error", &SerialBuffer[0]);
		mmwavedemo_text_count=findWordInSentence("Demo", &SerialBuffer[0]);
		if ((doneRspLoc > 0) || (errorRsp > 0)){
			char errCodeStr[10] = { 0 };
			int  strCnt = 0;
			rdCharCnt = (doneRspLoc > 0) ? (doneRspLoc +strlen("Done")): \
						(errorRsp + strlen("Error"));
			/* read till '\n' after "Done" or "Error" location from COM Port */
			
			while(findWordInSentence("Demo:/>", &SerialBuffer[rdCharCnt-1])<0){
				NoBytesRead=read(fd_w, &SerialBuffer[rdCnt], 16);
				rdCnt += NoBytesRead;	
			}
			/* if error string recieved */
			if (errorRsp > 0){
				strncpy(&errCodeStr[0], &SerialBuffer[rdCharCnt], (rdCnt - rdCharCnt));
				retVal = atoi(errCodeStr);
				/* If this is error then print the error value */
				printf("[ERROR] mmwave device returns [%d]\n", atoi(errCodeStr));
				goto EXIT;
			}

			break;
		}
		if(mmwavedemo_text_count>0)break;
		/* decrement the retry count */
		found_result--;
		/* it is in last retry to read Done/Error msg and not yet got RSP */
		if (found_result == 0)
		{
			retVal = -1;
			printf("[ERROR] mmwave device NO RSP for CLI CMD\n");
			goto EXIT;
		}

	}
	/* set retry count to max defined */
	found_result = 100;
   
EXIT:
	return(1);
}


int uart_logging_config_baudrate(int fd_B)
{
struct termios2 options2;
ioctl(fd_B, TCGETS2, &options2);
/* Set up serial port */
/*New settings*/
options2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
options2.c_oflag &= ~OPOST;
options2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
options2.c_cflag &= ~(CSIZE | PARENB);
options2.c_cflag |= CS8;
options2.c_cc[VTIME]=10;
options2.c_cc[VMIN]=255;
options2.c_cflag &= ~CBAUD;    //Remove current baud rate
options2.c_cflag |= BOTHER;    //Allow custom baud rate using int input
options2.c_ispeed = 833333;    //Set the input baud rate
options2.c_ospeed = 833333;    //Set the output baud rate
ioctl(fd_B, TCSETS2, &options2);
return fd_B;
}


int uart_A_config_baudrate(int fd_B)
{
struct termios2 options2;
ioctl(fd_B, TCGETS2, &options2);
/* Set up serial port */
/*New settings*/
options2.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
options2.c_oflag &= ~OPOST;
options2.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
options2.c_cflag &= ~(CSIZE | PARENB);
options2.c_cflag |= CS8;
options2.c_cc[VTIME]=10;
options2.c_cc[VMIN]=255;
options2.c_cflag &= ~CBAUD;    //Remove current baud rate
options2.c_cflag |= BOTHER;    //Allow custom baud rate using int input
options2.c_ispeed = 1250000;    //Set the input baud rate
options2.c_ospeed = 1250000;    //Set the output baud rate
ioctl(fd_B, TCSETS2, &options2);
return fd_B;
}
int getClosest(int val1, int val2,
int target)
    {
        if (target - val1 >= val2 - target)
            return val2;    
        else
            return val1;    
    }
	
int findClosestFreq(int arr[4], int target)
    {
        int n = 4;

        if (target <= arr[0])
            return 0;
        if (target >= arr[n - 1])
            return n - 1;
 
        int i = 0, j = n, mid = 0;
        while (i < j)
        {
            mid = (i + j) / 2;
 
            if (arr[mid] == target)
                return mid;
 
            if (target < arr[mid])
            {

                if (mid > 0 && target > arr[mid - 1])
                    if (arr[mid - 1] == getClosest(arr[mid - 1],
                                 arr[mid], target)){return mid-1;};
					if (arr[mid] == getClosest(arr[mid - 1],
                                 arr[mid], target)){return mid;}
                j = mid;            
            }

            else
            {
                if (mid < n-1 && target < arr[mid + 1])
                    if(arr[mid]==getClosest(arr[mid],
                         arr[mid + 1], target)){return mid;  }      
                    if(arr[mid + 1]==getClosest(arr[mid],
                         arr[mid + 1], target)){return mid + 1;  } 
                i = mid + 1; // update i
            }
        }

        return mid;
    }

int rxMaptoRxNum(int n){
 
int a[10],i;
int rxNum=0;    
   
for(i=0;n>0;i++)    
{    
a[i]=n%2;    
n=n/2;    
}
for(i=i-1;i>=0;i--)    
{ 
	rxNum+=a[i];       
}         

return rxNum;  
}  


void string2hexString(uint8_t* input, uint8_t* output,int SizeOfaChirp)
{
    int loop;
    int i; 
    i=0;
    loop=0;    
    while(SizeOfaChirp)
    {   
        // printf("%02x\n",input[loop]);
		sprintf((char*)(output+i),"%02x""%s", input[loop],", ");  
        i+=4;
		SizeOfaChirp--;
		loop+=1;
	}

    output[i++] = '\0';
}

