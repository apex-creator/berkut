/*
 *  Copyright (C) 2022-23 Texas Instruments Incorporated
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

/*
 *   This example configures the LIN module in LIN mode for The LIN module
 *   performs sends a data over LIN_1 which is connected to PC via PLIN_USB.
 *   The data is sent over LIN at 19200 baud rate.
 *
 *
 *   External Connections :
 *    - PLIN-USB connected to Windows Machine.
 *
 *   Watch Variables :
 *
 */

/* Included Files */
#include <kernel/dpl/DebugP.h>
#include <drivers/lin.h>
#include <ti_drivers_config.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "string.h"

/* Defines */
#define EXAMPLE_NAME        "[ LIN RESPONDER ] "
#define FRAME_LENGTH        (0x8)
#define LIN_ID              (0x10)

#define LIN_PASS            (0xABCD)
#define LIN_FAIL            (0xFFFF)
#define RECEIVED_DATA_LENGTH    8U
#define INVALID_ID          0xFFU
#define SLEEP_FRAME_PID     0x3C

#define APP_LIN_BASE_ADDR   (CONFIG_LIN1_BASE_ADDR)
#define APP_LIN_INTR_NUM_0  (CONFIG_LIN1_INTR_NUM_0)
#define APP_LIN_INTR_NUM_1  (CONFIG_LIN1_INTR_NUM_1)

typedef enum{
    LIN_RESPONSE_RX,
    LIN_RESPONSE_TX,
    LIN_RESPONSE_IGNORE,
}linResponseType_t;

typedef struct{
    uint16_t ID;
    linResponseType_t responseType;
    uint8_t datalength;
    uint16_t transmitData[8];
}linFrameProcessStruct_s;

linFrameProcessStruct_s linFrameDatabase[] = {
/*    ID    Response Type     Length    Response Frame (if Any) */
    { 0x23, LIN_RESPONSE_TX,    4,      { 0x12, 0x34, 0x56, 0x78 }                  },
    { 0x30, LIN_RESPONSE_TX,    8,      { 0x30, 0xa4, 0xd3, 0xcc, 0xf4, 0x67, 0xff, 0x7D }},
    { 0x15, LIN_RESPONSE_TX,    6,      { 0x15, 0x76, 0x98, 0x45, 0xfa, 0xFE }            },
    { SLEEP_FRAME_PID, LIN_RESPONSE_RX,    8       },
    { 0x1A, LIN_RESPONSE_RX,    3       },
    { 0x1E, LIN_RESPONSE_RX,    8       },
    { 0x2F, LIN_RESPONSE_RX,    5       },
    { 0x12, LIN_RESPONSE_RX,    2       },
};      

uint16_t lin_sleepFrame[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* Globals */
uint16_t result;
uint16_t txIndex;
uint16_t rxData[8] = {0x00};

static HwiP_Object gLinHwiObject_0, gLinHwiObject_1;
volatile uint32_t vectorOffset = 0;
volatile uint32_t vectorOffsetErr = 0;
volatile uint32_t linErrorFlag = 0;
volatile uint8_t txDoneID = INVALID_ID;
volatile uint8_t rxReceivedID = INVALID_ID;
volatile uint8_t ignoreID = INVALID_ID;
volatile uint8_t wakeupInterruptFlag = 0;
volatile uint8_t sentFrameLength = 0;
uint8_t exitFlag = 0;

static void level0ISR(void * args);
static void level1ISR(void * args);

/* lin_external_responder_interrupts_main */
void lin_external_responder_interrupts_main(void)
{
    uint16_t                error = 0;
    HwiP_Params             hwiPrms;
    int32_t                 status = SystemP_SUCCESS;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* Clear screen command for serial terminal*/
    DebugP_log("\x1b[2J\x1b[;H\r\n");
    
    DebugP_log(EXAMPLE_NAME"LIN External Responder Interrupts, application started ...\r\n");
    DebugP_log(EXAMPLE_NAME"\r\n");  

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_LIN_INTR_NUM_0;
    hwiPrms.callback    = &level0ISR;
    hwiPrms.priority    = 5;
    status              = HwiP_construct(&gLinHwiObject_0, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum      = APP_LIN_INTR_NUM_1;
    hwiPrms.callback    = &level1ISR;
    hwiPrms.priority    = 6;
    status              = HwiP_construct(&gLinHwiObject_1, &hwiPrms);
    DebugP_assert(status == SystemP_SUCCESS);

    /* Enter Software Reset State */
    LIN_enterSoftwareReset(APP_LIN_BASE_ADDR);

    /* Enable multi-buffer mode */
    LIN_enableMultibufferMode(APP_LIN_BASE_ADDR);

    /* Enable Fixed baud rate mode */
    LIN_disableAutomaticBaudrate(APP_LIN_BASE_ADDR);

    /* Reaching the Baud of 19200 */
    LIN_setBaudRatePrescaler(APP_LIN_BASE_ADDR, 130U, 0U);

    /* Enable the triggering of checksum compare on extended frames */
    LIN_triggerChecksumCompare(APP_LIN_BASE_ADDR);
    
    /* Finally exit SW reset and enter LIN ready state */
    LIN_exitSoftwareReset(APP_LIN_BASE_ADDR);

    while(!exitFlag)
    {
        /* Check if new data has been received */
        if(rxReceivedID != INVALID_ID)
        {
            DebugP_log(EXAMPLE_NAME"New Frame Received 0x%02X(ID) ",rxReceivedID); 

            /* Read the received data in the receive buffers */
            LIN_getData(APP_LIN_BASE_ADDR, rxData);
          
            for (uint8_t dataIndex=0; dataIndex < sentFrameLength; dataIndex++)
            {
                DebugP_log("%02X ",rxData[dataIndex]);
            }   
            
            /* Check if Sleep Frame ID was received */
            if(SLEEP_FRAME_PID == rxReceivedID)      
            {
                /* Check if sleep frame was received */
                if(!memcmp((uint8_t*)rxData, (uint8_t*)lin_sleepFrame, 8))
                {
                    DebugP_log("\r\n"EXAMPLE_NAME"Sleep Frame Received. Going to sleep ...");
                    /* Set Lin to powerdown or sleep mode */
                    LIN_enterSleep(APP_LIN_BASE_ADDR);
                }
            }
            DebugP_log("\r\n"EXAMPLE_NAME"\r\n");

            if(rxReceivedID == 0x12)
            {
                exitFlag = 1;
            }

            rxReceivedID = INVALID_ID;   
        }

        /* Check if the response has been transmitted for the requested ID*/
        if(txDoneID != INVALID_ID)
        {
            DebugP_log(EXAMPLE_NAME"Request 0x%02Xh(ID) response frame sent ",txDoneID);
            for (uint8_t dataIndex=0; dataIndex < sentFrameLength; dataIndex++)
            {
                DebugP_log("%02X ",linFrameDatabase[txIndex].transmitData[dataIndex]);
            }
            DebugP_log("\r\n"EXAMPLE_NAME"\r\n");  
            txDoneID = INVALID_ID;
        }

        /* Check if an unknown id is received */
        if(ignoreID != INVALID_ID)
        {
            DebugP_log(EXAMPLE_NAME"Unknown ID Received 0x%02Xh \r\n",ignoreID);
            DebugP_log(EXAMPLE_NAME"\n"); 
            break;
        }

        /* Check if wakeup has occurred */
        if(wakeupInterruptFlag)
        {
            wakeupInterruptFlag = 0;
            DebugP_log(EXAMPLE_NAME"Wakeup detected. Lin now active.\r\n"); 
        }
        
        /* Check if any errors were triggered */
        if(linErrorFlag)
        {            
            DebugP_log(EXAMPLE_NAME"ERROR : ");
            switch(linErrorFlag)
            {
                case LIN_VECT_ISFE :
                    DebugP_log("Inconsistent Sync Field error\r\n");
                break;
                case LIN_VECT_PBE :
                    DebugP_log("Physical Bus error\r\n");
                break;
                case LIN_VECT_PE :
                    DebugP_log("Parity error\r\n");
                break;
                case LIN_VECT_FE :
                    DebugP_log("Framing error\r\n");
                break;
                case LIN_VECT_CE :
                    DebugP_log("Checksum error\r\n");
                break;
                case LIN_VECT_OE :
                    DebugP_log("Overrun error\r\n");
                break;
                case LIN_VECT_BE :
                    DebugP_log("Bit error\r\n");
                break;
                case LIN_VECT_NRE :
                    DebugP_log("No Response error\r\n");
                break;
                default: 
                    DebugP_log("Unknown error %X\r\n",linErrorFlag);
                break;
            }
            linErrorFlag = 0;
            error++;
        }
    }

    /* Check if any data errors occurred */
    if(error == 0)
    {
        result = LIN_PASS;
        DebugP_log(EXAMPLE_NAME"All tests have PASSED!!\r\n");
    }
    else
    {
        result = LIN_FAIL;
        DebugP_log(EXAMPLE_NAME"Test FAILED!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
}

/* 
* lin_process_frames makes the decision as to what is to be done with the received 
* ID, i.e. if we need to respond or receive or ignore the particular id that is
* received.
*/
void lin_process_frames(uint8_t receivedPID)
{ 
    /* Convert PID to ID */
    uint8_t receivedID = receivedPID & 0x3F;

    for(uint8_t i = 0; i < (sizeof(linFrameDatabase)/sizeof(linFrameProcessStruct_s)); i++)
    {
        /* Check if the Received ID is present in our database */
        if(linFrameDatabase[i].ID == receivedID)
        {
            /* Check if a response is to be sent back to Commander */
            if(linFrameDatabase[i].responseType == LIN_RESPONSE_TX)
            {
                /* Set the number of bytes of data to be sent as response */
                sentFrameLength = linFrameDatabase[i].datalength;
                LIN_setFrameLength(APP_LIN_BASE_ADDR, sentFrameLength);
                /* Set the buffer from where the data is to be sent (of length sentFrameLength)*/
                LIN_sendData(APP_LIN_BASE_ADDR, (uint16_t*)linFrameDatabase[i].transmitData);
                txDoneID = receivedID;
                txIndex = i;
                break;
            }               
            /* Check if only a reception of data without response is needed */     
            else if(linFrameDatabase[i].responseType == LIN_RESPONSE_RX)
            {
                /* Set the number of bytes of data to be sent as response */
                sentFrameLength = linFrameDatabase[i].datalength;
                LIN_setFrameLength(APP_LIN_BASE_ADDR, sentFrameLength);
                rxReceivedID = receivedID;
                break;
            }
        }
    }
    /* Ignore the ID if not present in databse */
    if((txDoneID == INVALID_ID) && (rxReceivedID == INVALID_ID))
    {
        ignoreID = receivedID;
    }
}

/*
* level0ISR is responsible for getting the ID interrupt, and determine response
* transmission if necessary.
*/
static void level0ISR(void * args)
{    
    do
    {
        /* Read the high priority interrupt vector */
        vectorOffset = LIN_getInterruptLine0Offset(APP_LIN_BASE_ADDR);

        switch(vectorOffset)
        {
            case LIN_VECT_ID:
                /* Fetch the received ID and process the same */
                lin_process_frames( LIN_getRxIdentifier(APP_LIN_BASE_ADDR) );
            break;
            case LIN_VECT_TX:
                /* Triggered on succcessful transmission of response */
                LIN_disableDataTransmitter(APP_LIN_BASE_ADDR);
                LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_INT_TX);        
                LIN_enableDataTransmitter(APP_LIN_BASE_ADDR);
            break;
            case LIN_VECT_RX:
                /* Triggered on succcessful reception of data */
                LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_INT_RX);
            break;
            case LIN_VECT_WAKEUP:
                /* Triggered on wakeup from sleep due to a wakeup frame received */
                LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_WAKEUP);
                wakeupInterruptFlag = 1;
            default:
            break;
        }  
    }
    while(vectorOffset != LIN_VECT_NONE); 
}

/*
* level1ISR is used for handling any Errors that are triggered during 
* transmission or reception.
*/
static void level1ISR(void * args)
{
    vectorOffsetErr = LIN_getInterruptLine1Offset(APP_LIN_BASE_ADDR);
    if(vectorOffsetErr == LIN_VECT_NRE)
    {
        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_NRE);
        /* Ignoring a frame results in NRE error because the responder 
        does not respond with a message*/
        if(ignoreID != INVALID_ID)
        {
            ignoreID = INVALID_ID;
            vectorOffsetErr = 0;
            /* Trigger a software reset to reset the 
            state of the LIN instance */
            LIN_performSoftwareReset(APP_LIN_BASE_ADDR);
        }
    }
    else if(vectorOffsetErr == LIN_VECT_CE)
    {
        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_CE);
    }
    else if(vectorOffsetErr == LIN_VECT_FE)
    {
        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_FE);
    }
    else if(vectorOffsetErr == LIN_VECT_OE)
    {
        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_OE);
    }
    else if(vectorOffsetErr == LIN_VECT_BE)
    {
        LIN_clearInterruptStatus(APP_LIN_BASE_ADDR, LIN_VECT_BE);
    }
    linErrorFlag = vectorOffsetErr; 
}
