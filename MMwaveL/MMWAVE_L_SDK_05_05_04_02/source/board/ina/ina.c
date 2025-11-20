/*
 *  Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/ 
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
#include <drivers/i2c.h>
#include <board/ina.h>

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

uint8_t                 rxData[16];
uint8_t                 txData[16];
I2C_Transaction         i2cTransaction;

static int32_t i2cReadAddress(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint8_t i2cTargetRegAddress);
static int32_t i2cWriteAddress(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint8_t i2cTargetRegAddress, uint16_t value);

int32_t i2cReadAddress(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint8_t i2cTargetRegAddress)
{
    uint32_t        status;
    int32_t         errCode = 0;

    /* Reset the transmit and receive buffer */
    memset(&rxData, 0, sizeof (rxData));

    /* Scan for the slave address */
    txData[0] = i2cTargetRegAddress;
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.targetAddress = i2cTargetAddress;
    i2cTransaction.writeBuf = txData;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxData;
    i2cTransaction.readCount = 0;

    /* Writing to slave address */
    status = I2C_transfer(i2cHandle, &i2cTransaction);

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Failed to read!\r\n");
        errCode = -1;
    }
    else
    {
        /* Read from slave */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.targetAddress = i2cTargetAddress;
        i2cTransaction.writeBuf = txData;
        i2cTransaction.writeCount = 0;
        i2cTransaction.readBuf = rxData;
        i2cTransaction.readCount = 4;

        status = I2C_transfer(i2cHandle, &i2cTransaction);
        if(status != SystemP_SUCCESS)
        {
            DebugP_log("Failed to read!\r\n");
            errCode = -1;
        }
    }
    return errCode;
}

static int32_t i2cWriteAddress(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint8_t i2cTargetRegAddress, uint16_t value)
{
    uint32_t        status;
    int32_t         errCode = 0;

    /* Reset the transmit and receive buffer */
    memset(&rxData, 0, sizeof (rxData));
    memset(&txData, 0, sizeof (txData));

    /* Scan for the slave address */
    txData[0] = i2cTargetRegAddress;
    txData[2] = (uint8_t)value;
    txData[1] = (uint8_t)(value >> 8);
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.targetAddress = i2cTargetAddress;
    i2cTransaction.writeBuf = txData;
    i2cTransaction.writeCount = 3;
    i2cTransaction.readBuf = rxData;
    i2cTransaction.readCount = 0;

    /* Writing to slave address */
    status = I2C_transfer(i2cHandle, &i2cTransaction);
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Failed to read!\r\n");
        errCode = -1;
    }

    return errCode;
}

void SensorConfig228(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint32_t config0Val, uint32_t config1Val, uint32_t shuntTempVal, uint32_t calibVal)
{
    uint8_t         i2cTargetRegAddress;
    int32_t         retVal = 0;
    uint16_t        dieID = 0;

    i2cTargetRegAddress = INA228_DIE_ID_REG;
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    dieID = rxData[0]<<8 | rxData[1];
    dieID = dieID >> 4;

    if(dieID == INA228_DIE_ID_REG_VALUE)
    {

    i2cTargetRegAddress = INA228_CONFIG_REG0;
    retVal = i2cWriteAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress, config0Val);
    DebugP_assert(retVal == 0);
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    DebugP_assert(retVal == 0);

    i2cTargetRegAddress = INA228_CONFIG_REG1;
    retVal = i2cWriteAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress, config1Val);
    DebugP_assert(retVal == 0);
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    DebugP_assert(retVal == 0);

    // Configure the INA228 shunt temperature register for temp compensation
    i2cTargetRegAddress = INA228_SHUNT_TEMP_REG;
    retVal = i2cWriteAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress, shuntTempVal);
    DebugP_assert(retVal == 0);
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    DebugP_assert(retVal == 0);

    // Calibrate the INA228 - LSB is 1uA/bit for 40mohm and 2uA/bit for 20mohm (1.2_RF rail)
    i2cTargetRegAddress = INA228_CALIB_REG;
    retVal = i2cWriteAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress, calibVal);
    DebugP_assert(retVal == 0);
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    DebugP_assert(retVal == 0);
    }
}

float currentRead228(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint32_t currentLsb)
{
    uint8_t i2cTargetRegAddress;
    int32_t retVal = 0;
    float iCurrent = (float)0xFFFFFFFF;
    uint16_t dieID = 0;

    i2cTargetRegAddress = INA228_DIE_ID_REG;
    retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
    dieID = rxData[0]<<8 | rxData[1];
    dieID = dieID >> 4;

    struct {
        int32_t iCurrentReg : 20;
        int32_t reserved : 12;
    }int20_t_;

    if(dieID == INA228_DIE_ID_REG_VALUE)
    {
        int20_t_.iCurrentReg = 0;
        // Read current register
        i2cTargetRegAddress = INA228_CURRENT_REG;   //current reg
        retVal = i2cReadAddress(i2cHandle, i2cTargetAddress, i2cTargetRegAddress);
        DebugP_assert(retVal == 0);

        int20_t_.iCurrentReg =  ((rxData[0]<<12) | (rxData[1]<<4) | (rxData[2]>>4));
        iCurrent = int20_t_.iCurrentReg * currentLsb * pow(10,-6); //in Ampere
    }
    return(iCurrent < 0 ? -iCurrent : iCurrent);

}