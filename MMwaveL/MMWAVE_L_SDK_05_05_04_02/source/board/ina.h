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
/**
 *  \defgroup BOARD_INA_MODULE APIs for INA
 *  \ingroup BOARD_MODULE
 *
 *  This module contains APIs to program and use the INA Sensor.
 *  See \ref BOARD_INA_PAGE for more details.
 *
 *  @{
 */

/* INA228 registers */
#define INA228_CONFIG_REG0                (0x00)
#define INA228_CONFIG_REG1                (0x01)
#define INA228_CURRENT_REG                (0x7)
#define INA228_CALIB_REG                  (0x2)
#define INA228_SHUNT_TEMP_REG             (0x3)
#define INA228_DIE_ID_REG                 (0x3f)
#define INA228_DIE_ID_REG_VALUE           (552U)


#include <drivers/i2c.h>

/**
 * \brief Configure INA228 sensor
 *
 * \param i2cHandle [in] Handle to I2C driver
 * \param i2cTargetAddress [in] INA Sensor address
 * \param config0Val [in] To enable/disable Temperature compensation
 * \param config1Val [in] ADC Mode
 * \param shuntTempVal [in] Shunt temperature coefficient value to compensate for temperature based effects 
 * \param calibVal [in] The shunt calibration is a measure of the current resolution for the measurements and is derived based on Max current measurable by the design
 */

void SensorConfig228(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint32_t config0Val, uint32_t config1Val, uint32_t shuntTempVal, uint32_t calibVal);

/**
 * \brief Read INA228 sensor Current value
 *
 * \param i2cHandle [in] Handle to I2C driver
 * \param i2cTargetAddress [in] INA Sensor address
 * \param currentLsb [in] Current resolution for the measurements
 *
 * \return Current value in mA
 */
float currentRead228(I2C_Handle i2cHandle, uint8_t i2cTargetAddress, uint32_t currentLsb);

/** @} */

