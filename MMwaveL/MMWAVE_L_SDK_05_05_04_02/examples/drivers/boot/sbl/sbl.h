/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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

/** @defgroup SBL      SBL
 */

#ifndef SBL_H_
#define SBL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "transport.h"

#define     MINUS_ONE           -((int32_t)1)

#define MMWAVE_ERRNO_SBL_BASE                (-4400)
/**
@defgroup SBL_ERROR_CODE            SBL Error code
@ingroup SBL
@brief
*   The section has a list of all the error codes which are returned to the application.
*   Base error code for the SBL module is defined in the
*  \include ti/common/mmwave_error.h
 @{ */

/**
 * @brief   No Error
 */
#define SBL_EOK                         (0)

/**
 * @brief   Error Code: Invalid argument
 */
#define SBL_EINVAL                      (MMWAVE_ERRNO_SBL_BASE-1)

/**
 * @brief   Error Code: Too many synchronization errors.
 */
#define SBL_ESYNC                       (MMWAVE_ERRNO_SBL_BASE-2)

/**
 * @brief   Error Code: Too many unsuccessful retransmissions.
 */
#define SBL_ERETRANS                    (MMWAVE_ERRNO_SBL_BASE-3)

/**
 * @brief   Error Code: Cancelled by remote peer.
 */
#define SBL_ECANCEL                     (MMWAVE_ERRNO_SBL_BASE-4)

/**
 * @brief   Error Code: Out of memory
 */
#define SBL_ENOMEM                      (MMWAVE_ERRNO_SBL_BASE-5)

/**
 * @brief   Error Code: Exceeds maximum file size
 */
#define SBL_EMAXMEM                     (MMWAVE_ERRNO_SBL_BASE-6)

/** @}*/

/**
@defgroup SBL_CONFIG_PARAMS            SBL Configurable parameters
@ingroup SBL
@brief
*   The section has a list of configurable defines
 @{ */

/**
 * @brief   Number of seconds the SBL will wait before autobooting the existing image.
 *          This allows the user to press a key to stop the autoboot and flash a new image.
 */
#define SBL_AUTOBOOT_COUNT                  10U

/**
 * @brief   ASCII code of CR and Space key. Press CR or Space to stop autoboot.
 */
#define SBL_CR_KEY                          13U
#define SBL_SPACE_KEY                       32U

/**
 * @brief   This is the maximum size(1MB) of the application metaimage which the SBL will attempt to flash.
 *          This define protects against erraneously overwriting the backup default image.
 */
#define SBL_MAX_METAIMAGE_SIZE              (512U * 1024U)

/**
 * @brief   This is the backup default image offset.
 * Note: SBL only reads from this location. It does not modify the factory default backup image.
 */
#define SBL_BACKUP_IMAGE_OFFSET             0x00180000U
/** @}*/

/* SBL image Location in flash */
#define M_META_SBL       (uint32_t)(0x00000U)

/* Image Flashed by SBL: 64KB is reserved */
#define M_META_IMAGE       ((uint32_t)M_META_SBL + (uint32_t)(0x80000U))

/* Factory Backup Image */
#define M_META_IMAGE_BACKUP       ((uint32_t)M_META_SBL + (uint32_t)(0x180000U))

typedef struct
{
    uint64_t errorStatus;

    SBL_Transport trans;

    uint8_t fecEclipseMode;

    uint8_t fecShRamPresent;

    /* Request Loading... */
    uint8_t metaimageUpdate;
}sbl_Obj_t;

extern sbl_Obj_t sblObj;

void sbl_computeCrc32Cpu(uint32_t* data, uint32_t numbytes, uint32_t* crc32);
void sbl_computeCrc16Cpu(uint16_t* data, uint32_t numbytes, uint16_t* crc16);
void sbl_configureCrc32Cpu();
void sbl_sendDataCrc32Cpu(uint32_t* data, uint32_t numbytes);
uint32_t sbl_getCrc32Cpu();
void sbl_switchtoapp() __attribute__ ((section (".jumptoApp")));

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SBL_H_ */

