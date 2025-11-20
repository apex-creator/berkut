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
#ifndef RIPPLE_FLASH_H
#define RIPPLE_FLASH_H
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include <string.h>

#include <board/flash.h>
#include <kernel/dpl/CacheP.h>

#include "ti_board_open_close.h"

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/**
 * @brief   Error Code: Flash Base Error Code
 */
#define MMWAVE_ERRNO_MMWDEMO_FLASH_BASE                                     (-50100)

/**
 * @brief   Error Code: Invalid variable
 */
#define MMWDEMO_FLASH_EINVAL                                                (MMWAVE_ERRNO_MMWDEMO_FLASH_BASE-1)

/**
 * @brief   Error Code: receive error when accessing QSPI interface
 */
#define MMWDEMO_FLASH_EINVAL__QSPI                                          (MMWAVE_ERRNO_MMWDEMO_FLASH_BASE-2)


/**
 * @brief   Error Code: receive error when accessing QSPI Flash
 */
#define MMWDEMO_FLASH_EINVAL__QSPIFLASH                                     (MMWAVE_ERRNO_MMWDEMO_FLASH_BASE-3)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef struct mmwDemo_Flash_t
{

    /*! @brief   QSPI flash driver handle */
    Flash_Handle      QSPIFlashHandle;

    /*! @brief   Module initialized flag */
    bool              initialized;
}mmwDemo_Flash;

int32_t ripple_flashInit(void);
int32_t ripple_flashRead(uint32_t flashOffset, uint8_t *readBuf, uint32_t size);
int32_t ripple_flashWrite(uint32_t flashOffset, uint8_t *writeBuf, uint32_t size);
extern int32_t ripple_flashEraseOneSector(uint32_t flashOffset, uint32_t* blockNum, uint32_t* pageNum);

#ifdef __cplusplus
}
#endif

#endif /* HLASH_H */