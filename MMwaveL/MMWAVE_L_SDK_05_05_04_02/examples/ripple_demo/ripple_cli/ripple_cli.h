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

#ifndef RIPPLE_CLI_H
#define RIPPLE_CLI_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/uart.h>
#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* These defines need to move to common files. */
#define MMWAVE_SDK_VERSION_BUILD  2
#define MMWAVE_SDK_VERSION_BUGFIX 4
#define MMWAVE_SDK_VERSION_MINOR  5
#define MMWAVE_SDK_VERSION_MAJOR  5

#define CLI_MAX_CMD          32
#define READ_LINE_BUFSIZE    256
#define CLI_MAX_ARGS         40
#define CLI_TASK_STACK_SIZE  (4 *1024U)

typedef void*   CLI_Handle;
/*typedef for cmd Handler Functions*/
typedef int32_t (*CLI_CmdHandler)(int32_t argc, char* argv[]);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct CLI_CmdTableEntry_t
{
    /**
     * @brief   Command string
     */
    char*               cmd;

    /**
     * @brief   CLI Command Help string
     */
    char*               helpString;

    /**
     * @brief   Command Handler to be executed
     */
    CLI_CmdHandler      cmdHandlerFxn;
}CLI_CmdTableEntry;


typedef struct CLI_Cfg_t
{
    /**
     * @brief   CLI Prompt string (if any to be displayed)
     */
    char*               cliPrompt;

    /**
     * @brief   UART Handle used by the CLI
     */
    UART_Handle         UartHandle;

    /**
     * @brief   Task Priority: The CLI executes in the context of a task
     * which executes with this priority
     */
    uint8_t             taskPriority;
     /**
     * @brief   This is the table which specifies the supported CLI commands
     */
    CLI_CmdTableEntry   tableEntry[CLI_MAX_CMD];
}CLI_Cfg;

typedef struct CLI_MCB_t
{
    /**
     * @brief   Configuration which was used to configure the CLI module
     */
    CLI_Cfg         cfg;

    /**
     * @brief   This is the number of CLI commands which have been added to the module
     */
    uint32_t        numCLICommands;

    /**
     * @brief   CLI Task Handle:
     */
    TaskHandle_t     cliTaskHandle;
}CLI_MCB;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t CLI_open (CLI_Cfg* ptrCLICfg);
void    CLI_write (const char* format, ...);
void    ripple_cli_init (uint8_t taskPriority);
int32_t CLI_MMWStart(void);
int32_t CLI_MMWaveSensorStart (int32_t argc, char* argv[]);

#ifdef __cplusplus
}
#endif

#endif /* CLI_H */