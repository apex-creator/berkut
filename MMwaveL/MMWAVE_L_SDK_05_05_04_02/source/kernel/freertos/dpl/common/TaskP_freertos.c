/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <kernel/dpl/TaskP.h>
#include <FreeRTOS.h>
#include <task.h>

#define TaskP_STACK_SIZE_MIN    (128U)

typedef struct TaskP_Struct_ {
    StaticTask_t taskObj;
    TaskHandle_t taskHndl;
} TaskP_Struct;

void TaskP_Params_init(TaskP_Params *params)
{
    params->name = "Task (DPL)";
    params->stackSize = 0;
    params->stack = NULL;
    params->priority = (TaskP_PRIORITY_HIGHEST - TaskP_PRIORITY_LOWEST) / 2;
    params->args = NULL;
    params->taskMain = NULL;
}

int32_t TaskP_construct(TaskP_Object *obj, TaskP_Params *params)
{
    int32_t status = SystemP_SUCCESS;
    TaskP_Struct *taskObj = (TaskP_Struct *)obj;

    DebugP_assert(sizeof(TaskP_Struct) <= sizeof(TaskP_Object));
    DebugP_assert(params != NULL);
    DebugP_assert(taskObj != NULL);
    DebugP_assert(params->stackSize >= TaskP_STACK_SIZE_MIN);
    DebugP_assert( (params->stackSize & (sizeof(configSTACK_DEPTH_TYPE) - 1)) == 0);
    DebugP_assert(params->stack != NULL );
    DebugP_assert( ((uintptr_t)params->stack & (sizeof(configSTACK_DEPTH_TYPE) - 1)) == 0);
    DebugP_assert(params->taskMain != NULL );

    /* if prority is out of range, adjust to bring it in range */
    if(params->priority > TaskP_PRIORITY_HIGHEST)
    {
        params->priority = TaskP_PRIORITY_HIGHEST;
    }
    if(params->priority <= TaskP_PRIORITY_LOWEST)
    {
        params->priority = TaskP_PRIORITY_LOWEST;
    }
    taskObj->taskHndl = xTaskCreateStatic( params->taskMain, /* Pointer to the function that implements the task. */
                                  params->name,              /* Text name for the task.  This is to facilitate debugging only. */
                                  params->stackSize/(sizeof(configSTACK_DEPTH_TYPE)),  /* Stack depth in units of StackType_t typically uint32_t on 32b CPUs */
                                  params->args,       /* task specific args */
                                  params->priority,   /* task priority, 0 is lowest priority, configMAX_PRIORITIES-1 is highest */
                                  (StackType_t*)params->stack,      /* pointer to stack base */
                                  &taskObj->taskObj); /* pointer to statically allocated task object memory */
    if(taskObj->taskHndl == NULL)
    {
        status = SystemP_FAILURE;
    }
    return status;
}

void TaskP_destruct(TaskP_Object *obj)
{
    TaskP_Struct *taskObj = (TaskP_Struct *)obj;

    if(taskObj && taskObj->taskHndl)
    {
        vTaskDelete(taskObj->taskHndl);
        taskObj->taskHndl = NULL;
    }
}

void TaskP_yield()
{
    taskYIELD();
}

void TaskP_exit()
{
    vTaskDelete(NULL);
}
