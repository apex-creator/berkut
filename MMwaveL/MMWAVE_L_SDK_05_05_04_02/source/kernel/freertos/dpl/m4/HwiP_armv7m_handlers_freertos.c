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

#include <kernel/nortos/dpl/m4/HwiP_armv7m.h>
#include <drivers/hw_include/csl_types.h>

void HWI_SECTION HwiP_interrupt_handler()
{
    volatile uint32_t *addr;
    uint32_t activeIntNum;

    addr = ICSR;
    activeIntNum = (*addr & 0xFF);

    if(    (activeIntNum < HwiP_MAX_INTERRUPTS)
        && (activeIntNum >= 15) /* sys tick or external NIVC interrupt */
        && (gHwiCtrl.isr[activeIntNum] != NULL)
        )
    {
        gHwiCtrl.isr[activeIntNum](gHwiCtrl.isrArgs[activeIntNum]);
    }
}

void HWI_SECTION HwiP_nmi_handler()
{
    if(NULL != gHwiCtrl.isr[NMI_INTERRUPT_NUMBER])
    {
        gHwiCtrl.isr[NMI_INTERRUPT_NUMBER](gHwiCtrl.isrArgs[NMI_INTERRUPT_NUMBER]);
    }
    else
    {
        volatile uint32_t loop = 1;
        while(loop);
    }
}

void HWI_SECTION HwiP_hardFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_memFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_busFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_usageFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_reserved_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_debugMon_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void vPortSVCHandler( void );
void xPortPendSVHandler( void );

extern uint32_t __STACK_END;
extern void _c_int00();

uint32_t __attribute__((section(".vectors"), aligned(32))) gHwiP_vectorTable[HwiP_MAX_INTERRUPTS]  = {
    (uint32_t)&__STACK_END,             /* 0 */
    (uint32_t)&_c_int00,                /* 1 */
    (uint32_t)&HwiP_nmi_handler,        /* 2 */
    (uint32_t)&HwiP_hardFault_handler,  /* 3 */
    (uint32_t)&HwiP_memFault_handler,   /* 4 */
    (uint32_t)&HwiP_busFault_handler,   /* 5 */
    (uint32_t)&HwiP_usageFault_handler, /* 6 */
    (uint32_t)&HwiP_reserved_handler,   /* 7 */
    (uint32_t)&HwiP_reserved_handler,   /* 8 */
    (uint32_t)&HwiP_reserved_handler,   /* 9 */
    (uint32_t)&HwiP_reserved_handler,   /* 10 */
    (uint32_t)&vPortSVCHandler,         /* 11 */
    (uint32_t)&HwiP_debugMon_handler,   /* 12 */
    (uint32_t)&HwiP_reserved_handler,   /* 13 */
    (uint32_t)&xPortPendSVHandler,      /* 14 */
    (uint32_t)&HwiP_interrupt_handler,  /* 15 */ /* SysTick */
    /* rest of the handlers are setup in HwiP_init and would be for
     * the 'external' NVIC interrupts
     */
    (uint32_t)&HwiP_reserved_handler,   /* 0 */
    (uint32_t)&HwiP_reserved_handler,   /* 1 */
    (uint32_t)&HwiP_reserved_handler,   /* 2 */
    (uint32_t)&HwiP_reserved_handler,   /* 3 */
    (uint32_t)&HwiP_reserved_handler,   /* 4 */
    (uint32_t)&HwiP_reserved_handler,   /* 5 */
    (uint32_t)&HwiP_reserved_handler,   /* 6 */
    (uint32_t)&HwiP_reserved_handler,   /* 7 */
    (uint32_t)&HwiP_reserved_handler,   /* 8 */
    (uint32_t)&HwiP_reserved_handler,   /* 9 */
    (uint32_t)&HwiP_reserved_handler,   /* 10 */
    (uint32_t)&HwiP_reserved_handler,   /* 11 */
    (uint32_t)&HwiP_reserved_handler,   /* 12 */
    (uint32_t)&HwiP_reserved_handler,   /* 13 */
    (uint32_t)&HwiP_reserved_handler,   /* 14 */
    (uint32_t)&HwiP_reserved_handler,   /* 15 */
    (uint32_t)&HwiP_reserved_handler,   /* 16 */
    (uint32_t)&HwiP_reserved_handler,   /* 17 */
    (uint32_t)&HwiP_reserved_handler,   /* 18 */
    (uint32_t)&HwiP_reserved_handler,   /* 19 */
    (uint32_t)&HwiP_reserved_handler,   /* 20 */
    (uint32_t)&HwiP_reserved_handler,   /* 21 */
    (uint32_t)&HwiP_reserved_handler,   /* 22 */
    (uint32_t)&HwiP_reserved_handler,   /* 23 */
    (uint32_t)&HwiP_reserved_handler,   /* 24 */
    (uint32_t)&HwiP_reserved_handler,   /* 25 */
    (uint32_t)&HwiP_reserved_handler,   /* 26 */
    (uint32_t)&HwiP_reserved_handler,   /* 27 */
    (uint32_t)&HwiP_reserved_handler,   /* 28 */
    (uint32_t)&HwiP_reserved_handler,   /* 29 */
    (uint32_t)&HwiP_reserved_handler,   /* 30 */
    (uint32_t)&HwiP_reserved_handler,   /* 31 */
    (uint32_t)&HwiP_reserved_handler,   /* 32 */
    (uint32_t)&HwiP_reserved_handler,   /* 33 */
    (uint32_t)&HwiP_reserved_handler,   /* 34 */
    (uint32_t)&HwiP_reserved_handler,   /* 35 */
    (uint32_t)&HwiP_reserved_handler,   /* 36 */
    (uint32_t)&HwiP_reserved_handler,   /* 37 */
    (uint32_t)&HwiP_reserved_handler,   /* 38 */
    (uint32_t)&HwiP_reserved_handler,   /* 39 */
    (uint32_t)&HwiP_reserved_handler,   /* 40 */
    (uint32_t)&HwiP_reserved_handler,   /* 41 */
    (uint32_t)&HwiP_reserved_handler,   /* 42 */
    (uint32_t)&HwiP_reserved_handler,   /* 43 */
    (uint32_t)&HwiP_reserved_handler,   /* 44 */
    (uint32_t)&HwiP_reserved_handler,   /* 45 */
    (uint32_t)&HwiP_reserved_handler,   /* 46 */
    (uint32_t)&HwiP_reserved_handler,   /* 47 */
    (uint32_t)&HwiP_reserved_handler,   /* 48 */
    (uint32_t)&HwiP_reserved_handler,   /* 49 */
    (uint32_t)&HwiP_reserved_handler,   /* 50 */
    (uint32_t)&HwiP_reserved_handler,   /* 51 */
    (uint32_t)&HwiP_reserved_handler,   /* 52 */
    (uint32_t)&HwiP_reserved_handler,   /* 53 */
    (uint32_t)&HwiP_reserved_handler,   /* 54 */
    (uint32_t)&HwiP_reserved_handler,   /* 55 */
    (uint32_t)&HwiP_reserved_handler,   /* 56 */
    (uint32_t)&HwiP_reserved_handler,   /* 57 */
    (uint32_t)&HwiP_reserved_handler,   /* 58 */
    (uint32_t)&HwiP_reserved_handler,   /* 59 */
    (uint32_t)&HwiP_reserved_handler,   /* 60 */
    (uint32_t)&HwiP_reserved_handler,   /* 61 */
    (uint32_t)&HwiP_reserved_handler,   /* 62 */
    (uint32_t)&HwiP_reserved_handler    /* 63 */
};
