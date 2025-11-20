/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/* This example demonstrates the usage of shared memory available. 512KB of shared RAM is available in the following configurations.
*  256KB between HWASS and APPSS, 96KB between HWASS and FECSS , 160KB dedicated to HWASS
*  This example shows the usage of 256KB HWASS/APPSS Shared memory as APPSS memory and usage of 160KB HWASS Memory
*  in APPSS software (helpful in cases when complete 160KB memory is not used for HWA). 
*
*  256 KB APPSS/HWASS Shared Memory is accessible from 0x00480000 - 0x004BFFFF when configured as APPSS Memory
*
* Important Note: When shared memory is allocated to M4/M3, back to back full word write access to a location followed by sub-word write access 
* to another location corrupts data in previously written location. Refer to Errata Dig#14 "Corrupted Data Store for Partial Write in Shared Memory" in Errata document.
* Workarounds: 
* 1. Disable ECC for non functional safety devices – ECC is disabled for shared memories
*    in RBL for non functional safety devices.
* 2. Use shared memories as code memory "only" when shared with M3/M4 processor.
* Though this examples demonstrates the use of shared memory for data section etc. precautions mentioned above must be taken care of.
*/

#include <stdio.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/soc.h>


#define ARRAY_SIZE              (204800)    //200 KB

/* Test Variable allocation to HWASS/APPSS Shared Memory (.data section)*/
uint8_t test_var1 = 1; 
uint8_t test_var2 = 2;
/* Test Variable for forcing allocation to HWASS and FECSS Shared Memory*/
__attribute__((section(".l3"))) uint8_t testhwameml3[ARRAY_SIZE];  

void shared_memory()
{
    Drivers_open();
    Board_driversOpen();
    /* 160KB HWASS Memory and 96KB FECSS Shared RAM has to be initialized before using it as RBL does not initialize it */
   SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT|SOC_RCM_MEMINIT_FECSS_SHRAM_INIT);


    /* Updating values of test variables stored in HWASS Shared RAM and FECSS Shared RAM (.l3 section)*/ 

   for(int i=0; i<ARRAY_SIZE;i++)
   {
        testhwameml3[i]=0xFF;
   }

    DebugP_log("************* Accessing Data Section stored in APPSS SHARED RAM *************\r\n\r\n");
    /* Updating values of test variables stored in APPSS Shared RAM */
    DebugP_log("Initial value of test_var1 (APPSS Shared RAM) is %d \r\n",test_var1);
    DebugP_log("updating value of test_var1 (APPSS Shared RAM) to 10 \r\n");
    test_var1=10;
    DebugP_log("value of test_var1 (APPSS Shared RAM) is %d \r\n\r\n",test_var1);

    DebugP_log("Initial value of test_var2 (APPSS Shared RAM) is %d \r\n",test_var2);
    DebugP_log("updating value of test_var2 (APPSS Shared RAM) to 20 \r\n");
    test_var2=20;
    DebugP_log("value of test_var2 (APPSS Shared RAM) is %d \r\n\r\n",test_var2);

    DebugP_log("************* Accessing Variables stored in HWASS SHARED RAM *************\r\n\r\n");
    DebugP_log("value of testhwameml3[0] (HWA) is %d \r\n\r\n",testhwameml3[0]);
    DebugP_log("value of testhwameml3[1] (HWA)is %d \r\n\r\n",testhwameml3[1]);

    DebugP_log("*************Accessing Variables stored in FECSS SHARED RAM *************\r\n\r\n");
    DebugP_log("value of testhwameml3[163900] (FECSS) is %d \r\n\r\n",testhwameml3[163900]);
    DebugP_log("value of testhwameml3[163901] (FECSS)is %d \r\n\r\n",testhwameml3[163901]);
    
    Board_driversClose();
    Drivers_close();
    
}