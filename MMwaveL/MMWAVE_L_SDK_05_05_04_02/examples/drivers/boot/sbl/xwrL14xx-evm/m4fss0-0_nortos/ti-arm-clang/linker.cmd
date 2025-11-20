
/* make sure below retain is there in your linker command file, it keeps the vector table in the final binary */
--retain="*(.vectors)"
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=4096
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=512


SECTIONS
{
    /* This has the M4F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > M4F_VECS
    .bss:    {} palign(8) > M4F_RAM     /* This is where uninitialized globals go */
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)
    .text:   {} palign(8) > M4F_SHA_RAM    /* This is where code resides */
    .data:   {} palign(8) > M4F_RAM    /* This is where initialized globals and static go */
    .rodata: {} palign(8) > M4F_SHA_RAM     /* This is where const's go */
    .sysmem: {} palign(8) > M4F_RAM    /* This is where the malloc heap goes */
    .stack:  {} palign(8) > M4F_RAM     /* This is where the main() stack goes */
    /* This section has code that configures the APPSS/HWASS shared memory either to APPSS or HWASS based on SH_MEM_CONFIG parameter in meta header.
       Hence this code has to run from regular RAM */
    .jumptoApp: {} palign(8) > M4F_RAM
    .jumptoAppdata: {} palign(8) > M4F_RAM
}

/* Using 256KB of M4 Shared Memory for SBL */
/* Using 64 bytes of RAM for Data, BSS, Stack and Heap of SBL. This is because of Errata DIG#14: Corrupted Data Store for Partial Write in Shared Memory. Workaround
   is to use shared memory as read only memory or disable ECC for APPSS/HWASS memory when shared with APPSS */
MEMORY
{
    M4F_VECS : ORIGIN = 0x00480000 , LENGTH = 0x00000200
    M4F_SHA_RAM  : ORIGIN = 0x00480200 , LENGTH = 0x00040000 - 0x00000200
    M4F_RAM  : ORIGIN = 0x00470000 , LENGTH = 0x00010000
}
