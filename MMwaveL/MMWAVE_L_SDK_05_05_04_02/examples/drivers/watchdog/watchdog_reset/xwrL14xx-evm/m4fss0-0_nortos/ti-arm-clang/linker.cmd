
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
--stack_size=8192
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=8192


SECTIONS
{
    /* This has the M4F entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > M4F_VECS
    .bss:    {} palign(8) > M4F_RAM12     /* This is where uninitialized globals go */
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)
    /*
    * Maintaining a backup of data section so as to restore after watchdog reset
    * As program executes data section will be updated. During watchdog/warm reset,
    * as RBL does not reload from flash we copy the data section from backed up memory
    * location.
    */
    GROUP                       
    {
        .data:   {} palign(8)   /* This is where initialized globals and static go */
        LOAD_START(__LOAD_DATA_START)
        LOAD_END(__LOAD_DATA_END)
        RUN_START(__RUN_DATA_START)
    } load > M4F_DATALOAD,  run > M4F_DATARUN
    .text:   {} align(8) >> M4F_RAM12 | M4F_RAM3     /* This is where code resides */
    .rodata: {} align(8) >> M4F_RAM12 | M4F_RAM3     /* This is where const's go */
    .sysmem: {} palign(8) > M4F_RBL     /* This is where the malloc heap goes */
    .stack:  {} palign(8) > M4F_RBL     /* This is where the main() stack goes */
    .l3:     {} palign(8) > HWASS_SHM_MEM     /* This is where L3 data goes */
}

MEMORY
{
    M4F_VECS : ORIGIN = 0x00400000 , LENGTH = 0x00000200
    M4F_RAM12  : ORIGIN = 0x00400200 , LENGTH = (0x00058000 - 0x200) /* 32KB of RAM2 is being used by RBL */
    M4F_RBL    : ORIGIN = 0x00458000 , LENGTH = 0x8000 /* 32KB of RAM2 is being used by RBL */
    M4F_RAM3  : ORIGIN = 0x00460000 , LENGTH = 0x0001F000
    /* when using multi-core application's i.e more than one m4F active, make sure
     * this memory does not overlap with other m4F's
     */
    M4F_DATALOAD     : ORIGIN = 0x0047F000 , LENGTH = 0x800  /*Memory space where data section is loaded*/
    M4F_DATARUN     : ORIGIN = 0x0047F800 , LENGTH = 0x800  /*Memory space of data section during runtime*/

    HWASS_SHM_MEM : ORIGIN = 0x60000000, LENGTH = 0x00080000 /* 256KB in APPSS PD, 96KB in FECSS PD and 160KB in HWA PD */
}
