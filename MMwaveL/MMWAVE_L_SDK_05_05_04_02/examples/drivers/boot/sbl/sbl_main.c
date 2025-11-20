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

#include "kernel/dpl/DebugP.h"
#include "kernel/dpl/AddrTranslateP.h"
#include "board/flash.h"
#include "drivers/hw_include/cslr_soc.h"
#include "drivers/hw_include/hw_types.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "sbl.h"
#include "app_errors.h"
#include "bootload.h"
#include "parser.h"

/**************************************************************************
 *************************** Global Variables *****************************
 **************************************************************************/
 
 sbl_Obj_t sblObj;

/**************************** CRC-32 EDMA Parameters***********************/
uint32_t   edmaBaseAddr, edmaRegionId, crcdmaCh, crcTcc, crcParam;
EDMACCPaRAMEntry   crcEdmaParam;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
volatile uint32_t appBootVec __attribute__ ((section (".jumptoAppdata")));
volatile uint32_t* vtor __attribute__((section (".jumptoAppdata")));
volatile uint32_t* appShmClk __attribute__ ((section (".jumptoAppdata")));
volatile uint32_t* topPrcmShmCfg __attribute__ ((section (".jumptoAppdata")));
volatile uint32_t* appRcmIpccfgclkgate1 __attribute__ ((section (".jumptoAppdata")));
void sbl_switchtoapp()
{
    vtor = (volatile uint32_t*)0xE000ED08;
    appShmClk = (volatile uint32_t*)0x56060398;
    topPrcmShmCfg = (volatile uint32_t*)0x5A040500;
    appRcmIpccfgclkgate1 = (volatile uint32_t*)0x56040078;
    appBootVec = t_Metaheader.w_BootVector; 
    *vtor = t_Metaheader.w_BootVector;

    /*Ungating APP_CTRL clock as it was gated in System Deinit*/
    *appRcmIpccfgclkgate1 = *appRcmIpccfgclkgate1 & (0xF8FFFFFF);
    if(t_Metaheader.w_ShMemAlloc == 0)
    {
        
        // Configure the Shared RAM
        *topPrcmShmCfg = ((*topPrcmShmCfg & ~CSL_TOP_PRCM_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_APPSS_CONFIG_MASK) | 0x0);

        *appShmClk = 0x5;
    }
    else if (t_Metaheader.w_ShMemAlloc == 1)
    {
        
        // Configure the Shared RAM
        *topPrcmShmCfg = ((*topPrcmShmCfg & ~CSL_TOP_PRCM_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_APPSS_CONFIG_MASK) | 0x7);

        *appShmClk = 0x6;
    }
    else if (t_Metaheader.w_ShMemAlloc == 2)
    {
        
        // Configure the Shared RAM
        *topPrcmShmCfg = ((*topPrcmShmCfg & ~CSL_TOP_PRCM_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_APPSS_CONFIG_MASK) | 0x38);

        *appShmClk = 0x9;
    }
    else if (t_Metaheader.w_ShMemAlloc == 3)
    {
        
        // Configure the Shared RAM
        *topPrcmShmCfg = ((*topPrcmShmCfg & ~CSL_TOP_PRCM_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_REG_HWA_PD_MEM_SHARE_APPSS_CONFIG_MASK) | 0x3f);

        *appShmClk = 0xA;
    }
    else
    {
    }
    /*Gating APP_CTRL clock for clean handoff to application*/
    *appRcmIpccfgclkgate1 = *appRcmIpccfgclkgate1 | (0x07000000);
    __asm(" MSR MSP, %0" : :"r" ((uint32_t)*(uint32_t*)appBootVec));
    __asm("BX %0"::"r" ((uint32_t)*(uint32_t*)(appBootVec + 4)));
}

/* Configure an EDMA channel to transfer meta image to CRC engine for calculating CRC32*/
uint32_t sbl_configureEdmaCrc()
{
    uint32_t   retVal;
    edmaBaseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    edmaRegionId = EDMA_getRegionId(gEdmaHandle[0]);
    crcdmaCh = EDMA_RESOURCE_ALLOC_ANY;
    retVal = EDMA_allocDmaChannel(gEdmaHandle[0], &crcdmaCh);
    crcTcc = EDMA_RESOURCE_ALLOC_ANY;
    retVal = EDMA_allocTcc(gEdmaHandle[0], &crcTcc);
    crcParam = EDMA_RESOURCE_ALLOC_ANY;
    retVal = EDMA_allocParam(gEdmaHandle[0], &crcParam);
    /* Request channel */
    EDMAConfigureChannelRegion(edmaBaseAddr, edmaRegionId, EDMA_CHANNEL_TYPE_DMA,
         crcdmaCh, crcTcc, crcParam, 0);
    return retVal;
}

void sbl_computeCrc32Cpu(uint32_t* data, uint32_t numbytes, uint32_t* crc32)
{
    uint32_t                patternCnt, baseAddr;
    CRC_Channel_t           chNumber;
    CRC_SignatureRegAddr    psaSignRegAddr;
    uint32_t app_crc_sect_cnt = 1U;
    uint32_t app_crc_watchdog_preload_val = 0U;
    uint32_t app_crc_block_preload_val = 0U;
    uint32_t app_crc_type = CRC_TYPE_32bit;
    uint32_t app_crc_data_size = CRC_DW_32bit;
    uint32_t app_crc_bit_swap = 1;
    uint32_t app_crc_byte_swap = 0;
    CRC_Signature calcrc;

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    patternCnt         = (numbytes / 4);
    chNumber           = CRC_CHANNEL_1;


    /* Get CRC PSA signature register address */
    CRC_getPSASigRegAddr(baseAddr, chNumber, &psaSignRegAddr);

    /* Initialize and Configure CRC channel */
    CRC_initialize(baseAddr, chNumber, app_crc_watchdog_preload_val, app_crc_block_preload_val);

    CRC_configure(baseAddr, chNumber, patternCnt, app_crc_sect_cnt, CRC_OPERATION_MODE_FULLCPU, app_crc_type, app_crc_data_size ,app_crc_bit_swap, app_crc_byte_swap);

    /* Reset the CRC channel*/
    CRC_channelReset(baseAddr, chNumber);

    /* Program Param Set to tranfer data to CRC engine for CRC32 calculation */
    EDMACCPaRAMEntry_init(&crcEdmaParam);
    crcEdmaParam.srcAddr       = (uint32_t) ((uint8_t*)data + 0x22000000U);
    crcEdmaParam.destAddr      = (uint32_t) (psaSignRegAddr.regL);
    crcEdmaParam.aCnt          = (uint16_t) 4;
    crcEdmaParam.bCnt          = (uint16_t) numbytes/4;
    crcEdmaParam.cCnt          = (uint16_t) 1;
    crcEdmaParam.bCntReload    = (uint16_t) numbytes/4;
    crcEdmaParam.srcBIdx       = (int16_t) 4;
    crcEdmaParam.destBIdx      = (int16_t) 0;
    crcEdmaParam.srcCIdx       = (int16_t) (-1 * numbytes);
    crcEdmaParam.destCIdx      = (int16_t) 0;
    crcEdmaParam.linkAddr      = 0xFFFFU;
    crcEdmaParam.opt          |= (EDMA_OPT_SYNCDIM_MASK | EDMA_OPT_TCINTEN_MASK | ((((uint32_t)crcTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) | (EDMA_FIFO_WIDTH_32BIT << EDMA_TPCC_OPT_FWID_SHIFT)); 

    EDMASetPaRAM(edmaBaseAddr, crcParam, &crcEdmaParam);
    /* Start the transfer */
    EDMAEnableTransferRegion(edmaBaseAddr, edmaRegionId, crcdmaCh,
             EDMA_TRIG_MODE_MANUAL);
    /* Wait for the CRC calculation to be complete for requested data and size */
    while(EDMAReadIntrStatusRegion(edmaBaseAddr, edmaRegionId, crcTcc) != 1);
    EDMAClrIntrRegion(edmaBaseAddr, edmaRegionId, crcTcc);

    /* Fetch CRC signature value */
    CRC_getPSASig(baseAddr, chNumber, &calcrc);
    *crc32 = calcrc.regL;
    return;
}

void sbl_configureCrc32Cpu()
{
    uint32_t                baseAddr,patternCnt;
    CRC_Channel_t           chNumber;
    uint32_t app_crc_sect_cnt = 1U;
    uint32_t app_crc_watchdog_preload_val = 0U;
    uint32_t app_crc_block_preload_val = 0U;
    uint32_t app_crc_type = CRC_TYPE_32bit;
    uint32_t app_crc_data_size = CRC_DW_32bit;
    uint32_t app_crc_bit_swap = 1;
    uint32_t app_crc_byte_swap = 0;

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    chNumber           = CRC_CHANNEL_1;
    patternCnt         = 0;

    /* Initialize and Configure CRC channel */
    CRC_initialize(baseAddr, chNumber, app_crc_watchdog_preload_val, app_crc_block_preload_val);

    CRC_configure(baseAddr, chNumber, patternCnt, app_crc_sect_cnt, CRC_OPERATION_MODE_FULLCPU, app_crc_type, app_crc_data_size ,app_crc_bit_swap, app_crc_byte_swap);

    /* Reset the CRC channel*/
    CRC_channelReset(baseAddr, chNumber);
}

void sbl_sendDataCrc32Cpu(uint32_t* data, uint32_t numbytes)
{
    uint32_t                patternCnt, baseAddr;
    CRC_Channel_t           chNumber;
    CRC_SignatureRegAddr    psaSignRegAddr;

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    patternCnt         = (numbytes / 4);
    chNumber           = CRC_CHANNEL_1;
    HW_WR_FIELD32(baseAddr + CRC_PCOUNT_REG1,
                              CRC_PCOUNT_REG1_PAT_COUNT1,
                              patternCnt);

    /* Get CRC PSA signature register address */
    CRC_getPSASigRegAddr(baseAddr, chNumber, &psaSignRegAddr);

    /* Program Param Set to tranfer data to CRC engine for CRC32 calculation */
    EDMACCPaRAMEntry_init(&crcEdmaParam);
    crcEdmaParam.srcAddr       = (uint32_t) ((uint8_t*)data + 0x22000000U);
    crcEdmaParam.destAddr      = (uint32_t) (psaSignRegAddr.regL);
    crcEdmaParam.aCnt          = (uint16_t) 4;
    crcEdmaParam.bCnt          = (uint16_t) numbytes/4;
    crcEdmaParam.cCnt          = (uint16_t) 1;
    crcEdmaParam.bCntReload    = (uint16_t) numbytes/4;
    crcEdmaParam.srcBIdx       = (int16_t) 4;
    crcEdmaParam.destBIdx      = (int16_t) 0;
    crcEdmaParam.srcCIdx       = (int16_t) (-1 * numbytes);
    crcEdmaParam.destCIdx      = (int16_t) 0;
    crcEdmaParam.linkAddr      = 0xFFFFU;
    crcEdmaParam.opt          |= (EDMA_OPT_SYNCDIM_MASK | EDMA_OPT_TCINTEN_MASK | ((((uint32_t)crcTcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK) | (EDMA_FIFO_WIDTH_32BIT << EDMA_TPCC_OPT_FWID_SHIFT)); 

    EDMASetPaRAM(edmaBaseAddr, crcParam, &crcEdmaParam);
    /* Start the transfer */
    EDMAEnableTransferRegion(edmaBaseAddr, edmaRegionId, crcdmaCh,
             EDMA_TRIG_MODE_MANUAL);
    /* Wait for the CRC calculation to be complete for requested data and size */
    while(EDMAReadIntrStatusRegion(edmaBaseAddr, edmaRegionId, crcTcc) != 1);
    EDMAClrIntrRegion(edmaBaseAddr, edmaRegionId, crcTcc);
    return;
}

uint32_t sbl_getCrc32Cpu()
{
    uint32_t                baseAddr;
    CRC_Channel_t           chNumber;
    CRC_Signature calcrc;

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    chNumber           = CRC_CHANNEL_1;
    /* Fetch CRC signature value */
    CRC_getPSASig(baseAddr, chNumber, &calcrc);
    return calcrc.regL;
}


void sbl_computeCrc16Cpu(uint16_t* data, uint32_t numbytes, uint16_t* crc16)
{
    uint32_t                loopCnt, patternCnt, baseAddr;
    CRC_Channel_t           chNumber;
    CRC_SignatureRegAddr    psaSignRegAddr;
    uint32_t app_crc_sect_cnt = 1U;
    uint32_t app_crc_watchdog_preload_val = 0U;
    uint32_t app_crc_block_preload_val = 0U;
    uint32_t app_crc_type = CRC_TYPE_16bit;
    uint32_t app_crc_data_size = CRC_DW_16bit;
    uint32_t app_crc_bit_swap = 1;
    uint32_t app_crc_byte_swap = 0;
    CRC_Signature calcrc;

    /* Configure CRC parameters */
    baseAddr           = (uint32_t) AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    patternCnt         = (numbytes / 2);
    chNumber           = CRC_CHANNEL_2;


    /* Get CRC PSA signature register address */
    CRC_getPSASigRegAddr(baseAddr, chNumber, &psaSignRegAddr);

    /* Initialize and Configure CRC channel */
    CRC_initialize(baseAddr, chNumber, app_crc_watchdog_preload_val, app_crc_block_preload_val);

    CRC_configure(baseAddr, chNumber, patternCnt, app_crc_sect_cnt, CRC_OPERATION_MODE_FULLCPU, app_crc_type, app_crc_data_size ,app_crc_bit_swap, app_crc_byte_swap);

    /* Reset the CRC channel*/
    CRC_channelReset(baseAddr, chNumber);

    /* compute the CRC by writing the data buffer on which CRC computation is needed */
    for (loopCnt = 0; loopCnt < patternCnt ; loopCnt++)
    {
        HW_WR_REG16(psaSignRegAddr.regL, data[loopCnt]);
    }

    /* Fetch CRC signature value */
    CRC_getPSASig(baseAddr, chNumber, &calcrc);
    *crc16 = calcrc.regL;
    return;
}

/*!
 *  @b Description
 *  @n
 *      This function download the application meta imageover over a device peripheral.
 *      It does the following:
 *      1. Erases the portion of SFLASH where the Metaimage has to be stored.
 *      2. Receives the Metaimage over transport interface.
 *      3. Stores the image into the SFLASH.
 *
 *  @param[in]  qspiFlashHandle
 *      Handle of QSPI Flash module.
 *  @param[in]  flashAddr
 *      Address of SFLASH location where the application meta image is written to.
 *
 *  \ingroup SBL_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   SBL Error code
 */
int32_t SBL_imageFlasher(Flash_Handle qspiFlashHandle, uint32_t flashAddr)
{
    int32_t     totDataLen = 0;
    int32_t     retVal = 0;
    uint32_t    numBlocksErase = 0;
    uint32_t    index, offset;
    Flash_Attrs* attr;

    SBL_printf("Debug: Erasing SFlash...\r\n");

    attr = Flash_getAttrs(CONFIG_FLASH0);
    /* Calculate the number of blocks to erase.*/
    numBlocksErase = SBL_MAX_METAIMAGE_SIZE / (attr->blockSize);
    offset         = (flashAddr / (attr->blockSize));

    /* Erase FLASH memory*/
    for (index = 0; index < numBlocksErase; index++)
    {
        retVal = Flash_eraseBlk(qspiFlashHandle, (index + offset));
        if(retVal != 0)
        {
            goto exitImageFlasher;
        }
    }
    SBL_printf("Debug: Flash Erase complete\r\n");

    totDataLen = SBL_transportDownloadFile (qspiFlashHandle, flashAddr, SBL_MAX_METAIMAGE_SIZE);

    if (totDataLen < 0)
    {
        SBL_printf("Error: File download failed.\r\n");
        retVal = totDataLen;
    }
    else
    {
        SBL_printf("Debug: Total data written = 0x%x\r\n", totDataLen);
    }
exitImageFlasher:
    return retVal;
}


/**************************************************************************
 *************************** Function Definitions *************************
 **************************************************************************/

void sbl_main(void *args)
{
    /* Open drivers for peripherals*/
    Drivers_open();
    Board_driversOpen();
    int32_t retVal = MINUS_ONE, downldSuccess = MINUS_ONE;
    uint8_t autoboot = SBL_AUTOBOOT_COUNT;
    uint8_t userInput = 0;
    uint32_t efuseRow16 = 0;

    SBL_printf ("\r\n");
    SBL_printf ("**********************************************\r\n");
    SBL_printf ("Secondary Bootloader Application Start \r\n");
    SBL_printf ("**********************************************\r\n");

    SBL_printf ("Press CR key or Space key to stop auto boot and Update Meta Image...\r\n");
    SBL_printf ("Loading existing Meta Image from Flash in ");

    efuseRow16 = HW_RD_REG32(CSL_TOP_EFUSE_U_BASE + CSL_TOP_EFUSE_EFUSE0_ROW_16);

    /* Read FEC Eclipse mode */
    if(((efuseRow16 >> 5) & 0x1U) == 0x1U)
    {
        sblObj.fecShRamPresent = 1;
        if (((efuseRow16 >> 6) & 0x1U) == 0x1U)
        {
            sblObj.fecEclipseMode = 1U;
        }
        else
        {
            sblObj.fecEclipseMode = 0U;
        }
    }

    do
    {
        retVal = SBL_transportRead((uint8_t*)&userInput, 1U);

        /* Stop autoboot only if the input character is CR key or Space key */
        if (retVal == 0)
        {  
            if ((userInput != SBL_CR_KEY) && (userInput != SBL_SPACE_KEY))
            {
                retVal = MINUS_ONE;
            }
        }

        /* Check if user interrupted the autoboot */
        if (retVal == 0)
        {
            sblObj.metaimageUpdate = 1;
            SBL_printf ("\r\n\nUpdate Meta Image selected\r\n");
        }
        else
        {
            SBL_printf ("  %d", autoboot--);
            continue;
        }
    }while((retVal != 0) && (autoboot != 0));

    if(sblObj.metaimageUpdate == 1)
    {
        SBL_printf ("\r\n Loading Application over UART...\r\n");
        downldSuccess = SBL_imageFlasher(gFlashHandle[CONFIG_FLASH0], M_META_IMAGE);
        if(downldSuccess == 0)
            SBL_printf ("\r\n Image Loading Successful...\r\n");
        else
            SBL_printf ("\r\n Image Loading Failed...\r\n");
        sblObj.metaimageUpdate = 0;
    }
    /* Configuring EDMA for Sending data to CRC engine */
    sbl_configureEdmaCrc();

    SBL_printf ("\r\n Booting the Application.\r\n");
    retVal = bootload_qspi(M_META_IMAGE);

    if(retVal != 0)
    {
        SBL_printf ("\r\n No Valid application found. Booting backup factory default image...\r\n");
        /* Perform Boot Loading of default image */
        retVal = bootload_qspi(M_META_IMAGE_BACKUP);
    }

    if (retVal == 0)
    {
        Board_driversClose();
        Drivers_close();
        Board_deinit();
        System_deinit();
        sbl_switchtoapp();
    }
    else
    {
        SBL_printf ("\r\n No Valid Image found in flash. Try loading Valid Application...\r\n");
    }
}

