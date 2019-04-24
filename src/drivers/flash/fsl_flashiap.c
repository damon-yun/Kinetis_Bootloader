/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_flashiap.h"
#include "fsl_flash.h"

#define HZ_TO_KHZ_DIV 1000

/*******************************************************************************
 * Static Code
 ******************************************************************************/

static status_t translate_iap_status(uint32_t status)
{
    /* Translate IAP return code to sdk status code */
    if (status == kStatus_Success)
    {
        return status;
    }
    else
    {
        return MAKE_STATUS(kStatusGroup_FLASHIAP, status);
    }
}

/*! @brief Gets the right address, sector and block size of current flash type which is indicated by address.*/
static status_t flash_get_matched_operation_info(flash_config_t           *config,
                                                 uint32_t                  address,
                                                 flash_operation_config_t *info)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Clean up info Structure*/
    memset(info, 0, sizeof(flash_operation_config_t));

    info->convertedAddress = address - config->PFlashBlockBase;
    info->activeSectorSize = config->PFlashSectorSize;
    info->activeBlockSize  = config->PFlashTotalSize / config->PFlashBlockCount;
//    info->activePageSize   = config->PFlashPageSize;

    info->blockWriteUnitSize         = FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE;
    info->sectorCmdAddressAligment   = FSL_FEATURE_SYSCON_FLASH_SECTOR_CMD_ADDRESS_ALIGMENT;
    info->sectionCmdAddressAligment  = FSL_FEATURE_SYSCON_FLASH_SECTION_CMD_ADDRESS_ALIGMENT;
    info->resourceCmdAddressAligment = FSL_FEATURE_SYSCON_FLASH_RESOURCE_CMD_ADDRESS_ALIGMENT;
    info->checkCmdAddressAligment    = FSL_FEATURE_SYSCON_FLASH_CHECK_CMD_ADDRESS_ALIGMENT;

    return kStatus_FLASH_Success;
}

/*! @brief Validates the range and alignment of the given address range.*/
static status_t flash_check_range(flash_config_t *config,
                                  uint32_t        startAddress,
                                  uint32_t        lengthInBytes,
                                  uint32_t        alignmentBaseline)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Verify the start and length are alignmentBaseline aligned. */
    if ((startAddress  & (alignmentBaseline - 1)) ||
        (lengthInBytes & (alignmentBaseline - 1)))
    {
        return kStatus_FLASH_AlignmentError;
    }

    /* check for valid range of the target addresses */
    if ((startAddress < config->PFlashBlockBase) ||
        ((startAddress + lengthInBytes) > (config->PFlashBlockBase + config->PFlashTotalSize)))
    {
        return kStatus_FLASH_AddressError;
    }

    return kStatus_FLASH_Success;
}


/*******************************************************************************
 * Public Code: Flash IAP API
 ******************************************************************************/

status_t FLASHIAP_PrepareSectorForWrite(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_PrepareSectorforWrite;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_CopyRamToFlash(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_CopyRamToFlash;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    command[4] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_EraseSector(uint32_t startSector, uint32_t endSector, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_EraseSector;
    command[1] = startSector;
    command[2] = endSector;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ErasePage(uint32_t startPage, uint32_t endPage, uint32_t systemCoreClock)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_ErasePage;
    command[1] = startPage;
    command[2] = endPage;
    command[3] = systemCoreClock / HZ_TO_KHZ_DIV;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_BlankCheckSector(uint32_t startSector, uint32_t endSector)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_BlankCheckSector;
    command[1] = startSector;
    command[2] = endSector;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_Compare(uint32_t dstAddr, uint32_t *srcAddr, uint32_t numOfBytes)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_Compare;
    command[1] = dstAddr;
    command[2] = (uint32_t)srcAddr;
    command[3] = numOfBytes;
    iap_entry(command, result);

    return translate_iap_status(result[0]);
}


/**********************************************************************************
 *  Add By Damon
 *********************************************************************************/
status_t FLASHIAP_ReadPID(uint32_t *pPIDBbuf)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_ReadPartId;
    iap_entry(command, result);

    *pPIDBbuf = result[1];

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ReadBootCodeVersion(uint32_t *pBootVersion)
{
    uint32_t command[5], result[4];

    command[0] = kIapCmd_FLASHIAP_Read_BootromVersion;
    iap_entry(command, result);

    *pBootVersion = result[1];

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ReinvokeIsp(uint32_t ispMode)
{
    uint32_t command[5], result[5];

    command[0] = kIapCmd_FLASHIAP_ReinvokeISP;
    command[1] = ispMode;
    iap_entry(command, result);

    /*Will Not Arrived here unless a Mistake */
    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ReadUID(uint32_t *pUID)
{
    uint32_t command[5], result[5];

    command[0] = kIapCmd_FLASHIAP_ReadUid;
    iap_entry(command, result);

    pUID[0] = result[1];
    pUID[1] = result[2];
    pUID[2] = result[3];
    pUID[3] = result[4];

    return translate_iap_status(result[0]);
}

status_t FLASHIAP_ReadSignature(uint32_t *pSignature)
{
    uint32_t command[5], result[5];

    command[0] = kIapCmd_FLASHIAP_ReadMisr;
    iap_entry(command, result);

    pSignature[0] = result[1];

    return translate_iap_status(result[0]);
}

/*******************************************************************************
 * Used For kBoot flash friver Code
 ******************************************************************************/
//Device Hex coding
#define __PID_LPC54113J128_     0x06254113
#define __PID_LPC54113J256_     0x36454113
#define __PID_LPC54114J256_     0x36454114

status_t FLASH_Init(flash_config_t *config)
{
    uint32_t flashDensity;
    uint32_t u32PID = 0;

    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Read The PID To  Decode MCU's Flash Size */
    FLASHIAP_ReadPID(&u32PID);

    switch (u32PID) {
    case __PID_LPC54113J128_:
        flashDensity = FSL_FEATURE_SYSCON_FLASH128_SIZE_BYTES;
        break;
        
    case __PID_LPC54113J256_:
    case __PID_LPC54114J256_:    
        flashDensity = FSL_FEATURE_SYSCON_FLASH256_SIZE_BYTES;
        break;

    default :
        flashDensity = FSL_FEATURE_SYSCON_FLASH128_SIZE_BYTES;
        break;
    }

    /* fill out a few of the structure members */
    config->PFlashBlockBase = FSL_FEATURE_FLASH_FLASH_START_ADDRESS;
    config->PFlashTotalSize = flashDensity;
    config->PFlashBlockCount = FSL_FEATURE_SYSCON_FLASH_BLOCK_NUM;
    config->PFlashSectorSize = FSL_FEATURE_SYSCON_FLASH_SECTOR_SIZE_BYTES;

#if defined(FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL) && FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL
    config->PFlashAccessSegmentSize = kFLASH_AccessSegmentBase << FTFx->FACSS;
    config->PFlashAccessSegmentCount = FTFx->FACSN;
#else
    config->PFlashAccessSegmentSize = 0;
    config->PFlashAccessSegmentCount = 0;
#endif /* FSL_FEATURE_FLASH_HAS_ACCESS_CONTROL */

    config->PFlashCallback = NULL;


#if 0   //LPC54114 Don't support this feature

/* copy required flash commands to RAM */
#if (FLASH_DRIVER_IS_FLASH_RESIDENT && !FLASH_DRIVER_IS_EXPORTED)
    if (kStatus_FLASH_Success != flash_check_execute_in_ram_function_info(config))
    {
        s_flashExecuteInRamFunctionInfo.activeFunctionCount = 0;
        s_flashExecuteInRamFunctionInfo.flashRunCommand = s_flashRunCommand;
        s_flashExecuteInRamFunctionInfo.flashCacheClearCommand = s_flashCacheClearCommand;
        config->flashExecuteInRamFunctionInfo = &s_flashExecuteInRamFunctionInfo.activeFunctionCount;
        FLASH_PrepareExecuteInRamFunctions(config);
    }
#endif

    config->FlexRAMBlockBase = FSL_FEATURE_FLASH_FLEX_RAM_START_ADDRESS;
    config->FlexRAMTotalSize = FSL_FEATURE_FLASH_FLEX_RAM_SIZE;

#if FLASH_SSD_IS_FLEXNVM_ENABLED
    {
        status_t returnCode;
        config->DFlashBlockBase = FSL_FEATURE_FLASH_FLEX_NVM_START_ADDRESS;
        returnCode = flash_update_flexnvm_memory_partition_status(config);
        if (returnCode != kStatus_FLASH_Success)
        {
            return returnCode;
        }
    }
#endif

#endif 


    return kStatus_FLASH_Success;
}


status_t FLASH_SetCallback(flash_config_t *config, flash_callback_t callback)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    config->PFlashCallback = callback;

    return kStatus_FLASH_Success;
}

//#if FLASH_DRIVER_IS_FLASH_RESIDENT
//status_t FLASH_PrepareExecuteInRamFunctions(flash_config_t *config)
//{
//    flash_execute_in_ram_function_config_t *flashExecuteInRamFunctionInfo;

//    if (config == NULL)
//    {
//        return kStatus_FLASH_InvalidArgument;
//    }

//    flashExecuteInRamFunctionInfo = (flash_execute_in_ram_function_config_t *)config->flashExecuteInRamFunctionInfo;

//    copy_flash_run_command(flashExecuteInRamFunctionInfo->flashRunCommand);
//    copy_flash_cache_clear_command(flashExecuteInRamFunctionInfo->flashCacheClearCommand);
//    flashExecuteInRamFunctionInfo->activeFunctionCount = kFLASH_ExecuteInRamFunctionTotalNum;

//    return kStatus_FLASH_Success;
//}
//#endif /* FLASH_DRIVER_IS_FLASH_RESIDENT */

status_t FLASH_EraseAll(flash_config_t *config, uint32_t key)
{
    uint8_t  sector_start = 0;
    uint8_t  sector_end   = (config->PFlashTotalSize / config->PFlashSectorSize) - 1;

    status_t returnCode;

    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Prepare the Sector for erase operation */
    returnCode = FLASHIAP_PrepareSectorForWrite(sector_start, sector_end);

    if (kStatus_FLASHIAP_Success == returnCode) {
        /* Erase all Sector */
        returnCode = FLASHIAP_EraseSector(sector_start,
                                         sector_end,
                                         SystemCoreClock);
    }

    return returnCode;
}


status_t FLASH_Erase(flash_config_t *config,        uint32_t start,
                     uint32_t        lengthInBytes, uint32_t key)
{
    uint8_t  start_sector = 0;
    uint8_t  end_sector   = 0;
    uint16_t start_page   = 0;
    uint16_t end_page     = 0;
    uint32_t end_address;
    status_t return_code;
    
    flash_operation_config_t flashInfo;

    if ((NULL == config) || (0 == lengthInBytes)) {
        return kStatus_FLASH_InvalidArgument;
    }

    flash_get_matched_operation_info(config, start, &flashInfo);

    /* Check the supplied address range. The Erase unit based on page */
    return_code = flash_check_range(config,        start,
                                    lengthInBytes, flashInfo.activeSectorSize);
    if (return_code)
    {
        return return_code;
    }

    start       = flashInfo.convertedAddress;
    end_address = start + lengthInBytes - 1;

    start_sector = start / flashInfo.activeSectorSize;
    end_sector   = end_address / flashInfo.activeSectorSize;

    /* Disable irq */
    __disable_irq();

    FLASHIAP_PrepareSectorForWrite(start_sector, end_sector);

    /* \TODO Fsl Flash 只区分到sector size ，因此放弃LPC的page erase 命令
     */
    return_code = FLASHIAP_EraseSector(start_sector, end_sector, SystemCoreClock);

    /* Resume irq */
    __enable_irq();

    /* calling flash callback function if it is available */
    if (config->PFlashCallback)
    {
        config->PFlashCallback();
    }

    return (return_code);
}

status_t FLASH_Program(flash_config_t *config, uint32_t start,
                       uint32_t       *src,    uint32_t length_in_bytes)
{
    uint8_t  start_sector = 0;
    uint8_t  end_sector   = 0;
    status_t return_code;
    flash_operation_config_t flash_info;

    if ((NULL == src) || (0 == length_in_bytes)) {
        return kStatus_FLASH_InvalidArgument;
    }

    flash_get_matched_operation_info(config, start, &flash_info);

    /* Check the supplied address range. The Pragram Unit Based On Page */
    return_code = flash_check_range(config,          start,
                                    length_in_bytes, flash_info.blockWriteUnitSize);
    if (return_code)
    {
        return return_code;
    }

    start = flash_info.convertedAddress;

    start_sector = start / flash_info.activeSectorSize;
    end_sector   = start_sector + length_in_bytes / flash_info.activeSectorSize;

    /* Disable irq */
    __disable_irq();

    FLASHIAP_PrepareSectorForWrite(start_sector, end_sector);
    return_code = FLASHIAP_CopyRamToFlash(start,           src,
                                       flash_info.blockWriteUnitSize, SystemCoreClock);

    /* Resume irq */
    __enable_irq();

    return (return_code);

}

status_t FLASH_GetSecurityState(flash_config_t *config, flash_security_state_t *state)
{
    if ((config == NULL) || (state == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Flash in unsecured state */
    *state = kFLASH_SecurityStateNotSecure;

    return (kStatus_FLASH_Success);
}

status_t FLASH_ReadResource(flash_config_t *config,
                            uint32_t        start,
                            uint32_t       *dst,
                            uint32_t        lengthInBytes,
                            flash_read_resource_option_t option)
{
    status_t returnCode;

    if ((NULL == config) || (NULL == dst) || (0 == lengthInBytes)) {
        return kStatus_FLASH_InvalidArgument;
    }

    switch (option) {

    case kFLASH_ResourceOptionPidNumber:
        returnCode = FLASHIAP_ReadPID(dst);
        break;

    case kFLASH_ResourceOptionBootCode:
        returnCode = FLASHIAP_ReadBootCodeVersion(dst);
        break;

    case kFLASH_ResourceOptionUID:
        returnCode = FLASHIAP_ReadUID(dst);
        break;

    case kFLASH_ResourceOptionSignature:
        returnCode = FLASHIAP_ReadSignature(dst);
        break;

    default:
        return kStatus_FLASH_CommandNotSupported;
    }

    return returnCode;
}

status_t FLASH_SecurityBypass(flash_config_t *config, const uint8_t *backdoorKey)
{
    if (NULL == config) {
        return kStatus_FLASH_InvalidArgument;
    }

    /** TODO In LPC5411X not support this feature */

    return (kStatus_FLASH_Success);
}

status_t FLASH_VerifyEraseAll(flash_config_t *config, flash_margin_value_t margin)
{
    status_t returnCode;

    if (NULL == config) {
        return kStatus_FLASH_InvalidArgument;
    }

    /* Check All Sector Whether Blank */
    returnCode =FLASHIAP_BlankCheckSector(0,
                                        config->PFlashTotalSize  /
                                        config->PFlashBlockCount /
                                        config->PFlashSectorSize);

    return (returnCode);
}

status_t FLASH_VerifyErase(flash_config_t *config,        uint32_t             start,
                           uint32_t        lengthInBytes, flash_margin_value_t margin)
{
    /* Check arguments. */
    uint8_t start_sector;
    uint8_t end_sector;
    status_t returnCode;
    
    flash_operation_config_t flashInfo;

    /*
     * TODO LPC5411X检查扇区是否擦除完全不好操作，因为检验只提供了基于扇区的检测，一个扇区的大小为
     *      32KB，而为了擦除方便，使用以页为单位进行擦除的，所以就产生了冲突，故此战且不提供擦除验证
     *      函数，统一返回kStatus_FLASH_Success
     */
//    flash_get_matched_operation_info(config, start, &flashInfo);
//
//    returnCode = flash_check_range(config, start, lengthInBytes, flashInfo.sectionCmdAddressAligment);
//    if (returnCode)
//    {
//        return returnCode;
//    }
//
//    start     = flashInfo.convertedAddress;
//    start_sector = start / flashInfo->activeSectorSize;
//    end_sector   = start + lengthInBytes / flashInfo->activeSectorSize;



    return kStatus_FLASH_Success;
}

status_t FLASH_VerifyProgram(flash_config_t *config,
                             uint32_t start,
                             uint32_t lengthInBytes,
                             const uint32_t *expectedData,
                             flash_margin_value_t margin,
                             uint32_t *failedAddress,
                             uint32_t *failedData)
{
    status_t return_code;
    
    flash_operation_config_t flash_info;

    if (expectedData == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    flash_get_matched_operation_info(config, start, &flash_info);

    return_code = flash_check_range(config, start, lengthInBytes,
                                    flash_info.checkCmdAddressAligment);
    if (return_code)
    {
        return return_code;
    }

    start = flash_info.convertedAddress;

    return_code = FLASHIAP_Compare(start, (uint32_t *)expectedData, lengthInBytes);

    return return_code;
}

status_t FLASH_GetProperty(flash_config_t       *config,
                           flash_property_tag_t  whichProperty,
                           uint32_t             *value)
{
    if ((config == NULL) || (value == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kFLASH_PropertyPflashSectorSize:
            /* \note Erase based on page */
            *value = config->PFlashSectorSize;
            break;

        case kFLASH_PropertyPflashTotalSize:
            *value = config->PFlashTotalSize;
            break;

        case kFLASH_PropertyPflashBlockSize:
            *value = config->PFlashTotalSize / config->PFlashBlockCount;
            break;

        case kFLASH_PropertyPflashBlockCount:
            *value = config->PFlashBlockCount;
            break;

        case kFLASH_PropertyPflashBlockBaseAddr:
            *value = config->PFlashBlockBase;
            break;

        case kFLASH_PropertyPflashFacSupport:
            *value = 0;
            break;

        case kFLASH_PropertyPflashAccessSegmentSize:
            *value = config->PFlashAccessSegmentSize;
            break;

        case kFLASH_PropertyPflashAccessSegmentCount:
            *value = config->PFlashAccessSegmentCount;
            break;

        case kFLASH_PropertyFlexRamBlockBaseAddr:
            *value = config->FlexRAMBlockBase;
            break;

        case kFLASH_PropertyFlexRamTotalSize:
            *value = config->FlexRAMTotalSize;
            break;

        default: /* catch inputs that are not recognized */
            return kStatus_FLASH_UnknownProperty;
    }

    return kStatus_FLASH_Success;
}

/** End of file */


