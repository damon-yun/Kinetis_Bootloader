/*
 * Copyright (c) 2014-2015, Freescale Semiconductor, Inc.
 * All rights reserved.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "property/property.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_assert.h"

#include "fsl_clock.h"  //add by damon.zhang
#include "fsl_power.h"
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bootloader_common.h for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
#if BL_TARGET_FLASH
    static uint32_t s_defaultClockDivider;

    if (option == kClockOption_EnterBootloader)
    {
        // General procedure to be implemented:
        // 1. Read clock flags and divider from bootloader config in property store
        bootloader_configuration_data_t *config = &g_bootloaderContext.propertyInterface->store->configurationData;
        uint8_t options = config->clockFlags;

        // Check if the USB HID peripheral is enabled. If it is enabled, we always
        // use the external OSC.
        bool isUsbEnabled = config->enabledPeripherals & kPeripheralType_USB_HID;

        // 2. If NOT High Speed and USB is NOT enabled, do nothing (use reset clock config)
        if ((options & kClockFlag_HighSpeed) && !isUsbEnabled)
        {
            /*!< Set up the clock sources */
            /*!< Set up FRO */
            POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
            CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                        being below the voltage for current speed */
            CLOCK_SetupFROClocking(12000000U);                    /*!< Set up FRO to the 12 MHz, just for sure */
            POWER_SetVoltageForFreq(12000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
            CLOCK_SetFLASHAccessCyclesForFreq(12000000U);   /*!< Set FLASH wait states for core */

            /*!< Set up dividers */
            CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Set AHBCLKDIV divider to value 1 */

            /*!< Set up clock selectors - Attach clocks to the peripheries */
            CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch MAIN_CLK to FRO12M */
            /*!< Set SystemCoreClock variable. */
            SystemCoreClock = 12000000;

            return;
        }

        /*!< Set up the clock sources */
        /*!< Set up FRO */
        POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
        CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                    being below the voltage for current speed */
        POWER_SetVoltageForFreq(96000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
        CLOCK_SetFLASHAccessCyclesForFreq(96000000U);   /*!< Set FLASH wait states for core */

        CLOCK_SetupFROClocking(96000000U);              /*!< Set up high frequency FRO output to selected frequency */

        /*!< Set up dividers */
        CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Set AHBCLKDIV divider to value 1 */

        /*!< Set up clock selectors - Attach clocks to the peripheries */
        CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK);                  /*!< Switch MAIN_CLK to FRO_HF */
        /*!< Set SystemCoreClock variable. */
        SystemCoreClock = 96000000;

    }
    else if (option == kClockOption_ExitBootloader)
    {
        /*!< Set up the clock sources */
        /*!< Set up FRO */
        POWER_DisablePD(kPDRUNCFG_PD_FRO_EN);                   /*!< Ensure FRO is on  */
        CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch to FRO 12MHz first to ensure we can change voltage without accidentally
                                                                    being below the voltage for current speed */
        CLOCK_SetupFROClocking(12000000U);                    /*!< Set up FRO to the 12 MHz, just for sure */
        POWER_SetVoltageForFreq(12000000U);             /*!< Set voltage for the one of the fastest clock outputs: System clock output */
        CLOCK_SetFLASHAccessCyclesForFreq(12000000U);   /*!< Set FLASH wait states for core */

        /*!< Set up dividers */
        CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);                  /*!< Set AHBCLKDIV divider to value 1 */

        /*!< Set up clock selectors - Attach clocks to the peripheries */
        CLOCK_AttachClk(kFRO12M_to_MAIN_CLK);                  /*!< Switch MAIN_CLK to FRO12M */
        /*!< Set SystemCoreClock variable. */
        SystemCoreClock = 12000000;
    }

#endif // BL_TARGET_FLASH
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
