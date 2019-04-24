/*
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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
#include "fsl_device_registers.h"
#include "flexcommusart/fsl_usart.h"
#include "drivers/systick/systick.h"
#include "drivers/watchdog/fsl_watchdog.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "smc/smc.h"

#include "fsl_clock.h"  //add by damon.zhang
#include "fsl_reset.h"
#include "fsl_power.h"
#include "fsl_iocon.h"
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define BOOT_PIN_NUMBER 3
#define BOOT_PIN_PORT PORTC
#define BOOT_PIN_GPIO GPIOC
#define BOOT_PIN_ALT_MODE 1
#define BOOT_PIN_DEBOUNCE_READ_COUNT 500

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void init_hardware(void)
{
    //exit_vlpr();  //exit low power mode
    CLOCK_EnableClock(kCLOCK_Iocon);
    //Enable GPIO0~3 Clock
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);
    CLOCK_EnableClock(kCLOCK_Gpio2);
    CLOCK_EnableClock(kCLOCK_Gpio3);

    // /* Enable FLEXCOMM 0 Clk */
    // CLOCK_EnableClock(kCLOCK_FlexComm0);
     /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
     CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
     /* Reset Flexcomm 0 */
     RESET_PeripheralReset(kFC0_RST_SHIFT_RSTn);

#if DEBUG
//    // Enable the pins for the debug USART1
//    PORTC->PCR[3] = PORT_PCR_MUX(3); // USART1_RX is PTC3 in ALT3
//    PORTC->PCR[4] = PORT_PCR_MUX(3); // USART1_TX is PTC4 in ALT3
#endif
}

void deinit_hardware(void)
{
    //Enable GPIO0~3 Clock
    CLOCK_DisableClock(kCLOCK_Gpio0);
    CLOCK_DisableClock(kCLOCK_Gpio1);
    CLOCK_DisableClock(kCLOCK_Gpio2);
    CLOCK_DisableClock(kCLOCK_Gpio3);
}

uint32_t get_bus_clock(void)
{
    /*! @brief	Return Frequency of Core System
     *  @return	Frequency of Core System
     */
    return CLOCK_GetCoreSysClkFreq();
}

// Keep this function here to ensure compatibility, all usb related configuration
// is maintained by usb stack itself.
bool usb_clock_init(void)
{
    const uint32_t port1_pin6_config = (
    IOCON_FUNC7 |                                        /* Pin is configured as USB0_VBUS */
    IOCON_MODE_INACT |                                   /* No addition pin function */
    IOCON_DIGITAL_EN |                                   /* Enables digital function */
    IOCON_INPFILT_OFF |                                  /* Input filter disabled */
    IOCON_OPENDRAIN_EN                                   /* Open drain is disabled */
  );
    IOCON_PinMuxSet(IOCON, 1, 6, port1_pin6_config); /* PORT1 PIN6 (coords: 26) is configured as USB0_VBUS */
    POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*Turn on USB Phy */
    /* enable USB IP clock */
    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcFro, CLOCK_GetFreq(kCLOCK_FroHf));
    return true;
}

// Keep this function here to ensure compatibility, all usb related configuration
// is maintained by usb stack itself.
bool usb_clock_deinit(void)
{
    /* disable USB IP clock */
    CLOCK_DisableClock(kCLOCK_Usbd0);
    return true;
}


uint32_t get_uart_clock(uint32_t instance)
{
    return CLOCK_GetFlexCommClkFreq(instance);
}

bool is_boot_pin_asserted(void)
{
#if defined(BL_TARGET_FLASH) & !defined(FREEDOM)
//    // Initialize boot pin for GPIO
//    BOOT_PIN_PORT->PCR[BOOT_PIN_NUMBER] |= PORT_PCR_MUX(BOOT_PIN_ALT_MODE);

//    // Set boot pin as an input
//    BOOT_PIN_GPIO->PDDR &= (uint32_t) ~(1 << BOOT_PIN_NUMBER);
//    // Set boot pin pullup enabled, pullup select, filter enable
//    BOOT_PIN_PORT->PCR[BOOT_PIN_NUMBER] |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_PFE_MASK);

//    uint32_t readCount = 0;

//    // Sample the pin a number of times
//    for (uint32_t i = 0; i < BOOT_PIN_DEBOUNCE_READ_COUNT; i++)
//    {
//        readCount += ((BOOT_PIN_GPIO->PDIR) >> BOOT_PIN_NUMBER) & 1;
//    }

//    // boot pin is pulled high so we are measuring lows, make sure most of our measurements
//    // registered as low
//    return (readCount < (BOOT_PIN_DEBOUNCE_READ_COUNT / 2));
    return false;
#else
    // Boot pin for Flash only target
    return false;
#endif
}

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void update_available_peripherals()
{
}

// @brief Initialize watchdog
void bootloader_watchdog_init(void)
{
//    systick_init(SystemCoreClock / 64);
//    systick_set_hook(bootloader_watchdog_service);
}

void bootloader_watchdog_service(void)
{
//    lock_acquire();
//    fsl_watchdog_service();
//    lock_release();
}

void bootloader_watchdog_deinit(void)
{
//    systick_shutdown();
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
