/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#include "bootloader/bl_context.h"
#include "bootloader/bl_irq_common.h"
#include "bootloader_common.h"
#include "autobaud/autobaud.h"
#include "packet/serial_packet.h"
#include "fsl_device_registers.h"
#include "flexcommusart/fsl_usart.h"
#include "utilities/fsl_assert.h"

#if (BL_CONFIG_UART)

//! @addtogroup usart_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#define USART_TYPE_REGS_BASE(x) ((USART_Type *)REGS_USART_BASE(x))

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static USART_Type *get_usart_baseAddr(uint32_t instance);
static bool usart_poll_for_activity(const peripheral_descriptor_t *self);
static status_t usart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void usart_full_shutdown(const peripheral_descriptor_t *self);

static status_t usart_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_usartControlInterface = {
    .pollForActivity = usart_poll_for_activity, .init = usart_full_init, .shutdown = usart_full_shutdown, .pump = 0
};

const peripheral_byte_inteface_t g_usartByteInterface = {.init = NULL, .write = usart_write };

static serial_byte_receive_func_t s_usart_byte_receive_callback;
static bool g_usartInitDone = false;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
static USART_Type *get_usart_baseAddr(uint32_t instance)
{
    switch (instance)
    {
        default:
        case 0:
            return (USART_Type *)USART0_BASE;
        case 1:
            return USART1;
#if (FSL_FEATURE_SOC_USART_COUNT > 2U)
        case 2:
            return USART2;
#if (FSL_FEATURE_SOC_USART_COUNT > 3U)
        case 3:
            return USART3;
#if (FSL_FEATURE_SOC_USART_COUNT > 4U)
        case 4:
            return USART4;
#if (FSL_FEATURE_SOC_USART_COUNT > 5U)
        case 5:
            return USART5;
#if (FSL_FEATURE_SOC_USART_COUNT > 6U)
        case 6:
            return USART6;
#if (FSL_FEATURE_SOC_USART_COUNT > 7U)
        case 7:
            return USART7;
#endif // (FSL_FEATURE_SOC_USART_COUNT > 7U)
#endif // (FSL_FEATURE_SOC_USART_COUNT > 6U)
#endif // (FSL_FEATURE_SOC_USART_COUNT > 5U)
#endif // (FSL_FEATURE_SOC_USART_COUNT > 4U)
#endif // (FSL_FEATURE_SOC_USART_COUNT > 3U)
#endif // (FSL_FEATURE_SOC_USART_COUNT > 2U)
    }
}

bool usart_poll_for_activity(const peripheral_descriptor_t *self)
{
    uint32_t baud;
    status_t autoBaudCompleted = autobaud_get_rate(self->instance, &baud);
    if (autoBaudCompleted == kStatus_Success)
    {
        usart_config_t userConfig;
        USART_Type *base = get_usart_baseAddr(self->instance);
        USART_GetDefaultConfig(&userConfig);

        userConfig.baudRate_Bps = baud;
        userConfig.enableTx = true;
        userConfig.enableRx = true;

        if (USART_Init(base, &userConfig, get_uart_clock(self->instance)) == kStatus_Success)
        {
            UART_SetSystemIRQ(self->instance, kPeripheralEnableIRQ);
            /* Enable RX interrupt. */
            USART_EnableInterrupts(base, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);
            
            // Configure selected pin as usart peripheral interface
            self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);

            // This was the byte pattern identified in autobaud detection, inform the command layer
            s_usart_byte_receive_callback(kFramingPacketStartByte);
            s_usart_byte_receive_callback(kFramingPacketType_Ping);

            g_usartInitDone = true;

            return true;
        }
        else
        {
            autobaud_init(self->instance);
        }
    }

    return false;
}

status_t usart_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    s_usart_byte_receive_callback = function;

    // Since we are using autobaud once the detection is completed
    // it will call the USART initialization and remux the pins when it completes
    self->pinmuxConfig(self->instance, kPinmuxType_GPIO);

    // Init autobaud detector.
    autobaud_init(self->instance);

    return kStatus_Success;
}

void usart_full_shutdown(const peripheral_descriptor_t *self)
{
    if (g_usartInitDone)
    {
        UART_SetSystemIRQ(self->instance, kPeripheralDisableIRQ);
        USART_Deinit(get_usart_baseAddr(self->instance));
    }

//! Note: if not deinit autobaud(IRQ method), user app may encounters hardfault
//! if it doesn't provide related pin interrupt service routine.
#if BL_USART_AUTOBAUD_IRQ
    // De-init autobaud detector.
    autobaud_deinit(self->instance);
#endif
    // Restore selected pin to default state to reduce IDD.
    self->pinmuxConfig(self->instance, kPinmuxType_Default);
}

status_t usart_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount)
{
    USART_WriteBlocking(get_usart_baseAddr(self->instance), buffer, byteCount);
    return kStatus_Success;
}

/********************************************************************/
/*
 * USART0 IRQ Handler
 *
 */
void FLEXCOMM0_IRQHandler(void)
{
    uint8_t data;
    uint32_t status = USART_GetStatusFlags(USART0);
    /* If new data arrived. */
    if (kUSART_RxFifoNotEmptyFlag & status)
    {
        data = USART_ReadByte(USART0);
        s_usart_byte_receive_callback(data);
    }
    if (kUSART_RxError & status)
        USART_ClearStatusFlags(USART0,kUSART_RxError);
}

/********************************************************************/
/*
 * USART1 IRQ Handler
 *
 */
void FLEXCOMM1_IRQHandler(void)
{
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(USART1))
    {
        s_usart_byte_receive_callback(USART_ReadByte(USART1));
    }
}

//! @}

#endif // (BL_CONFIG_USART)

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
