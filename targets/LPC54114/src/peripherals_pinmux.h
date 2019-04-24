/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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

#include "fsl_device_registers.h"


#include "fsl_iocon.h"  //add by damon ,modfitied for lpc54114
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! peripheral enable configurations
#define BL_ENABLE_PINMUX_UART0  (BL_CONFIG_UART)
#define BL_ENABLE_PINMUX_SPI0   (BL_CONFIG_SPI)
#define BL_ENABLE_PINMUX_I2C0   (BL_CONFIG_I2C)

//! UART pinmux configurations

//RX
#define UART0_RX_GPIO_PORT_IDX      (0)
#define UART0_RX_GPIO_PIN_IDX       (0)                // PIO0_0

#define UART0_RX_FUNC_ALT_MODE      (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)


#define UART0_RX_GPIO_ALT_MODE      (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)


#define UART0_RX_GPIO_PIN_INT_NUM   kPINT_PinInt0

#define UART0_RX_GPIO_IRQHandler    PIN_INT0_IRQHandler

//TX
#define UART0_TX_GPIO_PORT_IDX      (0)
#define UART0_TX_GPIO_PIN_IDX       (1)                // PIO0_1

#define UART0_TX_FUNC_ALT_MODE      (IOCON_FUNC1 | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF)

#define UART0_TX_GPIO_ALT_MODE      (IOCON_FUNC0 | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN)

// //! I2C pinmux configurations
// #ifdef TOWER
// #define I2C0_SCL_PORT_BASE PORTE
// #define I2C0_SCL_GPIO_PIN_NUM 24             // PTE24
// #define I2C0_SCL_FUNC_ALT_MODE kPORT_MuxAlt5 // ALT mode for I2C1 SCL functionality
// #define I2C0_SDA_PORT_BASE PORTE
// #define I2C0_SDA_GPIO_PIN_NUM 25             // PTE25
// #define I2C0_SDA_FUNC_ALT_MODE kPORT_MuxAlt5 // ALT mode for I2C1 SDA functionality
// #else                                        // FREEDOM
// #define I2C0_SCL_PORT_BASE PORTC
// #define I2C0_SCL_GPIO_PIN_NUM 8              // PTC8
// #define I2C0_SCL_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for I2C1 SCL functionality
// #define I2C0_SDA_PORT_BASE PORTC
// #define I2C0_SDA_GPIO_PIN_NUM 9              // PTC9
// #define I2C0_SDA_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for I2C1 SDA functionality
// #endif

// //! SPI pinmux configurations
// #define SPI0_PCS_PORT_BASE PORTD
// #define SPI0_PCS_GPIO_PIN_NUM 0              // PTD0
// #define SPI0_PCS_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for SPI1 PCS functionality
// #define SPI0_SCK_PORT_BASE PORTD
// #define SPI0_SCK_GPIO_PIN_NUM 1              // PTD1
// #define SPI0_SCK_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for SPI1 SCK functionality
// #define SPI0_SOUT_PORT_BASE PORTD
// #define SPI0_SOUT_GPIO_PIN_NUM 2              // PTD2
// #define SPI0_SOUT_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for SPI1 SOUT functionality
// #define SPI0_SIN_PORT_BASE PORTD
// #define SPI0_SIN_GPIO_PIN_NUM 3              // PTD3
// #define SPI0_SIN_FUNC_ALT_MODE kPORT_MuxAlt2 // ALT mode for SPI1 SIN functionality

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
