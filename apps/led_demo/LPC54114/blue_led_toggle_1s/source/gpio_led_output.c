/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_BOARD_TEST_GPIO_PORT1 BOARD_LED_GREEN_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT2 BOARD_LED_RED_GPIO_PORT
#define APP_BOARD_TEST_GPIO_PORT3 BOARD_LED_BLUE_GPIO_PORT
#define APP_BOARD_TEST_LED1_PIN BOARD_LED_GREEN_GPIO_PIN
#define APP_BOARD_TEST_LED2_PIN BOARD_LED_RED_GPIO_PIN
#define APP_BOARD_TEST_LED3_PIN BOARD_LED_BLUE_GPIO_PIN
#define APP_SW1_PORT BOARD_SW1_GPIO_PORT
#define APP_SW2_PORT BOARD_SW2_GPIO_PORT
#define APP_SW1_PIN BOARD_SW1_GPIO_PIN
#define APP_SW2_PIN BOARD_SW2_GPIO_PIN


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */
void delay(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 2000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t port_state = 0;

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Board pin, clock, debug console init */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    /* enable clock for GPIO*/
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    BOARD_InitPins();
    BOARD_BootClockFROHF48M();
    BOARD_InitDebugConsole();

    /* Print a note to terminal. */
    PRINTF("\r\n GPIO Driver example\r\n");
    PRINTF("\r\n The LED is taking turns to shine.\r\n");

    /* Init output LED GPIO. */
    GPIO_PinInit(GPIO, APP_BOARD_TEST_GPIO_PORT1, APP_BOARD_TEST_LED1_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, APP_BOARD_TEST_GPIO_PORT1, APP_BOARD_TEST_LED1_PIN, 1);

    GPIO_PinInit(GPIO, APP_BOARD_TEST_GPIO_PORT2, APP_BOARD_TEST_LED2_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, APP_BOARD_TEST_GPIO_PORT2, APP_BOARD_TEST_LED2_PIN, 1);

    GPIO_PinInit(GPIO, APP_BOARD_TEST_GPIO_PORT3, APP_BOARD_TEST_LED3_PIN, &led_config);
    GPIO_WritePinOutput(GPIO, APP_BOARD_TEST_GPIO_PORT3, APP_BOARD_TEST_LED3_PIN, 1);

    GPIO_ClearPinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN | 1 << APP_BOARD_TEST_LED3_PIN);
    GPIO_SetPinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN | 1 << APP_BOARD_TEST_LED3_PIN);

    GPIO_ClearPinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED3_PIN);
    GPIO_SetPinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED3_PIN);

    GPIO_TogglePinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN | 1 << APP_BOARD_TEST_LED3_PIN);
    GPIO_TogglePinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN | 1 << APP_BOARD_TEST_LED3_PIN);

    GPIO_TogglePinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN);
    GPIO_TogglePinsOutput(GPIO, 1, 1 << APP_BOARD_TEST_LED1_PIN);

    GPIO_TogglePinsOutput(GPIO, APP_BOARD_TEST_GPIO_PORT2, 1 << APP_BOARD_TEST_LED2_PIN);
    GPIO_TogglePinsOutput(GPIO, APP_BOARD_TEST_GPIO_PORT2, 1 << APP_BOARD_TEST_LED2_PIN);

    /* Port masking */
    GPIO_SetPortMask(GPIO, APP_BOARD_TEST_GPIO_PORT2, 0x0000ffff);
    GPIO_WriteMPort(GPIO, APP_BOARD_TEST_GPIO_PORT2, 0xffffffff);
    port_state = GPIO_ReadPinsInput(GPIO, 0);
    PRINTF("\r\n Standard port read: %x\r\n", port_state);
    port_state = GPIO_ReadMPort(GPIO, 0);
    PRINTF("\r\n Masked port read: %x\r\n", port_state);

    while (1)
    {
        PRINTF("\r\n User App Running: %x\r\n", port_state++);
        GPIO_TogglePinsOutput(GPIO, APP_BOARD_TEST_GPIO_PORT1, 1u << APP_BOARD_TEST_LED1_PIN);
        delay();
    }
}
