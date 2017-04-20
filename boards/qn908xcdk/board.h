/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Configure the board type: DK or EVB */
#define CFG_BOARD_TYPE BOARD_QN908X_DK

#define BOARD_QN908X_DK 1U
#define BOARD_QN908X_EVB 0U

#ifdef NXP_BLE_STACK
#define MANUFACTURER_NAME "NXP"
#endif

/*! @brief The board name: "QN908XDK" or "QN908XEVB" */
#if ((defined(CFG_BOARD_TYPE)) && (CFG_BOARD_TYPE == BOARD_QN908X_DK))
#define BOARD_NAME "QN908XDK"
#elif((defined(CFG_BOARD_TYPE)) && (CFG_BOARD_TYPE == BOARD_QN908X_EVB))
#define BOARD_NAME "QN908XEVB"
#endif

/*! @brief The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) USART0
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

#if ((defined(CFG_BOARD_TYPE)) && (CFG_BOARD_TYPE == BOARD_QN908X_DK))

/* Board leds */
#define BOARD_LED_GPIO GPIOA
#define BOARD_LED_RED_GPIO_PIN 31U
#define BOARD_LED_GREEN_GPIO_PIN 25U
#define BOARD_LED_BLUE_GPIO_PIN 13U
#define BOARD_LED_NUM 3

#define BOARD_LED_RED_GPIO_PIN_MASK (1U << BOARD_LED_RED_GPIO_PIN)
#define BOARD_LED_GREEN_GPIO_PIN_MASK (1U << BOARD_LED_GREEN_GPIO_PIN)
#define BOARD_LED_BLUE_GPIO_PIN_MASK (1U << BOARD_LED_BLUE_GPIO_PIN)

/* Board buttons */
#define BOARD_SW_GPIO GPIOA
#define BOARD_SW1_GPIO_PIN 24U
#define BOARD_SW1_NAME "SW1"
#define BOARD_SW2_GPIO_PIN 19U
#define BOARD_SW2_NAME "SW2"
#define BOARD_SW_NUM 2U

/* Board led color mapping */
#define LOGIC_LED_ON 1U
#define LOGIC_LED_OFF 0U

#define LED_RED_INIT(output)                             \
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_RED_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})                        /*!< Enable LED_RED */
#define LED_RED_ON() GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN)        /*!< Turn off LED_RED */
#define LED_RED_OFF() GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN)     /*!< Turn on LED_RED */
#define LED_RED_TOGGLE() GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Toggle on LED_RED */

#define LED_GREEN_INIT(output)                             \
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GREEN_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})                        /*!< Enable LED_GREEN */
#define LED_GREEN_ON() GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN)    /*!< Turn off LED_GREEN */
#define LED_GREEN_OFF() GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn on LED_GREEN */
#define LED_GREEN_TOGGLE() \
    GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Toggle on LED_GREEN */

#define LED_BLUE_INIT(output)                             \
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_BLUE_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)})                      /*!< Enable LED_BLUE */
#define LED_BLUE_ON() GPIO_SetPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN)    /*!< Turn off LED_BLUE  */
#define LED_BLUE_OFF() GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn on LED_BLUE */
#define LED_BLUE_TOGGLE() \
    GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Toggle on LED_BLUE */

#elif((defined(CFG_BOARD_TYPE)) && (CFG_BOARD_TYPE == BOARD_QN908X_EVB))

/* Board leds */
#define BOARD_LED_GPIO GPIOA
#define BOARD_LED1_GPIO_PIN 10U
#define BOARD_LED2_GPIO_PIN 11U
#define BOARD_LED3_GPIO_PIN 18U
#define BOARD_LED4_GPIO_PIN 19U
#define BOARD_LED_NUM 4U

#define BOARD_LED1_GPIO_PIN_MASK (1U << BOARD_LED1_GPIO_PIN)
#define BOARD_LED2_GPIO_PIN_MASK (1U << BOARD_LED2_GPIO_PIN)
#define BOARD_LED3_GPIO_PIN_MASK (1U << BOARD_LED3_GPIO_PIN)
#define BOARD_LED4_GPIO_PIN_MASK (1U << BOARD_LED4_GPIO_PIN)

/* Board buttons */
#define BOARD_SW_GPIO GPIOA
#define BOARD_SW1_GPIO_PIN 0U
#define BOARD_SW1_NAME "SW1"
#define BOARD_SW2_GPIO_PIN 1U
#define BOARD_SW2_NAME "SW2"
#define BOARD_SW_NUM 2U

#define LOGIC_LED_ON 0U
#define LOGIC_LED_OFF 1U

#define BOARD_LED_RED_GPIO_PIN BOARD_LED1_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN BOARD_LED2_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN BOARD_LED3_GPIO_PIN

#define BOARD_LED_RED_GPIO_PIN_MASK BOARD_LED1_GPIO_PIN_MASK
#define BOARD_LED_GREEN_GPIO_PIN_MASK BOARD_LED2_GPIO_PIN_MASK
#define BOARD_LED_BLUE_GPIO_PIN_MASK BOARD_LED3_GPIO_PIN_MASK

#endif

/* SWD pin will be used for wakeup source */
#define BOARD_SWD_GPIO_PIN 23U

#define BOARD_SW1_GPIO_PIN_MASK (1U << BOARD_SW1_GPIO_PIN)
#define BOARD_SW2_GPIO_PIN_MASK (1U << BOARD_SW2_GPIO_PIN)

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

status_t BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
