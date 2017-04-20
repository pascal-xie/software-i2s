/*
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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

#include "button.h"
#if defined(CFG_BLE_PRJ)
#include "app_ble.h"
#endif
#ifdef MBED_PORT
extern void button2_pressed(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

static tm_timer_id_t s_buttonTimerID[BOARD_SW_NUM];

static const uint32_t s_buttonGpioPin[BOARD_SW_NUM] = {BOARD_SW1_GPIO_PIN, BOARD_SW2_GPIO_PIN};
static const uint32_t s_buttonGpioPinMask[BOARD_SW_NUM] = {BOARD_SW1_GPIO_PIN_MASK, BOARD_SW2_GPIO_PIN_MASK};

/*******************************************************************************
 * Code
 ******************************************************************************/

void BUTTON_Init(void)
{
    for (int i = 0; i < BOARD_SW_NUM; i++)
    {
        GPIO_PinInit(BOARD_SW_GPIO, s_buttonGpioPin[i], &(gpio_pin_config_t){kGPIO_DigitalInput, (0U)});
        s_buttonTimerID[i] = TM_AllocateTimer();
    }

    BUTTON_EnableInterrupt();
}

void BUTTON_EnableInterrupt(void)
{
    uint32_t swMask = 0;
    for (int i = 0; i < BOARD_SW_NUM; i++)
    {
        swMask |= s_buttonGpioPinMask[i];
    }
    GPIO_SetFallingEdgeInterrupt(BOARD_SW_GPIO, swMask);
    GPIO_EnableInterrupt(BOARD_SW_GPIO, swMask);

    NVIC_EnableIRQ(GPIOA_IRQn);
}

void BUTTON_SetGpioWakeupLevel(GPIO_Type *base, uint32_t pin, uint8_t level)
{
    if (base == GPIOA)
    {
        // configure gpio wakeup level
        if (level == 1U)
        {
            SYSCON->PIO_WAKEUP_LVL0 = SYSCON->PIO_WAKEUP_LVL0 & ~(1U << pin);
        }
        else
        {
            SYSCON->PIO_WAKEUP_LVL0 = SYSCON->PIO_WAKEUP_LVL0 | (1U << pin);
        }
        // enable gpio wakeup pin
        SYSCON->PIO_WAKEUP_EN0 = SYSCON->PIO_WAKEUP_EN0 | (1U << pin);
    }
    else
    {
        // configure gpio wakeup level
        if (level == 1U)
        {
            SYSCON->PIO_WAKEUP_LVL1 = SYSCON->PIO_WAKEUP_LVL1 & ~(1U << pin);
        }
        else
        {
            SYSCON->PIO_WAKEUP_LVL1 = SYSCON->PIO_WAKEUP_LVL1 | (1U << pin);
        }
        // enable gpio wakeup pin
        SYSCON->PIO_WAKEUP_EN1 = SYSCON->PIO_WAKEUP_EN1 | (1U << pin);
    }
}

/*!
 * @brief Handles the Button debounce timer.
 */
static void BUTTON_DebounceHandler(uint32_t pin_mask)
{
    // check the button is still pressed, low level expected.
    if (!(BOARD_SW_GPIO->DATA & pin_mask))
    {
#ifdef MBED_PORT
        if (pin_mask == BOARD_SW2_GPIO_PIN_MASK)
        {
            button2_pressed();
        }
#else
#if defined(CFG_BLE_PRJ)
        // send the button down message to APP task
        struct app_button_down_cmd *cmd = KE_MSG_ALLOC(APP_MSG_BUTTON_DOWN, TASK_APP, TASK_APP, app_button_down_cmd);

        cmd->pin = pin_mask;

        APP_MsgSend(cmd);
#endif
#endif
        // or add user code here to process the button down
    }
}

void GPIOA_IRQHandler(void)
{
    volatile uint32_t int_state;

    /* Get interrupt state*/
    int_state = GPIOA->INTSTATUS;
    /* Clear interrupt flag */
    GPIOA->INTSTATUS = int_state;

    for (int i = 0; i < BOARD_SW_NUM; i++)
    {
        if (int_state & s_buttonGpioPinMask[i])
        {
            TM_SetTimer(s_buttonTimerID[i], kTM_SingleShotTimer, BUTTON_CHK_DLY, (tm_callback_t)BUTTON_DebounceHandler,
                        (void *)(s_buttonGpioPinMask[i]));
        }
    }
}
