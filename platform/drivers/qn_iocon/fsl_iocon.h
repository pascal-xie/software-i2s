/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef _FSL_IOCON_H_
#define _FSL_IOCON_H_

#include "fsl_common.h"

/*!
 * @addtogroup lpc_iocon
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IOCON SYSCON

/*! @name Driver version */
/*@{*/
/*! @brief IOCON driver version 2.0.0. */
#define LPC_IOCON_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/**
 * @brief Array of IOCON pin definitions passed to IOCON_SetPinMuxing() must be in this format
 */
typedef struct _iocon_group
{
    uint32_t port : 8;      /* Pin port */
    uint32_t pin : 8;       /* Pin number */
    uint32_t modefunc : 16; /* Function and mode */
} iocon_group_t;

/**
 * @brief IOCON function, mode and drive selection definitions
 * @note See the User Manual for specific drive levels, modes and functions supported by the various pins.
 */
#define IOCON_FUNC0 0x0 /*!< Selects pin function 0 */
#define IOCON_FUNC1 0x1 /*!< Selects pin function 1 */
#define IOCON_FUNC2 0x2 /*!< Selects pin function 2 */
#define IOCON_FUNC3 0x3 /*!< Selects pin function 3 */
#define IOCON_FUNC4 0x4 /*!< Selects pin function 4 */
#define IOCON_FUNC5 0x5 /*!< Selects pin function 5 */
#define IOCON_FUNC6 0x6 /*!< Selects pin function 6 */
#define IOCON_FUNC7 0x7 /*!< Selects pin function 7 */

#define IOCON_MODE_HIGHZ (0x0 << 4)    /*!< Selects High-Z function */
#define IOCON_MODE_PULLDOWN (0x1 << 4) /*!< Selects pull-down function */
#define IOCON_MODE_PULLUP (0x2 << 4)   /*!< Selects pull-up function */

#define IOCON_DRIVE_LOW (0x0 << 6)   /*!< Enable low drive strength */
#define IOCON_DRIVE_HIGH (0x1 << 6)  /*!< Enable high drive strength */
#define IOCON_DRIVE_EXTRA (0x1 << 7) /*!< Enable extra drive, only valid for PA06/PA11/PA19/PA26/PA27 */

/**
 * @brief Pull mode
 */
typedef enum _iocon_pull_mode
{
    kIOCON_HighZ = 0U, /*!< High Z */
    kIOCON_PullDown,   /*!< Pull down */
    kIOCON_PullUp      /*!< Pull up */
} iocon_pull_mode_t;

/**
 * @brief Drive strength
 */
typedef enum _iocon_drive_strength
{
    kIOCON_LowDriveStrength = 0U,      /*!< Low-drive */
    kIOCON_HighDriveStrength,          /*!< High-drive */
    kIOCON_LowDriveWithExtraStrength,  /*!< Low-drive with extra */
    kIOCON_HighDriveWithExtraStrength, /*!< High-drive with extra */
} iocon_drive_strength_t;

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * @brief   Sets I/O control pin mux
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   port            GPIO port to mux (value from 0 ~ 1)
 * @param   pin             GPIO pin to mux (value from 0 ~ 31)
 * @param   modeFunc        OR'ed values of type IOCON_*
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_PinMuxSet(SYSCON_Type *base, uint8_t port, uint8_t pin, uint32_t modeFunc)
{
    assert(((port == 0) && (pin <= 31)) || ((port == 1) && (pin <= 2)));

    uint8_t pinMuxIndex = (pin >> 3);
    uint8_t pinMuxLocation = ((pin & 0x7) << 2);
    uint8_t pinPullIndex = (pin >> 4);
    uint8_t pinPullLocation = ((pin & 0xF) << 1);

    if (port == 0)
    {
        base->PIO_FUNC_CFG[pinMuxIndex] &= ~(0x07 << pinMuxLocation);
        base->PIO_FUNC_CFG[pinMuxIndex] |= (modeFunc & 0x07) << pinMuxLocation;

        pinPullIndex = (pin >> 4);
    }
    else if (port == 1)
    {
        pinPullIndex = 2;

        if ((pin == 0) || (pin == 1))
        {
            base->PIO_CFG_MISC &= ~(1 << pin);
            base->PIO_CFG_MISC |= (modeFunc & 0x01) << pin;
        }
        else if (pin == 2)
        {
            base->PIO_CFG_MISC &= ~(1 << 16);
            base->PIO_CFG_MISC |= (modeFunc & 0x01) << 16;
        }
    }

    base->PIO_PULL_CFG[pinPullIndex] &= ~(0x03 << pinPullLocation);
    base->PIO_PULL_CFG[pinPullIndex] |= ((modeFunc >> 4) & 0x03) << pinPullLocation;

    base->PIO_DRV_CFG[port] &= ~(1 << pin);
    base->PIO_DRV_CFG[port] |= (((modeFunc >> 6) & 0x01)) << pin;

    if ((port == 0) && ((pin == 6) || (pin == 11) || (pin == 19) || (pin == 26) || (pin == 27)))
    {
        base->PIO_DRV_CFG[2] &= ~(1 << pin);
        base->PIO_DRV_CFG[2] |= (((modeFunc >> 7) & 0x01)) << pin;
    }
}

/**
 * @brief   Set all I/O control pin muxing
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   pinArray        Pointer to array of pin mux selections
 * @param   arrayLength     Number of entries in pinArray
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_SetPinMuxing(SYSCON_Type *base, const iocon_group_t *pinArray, uint32_t arrayLength)
{
    uint32_t i;

    for (i = 0; i < arrayLength; i++)
    {
        IOCON_PinMuxSet(base, pinArray[i].port, pinArray[i].pin, pinArray[i].modefunc);
    }
}

/**
 * @brief   Sets I/O control pin function
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   port            GPIO port (value from 0 ~ 1)
 * @param   pin             GPIO pin (value from 0 ~ 31)
 * @param   func            Pin fucntion (value from 0 ~ 7)
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_FuncSet(SYSCON_Type *base, uint8_t port, uint8_t pin, uint8_t func)
{
    assert(((port == 0) && (pin <= 31)) || ((port == 1) && (pin <= 2)));
    uint8_t index = (pin >> 3);
    uint8_t pinLocation = ((pin & 0x7) << 2);

    if (port == 0)
    {
        base->PIO_FUNC_CFG[index] &= ~(0x07 << pinLocation);
        base->PIO_FUNC_CFG[index] |= (func & 0x07) << pinLocation;
    }
    else if (port == 1)
    {
        if ((pin == 0) || (pin == 1))
        {
            base->PIO_CFG_MISC &= ~(1 << pin);
            base->PIO_CFG_MISC |= (func & 0x01) << pin;
        }
        else if (pin == 2)
        {
            base->PIO_CFG_MISC &= ~(1 << 16);
            base->PIO_CFG_MISC |= (func & 0x01) << 16;
        }
    }
}

/**
 * @brief   Sets I/O control drive capability
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   port            GPIO port (value from 0 ~ 1)
 * @param   pin             GPIO pin (value from 0 ~ 31)
 * @param   strength        Drive strength (Extra option is only valid for PA06/PA11/PA19/PA26/PA27)
 *        - kIOCON_LowDriveStrength = 0U - Low-drive strength is configured.
 *        - kIOCON_HighDriveStrength = 1U - High-drive strength is configured
 *        - kIOCON_LowDriveWithExtraStrength = 2U - Low-drive with extra strength is configured
 *        - kIOCON_HighDriveWithExtraStrength = 3U - High-drive with extra strength is configured
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_DriveSet(SYSCON_Type *base, uint8_t port, uint8_t pin, uint8_t strength)
{
    assert(((port == 0) && (pin <= 31)) || ((port == 1) && (pin <= 2)));
    base->PIO_DRV_CFG[port] &= ~(1 << pin);
    base->PIO_DRV_CFG[port] |= (strength & 0x01) << pin;
    if ((port == 0) && ((pin == 6) || (pin == 11) || (pin == 19) || (pin == 26) || (pin == 27)))
    {
        base->PIO_DRV_CFG[2] &= ~(1 << pin);
        base->PIO_DRV_CFG[2] |= (((strength >> 1) & 0x01)) << pin;
    }
}

/**
 * @brief   Sets I/O control pull configuration
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   port            GPIO port (value from 0 ~ 1)
 * @param   pin             GPIO pin (value from 0 ~ 31)
 * @param   pullMode        Pull mode
 *        - kIOCON_HighZ = 0U - High Z is configured.
 *        - kIOCON_PullDown = 1U - Pull-down is configured
 *        - kIOCON_PullUp = 2U - Pull-up is configured
 * @return  Nothing
 */
__STATIC_INLINE void IOCON_PullSet(SYSCON_Type *base, uint8_t port, uint8_t pin, uint8_t pullMode)
{
    assert(((port == 0) && (pin <= 31)) || ((port == 1) && (pin <= 2)));
    uint32_t index = (port == 1) ? 2 : (pin >> 4);
    uint32_t pinLocation = ((pin & 0xF) << 1);

    base->PIO_PULL_CFG[index] &= ~(0x03 << pinLocation);
    base->PIO_PULL_CFG[index] |= pullMode << pinLocation;
}

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_IOCON_H_ */
