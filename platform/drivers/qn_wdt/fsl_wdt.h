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

#ifndef _FSL_WDT_H_
#define _FSL_WDT_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief WDT driver version */
#define FSL_WDT_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief Describes WDT configuration structure. */
typedef struct _wdt_config
{
    bool enableWdtReset; /*!< true: Watchdog timeout will cause a chip reset
                                   false: Watchdog timeout will not cause a chip reset */
    uint32_t loadValue;  /*!< Load value, default value is 0xFFFFFFFF */
} wdt_config_t;

/*******************************************************************************
* API
******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
/*!
 * @brief Initializes the WDT with configuration.
 *
 * This function initializes the WDT.
 *
 * @param base WDT peripheral base address.
 * @param config pointer to configuration structure
 */
void WDT_Init(WDT_Type *base, const wdt_config_t *config);

/*!
 * @brief Disable the WDT peripheral.
 *
 * This function shuts down the WDT.
 *
 * @param base WDT peripheral base address.
 */
void WDT_Deinit(WDT_Type *base);

/*!
 * @brief  Unlock WDT access
 *
 * This function unlock WDT access.
 *
 * @param base WDT peripheral base address.
 */
static inline void WDT_Unlock(WDT_Type *base)
{
    base->LOCK = 0x1ACCE551;
}

/*!
 * @brief  Lock WDT access
 *
 * This function unlock WDT access.
 *
 * @param base WDT peripheral base address.
 */
static inline void WDT_Lock(WDT_Type *base)
{
    base->LOCK = 0;
}

/*!
 * @brief Clears status flags.
 *
 * This function clears WDT status flag.
 *
 * @param base WDT peripheral base address.
 */
static inline void WDT_ClearStatusFlags(WDT_Type *base)
{
    WDT_Unlock(base);
    base->INT_CLR = WDT_INT_CLR_INTCLR_MASK;
    WDT_Lock(base);
}

/*!
 * @brief Initializes WDT configure sturcture
 *
 * This function initializes the WDT configure structure to default value. The default
 * value are:
 * @code
 *   config->enableWdtReset = true;
 *   config->loadValue = 0xffffffff;
 * @endcode
 * @param config pointer to WDT config structure
 */
void WDT_GetDefaultConfig(wdt_config_t *config);

/*!
 * @brief Refresh WDT counter
 *
 * This function feeds the WDT.
 * This function should be called before WDT timer is in timeout.
 *
 * @param base WDT peripheral base address.
 * @param  cycle  time-out interval
 */
static inline void WDT_Refresh(WDT_Type *base, uint32_t cycle)
{
    WDT_Unlock(base);
    base->LOAD = cycle;
    WDT_Lock(base);
}
#ifdef __cplusplus
}
#endif

/*!
 * @}
 */

#endif /* _FSL_WDT_H_*/
