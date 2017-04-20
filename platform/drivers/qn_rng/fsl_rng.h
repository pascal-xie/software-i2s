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

#ifndef _FSL_RNG_H_
#define _FSL_RNG_H_

#include "fsl_common.h"

/*!
 * @addtogroup qn_rng
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief QN RNG driver version 2.0.0. */
#define FSL_RNG_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * @brief List of RNG flags
 */
enum _rng_status_flags
{
    kRNG_BusyFlag = RNG_STAT_BUSY_MASK, /*!< RNG busy flag */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization
 * @{
 */

/*!
 * @brief Initializes the RNG.
 *
 * This function initializes the RNG.
 * @param base RNG base address
 */
void RNG_Init(RNG_Type *base);

/*!
 * @brief Deinitialize the RNG module.
 *
 * @param base RNG base address
 */
void RNG_Deinit(RNG_Type *base);

/*!
 * @brief Enable/Disable RNG.
 *
 * @param base RNG peripheral base address.
 * @param enable true to enable the RNG, false to disable.
 */
static inline void RNG_Enable(RNG_Type *base, bool enable)
{
    if (enable)
    {
        base->CTRL |= RNG_CTRL_ENABLE_MASK;
    }
    else
    {
        base->CTRL &= ~RNG_CTRL_ENABLE_MASK;
    }
}

/*! @}*/

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enable RNG interrupt.
 *
 * @param base RNG peripheral base address.
 */
static inline void RNG_EnableInterrupt(RNG_Type *base)
{
    base->INTEN |= RNG_INTEN_DONE_INTEN_MASK;
}

/*!
 * @brief Disable RNG interrupt.
 *
 * @param base RNG peripheral base address.
 */
static inline void RNG_DisableInterrupt(RNG_Type *base)
{
    base->INTEN &= ~RNG_INTEN_DONE_INTEN_MASK;
}

/*!
 * @brief Clear RNG interrupt flag.
 *
 * @param base RNG peripheral base address.
 */
static inline void RNG_ClearInterruptFlag(RNG_Type *base)
{
    base->INT = RNG_INT_DONE_MASK;
}

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Get RNG status flag
 *
 * @param base RNG peripheral base address.
 *
 * @return RNG status
 */
static inline uint32_t RNG_GetStatusFlags(RNG_Type *base)
{
    return base->STAT;
}

/*! @}*/

/*!
 * @name Get Random Number Interface
 * @{
 */

/*!
 * @brief Start random number generation.
 *
 * @param base RNG peripheral base address.
 */
static inline void RNG_Start(RNG_Type *base)
{
    base->CTRL |= RNG_CTRL_START_MASK;
}

/*!
 * @brief Get random number
 *
 * @param base RNG peripheral base address.
 *
 * @return random number
 */
static inline uint32_t RNG_GetRandomNumber(RNG_Type *base)
{
    return base->DATA;
}

/*!
 * @brief Gets random data.
 *
 * This function gets random data from the RNG.
 *
 * @param base  RNG base address
 * @param data  Pointer address used to store random data
 * @param dataSize  Size of the buffer pointed by the data parameter
 * @return random data
 */
status_t RNG_GetRandomData(RNG_Type *base, uint8_t *data, size_t dataSize);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_RNG_H_ */
