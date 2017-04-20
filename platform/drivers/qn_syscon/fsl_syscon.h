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

#ifndef _FSL_SYSCON_H_
#define _FSL_SYSCON_H_

#include "fsl_common.h"

/*!
 * @addtogroup lpc_syscon
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief SYSCON driver version 2.0.0. */
#define LPC_SYSCON_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * @brief   Sets load cap for on bard crystal
 * @param   base            The base of SYSCON peripheral on the chip
 * @param   xtalType        XTAL type (1 : 16/32MHz, 0: 32KHz)
 * @param   loadCap         load cap value
 * @return  Nothing
 */
__STATIC_INLINE void SYSCON_SetLoadCap(SYSCON_Type *base, uint8_t xtalType, uint8_t loadCap)
{
    if (xtalType)
    {
        base->ANA_CTRL0 = (SYSCON->ANA_CTRL0 &
                           ~(SYSCON_ANA_CTRL0_XTAL_EXTRA_CAP_MASK | SYSCON_ANA_CTRL0_XTAL_LOAD_CAP_MASK |
                             SYSCON_ANA_CTRL0_XTAL_AMP_MASK)) |
                          ((loadCap & 0x7F) << SYSCON_ANA_CTRL0_XTAL_LOAD_CAP_SHIFT) | SYSCON_ANA_CTRL0_XTAL_AMP(0x2);
    }
    else
    {
        base->XTAL32K_CTRL =
            (SYSCON->XTAL32K_CTRL &
             ~(SYSCON_XTAL32K_CTRL_XTAL32K_EXTRA_CAP_MASK | SYSCON_XTAL32K_CTRL_XTAL32K_LOAD_CAP_MASK)) |
            ((loadCap & 0x7F) << SYSCON_XTAL32K_CTRL_XTAL32K_LOAD_CAP_SHIFT);
    }
}

/* @} */

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_SYSCON_H_ */
