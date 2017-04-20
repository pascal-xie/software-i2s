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

/**
 * @addtogroup BOD
 * @{
 */
#include "fsl_bod.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOD_Init(SYSCON_Type *base, const bod_config_t *config)
{
    base->ANA_EN &= ~(SYSCON_ANA_EN_BOD_EN_MASK | SYSCON_ANA_EN_BOD_AMP_EN_MASK | SYSCON_ANA_EN_BOD_THR_MASK |
                      SYSCON_ANA_EN_BOD_THR_MASK);
    base->ANA_EN |= (SYSCON_ANA_EN_BOD_THR(config->int_thr) | SYSCON_ANA_EN_BOR_THR(config->reset_thr));
}
void BOD_Enable(SYSCON_Type *base, uint8_t mode)
{
    volatile uint32_t delayX;
    uint32_t enReg = 0;
    uint32_t ampReg = 0;
    if (mode & kBOD_InterruptEnable)
    {
        ampReg |= SYSCON_ANA_EN_BOD_AMP_EN_MASK;
        enReg |= SYSCON_ANA_EN_BOD_EN_MASK;
    }
    if (mode & kBOD_ResetEnable)
    {
        ampReg |= SYSCON_ANA_EN_BOR_AMP_EN_MASK;
        enReg |= SYSCON_ANA_EN_BOR_EN_MASK;
    }
    base->ANA_EN |= ampReg;
    /*100us delay for BOD_AMP stable*/
    for (delayX = 0; delayX < 800; delayX++)
    {
    }
    base->ANA_EN |= enReg;
}

void BOD_Disable(SYSCON_Type *base, uint8_t mode)
{
    uint32_t enReg = 0;
    if (mode & kBOD_InterruptEnable)
    {
        enReg |= SYSCON_ANA_EN_BOD_EN_MASK;
    }
    if (mode & kBOD_ResetEnable)
    {
        enReg |= SYSCON_ANA_EN_BOR_EN_MASK;
    }
    base->ANA_EN &= ~enReg;
}

void BOD_Deinit(SYSCON_Type *base)
{
    base->ANA_EN &= ~(SYSCON_ANA_EN_BOD_EN_MASK | SYSCON_ANA_EN_BOR_EN_MASK);
}

void BOD_GetDefaultConfig(bod_config_t *config)
{
    config->int_thr = kBOD_InterruptThreshold2;
    config->reset_thr = kBOD_ResetThreshold2;
}
