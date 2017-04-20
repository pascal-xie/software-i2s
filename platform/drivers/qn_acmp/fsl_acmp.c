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
 * @addtogroup ACMP
 * @{
 */
#include "fsl_acmp.h"

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
void ACMP_Init(SYSCON_Type *base, const acmp_config_t *config)
{
    uint32_t mask = 0;
    uint32_t reg = 0;

    if (config->ch == kACMP_Channel0)
    {
        mask = SYSCON_ANA_EN_ACMP_VREF_SEL_MASK | SYSCON_ANA_EN_ACMP0_REF_MASK | SYSCON_ANA_EN_ACMP0_EDGE_SEL_MASK |
               SYSCON_ANA_EN_ACMP0_HYST_EN_MASK;
        if (config->refSrc != kACMP_ReferenceSourceExternalReferenceVoltage)
        {
            reg |= SYSCON_ANA_EN_ACMP_VREF_SEL(config->refSrc) | SYSCON_ANA_EN_ACMP0_REF(config->refDiv);
        }
        reg |= SYSCON_ANA_EN_ACMP0_EDGE_SEL(config->trigerEdge) | SYSCON_ANA_EN_ACMP0_HYST_EN(config->hystEn);
        base->ANA_EN &= ~mask;
        base->ANA_EN |= reg;
    }
    else
    {
        mask = SYSCON_ANA_EN_ACMP_VREF_SEL_MASK | SYSCON_ANA_EN_ACMP1_REF_MASK | SYSCON_ANA_EN_ACMP1_EDGE_SEL_MASK |
               SYSCON_ANA_EN_ACMP1_HYST_EN_MASK;
        if (config->refSrc != kACMP_ReferenceSourceExternalReferenceVoltage)
        {
            reg |= SYSCON_ANA_EN_ACMP_VREF_SEL(config->refSrc) | SYSCON_ANA_EN_ACMP1_REF(config->refDiv);
        }
        reg |= SYSCON_ANA_EN_ACMP1_EDGE_SEL(config->trigerEdge) | SYSCON_ANA_EN_ACMP1_HYST_EN(config->hystEn);
        base->ANA_EN &= ~mask;
        base->ANA_EN |= reg;
    }
}

void ACMP_Enable(SYSCON_Type *base, acmp_channel_t ch)
{
    if (ch == kACMP_Channel0)
    {
        base->ANA_EN |= SYSCON_ANA_EN_ACMP0_EN_MASK;
    }
    else
    {
        base->ANA_EN |= SYSCON_ANA_EN_ACMP1_EN_MASK;
    }
}

void ACMP_Disable(SYSCON_Type *base, acmp_channel_t ch)
{
    if (ch == kACMP_Channel0)
    {
        base->ANA_EN &= ~SYSCON_ANA_EN_ACMP0_EN_MASK;
    }
    else
    {
        base->ANA_EN &= ~SYSCON_ANA_EN_ACMP1_EN_MASK;
    }
}

void ACMP_EnableInterrupts(SYSCON_Type *base, acmp_channel_t ch)
{
    if (ch == kACMP_Channel0)
    {
        base->ANA_EN |= SYSCON_ANA_EN_ACMP0_INTEN_MASK;
    }
    else
    {
        base->ANA_EN |= SYSCON_ANA_EN_ACMP1_INTEN_MASK;
    }
}

void ACMP_DisableInterrupts(SYSCON_Type *base, acmp_channel_t ch)
{
    if (ch == kACMP_Channel0)
    {
        base->ANA_EN &= ~SYSCON_ANA_EN_ACMP0_INTEN_MASK;
    }
    else
    {
        base->ANA_EN &= ~SYSCON_ANA_EN_ACMP1_INTEN_MASK;
    }
}

uint8_t ACMP_GetValue(SYSCON_Type *base, acmp_channel_t ch)
{
    if (ch == kACMP_Channel0)
    {
        return (base->ANA_EN & SYSCON_ANA_EN_ACMP0_OUT_MASK) >> SYSCON_ANA_EN_ACMP0_OUT_SHIFT;
    }
    else
    {
        return (base->ANA_EN & SYSCON_ANA_EN_ACMP1_OUT_MASK) >> SYSCON_ANA_EN_ACMP1_OUT_SHIFT;
    }
}

void ACMP_GetDefaultConfig(acmp_config_t *config)
{
    config->ch = kACMP_Channel0;
    config->refSrc = kACMP_ReferenceSourceExternalReferenceVoltage;
    config->hystEn = kACMP_HysteresisDisable;
    config->trigerEdge = kACMP_TrigerRising;
    config->refDiv = kACMP_ReferenceVoltageDivider1;
}
