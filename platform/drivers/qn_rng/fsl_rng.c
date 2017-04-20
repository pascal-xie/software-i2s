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

#include "fsl_rng.h"
#include "fsl_adc.h"

/*******************************************************************************
 * Definitions
 *****************************************************************************/
#define RNG_ADC_CHANNEL (15U)
#define RNG_ADC_CFG_IDX (0U)
#define RNG_ADC_TRIGGER (kADC_TriggerSelectRNG)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void RNG_Init(RNG_Type *base)
{
    adc_config_t adcConfigStruct;
    adc_sd_config_t adcSdConfigStruct;

    /**
     * Initial ADC
     */
    ADC_GetDefaultConfig(&adcConfigStruct);
    adcConfigStruct.channelEnable = (1U << RNG_ADC_CHANNEL);
    adcConfigStruct.channelConfig = (RNG_ADC_CFG_IDX << RNG_ADC_CHANNEL);
    adcConfigStruct.triggerSource = RNG_ADC_TRIGGER;
    adcConfigStruct.convMode = kADC_ConvModeBurst;
    adcConfigStruct.clock = kADC_Clock2M;
    ADC_Init(ADC, &adcConfigStruct);

    /* Initial ADC Sigma Delta(SD) configuration */
    ADC_GetSdDefaultConfig(&adcSdConfigStruct);
    ADC_SetSdConfig(ADC, RNG_ADC_CFG_IDX, &adcSdConfigStruct);

    /* Enable ADC */
    ADC_Enable(ADC, true);

    /* Fix data width to 8bits */
    base->CTRL = (base->CTRL & ~RNG_CTRL_NUM_MASK) | RNG_CTRL_NUM(0);
}

void RNG_Deinit(RNG_Type *base)
{
    ADC_Deinit(ADC);
}

status_t RNG_GetRandomData(RNG_Type *base, uint8_t *data, size_t dataSize)
{
    status_t result = kStatus_Success;

    if (base && data && dataSize)
    {
        while (dataSize--)
        {
            /* RNG start */
            RNG_Start(base);
            /* Wait for RNG not busy */
            while (RNG_GetStatusFlags(base) & kRNG_BusyFlag)
                ;
            /* Get random number */
            *data++ = (uint8_t)RNG_GetRandomNumber(base);
        }
    }
    else
    {
        result = kStatus_InvalidArgument;
    }

    return result;
}
