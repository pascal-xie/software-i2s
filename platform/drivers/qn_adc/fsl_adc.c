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

#include "fsl_adc.h"

/*******************************************************************************
 * Definitions
 *****************************************************************************/
#define ADC_FIR_STABLE ADC_CFG_DS_DATA_STABLE(3U << 2)
#define ADC_HALFBAND_STABLE ADC_CFG_DS_DATA_STABLE(14U << 2)

#define ADC_HALFBAND_MULTIPLY (1.02392578125)
#define ADC_BANDGAP_VOLTAGE (1200.0)

#define ADC_BANDGAP_INFO_ADDR (FSL_FEATURE_FLASH_ADDR_OF_BANDGAP_VOL)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get clock index
 *
 * @param base ADC peripheral base address
 *
 * @return clock index
 */
static uint8_t ADC_GetClockIndex(ADC_Type *base);

/*!
 * @brief Get instance number for ADC module.
 *
 * @param base ADC peripheral base address
 */
static uint32_t ADC_GetInstance(ADC_Type *base);

/*!
 * @brief Get channel conversion result.
 *
 * @param base  ADC peripheral base address.
 * @param ch ADC channel number.
 * @param idx Configure register index, 0 for register CFG0 and 1 for register CFG1.
 *
 * @return Conversion result
 */
static uint32_t ADC_GetChannelConversionResult(ADC_Type *base, uint8_t ch, const adc_config_t *config);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to ADC bases for each instance. */
static ADC_Type *const s_adcBases[] = ADC_BASE_PTRS;

/*! @brief Pointers to ADC clocks for each instance. */
static const clock_ip_name_t s_adcClocks[] = ADC_CLOCKS;

/*! @brief Pointers to ADC resets for each instance. */
static const reset_ip_name_t s_adcResets[] = ADC_RSTS;

/**
 * Scan interval match table
 *
 *     ADC clock       Scan interval
 *       31.25K         4cycles
 *       62.5K          8cycles
 *       125K           16cycles
 *       250K           32cycles
 *       500K           64cycles
 *        1M            128cycles
 *        2M            256cycles
 *       32K            4cycles
 */
static const uint8_t s_adcScanInterval[] = {1, 2, 3, 4, 5, 6, 7, 1};
static const uint8_t s_adcVrefVinnBias[] = {2, 3, 3, 3, 3, 3, 5, 2};
static const uint8_t s_adcClock[] = {0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1F};
static const uint32_t s_adcBias[] = {
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(1)), /* clock 31.25K */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(3)), /* clock 62.5K */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(3)), /* clock 125K */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(3)), /* clock 250K */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(3)), /* clock 500K */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(3)), /* clock 1M */
    (ADC_ANA_CTRL_ADC_BM(4)),                                 /* clock 2M */
    (ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_ADC_BM(1))  /* clock 32K */
};

static const uint32_t s_adcPgaBias[][5] = {
    /* Clock 31.25K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(0),  /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(0),  /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(2),  /* PGA gain4 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4),  /* PGA gain8 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4)}, /* PGA gain16 */
    /* Clock 62.5K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(5), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(5),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(5)},                             /* PGA gain16 */
    /* Clock 125K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(5), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(5),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(5)},                             /* PGA gain16 */
    /* Clock 250K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(5), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(5),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(5)},                             /* PGA gain16 */
    /* Clock 500K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(5), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(5),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(5)},                             /* PGA gain16 */
    /* Clock 1M */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(3), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(5), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(5),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(5)},                             /* PGA gain16 */
    /* Clock 2M */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4), /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4), /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4), /* PGA gain4 */
     ADC_BG_BF_PGA_BM(4),                              /* PGA gain8 */
     ADC_BG_BF_PGA_BM(4)},                             /* PGA gain16 */
    /* Clock 32K */
    {ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(0),  /* PGA gain1 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(0),  /* PGA gain2 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(2),  /* PGA gain4 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4),  /* PGA gain8 */
     ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM(4)}, /* PGA gain16 */
};

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint8_t ADC_GetClockIndex(ADC_Type *base)
{
    uint8_t i;
    uint8_t size = sizeof(s_adcClock) / sizeof(s_adcClock[0]);
    uint8_t clock = (uint8_t)((base->CTRL & ADC_CTRL_CLKSEL_MASK) >> ADC_CTRL_CLKSEL_SHIFT);
    for (i = 0; (i < size) && (clock != s_adcClock[i]); i++)
        ;
    return i;
}

static uint32_t ADC_GetInstance(ADC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_ADC_COUNT; instance++)
    {
        if (s_adcBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_ADC_COUNT);

    return instance;
}

void ADC_Init(ADC_Type *base, const adc_config_t *config)
{
    assert(config != NULL);

    uint8_t clock;
    uint32_t mask, tmp32;

    /* Enable the clock */
    CLOCK_EnableClock(s_adcClocks[ADC_GetInstance(base)]);

    /* Reset ADC */
    RESET_PeripheralReset(s_adcResets[ADC_GetInstance(base)]);

    /* Configure ADC working clock, reference source, data format, trigger source and convert mode */
    mask = ADC_CTRL_CLKSEL_MASK | ADC_CTRL_VREF_SEL_MASK | ADC_CTRL_CH_IDX_EN_MASK | ADC_CTRL_DATA_FORMAT_MASK |
           ADC_CTRL_TRIGGER_MASK | ADC_CTRL_CONV_MODE_MASK | ADC_CTRL_SCAN_EN_MASK;
    tmp32 = ADC_CTRL_CLKSEL(s_adcClock[config->clock]) | ADC_CTRL_VREF_SEL(config->refSource) | config->dataFormat |
            (uint32_t)config->convMode | ADC_CTRL_TRIGGER(config->triggerSource);
    base->CTRL = (base->CTRL & ~mask) | tmp32;
    /* Configure channel enable mask */
    base->CH_SEL = (uint32_t)config->channelEnable;
    /* Configure channel sigma delta selection */
    base->CH_CFG = (uint32_t)config->channelConfig;

    /* Configure scan interval */
    clock = config->clock;
    base->CFG[0] = (base->CFG[0] & ~ADC_CFG_SCAN_INTV_MASK) | ADC_CFG_SCAN_INTV(s_adcScanInterval[clock]);
    base->CFG[1] = (base->CFG[1] & ~ADC_CFG_SCAN_INTV_MASK) | ADC_CFG_SCAN_INTV(s_adcScanInterval[clock]);

    /* Configure ADC, Vref, VINN_IN, VINN_OUT bias */
    mask = ADC_ANA_CTRL_ADC_BM_MASK | ADC_ANA_CTRL_ADC_BM_DIV2_MASK | ADC_ANA_CTRL_VREF_BM_MASK |
           ADC_ANA_CTRL_VINN_IN_BM_MASK | ADC_ANA_CTRL_VINN_OUT_BM_MASK;
    tmp32 = s_adcVrefVinnBias[clock];
    tmp32 = s_adcBias[clock] | ADC_ANA_CTRL_VREF_BM(tmp32) | ADC_ANA_CTRL_VINN_IN_BM(tmp32) |
            ADC_ANA_CTRL_VINN_OUT_BM(tmp32);
    base->ANA_CTRL = (base->ANA_CTRL & ~mask) | tmp32;
}

void ADC_Deinit(ADC_Type *base)
{
    /* Disable the clock */
    CLOCK_DisableClock(s_adcClocks[ADC_GetInstance(base)]);
}

void ADC_GetDefaultConfig(adc_config_t *config)
{
    config->channelEnable = 0;                          /* Default does not enable any channel */
    config->channelConfig = 0;                          /* Default use sigma delta configure 0 */
    config->triggerSource = kADC_TriggerSelectSoftware; /* Default use software trigger */
    config->convMode = kADC_ConvModeSingle;             /* Default use single convert mode */
    config->clock = kADC_Clock500K;
    config->refSource = kADC_RefSourceBandgapWithDriver;
    config->dataFormat = kADC_DataFormat1WithIdx;
}

void ADC_SetSdConfig(ADC_Type *base, uint8_t idx, const adc_sd_config_t *config)
{
    uint8_t clock, gain;
    uint32_t mask, tmp32;
    mask = ADC_CFG_PGA_GAIN_MASK | ADC_CFG_PGA_BP_MASK | ADC_CFG_PGA_VINN_MASK | ADC_CFG_ADC_GAIN_MASK |
           ADC_CFG_VREF_GAIN_MASK | ADC_CFG_DOWN_SAMPLE_RATE_MASK | ADC_CFG_DS_DATA_STABLE_MASK |
           ADC_CFG_SCAN_INTV_MASK | ADC_CFG_PGA_VCM_EN_MASK | ADC_CFG_PGA_VCM_DIR_MASK | ADC_CFG_PGA_VCM_MASK |
           ADC_CFG_ADC_VCM_MASK;

    tmp32 = ADC_CFG_ADC_GAIN(config->gain) | ADC_CFG_VREF_GAIN(config->refGain) | config->pgaGain |
            ADC_CFG_PGA_VINN(config->vinnSelect) | ADC_CFG_DOWN_SAMPLE_RATE(config->downSample) |
            ADC_CFG_PGA_VCM_DIR(config->adjustDirection) | ADC_CFG_PGA_VCM(config->adjustValue) |
            ADC_CFG_ADC_VCM(config->vcmSelect);
    /* Enable the PGA adjust feature if adjust value does not match 0 */
    tmp32 |= (config->adjustValue != 0) ? ADC_CFG_PGA_VCM_EN_MASK : 0;
    /* Configure stable number depand on down sample rate */
    tmp32 |= (config->downSample == kADC_DownSample256) ? ADC_HALFBAND_STABLE : ADC_FIR_STABLE;
    base->CFG[idx] = (base->CFG[idx] & ~mask) | tmp32;

    /* Configure PGA bias */
    clock = ADC_GetClockIndex(base);
    gain = (base->CFG[0] & ADC_CFG_PGA_GAIN_MASK) > (base->CFG[1] & ADC_CFG_PGA_GAIN_MASK) ?
               (uint8_t)(base->CFG[0] & ADC_CFG_PGA_GAIN_MASK) :
               (uint8_t)(base->CFG[1] & ADC_CFG_PGA_GAIN_MASK);
    mask = ADC_BG_BF_PGA_BM_DIV2_MASK | ADC_BG_BF_PGA_BM_MASK;
    tmp32 = s_adcPgaBias[clock][gain];
    base->BG_BF = (base->BG_BF & ~mask) | tmp32;
}

void ADC_GetSdDefaultConfig(adc_sd_config_t *config)
{
    config->gain = kADC_Gain1;
    config->refGain = kADC_RefGain1P5;
    config->pgaGain = kADC_PgaGain1;
    config->vinnSelect = kADC_VinnSelectVref0P75;
    config->downSample = kADC_DownSample256;
    config->adjustDirection = kADC_PgaAdjustMoveDown;
    config->adjustValue = 0;
    config->vcmSelect = kADC_VcmVoltage4D8;
}

void ADC_WindowCompareConfig(ADC_Type *base, const adc_window_compare_config_t *config)
{
    if (!config)
    {
        /* Disable window comparation */
        base->CTRL &= ~ADC_CTRL_WCMP_EN_MASK;
        return;
    }

    /* Configure window comparation threshold */
    base->WCMP_THR = ADC_WCMP_THR_WCMP_THR_HIGH(config->highValue) | ADC_WCMP_THR_WCMP_THR_LOW(config->lowValue);
    /* Enable window comparation */
    base->CTRL |= ADC_CTRL_WCMP_EN_MASK;
}

float ADC_ConversionResult2Mv(ADC_Type *base, uint8_t ch, uint8_t idx, float vref, uint32_t vinn, uint32_t result)
{
    int32_t res_mv, vinn_mv;
    float fmv;
    adc_data_format_t format;

    format = (adc_data_format_t)(base->CTRL & (ADC_CTRL_CH_IDX_EN_MASK | ADC_CTRL_DATA_FORMAT_MASK));
    switch (format)
    {
        case kADC_DataFormat0WithoutIdx:
            res_mv = (int32_t)result;
            vinn_mv = (int32_t)vinn;
            break;
        case kADC_DataFormat0WithIdx:
            ch = (uint8_t)(result & 0x1F);
            res_mv = (int32_t)(result & (~0x1F));
            vinn_mv = (int32_t)(vinn & (~0x1F));
            break;
        case kADC_DataFormat1WithoutIdx:
            res_mv = (int32_t)(result << 9);
            vinn_mv = (int32_t)(vinn << 9);
            break;
        case kADC_DataFormat1WithIdx:
            ch = (uint8_t)((result >> 27) & 0x1F);
            res_mv = result << 9;
            vinn_mv = vinn << 9;
            break;
        default:
            break;
    }

    fmv = (float)res_mv / (float)0x80000000 * vref;
    if ((1U << ch) & 0x4FF0)
    {
        fmv += (float)vinn_mv / (float)0x80000000 * vref;
    }

    /* If half-band filter(down sample 8) used, need multiply (1 + 2^-6 + 2^-7 + 2^-11) = 1.02392578125 */
    if ((base->CFG[idx] & ADC_CFG_DOWN_SAMPLE_RATE_MASK) == ADC_CFG_DOWN_SAMPLE_RATE(kADC_DownSample256))
    {
        fmv *= (float)ADC_HALFBAND_MULTIPLY;
    }

    return fmv;
}

static uint32_t ADC_GetChannelConversionResult(ADC_Type *base, uint8_t ch, const adc_config_t *config)
{
    uint32_t mask, tmp32, vinn;

    /* Disable ADC */
    ADC_Enable(base, false);
    /* Empty buffer */
    ADC_EmptyChannelConversionBuffer(base);

    /* Configure channel = 20, software trigger, single convert mode */
    base->CH_SEL = 1U << ch;
    mask = ADC_CTRL_TRIGGER_MASK | ADC_CTRL_CONV_MODE_MASK | ADC_CTRL_SCAN_EN_MASK;
    tmp32 = ADC_CTRL_TRIGGER(kADC_TriggerSelectSoftware) | (uint32_t)kADC_ConvModeSingle;
    base->CTRL = (base->CTRL & ~mask) | tmp32;

    /* Enable ADC */
    ADC_Enable(base, true);
    /* Software trigger */
    ADC_DoSoftwareTrigger(base);
    /* Wait for convert complete */
    while (!(ADC_GetStatusFlags(base) & kADC_DataReadyFlag))
        ;
    /* Get the result */
    vinn = ADC_GetConversionResult(base);
    /* Disable ADC */
    ADC_Enable(base, false);

    /* Restore user configure */
    base->CH_SEL = config->channelEnable;
    tmp32 = ADC_CTRL_TRIGGER(config->triggerSource) | (uint32_t)config->convMode;
    base->CTRL = (base->CTRL & ~mask) | tmp32;
    return vinn;
}

float ADC_GetBandgapCalibrationResult(ADC_Type *base, uint8_t idx)
{
    float cal;

    (void)base;
    cal = (*(uint32_t *)ADC_BANDGAP_INFO_ADDR == 0xFFFFFFFF) ? ADC_BANDGAP_VOLTAGE : *(uint32_t *)ADC_BANDGAP_INFO_ADDR;
    if ((base->CFG[idx] & ADC_CFG_VREF_GAIN_MASK) == ADC_CFG_VREF_GAIN(kADC_RefGain1P5))
    {
        cal *= 1.5f;
    }

    return cal;
}

uint32_t ADC_GetVinnCalibrationResult(ADC_Type *base, const adc_config_t *config)
{
    return ADC_GetChannelConversionResult(base, 20, config);
}

uint32_t ADC_GetOffsetCalibrationResult(ADC_Type *base, const adc_config_t *config)
{
    return ADC_GetChannelConversionResult(base, 15, config);
}
