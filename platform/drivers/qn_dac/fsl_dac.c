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
 * @addtogroup DAC
 * @{
 */
#include "fsl_dac.h"
#include "fsl_clock.h"
#include "fsl_reset.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address to be used to gate or ungate the module clock
 *
 * @param base DAC peripheral base address
 *
 * @return The DAC instance
 */
static uint32_t DAC_GetInstance(DAC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to DAC bases for each instance. */
static DAC_Type *const s_dacBases[] = DAC_BASE_PTRS;

/*! @brief Pointers to DAC clocks for each instance. */
static const clock_ip_name_t s_dacClocks[] = DAC_CLOCKS;

/*! @brief Pointers to DAC resets for each instance. */
static const reset_ip_name_t s_dacResets[] = DAC_RSTS;
/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t DAC_GetInstance(DAC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_DAC_COUNT; instance++)
    {
        if (s_dacBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_DAC_COUNT);

    return instance;
}
void DAC_Init(DAC_Type *base, const dac_config_t *config)
{
    assert(config != NULL);
    /* Enable the clock */
    CLOCK_EnableClock(s_dacClocks[DAC_GetInstance(base)]);
    /* Reset the module*/
    RESET_PeripheralReset(s_dacResets[DAC_GetInstance(base)]);

    /*Analog configuration*/
    base->ANA_CFG = DAC_ANA_CFG_DAC_AMP(config->ana_cfg.amp) | DAC_ANA_CFG_FILTER_BW(config->ana_cfg.filter_bandwidth) |
                    DAC_ANA_CFG_FILTER_150K_EN(config->ana_cfg.filter_150k_en) | DAC_ANA_CFG_VCM(config->ana_cfg.vcm);

    base->SIN_CFG0 = DAC_SIN_CFG0_SIN_FREQ(config->sin_cfg.freq) | DAC_SIN_CFG0_SIN_AMP(config->sin_cfg.amp);

    base->SIN_CFG1 = DAC_SIN_CFG1_SIN_DC(config->sin_cfg.dc_offset);

    base->GAIN_CTRL = DAC_GAIN_CTRL_GAIN_CTRL(config->gain_ctrl);

    /*Dac ctrl*/
    base->CTRL = DAC_CTRL_SIN_EN(config->sin_cfg.en) | DAC_CTRL_MOD_EN(config->mod_cfg.en) |
                 DAC_CTRL_MOD_WD(config->mod_cfg.out_wd) | DAC_CTRL_SMPL_RATE(config->mod_cfg.smpl_rate) |
                 DAC_CTRL_SGN_INV(config->sign_inv) | DAC_CTRL_BUF_IN_ALGN(config->output) |
                 DAC_CTRL_BUF_OUT_ALGN(config->input) | DAC_CTRL_TRG_MODE(config->trg_cfg.mode) |
                 DAC_CTRL_TRG_EDGE(config->trg_cfg.edge) | DAC_CTRL_TRG_SEL(config->trg_cfg.src) |
                 DAC_CTRL_CLK_DIV(config->clk_div) | DAC_CTRL_CLK_INV(config->clk_inv);
}

void DAC_GetDefaultConfig(dac_config_t *config)
{
    assert(config != NULL);
    uint32_t apb_clk = CLOCK_GetFreq(kCLOCK_ApbClk);

    config->ana_cfg.amp = kDAC_Amplitude100pct;
    config->ana_cfg.filter_bandwidth = kDAC_FilterBandwidth56FF;
    config->ana_cfg.filter_150k_en = kDAC_Filter150kEnable150Khz;
    config->ana_cfg.vcm = kDAC_VoltageCommonMode1500mv;

    config->sin_cfg.en = kDAC_SinWaveDisable;
    config->mod_cfg.en = kDAC_ModulatorDisable;
    config->mod_cfg.out_wd = kDAC_ModulatorWidth1bit;
    config->mod_cfg.smpl_rate = kDAC_SampleRate8;
    config->sign_inv = 0;
    config->output = kDAC_BufferOutAlignRight;
    config->input = kDAC_BufferInAlignRight;
    config->trg_cfg.mode = kDAC_TriggerModeSingleMode;
    config->trg_cfg.edge = kDAC_TriggerEdgeSelectPositiveEdge;
    config->trg_cfg.src = kDAC_TriggerSelectSoftware;
    config->clk_div = apb_clk / 1000000 / 2 - 1;
    config->clk_inv = 0;

    config->sin_cfg.freq = 0;
    config->sin_cfg.amp = 0;
    config->sin_cfg.dc_offset = 0;

    config->gain_ctrl = 0x10;
}

void DAC_Deinit(DAC_Type *base)
{
    CLOCK_DisableClock(s_dacClocks[DAC_GetInstance(base)]);
}
