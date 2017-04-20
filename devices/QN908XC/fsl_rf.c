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

#include "fsl_rf.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint8_t rf_tx_gain_tab[kTxPowerInvalid] = {
        [kTxPowerMinus33dBm] = 0x00, /*!< -33 dBm */
        [kTxPowerMinus27dBm] = 0x01, /*!< -27 dBm */
        [kTxPowerMinus23dBm] = 0x02, /*!< -23 dBm */
        [kTxPowerMinus21dBm] = 0x03, /*!< -21 dBm */
        [kTxPowerMinus19dBm] = 0x04, /*!< -19 dBm */
        [kTxPowerMinus17dBm] = 0x05, /*!< -17 dBm */
        [kTxPowerMinus16dBm] = 0x06, /*!< -16 dBm */
        [kTxPowerMinus15dBm] = 0x07, /*!< -15 dBm */
        [kTxPowerMinus14dBm] = 0x08, /*!< -14 dBm */
        [kTxPowerMinus13dBm] = 0x0A, /*!< -13 dBm */
        [kTxPowerMinus12dBm] = 0x0B, /*!< -12 dBm */
        [kTxPowerMinus11dBm] = 0x0D, /*!< -11 dBm */
        [kTxPowerMinus10dBm] = 0x0E, /*!< -10 dBm */
        [kTxPowerMinus9dBm] = 0x10,  /*!<  -9 dBm */
        [kTxPowerMinus8dBm] = 0x13,  /*!<  -8 dBm */
        [kTxPowerMinus7dBm] = 0x15,  /*!<  -7 dBm */
        [kTxPowerMinus6dBm] = 0x19,  /*!<  -6 dBm */
        [kTxPowerMinus5dBm] = 0x1E,  /*!<  -5 dBm */
        [kTxPowerMinus4dBm] = 0x22,  /*!<  -4 dBm */
        [kTxPowerMinus3dBm] = 0x29,  /*!<  -3 dBm */
        [kTxPowerMinus2dBm] = 0x31,  /*!<  -2 dBm */
        [kTxPowerMinus1dBm] = 0x3E,  /*!<  -1 dBm */
        [kTxPower0dBm] = 0x4F,       /*!<   0 dBm */
        [kTxPower1dBm] = 0x6D,       /*!<   1 dBm */
        [kTxPower2dBm] = 0x9F,       /*!<   2 dBm */
        [kTxPower3dBm] = 0xFE        /*!<   3 dBm */
};

static const uint8_t rf_tx_gain_tab_dcdc[kTxPowerInvalid] = {
        [kTxPowerMinus33dBm] = 0x00, /*!< -33 dBm */
        [kTxPowerMinus27dBm] = 0x01, /*!< -27 dBm */
        [kTxPowerMinus23dBm] = 0x02, /*!< -23 dBm */
        [kTxPowerMinus21dBm] = 0x03, /*!< -21 dBm */
        [kTxPowerMinus19dBm] = 0x04, /*!< -19 dBm */
        [kTxPowerMinus17dBm] = 0x05, /*!< -17 dBm */
        [kTxPowerMinus16dBm] = 0x06, /*!< -16 dBm */
        [kTxPowerMinus15dBm] = 0x07, /*!< -15 dBm */
        [kTxPowerMinus14dBm] = 0x08, /*!< -14 dBm */
        [kTxPowerMinus13dBm] = 0x0A, /*!< -13 dBm */
        [kTxPowerMinus12dBm] = 0x0B, /*!< -12 dBm */
        [kTxPowerMinus11dBm] = 0x0D, /*!< -11 dBm */
        [kTxPowerMinus10dBm] = 0x0E, /*!< -10 dBm */
        [kTxPowerMinus9dBm] = 0x10,  /*!<  -9 dBm */
        [kTxPowerMinus8dBm] = 0x13,  /*!<  -8 dBm */
        [kTxPowerMinus7dBm] = 0x15,  /*!<  -7 dBm */
        [kTxPowerMinus6dBm] = 0x19,  /*!<  -6 dBm */
        [kTxPowerMinus5dBm] = 0x1E,  /*!<  -5 dBm */
        [kTxPowerMinus4dBm] = 0x22,  /*!<  -4 dBm */
        [kTxPowerMinus3dBm] = 0x29,  /*!<  -3 dBm */
        [kTxPowerMinus2dBm] = 0x31,  /*!<  -2 dBm */
        [kTxPowerMinus1dBm] = 0x3E,  /*!<  -1 dBm */
        [kTxPower0dBm] = 0x4F,       /*!<   0 dBm */
        [kTxPower1dBm] = 0x6D,       /*!<   1 dBm */
        [kTxPower2dBm] = 0x9F,       /*!<   2 dBm */
        [kTxPower3dBm] = 0xFE        /*!<   3 dBm */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void RF_SetTxPowerLevel(SYSCON_Type *base, tx_power_t txpwr)
{
    uint8_t *temp;

    if (base->PMU_CTRL1 & SYSCON_PMU_CTRL1_BUCK_CTRL_MASK) /* DC-DC bypassed */
    {
        temp = (uint8_t *)rf_tx_gain_tab;
    }
    else
    {
        temp = (uint8_t *)rf_tx_gain_tab_dcdc;
    }

    base->ANA_CTRL0 = (base->ANA_CTRL0 & ~SYSCON_ANA_CTRL0_PA_POWER_MASK) | SYSCON_ANA_CTRL0_PA_POWER(temp[txpwr]);
}

tx_power_t RF_GetTxPowerLevel(SYSCON_Type *base)
{
    uint8_t *temp;
    uint8_t cnt;
    tx_power_t level = kTxPowerInvalid;
    uint32_t val = base->ANA_CTRL0 & SYSCON_ANA_CTRL0_PA_POWER_MASK;

    if (base->PMU_CTRL1 & SYSCON_PMU_CTRL1_BUCK_CTRL_MASK) /* DC-DC bypassed */
    {
        temp = (uint8_t *)rf_tx_gain_tab;
    }
    else
    {
        temp = (uint8_t *)rf_tx_gain_tab_dcdc;
    }

    for (cnt = 0U; cnt < kTxPowerInvalid; cnt++)
    {
        if (temp[cnt] == val)
        {
            level = (tx_power_t)cnt;
            break;
        }
    }

    return level;
}

void RF_ConfigRxMode(SYSCON_Type *base, rx_mode_t rm)
{
    switch (rm)
    {
        case kRxModeHighEfficiency:
            BLEDP->DP_TOP_SYSTEM_CTRL = (BLEDP->DP_TOP_SYSTEM_CTRL & ~BLEDP_DP_TOP_SYSTEM_CTRL_DET_MODE_MASK);
            /* reduce LNA current tp 50% */
            CALIB->RRF1 = (CALIB->RRF1 & ~CALIB_RRF1_RRF_BM_LNA_MASK) | CALIB_RRF1_RRF_BM_LNA(0U);
            /* reduce the current PPF to 50% */
            /* CALIB->PLL48_PPF = (CALIB->PLL48_PPF & ~CALIB_PLL48_PPF_PPF_BM_MASK) | CALIB_PLL48_PPF_PPF_BM(0U);*/
            break;
        case kRxModeBalanced:
            break;
        case kRxModeHighPerformance:
        default:
            /* LNA current */
            CALIB->RRF1 = (CALIB->RRF1 & ~CALIB_RRF1_RRF_BM_LNA_MASK) | CALIB_RRF1_RRF_BM_LNA(0x3U);
            /* PPF current */
            CALIB->PLL48_PPF = (CALIB->PLL48_PPF & ~CALIB_PLL48_PPF_PPF_BM_MASK) | CALIB_PLL48_PPF_PPF_BM(0x3U);
            break;
    }
}
