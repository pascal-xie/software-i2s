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

#ifndef _FSL_RF_H_
#define _FSL_RF_H_

#include "fsl_common.h"

/*! @addtogroup rf */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Definitions
 *****************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief QN9080 radio frequency version 2.0.0. */
#define FSL_QN9080_RADIO_FREQUENCY_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * @brief  Output power at SMA socket, depends on off-chip RF circuit on PCB.
 */
typedef enum _tx_power
{
    kTxPowerMinus33dBm = 0U, /*!< -33 dBm */
    kTxPowerMinus27dBm,      /*!< -27 dBm */
    kTxPowerMinus23dBm,      /*!< -23 dBm */
    kTxPowerMinus21dBm,      /*!< -21 dBm */
    kTxPowerMinus19dBm,      /*!< -19 dBm */
    kTxPowerMinus17dBm,      /*!< -17 dBm */
    kTxPowerMinus16dBm,      /*!< -16 dBm */
    kTxPowerMinus15dBm,      /*!< -15 dBm */
    kTxPowerMinus14dBm,      /*!< -14 dBm */
    kTxPowerMinus13dBm,      /*!< -13 dBm */
    kTxPowerMinus12dBm,      /*!< -12 dBm */
    kTxPowerMinus11dBm,      /*!< -11 dBm */
    kTxPowerMinus10dBm,      /*!< -10 dBm */
    kTxPowerMinus9dBm,       /*!< -9  dBm */
    kTxPowerMinus8dBm,       /*!< -8  dBm */
    kTxPowerMinus7dBm,       /*!< -7  dBm */
    kTxPowerMinus6dBm,       /*!< -6  dBm */
    kTxPowerMinus5dBm,       /*!< -5  dBm */
    kTxPowerMinus4dBm,       /*!< -4  dBm */
    kTxPowerMinus3dBm,       /*!< -3  dBm */
    kTxPowerMinus2dBm,       /*!< -2  dBm */
    kTxPowerMinus1dBm,       /*!< -1  dBm */
    kTxPower0dBm,            /*!<  0  dBm */
    kTxPower1dBm,            /*!<  1  dBm */
    kTxPower2dBm,            /*!<  2  dBm */
    kTxPower3dBm,            /*!<  3  dBm */
    kTxPowerInvalid
} tx_power_t;

/*!
 * @brief  Rx mode of the chip.
 */
typedef enum _tx_mode
{
    kRxModeHighPerformance = 0U, /*!< High performance mode, but the power consumption is high. */
    kRxModeBalanced,             /*!< Balanced mode, a trade-off between performance and power consumpiton. */
    kRxModeHighEfficiency,       /*!< High efficiency mode, but rx performance will not be so good. */
} rx_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief  Set Tx Power setting
 * @param   txpwr  tx power level defined by enum @tx_power_t
 */
void RF_SetTxPowerLevel(SYSCON_Type *base, tx_power_t txpwr);

/*!
 * @brief  Get Tx Power setting value
 * @return Tx Power setting value defined by enum @tx_power_t
 */
tx_power_t RF_GetTxPowerLevel(SYSCON_Type *base);

/*!
 * @brief  Set Rx mode
 * @param  Rx mode defined by enum @rx_mode_t
 */
void RF_ConfigRxMode(SYSCON_Type *base, rx_mode_t rm);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _FSL_RF_H_ */
