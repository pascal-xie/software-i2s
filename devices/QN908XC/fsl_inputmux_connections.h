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

#ifndef _FSL_INPUTMUX_CONNECTIONS_
#define _FSL_INPUTMUX_CONNECTIONS_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @addtogroup qn_inputmux
 * @{
 */

/*! @brief Periphinmux IDs */
#define PINTSEL_PMUX_ID (0U)
#define DMA_TRIG0_PMUX_ID (1U)
#define DMA_OTRIG_PMUX_ID (4U)
#define PMUX_SHIFT (20U)

/*! @brief INPUTMUX connections type */
typedef enum _inputmux_connection_t
{
    /*!< Pin Interrupt. */
    kINPUTMUX_GpioPort0Pin0ToPintsel = 0U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin1ToPintsel = 1U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin2ToPintsel = 2U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin3ToPintsel = 3U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin4ToPintsel = 4U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin5ToPintsel = 5U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin6ToPintsel = 6U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin7ToPintsel = 7U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin8ToPintsel = 8U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin9ToPintsel = 9U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin10ToPintsel = 10U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin11ToPintsel = 11U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin12ToPintsel = 12U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin13ToPintsel = 13U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin14ToPintsel = 14U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin15ToPintsel = 15U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin16ToPintsel = 16U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin17ToPintsel = 17U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin18ToPintsel = 18U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin19ToPintsel = 19U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin20ToPintsel = 20U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin21ToPintsel = 21U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin22ToPintsel = 22U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin23ToPintsel = 23U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin24ToPintsel = 24U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin25ToPintsel = 25U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin26ToPintsel = 26U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin27ToPintsel = 27U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin28ToPintsel = 28U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin29ToPintsel = 29U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin30ToPintsel = 30U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_GpioPort0Pin31ToPintsel = 31U + (PINTSEL_PMUX_ID << PMUX_SHIFT),
    /*!< DMA ITRIG. */
    kINPUTMUX_FspReqToDma = 0U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_RtcSecondIrqToDma = 1U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Sct0DmaReq0ToDma = 2U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Sct0DmaReq1ToDma = 3U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer0M0ToDma = 4U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer0M1ToDma = 5U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer1M0ToDma = 6U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer1M1ToDma = 7U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer2M0ToDma = 8U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer2M1ToDma = 9U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer3M0ToDma = 10U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Ctimer3M1ToDma = 11U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt0ToDma = 12U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt1ToDma = 13U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt2ToDma = 14U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_PinInt3ToDma = 15U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig0ToDma = 16U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig1ToDma = 17U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig2ToDma = 18U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_Otrig3ToDma = 19U + (DMA_TRIG0_PMUX_ID << PMUX_SHIFT),
    /*!< DMA OTRIG. */
    kINPUTMUX_DmaFlexcomm0RxTrigoutToTriginChannels = 0U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm0TxTrigoutToTriginChannels = 1U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm1RxTrigoutToTriginChannels = 2U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm1TxTrigoutToTriginChannels = 3U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm2RxTrigoutToTriginChannels = 4U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm2TxTrigoutToTriginChannels = 5U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm3RxTrigoutToTriginChannels = 6U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaFlexcomm3TxTrigoutToTriginChannels = 7U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaSpifiTrigoutToTriginChannels = 8U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaPropRxTrigoutToTriginChannels = 9U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaPropTxTrigoutToTriginChannels = 10U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaLow256KFlashTxTrigoutToTriginChannels = 11U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaHigh256KFlashTxTrigoutToTriginChannels = 12U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaDacTxTrigoutToTriginChannels = 13U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaAdcRxTrigoutToTriginChannels = 14U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaCsReqTrigoutToTriginChannels = 15U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaMux0TrigoutToTriginChannels = 16U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaMux1TrigoutToTriginChannels = 17U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaMux2TrigoutToTriginChannels = 18U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
    kINPUTMUX_DmaMux3TrigoutToTriginChannels = 19U + (DMA_OTRIG_PMUX_ID << PMUX_SHIFT),
} inputmux_connection_t;

/*@}*/

#endif /* _FSL_INPUTMUX_CONNECTIONS_ */
