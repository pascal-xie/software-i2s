/*
** ###################################################################
**     Version:             rev. 1.0, 2016-06-27
**     Build:               b160620
**
**     Abstract:
**         Chip specific module features.
**
**     Copyright (c) 2016 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2016-06-27)
**         Initial version.
**
** ###################################################################
*/

#ifndef _QN908X_FEATURES_H_
#define _QN908X_FEATURES_H_

/* SOC module features */

/* @brief ACMP availability on the SoC. */
#define FSL_FEATURE_SOC_ACMP_COUNT (2)
/* @brief ADC availability on the SoC. */
#define FSL_FEATURE_SOC_ADC_COUNT (1)
/* @brief BOD availability on the SoC. */
#define FSL_FEATURE_SOC_BOD_COUNT (1)
/* @brief CAL availability on the SoC. */
#define FSL_FEATURE_SOC_CAL_COUNT (1)
/* @brief CRC availability on the SoC. */
#define FSL_FEATURE_SOC_CRC_COUNT (1)
/* @brief CS availability on the SoC. */
#define FSL_FEATURE_SOC_CS_COUNT (1)
/* @brief DAC availability on the SoC. */
#define FSL_FEATURE_SOC_DAC_COUNT (1)
/* @brief DMA availability on the SoC. */
#define FSL_FEATURE_SOC_DMA_COUNT (1)
/* @brief FLASH availability on the SoC. */
#define FSL_FEATURE_SOC_FLASH_COUNT (1)
/* @brief FLEXCOMM availability on the SoC. */
#define FSL_FEATURE_SOC_FLEXCOMM_COUNT (4)
/* @brief FSP availability on the SoC. */
#define FSL_FEATURE_SOC_FSP_COUNT (1)
/* @brief GPIO availability on the SoC. */
#define FSL_FEATURE_SOC_GPIO_COUNT (1)
/* @brief I2C availability on the SoC. */
#define FSL_FEATURE_SOC_I2C_COUNT (2)
/* @brief INPUTMUX availability on the SoC. */
#define FSL_FEATURE_SOC_INPUTMUX_COUNT (1)
/* @brief QDEC availability on the SoC. */
#define FSL_FEATURE_SOC_QDEC_COUNT (2)
/* @brief RNG availability on the SoC. */
#define FSL_FEATURE_SOC_RNG_COUNT (1)
/* @brief RTC availability on the SoC. */
#define FSL_FEATURE_SOC_RTC_COUNT (1)
/* @brief SCT availability on the SoC. */
#define FSL_FEATURE_SOC_SCT_COUNT (1)
/* @brief SPI availability on the SoC. */
#define FSL_FEATURE_SOC_SPI_COUNT (2)
/* @brief SPIFI availability on the SoC. */
#define FSL_FEATURE_SOC_SPIFI_COUNT (1)
/* @brief SYSCON availability on the SoC. */
#define FSL_FEATURE_SOC_SYSCON_COUNT (1)
/* @brief TIMER availability on the SoC. */
#define FSL_FEATURE_SOC_CTIMER_COUNT (4)
/* @brief USART availability on the SoC. */
#define FSL_FEATURE_SOC_USART_COUNT (2)
/* @brief USB availability on the SoC. */
#define FSL_FEATURE_SOC_USB_COUNT (1)
/* @brief WDT availability on the SoC. */
#define FSL_FEATURE_SOC_WDT_COUNT (1)

/* DMA module features */

/* @brief Number of channels */
#define FSL_FEATURE_DMA_NUMBER_OF_CHANNELS (20)

/* PINT module features */

/* @brief Number of connected outputs */
#define FSL_FEATURE_PINT_NUMBER_OF_CONNECTED_OUTPUTS (4)

/* CTIMER module features */

/* @brief Has CTIMER IR_CR3INT (register bit IR[CR3INT]). */
#define FSL_FEATURE_CTIMER_HAS_IR_CR3INT (0)
/* @brief Has CTIMER CCR_CAP3 (register bits CCR[CAP3RE][CAP3FE][CAP3I]). */
#define FSL_FEATURE_CTIMER_HAS_CCR_CAP3 (0)

/* SCT module features */

/* @brief Number of events */
#define FSL_FEATURE_SCT_NUMBER_OF_EVENTS (10)
/* @brief Number of states */
#define FSL_FEATURE_SCT_NUMBER_OF_STATES (10)
/* @brief Number of match capture */
#define FSL_FEATURE_SCT_NUMBER_OF_MATCH_CAPTURE (10)

/* FLASH module features */

/* @brief Flash size in bytes */
#define FSL_FEATURE_FLASH_SIZE_BYTES (512 * 1024)
/* @brief Flash page size in bytes */
#define FSL_FEATURE_FLASH_PAGE_SIZE_BYTES (2048)
/* @brief Flash base address */
#define FSL_FEATURE_FLASH_BASE_ADDR (0x21000000U)
/* @brief Flash read base address */
#define FSL_FEATURE_FLASH_READ_BASE_ADDR (0x31000000U)
/* @brief Flash lock bit address */
#define FSL_FEATURE_FLASH_LOCK_BIT_STORE_ADDR (0x2107F800U)
/* @brief Flash information page address */
#define FSL_FEATURE_FLASH_INFO_BASE_ADDR (0x210B0000U)

/* @brief CRC for boot info (4 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_BOOT_CRC (0x210B0700U)
/* @brief Boot version (4 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_BOOT_VERSION (0x210B0704U)
/* @brief Boot feature (4 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_BOOT_FEATURE (0x210B0708U)

/* @brief Temperature sensor calibration value (4 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_TEMP_CAL (0x210B07F0U)
/* @brief Bandgap voltage for ADC reference calibration (4 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_BANDGAP_VOL (0x210B07F4U)
/* @brief Vendor bluetooth address(MAC) (6 Bytes) */
#define FSL_FEATURE_FLASH_ADDR_OF_VENDOR_BD_ADDR (0x210B07FAU)

#endif /* _QN908X_FEATURES_H_ */
