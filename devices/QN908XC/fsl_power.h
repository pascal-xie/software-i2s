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

#ifndef _FSL_POWER_H_
#define _FSL_POWER_H_

#include "fsl_common.h"

/*! @addtogroup ksdk_common */
/*! @{ */

/*! @file */

/*******************************************************************************
 * Definitions
 *****************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief QN9080 power version 2.0.0. */
#define FSL_QN9080_POWER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief Power name used for POWER_EnablePD/POWER_DisablePD. */
/*------------------------------------------------------------------------------
 pd_bit_t definition:
------------------------------------------------------------------------------*/
#define MAKE_PD_BITS(reg, slot) ((reg << 8) | slot)
#define PMUCTRL0 0x0
#define PMUCTRL1 0x1
#define PMUCTRL2 0x2

typedef enum pd_bits
{
    kPDRUNCFG_PD_MEM0 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM0_DIS_SHIFT),
    kPDRUNCFG_PD_MEM1 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM1_DIS_SHIFT),
    kPDRUNCFG_PD_MEM2 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM2_DIS_SHIFT),
    kPDRUNCFG_PD_MEM3 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM3_DIS_SHIFT),
    kPDRUNCFG_PD_MEM4 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM4_DIS_SHIFT),
    kPDRUNCFG_PD_MEM5 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM5_DIS_SHIFT),
    kPDRUNCFG_PD_MEM6 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM6_DIS_SHIFT),
    kPDRUNCFG_PD_MEM7 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM7_DIS_SHIFT),
    kPDRUNCFG_PD_MEM8 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM8_DIS_SHIFT),
    kPDRUNCFG_PD_MEM9 = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MEM9_DIS_SHIFT),
    kPDRUNCFG_PD_BLE = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_BLE_DIS_SHIFT),
    kPDRUNCFG_PD_FIR = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_FIR_DIS_SHIFT),
    kPDRUNCFG_PD_FSP = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_FSP_DIS_SHIFT),
    kPDRUNCFG_ANA_PWROFF = MAKE_PD_BITS(PMUCTRL0, SYSCON_PMU_CTRL0_MCU_MODE_SHIFT),

    kPDRUNCFG_PD_RCO32K = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_RCO32K_DIS_SHIFT),
    kPDRUNCFG_PD_XTAL32K = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_XTAL32K_DIS_SHIFT),
    kPDRUNCFG_PD_XTAL = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_XTAL_DIS_SHIFT),
    kPDRUNCFG_PD_OSC32M = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_OSC32M_DIS_SHIFT),
    kPDRUNCFG_PD_USBPLL = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_USBPLL_DIS_SHIFT),
    kPDRUNCFG_PD_ADC_BUF = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_ADC_BUF_DIS_SHIFT),
    kPDRUNCFG_PD_ADC_BG = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_ADC_BG_DIS_SHIFT),
    kPDRUNCFG_PD_ADC = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_ADC_DIS_SHIFT),
    kPDRUNCFG_PD_ADC_VCM = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_ADC_VCM_DIS_SHIFT),
    kPDRUNCFG_PD_ADC_VREF = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_ADC_VREF_DIS_SHIFT),
    kPDRUNCFG_PD_DAC = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_DAC_DIS_SHIFT),
    kPDRUNCFG_PD_CAP_SEN = MAKE_PD_BITS(PMUCTRL1, SYSCON_PMU_CTRL1_CAP_SEN_DIS_SHIFT),

#if defined(CFG_QN908XA)
    kPDRUNCFG_PD_VREG_A = MAKE_PD_BITS(PMUCTRL2, SYSCON_PMU_CTRL2_VREG_A_DIS_SHIFT),
#endif
} pd_bit_t;

/*! @brief Power modes. */
typedef enum _power_mode
{
    kPmActive,     /*!< CPU is executing */
    kPmSleep,      /*!< CPU clock is gated */
    kPmPowerDown0, /*!< Power is shut down except for always on domain, 32k clock and selected wakeup source */
    kPmPowerDown1  /*!< Power is shut down except for always on domain and selected wakeup source */
} power_mode_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Work around for PMU_CTRL1's hardware issue. Refer to Errata PMU.1.
 */
void POWER_WritePmuCtrl1(SYSCON_Type *base, uint32_t mask, uint32_t value);

/*!
 * @brief Enable power down. Note that enabling the bit powers down the peripheral
 *
 * @param en any value defined by enum @pd_bit_t
 */
void POWER_EnablePD(pd_bit_t en);

/*!
 * @brief Disable power down. Note that disabling the bit powers up the peripheral
 *
 * @param en any value defined by enum @pd_bit_t
 */
void POWER_DisablePD(pd_bit_t en);

/*!
 * @brief Enable or disable DC-DC.
 *
 * @param flag true to enable the DC-DC, false to disable.
 */
void POWER_EnableDCDC(bool flag);

/*!
 * @brief Enable or disable ADC power.
 *
 * @param flag true to enable the ADC power, false to disable.
 */
void POWER_EnableADC(bool flag);

/*!
 * @brief Latch the output status and level of GPIO during power down.
 *
 * During power down, GPIO registers at GPIOA_BASE and GPIOB_BASE will get lost,
 * and GPIO controller loses control of the pads. This result in uncetain level
 * on pads during power down. Use this function to capture the current GPIO
 * output status and level to always-on registers, SYSCON->PIO_CAP_OUT0/1 and
 * SYSCON->PIO_CAP_OE0/1, and hand over control of pads to these always-on registers.
 */
void POWER_LatchIO(void);

/*!
 * @brief Restore the gpio output control registers and take over controll of gpio pads.
 *
 * Should be called in pair with POWER_LatchIO().
 */
void POWER_RestoreIO(void);

/*!
 * @brief Configure the SWDIO to gpio and as wakeup source before power down.
 *
 * By using this, the chip can be waked up by swd debugger from power down.
 */
void POWER_EnableSwdWakeup(void);

/*!
 * @brief Recover swdio's pin-mux configuration that swd access is availale
 *          after waking up from power down.
 */
void POWER_RestoreSwd(void);

/*!
 * @brief Check if any wake io is active.
 */
bool POWER_GpioActiveRequest(void);

/*!
 * @brief Make the chip enter sleep mode.
 */
void POWER_EnterSleep(void);

/*!
 * @brief Make the chip enter power down mode.
 *
 * If 32k clock source is on before calling this, the chip will go to power down 0.
 * If 32k clock source is turned off before this, the chip will go to power down 1.
 *
 * @param exclude_from_pd when entering power down, leave the modules indicated by exclude_from_pd on.
 */
void POWER_EnterPowerDown(uint32_t exclude_from_pd);

/*!
 * @ brief Init of power management unit.
 */
void POWER_Init(void);

/*!
 * @brief Register address to bootloader.
 *
 * This is an ROM API. The ram function address registered will be stored in a
 * global variable in the reserved ram area for bootloader. After waking up from
 * power down, bootloader will jump to the function registered.
 *
 * @param ram_addr a function address in ram area.
 */
typedef void (*p_POWER_RegisterWakeupEntry)(uint32_t ram_addr);
static inline void POWER_RegisterWakeupEntry(uint32_t ram_addr)
{
#if defined(CFG_QN908XC)
    ((p_POWER_RegisterWakeupEntry)(0x03025a59))(ram_addr);
#else
    ((p_POWER_RegisterWakeupEntry)(0x0302d865))(ram_addr);
#endif
}

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _FSL_POWER_H_ */
