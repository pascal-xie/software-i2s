/*
** ###################################################################
**     Processors:          QN908X
**                          QN908X
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    QN908X User manual Rev. 1.0 16 February 2016
**     Version:             rev. 1.0, 2016-04-29
**     Build:               b160525
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
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
**     - rev. 1.0 (2016-04-29)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file system_QN908X
 * @version 1.0
 * @date 2016-04-29
 * @brief Device specific configuration file for QN908X (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"

extern void *__Vectors;

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  Setup the microcontroller system.
 *****************************************************************************************
 */
void SystemInit(void)
{
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) || (defined(__VFP_FP__) && !defined(__SOFTFP__))
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10, CP11 Full Access */
#endif                                                 /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */
    SCB->VTOR = (uint32_t)&__Vectors;

    /* Disable chip test mode */
    *((uint32_t *)0x40000080) &= 0x0FFFFFFF;

    /* Disable watchdog timer */
    WDT->LOCK = 0x1ACCE551;
    WDT->CTRL = 0;
    WDT->LOCK = 0;

    /* Save power in active mode */
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_CGBYPASS_MASK) | SYSCON_CLK_CTRL_CGBYPASS(0U);

    /* speed up the startup of XTAL by decrease load cap */
    SYSCON->XTAL_CTRL = (SYSCON->XTAL_CTRL &
                         ~(SYSCON_XTAL_CTRL_XTAL_SU_CA_REG_MASK | SYSCON_XTAL_CTRL_XTAL_SU_CB_REG_MASK |
                           SYSCON_XTAL_CTRL_XTAL_XCUR_BOOST_REG_MASK)) |
                        SYSCON_XTAL_CTRL_XTAL_SU_CA_REG(0U) | SYSCON_XTAL_CTRL_XTAL_SU_CB_REG(0U) |
                        SYSCON_XTAL_CTRL_XTAL_XCUR_BOOST_REG_MASK;

    /* change crystal load cap to (0.35 * 0x08 + 5) = 7.8pF for DK board, and half of default voltage*/
    SYSCON->ANA_CTRL0 = (SYSCON->ANA_CTRL0 & ~(SYSCON_ANA_CTRL0_XTAL_LOAD_CAP_MASK | SYSCON_ANA_CTRL0_XTAL_AMP_MASK)) |
                        SYSCON_ANA_CTRL0_XTAL_LOAD_CAP(8U) | SYSCON_ANA_CTRL0_XTAL_AMP(2U);

    /* Adjust mem & pmu voltage */
    /* if VDD_PMU_SET_ULTRA_LOW bit is set, SYSCON_ANA_CTRL1_VDD_MEM_SET_PDM_MASK will be ignored */
    SYSCON->ANA_CTRL1 = (SYSCON->ANA_CTRL1 &
                         ~(SYSCON_ANA_CTRL1_VDD_PMU_SET_PDM_MASK | SYSCON_ANA_CTRL1_VDD_PMU_SET_MASK |
                           SYSCON_ANA_CTRL1_VDD_MEM_SET_PDM_MASK | SYSCON_ANA_CTRL1_VDD_MEM_SET_MASK |
                           SYSCON_ANA_CTRL1_VDD_PMU_SET_ULTRA_LOW_MASK | SYSCON_ANA_CTRL1_IV_VREG11_SET_MASK |
                           SYSCON_ANA_CTRL1_DVREG11_SET_DIG_MASK)) |
                        SYSCON_ANA_CTRL1_VDD_PMU_SET_PDM(3U) | SYSCON_ANA_CTRL1_VDD_PMU_SET(3U) |
                        SYSCON_ANA_CTRL1_VDD_MEM_SET_PDM(3U) | SYSCON_ANA_CTRL1_VDD_MEM_SET(3U) |
                        SYSCON_ANA_CTRL1_VDD_PMU_SET_ULTRA_LOW(0U) | SYSCON_ANA_CTRL1_IV_VREG11_SET(2U) |
                        SYSCON_ANA_CTRL1_DVREG11_SET_DIG(2U);

    /* config flash to be powered down during power down mode, this is reset value */
    SYSCON->PMU_CTRL2 = SYSCON->PMU_CTRL2 | SYSCON_PMU_CTRL2_FLSH_PDM_DIS_MASK;

    /* flatten RX sensitivity across 2402-2480M */
    CALIB->RRF1 = (CALIB->RRF1 & ~(CALIB_RRF1_RRF_RX_INCAP1_MASK | CALIB_RRF1_RRF_LOAD_CAP_MASK)) |
                  CALIB_RRF1_RRF_RX_INCAP1(4U) | CALIB_RRF1_RRF_LOAD_CAP(6U);

    /* Sub module clock setting:
     * enable Data Path 16/8MHz clock(some of the flash operations need this, too)
     * enable BiV clock include RTC BiV register  */
    SYSCON->CLK_EN = SYSCON_CLK_EN_CLK_DP_EN_MASK | SYSCON_CLK_EN_CLK_BIV_EN_MASK;

    /* workaround for bootloader: set pin mux registers to reset value */
    SYSCON->PIO_FUNC_CFG[1] = 0x0U;
    SYSCON->PIO_FUNC_CFG[2] = 0x0U;

    /* workaround for bootloader: bootloader may enable these submodules */
    SYSCON->RST_SW_SET = SYSCON_RST_SW_SET_SET_FC0_RST_MASK | SYSCON_RST_SW_SET_SET_FC3_RST_MASK;
    SYSCON->RST_SW_CLR = SYSCON_RST_SW_CLR_CLR_FC0_RST_MASK | SYSCON_RST_SW_CLR_CLR_FC3_RST_MASK;
    SYSCON->CLK_DIS = SYSCON_CLK_DIS_CLK_FC0_DIS_MASK | SYSCON_CLK_DIS_CLK_FC3_DIS_MASK;

#if defined(CFG_QN908XA)
    /* Remove pullup from USB pins */
    SYSCON->PIO_PULL_CFG[1] =
        SYSCON->PIO_PULL_CFG[1] & ~(SYSCON_PIO_PULL_CFG_PA26_PULL_MASK | SYSCON_PIO_PULL_CFG_PA27_PULL_MASK);
    SYSCON->USB_CFG = ((1 << SYSCON_USB_CFG_DPPUEN_B_PHY_POL_SHIFT) | (0 << SYSCON_USB_CFG_DPPUEN_B_PHY_SEL_SHIFT) |
                       (0 << SYSCON_USB_CFG_USB_VBUS_SHIFT) | (1 << SYSCON_USB_CFG_USB_PHYSTDBY_SHIFT) |
                       (1 << SYSCON_USB_CFG_USB_PHYSTDBY_WEN_SHIFT));
#endif

    /* use close loop mode by default */
    CALIB->LO1 = (CALIB->LO1 | (CALIB_LO1_TX_PLLPFD_EN_MASK | CALIB_LO1_RX_PLLPFD_EN_MASK));

    /* Wait for XTAL ready */
    while (!(SYSCON->SYS_MODE_CTRL & SYSCON_SYS_MODE_CTRL_XTAL_RDY_MASK))
        ;
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate(void)
{
}

/// @} SYSTEM
