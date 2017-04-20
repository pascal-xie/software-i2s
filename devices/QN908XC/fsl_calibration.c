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

#include "fsl_device_registers.h"
#include "fsl_calibration.h"
#include "fsl_power.h"
#include "clock_config.h" /* for BOARD_XTAL0_CLK_HZ */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void CALIB_CalibPLL48M(void)
{
    uint32_t code;

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_REF_CAL_DONE_INT_MASK;

    /* start calib */
    CALIB->START = CALIB_START_REF_CLB_START_MASK;
    /* wait until done */
    while (!(CALIB->INT_RAW & CALIB_INT_RAW_REF_CAL_DONE_INT_MASK))
        ;

    /* write code to cfg */
    code = CALIB->RCO_RC_REF_OSC_CODE & CALIB_RCO_RC_REF_OSC_CFG_PLL48_ENREF_CFG_MASK;
    CALIB->RCO_RC_REF_OSC_CFG &=
        (~(CALIB_RCO_RC_REF_OSC_CFG_REF_CAL_DIS_MASK | CALIB_RCO_RC_REF_OSC_CFG_PLL48_ENREF_CFG_MASK));
    CALIB->RCO_RC_REF_OSC_CFG |= (CALIB_RCO_RC_REF_OSC_CFG_REF_CAL_DIS_MASK | code);

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_REF_CAL_DONE_INT_MASK;
}

/*!
 * @brief Calibration of RC.
 *
 * @param None.
 */
static void CALIB_CalibRC(void)
{
    uint32_t code;
    uint32_t i;

    /* power on for rc calib */
    SYSCON->PMU_CTRL2 &= ~SYSCON_PMU_CTRL2_RC_CAL_DIS_MASK;

    /* 1->0 to start rc calib */
    CALIB->RCO_RC_REF_OSC_CFG = (CALIB->RCO_RC_REF_OSC_CFG | CALIB_RCO_RC_REF_OSC_CFG_RC_CAL_REQ_MASK);
    CALIB->RCO_RC_REF_OSC_CFG = (CALIB->RCO_RC_REF_OSC_CFG & ~CALIB_RCO_RC_REF_OSC_CFG_RC_CAL_REQ_MASK);

    /* only rc calib need delay before done, other calibs do not need. */
    for (i = 0U; i < 9000U; ++i)
    {
        __asm("NOP");
    }

    /* wait until done */
    while (!(CALIB->DONE & CALIB_DONE_RC_CAL_DONE_MASK))
        ;

    /* write code to cfg, and disable rc calib */
    code = CALIB->RCO_RC_REF_OSC_CODE & CALIB_RCO_RC_REF_OSC_CFG_CAU_RC_CAL_REG_IN_MASK;
    CALIB->RCO_RC_REF_OSC_CFG =
        (CALIB->RCO_RC_REF_OSC_CFG &
         ~(CALIB_RCO_RC_REF_OSC_CFG_CAU_RC_CAL_REG_IN_MASK | CALIB_RCO_RC_REF_OSC_CFG_CAU_RC_CAL_DIS_MASK)) |
        code | CALIB_RCO_RC_REF_OSC_CFG_CAU_RC_CAL_DIS(1U);

    /* power off for rc calib */
    SYSCON->PMU_CTRL2 |= SYSCON_PMU_CTRL2_RC_CAL_DIS_MASK;
}

/*!
 * @brief Calibration of external High Frequency(16M or 32M) Crystal.
 *
 * Calibrates the XTAL's current for lower power consumption. Has nothing to do with XTAL's precision.
 *
 * @param None.
 */
static void CALIB_CalibXTAL(void)
{
    uint32_t code;

    /* power on OSC32M for XTAL calib */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(0U));

    /* switch to OSC32M, and divide it to 16M */
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~(SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK | SYSCON_CLK_CTRL_CLK_OSC32M_DIV_MASK)) |
                       SYSCON_CLK_CTRL_SYS_CLK_SEL(0U) | SYSCON_CLK_CTRL_CLK_OSC32M_DIV(1U);

    CALIB->CTRL = (CALIB->CTRL & ~(CALIB_CTRL_XTL_PO_TIM_MASK | CALIB_CTRL_XTL_CAL_TIM_MASK)) |
                  CALIB_CTRL_XTL_PO_TIM(3U)     /* XTL_PO_TIM_4MS */
                  | CALIB_CTRL_XTL_CAL_TIM(2U); /* XTL_CAL_TIM_10MS */

    CALIB->INT_RAW = CALIB_INT_RAW_XTL_CAL_DONE_INT_MASK;
    CALIB->START = CALIB_START_XTL_CLB_START_MASK;

    while (!(CALIB->INT_RAW & CALIB_INT_RAW_XTL_CAL_DONE_INT_MASK))
        ;
    code = CALIB->XTL_CODE & CALIB_XTL_CFG_XTL_XICTRL_CFG_MASK;
    CALIB->XTL_CFG = (CALIB->XTL_CFG & ~(CALIB_XTL_CFG_XTL_XICTRL_CFG_MASK | CALIB_XTL_CFG_XTL_CAL_DIS_MASK)) |
                     CALIB_XTL_CFG_XTL_CAL_DIS(1U) | code;

    CALIB->INT_RAW = CALIB_INT_RAW_XTL_CAL_DONE_INT_MASK;

    /* switch back to XTAL */
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_SYS_CLK_SEL_MASK) | SYSCON_CLK_CTRL_SYS_CLK_SEL(1U);
}

/*!
 * @brief Calibration of internal High Frequency(32M) OSC.
 *
 * Calibrates the OSC32M to make it more precise.
 *
 * @param None.
 */
#if defined(CFG_QN908XA)
static void CALIB_CalibOSC32M(void)
{
    int i;
    uint32_t code;

    /* power on OSC32M */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(0U));

    for (i = 0x1F; i >= 0; i--)
    {
        /* clear int */
        CALIB->INT_RAW = CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK;

        /* disable the code */
        CALIB->RCO_RC_REF_OSC_CFG |= CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;

        code = CALIB->RCO_RC_REF_OSC_CFG & ~CALIB_RCO_RC_REF_OSC_CFG_CAU_OSC_CUR_CFG_MASK;
        code |= (i << CALIB_RCO_RC_REF_OSC_CFG_CAU_OSC_CUR_CFG_SHIFT);
        CALIB->RCO_RC_REF_OSC_CFG = code;

        /* start calib */
        CALIB->START = CALIB_START_OSC_CLB_START_MASK;
        /* wait until done */
        while (!(CALIB->INT_RAW & CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK))
            ;

        /* clear disable of the code */
        CALIB->RCO_RC_REF_OSC_CFG &= ~CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;
        /* clear int */
        CALIB->INT_RAW = CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK;
        /* check the code status */
        code = CALIB->RCO_RC_REF_OSC_CODE & CALIB_RCO_RC_REF_OSC_CODE_CAU_OSC_CUR_MASK;

        if (code != 0U)
        {
            CALIB->RCO_RC_REF_OSC_CFG |= CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;
            break;
        }
    }

    /* power off OSC32M */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(1U));
}
#elif defined(CFG_QN908XC)
#define OSC_CAL_OV_TIME (0x20U)
static void CALIB_CalibOSC32M(void)
{
    int i, j;
    uint32_t code;
    uint32_t osc_cur = 0;

    /* power on OSC32M */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(0U));

    for (i = 0x4; i >= 0; i--)
    {
        /* clear int */
        CALIB->INT_RAW = CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK;

        /* disable the code */
        CALIB->RCO_RC_REF_OSC_CFG |= CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;

        osc_cur |= (1 << i);
        code = CALIB->RCO_RC_REF_OSC_CFG & ~CALIB_RCO_RC_REF_OSC_CFG_CAU_OSC_CUR_CFG_MASK;
        code |= (osc_cur << CALIB_RCO_RC_REF_OSC_CFG_CAU_OSC_CUR_CFG_SHIFT);
        CALIB->RCO_RC_REF_OSC_CFG = code;

        /* start calib */
        CALIB->START = CALIB_START_OSC_CLB_START_MASK;
        /* wait until done */
        j = 0;
        while(1)
        {
            /* calibration done */
            if (CALIB->INT_RAW & CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK)
            {
                break;
            }
            /* over time, need to increase code*/
            else if (j == OSC_CAL_OV_TIME)
            {
                break;
            }
            j++;
        }
        
        /* clear disable of the code */
        CALIB->RCO_RC_REF_OSC_CFG &= ~CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;
        /* clear int */
        CALIB->INT_RAW = CALIB_INT_RAW_OSC_CAL_DONE_INT_MASK;
        /* check the code status */
        code = CALIB->RCO_RC_REF_OSC_CODE & CALIB_RCO_RC_REF_OSC_CODE_CAU_OSC_CUR_MASK;

        /* code is large, current bit need to change to 0*/
        if ((!((code >> CALIB_RCO_RC_REF_OSC_CODE_CAU_OSC_CUR_SHIFT) & 0x10)))// && (j != OSC_CAL_OV_TIME))
        {
            osc_cur &= ~(1 << i);
        }
    }
    CALIB->RCO_RC_REF_OSC_CFG |= CALIB_RCO_RC_REF_OSC_CFG_OSC_CAL_DIS_MASK;

    /* power off OSC32M */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_OSC32M_DIS_MASK, SYSCON_PMU_CTRL1_OSC32M_DIS(1U));
}
#endif
/*!
 * @brief Calibration of internal Low Frequency(32.000K) RCO.
 *
 * Calibrates the RCO32K to make it more precise.
 *
 * @param None.
 */
static void CALIB_CalibRCO32K(void)
{
    uint32_t code;

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_RCO_CAL_DONE_INT_MASK;
    /* power on RCO32K */
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_RCO32K_DIS_MASK, SYSCON_PMU_CTRL1_RCO32K_DIS(0U));

    /* switch to RCO32K */
    SYSCON->CLK_CTRL = (SYSCON->CLK_CTRL & ~SYSCON_CLK_CTRL_CLK_32K_SEL_MASK) | SYSCON_CLK_CTRL_CLK_32K_SEL(1U);

    CALIB->RCO_RC_REF_OSC_CFG &= ~CALIB_RCO_RC_REF_OSC_CFG_RCO_CAL_DIS_MASK;

    /* start calib */
    CALIB->START = CALIB_START_RCO_CLB_START(1U);
    /* wait until done */
    while (!(CALIB->INT_RAW & CALIB_INT_RAW_RCO_CAL_DONE_INT_MASK))
        ;
    /* write code to cfg */
    code = CALIB->RCO_RC_REF_OSC_CODE & CALIB_RCO_RC_REF_OSC_CFG_CAU_RCO_CAP_CFG_MASK;
    CALIB->RCO_RC_REF_OSC_CFG =
        (CALIB->RCO_RC_REF_OSC_CFG &
         ~(CALIB_RCO_RC_REF_OSC_CFG_CAU_RCO_CAP_CFG_MASK | CALIB_RCO_RC_REF_OSC_CFG_RCO_CAL_DIS_MASK)) |
        code | CALIB_RCO_RC_REF_OSC_CFG_RCO_CAL_DIS(1U);

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_RCO_CAL_DONE_INT_MASK;
}

static void CALIB_PowerOn(void)
{
    uint32_t code;

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_PO_CAL_DONE_INT_MASK;
    /* start calib */
    CALIB->START = CALIB_START_PO_CLB_START(1U);
    /* wait until done */
    while (!(CALIB->INT_RAW & CALIB_INT_RAW_PO_CAL_DONE_INT_MASK))
        ;

    /* clear int */
    CALIB->INT_RAW = CALIB_INT_RAW_PO_CAL_DONE_INT_MASK;

    code = CALIB->VCOA_KVCO2M_CODE & CALIB_VCOA_KVCO2M_CFG_KCALF2M_CFG_MASK;
    CALIB->VCOA_KVCO2M_CFG = (CALIB->VCOA_KVCO2M_CFG & ~CALIB_VCOA_KVCO2M_CFG_KCALF2M_CFG_MASK) | code;

    CALIB->VCOF_KVCO_CFG =
        CALIB->VCOF_KVCO_PO_CODE | CALIB_VCOF_KVCO_CFG_VCOF_CAL_DIS(1U) | CALIB_VCOF_KVCO_CFG_KVCO_DIS(1U);

    BLEDP->DP_TOP_SYSTEM_CTRL = (BLEDP->DP_TOP_SYSTEM_CTRL &
                                 ~(BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ_MASK |
                                   BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL_MASK)) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ(0U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ(1U) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL(1U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL(1U);

    /* start calib */
    CALIB->VCOA_KVCO2M_CFG = (CALIB->VCOA_KVCO2M_CFG | CALIB_VCOA_KVCO2M_CFG_VCOA_CAL_REQ_MASK);
    CALIB->VCOA_KVCO2M_CFG = (CALIB->VCOA_KVCO2M_CFG & ~CALIB_VCOA_KVCO2M_CFG_VCOA_CAL_REQ_MASK);

    /* wait until done */
    while (!(CALIB->DONE & CALIB_DONE_VCOA_CAL_DONE_MASK))
        ;

    BLEDP->DP_TOP_SYSTEM_CTRL = (BLEDP->DP_TOP_SYSTEM_CTRL &
                                 ~(BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ_MASK |
                                   BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL_MASK)) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ(1U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ(0U) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL(1U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL(1U);

    /* start calib */
    CALIB->VCOA_KVCO2M_CFG = (CALIB->VCOA_KVCO2M_CFG | CALIB_VCOA_KVCO2M_CFG_VCOA_CAL_REQ_MASK);
    CALIB->VCOA_KVCO2M_CFG = (CALIB->VCOA_KVCO2M_CFG & ~CALIB_VCOA_KVCO2M_CFG_VCOA_CAL_REQ_MASK);

    /* wait until done */
    while (!(CALIB->DONE & CALIB_DONE_VCOA_CAL_DONE_MASK))
        ;

    CALIB->VCOA_KVCO2M_CFG = CALIB->VCOA_KVCO2M_CODE | CALIB_VCOA_KVCO2M_CFG_VCOA_CAL_DIS(1U);

    BLEDP->DP_TOP_SYSTEM_CTRL = (BLEDP->DP_TOP_SYSTEM_CTRL &
                                 ~(BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ_MASK |
                                   BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL_MASK | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL_MASK)) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_REQ(0U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_REQ(0U) |
                                BLEDP_DP_TOP_SYSTEM_CTRL_TX_EN_SEL(0U) | BLEDP_DP_TOP_SYSTEM_CTRL_RX_EN_SEL(0U);
#if defined(CFG_QN908XA)
    uint32_t rx_slope_cnt;

    rx_slope_cnt = CALIB->VCOF_CNT_SLOPE;
    rx_slope_cnt ^=
        ((0x40U << CALIB_VCOF_CNT_SLOPE_TX_VCOF_CNT_SHIFT) | (0x40U << CALIB_VCOF_CNT_SLOPE_RX_VCOF_CNT_SHIFT));
    rx_slope_cnt |= (0x1fU << CALIB_VCOF_CNT_SLOPE_RX_SLOPE_SHIFT);

    CALIB->VCOF_CNT_SLOPE = rx_slope_cnt;
#endif
    CALIB->VCO_MOD_CFG = 0U;
}

/*!
 * @brief Calibration at system startup.
 *
 * Do a series of calibrations at system startup.
 * Note: bus clock(i.e. AHB clock) must be set to 16M.
 *
 * @param None.
 */
void CALIB_SystemCalib(void)
{
    CLOCK_AttachClk((BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? k32M_to_XTAL_CLK : k16M_to_XTAL_CLK);

    /* Configure AHB clock, AHBCLK = SYSCLK/(div+1) */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, (BOARD_XTAL0_CLK_HZ == CLK_XTAL_16MHZ) ? 0U : 1U);

    /* SYSCLK comes from XTAL */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);

    /* enable ble clock and calibration clock */
    SYSCON->CLK_EN = SYSCON_CLK_EN_CLK_BLE_EN_MASK | SYSCON_CLK_EN_CLK_CAL_EN_MASK;

#if defined(CFG_QN908XA)
    /* workaround for buck's 10+mA glitch. */
    CALIB->LO0 = (CALIB->LO0 & ~CALIB_LO0_VCO_SAMP_EN_MASK);
#endif

    /* workaround for buck's short inductor
     * fix for buck's frequency (tune tmos to adjust efficiency of buck)
     * increase buck constant on time to get better RX and TX performance
     * only use bjt comparator to save buck power */
    SYSCON->BUCK =
        (SYSCON->BUCK & ~(SYSCON_BUCK_BUCK_IND_USE_EN_MASK | SYSCON_BUCK_BUCK_TMOS_MASK | SYSCON_BUCK_BUCK_ISEL_MASK)) |
        SYSCON_BUCK_BUCK_IND_USE_EN(0U) | SYSCON_BUCK_BUCK_TMOS(0x0EU) | SYSCON_BUCK_BUCK_ISEL(2U);

    /* using diff lo clk */
    CALIB->LO1 &= ~CALIB_LO1_DIV_DIFF_CLK_LO_DIS_MASK;

    /* AA_ERROR */
    BLEDP->DP_AA_ERROR_CTRL = 0x0000000EU;

    CALIB_CalibRC();
    CALIB_CalibXTAL();
    CALIB_CalibOSC32M();
    CALIB_CalibRCO32K();

    /* slope_tx & slope_rx write error */
    CALIB_PowerOn();

#if defined(CFG_QN908XA)
    /* workaround for buck's 10+mA glitch. */
    CALIB->LO0 |= CALIB_LO0_VCO_SAMP_EN_MASK;

    /* PPF_IQSW bit default value should be 1 */
    CALIB->PLL48_PPF |= CALIB_PLL48_PPF_PPF_IQSW_MASK;
#endif

    /* imr = 1 */
    CALIB->VCO_MOD_CFG |= CALIB_VCO_MOD_CFG_IMR_MASK;

    /* bypass hop calibration delay, then it is a little earlier than tx/rx */
    CALIB->CAL_DLY |= CALIB_CAL_DLY_HOP_DLY_BP_MASK;

    /* change PA power to 0dBm */
    SYSCON->ANA_CTRL0 = (SYSCON->ANA_CTRL0 & ~SYSCON_ANA_CTRL0_PA_POWER_MASK) | SYSCON_ANA_CTRL0_PA_POWER(0x89U);

    /* power down VREGA in mcu mode */
#if defined(CFG_QN908XA)
    SYSCON->PMU_CTRL2 |= SYSCON_PMU_CTRL2_VREG_A_DIS_MASK;
#endif
    /* power down the analog part at rx state when the air signal is over */
    SYSCON->PMU_CTRL2 = (SYSCON->PMU_CTRL2 & ~SYSCON_PMU_CTRL2_RX_EN_SEL_MASK) |
                        SYSCON_PMU_CTRL2_RX_EN_SEL(0U);
}
