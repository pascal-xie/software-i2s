/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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

#include "fsl_common.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @ brief Run at 8M AHB no matter 16M or 32M XTAL is used.
 */
void BOARD_BootClockLSRUN(void)
{
    /* when CLK_XTAL_SEL is set to 1(means 32M xtal is used), XTAL_DIV is valid */
    CLOCK_SetClkDiv(kCLOCK_DivXtalClk, (BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? 1U : 0U);

    /* Configure AHB clock, AHBCLK = SYSCLK/(div+1) */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U);

    /* Configure APB clock, APBCLK = AHBCLK/(div+1)*/
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);

    /* Select XTAL clock frequency: 32M or 16M*/
    CLOCK_AttachClk((BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? k32M_to_XTAL_CLK : k16M_to_XTAL_CLK);

    /* SYSCLK comes from XTAL */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);

    /* Select 32k clock source: 32768Hz XTAL or 32000Hz RCO */
    CLOCK_AttachClk((BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ) ? kXTAL32K_to_32K_CLK : kRCO32K_to_32K_CLK);
}

/*!
 * @ brief Run at 16M AHB no matter 16M or 32M XTAL is used.
 */
void BOARD_BootClockRUN(void)
{
    /* when CLK_XTAL_SEL is set to 1(means 32M xtal is used), XTAL_DIV is valid */
    CLOCK_SetClkDiv(kCLOCK_DivXtalClk, (BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? 1U : 0U);

    /* Configure AHB clock, AHBCLK = SYSCLK/(div+1) */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 0U);

    /* Configure APB clock, APBCLK = AHBCLK/(div+1)*/
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);

    /* Select XTAL clock frequency: 32M or 16M*/
    CLOCK_AttachClk((BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? k32M_to_XTAL_CLK : k16M_to_XTAL_CLK);

    /* SYSCLK comes from XTAL */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);

    /* Select 32k clock source: 32768Hz XTAL or 32000Hz RCO */
    CLOCK_AttachClk((BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ) ? kXTAL32K_to_32K_CLK : kRCO32K_to_32K_CLK);
}

/*!
 * @ brief Run at 32M AHB when 32M XTAL is used.
 *
 * Note: Only use this function when 32M XTAL is used.
 */
void BOARD_BootClockHSRUN(void)
{
    /* Configure AHB clock, AHBCLK = SYSCLK/(div+1) */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 0U);

    /* Configure APB clock, APBCLK = AHBCLK/(div+1)*/
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);

    /* Select XTAL clock frequency: 32M or 16M*/
    CLOCK_AttachClk((BOARD_XTAL0_CLK_HZ == CLK_XTAL_32MHZ) ? k32M_to_XTAL_CLK : k16M_to_XTAL_CLK);

    /* SYSCLK comes from XTAL */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);

    /* Select 32k clock source: 32768Hz XTAL or 32000Hz RCO */
    CLOCK_AttachClk((BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ) ? kXTAL32K_to_32K_CLK : kRCO32K_to_32K_CLK);
}
