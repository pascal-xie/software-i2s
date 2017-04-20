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

#ifndef _FSL_RESET_H_
#define _FSL_RESET_H_

#include "fsl_common.h"

/*!
 * @addtogroup ksdk_common
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief Enumeration for peripheral reset control bits
 *
 * Defines the enumeration for peripheral reset control bits in PRESETCTRL/ASYNCPRESETCTRL registers
 */
typedef enum _SYSCON_RSTn
{
    kFC0_RST_SHIFT_RSTn = 0,     /**< Flexcomm Interface 0 reset control */
    kFC1_RST_SHIFT_RSTn = 1,     /**< Flexcomm Interface 1 reset control */
    kFC2_RST_SHIFT_RSTn = 2,     /**< Flexcomm Interface 2 reset control */
    kFC3_RST_SHIFT_RSTn = 3,     /**< Flexcomm Interface 3 reset control */
    kTIM0_RST_SHIFT_RSTn = 4,    /**< CTimer0 reset control */
    kTIM1_RST_SHIFT_RSTn = 5,    /**< CTimer1 reset control */
    kTIM2_RST_SHIFT_RSTn = 6,    /**< CTimer2 reset control */
    kTIM3_RST_SHIFT_RSTn = 7,    /**< CTimer3 reset control */
    kSCT0_RST_SHIFT_RSTn = 8,    /**< SCTimer/PWM 0 (SCT0) reset control */
    kWDT_RST_SHIFT_RSTn = 9,     /**< WDT reset control */
    kUSB_RST_SHIFT_RSTn = 10,    /**< USB reset control */
    kGPIO_RST_SHIFT_RSTn = 11,   /**< GPIO reset control */
    kRTC_RST_SHIFT_RSTn = 12,    /**< RTC reset control */
    kADC_RST_SHIFT_RSTn = 13,    /**< ADC reset control */
    kDAC_RST_SHIFT_RSTn = 14,    /**< DAC reset control */
    kCS_RST_SHIFT_RSTn = 15,     /**< Capacitive Sense reset control */
    kFSP_RST_SHIFT_RSTn = 16,    /**< FSP reset control */
    kDMA_RST_SHIFT_RSTn = 32,    /**< (Do not execute reset as default)DMA reset control */
    kPINT_RST_SHIFT_RSTn = 32,   /**< (Do not execute reset as default)Pin interrupt (PINT) reset control */
    kMUX_RST_SHIFT_RSTn = 32,    /**< (Do not execute reset as default)Input mux reset control */
    kQDEC0_RST_SHIFT_RSTn = 19,  /**< QDEC0 reset control */
    kQDEC1_RST_SHIFT_RSTn = 20,  /**< QDEC1 reset control */
    kSPIFI_RST_SHIFT_RSTn = 22,  /**< SPIFI reset control */
    kCAL_RST_SHIFT_RSTn = 25,    /**< Calibration reset control */
    kCPU_RST_SHIFT_RSTn = 26,    /**< CPU reset control */
    kBLE_RST_SHIFT_RSTn = 27,    /**< BLE reset control */
    kFLASH_RST_SHIFT_RSTn = 28,  /**< Flash reset control */
    kDP_RST_SHIFT_RSTn = 29,     /**< Data path reset control */
    kREG_RST_SHIFT_RSTn = 30,    /**< Retention register reset control */
    kREBOOT_RST_SHIFT_RSTn = 31, /**< Reboot reset control */
} SYSCON_RSTn_t;

#define FLEXCOMM_RSTS                                                                      \
    {                                                                                      \
        kFC0_RST_SHIFT_RSTn, kFC1_RST_SHIFT_RSTn, kFC2_RST_SHIFT_RSTn, kFC3_RST_SHIFT_RSTn \
    } /* Reset bits for FLEXCOMM peripheral */
#define CTIMER_RSTS                                                                            \
    {                                                                                          \
        kTIM0_RST_SHIFT_RSTn, kTIM1_RST_SHIFT_RSTn, kTIM2_RST_SHIFT_RSTn, kTIM3_RST_SHIFT_RSTn \
    } /* Reset bits for TIMER peripheral */
#define SCT_RSTS             \
    {                        \
        kSCT0_RST_SHIFT_RSTn \
    } /* Reset bits for SCT0 peripheral */
#define WDT_RSTS            \
    {                       \
        kWDT_RST_SHIFT_RSTn \
    } /* Reset bits for WDT peripheral */
#define USB_RSTS            \
    {                       \
        kUSB_RST_SHIFT_RSTn \
    } /* Reset bits for USB */
#define GPIO_RSTS            \
    {                        \
        kGPIO_RST_SHIFT_RSTn \
    } /* Reset bits for GPIO peripheral */
#define RTC_RSTS            \
    {                       \
        kRTC_RST_SHIFT_RSTn \
    } /* Reset bits for RTC peripheral */
#define ADC_RSTS            \
    {                       \
        kADC_RST_SHIFT_RSTn \
    } /* Reset bits for ADC peripheral */
#define DAC_RSTS            \
    {                       \
        kDAC_RST_SHIFT_RSTn \
    } /* Reset bits for DAC peripheral */
#define CS_RSTS            \
    {                      \
        kCS_RST_SHIFT_RSTn \
    } /* Reset bits for CS peripheral */
#define FSP_RSTS            \
    {                       \
        kFSP_RST_SHIFT_RSTn \
    } /* Reset bits for FSP peripheral */
#define DMA_RSTS            \
    {                       \
        kDMA_RST_SHIFT_RSTn \
    } /* Reset bits for DMA peripheral */
#define QDEC_RSTS                                    \
    {                                                \
        kQDEC0_RST_SHIFT_RSTn, kQDEC1_RST_SHIFT_RSTn \
    } /* Reset bits for QDEC peripheral */
#define SPIFI_RSTS            \
    {                         \
        kSPIFI_RST_SHIFT_RSTn \
    } /* Reset bits for SPIFI peripheral */
#define CAL_RSTS            \
    {                       \
        kCAL_RST_SHIFT_RSTn \
    } /* Reset bits for Calibration peripheral */
#define CPU_RSTS            \
    {                       \
        kCPU_RST_SHIFT_RSTn \
    } /* Reset bits for CPU peripheral */
#define BLE_RSTS            \
    {                       \
        kBLE_RST_SHIFT_RSTn \
    } /* Reset bits for BLE peripheral */
#define FLASH_RSTS            \
    {                         \
        kFLASH_RST_SHIFT_RSTn \
    } /* Reset bits for FLASH peripheral */
#define DP_RSTS            \
    {                      \
        kDP_RST_SHIFT_RSTn \
    } /* Reset bits for DP peripheral */
#define REG_RSTS            \
    {                       \
        kREG_RST_SHIFT_RSTn \
    } /* Reset bits for REG peripheral */
#define REBOOT_RSTS            \
    {                          \
        kREBOOT_RST_SHIFT_RSTn \
    } /* Reset bits for REBOOT peripheral */

typedef SYSCON_RSTn_t reset_ip_name_t;

/*!
 * @brief Reset source
 */
typedef enum _reset_source
{
    kRESET_SrcPowerOn,     /**< Power on reset */
    kRESET_SrcBrownDown,   /**< Brown down reset */
    kRESET_SrcExternalPin, /**< External pin reset */
    kRESET_SrcWatchDog,    /**< Watch dog reset */
    kRESET_SrcLockUp,      /**< Lock up reset */
    kRESET_SrcReboot,      /**< Reboot reset */
    kRESET_SrcCpuSystem,   /**< CPU system reset */
    kRESET_SrcWakeUp,      /**< Wake up reset */
    kRESET_SrcCpuSoftware, /**< CPU software reset */
} reset_source_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Assert reset to peripheral.
 *
 * Asserts reset signal to specified peripheral module.
 *
 * @param peripheral Assert reset to this peripheral. The enum argument contains encoding of reset register
 *                   and reset bit position in the reset register.
 *
 * @note The peripheral will be in reset state until function RESET_ClearPeripheralReset(...) called.
 */
static inline void RESET_SetPeripheralReset(reset_ip_name_t peripheral)
{
    SYSCON->RST_SW_SET = (1U << peripheral);
}

/*!
 * @brief Clear reset to peripheral.
 *
 * Clears reset signal to specified peripheral module, allows it to operate.
 *
 * @param peripheral Clear reset to this peripheral. The enum argument contains encoding of reset register
 *                   and reset bit position in the reset register.
 */
static inline void RESET_ClearPeripheralReset(reset_ip_name_t peripheral)
{
    SYSCON->RST_SW_CLR = (1U << peripheral);
}

/*!
 * @brief Reset peripheral module.
 *
 * Reset peripheral module.
 *
 * @param peripheral Peripheral to reset. The enum argument contains encoding of reset register
 *                   and reset bit position in the reset register.
 */
static inline void RESET_PeripheralReset(reset_ip_name_t peripheral)
{
    RESET_SetPeripheralReset(peripheral);
    RESET_ClearPeripheralReset(peripheral);
}

/*!
 * @brief Reset DMA, PINT and InputMux module.
 */
static inline void RESET_SetDmaPintImputMuxReset(void)
{
    RESET_PeripheralReset((reset_ip_name_t)SYSCON_RST_SW_SET_SET_DMA_RST_SHIFT);
}

/*!
 * @brief This function is used to get the CPU start up source.
 *
 * @return Reset source @ref reset_source_t
 */
reset_source_t RESET_GetResetSource(void);

/*!
 * @brief Clear the reset source
 */
static inline void RESET_ClearResetSource(void)
{
    SYSCON->RST_CAUSE_SRC = SYSCON_RST_CAUSE_SRC_RST_CAUSE_CLR_MASK;
}

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_RESET_H_ */
