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
 * Author: Jianhua Xie <Jianhua.xie@nxp.com>, <pascal.xie@qq.com>
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
#ifndef _FSL_SOFTWARE_I2S_H_
#define _FSL_SOFTWARE_I2S_H_

#include <stdint.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "fsl_spi.h"
#include "fsl_sctimer.h"
#include "fsl_spi_dma.h"
#include "fsl_dma.h"

/*!
 * @addtogroup software_i2s
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @name Driver version */
/*@{*/
/*! @brief Software I2S Master driver version 2.0.0. */
#define FSL_SOFTWARE_I2S_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * Select polarities of the used and unused chip selects.
 * Used Chip Select polarity:
 * CS0 - Active LOW
 * Unused Chip Select polarities :
 * CS1 - Active LOW
 * CS2 - Active HIGH
 * CS3 - Active HIGH
 */
#define SOFTWARE_I2S_SPI_SPOL (kSPI_SpolActiveAllLow | kSPI_Spol2ActiveHigh | kSPI_Spol3ActiveHigh)
#define SOFTWARE_I2S_SPI_SLAVE SPI0
#define SOFTWARE_I2S_SPI_SLAVE_IRQ FLEXCOMM2_IRQn
#define SOFTWARE_I2S_SPI_SSEL 0 /* SSEL PinMux == 0XC00 */
#define SOFTWARE_I2S_IRQHandler FLEXCOMM2_IRQHandler

#define MEMS_MIC_IS_16BIT TRUE
#define SOFTWARE_I2S_WS_PIN_OUT kSCTIMER_Out_3
#define SOFTWARE_I2S_BCK_PIN_OUT kSCTIMER_Out_2
/* Each FIFO entry is 16bit or 32bit, FIFO SIZE should be multiple of 4, and smaller than RAM Size */
#define SOFTWARE_I2S_FIFO_SIZE (1024)
#define SOFTWARE_I2S_DELAY_FOR_MIC_WAKEUP 1600000

#define SOFTWARE_I2S_DMA DMA0
#define SOFTWARE_I2S_DMA_RX_CHANNEL 4
/* below size should be little than DMA_MAX_TRANSFER_COUNT, and better bigger than 64 */
#define SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE (64)

/* Please don't change below lines */
#define SOFTWARE_I2S_WS_COUNTER kSCTIMER_Counter_L
#define SOFTWARE_I2S_BCK_COUNTER kSCTIMER_Counter_H

#define SOFTWARE_I2S_Type SCT_Type
#define SOFTWARE_I2S_BASE_PTRS SCT_BASE_PTRS
#define SOFTWARE_I2S_CYCLE_MAGIC 8U

/* The count of SOFTWARE_I2S is equal to the count of SCT */
#define SOFTWARE_I2S0 SCT0
#define FSL_FEATURE_SOC_SOFTWARE_I2S_COUNT FSL_FEATURE_SOC_SCT_COUNT

/*!
 * @brief Clock source.
 */
typedef enum _software_i2s_bclk
{
    kI2S_BCLK_500KHz = 500000U, /*!< I2S bit clock 500 kHz */
    kI2S_BCLK_1MHz = 1000000U, /*!< I2S bit clock 1 MHz */
    kI2S_BCLK_2MHz = 2000000U, /*!< I2S bit clock 2 MHz */
    kI2S_BCLK_4MHz = 4000000U, /*!< I2S bit clock 4 MHz */
} software_i2s_bclk_t;

/*!
 * @brief ws - means word select clock or channel select clock, ws = bclk/64.
 */
typedef enum _software_i2s_ws
{
    kI2S_WS_CLK_08KHZ = 0U,  /*!< I2S WS Clock 08 kHz */
    kI2S_WS_CLK_16KHZ,      /*!< I2S WS Clock 16 kHz */
    kI2S_WS_CLK_32KHZ,      /*!< I2S WS Clock 32 kHz */
    kI2S_WS_CLK_64KHZ,      /*!< I2S WS Clock 64 kHz */
} software_i2s_ws_t;

/*!
 * @brief sampling_mode - I2S sampling mode rising or falling.
 */
typedef enum _software_i2s_sampling_mode
{
    kI2S_SAMPLING_MODE_RISING = 0U,  /*!< I2S SAMPLING at RISING edge*/
    kI2S_SAMPLING_MODE_FALLING,      /*!< I2S SAMPLING at FALLING edge */
} software_i2s_sampling_mode_t;

/*!
 * @brief io mode - I2S io interface access mode.
 */
typedef enum _software_i2s_mode
{
    kI2S_IO_IRQ = 0U, /*!< I2S interface interrupt mode */
    kI2S_IO_DMA,      /*!< I2S interface dma mode */
} software_i2s_mode_t;

/*!
 * @brief Define structure for configuring the block
 */
typedef struct _software_i2s_config_t
{
    sctimer_config_t sct_config;   /*!< SCT config on which software i2s depends */
    software_i2s_bclk_t bck_speed; /*!< I2S bit clock speed */
    software_i2s_ws_t ws_speed;    /*!< I2S ws clock speed */
    sctimer_out_t bck_pin;         /*!< I2S bit clock pin */
    sctimer_out_t ws_pin;          /*!< I2S ws pin */
    software_i2s_mode_t io_mode;   /*!< I2S io interface access mode */
    spi_slave_config_t spi_config; /*!< I2S internal SPI config */
    uint8_t left_bias_bits;        /*!< I2S left shift bias bits */
    software_i2s_sampling_mode_t sampling_mode; /*!< I2S sampling_mode rising or falling */
} software_i2s_config_t;

/*!
 * @brief Define software i2s fifo structure
 */
typedef struct _i2s_fifo
{
/*
 * I2S FIFO size and width.
 * use micro definition rather than an union to reduce memory.
 */
#if defined(MEMS_MIC_IS_32BIT) && (MEMS_MIC_IS_32BIT == TRUE)
    uint32_t fifo[SOFTWARE_I2S_FIFO_SIZE]; /*!< 32bit FIFO */
#elif defined(MEMS_MIC_IS_16BIT) && (MEMS_MIC_IS_16BIT == TRUE)
    uint16_t fifo[SOFTWARE_I2S_FIFO_SIZE]; /*!< 16bit FIFO */
#else
#error "This CPU doesn't support this MEMS MIC format"
#endif
    uint32_t rp;                          /*!< FIFO read pointer */
    uint32_t wp;                          /*!< FIFO write pointer */
    bool i2s_stash_finish_cmd_and_status; /*!< Software i2s stash finish command and finish status */
    bool LowHalfWord; /*!< Software I2S data format supports upto 32 bits, this field marks low/high half word */
    bool is_newline;  /*!< word wrap flag */
} software_i2s_fifo_t;

/* Globle variables list */
extern software_i2s_fifo_t i2s_fifo_rd;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Gets an available pre-defined settings for initial configuration.
 *
 * Initializes the default configuration structure with a minimal settings.
 * The default values are:
 * @code
 *   config->bck_speed = kI2S_BCLK_1MHz;
 *   config->ws_speed = kI2S_WS_CLK_16KHZ;
 *   config->bck_pin = SOFTWARE_I2S_BCK_PIN_OUT;
 *   config->ws_pin = SOFTWARE_I2S_WS_PIN_OUT;
 * @endcode
 * @param config Pointer to configuration structure.
 */
void SOFTWARE_I2S_GetDefaultConfig(software_i2s_config_t *config);

/*!
 * @brief Initialize the Software I2S Master module.
 *
 * This function initializes the Software I2S module, including:
 *  - Enable Software I2S module clock.
 *  - Reset Software I2S module.
 *  - Configure the Software I2S with user configuration.
 *
 * @param base Software I2S peripheral base address.
 * @param config Pointer to configuration structure, see to #software_i2s_config_t.
 *
 * @return kStatus_Success indicates success; Else indicates failure.
 */
status_t SOFTWARE_I2S_Init(SOFTWARE_I2S_Type *base, software_i2s_config_t *config);

/*!
 * @brief Deinitialize the Software I2S module.
 *
 * This function de-nitializes the Software I2S module, including:
 *  - Disable the Software I2S module clock.
 *
 * @param base Software I2S peripheral base address.
 */
void SOFTWARE_I2S_Deinit(SOFTWARE_I2S_Type *base);

/*!
 * @brief Enable/Disable the Software I2S Master.
 *
 * @param base     Software I2S peripheral base address.
 * @param enable   true to enable the Software I2S, false to disable.
 */
void SOFTWARE_I2S_Enable(SOFTWARE_I2S_Type *base, bool enable);

/*!
 * @brief Prepare a non-blocking SPI transfer using DMA with double buffer.
 *
 * @note This interface returned immediately after transfer initiates, users should call
 * SPI_GetTransferStatus to poll the transfer status to check whether SPI transfer finished.
 *
 * @param spibase SPI peripheral base address.
 * @param dmabase DMA peripheral base address.
 * @param rx_ch DMA rx channel number.
 * @param h_i2s_dma_rx DMA rx handler.
 * @param h_i2s_dma_tx DMA tx handler.
 * @param h_slave SPI peripheral DMA handler.
 * @param cb DMA double buffer call back.
 * @param userdata User Data for DMA double buffer call back.
 * @param i2sXfer spi transfer structure for DDB.
 * @param Desc array Current DMA description and Next DMA description.
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_SPI_Busy SPI is not idle, is running another transfer.
 */
status_t SOFTWARE_I2S_PrepareDDB(SPI_Type *spibase,
                                 DMA_Type *dmabase,
                                 int rx_ch,
                                 dma_handle_t *h_i2s_dma_rx,
                                 dma_handle_t *h_i2s_dma_tx,
                                 spi_dma_handle_t *h_slave,
                                 dma_callback cb,
                                 void *userdata,
                                 spi_transfer_t *i2sXfer,
                                 dma_descriptor_t Desc[]);

/*! @}*/

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* _FSL_SOFTWARE_I2S_H_ */
