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
#ifndef _FSL_FLASH_DMA_H_
#define _FSL_FLASH_DMA_H_

#include "fsl_flash.h"
#include "fsl_dma.h"

/*!
 * @addtogroup flash_dma_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _flash_dma_handle flash_dma_handle_t;

/*! @brief FLASH DMA callback called at the end of transfer. */
typedef void (*flash_dma_callback_t)(FLASH_Type *base, flash_dma_handle_t *handle, status_t status, void *userData);

/*! @brief FLASH DMA handle, users should not touch the content of the handle.*/
struct _flash_dma_handle
{
    bool writeInProgress;          /*!< write finished */
    bool readInProgress;           /*!< read finished */
    dma_handle_t *writeHandle;     /*!< DMA handler for FLASH write */
    dma_handle_t *readHandle;      /*!< DMA handler for FLASH read */
    flash_dma_callback_t callback; /*!< Callback for FLASH DMA transfer */
    void *userData;                /*!< User Data for FLASH DMA callback */
    uint32_t state;                /*!< Internal state of FLASH DMA transfer */
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name DMA Transactional
 * @{
 */

/*!
 * @brief Initialize the FLASH DMA handle.
 *
 * This function initializes the FLASH DMA handle.
 *
 * @param base FLASH peripheral base address.
 * @param handle FLASH handle pointer.
 * @param callback User callback function called at the end of a transfer.
 * @param userData User data for callback.
 * @param writeHandle DMA handle pointer for FLASH write, the handle shall be static allocated by users.
 * @param readHandle DMA handle pointer for FLASH read, the handle shall be static allocated by users.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 */
status_t FLASH_CreateHandleDMA(FLASH_Type *base,
                               flash_dma_handle_t *handle,
                               flash_dma_callback_t callback,
                               void *userData,
                               dma_handle_t *writeHandle,
                               dma_handle_t *readHandle);

/*!
 * @brief Perform a non-blocking FLASH read using DMA.
 *
 * @note This interface returned immediately after transfer initiates
 *
 * @param base FLASH peripheral base address.
 * @param handle FLASH DMA handle pointer.
 * @param config pointer to configuration structure
 * @param addr  Specifies the start address of the FLASH to be written, the address should be aligned with 4 bytes
 * @param  pBuf  Pointer of the read data buffer
 * @param  lengthInWords  The size of data to be written
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLASH_Busy FLASH is not idle, is running another transfer.
 */
status_t FLASH_StartReadDMA(FLASH_Type *base,
                            flash_dma_handle_t *handle,
                            flash_config_t *config,
                            uint32_t addr,
                            const uint32_t *pBuf,
                            uint32_t lengthInWords);

/*!
 * @brief Perform a non-blocking FLASH write using DMA.
 *
 * @note This interface returned immediately after transfer initiates
 *
 * @param base FLASH peripheral base address.
 * @param handle FLASH DMA handle pointer.
 * @param config pointer to configuration structure
 * @param addr  Specifies the start address of the FLASH to be written, the address should be aligned with 4 bytes
 * @param  pBuf  Pointer of the write data buffer
 * @param  lengthInWords  The size of data to be written
 * @retval kStatus_Success Successfully start a transfer.
 * @retval kStatus_InvalidArgument Input argument is invalid.
 * @retval kStatus_FLASH_Busy FLASH is not idle, is running another transfer.
 */
status_t FLASH_StartWriteDMA(FLASH_Type *base,
                             flash_dma_handle_t *handle,
                             flash_config_t *config,
                             uint32_t addr,
                             const uint32_t *pBuf,
                             uint32_t lengthInWords);

/*!
 * @brief Abort a FLASH transfer using DMA.
 *
 * @param base FLASH peripheral base address.
 * @param handle FLASH DMA handle pointer.
 */
void FLASH_AbortDMA(FLASH_Type *base, flash_dma_handle_t *handle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif
