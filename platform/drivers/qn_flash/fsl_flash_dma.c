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

#include "fsl_flash_dma.h"

/*******************************************************************************
 * Definitons
 ******************************************************************************/
/*<! Structure definition for flash_dma_private_handle_t. The structure is private. */
typedef struct _flash_dma_private_handle
{
    FLASH_Type *base;
    flash_dma_handle_t *handle;
} flash_dma_private_handle_t;

/*! @brief FLASH transfer state, which is used for FLASH transactiaonl APIs' internal state. */
enum _flash_dma_states_t
{
    kFLASH_Idle = 0x0, /*!< FLASH is idle state */
    kFLASH_Busy        /*!< FLASH is busy tranferring data. */
};

/*<! Private handle only used for internally. */
static flash_dma_private_handle_t s_dmaPrivateHandle;
#define FLASH_256K (256 * 1024)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief DMA callback function for FLASH write.
 *
 * @param handle DMA handle pointer.
 * @param userData User data for DMA callback function.
 */
static void FLASH_WriteDMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode);

/*!
 * @brief DMA callback function for FLASH read.
 *
 * @param handle DMA handle pointer.
 * @param userData User data for DMA callback function.
 */
static void FLASH_ReadDMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
* Code
******************************************************************************/
static void FLASH_WriteDMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode)
{
    flash_dma_private_handle_t *privHandle = (flash_dma_private_handle_t *)userData;
    flash_dma_handle_t *flashHandle = privHandle->handle;
    FLASH_Type *base = privHandle->base;

    /* change the state */
    flashHandle->writeInProgress = false;

    /* All finished, call the callback */
    if ((flashHandle->writeInProgress == false) && (flashHandle->readInProgress == false))
    {
        flashHandle->state = kFLASH_Idle;
        if (flashHandle->callback)
        {
            (flashHandle->callback)(base, flashHandle, kStatus_FLASH_WriteDmaIdle, flashHandle->userData);
        }
    }
}

static void FLASH_ReadDMACallback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode)
{
    flash_dma_private_handle_t *privHandle = (flash_dma_private_handle_t *)userData;
    flash_dma_handle_t *flashHandle = privHandle->handle;
    FLASH_Type *base = privHandle->base;

    /* change the state */
    flashHandle->readInProgress = false;

    /* All finished, call the callback */
    if ((flashHandle->writeInProgress == false) && (flashHandle->readInProgress == false))
    {
        flashHandle->state = kFLASH_Idle;
        if (flashHandle->callback)
        {
            (flashHandle->callback)(base, flashHandle, kStatus_FLASH_ReadDmaIdle, flashHandle->userData);
        }
    }
}

status_t FLASH_CreateHandleDMA(FLASH_Type *base,
                               flash_dma_handle_t *handle,
                               flash_dma_callback_t callback,
                               void *userData,
                               dma_handle_t *writeHandle,
                               dma_handle_t *readHandle)
{
    if ((NULL == base) || (NULL == handle))
    {
        return kStatus_InvalidArgument;
    }

    /* Set flash base to handle */
    handle->writeHandle = writeHandle;
    handle->readHandle = readHandle;
    handle->callback = callback;
    handle->userData = userData;

    /* Set FLASH state to idle */
    handle->state = kFLASH_Idle;

    /* Set handle to global state */
    s_dmaPrivateHandle.base = base;
    s_dmaPrivateHandle.handle = handle;

    /* Install callback for Tx dma channel */
    DMA_SetCallback(handle->writeHandle, FLASH_WriteDMACallback, &s_dmaPrivateHandle);
    DMA_SetCallback(handle->readHandle, FLASH_ReadDMACallback, &s_dmaPrivateHandle);

    return kStatus_Success;
}

status_t FLASH_StartWriteDMA(FLASH_Type *base,
                             flash_dma_handle_t *handle,
                             flash_config_t *config,
                             uint32_t addr,
                             const uint32_t *pBuf,
                             uint32_t lengthInWords)
{
    dma_transfer_config_t dma_config = {0};

    if ((NULL == base) || (NULL == handle) || (NULL == config) || (NULL == pBuf) || (0 == lengthInWords))
    {
        return kStatus_InvalidArgument;
    }

    if ((addr < config->blockBase) ||
        ((addr + lengthInWords * sizeof(uint32_t)) > (config->blockBase + config->totalSize)))
    {
        return kStatus_FLASH_AddressError;
    }

    /* Check if the device is busy */
    if (handle->state == kFLASH_Busy)
    {
        return kStatus_FLASH_Busy;
    }
    else
    {
        handle->state = kFLASH_Busy;
        if (config->totalSize != FLASH_256K)
        {
            base->SMART_CTRL = ((config->smartWriteEnable ?
                                     (FLASH_SMART_CTRL_SMART_WRITEH_EN_MASK | FLASH_SMART_CTRL_SMART_WRITEL_EN_MASK) :
                                     0U) |
                                FLASH_SMART_CTRL_PRGMH_EN_MASK | FLASH_SMART_CTRL_PRGML_EN_MASK);
        }
        else
        {
            base->SMART_CTRL = ((config->smartWriteEnable ? (FLASH_SMART_CTRL_SMART_WRITEH_EN_MASK) : 0U) |
                                FLASH_SMART_CTRL_PRGMH_EN_MASK);
        }
        DMA_PrepareTransfer(&dma_config, (void *)pBuf, (void *)addr, sizeof(uint32_t), lengthInWords * sizeof(uint32_t),
                            kDMA_MemoryToMemory, NULL);
        /* Submit transfer. */
        DMA_SubmitTransfer(handle->writeHandle, &dma_config);

        handle->writeInProgress = true;
        DMA_StartTransfer(handle->writeHandle);
    }
    return kStatus_Success;
}

status_t FLASH_StartReadDMA(FLASH_Type *base,
                            flash_dma_handle_t *handle,
                            flash_config_t *config,
                            uint32_t addr,
                            const uint32_t *pBuf,
                            uint32_t lengthInWords)
{
    dma_transfer_config_t dma_config = {0};

    if ((NULL == base) || (NULL == handle) || (NULL == config) || (NULL == pBuf) || (0 == lengthInWords))
    {
        return kStatus_InvalidArgument;
    }

    if ((addr < config->blockBase) ||
        ((addr + lengthInWords * sizeof(uint32_t)) > (config->blockBase + config->totalSize)))
    {
        return kStatus_FLASH_AddressError;
    }

    /* Check if the device is busy */
    if (handle->state == kFLASH_Busy)
    {
        return kStatus_FLASH_Busy;
    }
    else
    {
        handle->state = kFLASH_Busy;

        DMA_PrepareTransfer(&dma_config, (void *)addr, (void *)pBuf, sizeof(uint32_t), lengthInWords * sizeof(uint32_t),
                            kDMA_MemoryToMemory, NULL);
        /* Submit transfer. */
        DMA_SubmitTransfer(handle->readHandle, &dma_config);
        handle->readInProgress = true;
        DMA_StartTransfer(handle->readHandle);
    }
    return kStatus_Success;
}
void FLASH_AbortDMA(FLASH_Type *base, flash_dma_handle_t *handle)
{
    assert(NULL != handle);

    /* Stop tx transfer first */
    DMA_AbortTransfer(handle->writeHandle);
    /* Then rx transfer */
    DMA_AbortTransfer(handle->readHandle);

    /* Set the handle state */
    handle->writeInProgress = false;
    handle->readInProgress = false;
    handle->state = kFLASH_Idle;
}
