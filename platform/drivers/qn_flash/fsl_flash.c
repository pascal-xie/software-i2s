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

#include "fsl_flash.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLASH_256K (256 * 1024)
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
* Prototypes
******************************************************************************/
/*!
 * @brief Check the FLASH controller is busy or not.
 *
 * @param base FLASH peripheral base address.
 * @return 0 is not busy.
 */

static inline uint32_t FLASH_BusyStatusCheck(FLASH_Type *base);
/*!
 * @brief Check the FLASH controller in error status or not.
 *
 * @param base FLASH peripheral base address.
 * @return status_t.
 */
static inline status_t FLASH_StatusCheck(FLASH_Type *base);
/*******************************************************************************
 * Code
 ******************************************************************************/
static inline uint32_t FLASH_BusyStatusCheck(FLASH_Type *base)
{
    return (FLASH_GetBusyStatusFlags(base) &
            (kFLASH_EraseBusyL | kFLASH_WriteBusyL | kFLASH_EraseBusyH | kFLASH_WriteBusyH));
}

static inline status_t FLASH_StatusCheck(FLASH_Type *base)
{
    status_t status = FLASH_GetStatusFlags(base);
    if (!(status &
          ~(FLASH_INT_STAT_ERASEL_INT_MASK | FLASH_INT_STAT_WRITEL_INT_MASK | FLASH_INT_STAT_ERASEH_INT_MASK |
            FLASH_INT_STAT_WRITEH_INT_MASK | FLASH_INT_STAT_WRBUFL_INT_MASK | FLASH_INT_STAT_WRBUFH_INT_MASK)))
    {
        return kStatus_FLASH_Success;
    }
    else if (status & (FLASH_INT_STAT_LOCKL_INT_MASK | FLASH_INT_STAT_LOCKH_INT_MASK))
    {
        return kStatus_FLASH_ProtectionViolation;
    }
    else if (status & (FLASH_INT_STAT_AHBL_INT_MASK | FLASH_INT_STAT_AHBH_INT_MASK))
    {
        return kStatus_FLASH_AHBError;
    }
    else if (status & (FLASH_INT_STAT_WRITE_FAIL_L_INT_MASK | FLASH_INT_STAT_WRITE_FAIL_H_INT_MASK))
    {
        return kStatus_FLASH_WriteError;
    }
    else if (status & (FLASH_INT_STAT_ERASE_FAIL_L_INT_MASK | FLASH_INT_STAT_ERASE_FAIL_H_INT_MASK))
    {
        return kStatus_FLASH_EraseError;
    }
    else
        return kStatus_FLASH_Fail;
}

status_t FLASH_Init(FLASH_Type *base, const flash_config_t *config)
{
    if (config == NULL)
    {
        return kStatus_FLASH_InvalidArgument;
    }

    base->SMART_CTRL |=
        (FLASH_SMART_CTRL_SMART_ERASEH_EN_MASK | FLASH_SMART_CTRL_SMART_ERASEL_EN_MASK |
         FLASH_SMART_CTRL_MAX_ERASE(config->smartMaxEraseTime) | FLASH_SMART_CTRL_MAX_WRITE(config->smartMaxWriteTime));

    base->TIME_CTRL = FLASH_TIME_CTRL_TIME_BASE(config->timeBase) | FLASH_TIME_CTRL_PRGM_CYCLE(config->programCycle);

    base->ERASE_TIME = FLASH_ERASE_TIME_ERASE_TIME_BASE(config->eraseTimeBase);

    /*load lock bit and protection bit, load redundancy page info*/
    base->INI_RD_EN = FLASH_INI_RD_EN_INI_RD_EN_MASK;
    while ((FLASH_GetBusyStatusFlags(base) & (FLASH_STATUS1_FSH_STA_MASK | FLASH_STATUS1_INI_RD_DONE_MASK)) !=
           FLASH_STATUS1_INI_RD_DONE_MASK)
        ;
    return kStatus_FLASH_Success;
}

void FLASH_GetDefaultConfig(flash_config_t *config)
{
    uint32_t ahb_freq = CLOCK_GetFreq(kCLOCK_BusClk);

    config->blockBase = FSL_FEATURE_FLASH_BASE_ADDR;
    config->totalSize = FSL_FEATURE_FLASH_SIZE_BYTES;
    config->pageSize = FSL_FEATURE_FLASH_PAGE_SIZE_BYTES;
    config->eraseTimeBase = FLASH_ERASE_TIME_BASE;
    config->timeBase = FLASH_TIME_BASE(ahb_freq);
    config->programCycle = FLASH_PROG_CYCLE;
    config->smartMaxEraseTime = FLASH_SMART_MAX_ERASE_TIME;
    config->smartMaxWriteTime = FLASH_SMART_MAX_WRITE_TIME;
    config->smartWriteEnable = false;
}

status_t FLASH_PageErase(FLASH_Type *base, flash_config_t *config, uint8_t pageIdx)
{
    if ((config == NULL) || ((pageIdx > 127U) && (config->totalSize == FLASH_256K)))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    if ((pageIdx < 128U) && (config->totalSize != FLASH_256K))
    {
        base->ERASE_CTRL = (FLASH_ERASE_CTRL_PAGE_ERASEL_EN_MASK | FLASH_ERASE_CTRL_PAGE_IDXL(pageIdx));
    }
    else
    {
        base->ERASE_CTRL = (FLASH_ERASE_CTRL_PAGE_ERASEH_EN_MASK | FLASH_ERASE_CTRL_PAGE_IDXH(pageIdx));
    }

    /*Wait the last operation is over.*/
    while (FLASH_BusyStatusCheck(base))
        ;

    return FLASH_StatusCheck(base);
}

status_t FLASH_BlockErase(FLASH_Type *base, flash_config_t *config, uint32_t block)
{
    if ((config == NULL) || ((block & kFLASH_Block1) && (config->totalSize == FLASH_256K)))
    {
        return kStatus_FLASH_InvalidArgument;
    }
    if (config->totalSize != FLASH_256K)
    {
        base->ERASE_CTRL = ((block << FLASH_ERASE_CTRL_HALF_ERASEL_EN_SHIFT) &
                            (FLASH_ERASE_CTRL_HALF_ERASEL_EN_MASK | FLASH_ERASE_CTRL_HALF_ERASEH_EN_MASK));
    }
    else
    {
        base->ERASE_CTRL = FLASH_ERASE_CTRL_HALF_ERASEH_EN_MASK;
    }

    /*Wait the last operation is over.*/
    while (FLASH_BusyStatusCheck(base))
        ;
    return FLASH_StatusCheck(base);
}

status_t FLASH_Program(
    FLASH_Type *base, flash_config_t *config, uint32_t addr, const uint8_t *pBuf, uint32_t lengthInBytes)
{
    uint32_t data = 0U;
    uint32_t offset = 0U;
    uint32_t len = 0U;
    uint32_t i = 0U;
    uint32_t j = config->smartMaxWriteTime;
    uint32_t m = 0U;
    uint32_t n = 0U;

    if ((config == NULL) || (pBuf == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }
    if ((addr < config->blockBase) || ((addr + lengthInBytes) > (config->blockBase + config->totalSize)))
    {
        return kStatus_FLASH_AddressError;
    }

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

    while (j--) /*Retry config->smartMaxWriteTime times*/
    {
        const uint8_t *tmp_buf = pBuf;
        uint32_t tmp_addr = addr;
        /*1- address unaligned with 4 bytes */
        offset = tmp_addr & 0x03U;
        if (offset)
        {
            len = 4U - offset;
            data = *((volatile uint32_t *)(tmp_addr & (~0x03U)));
            len = (len < lengthInBytes) ? len : lengthInBytes;
            for (i = 0U; i < len; i++)
            {
                *((uint8_t *)(&data) + offset + i) = *tmp_buf++;
            }
            *((volatile uint32_t *)(tmp_addr & (~0x03U))) = data;
            tmp_addr += len;
        }

        /*2- word copy*/
        m = ((lengthInBytes - len) & 0x7FFFFU) >> 2U;
        n = (lengthInBytes - len) & 0x03U;
        if ((uint32_t)tmp_buf & 0x03U)
        {
            while (m--)
            {
                /*Using byte copy insure the wirte is ok, while the p_buf is not 4 byte ailgn*/

                union
                {
                    uint32_t word;
                    uint8_t byte[4];
                } tmp;
                tmp.byte[0] = *tmp_buf++;
                tmp.byte[1] = *tmp_buf++;
                tmp.byte[2] = *tmp_buf++;
                tmp.byte[3] = *tmp_buf++;
                *((volatile uint32_t *)(tmp_addr)) = tmp.word;
                tmp_addr += 4U;
            }
        }
        else
        {
            while (m--)
            {
                *((volatile uint32_t *)(tmp_addr)) = *((volatile uint32_t *)tmp_buf);
                tmp_addr += 4U;
                tmp_buf += 4U;
            }
        }

        /*3 rest of data*/
        if (n)
        {
            uint32_t last_word = *((volatile uint32_t *)(tmp_addr));
            for (i = 0U; i < n; i++)
            {
                ((uint8_t *)&last_word)[i] = *tmp_buf++;
            }
            *((volatile uint32_t *)(tmp_addr)) = last_word;
        }
        /*wait write over*/
        while (FLASH_BusyStatusCheck(base))
            ;
        if (config->smartWriteEnable)
        {
            return FLASH_StatusCheck(base);
        }
        else
        {
            uint32_t raddr = FSL_FEATURE_FLASH_READ_BASE_ADDR | (addr & 0x7FFFFU);
            uint8_t *p_buf = (uint8_t *)pBuf;
            uint32_t n_byte = lengthInBytes;
            while (n_byte && (*p_buf++ == *((volatile uint8_t *)(raddr++))))
            {
                if (n_byte)
                    n_byte--;
            }
            if (!n_byte)
            {
                return kStatus_FLASH_Success;
            }
        }
    }
    return kStatus_FLASH_Fail;
}

status_t FLASH_GetLockBit(FLASH_Type *base, flash_config_t *config, flash_lock_bit_t *lockBit)
{
    uint32_t *pLockBit = (uint32_t *)lockBit;

    if ((config == NULL) || (lockBit == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    *pLockBit++ = base->LOCK_STAT0;
    *pLockBit++ = base->LOCK_STAT1;
    *pLockBit++ = base->LOCK_STAT2;
    *pLockBit++ = base->LOCK_STAT3;
    *pLockBit++ = base->LOCK_STAT4;
    *pLockBit++ = base->LOCK_STAT5;
    *pLockBit++ = base->LOCK_STAT6;
    *pLockBit++ = base->LOCK_STAT7;
    *pLockBit = base->LOCK_STAT8;

    return kStatus_FLASH_Success;
}

status_t FLASH_SetLockBit(FLASH_Type *base, flash_config_t *config, flash_lock_bit_t *lockBit)
{
    uint32_t result = kStatus_Success;

    if ((config == NULL) || (lockBit == NULL))
    {
        return kStatus_FLASH_InvalidArgument;
    }

    /*If last page is lock,unlock it*/
    if (!(base->LOCK_STAT7 & 0x80000000U))
    {
        base->ERASE_PASSWORD = 0x5B6C013AU;

        FLASH_PageErase(base, config, (uint8_t)FLASH_ADDR_TO_PAGE(FSL_FEATURE_FLASH_LOCK_BIT_STORE_ADDR));
        while (FLASH_BusyStatusCheck(base))
            ;
        /*Enable controller read lock information*/
        base->INI_RD_EN = FLASH_INI_RD_EN_INI_RD_EN_MASK;
        while ((FLASH_GetBusyStatusFlags(base) & (FLASH_STATUS1_FSH_STA_MASK | FLASH_STATUS1_INI_RD_DONE_MASK)) !=
               FLASH_STATUS1_INI_RD_DONE_MASK)
            ;
    }
    result = FLASH_Program(base, config, FSL_FEATURE_FLASH_LOCK_BIT_STORE_ADDR, (uint8_t *)lockBit, 9 * 4);

    /*Enable lock bit setting*/
    base->INI_RD_EN = FLASH_INI_RD_EN_INI_RD_EN_MASK;
    while ((FLASH_GetBusyStatusFlags(base) & (FLASH_STATUS1_FSH_STA_MASK | FLASH_STATUS1_INI_RD_DONE_MASK)) !=
           FLASH_STATUS1_INI_RD_DONE_MASK)
        ;
    return result;
}
