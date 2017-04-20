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
#include "fsl_software_i2s.h"

/*******************************************************************************
 * Definitions
 *****************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base SOFTWARE_I2S (SCTimer) peripheral base address
 *
 * @return The SOFTWARE_I2S (SCTimer) instance
 */
static uint32_t SOFTWARE_I2S_GetInstance(SCT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to SOFTWARE_I2S (SCT) bases for each instance. */
static SOFTWARE_I2S_Type *const s_software_i2sBases[] = SOFTWARE_I2S_BASE_PTRS;
/*! @brief internal SOFTWARE_I2S config array */
static software_i2s_config_t g_configs[FSL_FEATURE_SOC_SOFTWARE_I2S_COUNT] = {0};
/*! @brief internal SOFTWARE_I2S register level fifo */
static uint32_t software_i2s_reg_rx_fifo[SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE] = {0};
/* SPI DMA double buffer transfer helper */
static dma_handle_t h_i2s_dma_rx, h_i2s_dma_tx;
static spi_dma_handle_t h_slave;
static spi_transfer_t i2sXfer;
/* I2S focus on the 2nd pulse, but SPI focus on the 1st pulse,
 * in order to get rid of the difference, left shift bias bits
 * is provided.
 */
static uint8_t leftshift_bias_bits = 0;
/*! @brief software i2s fifo for each instance. */
software_i2s_fifo_t i2s_fifo_rd = {0};

/*! @brief Static table of descriptors */
#if defined(__ICCARM__)
#pragma data_alignment = 16
dma_descriptor_t g_pingpong_desc[] = {{0}, {0}};
#elif defined(__CC_ARM)
__attribute__((aligned(16))) dma_descriptor_t g_pingpong_desc[] = {{0}, {0}};
#elif defined(__GNUC__)
__attribute__((aligned(16))) dma_descriptor_t g_pingpong_desc[] = {{0}, {0}};
#endif

#if defined(__ICCARM__)
#pragma data_alignment = 4
static spi_dma_txdummy_t s_txDummy[FSL_FEATURE_SOC_SPI_COUNT] = {0};
#elif defined(__CC_ARM)
__attribute__((aligned(4))) static spi_dma_txdummy_t s_txDummy[FSL_FEATURE_SOC_SPI_COUNT] = {0};
#elif defined(__GNUC__)
__attribute__((aligned(4))) static spi_dma_txdummy_t s_txDummy[FSL_FEATURE_SOC_SPI_COUNT] = {0};
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void SOFTWARE_I2S_DDB_Cb_Rx(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

static uint32_t SOFTWARE_I2S_GetInstance(SOFTWARE_I2S_Type *base)
{
    uint32_t instance;
    uint32_t software_i2sArrayCount = (sizeof(s_software_i2sBases) / sizeof(s_software_i2sBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < software_i2sArrayCount; instance++)
    {
        if (s_software_i2sBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < software_i2sArrayCount);

    return instance;
}

void *SOFTWARE_I2S_GetConfig(SOFTWARE_I2S_Type *base)
{
    int32_t instance;
    instance = SOFTWARE_I2S_GetInstance(base);
    if (instance < 0)
    {
        return NULL;
    }
    return &g_configs[instance];
}

static void SOFTWARE_I2S_SPI_Slave_Start_ISR(SOFTWARE_I2S_Type *base, bool enable)
{
    spi_slave_config_t *slave_config = NULL;
    software_i2s_config_t *software_i2s_config;

    software_i2s_config = SOFTWARE_I2S_GetConfig(base);
    slave_config = &(software_i2s_config->spi_config);

    if (enable)
    {
        SPI_SlaveInit(SOFTWARE_I2S_SPI_SLAVE, slave_config);
        EnableIRQ(SOFTWARE_I2S_SPI_SLAVE_IRQ);
        SPI_EnableInterrupts(SOFTWARE_I2S_SPI_SLAVE, kSPI_RxLvlIrq | kSPI_TxLvlIrq);
    }
    else
    {
        SPI_DisableInterrupts(SOFTWARE_I2S_SPI_SLAVE, kSPI_RxLvlIrq | kSPI_TxLvlIrq);
        DisableIRQ(SOFTWARE_I2S_SPI_SLAVE_IRQ);
        SPI_Deinit(SOFTWARE_I2S_SPI_SLAVE);
        i2s_fifo_rd.LowHalfWord = false;
    }
}
static void SOFTWARE_I2S_SPI_Slave_Start_DMA(SOFTWARE_I2S_Type *base, bool enable)
{
    SPI_Enable(SOFTWARE_I2S_SPI_SLAVE, enable);
}

static void SOFTWARE_I2S_XferToFifoWR(spi_transfer_t *xfer, uint32_t *fifowr)
{
    *fifowr |= xfer->configFlags & (uint32_t)kSPI_FrameDelay ? (uint32_t)kSPI_FrameDelay : 0;
    *fifowr |= xfer->configFlags & (uint32_t)kSPI_FrameAssert ? (uint32_t)kSPI_FrameAssert : 0;
}

static void SOFTWARE_I2S_SpiConfigToFifoWR(spi_config_t *config, uint32_t *fifowr)
{
    *fifowr |= (SPI_DEASSERT_ALL & (~SPI_DEASSERTNUM_SSEL(config->sselNum)));
    /* set width of data - range asserted at entry */
    *fifowr |= SPI_FIFOWR_LEN(config->dataWidth);
}

static void SOFTWARE_I2S_SPI_SetupDummy(uint32_t *dummy, spi_transfer_t *xfer, spi_config_t *spi_config_p)
{
    *dummy = SPI_DUMMYDATA;
    SOFTWARE_I2S_XferToFifoWR(xfer, dummy);
    SOFTWARE_I2S_SpiConfigToFifoWR(spi_config_p, dummy);
}

static status_t SOFTWARE_I2S_SPI_MasterTransferDMA_DoubleBuff(SPI_Type *base,
                                                              spi_dma_handle_t *handle,
                                                              spi_transfer_t *xfer,
                                                              dma_descriptor_t Desc[])
{
    int32_t instance;
    status_t result = kStatus_Success;
    spi_config_t *spi_config_p;

    assert(!((NULL == handle) || (NULL == xfer)));
    if ((NULL == handle) || (NULL == xfer))
    {
        return kStatus_InvalidArgument;
    }
    /* txData set and not aligned to sizeof(uint32_t) */
    assert(!((NULL != xfer->txData) && ((uint32_t)xfer->txData % sizeof(uint32_t))));
    if ((NULL != xfer->txData) && ((uint32_t)xfer->txData % sizeof(uint32_t)))
    {
        return kStatus_InvalidArgument;
    }
    /* rxData set and not aligned to sizeof(uint32_t) */
    assert(!((NULL != xfer->rxData) && ((uint32_t)xfer->rxData % sizeof(uint32_t))));
    if ((NULL != xfer->rxData) && ((uint32_t)xfer->rxData % sizeof(uint32_t)))
    {
        return kStatus_InvalidArgument;
    }
    /* byte size is zero or not aligned to sizeof(uint32_t) */
    assert(!((xfer->dataSize == 0) || (xfer->dataSize % sizeof(uint32_t))));
    if ((xfer->dataSize == 0) || (xfer->dataSize % sizeof(uint32_t)))
    {
        return kStatus_InvalidArgument;
    }
    /* cannot get instance from base address */
    instance = SPI_GetInstance(base);
    assert(!(instance < 0));
    if (instance < 0)
    {
        return kStatus_InvalidArgument;
    }

    /* Check if the device is busy */
    if (handle->state == kSPI_Busy)
    {
        return kStatus_SPI_Busy;
    }
    else
    {
        dma_transfer_config_t xferConfig = {0};
        spi_config_p = (spi_config_t *)SPI_GetConfig(base);

        handle->state = kStatus_SPI_Busy;
        handle->transferSize = xfer->dataSize;

        /* receive */
        SPI_EnableRxDMA(base, true);
        if (xfer->rxData)
        {
            DMA_PrepareTransfer(&xferConfig, (void *)&base->FIFORD, xfer->rxData, sizeof(uint32_t), (xfer->dataSize) / 2,
                                kDMA_PeripheralToMemory, &Desc[1]);
            DMA_SubmitTransfer(handle->rxHandle, &xferConfig);
						xferConfig.xfercfg.intA = false;
            xferConfig.xfercfg.intB = true;
            DMA_CreateDescriptor(&Desc[1], &xferConfig.xfercfg, (void *)&base->FIFORD, &xfer->rxData[(xfer->dataSize) / 2],
                                 &Desc[0]);
            xferConfig.xfercfg.intA = true;
            xferConfig.xfercfg.intB = false;
            DMA_CreateDescriptor(&Desc[0], &xferConfig.xfercfg, (void *)&base->FIFORD, xfer->rxData,
                                 &Desc[1]);
            handle->rxInProgress = true;
            DMA_StartTransfer(handle->rxHandle);
        }

        /* transmit */
        SPI_EnableTxDMA(base, true);
        SOFTWARE_I2S_SPI_SetupDummy(&s_txDummy[instance].word, xfer, spi_config_p);
        DMA_PrepareTransfer(&xferConfig, &s_txDummy[instance].word, (void *)&base->FIFOWR, sizeof(uint32_t),
                            xfer->dataSize, kDMA_StaticToStatic, NULL);
        result = DMA_SubmitTransfer(handle->txHandle, &xferConfig);
        if (result != kStatus_Success)
        {
            return result;
        }
        handle->txInProgress = true;
        DMA_StartTransfer(handle->txHandle);
    }

    return result;
}

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
void SOFTWARE_I2S_GetDefaultConfig(software_i2s_config_t *config)
{
    assert(config);

    sctimer_config_t *sct_config = &(config->sct_config);
    spi_slave_config_t *spiconfig = &(config->spi_config);
    SCTIMER_GetDefaultConfig(sct_config);

    /* Switch to two 16-bit counter mode */
    sct_config->enableCounterUnify = false;
    sct_config->prescale_h = 0;
    sct_config->prescale_l = 0;
    config->bck_speed = kI2S_BCLK_1MHz;
    config->ws_speed = kI2S_WS_CLK_16KHZ;
    config->sampling_mode = kI2S_SAMPLING_MODE_RISING;
    /*
     * As default, Pin07 (kSCTIMER_Out_2/SOFTWARE_I2S_BCK_PIN_OUT) is for i2s bit clock pin,
     * Pin06 (kSCTIMER_Out_3/SOFTWARE_I2S_WS_PIN_OUT) is for i2s ws output pin.
     */
    config->bck_pin = SOFTWARE_I2S_BCK_PIN_OUT;
    config->ws_pin = SOFTWARE_I2S_WS_PIN_OUT;
    config->io_mode = kI2S_IO_IRQ;

    SPI_SlaveGetDefaultConfig(spiconfig);
    spiconfig->sselPol = (spi_spol_t)SOFTWARE_I2S_SPI_SPOL;
    spiconfig->dataWidth = kSPI_Data16Bits;
    if (kI2S_SAMPLING_MODE_RISING == config->sampling_mode)
    {
        spiconfig->polarity = kSPI_ClockPolarityActiveHigh;
    }
    else
    {
        spiconfig->polarity = kSPI_ClockPolarityActiveLow;
    }
    spiconfig->rxWatermark = kSPI_RxFifo7;
}

static void SOFTWARE_I2S_Get_SCT_FrequencyDiv(uint32_t AHB_Freqency,
                                              software_i2s_config_t *config,
                                              uint32_t *matchValueH_bitClk,
                                              uint32_t *matchValueL_WsClk)
{
    uint32_t ws_speed;

    ws_speed = config->ws_speed;
    switch (ws_speed)
    {
        case kI2S_WS_CLK_08KHZ:
            config->bck_speed = kI2S_BCLK_500KHz;
            break;
        case kI2S_WS_CLK_16KHZ:
            config->bck_speed = kI2S_BCLK_1MHz;
            break;
        case kI2S_WS_CLK_32KHZ:
            config->bck_speed = kI2S_BCLK_2MHz;
            break;
        case kI2S_WS_CLK_64KHZ:
            config->bck_speed = kI2S_BCLK_4MHz;
            break;
        default:
            config->bck_speed = kI2S_BCLK_1MHz;
            config->ws_speed = kI2S_WS_CLK_16KHZ;
            break;
    }

    *matchValueH_bitClk = (AHB_Freqency / 2 / config->bck_speed) - 1;
    *matchValueL_WsClk = ((AHB_Freqency / 2 / config->bck_speed) << 6) - 1;
}

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
status_t SOFTWARE_I2S_Init(SOFTWARE_I2S_Type *base, software_i2s_config_t *config)
{
    assert(config);

    status_t i2s_status;
    uint32_t eventCounterL, eventCounterH;
    uint32_t matchValueL, matchValueH;
    uint32_t AHB_Clock;
    int32_t instance;
    const sctimer_config_t *sct_config = &(config->sct_config);
    spi_slave_config_t *slaveConfig = &(config->spi_config);

    AHB_Clock = CLOCK_GetFreq(kCLOCK_BusClk);
    SOFTWARE_I2S_Get_SCT_FrequencyDiv(AHB_Clock, config, &matchValueH, &matchValueL);

    i2s_status = SCTIMER_Init(base, sct_config);
    if (i2s_status != kStatus_Success)
        return i2s_status;

    /* Schedule a match event for Counter L every xxx useconds */
    i2s_status = SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly,
                                                matchValueL, 0,
                                                SOFTWARE_I2S_WS_COUNTER,
                                                &eventCounterL);
    if (i2s_status != kStatus_Success)
    {
        return i2s_status;
    }
    /* Toggle bck output pin when Counter L event occurs */
    SCTIMER_SetupOutputToggleAction(base, config->ws_pin, eventCounterL);
    /* Reset Counter L when Counter L event occurs */
    SCTIMER_SetupCounterLimitAction(SCT0, SOFTWARE_I2S_WS_COUNTER, eventCounterL);

    /* Schedule a match event for Counter H every yyy useconds */
    i2s_status = SCTIMER_CreateAndScheduleEvent(base, kSCTIMER_MatchEventOnly,
                                                matchValueH, 0,
                                                SOFTWARE_I2S_BCK_COUNTER,
                                                &eventCounterH);
    if (i2s_status != kStatus_Success)
    {
        return i2s_status;
    }
    /* Toggle ws output pin when Counter H event occurs */
    SCTIMER_SetupOutputToggleAction(base, config->bck_pin, eventCounterH);
    /* Reset Counter H when Counter H event occurs */
    SCTIMER_SetupCounterLimitAction(base, SOFTWARE_I2S_BCK_COUNTER, eventCounterH);
    instance = SOFTWARE_I2S_GetInstance(base);
		leftshift_bias_bits = config->left_bias_bits;
    if (kI2S_SAMPLING_MODE_RISING == config->sampling_mode)
    {
        slaveConfig->polarity = kSPI_ClockPolarityActiveHigh;
    }
    else
    {
        slaveConfig->polarity = kSPI_ClockPolarityActiveLow;
    }
    memcpy(&g_configs[instance], config, sizeof(software_i2s_config_t));

    SPI_SlaveInit(SOFTWARE_I2S_SPI_SLAVE, slaveConfig);
    if (kI2S_IO_DMA == config->io_mode) {
        i2sXfer.txData = NULL;
        i2sXfer.rxData = (uint8_t *)&software_i2s_reg_rx_fifo;
        i2sXfer.dataSize = sizeof(software_i2s_reg_rx_fifo[0]) *
                           SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE;

        i2s_status = SOFTWARE_I2S_PrepareDDB(SOFTWARE_I2S_SPI_SLAVE,
                                             SOFTWARE_I2S_DMA,
                                             SOFTWARE_I2S_DMA_RX_CHANNEL,
                                             &h_i2s_dma_rx, &h_i2s_dma_tx,
                                             &h_slave, SOFTWARE_I2S_DDB_Cb_Rx,
                                             &leftshift_bias_bits, &i2sXfer,
                                             g_pingpong_desc);

        return i2s_status;
    }

    return kStatus_Success;
}

/*!
 * @brief Deinitialize the Software I2S module.
 *
 * This function de-nitializes the Software I2S module, including:
 *  - Disable the Software I2S module clock.
 *
 * @param base Software I2S peripheral base address.
 */
void SOFTWARE_I2S_Deinit(SOFTWARE_I2S_Type *base)
{
    /* Disable the clock to save power consumption */
    SCTIMER_Deinit(base);
}

/*!
 * @brief Enable/Disable the Software I2S Master.
 *
 * @param base     Software I2S peripheral base address.
 * @param enable   true to enable the Software I2S, false to disable.
 */
void SOFTWARE_I2S_Enable(SOFTWARE_I2S_Type *base, bool enable)
{
    uint32_t AHB_Clock = CLOCK_GetFreq(kCLOCK_BusClk);
    software_i2s_config_t *software_i2s_config;

    software_i2s_config = SOFTWARE_I2S_GetConfig(base);
    if (enable)
    {
        SCTIMER_StartTimer(base, SOFTWARE_I2S_BCK_COUNTER);
        /* below codes need an optimization */
        if (CLK_XTAL_16MHZ == AHB_Clock)
        {
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
        }
        else if (CLK_XTAL_32MHZ == AHB_Clock)
        {
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
            __asm("NOP");
        }
        else
        {
            /* "this MCU frequency is not verified, so you need your own timing" */
        }
        /* start ws clock */
        SCTIMER_StartTimer(base, SOFTWARE_I2S_WS_COUNTER);
        for (volatile uint32_t x = 0; x < SOFTWARE_I2S_DELAY_FOR_MIC_WAKEUP; x++)
        {
            ;
        }
        if (kI2S_IO_IRQ == software_i2s_config->io_mode)
        {
            SOFTWARE_I2S_SPI_Slave_Start_ISR(base, enable);
        }
        else if (kI2S_IO_DMA == software_i2s_config->io_mode)
        {
            SOFTWARE_I2S_SPI_Slave_Start_DMA(base, enable);
        }
    }
    else
    {
        if (kI2S_IO_IRQ == software_i2s_config->io_mode)
        {
            SOFTWARE_I2S_SPI_Slave_Start_ISR(base, enable);
        }
        else if (kI2S_IO_DMA == software_i2s_config->io_mode)
        {
            SOFTWARE_I2S_SPI_Slave_Start_DMA(base, enable);
        }
        SCTIMER_StopTimer(base, SOFTWARE_I2S_WS_COUNTER);
        SCTIMER_StopTimer(base, SOFTWARE_I2S_BCK_COUNTER);
    }
}

void SOFTWARE_I2S_IRQHandler(void)
{
    while ((SPI_GetStatusFlags(SOFTWARE_I2S_SPI_SLAVE) & kSPI_RxNotEmptyFlag))
    {
#if defined(MEMS_MIC_IS_16BIT) && (MEMS_MIC_IS_16BIT == TRUE)
        if (!i2s_fifo_rd.LowHalfWord)
        {
            i2s_fifo_rd.fifo[i2s_fifo_rd.wp++] = SPI_ReadData(SOFTWARE_I2S_SPI_SLAVE) << 1;
        }
        else
        {
            /* in 16bit MEMS MIC scenario, drop tail 16bit */
            SPI_ReadData(SOFTWARE_I2S_SPI_SLAVE);
        }
        i2s_fifo_rd.LowHalfWord = !i2s_fifo_rd.LowHalfWord;
#elif defined(MEMS_MIC_IS_32BIT) && (MEMS_MIC_IS_32BIT == TRUE)
        uint16_t *p = (uint16_t *)&i2s_fifo_rd.fifo[i2s_fifo_rd.wp];
        if (!i2s_fifo_rd.LowHalfWord)
        {
            /* in 32bit MEMS MIC scenario, stash all 32bit, belows are the MSB 16bit of MIC */
            *(p + 1) = SPI_ReadData(SOFTWARE_I2S_SPI_SLAVE);
        }
        else
        {
            /* in 32bit MEMS MIC scenario, stash all 32bit, belows are the LSB 16bit of MIC*/
            *(p) = SPI_ReadData(SOFTWARE_I2S_SPI_SLAVE);
            i2s_fifo_rd.fifo[i2s_fifo_rd.wp++] = *((uint32_t *)p) << 1;
        }
        i2s_fifo_rd.LowHalfWord = !i2s_fifo_rd.LowHalfWord;
#endif
        /* This is a loop FIFO, wrap automatically */
        if (i2s_fifo_rd.wp >= SOFTWARE_I2S_FIFO_SIZE)
        {
            i2s_fifo_rd.wp = 0;
            i2s_fifo_rd.is_newline = true;
        }
    }
    while ((SPI_GetStatusFlags(SOFTWARE_I2S_SPI_SLAVE) & kSPI_TxNotFullFlag))
    {
        SPI_WriteData(SOFTWARE_I2S_SPI_SLAVE, 0, 0);
    }

    if (i2s_fifo_rd.i2s_stash_finish_cmd_and_status)
    {
        SPI_DisableInterrupts(SOFTWARE_I2S_SPI_SLAVE, kSPI_RxLvlIrq | kSPI_TxLvlIrq);
    }
}

/* DMA double buffer call back for rx (recording) */
void SOFTWARE_I2S_DDB_Cb_Rx(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    int i;
    uint16_t tmp16b;
    uint8_t left_shift_bits = *((uint8_t *)param);

/* in 16bit MEMS MIC scenario, drop tail 16bit */
    if (kDMA_IntA == tcds) {
				for (i = 0; i < SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE / 2; i += 4)
        {
            tmp16b = ((uint8_t)software_i2s_reg_rx_fifo[i + 0] << 8) |
                     ((uint8_t)software_i2s_reg_rx_fifo[i + 1] << 0);
            i2s_fifo_rd.fifo[i2s_fifo_rd.wp++] = tmp16b << left_shift_bits;
/* in 32bit MEMS MIC scenario, Please add lower 16 bit from software_i2s_reg_rx_fifo into i2s_fifo_rd.fifo */
        }
    } else if (kDMA_IntB == tcds) {
				for (i = SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE / 2; i < SOFTWARE_I2S_DMA_PINGPONG_BUFFER_SIZE; i += 4)
        {
            tmp16b = ((uint8_t)software_i2s_reg_rx_fifo[i + 0] << 8) |
                     ((uint8_t)software_i2s_reg_rx_fifo[i + 1] << 0);
            i2s_fifo_rd.fifo[i2s_fifo_rd.wp++] = tmp16b << left_shift_bits;
/* in 32bit MEMS MIC scenario, Please add lower 16 bit from software_i2s_reg_rx_fifo into i2s_fifo_rd.fifo */
        }
        /* This is a loop FIFO, wrap automatically */
        if (i2s_fifo_rd.wp >= SOFTWARE_I2S_FIFO_SIZE)
        {
            i2s_fifo_rd.wp = 0;
            i2s_fifo_rd.is_newline = true;
        }
    } else {
        assert((kDMA_IntA != tcds) && (kDMA_IntB != tcds));
    }
}

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
                                 dma_descriptor_t Desc[])
{
    DMA_Init(dmabase);
    DMA_EnableChannel(dmabase, rx_ch + 1);
    DMA_EnableChannel(dmabase, rx_ch);
    DMA_CreateHandle(h_i2s_dma_tx, dmabase, rx_ch + 1);
    DMA_CreateHandle(h_i2s_dma_rx, dmabase, rx_ch);

    SPI_SlaveTransferCreateHandleDMA(spibase, h_slave, NULL, NULL, h_i2s_dma_tx, h_i2s_dma_rx);
		DMA_SetCallback(h_i2s_dma_rx, cb, userdata);
    if (kStatus_Success != SOFTWARE_I2S_SPI_MasterTransferDMA_DoubleBuff(spibase, h_slave, i2sXfer, Desc))
    {
        return kStatus_Fail;
    }

    return kStatus_Success;
}
