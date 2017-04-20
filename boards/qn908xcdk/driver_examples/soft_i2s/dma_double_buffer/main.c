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

#include "board.h"
#include "app.h"
#include "fsl_debug_console.h"
#include "fsl_usart.h"
#include "adpcm.h"
#include "fsl_software_i2s.h"

typedef struct _adpcm_buf
{
    uint8_t buf[ADPCM_BUF_LENGTH];
    uint32_t wp, rp; /* write pointer, read pointer */
    adpcm_state_t s; /* adpcm state, which adpcm codec algrorithm needs */
    bool is_newline;
} adpcm_buf_t;

adpcm_buf_t adpcm_buf = {0};

static void pcm2adpcm_rt_helper(software_i2s_fifo_t *i2s_fifo_p, adpcm_buf_t *adpcm_p)
{
    if ((!i2s_fifo_p->is_newline) && ((i2s_fifo_p->wp - i2s_fifo_p->rp) < ADPCM_SRC_LENGTH_DBYTE))
        return;

    if ((i2s_fifo_p->is_newline) && ((SOFTWARE_I2S_FIFO_SIZE - i2s_fifo_p->rp) < ADPCM_SRC_LENGTH_DBYTE))
        return;

    adpcm_coder((short *)&i2s_fifo_p->fifo[i2s_fifo_p->rp], (char *)&adpcm_p->buf[adpcm_p->wp], ADPCM_SRC_LENGTH_BYTE,
                &adpcm_p->s);
    i2s_fifo_p->rp += ADPCM_SRC_LENGTH_DBYTE;
    adpcm_p->wp += ADPCM_DST_LENGTH_BYTE;

    if (i2s_fifo_p->rp >= SOFTWARE_I2S_FIFO_SIZE)
    {
        i2s_fifo_p->rp = 0;
        i2s_fifo_p->is_newline = false;
    }

    if (adpcm_p->wp >= ADPCM_BUF_LENGTH)
    {
        adpcm_p->wp = 0;
        adpcm_p->is_newline = true;
    }

    return;
}

int main(void)
{
    usart_config_t config;
    software_i2s_config_t software_i2s_config;
    uint8_t *p;

    /* Init the boards */
    BOARD_InitHardware();
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_FAST_BAUDRATE;
    config.enableTx = true;
    USART_Init(I2S_TX_USART, &config, CLOCK_GetFreq(kCLOCK_BusClk));

    SOFTWARE_I2S_GetDefaultConfig(&software_i2s_config);
    software_i2s_config.io_mode = kI2S_IO_DMA;
    software_i2s_config.ws_speed = kI2S_WS_CLK_64KHZ;
    software_i2s_config.spi_config.dataWidth = kSPI_Data8Bits;
    software_i2s_config.spi_config.rxWatermark = kSPI_RxFifo1;
    software_i2s_config.left_bias_bits = 1;
    /* some i2s sensors are SAMPLING MODE FALLING like SPH0645LM4H,
     * others are SAMPLING_MODE_RISING like ICS_43432, please refer
     * to your I2s senser datasheet.
     */
    software_i2s_config.sampling_mode = kI2S_SAMPLING_MODE_FALLING;
    SOFTWARE_I2S_Init(SOFTWARE_I2S0, &software_i2s_config);
    SOFTWARE_I2S_Enable(SOFTWARE_I2S0, true);

    while (adpcm_buf.wp < (ADPCM_BUF_LENGTH - 4))
    {
        pcm2adpcm_rt_helper(&i2s_fifo_rd, &adpcm_buf);
    }

    SOFTWARE_I2S_Enable(SOFTWARE_I2S0, false);

    p = (uint8_t *)&adpcm_buf.buf[adpcm_buf.rp];
    for (int i = 0; i < ADPCM_BUF_LENGTH; i++)
    {
        PRINTF("%c", *p++);
        adpcm_buf.rp++;
        if (adpcm_buf.rp >= ADPCM_BUF_LENGTH)
        {
            adpcm_buf.rp = 0;
            adpcm_buf.is_newline = false;
        }
    }

    while (true)
        ;
}
