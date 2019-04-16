# software_i2s
Uses NXP SCTimer and NXP LPC-SPI to provide a curtain of I2S interface emulation (software based I2S)

Add driver and example files for i2s bus emulation with timer and spi

There are 3 kinds of interfaces for popular MEMS microphones: analog
interface, I2S slave interface and digital PDM interface. For some Arm
cortex-M MCU like QN9080, QN9020 and other ARM Cortex MCU, neither
I2S master interface nor PDM interface is present.

This patches serial uses NXP SCTimer and NXP LPC-SPI to provide a
curtain of I2S interface emulation (software based I2S). The main
feature includes:
-------------------------------
1. Supports I2S sensor recording, will enhance to I2S sensor playback
2. I2S Pins configurable
3. Supports all PCM format, 16 bits and 32 bits can be switched
4. supports I2S SAMPLING_MODE_FALLING, verified SPH0645LM4H sensor
5. supports I2S SAMPLING_MODE_RISING, verified ICS_43432 sensor
6. I2S FIFO depth (entry size) configurable
7. I2S supports low power consumption - Clock can be closed
8. Supports IMA-Microsoft ADPCM codec/decoder
9. Supports I2S Interrupt mode to offload MCU
10.Supports I2S DMA mode with double buffer
11.Supports Sampling rate: 8kHz, 16kHz, 32kHz, 64kHz

to do list:
-------------------------------
1. support I2S playback sensor
2. support double channels and more
3. support BIG_ENDIAN and LITTLE_ENDIAN at all FIFO width
4. support DMA while playback to further offload MCU
5. support emulation by either 2 separate CTimers or a single SCTimer

limitation:
1. Only Mono for current version
2. DMA mode does not support low power mode.

Signed-off-by: Jianhua Xie <pascal.xie@qq.com>
