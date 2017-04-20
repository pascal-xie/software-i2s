Overview
========
The interrupt mems_mic_i2s example shows how to use software based i2s driver
to create i2s timing and how to use spi functional API to do interrupt transfer
as a slave and to receive microphone voice data:

In this example, the spi instance as slave. Slave receives data froma master
(I2S interface microphone).
This example needs to work with spi driver, soft_i2s driver, sctimer driver.

Hardware requirements
=====================
- Mini/micro USB cable
- QN908XDK board
- Daughter microphone board
- Personal Computer

Board settings
============
No special settings are required.

Prepare the Demo
===============
1.  Connect a USB cable between the host PC and the UART port (J2) on the
target board.

2.  Open a serial terminal with the following settings:
- 460800 baud rate
- 8 data bits
- No parity
- One stop bit
- No flow control

3.  Download the program to the target board.

4.  Either press the reset button on your board or launch
the debugger in your IDE to begin running the demo.

Running the demo
================
Ready to receive your uart log to file (ex: demo.wav),
Press reset key from keyboard and speak out to microphone.
Use goldwave to play the log file with below format:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

File Type:  Dialogic(vox)
Attributes: ADPCM 4 bit, mono
Rate(Hz):   16000
...

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Any issues, please reporte to Pascal.xie@qq.com,
or tell me: https://www.linkedin.com/in/jianhua-xie-387ab6b4/

