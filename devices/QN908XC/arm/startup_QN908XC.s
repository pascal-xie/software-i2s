;/*****************************************************************************
; * @file:    startup_QN908X.s
; * @purpose: CMSIS Cortex-M4 Core Device Startup File for the
; *           QN908X
; * @version: 1.0
; * @date:    2016-6-24
; *
; * Copyright: 1997 - 2016 Freescale Semiconductor, Inc. All Rights Reserved.
; *
; * Redistribution and use in source and binary forms, with or without modification,
; * are permitted provided that the following conditions are met:
; *
; * o Redistributions of source code must retain the above copyright notice, this list
; *   of conditions and the following disclaimer.
; *
; * o Redistributions in binary form must reproduce the above copyright notice, this
; *   list of conditions and the following disclaimer in the documentation and/or
; *   other materials provided with the distribution.
; *
; * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
; *   contributors may be used to endorse or promote products derived from this
; *   software without specific prior written permission.
; *
; * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
; * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
; * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
; * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
; * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
; * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
; * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
; *
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Limit|

__Vectors       DCD     |Image$$ARM_LIB_STACK$$ZI$$Limit| ; Top of Stack
                DCD     Reset_Handler               ; Reset Handler

                DCD     NMI_Handler
                DCD     HardFault_Handler
                DCD     MemManage_Handler
                DCD     BusFault_Handler
                DCD     UsageFault_Handler
__vector_table_0x1c
                DCD     0x00000000                  ; Checksum of the first 7 words
                DCD     0x000AA8FF                  ; CRP
                DCD     0x00000000                  ; Image type: Legacy = 0x00000000, Enhanced = 0xEDDC9494
                DCD     0x00000000                  ; Pointer to image header
                DCD     SVC_Handler
                DCD     DebugMon_Handler
                DCD     0
                DCD     PendSV_Handler
                DCD     SysTick_Handler

                ; External Interrupts
                DCD     EXT_GPIO_WAKEUP_IRQHandler  ;  0:  IO wakeup
                DCD     OSC_IRQHandler              ;  1:  BLE oscillator wakeup
                DCD     ACMP0_IRQHandler            ;  2:  ACMP0
                DCD     ACMP1_IRQHandler            ;  3:  ACMP1
                DCD     0                           ;  4:  Reserved
                DCD     RTC_SEC_IRQHandler          ;  5:  RTC second match
                DCD     RTC_FR_IRQHandler           ;  6:  RTC free running match
                DCD     CS_WAKEUP_IRQHandler        ;  7:  Capsense wakeup
                DCD     CS_IRQHandler               ;  8:  Capsense detection
                DCD     GPIOA_IRQHandler            ;  9:  GPIOA
                DCD     GPIOB_IRQHandler            ; 10:  GPIOB
                DCD     DMA0_IRQHandler             ; 11:  DMA
                DCD     PIN_INT0_IRQHandler         ; 12:  PINT0
                DCD     PIN_INT1_IRQHandler         ; 13:  PINT1
                DCD     PIN_INT2_IRQHandler         ; 14:  PINT2
                DCD     PIN_INT3_IRQHandler         ; 15:  PINT3
                DCD     OSC_INT_LOW_IRQHandler      ; 16:  Inverse of OSC_INT
                DCD     USB0_IRQHandler             ; 17:  USB
                DCD     FLEXCOMM0_IRQHandler        ; 18:  FLEXCOMM0
                DCD     FLEXCOMM1_IRQHandler        ; 19:  FLEXCOMM1
                DCD     FLEXCOMM2_IRQHandler        ; 20:  FLEXCOMM2
                DCD     FLEXCOMM3_IRQHandler        ; 21:  FLEXCOMM3
                DCD     BLE_IRQHandler              ; 22:  BLE
                DCD     FSP_IRQHandler              ; 23:  FSP
                DCD     QDEC0_IRQHandler            ; 24:  QDEC0
                DCD     QDEC1_IRQHandler            ; 25:  QDEC1
                DCD     CTIMER0_IRQHandler          ; 26:  Timer 0
                DCD     CTIMER1_IRQHandler          ; 27:  Timer 1
                DCD     CTIMER2_IRQHandler          ; 28:  Timer 2
                DCD     CTIMER3_IRQHandler          ; 29:  Timer 3
                DCD     WDT_IRQHandler              ; 30:  Watchdog Timer
                DCD     ADC_IRQHandler              ; 31:  ADC
                DCD     DAC_IRQHandler              ; 32:  DAC
                DCD     XTAL_READY_IRQHandler       ; 33:  XTAL ready
                DCD     FLASH_IRQHandler            ; 34:  FLASH
                DCD     SPIFI0_IRQHandler           ; 35:  SPIFI
                DCD     SCT0_IRQHandler             ; 36:  SCT
                DCD     0                           ; 37:  Reserved
                DCD     RNG_IRQHandler              ; 38:  RNG
                DCD     0                           ; 39:  Reserved
                DCD     CALIB_IRQHandler            ; 40:  Calibration
                DCD     0                           ; 41:  Reserved
                DCD     BLE_TX_IRQHandler           ; 42:  BLE Tx
                DCD     BLE_RX_IRQHandler           ; 43:  BLE Rx
                DCD     BLE_FREQ_HOP_IRQHandler     ; 44:  BLE FREQ HOP
                DCD     0                           ; 45:  Reserved
                DCD     0                           ; 46:  Reserved
                DCD     0                           ; 47:  Reserved
                DCD     0                           ; 48:  Reserved
                DCD     0                           ; 49:  Reserved
                DCD     0                           ; 50:  Reserved
                DCD     BOD_IRQHandler              ; 51:  BOD
                DCD     0x00000000                  ; 52:  Boot feature

                AREA    |.text|, CODE, READONLY

Reset_Handler   PROC
                EXPORT  Reset_Handler                   [WEAK]

                ; write 1 to SYS_MODE_CTRL's REMAP bit to remap flash at address 0x0
                LDR     R1, =0x40000014   ; SYS_MODE_CTRL
                LDR     R0, [R1]
                LDR     R2, =0x00000001
                ORR     R0, R0, R2
                STR     R0, [R1]

                LDR     R0, =|Image$$ARM_LIB_STACK$$ZI$$Limit|
                MSR     MSP, R0

                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP
                ALIGN

; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler     PROC
                EXPORT  NMI_Handler                     [WEAK]
                B       .
                ENDP

HardFault_Handler \
                PROC
                EXPORT  HardFault_Handler               [WEAK]
                B       .
                ENDP

MemManage_Handler     PROC
                EXPORT  MemManage_Handler               [WEAK]
                B       .
                ENDP

BusFault_Handler PROC
                EXPORT  BusFault_Handler                [WEAK]
                B       .
                ENDP

UsageFault_Handler PROC
                EXPORT  UsageFault_Handler              [WEAK]
                B       .
                ENDP

SVC_Handler     PROC
                EXPORT  SVC_Handler                     [WEAK]
                B       .
                ENDP

DebugMon_Handler PROC
                EXPORT  DebugMon_Handler                [WEAK]
                B       .
                ENDP

PendSV_Handler  PROC
                EXPORT  PendSV_Handler                  [WEAK]
                B       .
                ENDP

SysTick_Handler PROC
                EXPORT  SysTick_Handler                 [WEAK]
                B       .
                ENDP

EXT_GPIO_WAKEUP_IRQHandler\
                PROC
                EXPORT     EXT_GPIO_WAKEUP_IRQHandler   [WEAK]
                LDR        R0, =EXT_GPIO_WAKEUP_DriverIRQHandler
                BX         R0
                ENDP

OSC_IRQHandler\
                PROC
                EXPORT     OSC_IRQHandler               [WEAK]
                LDR        R0, =OSC_DriverIRQHandler
                BX         R0
                ENDP

ACMP0_IRQHandler\
                PROC
                EXPORT     ACMP0_IRQHandler             [WEAK]
                LDR        R0, =ACMP0_DriverIRQHandler
                BX         R0
                ENDP

ACMP1_IRQHandler\
                PROC
                EXPORT     ACMP1_IRQHandler             [WEAK]
                LDR        R0, =ACMP1_DriverIRQHandler
                BX         R0
                ENDP

RTC_SEC_IRQHandler\
                PROC
                EXPORT     RTC_SEC_IRQHandler           [WEAK]
                LDR        R0, =RTC_SEC_DriverIRQHandler
                BX         R0
                ENDP

RTC_FR_IRQHandler\
                PROC
                EXPORT     RTC_FR_IRQHandler            [WEAK]
                LDR        R0, =RTC_FR_DriverIRQHandler
                BX         R0
                ENDP

CS_WAKEUP_IRQHandler\
                PROC
                EXPORT     CS_WAKEUP_IRQHandler         [WEAK]
                LDR        R0, =CS_WAKEUP_DriverIRQHandler
                BX         R0
                ENDP

CS_IRQHandler\
                PROC
                EXPORT     CS_IRQHandler                [WEAK]
                LDR        R0, =CS_DriverIRQHandler
                BX         R0
                ENDP

GPIOA_IRQHandler\
                PROC
                EXPORT     GPIOA_IRQHandler             [WEAK]
                LDR        R0, =GPIOA_DriverIRQHandler
                BX         R0
                ENDP

GPIOB_IRQHandler\
                PROC
                EXPORT     GPIOB_IRQHandler             [WEAK]
                LDR        R0, =GPIOB_DriverIRQHandler
                BX         R0
                ENDP

DMA0_IRQHandler\
                PROC
                EXPORT     DMA0_IRQHandler              [WEAK]
                LDR        R0, =DMA0_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT0_IRQHandler\
                PROC
                EXPORT     PIN_INT0_IRQHandler         [WEAK]
                LDR        R0, =PIN_INT0_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT1_IRQHandler\
                PROC
                EXPORT     PIN_INT1_IRQHandler         [WEAK]
                LDR        R0, =PIN_INT1_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT2_IRQHandler\
                PROC
                EXPORT     PIN_INT2_IRQHandler         [WEAK]
                LDR        R0, =PIN_INT2_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT3_IRQHandler\
                PROC
                EXPORT     PIN_INT3_IRQHandler         [WEAK]
                LDR        R0, =PIN_INT3_DriverIRQHandler
                BX         R0
                ENDP

OSC_INT_LOW_IRQHandler\
                PROC
                EXPORT     OSC_INT_LOW_IRQHandler       [WEAK]
                LDR        R0, =OSC_INT_LOW_DriverIRQHandler
                BX         R0
                ENDP

USB0_IRQHandler\
                PROC
                EXPORT     USB0_IRQHandler              [WEAK]
                LDR        R0, =USB0_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM0_IRQHandler\
                PROC
                EXPORT     FLEXCOMM0_IRQHandler         [WEAK]
                LDR        R0, =FLEXCOMM0_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM1_IRQHandler\
                PROC
                EXPORT     FLEXCOMM1_IRQHandler         [WEAK]
                LDR        R0, =FLEXCOMM1_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM2_IRQHandler\
                PROC
                EXPORT     FLEXCOMM2_IRQHandler         [WEAK]
                LDR        R0, =FLEXCOMM2_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM3_IRQHandler\
                PROC
                EXPORT     FLEXCOMM3_IRQHandler         [WEAK]
                LDR        R0, =FLEXCOMM3_DriverIRQHandler
                BX         R0
                ENDP

BLE_IRQHandler\
                PROC
                EXPORT     BLE_IRQHandler               [WEAK]
                LDR        R0, =BLE_DriverIRQHandler
                BX         R0
                ENDP

FSP_IRQHandler\
                PROC
                EXPORT     FSP_IRQHandler               [WEAK]
                LDR        R0, =FSP_DriverIRQHandler
                BX         R0
                ENDP

QDEC0_IRQHandler\
                PROC
                EXPORT     QDEC0_IRQHandler             [WEAK]
                LDR        R0, =QDEC0_DriverIRQHandler
                BX         R0
                ENDP

QDEC1_IRQHandler\
                PROC
                EXPORT     QDEC1_IRQHandler             [WEAK]
                LDR        R0, =QDEC1_DriverIRQHandler
                BX         R0
                ENDP

CTIMER0_IRQHandler\
                PROC
                EXPORT     CTIMER0_IRQHandler           [WEAK]
                LDR        R0, =CTIMER0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER1_IRQHandler\
                PROC
                EXPORT     CTIMER1_IRQHandler           [WEAK]
                LDR        R0, =CTIMER1_DriverIRQHandler
                BX         R0
                ENDP

CTIMER2_IRQHandler\
                PROC
                EXPORT     CTIMER2_IRQHandler           [WEAK]
                LDR        R0, =CTIMER2_DriverIRQHandler
                BX         R0
                ENDP

CTIMER3_IRQHandler\
                PROC
                EXPORT     CTIMER3_IRQHandler           [WEAK]
                LDR        R0, =CTIMER3_DriverIRQHandler
                BX         R0
                ENDP

WDT_IRQHandler\
                PROC
                EXPORT     WDT_IRQHandler               [WEAK]
                LDR        R0, =WDT_DriverIRQHandler
                BX         R0
                ENDP

ADC_IRQHandler\
                PROC
                EXPORT     ADC_IRQHandler               [WEAK]
                LDR        R0, =ADC_DriverIRQHandler
                BX         R0
                ENDP

DAC_IRQHandler\
                PROC
                EXPORT     DAC_IRQHandler               [WEAK]
                LDR        R0, =DAC_DriverIRQHandler
                BX         R0
                ENDP

XTAL_READY_IRQHandler\
                PROC
                EXPORT     XTAL_READY_IRQHandler        [WEAK]
                LDR        R0, =XTAL_READY_DriverIRQHandler
                BX         R0
                ENDP

FLASH_IRQHandler\
                PROC
                EXPORT     FLASH_IRQHandler             [WEAK]
                LDR        R0, =FLASH_DriverIRQHandler
                BX         R0
                ENDP

SPIFI0_IRQHandler\
                PROC
                EXPORT     SPIFI0_IRQHandler            [WEAK]
                LDR        R0, =SPIFI0_DriverIRQHandler
                BX         R0
                ENDP

SCT0_IRQHandler\
                PROC
                EXPORT     SCT0_IRQHandler              [WEAK]
                LDR        R0, =SCT0_DriverIRQHandler
                BX         R0
                ENDP

RNG_IRQHandler\
                PROC
                EXPORT     RNG_IRQHandler               [WEAK]
                LDR        R0, =RNG_DriverIRQHandler
                BX         R0
                ENDP

CALIB_IRQHandler\
                PROC
                EXPORT     CALIB_IRQHandler             [WEAK]
                LDR        R0, =CALIB_DriverIRQHandler
                BX         R0
                ENDP

BLE_TX_IRQHandler\
                PROC
                EXPORT     BLE_TX_IRQHandler            [WEAK]
                LDR        R0, =BLE_TX_DriverIRQHandler
                BX         R0
                ENDP

BLE_RX_IRQHandler\
                PROC
                EXPORT     BLE_RX_IRQHandler            [WEAK]
                LDR        R0, =BLE_RX_DriverIRQHandler
                BX         R0
                ENDP

BLE_FREQ_HOP_IRQHandler\
                PROC
                EXPORT     BLE_FREQ_HOP_IRQHandler      [WEAK]
                LDR        R0, =BLE_FREQ_HOP_DriverIRQHandler
                BX         R0
                ENDP

BOD_IRQHandler\
                PROC
                EXPORT     BOD_IRQHandler               [WEAK]
                LDR        R0, =BOD_DriverIRQHandler
                BX         R0
                ENDP

Default_Handler PROC
                EXPORT     EXT_GPIO_WAKEUP_DriverIRQHandler     [WEAK]
                EXPORT     OSC_DriverIRQHandler                 [WEAK]
                EXPORT     ACMP0_DriverIRQHandler               [WEAK]
                EXPORT     ACMP1_DriverIRQHandler               [WEAK]
                EXPORT     RTC_SEC_DriverIRQHandler             [WEAK]
                EXPORT     RTC_FR_DriverIRQHandler              [WEAK]
                EXPORT     CS_WAKEUP_DriverIRQHandler           [WEAK]
                EXPORT     CS_DriverIRQHandler                  [WEAK]
                EXPORT     GPIOA_DriverIRQHandler               [WEAK]
                EXPORT     GPIOB_DriverIRQHandler               [WEAK]
                EXPORT     DMA0_DriverIRQHandler                [WEAK]
                EXPORT     PIN_INT0_DriverIRQHandler            [WEAK]
                EXPORT     PIN_INT1_DriverIRQHandler            [WEAK]
                EXPORT     PIN_INT2_DriverIRQHandler            [WEAK]
                EXPORT     PIN_INT3_DriverIRQHandler            [WEAK]
                EXPORT     OSC_INT_LOW_DriverIRQHandler         [WEAK]
                EXPORT     USB0_DriverIRQHandler                [WEAK]
                EXPORT     FLEXCOMM0_DriverIRQHandler           [WEAK]
                EXPORT     FLEXCOMM1_DriverIRQHandler           [WEAK]
                EXPORT     FLEXCOMM2_DriverIRQHandler           [WEAK]
                EXPORT     FLEXCOMM3_DriverIRQHandler           [WEAK]
                EXPORT     BLE_DriverIRQHandler                 [WEAK]
                EXPORT     FSP_DriverIRQHandler                 [WEAK]
                EXPORT     QDEC0_DriverIRQHandler               [WEAK]
                EXPORT     QDEC1_DriverIRQHandler               [WEAK]
                EXPORT     CTIMER0_DriverIRQHandler             [WEAK]
                EXPORT     CTIMER1_DriverIRQHandler             [WEAK]
                EXPORT     CTIMER2_DriverIRQHandler             [WEAK]
                EXPORT     CTIMER3_DriverIRQHandler             [WEAK]
                EXPORT     WDT_DriverIRQHandler                 [WEAK]
                EXPORT     ADC_DriverIRQHandler                 [WEAK]
                EXPORT     DAC_DriverIRQHandler                 [WEAK]
                EXPORT     XTAL_READY_DriverIRQHandler          [WEAK]
                EXPORT     FLASH_DriverIRQHandler               [WEAK]
                EXPORT     SPIFI0_DriverIRQHandler              [WEAK]
                EXPORT     SCT0_DriverIRQHandler                [WEAK]
                EXPORT     RNG_DriverIRQHandler                 [WEAK]
                EXPORT     CALIB_DriverIRQHandler               [WEAK]
                EXPORT     BLE_TX_DriverIRQHandler              [WEAK]
                EXPORT     BLE_RX_DriverIRQHandler              [WEAK]
                EXPORT     BLE_FREQ_HOP_DriverIRQHandler        [WEAK]
                EXPORT     BOD_DriverIRQHandler                 [WEAK]

EXT_GPIO_WAKEUP_DriverIRQHandler
OSC_DriverIRQHandler
ACMP0_DriverIRQHandler
ACMP1_DriverIRQHandler
RTC_SEC_DriverIRQHandler
RTC_FR_DriverIRQHandler
CS_WAKEUP_DriverIRQHandler
CS_DriverIRQHandler
GPIOA_DriverIRQHandler
GPIOB_DriverIRQHandler
DMA0_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
OSC_INT_LOW_DriverIRQHandler
USB0_DriverIRQHandler
FLEXCOMM0_DriverIRQHandler
FLEXCOMM1_DriverIRQHandler
FLEXCOMM2_DriverIRQHandler
FLEXCOMM3_DriverIRQHandler
BLE_DriverIRQHandler
FSP_DriverIRQHandler
QDEC0_DriverIRQHandler
QDEC1_DriverIRQHandler
CTIMER0_DriverIRQHandler
CTIMER1_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER3_DriverIRQHandler
WDT_DriverIRQHandler
ADC_DriverIRQHandler
DAC_DriverIRQHandler
XTAL_READY_DriverIRQHandler
FLASH_DriverIRQHandler
SPIFI0_DriverIRQHandler
SCT0_DriverIRQHandler
RNG_DriverIRQHandler
CALIB_DriverIRQHandler
BLE_TX_DriverIRQHandler
BLE_RX_DriverIRQHandler
BLE_FREQ_HOP_DriverIRQHandler
BOD_DriverIRQHandler


                B       .

                ENDP


                ALIGN


                END

