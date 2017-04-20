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
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;


                MODULE  ?cstartup
                ;; Forward declaration of sections.
                SECTION CSTACK:DATA:NOROOT(3)

                SECTION .intvec:CODE:NOROOT(2)

                EXTERN  __iar_program_start
                EXTERN  SystemInit
                PUBLIC  __vector_table
                PUBLIC  __vector_table_0x1c
                PUBLIC  __Vectors
                PUBLIC  __Vectors_End
                PUBLIC  __Vectors_Size

                DATA

__vector_table
                DCD     sfe(CSTACK)
                DCD     Reset_Handler

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
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size 	EQU 	__Vectors_End - __Vectors

                THUMB

                PUBWEAK Reset_Handler
                SECTION .text:CODE:REORDER:NOROOT(2)
; Reset Handler
Reset_Handler
                LDR     R1, =0x40000014
                LDR     R0, [R1]
                LDR     R2, =0x00000001
                ORR     R0, R0, R2
                STR     R0, [R1]

                LDR     R0, =sfe(CSTACK)
                MSR     MSP, R0

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__iar_program_start
                BX      R0

; Dummy Exception Handlers (infinite loops which can be modified)
        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
        B .

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B .

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B .

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B .

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B .

        PUBWEAK EXT_GPIO_WAKEUP_IRQHandler
        PUBWEAK EXT_GPIO_WAKEUP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
EXT_GPIO_WAKEUP_IRQHandler
        LDR     R0, =EXT_GPIO_WAKEUP_DriverIRQHandler
        BX      R0
        PUBWEAK OSC_IRQHandler
        PUBWEAK OSC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
OSC_IRQHandler
        LDR     R0, =OSC_DriverIRQHandler
        BX      R0
        PUBWEAK ACMP0_IRQHandler
        PUBWEAK ACMP0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ACMP0_IRQHandler
        LDR     R0, =ACMP0_DriverIRQHandler
        BX      R0
        PUBWEAK ACMP1_IRQHandler
        PUBWEAK ACMP1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ACMP1_IRQHandler
        LDR     R0, =ACMP1_DriverIRQHandler
        BX      R0
        PUBWEAK RTC_SEC_IRQHandler
        PUBWEAK RTC_SEC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
RTC_SEC_IRQHandler
        LDR     R0, =RTC_SEC_DriverIRQHandler
        BX      R0
        PUBWEAK RTC_FR_IRQHandler
        PUBWEAK RTC_FR_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
RTC_FR_IRQHandler
        LDR     R0, =RTC_FR_DriverIRQHandler
        BX      R0
        PUBWEAK CS_WAKEUP_IRQHandler
        PUBWEAK CS_WAKEUP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CS_WAKEUP_IRQHandler
        LDR     R0, =CS_WAKEUP_DriverIRQHandler
        BX      R0
        PUBWEAK CS_IRQHandler
        PUBWEAK CS_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CS_IRQHandler
        LDR     R0, =CS_DriverIRQHandler
        BX      R0
        PUBWEAK GPIOA_IRQHandler
        PUBWEAK GPIOA_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
GPIOA_IRQHandler
        LDR     R0, =GPIOA_DriverIRQHandler
        BX      R0
        PUBWEAK GPIOB_IRQHandler
        PUBWEAK GPIOB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
GPIOB_IRQHandler
        LDR     R0, =GPIOB_DriverIRQHandler
        BX      R0
        PUBWEAK DMA0_IRQHandler
        PUBWEAK DMA0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DMA0_IRQHandler
        LDR     R0, =DMA0_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT0_IRQHandler
        PUBWEAK PIN_INT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT0_IRQHandler
        LDR     R0, =PIN_INT0_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT1_IRQHandler
        PUBWEAK PIN_INT1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT1_IRQHandler
        LDR     R0, =PIN_INT1_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT2_IRQHandler
        PUBWEAK PIN_INT2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT2_IRQHandler
        LDR     R0, =PIN_INT2_DriverIRQHandler
        BX      R0
        PUBWEAK PIN_INT3_IRQHandler
        PUBWEAK PIN_INT3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
PIN_INT3_IRQHandler
        LDR     R0, =PIN_INT3_DriverIRQHandler
        BX      R0
        PUBWEAK OSC_INT_LOW_IRQHandler
        PUBWEAK OSC_INT_LOW_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
OSC_INT_LOW_IRQHandler
        LDR     R0, =OSC_INT_LOW_DriverIRQHandler
        BX      R0
        PUBWEAK USB0_IRQHandler
        PUBWEAK USB0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
USB0_IRQHandler
        LDR     R0, =USB0_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM0_IRQHandler
        PUBWEAK FLEXCOMM0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM0_IRQHandler
        LDR     R0, =FLEXCOMM0_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM1_IRQHandler
        PUBWEAK FLEXCOMM1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM1_IRQHandler
        LDR     R0, =FLEXCOMM1_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM2_IRQHandler
        PUBWEAK FLEXCOMM2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM2_IRQHandler
        LDR     R0, =FLEXCOMM2_DriverIRQHandler
        BX      R0
        PUBWEAK FLEXCOMM3_IRQHandler
        PUBWEAK FLEXCOMM3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLEXCOMM3_IRQHandler
        LDR     R0, =FLEXCOMM3_DriverIRQHandler
        BX      R0
        PUBWEAK BLE_IRQHandler
        PUBWEAK BLE_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BLE_IRQHandler
        LDR     R0, =BLE_DriverIRQHandler
        BX      R0
        PUBWEAK FSP_IRQHandler
        PUBWEAK FSP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FSP_IRQHandler
        LDR     R0, =FSP_DriverIRQHandler
        BX      R0
        PUBWEAK QDEC0_IRQHandler
        PUBWEAK QDEC0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
QDEC0_IRQHandler
        LDR     R0, =QDEC0_DriverIRQHandler
        BX      R0
        PUBWEAK QDEC1_IRQHandler
        PUBWEAK QDEC1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
QDEC1_IRQHandler
        LDR     R0, =QDEC1_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER0_IRQHandler
        PUBWEAK CTIMER0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER0_IRQHandler
        LDR     R0, =CTIMER0_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER1_IRQHandler
        PUBWEAK CTIMER1_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER1_IRQHandler
        LDR     R0, =CTIMER1_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER2_IRQHandler
        PUBWEAK CTIMER2_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER2_IRQHandler
        LDR     R0, =CTIMER2_DriverIRQHandler
        BX      R0
        PUBWEAK CTIMER3_IRQHandler
        PUBWEAK CTIMER3_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CTIMER3_IRQHandler
        LDR     R0, =CTIMER3_DriverIRQHandler
        BX      R0
        PUBWEAK WDT_IRQHandler
        PUBWEAK WDT_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
WDT_IRQHandler
        LDR     R0, =WDT_DriverIRQHandler
        BX      R0
        PUBWEAK ADC_IRQHandler
        PUBWEAK ADC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
ADC_IRQHandler
        LDR     R0, =ADC_DriverIRQHandler
        BX      R0
        PUBWEAK DAC_IRQHandler
        PUBWEAK DAC_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
DAC_IRQHandler
        LDR     R0, =DAC_DriverIRQHandler
        BX      R0
        PUBWEAK XTAL_READY_IRQHandler
        PUBWEAK XTAL_READY_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
XTAL_READY_IRQHandler
        LDR     R0, =XTAL_READY_DriverIRQHandler
        BX      R0
        PUBWEAK FLASH_IRQHandler
        PUBWEAK FLASH_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
FLASH_IRQHandler
        LDR     R0, =FLASH_DriverIRQHandler
        BX      R0
        PUBWEAK SPIFI0_IRQHandler
        PUBWEAK SPIFI0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SPIFI0_IRQHandler
        LDR     R0, =SPIFI0_DriverIRQHandler
        BX      R0
        PUBWEAK SCT0_IRQHandler
        PUBWEAK SCT0_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
SCT0_IRQHandler
        LDR     R0, =SCT0_DriverIRQHandler
        BX      R0
        PUBWEAK RNG_IRQHandler
        PUBWEAK RNG_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
RNG_IRQHandler
        LDR     R0, =RNG_DriverIRQHandler
        BX      R0
        PUBWEAK CALIB_IRQHandler
        PUBWEAK CALIB_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
CALIB_IRQHandler
        LDR     R0, =CALIB_DriverIRQHandler
        BX      R0
        PUBWEAK BLE_TX_IRQHandler
        PUBWEAK BLE_TX_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BLE_TX_IRQHandler
        LDR     R0, =BLE_TX_DriverIRQHandler
        BX      R0
        PUBWEAK BLE_RX_IRQHandler
        PUBWEAK BLE_RX_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BLE_RX_IRQHandler
        LDR     R0, =BLE_RX_DriverIRQHandler
        BX      R0
        PUBWEAK BLE_FREQ_HOP_IRQHandler
        PUBWEAK BLE_FREQ_HOP_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BLE_FREQ_HOP_IRQHandler
        LDR     R0, =BLE_FREQ_HOP_DriverIRQHandler
        BX      R0
        PUBWEAK BOD_IRQHandler
        PUBWEAK BOD_DriverIRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)
BOD_IRQHandler
        LDR     R0, =BOD_DriverIRQHandler
        BX      R0

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
DefaultISR
         B .
         END

