/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_QN908X.s                                                    */
/*  @purpose: CMSIS Cortex-M4 Core Device Startup File                                    */
/*            QN908X                                                              */
/*  @version: 1.0                                                                         */
/*  @date:    2016-11-2                                                                   */
/*  @build:   b161214                                                                     */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright (c) 1997 - 2016 , Freescale Semiconductor, Inc.                              */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of Freescale Semiconductor, Inc. nor the names of its               */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*                                                                                        */
/* Copyright (c) 2016 , NXP Semiconductors, Inc.                                          */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of NXP Semiconductors, Inc. nor the names of its                    */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/
    .syntax unified
    .arch armv7-m

    .section .isr_vector, "a"
    .align 2
    .globl __Vectors
__Vectors:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                               /* MPU Fault Handler*/
    .long   BusFault_Handler                                /* Bus Fault Handler*/
    .long   UsageFault_Handler                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0x000AA8FF                                      /* CRP*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                /* Debug Monitor Handler*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   EXT_GPIO_WAKEUP_IRQHandler                      /* IO wakeup */
    .long   OSC_IRQHandler                                  /* BLE oscillator wakeup */
    .long   ACMP0_IRQHandler                                /* ACMP0 */
    .long   ACMP1_IRQHandler                                /* ACMP1 */
    .long   0                                               /* Reserved */
    .long   RTC_SEC_IRQHandler                              /* RTC second match */
    .long   RTC_FR_IRQHandler                               /* RTC free running match */
    .long   CS_WAKEUP_IRQHandler                            /* Capsense wakeup */
    .long   CS_IRQHandler                                   /* Capsense detection */
    .long   GPIOA_IRQHandler                                /* GPIOA */
    .long   GPIOB_IRQHandler                                /* GPIOB */
    .long   DMA0_IRQHandler                                 /* DMA */
    .long   PIN_INT0_IRQHandler                             /* PINT0 */
    .long   PIN_INT1_IRQHandler                             /* PINT1 */
    .long   PIN_INT2_IRQHandler                             /* PINT2 */
    .long   PIN_INT3_IRQHandler                             /* PINT3 */
    .long   OSC_INT_LOW_IRQHandler                          /* Inverse of OSC_INT */
    .long   USB0_IRQHandler                                 /* USB */
    .long   FLEXCOMM0_IRQHandler                            /* FLEXCOMM0 */
    .long   FLEXCOMM1_IRQHandler                            /* FLEXCOMM1 */
    .long   FLEXCOMM2_IRQHandler                            /* FLEXCOMM2 */
    .long   FLEXCOMM3_IRQHandler                            /* FLEXCOMM3 */
    .long   BLE_IRQHandler                                  /* BLE */
    .long   FSP_IRQHandler                                  /* FSP */
    .long   QDEC0_IRQHandler                                /* QDEC0 */
    .long   QDEC1_IRQHandler                                /* QDEC1 */
    .long   CTIMER0_IRQHandler                              /* Timer 0 */
    .long   CTIMER1_IRQHandler                              /* Timer 1 */
    .long   CTIMER2_IRQHandler                              /* Timer 2 */
    .long   CTIMER3_IRQHandler                              /* Timer 3 */
    .long   WDT_IRQHandler                                  /* Watchdog Timer */
    .long   ADC_IRQHandler                                  /* ADC */
    .long   DAC_IRQHandler                                  /* DAC */
    .long   XTAL_READY_IRQHandler                           /* XTAL ready */
    .long   FLASH_IRQHandler                                /* FLASH */
    .long   SPIFI0_IRQHandler                               /* SPIFI */
    .long   SCT0_IRQHandler                                 /* SCT */
    .long   RTC_CAP_IRQHandler                              /* RTC capture */
    .long   RNG_IRQHandler                                  /* RNG */
    .long   0                                               /* Reserved */
    .long   CALIB_IRQHandler                                /* Calibration */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   0                                               /* Reserved */
    .long   BOD_IRQHandler                                  /* BOD */

    .size    __Vectors, . - __Vectors

    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    ldr     r1, =0x40000014
    ldr     r0, [R1]
    ldr     r2, =0x00000001
    orr     r0, R0, R2
    str     r0, [R1]
#ifndef __NO_SYSTEM_INIT
    ldr   r0,=SystemInit
    blx   r0
#endif
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

#if 1
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2
#endif /* __STARTUP_CLEAR_BSS */

#ifndef __START
#define __START _start
#endif
#ifndef __ATOLLIC__
    ldr   r0,=__START
    blx   r0
#else
    ldr   r0,=__libc_init_array
    blx   r0
    ldr   r0,=main
    bx    r0
#endif
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b DefaultISR
    .size DefaultISR, . - DefaultISR

    .align 1
    .thumb_func
    .weak NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    ldr   r0,=NMI_Handler
    bx    r0
    .size NMI_Handler, . - NMI_Handler

    .align 1
    .thumb_func
    .weak HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    ldr   r0,=HardFault_Handler
    bx    r0
    .size HardFault_Handler, . - HardFault_Handler

    .align 1
    .thumb_func
    .weak SVC_Handler
    .type SVC_Handler, %function
SVC_Handler:
    ldr   r0,=SVC_Handler
    bx    r0
    .size SVC_Handler, . - SVC_Handler

    .align 1
    .thumb_func
    .weak PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    ldr   r0,=PendSV_Handler
    bx    r0
    .size PendSV_Handler, . - PendSV_Handler

    .align 1
    .thumb_func
    .weak SysTick_Handler
    .type SysTick_Handler, %function
SysTick_Handler:
    ldr   r0,=SysTick_Handler
    bx    r0
    .size SysTick_Handler, . - SysTick_Handler

    .align 1
    .thumb_func
    .weak EXT_GPIO_WAKEUP_IRQHandler
    .type EXT_GPIO_WAKEUP_IRQHandler, %function
EXT_GPIO_WAKEUP_IRQHandler:
    ldr   r0,=EXT_GPIO_WAKEUP_DriverIRQHandler
    bx    r0
    .size EXT_GPIO_WAKEUP_IRQHandler, . - EXT_GPIO_WAKEUP_IRQHandler

    .align 1
    .thumb_func
    .weak OSC_IRQHandler
    .type OSC_IRQHandler, %function
OSC_IRQHandler:
    ldr   r0,=OSC_DriverIRQHandler
    bx    r0
    .size OSC_IRQHandler, . - OSC_IRQHandler

    .align 1
    .thumb_func
    .weak ACMP0_IRQHandler
    .type ACMP0_IRQHandler, %function
ACMP0_IRQHandler:
    ldr   r0,=ACMP0_DriverIRQHandler
    bx    r0
    .size ACMP0_IRQHandler, . - ACMP0_IRQHandler

    .align 1
    .thumb_func
    .weak ACMP1_IRQHandler
    .type ACMP1_IRQHandler, %function
ACMP1_IRQHandler:
    ldr   r0,=ACMP1_DriverIRQHandler
    bx    r0
    .size ACMP1_IRQHandler, . - ACMP1_IRQHandler

    .align 1
    .thumb_func
    .weak RTC_SEC_IRQHandler
    .type RTC_SEC_IRQHandler, %function
RTC_SEC_IRQHandler:
    ldr   r0,=RTC_SEC_DriverIRQHandler
    bx    r0
    .size RTC_SEC_IRQHandler, . - RTC_SEC_IRQHandler

    .align 1
    .thumb_func
    .weak RTC_FR_IRQHandler
    .type RTC_FR_IRQHandler, %function
RTC_FR_IRQHandler:
    ldr   r0,=RTC_FR_DriverIRQHandler
    bx    r0
    .size RTC_FR_IRQHandler, . - RTC_FR_IRQHandler

    .align 1
    .thumb_func
    .weak CS_WAKEUP_IRQHandler
    .type CS_WAKEUP_IRQHandler, %function
CS_WAKEUP_IRQHandler:
    ldr   r0,=CS_WAKEUP_DriverIRQHandler
    bx    r0
    .size CS_WAKEUP_IRQHandler, . - CS_WAKEUP_IRQHandler

    .align 1
    .thumb_func
    .weak CS_IRQHandler
    .type CS_IRQHandler, %function
CS_IRQHandler:
    ldr   r0,=CS_DriverIRQHandler
    bx    r0
    .size CS_IRQHandler, . - CS_IRQHandler

    .align 1
    .thumb_func
    .weak GPIOA_IRQHandler
    .type GPIOA_IRQHandler, %function
GPIOA_IRQHandler:
    ldr   r0,=GPIOA_DriverIRQHandler
    bx    r0
    .size GPIOA_IRQHandler, . - GPIOA_IRQHandler

    .align 1
    .thumb_func
    .weak GPIOB_IRQHandler
    .type GPIOB_IRQHandler, %function
GPIOB_IRQHandler:
    ldr   r0,=GPIOB_DriverIRQHandler
    bx    r0
    .size GPIOB_IRQHandler, . - GPIOB_IRQHandler

    .align 1
    .thumb_func
    .weak DMA0_IRQHandler
    .type DMA0_IRQHandler, %function
DMA0_IRQHandler:
    ldr   r0,=DMA0_DriverIRQHandler
    bx    r0
    .size DMA0_IRQHandler, . - DMA0_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT0_IRQHandler
    .type PIN_INT0_IRQHandler, %function
PIN_INT0_IRQHandler:
    ldr   r0,=PIN_INT0_DriverIRQHandler
    bx    r0
    .size PIN_INT0_IRQHandler, . - PIN_INT0_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT1_IRQHandler
    .type PIN_INT1_IRQHandler, %function
PIN_INT1_IRQHandler:
    ldr   r0,=PIN_INT1_DriverIRQHandler
    bx    r0
    .size PIN_INT1_IRQHandler, . - PIN_INT1_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT2_IRQHandler
    .type PIN_INT2_IRQHandler, %function
PIN_INT2_IRQHandler:
    ldr   r0,=PIN_INT2_DriverIRQHandler
    bx    r0
    .size PIN_INT2_IRQHandler, . - PIN_INT2_IRQHandler

    .align 1
    .thumb_func
    .weak PIN_INT3_IRQHandler
    .type PIN_INT3_IRQHandler, %function
PIN_INT3_IRQHandler:
    ldr   r0,=PIN_INT3_DriverIRQHandler
    bx    r0
    .size PIN_INT3_IRQHandler, . - PIN_INT3_IRQHandler

    .align 1
    .thumb_func
    .weak OSC_INT_LOW_IRQHandler
    .type OSC_INT_LOW_IRQHandler, %function
OSC_INT_LOW_IRQHandler:
    ldr   r0,=OSC_INT_LOW_DriverIRQHandler
    bx    r0
    .size OSC_INT_LOW_IRQHandler, . - OSC_INT_LOW_IRQHandler

    .align 1
    .thumb_func
    .weak USB0_IRQHandler
    .type USB0_IRQHandler, %function
USB0_IRQHandler:
    ldr   r0,=USB0_DriverIRQHandler
    bx    r0
    .size USB0_IRQHandler, . - USB0_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM0_IRQHandler
    .type FLEXCOMM0_IRQHandler, %function
FLEXCOMM0_IRQHandler:
    ldr   r0,=FLEXCOMM0_DriverIRQHandler
    bx    r0
    .size FLEXCOMM0_IRQHandler, . - FLEXCOMM0_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM1_IRQHandler
    .type FLEXCOMM1_IRQHandler, %function
FLEXCOMM1_IRQHandler:
    ldr   r0,=FLEXCOMM1_DriverIRQHandler
    bx    r0
    .size FLEXCOMM1_IRQHandler, . - FLEXCOMM1_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM2_IRQHandler
    .type FLEXCOMM2_IRQHandler, %function
FLEXCOMM2_IRQHandler:
    ldr   r0,=FLEXCOMM2_DriverIRQHandler
    bx    r0
    .size FLEXCOMM2_IRQHandler, . - FLEXCOMM2_IRQHandler

    .align 1
    .thumb_func
    .weak FLEXCOMM3_IRQHandler
    .type FLEXCOMM3_IRQHandler, %function
FLEXCOMM3_IRQHandler:
    ldr   r0,=FLEXCOMM3_DriverIRQHandler
    bx    r0
    .size FLEXCOMM3_IRQHandler, . - FLEXCOMM3_IRQHandler

    .align 1
    .thumb_func
    .weak BLE_IRQHandler
    .type BLE_IRQHandler, %function
BLE_IRQHandler:
    ldr   r0,=BLE_DriverIRQHandler
    bx    r0
    .size BLE_IRQHandler, . - BLE_IRQHandler

    .align 1
    .thumb_func
    .weak FSP_IRQHandler
    .type FSP_IRQHandler, %function
FSP_IRQHandler:
    ldr   r0,=FSP_DriverIRQHandler
    bx    r0
    .size FSP_IRQHandler, . - FSP_IRQHandler

    .align 1
    .thumb_func
    .weak QDEC0_IRQHandler
    .type QDEC0_IRQHandler, %function
QDEC0_IRQHandler:
    ldr   r0,=QDEC0_DriverIRQHandler
    bx    r0
    .size QDEC0_IRQHandler, . - QDEC0_IRQHandler

    .align 1
    .thumb_func
    .weak QDEC1_IRQHandler
    .type QDEC1_IRQHandler, %function
QDEC1_IRQHandler:
    ldr   r0,=QDEC1_DriverIRQHandler
    bx    r0
    .size QDEC1_IRQHandler, . - QDEC1_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER0_IRQHandler
    .type CTIMER0_IRQHandler, %function
CTIMER0_IRQHandler:
    ldr   r0,=CTIMER0_DriverIRQHandler
    bx    r0
    .size CTIMER0_IRQHandler, . - CTIMER0_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER1_IRQHandler
    .type CTIMER1_IRQHandler, %function
CTIMER1_IRQHandler:
    ldr   r0,=CTIMER1_DriverIRQHandler
    bx    r0
    .size CTIMER1_IRQHandler, . - CTIMER1_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER2_IRQHandler
    .type CTIMER2_IRQHandler, %function
CTIMER2_IRQHandler:
    ldr   r0,=CTIMER2_DriverIRQHandler
    bx    r0
    .size CTIMER2_IRQHandler, . - CTIMER2_IRQHandler

    .align 1
    .thumb_func
    .weak CTIMER3_IRQHandler
    .type CTIMER3_IRQHandler, %function
CTIMER3_IRQHandler:
    ldr   r0,=CTIMER3_DriverIRQHandler
    bx    r0
    .size CTIMER3_IRQHandler, . - CTIMER3_IRQHandler

    .align 1
    .thumb_func
    .weak WDT_IRQHandler
    .type WDT_IRQHandler, %function
WDT_IRQHandler:
    ldr   r0,=WDT_DriverIRQHandler
    bx    r0
    .size WDT_IRQHandler, . - WDT_IRQHandler

    .align 1
    .thumb_func
    .weak ADC_IRQHandler
    .type ADC_IRQHandler, %function
ADC_IRQHandler:
    ldr   r0,=ADC_DriverIRQHandler
    bx    r0
    .size ADC_IRQHandler, . - ADC_IRQHandler

    .align 1
    .thumb_func
    .weak DAC_IRQHandler
    .type DAC_IRQHandler, %function
DAC_IRQHandler:
    ldr   r0,=DAC_DriverIRQHandler
    bx    r0
    .size DAC_IRQHandler, . - DAC_IRQHandler

    .align 1
    .thumb_func
    .weak XTAL_READY_IRQHandler
    .type XTAL_READY_IRQHandler, %function
XTAL_READY_IRQHandler:
    ldr   r0,=XTAL_READY_DriverIRQHandler
    bx    r0
    .size XTAL_READY_IRQHandler, . - XTAL_READY_IRQHandler

    .align 1
    .thumb_func
    .weak FLASH_IRQHandler
    .type FLASH_IRQHandler, %function
FLASH_IRQHandler:
    ldr   r0,=FLASH_DriverIRQHandler
    bx    r0
    .size FLASH_IRQHandler, . - FLASH_IRQHandler

    .align 1
    .thumb_func
    .weak SPIFI0_IRQHandler
    .type SPIFI0_IRQHandler, %function
SPIFI0_IRQHandler:
    ldr   r0,=SPIFI0_DriverIRQHandler
    bx    r0
    .size SPIFI0_IRQHandler, . - SPIFI0_IRQHandler

    .align 1
    .thumb_func
    .weak SCT0_IRQHandler
    .type SCT0_IRQHandler, %function
SCT0_IRQHandler:
    ldr   r0,=SCT0_DriverIRQHandler
    bx    r0
    .size SCT0_IRQHandler, . - SCT0_IRQHandler

    .align 1
    .thumb_func
    .weak RTC_CAP_IRQHandler
    .type RTC_CAP_IRQHandler, %function
RTC_CAP_IRQHandler:
    ldr   r0,=RTC_CAP_DriverIRQHandler
    bx    r0
    .size RTC_CAP_IRQHandler, . - RTC_CAP_IRQHandler

    .align 1
    .thumb_func
    .weak RNG_IRQHandler
    .type RNG_IRQHandler, %function
RNG_IRQHandler:
    ldr   r0,=RNG_DriverIRQHandler
    bx    r0
    .size RNG_IRQHandler, . - RNG_IRQHandler

    .align 1
    .thumb_func
    .weak CALIB_IRQHandler
    .type CALIB_IRQHandler, %function
CALIB_IRQHandler:
    ldr   r0,=CALIB_DriverIRQHandler
    bx    r0
    .size CALIB_IRQHandler, . - CALIB_IRQHandler

    .align 1
    .thumb_func
    .weak BOD_IRQHandler
    .type BOD_IRQHandler, %function
BOD_IRQHandler:
    ldr   r0,=BOD_DriverIRQHandler
    bx    r0
    .size BOD_IRQHandler, . - BOD_IRQHandler


/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    DebugMon_Handler
	def_irq_handler    EXT_GPIO_WAKEUP_DriverIRQHandler
    def_irq_handler    OSC_DriverIRQHandler
    def_irq_handler    ACMP0_DriverIRQHandler
    def_irq_handler    ACMP1_DriverIRQHandler
    def_irq_handler    RTC_SEC_DriverIRQHandler
    def_irq_handler    RTC_FR_DriverIRQHandler
    def_irq_handler    CS_WAKEUP_DriverIRQHandler
    def_irq_handler    CS_DriverIRQHandler
    def_irq_handler    GPIOA_DriverIRQHandler
    def_irq_handler    GPIOB_DriverIRQHandler
    def_irq_handler    DMA0_DriverIRQHandler
    def_irq_handler    PIN_INT0_DriverIRQHandler
    def_irq_handler    PIN_INT1_DriverIRQHandler
    def_irq_handler    PIN_INT2_DriverIRQHandler
    def_irq_handler    PIN_INT3_DriverIRQHandler
    def_irq_handler    OSC_INT_LOW_DriverIRQHandler
    def_irq_handler    USB0_DriverIRQHandler
    def_irq_handler    FLEXCOMM0_DriverIRQHandler
    def_irq_handler    FLEXCOMM1_DriverIRQHandler
    def_irq_handler    FLEXCOMM2_DriverIRQHandler
    def_irq_handler    FLEXCOMM3_DriverIRQHandler
    def_irq_handler    BLE_DriverIRQHandler
    def_irq_handler    FSP_DriverIRQHandler
    def_irq_handler    QDEC0_DriverIRQHandler
    def_irq_handler    QDEC1_DriverIRQHandler
    def_irq_handler    CTIMER0_DriverIRQHandler
    def_irq_handler    CTIMER1_DriverIRQHandler
    def_irq_handler    CTIMER2_DriverIRQHandler
    def_irq_handler    CTIMER3_DriverIRQHandler
    def_irq_handler    WDT_DriverIRQHandler
    def_irq_handler    ADC_DriverIRQHandler
    def_irq_handler    DAC_DriverIRQHandler
    def_irq_handler    XTAL_READY_DriverIRQHandler
    def_irq_handler    FLASH_DriverIRQHandler
    def_irq_handler    SPIFI0_DriverIRQHandler
    def_irq_handler    SCT0_DriverIRQHandler
    def_irq_handler    RTC_CAP_DriverIRQHandler
    def_irq_handler    RNG_DriverIRQHandler
    def_irq_handler    CALIB_DriverIRQHandler
    def_irq_handler    BOD_DriverIRQHandler

    .end
