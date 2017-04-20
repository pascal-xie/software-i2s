//*****************************************************************************
// QN908XC startup code for use with MCUXpresso IDE
//
// Version : 030317
//*****************************************************************************
//
// Copyright(C) NXP Semiconductors, 2017
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// o Redistributions of source code must retain the above copyright notice, this list
//   of conditions and the following disclaimer.
//
// o Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// o Neither the name of copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from this
//   software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*****************************************************************************

#if defined (DEBUG)
#pragma GCC push_options
#pragma GCC optimize ("Og")
#endif // (DEBUG)

#if defined (__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
extern "C" {
    extern void __libc_init_array(void);
}
#endif
#endif

#define WEAK __attribute__ ((weak))
#define WEAK_AV __attribute__ ((weak, section(".after_vectors")))
#define ALIAS(f) __attribute__ ((weak, alias (#f)))

//*****************************************************************************
#if defined (__cplusplus)
extern "C" {
#endif

//*****************************************************************************
// Variable to store CRP value in. Will be placed automatically
// by the linker when "Enable Code Read Protect" selected.
// See crp.h header for more information
//*****************************************************************************
#include <NXP/crp.h>
__CRP const unsigned int CRP_WORD = CRP_NO_CRP ;

//*****************************************************************************
// Declaration of external SystemInit function
//*****************************************************************************
#if defined (__USE_CMSIS)
extern void SystemInit(void);
#endif // (__USE_CMSIS)

//*****************************************************************************
// Forward declaration of the core exception handlers.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//*****************************************************************************
     void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void MemManage_Handler(void);
WEAK void BusFault_Handler(void);
WEAK void UsageFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void DebugMon_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);

//*****************************************************************************
// Forward declaration of the application IRQ handlers. When the application
// defines a handler (with the same name), this will automatically take
// precedence over weak definitions below
//*****************************************************************************
WEAK void EXT_GPIO_WAKEUP_IRQHandler(void);
WEAK void OSC_IRQHandler(void);
WEAK void ACMP0_IRQHandler(void);
WEAK void ACMP1_IRQHandler(void);
WEAK void Reserved20_IRQHandler(void);
WEAK void RTC_SEC_IRQHandler(void);
WEAK void RTC_FR_IRQHandler(void);
WEAK void CS_WAKEUP_IRQHandler(void);
WEAK void CS_IRQHandler(void);
WEAK void GPIOA_IRQHandler(void);
WEAK void GPIOB_IRQHandler(void);
WEAK void DMA0_IRQHandler(void);
WEAK void PIN_INT0_IRQHandler(void);
WEAK void PIN_INT1_IRQHandler(void);
WEAK void PIN_INT2_IRQHandler(void);
WEAK void PIN_INT3_IRQHandler(void);
WEAK void OSC_LOW_IRQHandler(void);
WEAK void USB0_IRQHandler(void);
WEAK void FLEXCOMM0_IRQHandler(void);
WEAK void FLEXCOMM1_IRQHandler(void);
WEAK void FLEXCOMM2_IRQHandler(void);
WEAK void FLEXCOMM3_IRQHandler(void);
WEAK void BLE_IRQHandler(void);
WEAK void FSP_IRQHandler(void);
WEAK void QDEC0_IRQHandler(void);
WEAK void QDEC1_IRQHandler(void);
WEAK void CTIMER0_IRQHandler(void);
WEAK void CTIMER1_IRQHandler(void);
WEAK void CTIMER2_IRQHandler(void);
WEAK void CTIMER3_IRQHandler(void);
WEAK void WDT_IRQHandler(void);
WEAK void ADC_IRQHandler(void);
WEAK void DAC_IRQHandler(void);
WEAK void XTAL_READY_IRQHandler(void);
WEAK void FLASH_IRQHandler(void);
WEAK void SPIFI0_IRQHandler(void);
WEAK void SCT0_IRQHandler(void);
WEAK void Reserved53_IRQHandler(void);
WEAK void RNG_IRQHandler(void);
WEAK void Reserved55_IRQHandler(void);
WEAK void CALIB_IRQHandler(void);
WEAK void Reserved57_IRQHandler(void);
WEAK void Reserved58_IRQHandler(void);
WEAK void Reserved59_IRQHandler(void);
WEAK void Reserved60_IRQHandler(void);
WEAK void Reserved61_IRQHandler(void);
WEAK void Reserved62_IRQHandler(void);
WEAK void Reserved63_IRQHandler(void);
WEAK void Reserved64_IRQHandler(void);
WEAK void Reserved65_IRQHandler(void);
WEAK void Reserved66_IRQHandler(void);
WEAK void BOD_IRQHandler(void);

//*****************************************************************************
// Forward declaration of the driver IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the driver
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//*****************************************************************************
void EXT_GPIO_WAKEUP_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void OSC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ACMP0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ACMP1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved20_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_SEC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RTC_FR_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CS_WAKEUP_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CS_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIOA_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void GPIOB_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DMA0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void PIN_INT3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void OSC_LOW_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void USB0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXCOMM0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXCOMM1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXCOMM2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLEXCOMM3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void BLE_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FSP_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDEC0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void QDEC1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER1_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER2_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CTIMER3_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void DAC_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void XTAL_READY_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void FLASH_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SPIFI0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void SCT0_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved53_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void RNG_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved55_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void CALIB_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved57_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved58_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved59_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved60_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved61_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved62_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved63_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved64_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved65_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void Reserved66_DriverIRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_DriverIRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//*****************************************************************************
#if defined (__REDLIB__)
extern void __main(void);
#endif
extern int main(void);

//*****************************************************************************
// External declaration for the pointer to the stack top from the Linker Script
//*****************************************************************************
extern void _vStackTop(void);

//*****************************************************************************
#if defined (__cplusplus)
} // extern "C"
#endif

//*****************************************************************************
// The vector table.
// This relies on the linker script to place at correct location in memory.
//*****************************************************************************
extern void (* const g_pfnVectors[])(void);
extern void * __Vectors __attribute__ ((alias ("g_pfnVectors")));

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core Level - CM4
    &_vStackTop,                       // The initial stack pointer
    ResetISR,                          // The reset handler
    NMI_Handler,                       // The NMI handler
    HardFault_Handler,                 // The hard fault handler
    MemManage_Handler,                 // The MPU fault handler
    BusFault_Handler,                  // The bus fault handler
    UsageFault_Handler,                // The usage fault handler
    0xFFFFFFFF,                        // Checksum of the first 7 words
    0x000AA8FF,                        // Enable all the feature
    0,                                 // Enhanced image marker, 0x0, 0x0FFEB6B6    
    0xFFFFFFFF,                        // Pointer to image header
    SVC_Handler,                       // SVCall handler
    DebugMon_Handler,                  // Debug monitor handler
    0,                                 // Reserved
    PendSV_Handler,                    // The PendSV handler
    SysTick_Handler,                   // The SysTick handler

    // Chip Level - QN908XC
    EXT_GPIO_WAKEUP_IRQHandler,  // 16: Ext GPIO wakeup
    OSC_IRQHandler,              // 17: BLE wakeup
    ACMP0_IRQHandler,            // 18: Analog comparator0
    ACMP1_IRQHandler,            // 19: Analog comparator1
    Reserved20_IRQHandler,       // 20: Reserved interrupt
    RTC_SEC_IRQHandler,          // 21: RTC second
    RTC_FR_IRQHandler,           // 22: RTC free running
    CS_WAKEUP_IRQHandler,        // 23: Capacitive sense wakeup
    CS_IRQHandler,               // 24: Capacitive sense
    GPIOA_IRQHandler,            // 25: GPIO group A
    GPIOB_IRQHandler,            // 26: GPIO group B
    DMA0_IRQHandler,             // 27: DMA controller
    PIN_INT0_IRQHandler,         // 28: pin or pattern match engine slice 0
    PIN_INT1_IRQHandler,         // 29: pin or pattern match engine slice 1
    PIN_INT2_IRQHandler,         // 30: pin or pattern match engine slice 2
    PIN_INT3_IRQHandler,         // 31: pin or pattern match engine slice 3
    OSC_LOW_IRQHandler,          // 32: Inverse of OSC
    USB0_IRQHandler,             // 33: USB device
    FLEXCOMM0_IRQHandler,        // 34: Flexcomm Interface 0 (USART)
    FLEXCOMM1_IRQHandler,        // 35: Flexcomm Interface 1 (USART, I2C)
    FLEXCOMM2_IRQHandler,        // 36: Flexcomm Interface 2 (SPI, I2C)
    FLEXCOMM3_IRQHandler,        // 37: Flexcomm Interface 3 (SPI)
    BLE_IRQHandler,              // 38: BLE interrupts
    FSP_IRQHandler,              // 39: FSP
    QDEC0_IRQHandler,            // 40: QDEC0
    QDEC1_IRQHandler,            // 41: QDEC1
    CTIMER0_IRQHandler,          // 42: Standard counter/timer CTIMER0
    CTIMER1_IRQHandler,          // 43: Standard counter/timer CTIMER1
    CTIMER2_IRQHandler,          // 44: Standard counter/timer CTIMER2
    CTIMER3_IRQHandler,          // 45: Standard counter/timer CTIMER3
    WDT_IRQHandler,              // 46: Watch dog timer
    ADC_IRQHandler,              // 47: ADC
    DAC_IRQHandler,              // 48: DAC
    XTAL_READY_IRQHandler,       // 49: High frequency crystal ready
    FLASH_IRQHandler,            // 50: Flash
    SPIFI0_IRQHandler,           // 51: SPI flash interface
    SCT0_IRQHandler,             // 52: SCTimer/PWM
    Reserved53_IRQHandler,       // 53: Reserved interrupt
    RNG_IRQHandler,              // 54: Random number generator
    Reserved55_IRQHandler,       // 55: Reserved interrupt
    CALIB_IRQHandler,            // 56: Calibration
    Reserved57_IRQHandler,       // 57: Reserved interrupt
    Reserved58_IRQHandler,       // 58: Reserved interrupt
    Reserved59_IRQHandler,       // 59: Reserved interrupt
    Reserved60_IRQHandler,       // 60: Reserved interrupt
    Reserved61_IRQHandler,       // 61: Reserved interrupt
    Reserved62_IRQHandler,       // 62: Reserved interrupt
    Reserved63_IRQHandler,       // 63: Reserved interrupt
    Reserved64_IRQHandler,       // 64: Reserved interrupt
    Reserved65_IRQHandler,       // 65: Reserved interrupt
    Reserved66_IRQHandler,       // 66: Reserved interrupt
    BOD_IRQHandler,              // 67: Brown out dectect
}; /* End of g_pfnVectors */

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__ ((section(".after_vectors.init_data")))
void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int *pulSrc = (unsigned int*) romstart;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = *pulSrc++;
}

__attribute__ ((section(".after_vectors.init_bss")))
void bss_init(unsigned int start, unsigned int len) {
	unsigned int *pulDest = (unsigned int*) start;
	unsigned int loop;
	for (loop = 0; loop < len; loop = loop + 4)
		*pulDest++ = 0;
}

//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__ ((section(".after_vectors.reset")))
void ResetISR(void) {

    // Disable interrupts
    __asm volatile ("cpsid i");

#if defined (__USE_CMSIS)
// If __USE_CMSIS defined, then call CMSIS SystemInit code
    SystemInit();
#endif // (__USE_CMSIS)

    //
    // Copy the data sections from flash to SRAM.
    //
	unsigned int LoadAddr, ExeAddr, SectionLen;
	unsigned int *SectionTableAddr;

	// Load base address of Global Section Table
	SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
	while (SectionTableAddr < &__data_section_table_end) {
		LoadAddr = *SectionTableAddr++;
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		data_init(LoadAddr, ExeAddr, SectionLen);
	}

	// At this point, SectionTableAddr = &__bss_section_table;
	// Zero fill the bss segment
	while (SectionTableAddr < &__bss_section_table_end) {
		ExeAddr = *SectionTableAddr++;
		SectionLen = *SectionTableAddr++;
		bss_init(ExeAddr, SectionLen);
	}

#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will enable the FPU
#if defined (__VFP_FP__) && !defined (__SOFTFP__)
    //
    // Code to enable the Cortex-M4 FPU only included
    // if appropriate build options have been selected.
    // Code taken from Section 7.1, Cortex-M4 TRM (DDI0439C)
    //
    // Read CPACR (located at address 0xE000ED88)
    // Set bits 20-23 to enable CP10 and CP11 coprocessors
    // Write back the modified value to the CPACR
	asm volatile ("LDR.W R0, =0xE000ED88\n\t"
                  "LDR R1, [R0]\n\t"
                  "ORR R1, R1, #(0xF << 20)\n\t"
                  "STR R1, [R0]");
#endif // (__VFP_FP__) && !(__SOFTFP__)
#endif // (__USE_CMSIS)

#if !defined (__USE_CMSIS)
// Assume that if __USE_CMSIS defined, then CMSIS SystemInit code
// will setup the VTOR register

    // Check to see if we are running the code from a non-zero
    // address (eg RAM, external flash), in which case we need
    // to modify the VTOR register to tell the CPU that the
    // vector table is located at a non-0x0 address.
    unsigned int * pSCB_VTOR = (unsigned int *) 0xE000ED08;
    if ((unsigned int *)g_pfnVectors!=(unsigned int *) 0x00000000) {
        *pSCB_VTOR = (unsigned int)g_pfnVectors;
    }
#endif // (__USE_CMSIS)

#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
#endif

    // Reenable interrupts
    __asm volatile ("cpsie i");

#if defined (__REDLIB__)
	// Call the Redlib library, which in turn calls main()
	__main();
#else
	main();
#endif

	//
	// main() shouldn't return, but if it does, we'll just enter an infinite loop
	//
	while (1) {
		;
	}
}

//*****************************************************************************
// Default core exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
WEAK_AV void NMI_Handler(void)
{ while(1) {}
}

WEAK_AV void HardFault_Handler(void)
{ while(1) {}
}

WEAK_AV void MemManage_Handler(void)
{ while(1) {}
}

WEAK_AV void BusFault_Handler(void)
{ while(1) {}
}

WEAK_AV void UsageFault_Handler(void)
{ while(1) {}
}

WEAK_AV void SVC_Handler(void)
{ while(1) {}
}

WEAK_AV void DebugMon_Handler(void)
{ while(1) {}
}

WEAK_AV void PendSV_Handler(void)
{ while(1) {}
}

WEAK_AV void SysTick_Handler(void)
{ while(1) {}
}

//*****************************************************************************
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//*****************************************************************************
WEAK_AV void IntDefaultHandler(void)
{ while(1) {}
}

//*****************************************************************************
// Default application exception handlers. Override the ones here by defining
// your own handler routines in your application code. These routines call
// driver exception handlers or IntDefaultHandler() if no driver exception
// handler is included.
//*****************************************************************************
WEAK void EXT_GPIO_WAKEUP_IRQHandler(void)
{   EXT_GPIO_WAKEUP_DriverIRQHandler();
}

WEAK void OSC_IRQHandler(void)
{   OSC_DriverIRQHandler();
}

WEAK void ACMP0_IRQHandler(void)
{   ACMP0_DriverIRQHandler();
}

WEAK void ACMP1_IRQHandler(void)
{   ACMP1_DriverIRQHandler();
}

WEAK void Reserved20_IRQHandler(void)
{   Reserved20_DriverIRQHandler();
}

WEAK void RTC_SEC_IRQHandler(void)
{   RTC_SEC_DriverIRQHandler();
}

WEAK void RTC_FR_IRQHandler(void)
{   RTC_FR_DriverIRQHandler();
}

WEAK void CS_WAKEUP_IRQHandler(void)
{   CS_WAKEUP_DriverIRQHandler();
}

WEAK void CS_IRQHandler(void)
{   CS_DriverIRQHandler();
}

WEAK void GPIOA_IRQHandler(void)
{   GPIOA_DriverIRQHandler();
}

WEAK void GPIOB_IRQHandler(void)
{   GPIOB_DriverIRQHandler();
}

WEAK void DMA0_IRQHandler(void)
{   DMA0_DriverIRQHandler();
}

WEAK void PIN_INT0_IRQHandler(void)
{   PIN_INT0_DriverIRQHandler();
}

WEAK void PIN_INT1_IRQHandler(void)
{   PIN_INT1_DriverIRQHandler();
}

WEAK void PIN_INT2_IRQHandler(void)
{   PIN_INT2_DriverIRQHandler();
}

WEAK void PIN_INT3_IRQHandler(void)
{   PIN_INT3_DriverIRQHandler();
}

WEAK void OSC_LOW_IRQHandler(void)
{   OSC_LOW_DriverIRQHandler();
}

WEAK void USB0_IRQHandler(void)
{   USB0_DriverIRQHandler();
}

WEAK void FLEXCOMM0_IRQHandler(void)
{   FLEXCOMM0_DriverIRQHandler();
}

WEAK void FLEXCOMM1_IRQHandler(void)
{   FLEXCOMM1_DriverIRQHandler();
}

WEAK void FLEXCOMM2_IRQHandler(void)
{   FLEXCOMM2_DriverIRQHandler();
}

WEAK void FLEXCOMM3_IRQHandler(void)
{   FLEXCOMM3_DriverIRQHandler();
}

WEAK void BLE_IRQHandler(void)
{   BLE_DriverIRQHandler();
}

WEAK void FSP_IRQHandler(void)
{   FSP_DriverIRQHandler();
}

WEAK void QDEC0_IRQHandler(void)
{   QDEC0_DriverIRQHandler();
}

WEAK void QDEC1_IRQHandler(void)
{   QDEC1_DriverIRQHandler();
}

WEAK void CTIMER0_IRQHandler(void)
{   CTIMER0_DriverIRQHandler();
}

WEAK void CTIMER1_IRQHandler(void)
{   CTIMER1_DriverIRQHandler();
}

WEAK void CTIMER2_IRQHandler(void)
{   CTIMER2_DriverIRQHandler();
}

WEAK void CTIMER3_IRQHandler(void)
{   CTIMER3_DriverIRQHandler();
}

WEAK void WDT_IRQHandler(void)
{   WDT_DriverIRQHandler();
}

WEAK void ADC_IRQHandler(void)
{   ADC_DriverIRQHandler();
}

WEAK void DAC_IRQHandler(void)
{   DAC_DriverIRQHandler();
}

WEAK void XTAL_READY_IRQHandler(void)
{   XTAL_READY_DriverIRQHandler();
}

WEAK void FLASH_IRQHandler(void)
{   FLASH_DriverIRQHandler();
}

WEAK void SPIFI0_IRQHandler(void)
{   SPIFI0_DriverIRQHandler();
}

WEAK void SCT0_IRQHandler(void)
{   SCT0_DriverIRQHandler();
}

WEAK void Reserved53_IRQHandler(void)
{   Reserved53_DriverIRQHandler();
}

WEAK void RNG_IRQHandler(void)
{   RNG_DriverIRQHandler();
}

WEAK void Reserved55_IRQHandler(void)
{   Reserved55_DriverIRQHandler();
}

WEAK void CALIB_IRQHandler(void)
{   CALIB_DriverIRQHandler();
}

WEAK void Reserved57_IRQHandler(void)
{   Reserved57_DriverIRQHandler();
}

WEAK void Reserved58_IRQHandler(void)
{   Reserved58_DriverIRQHandler();
}

WEAK void Reserved59_IRQHandler(void)
{   Reserved59_DriverIRQHandler();
}

WEAK void Reserved60_IRQHandler(void)
{   Reserved60_DriverIRQHandler();
}

WEAK void Reserved61_IRQHandler(void)
{   Reserved61_DriverIRQHandler();
}

WEAK void Reserved62_IRQHandler(void)
{   Reserved62_DriverIRQHandler();
}

WEAK void Reserved63_IRQHandler(void)
{   Reserved63_DriverIRQHandler();
}

WEAK void Reserved64_IRQHandler(void)
{   Reserved64_DriverIRQHandler();
}

WEAK void Reserved65_IRQHandler(void)
{   Reserved65_DriverIRQHandler();
}

WEAK void Reserved66_IRQHandler(void)
{   Reserved66_DriverIRQHandler();
}

WEAK void BOD_IRQHandler(void)
{   BOD_DriverIRQHandler();
}

//*****************************************************************************

#if defined (DEBUG)
#pragma GCC pop_options
#endif // (DEBUG)
