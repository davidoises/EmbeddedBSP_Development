#include <stdio.h>
#include "mcu.h"

extern const struct pmc_driver_interface pmc_driver;
extern const struct pio_driver_interface pio_driver;
extern const struct adc_driver_interface adc_driver;
extern const struct uart_driver_interface uart_driver;

static void system_init(void);
static void delay(int n);

int main()
{
    system_init();

    // Enable peripheral clocks
    pio_driver.clock_init();
    adc_driver.clock_init();
    uart_driver.clock_init();

    // Configure peripherals
    adc_driver.init();
    uart_driver.init();
    pio_driver.init();

    // Start peripheral operation
    adc_driver.enable();
    uart_driver.enable();


    while(1)
    {
        uint16_t adc_reding = adc_driver.read();

        char output_msg[100];
		uint16_t count = sprintf(output_msg, "The ADC measurement is: %d\r\n", adc_reding);
        uart_driver.write((uint8_t *)output_msg, count);

        delay(1000);
    }
}

static void system_init(void)
{
#if defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
    // FPU enable
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/Atmel-44047-Cortex-M7-Microcontroller-Optimize-Usage-SAM-V71-V70-E70-S70-Architecture_Application-note.pdf
    fpu_enable();

    // Configure the wait states of the Embedded Flash Controller
    // Rerence uses 5 instead of 6 but following Atmel Start example
    EFC->EEFC_FMR =  EEFC_FMR_FWS(6);
#endif

    // Initialize system clocks
    pmc_driver.init();

#if defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
    // Disable watchdog
    WDT->WDT_MR |= WDT_MR_WDDIS;
#endif
}

static void delay(int n)
{
    int i;

    for (;n >0; n--)
    {
        for (i=0;i<1000;i++)
            __asm("nop");
    }
}
