#include "mcu.h"

extern const struct pmc_driver_interface pmc_driver;
extern const struct pio_driver_interface pio_driver;
extern const struct adc_driver_interface adc_driver;

static void system_init(void);

int main()
{
    system_init();

    // Enable peripheral clocks
    pio_driver.clock_init();
    adc_driver.clock_init();

    // Configure peripherals
    adc_driver.init();
    pio_driver.init();

    // Start peripheral operation
    adc_driver.enable();


    while(1)
    {
    }
}

static void system_init(void)
{
    // FPU enable
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/Atmel-44047-Cortex-M7-Microcontroller-Optimize-Usage-SAM-V71-V70-E70-S70-Architecture_Application-note.pdf
    fpu_enable();

    // Configure the wait states of the Embedded Flash Controller
    // Rerence uses 5 instead of 6 but following Atmel Start example
    EFC->EEFC_FMR =  EEFC_FMR_FWS(6);

    // Initialize system clocks
    pmc_driver.init();

    // Disable watchdog
    WDT->WDT_MR |= WDT_MR_WDDIS;
}
