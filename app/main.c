#include <stdio.h>
#include "mcu.h"
#include "pio_config.h"

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


    // Due to differences in configuration on test boards, leaving this out from main for samd21 target
#if defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)

    // PB12 as input with pull up
    pio_driver.mode_configuration(GPIO_PORTB, PIO_PB12, GPIO_DIRECTION_IN);
    pio_driver.pull_up_down_configuration(GPIO_PORTB, PIO_PB12, GPIO_PULL_UP);

    // PC9 as ouput (initially low)
    pio_driver.mode_configuration(LED_PORT, LED_PIN, GPIO_DIRECTION_OUT);
    pio_driver.set_io_level(LED_PORT, LED_PIN, false);

    // PC10 as ouput (initially low)
    pio_driver.mode_configuration(GPIO_PORTC, PIO_PC10, GPIO_DIRECTION_OUT);
    pio_driver.set_io_level(GPIO_PORTC, PIO_PC10, false);

    // PA9 UART0 Rx (Peripheral function A)
    pio_driver.mode_configuration(GPIO_PORTA, PIO_PA9, GPIO_PERIPH_A);

    // PA10 UART0 Tx (Peripheral function A)
    pio_driver.mode_configuration(GPIO_PORTA, PIO_PA10, GPIO_PERIPH_A);

    // PB1 AFEC 1 Channel 0
    pio_driver.mode_configuration(GPIO_PORTB, PIO_PB1, GPIO_DIRECTION_OFF);

#endif

    // Start peripheral operation
    adc_driver.enable();
    uart_driver.enable();


    while(1)
    {
        uint16_t adc_reding = adc_driver.read();

        char output_msg[100];
		uint16_t count = sprintf(output_msg, "The ADC value is: %d\r\n", adc_reding);
        uart_driver.write((uint8_t *)output_msg, count);

        pio_driver.set_io_level(LED_PORT, LED_PIN, false);
        delay(200);
        pio_driver.set_io_level(LED_PORT, LED_PIN, true);
        delay(100);
        pio_driver.set_io_level(LED_PORT, LED_PIN, false);
        delay(200);
        pio_driver.set_io_level(LED_PORT, LED_PIN, true);
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
        for (i=0;i<100;i++)
            __asm("nop");
    }
}
