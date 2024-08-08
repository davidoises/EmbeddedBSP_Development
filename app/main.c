
#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
#include "samd21.h"
#elif defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
#include "samv71.h"
#include "fpu.h"
#else
  #error Library does not support the specified device.
#endif

#include <stdio.h>


static void system_init(void);


static void delay(int n);

int main()
{

    system_init();

    while(1)
    {

        // uint16_t adc_reding = adc_read();
        uint16_t adc_reding = 5;

        char output_msg[100];
		uint16_t count = sprintf(output_msg, "The ADC measurement is: %d\r\n", adc_reding);

        // PORT->Group[0].OUT.reg &= ~PORT_PA27;
        // PORT->Group[1].OUT.reg &= ~PORT_PB03;
        // delay(200);
        // PORT->Group[0].OUT.reg |= PORT_PA27;
        // PORT->Group[1].OUT.reg |= PORT_PB03;
        // delay(100);
        // PORT->Group[0].OUT.reg &= ~PORT_PA27;
        // PORT->Group[1].OUT.reg &= ~PORT_PB03;
        // delay(200);
        // PORT->Group[0].OUT.reg |= PORT_PA27;
        // PORT->Group[1].OUT.reg |= PORT_PB03;
        // delay(1000);
    }
}

static void system_init(void)
{
    // FPU enable
    #if defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
    // // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/Atmel-44047-Cortex-M7-Microcontroller-Optimize-Usage-SAM-V71-V70-E70-S70-Architecture_Application-note.pdf
    fpu_enable();
    #endif

    // Configure the wait states of the Embedded Flash Controller
    // CONF_EFC_WAIT_STATE = 6
    // hri_efc_write_EEFC_FMR_FWS_bf(EFC, CONF_EFC_WAIT_STATE);

    // Set the clocks, 8MHZ source just change the prescaler to 1 instead of 8
	// SYSCTRL->OSC8M.bit.PRESC = 0;

    // usart_clock_init();
    // adc_clock_init();

    // usart_init();
    // adc_init();

    // ports_init();
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
