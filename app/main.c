
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
static void pmc_init(void);


static void delay(int n);

int main()
{

    system_init();

    // usart_clock_init();
    // adc_clock_init();

    // usart_init();
    // adc_init();

    // ports_init();

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
    // https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/Atmel-44047-Cortex-M7-Microcontroller-Optimize-Usage-SAM-V71-V70-E70-S70-Architecture_Application-note.pdf
    fpu_enable();

    // Configure the wait states of the Embedded Flash Controller
    // Rerence uses 5 instead of 6 but following Atmel Start example
    EFC->EEFC_FMR =  EEFC_FMR_FWS(6);

    // Initialize system clocks
    pmc_init();

    // Disable watchdog
    WDT->WDT_MR |= WDT_MR_WDDIS;
}

static void pmc_init(void)
{
    // Used to hold values and modify as needed before writing
    uint32_t temp;

    // Section 31.17 datasheet
    // Configure XOSC20M -> MAIN CLOCK
    // Note: CKGR_MOR requires to always have the key PASSWD written to it
    // 2. enable the main cystal oscillator
    temp = PMC->CKGR_MOR;
    // Cleaer the MOSCXTBY bit (standby)
    temp &= ~CKGR_MOR_MOSCXTBY;
    // Enable
    temp |= CKGR_MOR_MOSCXTEN;
    // CONF_XOSC20M_STARTUP_TIME = 62
    temp |= CKGR_MOR_MOSCXTST(62);
    // Add the pasword
    temp |= CKGR_MOR_KEY_PASSWD;
    PMC->CKGR_MOR = temp;
    // Wait for the oscillator to stabilize
    while (!(PMC->PMC_SR && PMC_SR_MOSCXTS)){}

    // 3. Setting MAINCK to use the main crystal oscillator
    PMC->CKGR_MOR |= (CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL);

    // 4. wait to confirm the change is good
    while (!(PMC->PMC_SR && PMC_SR_MOSCSELS)){}

    // Section 31.17 datasheet
    // 6. Configure MAIN CLOCK -> PLLA
    // Note: CKGR_PLLAR requires to always have 1 written to it on bit 29 (CKGR_PLLAR_ONE)
    PMC->CKGR_PLLAR = (CKGR_PLLAR_MULA((25-1)) | CKGR_PLLAR_DIVA(1) | CKGR_PLLAR_PLLACOUNT(63) | CKGR_PLLAR_ONE);
    // Wait for PLLA to stabilize
    while (!(PMC->PMC_SR && PMC_SR_LOCKA)){}

    // Section 31.17 datasheet
    // 7. Configure PLLA -> Mater Clock
    temp = PMC->PMC_MCKR;
    temp &= ~PMC_MCKR_PRES_Msk;
    temp |= PMC_MCKR_PRES(0);
    PMC->PMC_MCKR = temp; // Prescaler of 1
    while (!(PMC->PMC_SR && PMC_SR_MCKRDY)){}

    temp = PMC->PMC_MCKR;
    temp &= ~PMC_MCKR_MDIV_Msk;
    temp |= PMC_MCKR_MDIV(0);
    PMC->PMC_MCKR = temp; // Divider of 1
    while (!(PMC->PMC_SR && PMC_SR_MCKRDY)){}

    temp = PMC->PMC_MCKR;
    temp &= ~PMC_MCKR_CSS_Msk;
    temp |= PMC_MCKR_CSS_PLLA_CLK;
    PMC->PMC_MCKR = temp; // Select the PLLA
    while (!(PMC->PMC_SR && PMC_SR_MCKRDY)){}
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
