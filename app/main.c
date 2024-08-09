#include "mcu.h"
#include <stdio.h>


static void system_init(void);

static void pmc_init(void);
static void pmc_enable_peripheral_clock(uint32_t pid);

static void pio_clock_init(uint16_t *pio_pids, uint16_t pio_count);
static void pio_init(void);

static void usart_clock_init(void);
static void uart_init(void);
static void uart_enable(void);
static uint32_t usart_write(const uint8_t *buffer, const uint16_t length);
static uint32_t usart_read(uint8_t *buffer, const uint16_t length);

static void adc_clock_init(void);
static void adc_init(void);
static void adc_enable(void);
static uint16_t adc_read(void);


static void delay(int n);

uint16_t pio_pids[] = {ID_PIOB, ID_PIOC};

int main()
{

    // General system initialization and main clocks
    system_init();

    // Enable peripheral clocks
    pio_clock_init(pio_pids, sizeof(pio_pids)/sizeof(pio_pids[0]));
    usart_clock_init();
    adc_clock_init();

    // Configure peripherals
    uart_init();
    adc_init();

    pio_init();

    // Start peripheral operation
    uart_enable();
    adc_enable();

    while(1)
    {

        uint16_t adc_reding = adc_read();
        // uint16_t adc_reding = 5;

        char output_msg[100];
		uint16_t count = sprintf(output_msg, "The ADC measurement is: %d\r\n", adc_reding);
        usart_write((uint8_t *)output_msg, count);

        // PORT->Group[0].OUT.reg &= ~PORT_PA27;
        // PORT->Group[1].OUT.reg &= ~PORT_PB03;
        delay(200);
        // PORT->Group[0].OUT.reg |= PORT_PA27;
        // PORT->Group[1].OUT.reg |= PORT_PB03;
        delay(100);
        // PORT->Group[0].OUT.reg &= ~PORT_PA27;
        // PORT->Group[1].OUT.reg &= ~PORT_PB03;
        delay(200);
        // PORT->Group[0].OUT.reg |= PORT_PA27;
        // PORT->Group[1].OUT.reg |= PORT_PB03;
        delay(1000);
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

static void pmc_enable_peripheral_clock(uint32_t pid)
{
    // Refer to section 31.20.4 for PCER0 as an example
    // PCER0, PCSR0, PCDR0 have the same bit assignments
    // PCER0, PCSR0, PCDR0 control clocks for perpheral IDs [7,31]

    // Refer to section 31.20.23 for PCER1 as an example
    // PCER1, PCSR1, PCDR1 have the same bit assignments
    // PCER1, PCSR1, PCDR1 control clocks for perpheral IDs [32,62]

    if (pid < 32)
    {
        PMC->PMC_PCER0 = (1 << pid);
    }
    else if (pid < 64)
    {
        uint32_t relative_pid = pid -32;
        PMC->PMC_PCER1 = (1 << relative_pid);
    }

    // I2S peripherals need to follow the next steps but wont be implemented at this time
    // In theory the previous part can also be achieved through the regiser PMC_PCR
    // 1. Set the preripheral ID number
    // 2. Change the CLKDIV or GCLKSS if needed. By default GCLKCSS is SLOW_CLK
    // 3. Then set CMD = 1 (write) and EN = 1
}

static void pio_clock_init(uint16_t *pio_pids, uint16_t pio_count)
{
    for (uint16_t i = 0; i < pio_count; i++)
    {
        pmc_enable_peripheral_clock(pio_pids[i]);
    }
}

static void pio_init(void)
{
    uint32_t temp;
    // For the following PB port pins set 1 on their corresponding pin to enable the PIO functionality
    // MATRIX->CCFG_SYSIO
    // PB12 or ERASE
    // PB7 or TCK/SWCLK
    // PB6 or TMS/SWDIO
    // PB5 or TDO/TRACESWO
    // PB4 or TDI

    // Enableing and muxing
    // PIO_PER (PIO enable register) - controls if PIO is in control of the IO line, 1 means control by PIO
    // PIO_PDR (PIO disable register)
    // PIO_PSR (PIO status register)
    // PIO_ABCDSR0 -> 0 means A, 1 means B
    // PIO_ABCDSR1 -> 0 means C, 1 means D
    // Section 32.5.3 write to PIO_PDR to disable the PIO control and then write to PIO_ABCDSR0/1 to enable the corresponding muxed function

    // Direction (Output vs Input)
    // PIO_OER (Output enable register) - controls if the io is used as output, 1 means output
    // PIO_ODR (Output disable register)
    // PIO_OSR (Output status register)

    // Pull Up
    // PIO_PUER ( Pull Up Enable register) - controls pull up, 1 means pull up enabled
    // PIO_PUDR ( Pull Up Disable register)
    // PIO_PUSR ( Pull Up Status register)

    // Pull Down
    // PIO_PPDER ( Pull Down Enable register) - controls pull down, 1 means pull down enabled
    // PIO_PPDDR ( Pull Down Disable register)
    // PIO_PPDSR ( Pull Down Status register)

    // Output control
    // PIO_SODR ( Set Output Data register) - set the output value
    // PIO_CODR ( Clear Output Data register) - set the output value
    // PIO_ODSR ( Output status reigster)

    // PIO_PDSR ( Data status register) - reads the value on the line regardless of the configuration

    // PB12 input with pull up
    MATRIX->CCFG_SYSIO |= PIO_PB12;
    PIOB->PIO_ODR |= PIO_PB12; //  Disable output
    PIOB->PIO_PPDDR |= PIO_PB12; // Disable pulldown
    PIOB->PIO_PUER |= PIO_PB12; // Enable Pull up
    PIOB->PIO_PER |= PIO_PB12; // I/O mode

    // PC9 output, start as low output
    PIOC->PIO_CODR |= PIO_PC9; // Clear output
    PIOC->PIO_OER |= PIO_PC9; //  Enable output
    PIOC->PIO_PER |= PIO_PC9; // I/O mode

    // PC10 output, start as low output
    PIOC->PIO_CODR |= PIO_PC10; // Clear output
    PIOC->PIO_OER |= PIO_PC10; //  Enable output
    PIOC->PIO_PER |= PIO_PC10; // I/O mode

    // PA9 UART0 Rx (Peripheral function A)
    temp = PIOA->PIO_ABCDSR[0];
    temp &= ~PIO_PA9;
    PIOA->PIO_ABCDSR[0] = temp;

    temp = PIOA->PIO_ABCDSR[1];
    temp &= ~PIO_PA9;
    PIOA->PIO_ABCDSR[1] = temp;

    PIOA->PIO_PDR |= PIO_PA9; // peripheral mode

    // PA10 UART0 Tx (Peripheral function A)
    temp = PIOA->PIO_ABCDSR[0];
    temp &= ~PIO_PA10;
    PIOA->PIO_ABCDSR[0] = temp;

    temp = PIOA->PIO_ABCDSR[1];
    temp &= ~PIO_PA10;
    PIOA->PIO_ABCDSR[1] = temp;

    PIOA->PIO_PDR |= PIO_PA10; // peripheral mode

    // PB1 AFEC 1 Channel 0 (dont select peripheral function, just set as PIO)
    PIOB->PIO_PER |= PIO_PB1; // I/O mode
}

static void usart_clock_init(void)
{
    pmc_enable_peripheral_clock(ID_UART0);
}

static void uart_init(void)
{
    // Set the write protection key and disable write protection
    UART0->UART_WPMR = UART_WPMR_WPKEY_PASSWD;

    // UART_CR_RSTRX - Put the receiver into reset state
    // UART_CR_RSTTX - Put the transmiter into reset state
    // UART_CR_RXDIS - disable receiver
    // UART_CR_TXDIS - disable transmiter
    UART0->UART_CR = (UART_CR_RSTRX | UART_CR_RXDIS | UART_CR_RSTTX | UART_CR_TXDIS);

    // UART_CR_RSTSTA - Resets PARE, FRAME, CMP, OVRE in the UART_SR register
    UART0->UART_CR = UART_CR_RSTSTA;

    // UART_MR_BRSRCCK_PERIPH_CLK - Baud rate source clock is peripheral clock
    // UART_MR_PAR_NO - no parity
    // UART_MR_CHMODE_NORMAL - no testing mode
    // UART_MR_FILTER_DISABLED - no filter
    UART0->UART_MR = (UART_MR_BRSRCCK_PERIPH_CLK | UART_MR_PAR_NO | UART_MR_CHMODE_NORMAL | UART_MR_FILTER_DISABLED);

    // Configure the UART baud rate
    #define CONF_UART_0_BAUD 115200
    #define CONF_UART0_FREQUENCY 150000000
    #define CONF_UART_0_BAUD_CD ((CONF_UART0_FREQUENCY) / CONF_UART_0_BAUD / 16)
    UART0->UART_BRGR = UART_BRGR_CD(CONF_UART_0_BAUD_CD);
}

static void uart_enable(void)
{
    // Enable the receiver and transmiter
    UART0->UART_CR = (UART_CR_RXEN | UART_CR_TXEN);
}

static uint32_t usart_write(const uint8_t *buffer, const uint16_t length)
{
    // writing UART
    // data transfered to UART_THR
    // UART_SR.TXRDY indicates that the holding register is empty
    // UART_SR.TXEMPTY indicates that the holding register and internal shift register are empty

    uint32_t counter = 0;

    while (!(UART0->UART_SR & UART_SR_TXRDY)){}

    while (counter < length)
    {
        // Write into the transmit hold register
        UART0->UART_THR = UART_THR_TXCHR(buffer[counter]);

        while (!(UART0->UART_SR & UART_SR_TXRDY)){}

        counter++;
    }

    while (!(UART0->UART_SR & UART_SR_TXEMPTY)){}

    return counter;
}

static uint32_t usart_read(uint8_t *buffer, const uint16_t length)
{
    // reading UART
    // data transfered to UART_RHR
    // UART_SR.RXRDY will indicate that the data is ready and it clears on its own after reading
    // Not implemented, for now dummy structure

    uint32_t counter = 0;
    while (counter < length)
    {
        buffer[counter] = 'a' + counter;
        counter++;
    }

    return counter;
}

static void adc_clock_init(void)
{
    pmc_enable_peripheral_clock(ID_AFEC1);
}

static void adc_init(void)
{
    // Recommended transfer period
    // 64 AFE perios for start up time
    // Free run
    // Only software trigger
    AFEC1->AFEC_MR = (uint32_t)((AFEC_MR_TRANSFER(2)) | (AFEC_MR_TRACKTIM(15)) | (AFEC_MR_ONE) | (AFEC_MR_STARTUP_SUT64) | (AFEC_MR_PRESCAL(0x31)) | (AFEC_MR_FREERUN_ON) | (AFEC_MR_TRGSEL_AFEC_TRIG0) | (AFEC_MR_TRGEN_DIS));

    // Single trigger mode
    // Apend channel number to conversion result
    AFEC1->AFEC_EMR = (uint32_t)((AFEC_EMR_STM) | (AFEC_EMR_TAG));

	// Bias current control setting = 1
    // Programmable gain amplifiers 0 and 1 are on
    AFEC1->AFEC_ACR = (uint32_t)(AFEC_ACR_IBCTL(0x1) | (AFEC_ACR_PGA0EN | AFEC_ACR_PGA1EN));

    // Set the offset compensation to 512, mid value of the DAC in the AFEC
    // Only for channel 0 since this is the one we are measuring
    AFEC1->AFEC_CSELR = 0;
    AFEC1->AFEC_COCR = 0x200;
}

static void adc_enable(void)
{
    // Enabling channel 0
    AFEC1->AFEC_CHER = (1 << 0);

    // Trigger the first conversion, afterwards it should run in free run mode
    AFEC1->AFEC_CR = AFEC_CR_START;
}

static uint16_t adc_read(void)
{
    // Check for end of conversion flag for channel 0
    // Could check either EOC0 or DRDY since we are only using one channel
    while(!(AFEC1->AFEC_ISR & (1 << 0))){}

    // reading AFEC_CDR clears the EOC bit
    // reading AFEC_LCDR clears the DRDY bit
    // To read AFEC_CDR first select the channel in AFEC_CSELR
    AFEC1->AFEC_CSELR = 0;
    uint16_t res = AFEC1->AFEC_CDR;

    return res;
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
