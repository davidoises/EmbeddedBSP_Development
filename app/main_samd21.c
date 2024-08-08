
#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
#include "samd21.h"
#elif defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
#include "samdv71.h"
#else
  #error Library does not support the specified device.
#endif

#include <stdio.h>

static void system_init(void);
static void ports_init(void);

static void usart_clock_init(void);
static void usart_init(void);
static void usart_start(void);
static uint32_t usart_write(const uint8_t *buffer, const uint16_t length);

static void adc_wait_syncbusy(void);
static void adc_clock_init(void);
static void adc_init(void);
static void adc_start(void);
static uint16_t adc_read(void);

static void delay(int n);

int main()
{

    system_init();

    usart_start();

    adc_start();

    while(1)
    {

        uint16_t adc_reding = adc_read();

        char output_msg[100];
		uint16_t count = sprintf(output_msg, "The ADC measurement is: %d\r\n", adc_reding);
        usart_write((uint8_t *)output_msg, count);

        PORT->Group[0].OUT.reg &= ~PORT_PA27;
        PORT->Group[1].OUT.reg &= ~PORT_PB03;
        delay(200);
        PORT->Group[0].OUT.reg |= PORT_PA27;
        PORT->Group[1].OUT.reg |= PORT_PB03;
        delay(100);
        PORT->Group[0].OUT.reg &= ~PORT_PA27;
        PORT->Group[1].OUT.reg &= ~PORT_PB03;
        delay(200);
        PORT->Group[0].OUT.reg |= PORT_PA27;
        PORT->Group[1].OUT.reg |= PORT_PB03;
        delay(1000);
    }
}

static void system_init(void)
{
    // Set the clocks, 8MHZ source just change the prescaler to 1 instead of 8
	SYSCTRL->OSC8M.bit.PRESC = 0;

    usart_clock_init();
    adc_clock_init();

    usart_init();
    adc_init();

    ports_init();
}

static void ports_init(void)
{
    // GPIO PA27 as output
    PORT->Group[0].DIR.reg |= PORT_PA27;

    // GPIO PB03 as output
    PORT->Group[1].DIR.reg |= PORT_PB03;

    // ADC
    // PB02 as analog (Arduino connector J2, pin 14)
    // Enabling alternate function
    PORT->Group[1].PINCFG[2].bit.PMUXEN = 1;

    // PMUX[0] -> pins 0 and 1
    // PMUX[1] -> pins 2 and 3
    // B02 is an even pin
    // Peripheral function B = AIN10
    PORT->Group[1].PMUX[1].bit.PMUXE = 0x01;

    // UART
    // PB22 Tx, Peripheral function D = SERCOM5 PAD[2]
    PORT->Group[1].PINCFG[22].bit.PMUXEN = 1;
    PORT->Group[1].PMUX[11].bit.PMUXE = 0x03;

    // PB23 Rx, Peripheral function D = SERCOM5 PAD[3]
    PORT->Group[1].PINCFG[23].bit.PMUXEN = 1;
    PORT->Group[1].PMUX[11].bit.PMUXO = 0x03;
}

static void usart_clock_init(void)
{
    // Enabling the SERCOM5 clock
    PM->APBCMASK.bit.SERCOM5_ = 1;

    // Connecting Generic clock 0 as SERCOM 5 source
    GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
    GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_SERCOM5_CORE_Val;

    // Enable it
    GCLK->CLKCTRL.bit.CLKEN = 1;
}

static void usart_init(void)
{
    // INIT FOR UART FUNCTIONS
    // The following bits are synchronized when written:
    // - Software Reset bit in the CTRLA register (CTRLA.SWRST)
    // - Enable bit in the CTRLA register (CTRLA.ENABLE)
    // - Receiver Enable bit in the CTRLB register (CTRLB.RXEN)
    // - Transmitter Enable bit in the Control B register (CTRLB.TXEN)
    // CTRLB.RXEN is write-synchronized somewhat differently. See also 26.8.2  CTRLB for details.

    if (!SERCOM5->USART.SYNCBUSY.bit.SWRST)
    {
        if (SERCOM5->USART.CTRLA.bit.ENABLE)
        {
            SERCOM5->USART.CTRLA.bit.ENABLE = 0;
            while (SERCOM5->USART.SYNCBUSY.bit.ENABLE){}
        }
        SERCOM5->USART.CTRLA.bit.MODE = SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val;
        SERCOM5->USART.CTRLA.bit.SWRST = 1;
    }
    while (SERCOM5->USART.SYNCBUSY.bit.SWRST){}

    // INTERNAL CLOCK
    SERCOM5->USART.CTRLA.bit.MODE = SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val;
    // SYNCHRONOUS MODE (1)
    SERCOM5->USART.CTRLA.bit.CMODE = 1;
    // ASYNCHRONOUS MODE (0)
    SERCOM5->USART.CTRLA.bit.CMODE = 0;
    // SELECT RX PIN PB23 PAD[3]
    SERCOM5->USART.CTRLA.bit.RXPO = 3;
    // SELECT RX PIN PB22 PAD[2]
    SERCOM5->USART.CTRLA.bit.TXPO = 1;
    // Configure the character size (8 bits)
    SERCOM5->USART.CTRLB.bit.CHSIZE = 0;
    // Configure the order of the data (LSB is first)
    SERCOM5->USART.CTRLA.bit.DORD = 1;
    // Configure the stop bits (1 bits)
    SERCOM5->USART.CTRLB.bit.SBMODE = 0;
    // Configure the baud rate
    // BAUD reg = 65536 *( 1 - 16*(115200/8000000))
    SERCOM5->USART.BAUD.reg = 50436;

    // SINCE USART IS DISABLED THESE REGS ARE IMMEDIATELY WRITTEN
    SERCOM5->USART.CTRLB.bit.TXEN = 1;
    SERCOM5->USART.CTRLB.bit.RXEN = 1;
}

static void usart_start(void)
{
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE){}
}

static uint32_t usart_write(const uint8_t *buffer, const uint16_t length)
{
    uint32_t counter = 0;

    // When DRE is 1 it means the DATA reg is ready for new data
    while (!SERCOM5->USART.INTFLAG.bit.DRE ){}

    while (counter < length)
    {
        // In theory enter critical section
        SERCOM5->USART.DATA.bit.DATA = buffer[counter];
        // In theory exit critical section
        while (!SERCOM5->USART.INTFLAG.bit.DRE ){}
        counter++;
    }

    // INTFLAG.bit.TXC Can be checked to confirm transmission
    while (!SERCOM5->USART.INTFLAG.bit.TXC ){}
    return counter;
}

static void adc_clock_init(void)
{
    // Enabling the SERCOM5 clock
    PM->APBCMASK.bit.ADC_ = 1;

    // Connecting Generic clock 0 as SERCOM 5 source
    GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
    GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_ADC_Val;

    // Enable it
    GCLK->CLKCTRL.bit.CLKEN = 1;
}

static void adc_wait_syncbusy(void)
{
    while (ADC->STATUS.bit.SYNCBUSY){}
}

static void adc_init(void)
{
    // Read calib registers here
    uint16_t calib_reg = ADC_CALIB_BIAS_CAL((*(uint32_t *)ADC_FUSES_BIASCAL_ADDR >> ADC_FUSES_BIASCAL_Pos))
	            | ADC_CALIB_LINEARITY_CAL((*(uint64_t *)ADC_FUSES_LINEARITY_0_ADDR >> ADC_FUSES_LINEARITY_0_Pos));

    adc_wait_syncbusy();
    if (ADC->CTRLA.bit.ENABLE)
    {
        ADC->CTRLA.bit.ENABLE = 0;
        adc_wait_syncbusy();
    }

    ADC->CTRLA.bit.SWRST = 1;
    adc_wait_syncbusy();

    // Load calib registers here
    ADC->CALIB.reg = calib_reg;

    // INPUTCTRL
    ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
    adc_wait_syncbusy();
    ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN10_Val;
    adc_wait_syncbusy();

    // REFCTRL // use VCC/2 as the Aref
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;

    // CTRLB
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV16_Val;
    adc_wait_syncbusy();
    ADC->CTRLB.bit.CORREN = 0; // This seems to be additional correction besides calib
    adc_wait_syncbusy();
    ADC->CTRLB.bit.FREERUN = 1;
    adc_wait_syncbusy();

}

static void adc_start(void)
{
    adc_wait_syncbusy();
    ADC->CTRLA.bit.ENABLE = 1;
    adc_wait_syncbusy();

    ADC->SWTRIG.bit.START = 1;
    adc_wait_syncbusy();
}

static uint16_t adc_read(void)
{
    // wait for RESRDY
    while(!ADC->INTFLAG.bit.RESRDY){}
    adc_wait_syncbusy();
    uint16_t res = ADC->RESULT.reg;
    res &= 0x0FFF;

    return res;
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
