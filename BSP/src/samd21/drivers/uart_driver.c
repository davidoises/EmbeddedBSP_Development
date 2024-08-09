#include "uart_driver.h"

#include "mcu.h"
#include "pmc_driver.h"

#define CONF_UART_0_BAUD 115200
#define CONF_UART0_FREQUENCY 150000000
#define CONF_UART_0_BAUD_CD ((CONF_UART0_FREQUENCY) / CONF_UART_0_BAUD / 16)

static void uart_driver_init(void);
static void uart_driver_clock_init(void);
static void uart_driver_enable(void);
static uint32_t uart_driver_write(const uint8_t *buffer, const uint16_t length);
static uint32_t uart_driver_read(uint8_t *buffer, const uint16_t length);

extern const struct pmc_driver_interface pmc_driver;

extern const struct uart_driver_interface uart_driver;
const struct uart_driver_interface uart_driver = {
    .init = &uart_driver_init,
    .clock_init = &uart_driver_clock_init,
    .enable = &uart_driver_enable,
    .write = &uart_driver_write,
    .read = &uart_driver_read,
};

static void uart_driver_init(void)
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

static void uart_driver_clock_init(void)
{
    pmc_driver.enable_peripheral_clock(PM_APBCMASK_SERCOM5);

    // Connecting Generic clock 0 as SERCOM 5 source
    GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
    GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_SERCOM5_CORE_Val;

    // Enable it
    GCLK->CLKCTRL.bit.CLKEN = 1;
}

static void uart_driver_enable(void)
{
    SERCOM5->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM5->USART.SYNCBUSY.bit.ENABLE){}
}

static uint32_t uart_driver_write(const uint8_t *buffer, const uint16_t length)
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

static uint32_t uart_driver_read(uint8_t *buffer, const uint16_t length)
{
    // TODO
    // reading UART

    uint32_t counter = 0;
    while (counter < length)
    {
        buffer[counter] = 'a' + counter;
        counter++;
    }

    return counter;
}
