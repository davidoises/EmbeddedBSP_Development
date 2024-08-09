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
    UART0->UART_BRGR = UART_BRGR_CD(CONF_UART_0_BAUD_CD);
}

static void uart_driver_clock_init(void)
{
    pmc_driver.enable_peripheral_clock(ID_UART0);
}

static void uart_driver_enable(void)
{
    // Enable the receiver and transmiter
    UART0->UART_CR = (UART_CR_RXEN | UART_CR_TXEN);
}

static uint32_t uart_driver_write(const uint8_t *buffer, const uint16_t length)
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

static uint32_t uart_driver_read(uint8_t *buffer, const uint16_t length)
{
    // TODO
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
