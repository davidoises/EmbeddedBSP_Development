#include "pio_driver.h"

#include "mcu.h"
#include "pmc_driver.h"


static void pio_driver_init(void);
static void pio_driver_clock_init(void);
static void mode_configuration(const enum gpio_port port, const uint8_t pin, const enum gpio_direction mode);
static void pull_up_down_configuration(const enum gpio_port port, const uint8_t pin, const enum gpio_pull_mode mode);
static bool get_io_level(const enum gpio_port port, const uint8_t pin);
static void set_io_level(const enum gpio_port port, const uint8_t pin, const bool level);

extern const struct pmc_driver_interface pmc_driver;

extern const struct pio_driver_interface pio_driver;
const struct pio_driver_interface pio_driver = {
    .init = &pio_driver_init,
    .clock_init = &pio_driver_clock_init,
    .mode_configuration = &mode_configuration,
    .pull_up_down_configuration = &pull_up_down_configuration,
    .get_io_level = &get_io_level,
    .set_io_level = &set_io_level,
};

static void pio_driver_init(void)
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

static void pio_driver_clock_init(void)
{
    __asm("nop");
}

static void mode_configuration(const enum gpio_port port, const uint8_t pin, const enum gpio_direction mode)
{
    // TODO
}

static void pull_up_down_configuration(const enum gpio_port port, const uint8_t pin, const enum gpio_pull_mode mode)
{
    // TODO
}

static bool get_io_level(const enum gpio_port port, const uint8_t pin)
{
    // TODO
    return false;
}

static void set_io_level(const enum gpio_port port, const uint8_t pin, const bool level)
{
    // TODO
}
