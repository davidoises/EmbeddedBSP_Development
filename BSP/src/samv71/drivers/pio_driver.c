#include "pio_driver.h"

#include "mcu.h"
#include "pio_config.h"
#include "pmc_driver.h"


static void pio_driver_init(void);
static void pio_driver_clock_init(void);
static void mode_configuration(const enum gpio_port port, const uint32_t pin, const enum gpio_mode mode);
static void pull_up_down_configuration(const enum gpio_port port, const uint32_t pin, const enum gpio_pull_mode mode);
static bool get_io_level(const enum gpio_port port, const uint32_t pin);
static void set_io_level(const enum gpio_port port, const uint32_t pin, const bool level);

extern const uint16_t pio_pids[PIO_PID_COUNT];
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

static inline Pio *port_to_reg(const enum gpio_port port)
{
	/* PIO instance offset is 0x200 */
	return (Pio *)((uint32_t)PIOA + port * 0x200);
}

static void pio_driver_init(void)
{
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

    // uint32_t temp;

    // // PB12 input with pull up
    // MATRIX->CCFG_SYSIO |= PIO_PB12;
    // PIOB->PIO_ODR |= PIO_PB12; //  Disable output
    // PIOB->PIO_PPDDR |= PIO_PB12; // Disable pulldown
    // PIOB->PIO_PUER |= PIO_PB12; // Enable Pull up
    // PIOB->PIO_PER |= PIO_PB12; // I/O mode

    // // PC9 output, start as low output
    // PIOC->PIO_CODR |= PIO_PC9; // Clear output
    // PIOC->PIO_OER |= PIO_PC9; //  Enable output
    // PIOC->PIO_PER |= PIO_PC9; // I/O mode

    // // PC10 output, start as low output
    // PIOC->PIO_CODR |= PIO_PC10; // Clear output
    // PIOC->PIO_OER |= PIO_PC10; //  Enable output
    // PIOC->PIO_PER |= PIO_PC10; // I/O mode

    // // PA9 UART0 Rx (Peripheral function A)
    // temp = PIOA->PIO_ABCDSR[0];
    // temp &= ~PIO_PA9;
    // PIOA->PIO_ABCDSR[0] = temp;

    // temp = PIOA->PIO_ABCDSR[1];
    // temp &= ~PIO_PA9;
    // PIOA->PIO_ABCDSR[1] = temp;

    // PIOA->PIO_PDR |= PIO_PA9; // peripheral mode

    // // PA10 UART0 Tx (Peripheral function A)
    // temp = PIOA->PIO_ABCDSR[0];
    // temp &= ~PIO_PA10;
    // PIOA->PIO_ABCDSR[0] = temp;

    // temp = PIOA->PIO_ABCDSR[1];
    // temp &= ~PIO_PA10;
    // PIOA->PIO_ABCDSR[1] = temp;

    // PIOA->PIO_PDR |= PIO_PA10; // peripheral mode

    // // PB1 AFEC 1 Channel 0 (dont select peripheral function, just set as PIO)
    // PIOB->PIO_PER |= PIO_PB1; // I/O mode
}

static void pio_driver_clock_init(void)
{
    for (uint16_t i = 0; i < PIO_PID_COUNT; i++)
    {
        pmc_driver.enable_peripheral_clock(pio_pids[i]);
    }
}

static void mode_configuration(const enum gpio_port port, const uint32_t pin, const enum gpio_mode mode)
{
    uint32_t temp;

    Pio *const PIO_REG = port_to_reg(port);

    if (port == GPIO_PORTB)
    {
        if ((pin == PIO_PB4) || (pin == PIO_PB5) || (pin == PIO_PB6) || (pin == PIO_PB7) || (pin == PIO_PB12)) {
			MATRIX->CCFG_SYSIO |= PIO_PB12;
		}
    }

    switch (mode)
    {
	    case GPIO_DIRECTION_OFF:
	    case GPIO_DIRECTION_IN:
            PIO_REG->PIO_ODR |= pin; //  Disable output
            PIO_REG->PIO_PER |= pin; // I/O mode
		    break;

	    case GPIO_DIRECTION_OUT:
		    PIO_REG->PIO_OER |= pin; // Enable ouput
            PIO_REG->PIO_PER |= pin; // I/O mode
		    break;

	    case GPIO_PERIPH_A:
            // bit0: 0
            temp = PIO_REG->PIO_ABCDSR[0];
            temp &= ~pin;
            PIO_REG->PIO_ABCDSR[0] = temp;

            // bit1: 0
            temp = PIO_REG->PIO_ABCDSR[1];
            temp &= ~pin;
            PIO_REG->PIO_ABCDSR[1] = temp;

            PIO_REG->PIO_PDR |= pin; // peripheral mode
            break;

        case GPIO_PERIPH_B:
            // bit0: 1
            temp = PIO_REG->PIO_ABCDSR[0];
            temp |= pin;
            PIO_REG->PIO_ABCDSR[0] = temp;

            // bit1: 0
            temp = PIO_REG->PIO_ABCDSR[1];
            temp &= ~pin;
            PIO_REG->PIO_ABCDSR[1] = temp;

            PIO_REG->PIO_PDR |= pin; // peripheral mode
            break;

        case GPIO_PERIPH_C:
            // bit0: 0
            temp = PIO_REG->PIO_ABCDSR[0];
            temp &= ~pin;
            PIO_REG->PIO_ABCDSR[0] = temp;

            // bit1: 1
            temp = PIO_REG->PIO_ABCDSR[1];
            temp |= pin;
            PIO_REG->PIO_ABCDSR[1] = temp;

            PIO_REG->PIO_PDR |= pin; // peripheral mode
            break;

        case GPIO_PERIPH_D:
            // bit0: 0
            temp = PIO_REG->PIO_ABCDSR[0];
            temp |= pin;
            PIO_REG->PIO_ABCDSR[0] = temp;

            // bit1: 1
            temp = PIO_REG->PIO_ABCDSR[1];
            temp |= pin;
            PIO_REG->PIO_ABCDSR[1] = temp;

            PIO_REG->PIO_PDR |= pin; // peripheral mode
            break;
	}
}

static void pull_up_down_configuration(const enum gpio_port port, const uint32_t pin, const enum gpio_pull_mode mode)
{
    Pio *const PIO_REG = port_to_reg(port);

    switch (mode)
    {
        case GPIO_PULL_OFF:
            PIO_REG->PIO_PPDDR |= pin; // Disable pulldown
            PIO_REG->PIO_PUDR |= pin; // Disable Pull up
            break;

        case GPIO_PULL_UP:
            PIO_REG->PIO_PPDDR |= pin; // Disable pulldown
            PIO_REG->PIO_PUER |= pin; // Enable Pull up
            break;

        case GPIO_PULL_DOWN:
            PIO_REG->PIO_PPDER |= pin; // Enable pulldown
            PIO_REG->PIO_PUDR |= pin; // Disable Pull up
            break;
	}

}

static bool get_io_level(const enum gpio_port port, const uint32_t pin)
{
    Pio *const PIO_REG = port_to_reg(port);

    return ((PIO_REG->PIO_PDSR & pin) > 0);
}

static void set_io_level(const enum gpio_port port, const uint32_t pin, const bool level)
{
    Pio *const PIO_REG = port_to_reg(port);

    if (level)
    {
        PIO_REG->PIO_SODR |= pin;
	} else
    {
		PIO_REG->PIO_CODR |= pin;
	}
}
