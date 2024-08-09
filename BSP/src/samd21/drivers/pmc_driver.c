#include "pmc_driver.h"

#include "mcu.h"

static void pmc_driver_init(void);
static void pmc_driver_enable_peripheral_clock(const uint32_t mask);

extern const struct pmc_driver_interface pmc_driver;
const struct pmc_driver_interface pmc_driver = {
    .init = &pmc_driver_init,
    .enable_peripheral_clock = &pmc_driver_enable_peripheral_clock,
};

static void pmc_driver_init(void)
{
    // Set the clocks, 8MHZ source just change the prescaler to 1 instead of 8
	SYSCTRL->OSC8M.bit.PRESC = 0;
}

void pmc_driver_enable_peripheral_clock(const uint32_t mask)
{
    PM->APBCMASK.reg |= mask;
    // Enabling the SERCOM5 clock
    // PM->APBCMASK.reg |= PM_APBCMASK_SERCOM5;
    // PM->APBCMASK.reg |= PM_APBCMASK_ADC;
}
