#include "pmc_driver.h"

void pmc_enable_peripheral_clock(uint32_t pid)
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

    // TODO:
    // I2S peripherals need to follow the next steps but wont be implemented at this time
    // In theory the previous part can also be achieved through the regiser PMC_PCR
    // 1. Set the preripheral ID number
    // 2. Change the CLKDIV or GCLKSS if needed. By default GCLKCSS is SLOW_CLK
    // 3. Then set CMD = 1 (write) and EN = 1
}
