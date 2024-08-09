#include "pmc_driver.h"

#include "pmc_driver_interface.h"
#include "mcu.h"

static void pmc_driver_init(void);
static void pmc_enable_peripheral_clock(const uint32_t pid);

extern const struct pmc_driver_interface pmc_driver;
const struct pmc_driver_interface pmc_driver = {
    .init = &pmc_driver_init,
    .enable_peripheral_clock = &pmc_enable_peripheral_clock,
};

static void pmc_driver_init(void)
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

void pmc_enable_peripheral_clock(const uint32_t pid)
{
    // Refer to section 31.20.4 for PCER0 as an example
    // PCER0, PCSR0, PCDR0 have the same bit assignments
    // PCER0, PCSR0, PCDR0 control clocks for perpheral IDs [7,31]

    // Refer to section 31.20.23 for PCER1 as an example
    // PCER1, PCSR1, PCDR1 have the same bit assignments
    // PCER1, PCSR1, PCDR1 control clocks for perpheral IDs [32,62]

    if (pid < 32U)
    {
        PMC->PMC_PCER0 = (1U << pid);
    }
    else if (pid < 64U)
    {
        uint32_t relative_pid = pid - 32U;
        PMC->PMC_PCER1 = (1U << relative_pid);
    }

    // TODO:
    // I2S peripherals need to follow the next steps but wont be implemented at this time
    // In theory the previous part can also be achieved through the regiser PMC_PCR
    // 1. Set the preripheral ID number
    // 2. Change the CLKDIV or GCLKSS if needed. By default GCLKCSS is SLOW_CLK
    // 3. Then set CMD = 1 (write) and EN = 1
}
