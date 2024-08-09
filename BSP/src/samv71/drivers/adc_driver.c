#include "adc_driver.h"

#include "mcu.h"
#include "pmc_driver.h"

static void adc_driver_init(void);
static void adc_driver_clock_init(void);
static void adc_driver_enable(void);
static uint16_t adc_driver_read(void);

extern const struct pmc_driver_interface pmc_driver;

extern const struct adc_driver_interface adc_driver;
const struct adc_driver_interface adc_driver = {
    .init = &adc_driver_init,
    .clock_init = &adc_driver_clock_init,
    .enable = &adc_driver_enable,
    .read = &adc_driver_read,
};

static void adc_driver_init(void)
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

static void adc_driver_clock_init(void)
{
    pmc_driver.enable_peripheral_clock(ID_AFEC1);
}

static void adc_driver_enable(void)
{
    // Enabling channel 0
    AFEC1->AFEC_CHER = (1 << 0);

    // Trigger the first conversion, afterwards it should run in free run mode
    AFEC1->AFEC_CR = AFEC_CR_START;
}

static uint16_t adc_driver_read(void)
{
    // Check for end of conversion flag for channel 0
    // Could check either EOC0 or DRDY since we are only using one channel
    while(!(AFEC1->AFEC_ISR & (1 << 0))){}

    // reading AFEC_CDR clears the EOC bit
    // reading AFEC_LCDR clears the DRDY bit
    // To read AFEC_CDR first select the channel in AFEC_CSELR
    AFEC1->AFEC_CSELR = 0;
    const uint16_t res = AFEC1->AFEC_CDR;

    return res;
}
