#include "adc_driver.h"

#include "mcu.h"
#include "pmc_driver.h"

static void adc_driver_init(void);
static void adc_driver_clock_init(void);
static void adc_driver_enable(void);
static uint16_t adc_driver_read(void);
static void adc_driver_wait_syncbusy(void);

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
    // Read calib registers here
    uint16_t calib_reg = ADC_CALIB_BIAS_CAL((*(uint32_t *)ADC_FUSES_BIASCAL_ADDR >> ADC_FUSES_BIASCAL_Pos))
	            | ADC_CALIB_LINEARITY_CAL((*(uint64_t *)ADC_FUSES_LINEARITY_0_ADDR >> ADC_FUSES_LINEARITY_0_Pos));

    adc_driver_wait_syncbusy();
    if (ADC->CTRLA.bit.ENABLE)
    {
        ADC->CTRLA.bit.ENABLE = 0;
        adc_driver_wait_syncbusy();
    }

    ADC->CTRLA.bit.SWRST = 1;
    adc_driver_wait_syncbusy();

    // Load calib registers here
    ADC->CALIB.reg = calib_reg;

    // INPUTCTRL
    ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;
    adc_driver_wait_syncbusy();
    ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN10_Val;
    adc_driver_wait_syncbusy();

    // REFCTRL // use VCC/2 as the Aref
    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;

    // CTRLB
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV16_Val;
    adc_driver_wait_syncbusy();
    ADC->CTRLB.bit.CORREN = 0; // This seems to be additional correction besides calib
    adc_driver_wait_syncbusy();
    ADC->CTRLB.bit.FREERUN = 1;
    adc_driver_wait_syncbusy();
}

static void adc_driver_clock_init(void)
{
    pmc_driver.enable_peripheral_clock(PM_APBCMASK_ADC);

    // Connecting Generic clock 0 as SERCOM 5 source
    GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0_Val;
    GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_ADC_Val;

    // Enable it
    GCLK->CLKCTRL.bit.CLKEN = 1;
}

static void adc_driver_enable(void)
{
    adc_driver_wait_syncbusy();
    ADC->CTRLA.bit.ENABLE = 1;
    adc_driver_wait_syncbusy();

    ADC->SWTRIG.bit.START = 1;
    adc_driver_wait_syncbusy();
}

static uint16_t adc_driver_read(void)
{
    // wait for RESRDY
    while(!ADC->INTFLAG.bit.RESRDY){}
    adc_driver_wait_syncbusy();
    uint16_t res = ADC->RESULT.reg;
    res &= 0x0FFF;

    return res;
}

static void adc_driver_wait_syncbusy(void)
{
    while (ADC->STATUS.bit.SYNCBUSY){}
}
