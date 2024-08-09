#ifndef _MCU_H_
#define _MCU_H_

#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
#include "samd21.h"
#elif defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
#include "samv71.h"
#include "fpu.h"
#include "samv71/drivers/pio_driver.h"
#include "samv71/drivers/pmc_driver.h"
#include "samv71/drivers/adc_driver.h"
#else
  #error Library does not support the specified device.
#endif

#endif // _MCU_H_
