#ifndef _MCU_H_
#define _MCU_H_

#include <stdbool.h>

#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
#include "samd21.h"
#elif defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
#include "samv71.h"
#include "fpu.h"
#else
  #error Library does not support the specified device.
#endif

#include "pio_driver.h"
#include "pmc_driver.h"
#include "adc_driver.h"
#include "uart_driver.h"

#endif // _MCU_H_
