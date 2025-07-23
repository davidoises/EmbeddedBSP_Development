#ifndef _MCU_H_
#define _MCU_H_

#include <stdbool.h>

#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
#include "samd21.h"
#elif defined(__SAMV71Q21B__) || defined(__ATSAMV71Q21B__)
#include "samv71.h"
#include "fpu.h"
#elif defined(STM32F446xx)
#include "stm32f4xx.h"  
#else
  #error Library does not support the specified device.
#endif

#endif // _MCU_H_
