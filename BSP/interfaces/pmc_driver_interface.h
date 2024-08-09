#ifndef _PMC_DRIVER_INTERFACE_H_
#define _PMC_DRIVER_INTERFACE_H_

#include "mcu.h"

struct pmc_driver_interface
{
    void (*const init)(void);                                  // Initializes the peripheral
    void (*const enable_peripheral_clock)(const uint32_t pid); // Enables a peripheral's source clock
};

#endif // _PMC_DRIVER_INTERFACE_H_
