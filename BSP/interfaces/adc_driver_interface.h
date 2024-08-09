#ifndef _ADC_DRIVER_INTERFACE_H_
#define _ADC_DRIVER_INTERFACE_H_

#include "mcu.h"

struct adc_driver_interface
{
    void (*const init)(void);           // Initializes the peripheral
    void (*const clock_init)(void);     // Initializes the clock powering the peripheral
    void (*const enable)(void);         // Starts the operation of the transmitter and receiver
    uint16_t (*const read)(void);       // Reads N bytes over UART - blocking
};

#endif // _ADC_DRIVER_INTERFACE_H_
