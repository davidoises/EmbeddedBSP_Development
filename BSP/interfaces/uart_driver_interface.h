#ifndef _UART_DRIVER_INTERFACE_H_
#define _UART_DRIVER_INTERFACE_H_

#include "mcu.h"

struct uart_driver_interface
{
    void (*const init)(void);                                               // Initializes the peripheral
    void (*const clock_init)(void);                                         // Initializes the clock powering the peripheral
    void (*const enable)(void);                                             // Starts the operation of the transmitter and receiver
    uint32_t (*const write)(const uint8_t *buffer, const uint16_t length);  // Writes N bytes over UART - blocking
    uint32_t (*const read)(uint8_t *buffer, const uint16_t length);         // Reads N bytes over UART - blocking
};

#endif // _UART_DRIVER_INTERFACE_H_
