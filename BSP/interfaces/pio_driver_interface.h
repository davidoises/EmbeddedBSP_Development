#ifndef _PIO_DRIVER_INTERFACE_H_
#define _PIO_DRIVER_INTERFACE_H_

#include "mcu.h"

enum gpio_pull_mode { GPIO_PULL_OFF, GPIO_PULL_UP, GPIO_PULL_DOWN };

enum gpio_direction { GPIO_DIRECTION_OFF, GPIO_DIRECTION_IN, GPIO_DIRECTION_OUT };

enum gpio_port { GPIO_PORTA, GPIO_PORTB, GPIO_PORTC, GPIO_PORTD, GPIO_PORTE };

struct pio_driver_interface
{
    void (*const init)(void);       // Initializes the peripheral
    void (*const clock_init)(void); // Initializes the clock powering the peripheral
    void (*const mode_configuration)(const enum gpio_port, const uint8_t, const enum gpio_direction); //
    void (*const pull_up_down_configuration)(const enum gpio_port, const uint8_t, const enum gpio_pull_mode); //
    bool (*const get_io_level)(const enum gpio_port, const uint8_t); //
    void (*const set_io_level)(const enum gpio_port, const uint8_t, const bool); //
};

#endif // _PIO_DRIVER_INTERFACE_H_
