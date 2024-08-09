#ifndef _PIO_DRIVER_INTERFACE_H_
#define _PIO_DRIVER_INTERFACE_H_

struct pio_driver_interface
{
    void (*const init)(void);       // Initializes the peripheral
    void (*const clock_init)(void); // Initializes the clock powering the peripheral
};

#endif // _PIO_DRIVER_INTERFACE_H_
