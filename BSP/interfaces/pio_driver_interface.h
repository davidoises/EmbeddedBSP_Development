#ifndef _PIO_DRIVER_INTERFACE_H_
#define _PIO_DRIVER_INTERFACE_H_

struct pio_driver_interface
{
    void (*const init)(void);
};

#endif // _PIO_DRIVER_INTERFACE_H_
