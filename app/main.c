#include "mcu.h"
#include "pio_driver_interface.h"

extern const struct pio_driver_interface pio_driver;

int main()
{
    pio_driver.init();
    pio_driver.clock_init();

    while(1)
    {
    }
}
