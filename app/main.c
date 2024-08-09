#include "mcu.h"
#include "pio_driver_interface.h"

extern const struct pio_driver_interface pio_driver;

int main()
{
    pio_driver.init();

    while(1)
    {
    }
}
