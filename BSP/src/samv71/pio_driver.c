#include "pio_driver.h"
#include "pio_driver_interface.h"


static void pio_driver_init(void);

extern const struct pio_driver_interface pio_driver;
const struct pio_driver_interface pio_driver = {
    .init = &pio_driver_init,
};

static void pio_driver_init(void)
{
    __asm("nop");
}
