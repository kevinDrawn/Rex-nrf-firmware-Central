#include <sync_pin.h>

#include <errno.h>

static const struct device *const sync_port =
    DEVICE_DT_GET(SYNC_PIN_PORT_NODE);

int sync_pin_init(void)
{
    if (!device_is_ready(sync_port)) {
        return -ENODEV;
    }

    return gpio_pin_configure(sync_port, SYNC_PIN_NUMBER,
                              SYNC_PIN_INPUT_FLAGS);
}

int sync_pin_get_state(void)
{
    return gpio_pin_get_raw(sync_port, SYNC_PIN_NUMBER);
}
