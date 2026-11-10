#pragma once

#include <zephyr/drivers/gpio.h>

/* External active-high trigger: P0.05, idle low. */
#define SYNC_PIN_PORT_NODE DT_NODELABEL(gpio0)
#define SYNC_PIN_NUMBER 5U
#define SYNC_PIN_INPUT_FLAGS (GPIO_INPUT | GPIO_PULL_DOWN)

int sync_pin_init(void);

/* Physical level (0/1), or a negative GPIO error code. */
int sync_pin_get_state(void);
