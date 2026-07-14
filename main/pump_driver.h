#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pump_driver_init(gpio_num_t gpio, bool active_high);
esp_err_t pump_driver_set(bool enabled);
bool pump_driver_is_enabled(void);

#ifdef __cplusplus
}
#endif
