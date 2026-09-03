#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pump_driver_init(gpio_num_t gpio, bool active_high,
			   gpio_num_t block_gpio);
esp_err_t pump_driver_set(bool enabled);
bool pump_driver_is_enabled(void);
/* Seconds since the last successful OFF-to-ON transition; -1 before any start. */
int64_t pump_driver_last_start_age_s(void);
bool pump_driver_is_hardware_blocked(void);

#ifdef __cplusplus
}
#endif
