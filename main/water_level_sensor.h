#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "pump_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	gpio_num_t electrode_a_gpio;
	gpio_num_t electrode_b_gpio;
	uint16_t dry_threshold_mv;
	uint16_t wet_threshold_mv;
} water_level_sensor_config_t;

typedef struct {
	pump_level_state_t state;
	int voltage_ab_mv;
	int voltage_ba_mv;
	bool calibrated;
	bool approximate_fallback;
	esp_err_t error;
} water_level_snapshot_t;

esp_err_t water_level_sensor_init(const water_level_sensor_config_t *config);
esp_err_t water_level_sensor_read(water_level_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
