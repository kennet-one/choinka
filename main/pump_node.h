#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "pump_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	gpio_num_t level_a_gpio;
	gpio_num_t level_b_gpio;
	gpio_num_t pump_gpio;
} pump_node_pins_t;

typedef struct {
	pump_level_state_t level_state;
	int voltage_ab_mv;
	int voltage_ba_mv;
	bool adc_calibrated;
	bool approximate_fallback;
	bool pump_on;
	uint32_t pump_run_ms;
	uint32_t cooldown_remaining_ms;
	uint32_t timeout_count;
	uint8_t dry_streak;
	uint8_t wet_streak;
	pump_stop_reason_t last_stop_reason;
	esp_err_t last_error;
} pump_node_status_t;

/* Configure the pump output in its inactive state as early as possible. */
esp_err_t pump_node_early_safe_init(gpio_num_t pump_gpio);

esp_err_t pump_node_init(const pump_node_pins_t *pins);
esp_err_t pump_node_start_task(UBaseType_t priority);
bool pump_node_get_status(pump_node_status_t *status);

int pump_node_get_last_level_percent(void);
bool pump_node_is_pump_on(void);

const char *pump_node_level_name(pump_level_state_t state);
const char *pump_node_stop_reason_name(pump_stop_reason_t reason);

#ifdef __cplusplus
}
#endif
