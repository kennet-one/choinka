#include "pump_driver.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

typedef struct {
	bool initialized;
	bool active_high;
	bool enabled;
	gpio_num_t gpio;
} pump_driver_context_t;

static pump_driver_context_t s_driver;
static portMUX_TYPE s_driver_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t output_level(bool enabled)
{
	return enabled == s_driver.active_high ? 1U : 0U;
}

esp_err_t pump_driver_init(gpio_num_t gpio, bool active_high)
{
	if (!GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
		return ESP_ERR_INVALID_ARG;
	}
	if (s_driver.initialized) {
		return s_driver.gpio == gpio && s_driver.active_high == active_high
			       ? ESP_OK
			       : ESP_ERR_INVALID_STATE;
	}

	memset(&s_driver, 0, sizeof(s_driver));
	s_driver.gpio = gpio;
	s_driver.active_high = active_high;

	/* Set the inactive latch before enabling output mode. */
	esp_err_t err = gpio_set_level(gpio, active_high ? 0 : 1);
	if (err != ESP_OK) {
		return err;
	}
	gpio_config_t config = {
		.pin_bit_mask = 1ULL << gpio,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	err = gpio_config(&config);
	if (err != ESP_OK) {
		return err;
	}
	err = gpio_set_level(gpio, active_high ? 0 : 1);
	if (err != ESP_OK) {
		return err;
	}

	s_driver.enabled = false;
	s_driver.initialized = true;
	return ESP_OK;
}

esp_err_t pump_driver_set(bool enabled)
{
	esp_err_t err;
	portENTER_CRITICAL(&s_driver_lock);
	if (!s_driver.initialized) {
		portEXIT_CRITICAL(&s_driver_lock);
		return ESP_ERR_INVALID_STATE;
	}
	err = gpio_set_level(s_driver.gpio, output_level(enabled));
	if (err == ESP_OK) {
		s_driver.enabled = enabled;
	}
	portEXIT_CRITICAL(&s_driver_lock);
	return err;
}

bool pump_driver_is_enabled(void)
{
	portENTER_CRITICAL(&s_driver_lock);
	bool enabled = s_driver.initialized && s_driver.enabled;
	portEXIT_CRITICAL(&s_driver_lock);
	return enabled;
}
