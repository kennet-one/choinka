#include "pump_node.h"

#include <inttypes.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "pump_controller_selftest.h"
#include "pump_driver.h"
#include "sdkconfig.h"
#include "water_level_sensor.h"

#ifndef CONFIG_CHOINKA_PUMP_ACTIVE_HIGH
#define CONFIG_CHOINKA_PUMP_ACTIVE_HIGH 0
#endif
#ifndef CONFIG_CHOINKA_PUMP_BOOT_PULSE_ENABLE
#define CONFIG_CHOINKA_PUMP_BOOT_PULSE_ENABLE 0
#endif
#ifndef CONFIG_CHOINKA_PUMP_BOOT_PULSE_MS
#define CONFIG_CHOINKA_PUMP_BOOT_PULSE_MS 200
#endif
#ifndef CONFIG_CHOINKA_PUMP_CHECK_PERIOD_MS
#define CONFIG_CHOINKA_PUMP_CHECK_PERIOD_MS 1000
#endif
#ifndef CONFIG_CHOINKA_PUMP_MAX_RUN_MS
#define CONFIG_CHOINKA_PUMP_MAX_RUN_MS 3000
#endif
#ifndef CONFIG_CHOINKA_PUMP_MIN_PAUSE_MS
#define CONFIG_CHOINKA_PUMP_MIN_PAUSE_MS 60000
#endif
#ifndef CONFIG_CHOINKA_LEVEL_DRY_THRESHOLD_MV
#define CONFIG_CHOINKA_LEVEL_DRY_THRESHOLD_MV 1100
#endif
#ifndef CONFIG_CHOINKA_LEVEL_WET_THRESHOLD_MV
#define CONFIG_CHOINKA_LEVEL_WET_THRESHOLD_MV 1900
#endif
#ifndef CONFIG_CHOINKA_LEVEL_DRY_CONFIRM_CYCLES
#define CONFIG_CHOINKA_LEVEL_DRY_CONFIRM_CYCLES 3
#endif
#ifndef CONFIG_CHOINKA_LEVEL_WET_CONFIRM_CYCLES
#define CONFIG_CHOINKA_LEVEL_WET_CONFIRM_CYCLES 2
#endif

#define PUMP_SUMMARY_PERIOD_MS 15000U

static const char *TAG = "pump_node";

typedef struct {
	bool initialized;
	pump_node_pins_t pins;
	pump_controller_t controller;
	pump_node_status_t status;
	SemaphoreHandle_t status_mutex;
	TaskHandle_t task;
	esp_timer_handle_t safety_timer;
	portMUX_TYPE safety_lock;
	bool safety_timeout_pending;
	esp_err_t safety_driver_error;
} pump_node_context_t;

static pump_node_context_t s_pump = {
	.safety_lock = portMUX_INITIALIZER_UNLOCKED,
};

static void cleanup_partial_runtime(void)
{
	(void)pump_driver_set(false);
	if (s_pump.safety_timer) {
		(void)esp_timer_stop(s_pump.safety_timer);
		(void)esp_timer_delete(s_pump.safety_timer);
		s_pump.safety_timer = NULL;
	}
	if (s_pump.status_mutex) {
		vSemaphoreDelete(s_pump.status_mutex);
		s_pump.status_mutex = NULL;
	}
}

static uint64_t now_ms(void)
{
	return (uint64_t)esp_timer_get_time() / 1000ULL;
}

const char *pump_node_level_name(pump_level_state_t state)
{
	switch (state) {
	case PUMP_LEVEL_DRY:
		return "dry";
	case PUMP_LEVEL_WET:
		return "wet";
	default:
		return "unknown";
	}
}

const char *pump_node_stop_reason_name(pump_stop_reason_t reason)
{
	switch (reason) {
	case PUMP_STOP_BOOT_PULSE:
		return "boot_pulse";
	case PUMP_STOP_LEVEL_WET:
		return "level_wet";
	case PUMP_STOP_SENSOR_UNKNOWN:
		return "sensor_unknown";
	case PUMP_STOP_SAFETY_TIMEOUT:
		return "safety_timeout";
	case PUMP_STOP_DRIVER_ERROR:
		return "driver_error";
	default:
		return "none";
	}
}

esp_err_t pump_node_early_safe_init(gpio_num_t pump_gpio)
{
	esp_err_t err = pump_driver_init(pump_gpio,
					 CONFIG_CHOINKA_PUMP_ACTIVE_HIGH != 0);
	if (err != ESP_OK) {
		return err;
	}
	return pump_driver_set(false);
}

static void safety_timer_callback(void *arg)
{
	(void)arg;
	esp_err_t driver_error = pump_driver_set(false);
	TaskHandle_t task = NULL;

	portENTER_CRITICAL(&s_pump.safety_lock);
	s_pump.safety_timeout_pending = true;
	s_pump.safety_driver_error = driver_error;
	task = s_pump.task;
	portEXIT_CRITICAL(&s_pump.safety_lock);

	if (task) {
		xTaskNotifyGive(task);
	}
}

static bool take_safety_timeout(esp_err_t *driver_error)
{
	bool pending;
	portENTER_CRITICAL(&s_pump.safety_lock);
	pending = s_pump.safety_timeout_pending;
	if (driver_error) {
		*driver_error = s_pump.safety_driver_error;
	}
	s_pump.safety_timeout_pending = false;
	s_pump.safety_driver_error = ESP_OK;
	portEXIT_CRITICAL(&s_pump.safety_lock);
	return pending;
}

static esp_err_t apply_controller_action(const pump_controller_action_t *action,
					 uint64_t current_ms)
{
	if (!action) {
		return ESP_ERR_INVALID_ARG;
	}
	if (action->turn_off) {
		esp_err_t stop_error = esp_timer_stop(s_pump.safety_timer);
		if (stop_error != ESP_OK && stop_error != ESP_ERR_INVALID_STATE) {
			ESP_LOGW(TAG, "failed to stop pump safety timer: %s",
				 esp_err_to_name(stop_error));
		}
		esp_err_t err = pump_driver_set(false);
		if (err != ESP_OK) {
			pump_controller_note_driver_error(&s_pump.controller, current_ms);
			return err;
		}
	}

	if (action->turn_on) {
		esp_err_t err = esp_timer_start_once(
			s_pump.safety_timer,
			(uint64_t)CONFIG_CHOINKA_PUMP_MAX_RUN_MS * 1000ULL);
		if (err != ESP_OK) {
			pump_controller_note_driver_error(&s_pump.controller, current_ms);
			return err;
		}
		err = pump_driver_set(true);
		if (err != ESP_OK) {
			esp_timer_stop(s_pump.safety_timer);
			pump_driver_set(false);
			pump_controller_note_driver_error(&s_pump.controller, current_ms);
			return err;
		}
	}
	return ESP_OK;
}

static void publish_status(const water_level_snapshot_t *sensor,
			   esp_err_t last_error, uint64_t current_ms)
{
	pump_node_status_t next = {
		.level_state = sensor ? sensor->state : PUMP_LEVEL_UNKNOWN,
		.voltage_ab_mv = sensor ? sensor->voltage_ab_mv : 0,
		.voltage_ba_mv = sensor ? sensor->voltage_ba_mv : 0,
		.adc_calibrated = sensor && sensor->calibrated,
		.approximate_fallback = sensor && sensor->approximate_fallback,
		.pump_on = s_pump.controller.pump_on && pump_driver_is_enabled(),
		.cooldown_remaining_ms = pump_controller_cooldown_remaining_ms(
			&s_pump.controller, current_ms),
		.timeout_count = s_pump.controller.timeout_count,
		.dry_streak = s_pump.controller.dry_streak,
		.wet_streak = s_pump.controller.wet_streak,
		.last_stop_reason = s_pump.controller.last_stop_reason,
		.last_error = last_error,
	};
	if (next.pump_on) {
		uint64_t elapsed = current_ms - s_pump.controller.pump_started_ms;
		next.pump_run_ms = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
	}

	if (xSemaphoreTake(s_pump.status_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
		s_pump.status = next;
		xSemaphoreGive(s_pump.status_mutex);
	}
}

static void log_state_change(const pump_node_status_t *before,
			     const pump_node_status_t *after)
{
	if (!before || !after) {
		return;
	}
	if (before->level_state != after->level_state) {
		ESP_LOGI(TAG, "level %s -> %s (%d/%d mV, cal:%u)",
			 pump_node_level_name(before->level_state),
			 pump_node_level_name(after->level_state),
			 after->voltage_ab_mv, after->voltage_ba_mv,
			 after->adc_calibrated ? 1U : 0U);
	}
	if (before->pump_on != after->pump_on) {
		ESP_LOGI(TAG, "pump %s, reason:%s timeout_count:%" PRIu32,
			 after->pump_on ? "ON" : "OFF",
			 pump_node_stop_reason_name(after->last_stop_reason),
			 after->timeout_count);
	}
}

static void pump_node_task(void *arg)
{
	(void)arg;
	uint64_t last_summary_ms = 0;
	pump_node_status_t previous = {0};
	pump_node_get_status(&previous);

	for (;;) {
		water_level_snapshot_t sensor = {0};
		esp_err_t sensor_error = water_level_sensor_read(&sensor);
		if (sensor_error != ESP_OK) {
			sensor.state = PUMP_LEVEL_UNKNOWN;
		}

		esp_err_t safety_driver_error = ESP_OK;
		bool safety_timeout = take_safety_timeout(&safety_driver_error);
		uint64_t current_ms = now_ms();
		pump_controller_action_t action = pump_controller_step(
			&s_pump.controller, sensor.state, current_ms, safety_timeout);
		esp_err_t action_error = apply_controller_action(&action, current_ms);
		esp_err_t last_error = sensor_error != ESP_OK ? sensor_error : action_error;
		if (safety_driver_error != ESP_OK) {
			last_error = safety_driver_error;
			pump_controller_note_driver_error(&s_pump.controller, current_ms);
		}

		publish_status(&sensor, last_error, current_ms);
		pump_node_status_t current = {0};
		pump_node_get_status(&current);
		log_state_change(&previous, &current);
		previous = current;

		if (last_summary_ms == 0 || current_ms - last_summary_ms >=
					      PUMP_SUMMARY_PERIOD_MS) {
			last_summary_ms = current_ms;
			ESP_LOGI(TAG,
				 "state:%s pump:%u voltage:%d/%d mV cooldown:%" PRIu32
				 "ms stop:%s timeouts:%" PRIu32 " err:%s",
				 pump_node_level_name(current.level_state),
				 current.pump_on ? 1U : 0U,
				 current.voltage_ab_mv, current.voltage_ba_mv,
				 current.cooldown_remaining_ms,
				 pump_node_stop_reason_name(current.last_stop_reason),
				 current.timeout_count,
				 esp_err_to_name(current.last_error));
		}

		ulTaskNotifyTake(pdTRUE,
				 pdMS_TO_TICKS(CONFIG_CHOINKA_PUMP_CHECK_PERIOD_MS));
	}
}

esp_err_t pump_node_init(const pump_node_pins_t *pins)
{
	if (!pins) {
		return ESP_ERR_INVALID_ARG;
	}
	if (s_pump.initialized) {
		return s_pump.pins.level_a_gpio == pins->level_a_gpio &&
		       s_pump.pins.level_b_gpio == pins->level_b_gpio &&
		       s_pump.pins.pump_gpio == pins->pump_gpio
			       ? ESP_OK
			       : ESP_ERR_INVALID_STATE;
	}

	pump_controller_config_t controller_config = {
		.max_pump_ms = CONFIG_CHOINKA_PUMP_MAX_RUN_MS,
		.min_pause_ms = CONFIG_CHOINKA_PUMP_MIN_PAUSE_MS,
		.dry_confirm_cycles = CONFIG_CHOINKA_LEVEL_DRY_CONFIRM_CYCLES,
		.wet_confirm_cycles = CONFIG_CHOINKA_LEVEL_WET_CONFIRM_CYCLES,
	};
	if (!pump_controller_config_valid(&controller_config) ||
	    CONFIG_CHOINKA_LEVEL_DRY_THRESHOLD_MV >=
		    CONFIG_CHOINKA_LEVEL_WET_THRESHOLD_MV) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = pump_node_early_safe_init(pins->pump_gpio);
	if (err != ESP_OK) {
		return err;
	}
	water_level_sensor_config_t sensor_config = {
		.electrode_a_gpio = pins->level_a_gpio,
		.electrode_b_gpio = pins->level_b_gpio,
		.dry_threshold_mv = CONFIG_CHOINKA_LEVEL_DRY_THRESHOLD_MV,
		.wet_threshold_mv = CONFIG_CHOINKA_LEVEL_WET_THRESHOLD_MV,
	};
	err = water_level_sensor_init(&sensor_config);
	if (err != ESP_OK) {
		return err;
	}
	if (!pump_controller_selftest()) {
		ESP_LOGE(TAG, "pump controller self-test failed");
		return ESP_FAIL;
	}
	ESP_LOGI(TAG, "pump controller self-test passed");

	s_pump.status_mutex = xSemaphoreCreateMutex();
	if (!s_pump.status_mutex) {
		return ESP_ERR_NO_MEM;
	}
	esp_timer_create_args_t timer_args = {
		.callback = safety_timer_callback,
		.name = "pump_guard",
		.dispatch_method = ESP_TIMER_TASK,
	};
	err = esp_timer_create(&timer_args, &s_pump.safety_timer);
	if (err != ESP_OK) {
		cleanup_partial_runtime();
		return err;
	}

	s_pump.pins = *pins;
	pump_stop_reason_t initial_reason = PUMP_STOP_NONE;
	if (CONFIG_CHOINKA_PUMP_BOOT_PULSE_ENABLE &&
	    esp_reset_reason() == ESP_RST_POWERON) {
		err = pump_driver_set(true);
		if (err == ESP_OK) {
			vTaskDelay(pdMS_TO_TICKS(CONFIG_CHOINKA_PUMP_BOOT_PULSE_MS));
			err = pump_driver_set(false);
		}
		if (err != ESP_OK) {
			cleanup_partial_runtime();
			return err;
		}
		initial_reason = PUMP_STOP_BOOT_PULSE;
		ESP_LOGI(TAG, "cold-boot pump pulse completed (%d ms)",
			 CONFIG_CHOINKA_PUMP_BOOT_PULSE_MS);
	}

	uint64_t current_ms = now_ms();
	pump_controller_init(&s_pump.controller, &controller_config, current_ms,
			     initial_reason);
	s_pump.initialized = true;
	water_level_snapshot_t initial_sensor = {
		.state = PUMP_LEVEL_UNKNOWN,
	};
	publish_status(&initial_sensor, ESP_OK, current_ms);

	ESP_LOGI(TAG,
		 "initialized A:GPIO%d B:GPIO%d pump:GPIO%d active_high:%u",
		 (int)pins->level_a_gpio, (int)pins->level_b_gpio,
		 (int)pins->pump_gpio, CONFIG_CHOINKA_PUMP_ACTIVE_HIGH ? 1U : 0U);
	return ESP_OK;
}

esp_err_t pump_node_start_task(UBaseType_t priority)
{
	if (!s_pump.initialized) {
		return ESP_ERR_INVALID_STATE;
	}
	if (s_pump.task) {
		return ESP_OK;
	}
	TaskHandle_t task = NULL;
	BaseType_t created = xTaskCreate(pump_node_task, "pump_node", 5120, NULL,
					priority, &task);
	if (created != pdPASS) {
		return ESP_ERR_NO_MEM;
	}
	portENTER_CRITICAL(&s_pump.safety_lock);
	s_pump.task = task;
	portEXIT_CRITICAL(&s_pump.safety_lock);
	return ESP_OK;
}

bool pump_node_get_status(pump_node_status_t *status)
{
	if (!status || !s_pump.status_mutex) {
		return false;
	}
	if (xSemaphoreTake(s_pump.status_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
		return false;
	}
	*status = s_pump.status;
	xSemaphoreGive(s_pump.status_mutex);
	return true;
}

int pump_node_get_last_level_percent(void)
{
	pump_node_status_t status = {0};
	if (!pump_node_get_status(&status)) {
		return 0;
	}
	return status.level_state == PUMP_LEVEL_WET ? 100 : 0;
}

bool pump_node_is_pump_on(void)
{
	pump_node_status_t status = {0};
	return pump_node_get_status(&status) && status.pump_on;
}
