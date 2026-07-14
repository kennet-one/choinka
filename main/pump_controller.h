#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	PUMP_LEVEL_UNKNOWN = 0,
	PUMP_LEVEL_DRY,
	PUMP_LEVEL_WET,
} pump_level_state_t;

typedef enum {
	PUMP_STOP_NONE = 0,
	PUMP_STOP_BOOT_PULSE,
	PUMP_STOP_LEVEL_WET,
	PUMP_STOP_SENSOR_UNKNOWN,
	PUMP_STOP_SAFETY_TIMEOUT,
	PUMP_STOP_DRIVER_ERROR,
} pump_stop_reason_t;

typedef struct {
	uint32_t max_pump_ms;
	uint32_t min_pause_ms;
	uint8_t dry_confirm_cycles;
	uint8_t wet_confirm_cycles;
} pump_controller_config_t;

typedef struct {
	pump_controller_config_t config;
	bool pump_on;
	bool level_full;
	bool level_known;
	uint8_t dry_streak;
	uint8_t wet_streak;
	uint64_t pump_started_ms;
	uint64_t last_stopped_ms;
	uint32_t timeout_count;
	pump_stop_reason_t last_stop_reason;
} pump_controller_t;

typedef struct {
	bool turn_on;
	bool turn_off;
	pump_stop_reason_t stop_reason;
} pump_controller_action_t;

bool pump_controller_config_valid(const pump_controller_config_t *config);

void pump_controller_init(pump_controller_t *controller,
			  const pump_controller_config_t *config,
			  uint64_t now_ms,
			  pump_stop_reason_t initial_reason);

pump_controller_action_t pump_controller_step(pump_controller_t *controller,
					       pump_level_state_t level,
					       uint64_t now_ms,
					       bool safety_timeout);

void pump_controller_note_driver_error(pump_controller_t *controller,
				       uint64_t now_ms);

uint32_t pump_controller_cooldown_remaining_ms(const pump_controller_t *controller,
					       uint64_t now_ms);

#ifdef __cplusplus
}
#endif
