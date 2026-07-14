#include "pump_controller.h"

#include <limits.h>
#include <string.h>

static uint8_t increment_saturated(uint8_t value, uint8_t limit)
{
	if (value < limit) {
		value++;
	}
	return value;
}

bool pump_controller_config_valid(const pump_controller_config_t *config)
{
	return config && config->max_pump_ms > 0 && config->min_pause_ms > 0 &&
	       config->dry_confirm_cycles > 0 && config->wet_confirm_cycles > 0;
}

void pump_controller_init(pump_controller_t *controller,
			  const pump_controller_config_t *config,
			  uint64_t now_ms,
			  pump_stop_reason_t initial_reason)
{
	if (!controller || !pump_controller_config_valid(config)) {
		return;
	}

	memset(controller, 0, sizeof(*controller));
	controller->config = *config;
	controller->last_stopped_ms = now_ms;
	controller->last_stop_reason = initial_reason;
}

static pump_controller_action_t stop_pump(pump_controller_t *controller,
					  uint64_t now_ms,
					  pump_stop_reason_t reason)
{
	pump_controller_action_t action = {
		.turn_off = controller->pump_on,
		.stop_reason = reason,
	};
	controller->pump_on = false;
	controller->last_stopped_ms = now_ms;
	controller->last_stop_reason = reason;
	if (reason == PUMP_STOP_SAFETY_TIMEOUT) {
		controller->timeout_count++;
	}
	return action;
}

pump_controller_action_t pump_controller_step(pump_controller_t *controller,
					       pump_level_state_t level,
					       uint64_t now_ms,
					       bool safety_timeout)
{
	pump_controller_action_t action = {0};
	if (!controller || !pump_controller_config_valid(&controller->config)) {
		return action;
	}

	if (level == PUMP_LEVEL_WET) {
		controller->wet_streak = increment_saturated(
			controller->wet_streak, controller->config.wet_confirm_cycles);
		controller->dry_streak = 0;
	} else if (level == PUMP_LEVEL_DRY) {
		controller->dry_streak = increment_saturated(
			controller->dry_streak, controller->config.dry_confirm_cycles);
		controller->wet_streak = 0;
	} else {
		controller->dry_streak = 0;
		controller->wet_streak = 0;
		controller->level_known = false;
	}

	if (controller->wet_streak >= controller->config.wet_confirm_cycles) {
		controller->level_full = true;
		controller->level_known = true;
	}
	if (controller->dry_streak >= controller->config.dry_confirm_cycles) {
		controller->level_full = false;
		controller->level_known = true;
	}

	if (controller->pump_on) {
		if (safety_timeout ||
		    now_ms - controller->pump_started_ms >= controller->config.max_pump_ms) {
			return stop_pump(controller, now_ms, PUMP_STOP_SAFETY_TIMEOUT);
		}
		if (level == PUMP_LEVEL_UNKNOWN) {
			return stop_pump(controller, now_ms, PUMP_STOP_SENSOR_UNKNOWN);
		}
		if (controller->level_known && controller->level_full) {
			return stop_pump(controller, now_ms, PUMP_STOP_LEVEL_WET);
		}
		return action;
	}

	if (level == PUMP_LEVEL_UNKNOWN || !controller->level_known ||
	    controller->level_full ||
	    controller->dry_streak < controller->config.dry_confirm_cycles) {
		return action;
	}

	if (now_ms - controller->last_stopped_ms < controller->config.min_pause_ms) {
		return action;
	}

	controller->pump_on = true;
	controller->pump_started_ms = now_ms;
	action.turn_on = true;
	return action;
}

void pump_controller_note_driver_error(pump_controller_t *controller,
				       uint64_t now_ms)
{
	if (!controller) {
		return;
	}
	controller->pump_on = false;
	controller->last_stopped_ms = now_ms;
	controller->last_stop_reason = PUMP_STOP_DRIVER_ERROR;
}

uint32_t pump_controller_cooldown_remaining_ms(const pump_controller_t *controller,
					       uint64_t now_ms)
{
	if (!controller || now_ms <= controller->last_stopped_ms) {
		return controller ? controller->config.min_pause_ms : 0;
	}
	uint64_t elapsed = now_ms - controller->last_stopped_ms;
	if (elapsed >= controller->config.min_pause_ms) {
		return 0;
	}
	uint64_t remaining = controller->config.min_pause_ms - elapsed;
	return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}
