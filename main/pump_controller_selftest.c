#include "pump_controller_selftest.h"

#include "pump_controller.h"

#define CHECK(condition) do { if (!(condition)) return false; } while (0)

bool pump_controller_selftest(void)
{
	pump_controller_config_t config = {
		.max_pump_ms = 3000,
		.min_pause_ms = 60000,
		.dry_confirm_cycles = 3,
		.wet_confirm_cycles = 2,
	};
	pump_controller_t controller;
	pump_controller_init(&controller, &config, 1000, PUMP_STOP_BOOT_PULSE);

	pump_controller_action_t action = pump_controller_step(
		&controller, PUMP_LEVEL_DRY, 61000, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_UNKNOWN, 62000, false);
	CHECK(!action.turn_on && !controller.level_known);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 63000, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 64000, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 65000, false);
	CHECK(action.turn_on && controller.pump_on);

	action = pump_controller_step(&controller, PUMP_LEVEL_UNKNOWN, 65100, false);
	CHECK(action.turn_off && !controller.pump_on &&
	      controller.last_stop_reason == PUMP_STOP_SENSOR_UNKNOWN);

	pump_controller_init(&controller, &config, 0, PUMP_STOP_NONE);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60000, false);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60010, false);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 60020, false);
	CHECK(action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 63020, true);
	CHECK(action.turn_off && controller.timeout_count == 1 &&
	      controller.last_stop_reason == PUMP_STOP_SAFETY_TIMEOUT);

	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 123020, false);
	CHECK(action.turn_on);
	pump_controller_step(&controller, PUMP_LEVEL_WET, 123030, false);
	action = pump_controller_step(&controller, PUMP_LEVEL_WET, 123040, false);
	CHECK(action.turn_off && controller.last_stop_reason == PUMP_STOP_LEVEL_WET);

	return true;
}
