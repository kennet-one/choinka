#include "pump_controller_selftest.h"

#include "pump_controller.h"
#include "water_level_health.h"

#define CHECK(condition) do { if (!(condition)) return false; } while (0)

bool pump_controller_selftest(void)
{
	pump_level_state_t sensed = PUMP_LEVEL_UNKNOWN;
	CHECK(water_level_control_state(100, 2200, 1100, 1900, WATER_TEST_ASYMMETRY, false) == PUMP_LEVEL_WET);
	CHECK(water_level_control_state(100, 2200, 1100, 1900, WATER_TEST_ASYMMETRY, true) == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_control_state(100, 100, 1100, 1900, WATER_TEST_NOT_LOW, false) == PUMP_LEVEL_DRY);
	CHECK(water_level_control_state(100, 100, 1100, 1900, WATER_TEST_NOT_LOW, true) == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_control_state(100, 100, 1100, 1900, WATER_TEST_IO, false) == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_control_state(1500, 1500, 1100, 1900, WATER_TEST_UNCERTAIN, false) == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_health_check(100, 100, 50, 50, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_DRY);
	CHECK(water_level_health_check(2100, 2200, 100, 120, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_WET);
	CHECK(water_level_health_check(2100, 2200, 2100, 120, 1100, 1900, &sensed) == WATER_TEST_NOT_LOW);
	CHECK(sensed == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_health_check(100, 2200, 100, 120, 1100, 1900, &sensed) == WATER_TEST_ASYMMETRY);
	CHECK(sensed == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_health_check(2200, 100, 100, 120, 1100, 1900, &sensed) == WATER_TEST_ASYMMETRY);
	CHECK(water_level_health_check(1500, 2200, 100, 120, 1100, 1900, &sensed) == WATER_TEST_UNCERTAIN);
	CHECK(sensed == PUMP_LEVEL_UNKNOWN);
	CHECK(water_level_health_check(0, 0, -1, 0, 1100, 1900, &sensed) == WATER_TEST_IO);
	CHECK(water_level_health_check(1100, 1100, 1100, 1100, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_DRY);
	CHECK(water_level_health_check(1900, 1900, 0, 0, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_WET);
	/* Cable opens/shorts cannot be distinguished from real dry/wet here. */
	CHECK(water_level_health_check(0, 0, 0, 0, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_DRY);
	CHECK(water_level_health_check(3300, 3300, 0, 0, 1100, 1900, &sensed) == 0);
	CHECK(sensed == PUMP_LEVEL_WET);
	pump_controller_config_t config = {
		.max_pump_ms = 3000,
		.min_pause_ms = 60000,
		.dry_confirm_cycles = 3,
		.wet_confirm_cycles = 2,
	};
	pump_controller_t controller;
	pump_controller_init(&controller, &config, 1000, PUMP_STOP_BOOT_PULSE);

	pump_controller_action_t action = pump_controller_step(
		&controller, PUMP_LEVEL_DRY, 61000, false, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_UNKNOWN, 62000,
				      false, false);
	CHECK(!action.turn_on && !controller.level_known);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 63000,
				      false, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 64000,
				      false, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 65000,
				      false, false);
	CHECK(action.turn_on && controller.pump_on);

	action = pump_controller_step(&controller, PUMP_LEVEL_UNKNOWN, 65100,
				      false, false);
	CHECK(action.turn_off && !controller.pump_on &&
	      controller.last_stop_reason == PUMP_STOP_SENSOR_UNKNOWN);

	pump_controller_init(&controller, &config, 0, PUMP_STOP_NONE);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60000, false, false);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60010, false, false);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 60020,
				      false, false);
	CHECK(action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 60030,
				      false, true);
	CHECK(action.turn_off && !controller.pump_on &&
	      controller.dry_streak == 0 &&
	      controller.last_stop_reason == PUMP_STOP_HARDWARE_BLOCK);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 120030,
				      false, true);
	CHECK(!action.turn_on && controller.dry_streak == 0);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 120040,
				      false, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 120050,
				      false, false);
	CHECK(!action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 120060,
				      false, false);
	CHECK(action.turn_on);

	pump_controller_init(&controller, &config, 0, PUMP_STOP_NONE);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60000, false, false);
	pump_controller_step(&controller, PUMP_LEVEL_DRY, 60010, false, false);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 60020,
				      false, false);
	CHECK(action.turn_on);
	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 63020,
				      true, false);
	CHECK(action.turn_off && controller.timeout_count == 1 &&
	      controller.last_stop_reason == PUMP_STOP_SAFETY_TIMEOUT);

	action = pump_controller_step(&controller, PUMP_LEVEL_DRY, 123020,
				      false, false);
	CHECK(action.turn_on);
	pump_controller_step(&controller, PUMP_LEVEL_WET, 123030, false, false);
	action = pump_controller_step(&controller, PUMP_LEVEL_WET, 123040,
				      false, false);
	CHECK(action.turn_off && controller.last_stop_reason == PUMP_STOP_LEVEL_WET);

	return true;
}
