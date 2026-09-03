#pragma once

#include "pump_controller.h"

enum {
    WATER_TEST_IO = 1,
    WATER_TEST_NOT_LOW = 2,
    WATER_TEST_ASYMMETRY = 4,
    WATER_TEST_UNCERTAIN = 8,
};

/* Electrical plausibility only: an open cable can look dry, a short can look wet. */
uint8_t water_level_health_check(int high_ab, int high_ba, int low_ab, int low_ba,
                                int dry_mv, int wet_mv,
                                pump_level_state_t *level);

pump_level_state_t water_level_control_state(int high_ab, int high_ba,
    int dry_mv, int wet_mv, uint8_t test_flags, bool enforce);
