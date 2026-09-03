#include "water_level_health.h"

pump_level_state_t water_level_control_state(int high_ab, int high_ba,
    int dry_mv, int wet_mv, uint8_t test_flags, bool enforce)
{
    if (high_ab < 0 || high_ba < 0 || dry_mv < 0 || dry_mv >= wet_mv ||
        (test_flags & WATER_TEST_IO) || (enforce && test_flags)) {
        return PUMP_LEVEL_UNKNOWN;
    }
    /* Report-only mode preserves the deployed wet-priority classifier. */
    if (high_ab >= wet_mv || high_ba >= wet_mv) return PUMP_LEVEL_WET;
    if (high_ab <= dry_mv && high_ba <= dry_mv) return PUMP_LEVEL_DRY;
    return PUMP_LEVEL_UNKNOWN;
}

uint8_t water_level_health_check(int high_ab, int high_ba, int low_ab, int low_ba,
                                int dry_mv, int wet_mv,
                                pump_level_state_t *level)
{
    if (!level) return WATER_TEST_IO;
    *level = PUMP_LEVEL_UNKNOWN;
    if (dry_mv < 0 || dry_mv >= wet_mv ||
        high_ab < 0 || high_ba < 0 || low_ab < 0 || low_ba < 0) {
        return WATER_TEST_IO;
    }
    uint8_t flags = 0;
    if (low_ab > dry_mv || low_ba > dry_mv) flags |= WATER_TEST_NOT_LOW;
    bool dry_ab = high_ab <= dry_mv;
    bool dry_ba = high_ba <= dry_mv;
    bool wet_ab = high_ab >= wet_mv;
    bool wet_ba = high_ba >= wet_mv;
    if ((dry_ab && wet_ba) || (dry_ba && wet_ab)) flags |= WATER_TEST_ASYMMETRY;
    if ((!dry_ab && !wet_ab) || (!dry_ba && !wet_ba)) flags |= WATER_TEST_UNCERTAIN;
    if (!flags) {
        if (dry_ab && dry_ba) *level = PUMP_LEVEL_DRY;
        else if (wet_ab && wet_ba) *level = PUMP_LEVEL_WET;
        else flags |= WATER_TEST_ASYMMETRY;
    }
    return flags;
}
