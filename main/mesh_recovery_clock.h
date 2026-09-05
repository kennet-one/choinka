#ifndef CHOINKA_MESH_RECOVERY_CLOCK_H
#define CHOINKA_MESH_RECOVERY_CLOCK_H

#include <stdint.h>

/* A parent event is not proof of application recovery. Keep the first deadline. */
static inline uint32_t mesh_recovery_clock_start(uint32_t started_ms, uint32_t now)
{
    return started_ms != 0 ? started_ms : (now != 0 ? now : 1U);
}

static inline uint32_t mesh_recovery_clock_expedite(uint32_t started_ms,
                                                   uint32_t now,
                                                   uint32_t minimum_age_ms)
{
    if (started_ms != 0 && (uint32_t)(now - started_ms) >= minimum_age_ms) {
        return started_ms;
    }
    return now > minimum_age_ms ? now - minimum_age_ms : 1U;
}

#endif
