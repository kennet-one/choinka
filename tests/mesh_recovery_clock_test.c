#include <assert.h>
#include <stdio.h>
#include "mesh_recovery_clock.h"

int main(void)
{
    uint32_t started = mesh_recovery_clock_start(0, 100000U);
    /* Repeated no-parent scans used to postpone the hard timeout forever. */
    for (uint32_t now = 105000U; now <= 180000U; now += 5000U) {
        started = mesh_recovery_clock_expedite(started, now, 20000U);
    }
    assert((uint32_t)(180000U - started) >= 60000U);

    /* Parent flaps must not renew the ACK grace or discard elapsed outage time. */
    started = mesh_recovery_clock_start(0, 100000U);
    for (uint32_t now = 105000U; now <= 160000U; now += 5000U) {
        started = mesh_recovery_clock_start(started, now);
    }
    assert(started == 100000U);
    assert((uint32_t)(160000U - started) >= 60000U);

    /* A confirmed healthy interval explicitly clears the clock for a new outage. */
    started = 0;
    assert(mesh_recovery_clock_start(started, 200000U) == 200000U);
    started = UINT32_MAX - 10000U;
    assert(mesh_recovery_clock_expedite(started, 20000U, 20000U) == started);
    assert(mesh_recovery_clock_start(started, 20000U) == started);
    assert(mesh_recovery_clock_start(0, 0) == 1U);
    assert(mesh_recovery_clock_expedite(0, 5000U, 20000U) == 1U);
    puts("PASS: recovery deadlines survive repeated scans, parent flaps and tick wrap");
    return 0;
}
