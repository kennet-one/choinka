#include <stdio.h>
#include "pump_controller_selftest.h"

int main(void)
{
    if (!pump_controller_selftest()) {
        fputs("FAIL: pump controller / electrode plausibility\n", stderr);
        return 1;
    }
    puts("PASS: pump controller / electrode plausibility (no hardware accessed)");
    return 0;
}
