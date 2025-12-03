#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/**
 * Обробка текстового payload’у (backward compatible).
 * Сюди прилітають строки типу:
 *  "readtds", "TDSB123", "pm1", "pomp", "140", "flow" і т.д.
 */
void legacy_handle_text(const char *msg);

/**
 * Опціонально: чи це “сенсорне” повідомлення типу TDS/температура.
 * Може здатися корисним, коли будеш будувати логіку шлюза.
 */
bool legacy_is_sensor_value(const char *msg);

#ifdef __cplusplus
}
#endif
