#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	gpio_num_t level_a_gpio;	// електрод A (GPIO32)
	gpio_num_t level_b_gpio;	// електрод B (GPIO33)
	gpio_num_t pump_gpio;		// керування помпою (GPIO26)
} pump_node_pins_t;

/**
 * Ініціалізація вузла помпи + ADC.
 * Викликати ОДИН раз з app_main.
 */
esp_err_t pump_node_init(const pump_node_pins_t *pins);

/**
 * Запуск таски автополиву.
 * prio – пріоритет (наприклад 5).
 */
void pump_node_start_task(UBaseType_t prio);

/** Останній рівень (0 або 100) */
int pump_node_get_last_level_percent(void);

/** true, якщо помпа зараз увімкнена */
bool pump_node_is_pump_on(void);

#ifdef __cplusplus
}
#endif
