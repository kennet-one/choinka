#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Starts the periodic stack and CPU monitor task.
esp_err_t stack_monitor_start(UBaseType_t priority);

#ifdef __cplusplus
}
#endif
