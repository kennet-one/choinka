#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mesh_ota_receiver_start(void);
bool mesh_ota_receiver_active(void);
esp_err_t mesh_ota_receiver_handle_rx(const void *pkt_buf, size_t pkt_len);
esp_err_t mesh_ota_receiver_handle_v2(const void *payload, size_t payload_len);

#ifdef __cplusplus
}
#endif
