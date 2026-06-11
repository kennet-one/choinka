#pragma once

#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mesh_ota_receiver_handle_rx(const void *pkt_buf, size_t pkt_len);

#ifdef __cplusplus
}
#endif
