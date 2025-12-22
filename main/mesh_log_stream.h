#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Викликати 1 раз на старті (після log_time_vprintf_start())
void mesh_log_stream_init(const char *tag);

// Викликати коли нода реально підключилась до mesh (PARENT_CONNECTED)
void mesh_log_stream_on_mesh_connected(void);

// Викликати з mesh_rx_task() коли прийшов пакет типу MESH_LOG_TYPE_CTRL
esp_err_t mesh_log_stream_handle_rx(const void *pkt_buf, size_t pkt_len);

#ifdef __cplusplus
}
#endif
