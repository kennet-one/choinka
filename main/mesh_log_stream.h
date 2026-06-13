#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Call once during startup, after log_time_vprintf_start().
void mesh_log_stream_init(const char *tag);

// Call when the node is actually connected to mesh (PARENT_CONNECTED).
void mesh_log_stream_on_mesh_connected(void);
void mesh_log_stream_on_mesh_disconnected(void);

// Best-effort immediate NODEINFO beacon used by recovery watchdogs.
esp_err_t mesh_log_stream_send_nodeinfo_now(void);
esp_err_t mesh_log_stream_last_send_err(void);

// Call from mesh_rx_task() when MESH_LOG_TYPE_CTRL is received.
esp_err_t mesh_log_stream_handle_rx(const void *pkt_buf, size_t pkt_len);

#ifdef __cplusplus
}
#endif
