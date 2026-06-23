#pragma once

#include <stdbool.h>
#include <stdint.h>

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
void mesh_log_stream_kick_nodeinfo_burst(void);
esp_err_t mesh_log_stream_last_send_err(void);
/* Legacy name: this is local TX-accepted age, not proof that root received it. */
uint32_t mesh_log_stream_root_ok_age_ms(void);
bool mesh_log_stream_root_ok_fresh(uint32_t max_age_ms);
void mesh_log_stream_clear_root_ok(void);
bool mesh_log_stream_enabled(void);

// Call from mesh_rx_task() when MESH_LOG_TYPE_CTRL is received.
esp_err_t mesh_log_stream_handle_rx(const void *pkt_buf, size_t pkt_len);

#ifdef __cplusplus
}
#endif
