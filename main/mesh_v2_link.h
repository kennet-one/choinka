#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void mesh_v2_node_init(const char *tag);
void mesh_v2_node_on_mesh_connected(void);
esp_err_t mesh_v2_node_handle_rx(const void *pkt_buf, size_t pkt_len);

bool mesh_v2_node_ready(void);
esp_err_t mesh_v2_node_send_nodeinfo(void);
esp_err_t mesh_v2_node_send_log_line(const char *line);

#ifdef __cplusplus
}
#endif
