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
void mesh_v2_node_on_mesh_disconnected(void);
esp_err_t mesh_v2_node_handle_rx(const void *pkt_buf, size_t pkt_len);
void mesh_v2_node_update_topology(const uint8_t parent_mac[6],
                                  const uint8_t root_mac[6],
                                  uint16_t layer,
                                  uint16_t max_layer,
                                  int8_t parent_rssi,
                                  uint8_t child_count);

bool mesh_v2_node_ready(void);
bool mesh_v2_node_ack_fresh(uint32_t max_age_ms);
uint32_t mesh_v2_node_ack_age_ms(void);
void mesh_v2_node_kick_root(void);
void mesh_v2_node_set_recovery_phase(uint8_t phase);
esp_err_t mesh_v2_node_send_nodeinfo(void);
esp_err_t mesh_v2_node_send_log_line(const char *line);
esp_err_t mesh_v2_node_send_topology(void);

#ifdef __cplusplus
}
#endif
