#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "keemash_mesh_core.h"

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

typedef struct {
	uint32_t boot_seq;
	uint32_t last_recovery_action_ms;
	int32_t last_mesh_send_err;
	uint32_t ack_stale_count;
	uint32_t tx_without_ack_count;
	uint16_t reset_reason;
	uint16_t parent_disconnect_count;
	uint16_t no_parent_count;
	uint16_t rootless_count;
	uint16_t soft_reconnect_count;
	uint16_t mesh_restart_count;
	uint16_t diag_flags;
	uint8_t last_parent_disconnect_reason;
	uint8_t last_recovery_reason;
} mesh_v2_node_diag_t;

void mesh_v2_node_update_diagnostics(const mesh_v2_node_diag_t *diag);

bool mesh_v2_node_ready(void);
bool mesh_v2_node_ack_fresh(uint32_t max_age_ms);
uint32_t mesh_v2_node_ack_age_ms(void);
void mesh_v2_node_kick_root(void);
esp_err_t mesh_v2_node_force_hello(bool reset_session);
void mesh_v2_node_set_recovery_phase(uint8_t phase);
esp_err_t mesh_v2_node_send_nodeinfo(void);
esp_err_t mesh_v2_node_send_log_line(const char *line);
esp_err_t mesh_v2_node_send_topology(void);
esp_err_t mesh_v2_node_send_memory(void);
esp_err_t mesh_v2_node_send_ota_status(const mesh_v2_ota_status_payload_t *status);
esp_err_t mesh_v2_node_send_event(uint32_t command_id, const char *text);
bool mesh_v2_node_reliable_ready(void);
bool mesh_v2_node_reliable_stats(keemash_rel_stats_t *out);

#ifdef __cplusplus
}
#endif
