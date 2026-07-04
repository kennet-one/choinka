#include "mesh_v2_link.h"

#include <stddef.h>
#include <string.h>

#include "esp_err.h"
#include "esp_mesh.h"
#include "esp_wifi.h"

#include "keemash_mesh_hooks.h"
#include "legacy_proto.h"
#include "mesh_log_stream.h"

esp_err_t keemash_mesh_transport_send(const uint8_t dst[6], const void *packet, size_t packet_len)
{
	if (!packet || packet_len == 0) return ESP_ERR_INVALID_ARG;
	mesh_addr_t dest = {0};
	if (dst) memcpy(dest.addr, dst, 6);
	mesh_data_t data = {
		.data = (uint8_t *)packet,
		.size = packet_len,
		.proto = MESH_PROTO_BIN,
		.tos = MESH_TOS_P2P,
	};
	return esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

void keemash_mesh_get_local_mac(uint8_t mac[6])
{
	esp_wifi_get_mac(WIFI_IF_STA, mac);
}

bool keemash_mesh_node_on_control_command(const char *text)
{
	return legacy_handle_command(text);
}

void keemash_mesh_node_on_log_ctrl(bool enable)
{
	mesh_log_ctrl_packet_t ctrl = {0};
	ctrl.h.magic = MESH_PKT_MAGIC;
	ctrl.h.version = MESH_PKT_VERSION;
	ctrl.h.type = MESH_LOG_TYPE_CTRL;
	ctrl.enable = enable ? 1 : 0;
	mesh_log_stream_handle_rx(&ctrl, sizeof(ctrl));
}

uint32_t keemash_mesh_node_v1_ok_age_ms(void)
{
	return mesh_log_stream_root_ok_age_ms();
}

bool keemash_mesh_node_log_stream_enabled(void)
{
	return mesh_log_stream_enabled();
}
