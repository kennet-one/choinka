#include "mesh_v2_link.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "keemash_mesh_hooks.h"
#include "keemash_mesh_tx_broker.h"
#include "legacy_proto.h"
#include "mesh_log_stream.h"
#include "mesh_time_sync.h"
#include "pump_node.h"

static uint32_t s_status_command_exec_count = 0;
static keemash_mesh_tx_broker_t *s_tx_broker;
static bool mac_is_zero(const uint8_t mac[6]);
esp_err_t mesh_manual_reboot_schedule(uint16_t delay_ms, const char *reason);

static uint8_t tx_priority(const void *packet, size_t packet_len)
{
	if (packet_len < sizeof(mesh_v2_hdr_t)) return KEEMASH_REL_PRIORITY_NORMAL;
	const mesh_v2_hdr_t *h = packet;
	if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION_V2)
		return KEEMASH_REL_PRIORITY_NORMAL;
	if (h->type != MESH_V2_TYPE_RELIABLE_DATA ||
	    h->payload_len < sizeof(mesh_v2_reliable_hdr_t))
		return KEEMASH_REL_PRIORITY_CONTROL;
	const mesh_v2_reliable_hdr_t *rh =
		(const mesh_v2_reliable_hdr_t *)((const uint8_t *)packet + sizeof(*h));
	return rh->priority;
}

static esp_err_t raw_mesh_send(void *user, const uint8_t dst[6],
			       const void *packet, size_t packet_len)
{
	(void)user;
	if (!mesh_log_stream_transport_ready()) return ESP_ERR_INVALID_STATE;
	mesh_addr_t dest = {0};
	if (!mac_is_zero(dst)) memcpy(dest.addr, dst, 6);
	mesh_data_t data = {
		.data = (uint8_t *)packet,
		.size = packet_len,
		.proto = MESH_PROTO_BIN,
		.tos = MESH_TOS_P2P,
	};
	return esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

void mesh_v2_link_require(void)
{
	if (s_tx_broker) return;
	keemash_mesh_tx_broker_config_t cfg = {
		.slots = 24,
		.max_packet_size = 512,
		.task_stack_words = 4096,
		.task_priority = 7,
		.task_name = "mesh_tx",
		.raw_send = raw_mesh_send,
	};
	ESP_ERROR_CHECK(keemash_mesh_tx_broker_init(&s_tx_broker, &cfg));
}

static bool mac_is_zero(const uint8_t mac[6])
{
	static const uint8_t zero[6] = {0};
	return !mac || memcmp(mac, zero, sizeof(zero)) == 0;
}

esp_err_t keemash_mesh_transport_send(const uint8_t dst[6], const void *packet, size_t packet_len)
{
	if (!packet || packet_len == 0) return ESP_ERR_INVALID_ARG;
	if (!s_tx_broker) return ESP_ERR_INVALID_STATE;
	return keemash_mesh_tx_broker_submit(s_tx_broker, dst, packet, packet_len,
					     tx_priority(packet, packet_len));
}

esp_err_t mesh_v2_link_send(const uint8_t dst[6], const void *packet,
			    size_t packet_len, uint8_t priority)
{
	if (!packet || packet_len == 0) return ESP_ERR_INVALID_ARG;
	if (!s_tx_broker) return ESP_ERR_INVALID_STATE;
	return keemash_mesh_tx_broker_submit(s_tx_broker, dst, packet, packet_len,
					     priority);
}

void keemash_mesh_get_local_mac(uint8_t mac[6])
{
	esp_wifi_get_mac(WIFI_IF_STA, mac);
}

bool keemash_mesh_node_on_control_command(const char *text)
{
	return legacy_handle_command(text);
}

bool keemash_mesh_node_on_control_command_result(const char *text, uint8_t *status,
                                                 char *result, size_t result_size)
{
	if (!text || strcmp(text, "choinka.status") != 0) {
		if (text && strcmp(text, "system.reboot") == 0) {
			esp_err_t err = mesh_manual_reboot_schedule(1200, "reliable control");
			if (status) *status = err == ESP_OK ? MESH_V2_CONTROL_STATUS_OK
							 : MESH_V2_CONTROL_STATUS_FAILED;
			if (result && result_size > 0) {
				snprintf(result, result_size, "%s",
				         err == ESP_OK ? "manual reboot accepted" : "manual reboot busy");
				result[result_size - 1] = '\0';
			}
			return true;
		}
		return false;
	}

	pump_node_status_t pump_status = {0};
	if (!pump_node_get_status(&pump_status)) {
		if (status) {
			*status = MESH_V2_CONTROL_STATUS_FAILED;
		}
		if (result && result_size > 0) {
			snprintf(result, result_size, "pump status unavailable");
			result[result_size - 1] = '\0';
		}
		return true;
	}

	s_status_command_exec_count++;
	if (status) {
		*status = MESH_V2_CONTROL_STATUS_OK;
	}
	if (result && result_size > 0) {
		snprintf(result, result_size,
		         "exec=%lu level=%s pump=%u mv=%d/%d cal=%u cd=%lu stop=%s tout=%lu",
		         (unsigned long)s_status_command_exec_count,
		         pump_node_level_name(pump_status.level_state),
		         pump_status.pump_on ? 1U : 0U,
		         pump_status.voltage_ab_mv, pump_status.voltage_ba_mv,
		         pump_status.adc_calibrated ? 1U : 0U,
		         (unsigned long)pump_status.cooldown_remaining_ms,
		         pump_node_stop_reason_name(pump_status.last_stop_reason),
		         (unsigned long)pump_status.timeout_count);
		result[result_size - 1] = '\0';
	}
	return true;
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

void keemash_mesh_node_on_time_sync(const mesh_v2_time_payload_t *time_sync)
{
	(void)mesh_time_sync_apply_v2(time_sync);
}
