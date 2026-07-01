#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_attr.h"
#include "nvs_flash.h"

#include "legacy_proto.h"
#include "pump_node.h"
#include "stack_monitor.h"
#include "log_time_vprintf.h"
#include "mesh_proto.h"
#include "mesh_time_sync.h"
#include "mesh_log_stream.h"
#include "mesh_ota_receiver.h"
#include "mesh_v2_link.h"

/* -------------------------------------------------------------------------- */
/*  Constants / globals                                                       */
/* -------------------------------------------------------------------------- */

#define RX_SIZE          (256)
#define TX_INTERVAL_MS   (5000)
#define MESH_RECONNECT_CHECK_MS 5000U
#define MESH_RECONNECT_SOFT_MS  20000U
#define MESH_RECONNECT_HARD_MS  60000U
#define ROOT_ACK_FRESH_MS       30000U
#define ROOT_RECOVERY_BURST_MS  5000U
#define ROOT_RECOVERY_RESET_MS  15000U
#define ROOT_RECOVERY_SOFT_MS   20000U
#define ROOT_RECOVERY_HARD_MS   45000U
#define ROOT_RECOVERY_LOG_MS    15000U
#define MESH_SERVICE_STOP_RETRY_MS   10000U
#define MESH_SERVICE_STOP_TIMEOUT_MS 30000U
#define MANUAL_REBOOT_DELAY_MIN_MS 500U
#define MANUAL_REBOOT_DELAY_MAX_MS 5000U
#define MANUAL_REBOOT_DELAY_DEFAULT_MS 1200U
#define DIAG_RTC_MAGIC          0x43484f31U

#ifndef CONFIG_CHOINKA_DIRECT_ROOT_ONLY
#define CONFIG_CHOINKA_DIRECT_ROOT_ONLY 0
#endif

#ifndef CONFIG_CHOINKA_DIRECT_MAX_LAYER
#define CONFIG_CHOINKA_DIRECT_MAX_LAYER 2
#endif

static const char *MESH_TAG = "choinka";

/* Same MESH_ID on every node in this mesh network. */
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };

static bool       is_running        = true;
static volatile bool is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static uint8_t mesh_root_addr[6] = {0};
static int        mesh_layer        = -1;
static uint8_t    mesh_max_layer    = 0;
static uint8_t    mesh_child_count  = 0;
static esp_netif_t *netif_sta       = NULL;
static uint32_t   s_disconnected_since_ms = 0;
static uint32_t   s_last_reconnect_attempt_ms = 0;
static uint32_t   s_last_mesh_restart_ms = 0;
static TaskHandle_t s_mesh_reconnect_task = NULL;
static volatile bool s_mesh_service_started = false;

typedef enum {
	MESH_SERVICE_RECOVERY_IDLE = 0,
	MESH_SERVICE_RECOVERY_STOPPING,
	MESH_SERVICE_RECOVERY_STARTING,
} mesh_service_recovery_t;

static volatile mesh_service_recovery_t s_mesh_service_recovery =
	MESH_SERVICE_RECOVERY_IDLE;
static uint32_t s_mesh_service_action_ms = 0;
static uint32_t s_mesh_service_stop_started_ms = 0;

typedef enum {
	ROOT_RECOVERY_OK = 0,
	ROOT_RECOVERY_WAIT_ACK,
	ROOT_RECOVERY_NODEINFO_BURST,
	ROOT_RECOVERY_MESH_RECONNECT,
	ROOT_RECOVERY_MESH_RESTART,
} root_recovery_phase_t;

static root_recovery_phase_t s_root_recovery_phase = ROOT_RECOVERY_OK;
static uint32_t s_root_unhealthy_since_ms = 0;
static uint32_t s_last_root_burst_ms = 0;
static uint32_t s_last_root_soft_reconnect_ms = 0;
static uint32_t s_last_root_hard_restart_ms = 0;
static uint32_t s_last_root_recovery_log_ms = 0;
static bool s_manual_reboot_pending = false;
static uint16_t s_manual_reboot_delay_ms = MANUAL_REBOOT_DELAY_DEFAULT_MS;

typedef struct {
	uint32_t magic;
	uint32_t boot_seq;
} diag_rtc_state_t;

static RTC_NOINIT_ATTR diag_rtc_state_t s_diag_rtc;
static uint32_t s_boot_seq = 0;
static uint16_t s_reset_reason = 0;
static uint32_t s_parent_disconnect_count = 0;
static uint32_t s_no_parent_count = 0;
static uint32_t s_rootless_count = 0;
static uint32_t s_soft_reconnect_count = 0;
static uint32_t s_mesh_restart_count = 0;
static uint32_t s_ack_stale_count = 0;
static uint32_t s_tx_without_ack_count = 0;
static uint8_t s_last_parent_disconnect_reason = 0;
static uint8_t s_last_recovery_reason = MESH_V2_RECOVERY_REASON_NONE;
static int32_t s_last_mesh_send_err = ESP_OK;
static uint32_t s_last_recovery_action_ms = 0;

/* -------------------------------------------------------------------------- */
/*  Minimal local protocol                                                    */
/* -------------------------------------------------------------------------- */
/*
 * Packet format:
 *  magic    - 0xA5, identifies our packet
 *  version  - protocol version
 *  type     - packet type
 *  reserved - alignment / future use
 *  counter  - per-node packet counter
 *  src_mac  - sender MAC
 *  payload  - small zero-terminated text payload
 */

/* -------------------------------------------------------------------------- */
/*  Prototypes                                                                */
/* -------------------------------------------------------------------------- */

static void mesh_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data);

static void mesh_rx_task(void *arg);
static void mesh_reconnect_watchdog_task(void *arg);
static esp_err_t mesh_comm_start(void);
static esp_err_t mesh_service_init_and_start(void);
static esp_err_t mesh_service_request_restart(uint32_t now);
static bool mesh_service_recovery_step(uint32_t now);

static uint32_t tick_ms(void)
{
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static uint16_t diag_sat_u16(uint32_t v)
{
	return v > UINT16_MAX ? UINT16_MAX : (uint16_t)v;
}

static void diag_boot_init(void)
{
	if (s_diag_rtc.magic != DIAG_RTC_MAGIC) {
		s_diag_rtc.magic = DIAG_RTC_MAGIC;
		s_diag_rtc.boot_seq = 0;
	}
	s_diag_rtc.boot_seq++;
	s_boot_seq = s_diag_rtc.boot_seq;
	s_reset_reason = (uint16_t)esp_reset_reason();
	ESP_LOGI(MESH_TAG, "boot diag: seq=%lu reset_reason=%u",
	         (unsigned long)s_boot_seq,
	         (unsigned)s_reset_reason);
}

static void diag_note_send_err(esp_err_t err)
{
	s_last_mesh_send_err = (int32_t)err;
}

static void diag_note_recovery_action(uint32_t now, esp_err_t err)
{
	s_last_recovery_action_ms = now;
	diag_note_send_err(err);
}

static void diag_note_recovery_reason(uint32_t now, uint8_t reason)
{
	s_last_recovery_reason = reason;
	s_last_recovery_action_ms = now;
}

static bool root_ack_health_fresh(void)
{
	return mesh_v2_node_ack_fresh(ROOT_ACK_FRESH_MS);
}

static void diag_note_ack_stale(uint32_t now)
{
	s_ack_stale_count++;
	if (mesh_log_stream_last_send_err() == ESP_OK &&
	    mesh_log_stream_root_ok_fresh(ROOT_ACK_FRESH_MS)) {
		s_tx_without_ack_count++;
		diag_note_recovery_reason(now, MESH_V2_RECOVERY_REASON_TX_NO_ACK);
	} else {
		diag_note_recovery_reason(now, MESH_V2_RECOVERY_REASON_ACK_STALE);
	}
}

static void sync_v2_diagnostics(void)
{
	bool ack_health = root_ack_health_fresh();
	bool tx_without_ack = !ack_health &&
	                       mesh_log_stream_last_send_err() == ESP_OK &&
	                       mesh_log_stream_root_ok_fresh(ROOT_ACK_FRESH_MS);
	mesh_v2_node_diag_t diag = {
		.boot_seq = s_boot_seq,
		.last_recovery_action_ms = s_last_recovery_action_ms,
		.last_mesh_send_err = s_last_mesh_send_err,
		.ack_stale_count = s_ack_stale_count,
		.tx_without_ack_count = s_tx_without_ack_count,
		.reset_reason = s_reset_reason,
		.parent_disconnect_count = diag_sat_u16(s_parent_disconnect_count),
		.no_parent_count = diag_sat_u16(s_no_parent_count),
		.rootless_count = diag_sat_u16(s_rootless_count),
		.soft_reconnect_count = diag_sat_u16(s_soft_reconnect_count),
		.mesh_restart_count = diag_sat_u16(s_mesh_restart_count),
		.diag_flags = (ack_health ? MESH_V2_TOPO_DIAG_ACK_HEALTH : 0) |
		              (tx_without_ack ? MESH_V2_TOPO_DIAG_TX_WITHOUT_ACK : 0),
		.last_parent_disconnect_reason = s_last_parent_disconnect_reason,
		.last_recovery_reason = s_last_recovery_reason,
	};
	esp_err_t log_err = mesh_log_stream_last_send_err();
	if (log_err != ESP_OK) {
		diag.last_mesh_send_err = (int32_t)log_err;
	}
	mesh_v2_node_update_diagnostics(&diag);
}

static int8_t current_parent_rssi(void)
{
	wifi_ap_record_t ap;
	if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
		return ap.rssi;
	}
	return -127;
}

static esp_err_t copy_mesh_config_string(uint8_t *dst, size_t dst_sz,
                                         const char *src, size_t *out_len,
                                         const char *label)
{
	if (!dst || dst_sz == 0 || !src) {
		ESP_LOGE(MESH_TAG, "%s config string is invalid", label ? label : "mesh");
		return ESP_ERR_INVALID_ARG;
	}

	size_t len = strnlen(src, dst_sz + 1);
	if (len > dst_sz) {
		ESP_LOGE(MESH_TAG, "%s too long: %u > %u bytes",
		         label ? label : "mesh",
		         (unsigned)len,
		         (unsigned)dst_sz);
		return ESP_ERR_INVALID_SIZE;
	}

	memset(dst, 0, dst_sz);
	memcpy(dst, src, len);
	if (out_len) {
		*out_len = len;
	}
	return ESP_OK;
}

static void update_v2_topology(bool send_now)
{
	sync_v2_diagnostics();
	mesh_v2_node_update_topology(mesh_parent_addr.addr,
	                             mesh_root_addr,
	                             (mesh_layer > 0) ? (uint16_t)mesh_layer : 0,
	                             mesh_max_layer,
	                             current_parent_rssi(),
	                             mesh_child_count);
	if (send_now && is_mesh_connected) {
		esp_err_t err = mesh_v2_node_send_topology();
		diag_note_send_err(err);
	}
}

static void note_mesh_disconnected(void)
{
	if (is_mesh_connected || s_disconnected_since_ms == 0) {
		s_disconnected_since_ms = tick_ms();
	}
	is_mesh_connected = false;
	s_root_recovery_phase = ROOT_RECOVERY_OK;
	mesh_v2_node_set_recovery_phase((uint8_t)ROOT_RECOVERY_OK);
	mesh_log_stream_clear_root_ok();
	s_root_unhealthy_since_ms = 0;
	s_last_root_burst_ms = 0;
}

static void note_mesh_connected(void)
{
	is_mesh_connected = true;
	s_disconnected_since_ms = 0;
	s_last_reconnect_attempt_ms = 0;
	s_root_recovery_phase = ROOT_RECOVERY_WAIT_ACK;
	mesh_v2_node_set_recovery_phase((uint8_t)s_root_recovery_phase);
	s_root_unhealthy_since_ms = tick_ms();
	s_last_root_burst_ms = 0;
	s_last_root_soft_reconnect_ms = 0;
	s_last_root_hard_restart_ms = 0;
	s_last_root_recovery_log_ms = 0;
}

static const char *root_recovery_phase_name(root_recovery_phase_t phase)
{
	switch (phase) {
	case ROOT_RECOVERY_OK: return "ok";
	case ROOT_RECOVERY_WAIT_ACK: return "wait_ack";
	case ROOT_RECOVERY_NODEINFO_BURST: return "nodeinfo_burst";
	case ROOT_RECOVERY_MESH_RECONNECT: return "mesh_reconnect";
	case ROOT_RECOVERY_MESH_RESTART: return "mesh_restart";
	default: return "unknown";
	}
}

static void root_recovery_reset_ok(uint32_t now)
{
	bool was_recovering = s_root_recovery_phase != ROOT_RECOVERY_OK ||
	                      s_root_unhealthy_since_ms != 0;
	if (s_root_recovery_phase != ROOT_RECOVERY_OK && s_root_unhealthy_since_ms != 0) {
		uint32_t down_ms = (uint32_t)(now - s_root_unhealthy_since_ms);
		ESP_LOGI(MESH_TAG,
		         "root recovery: restored after %lu ms, ack_age=%lu tx_age=%lu",
		         (unsigned long)down_ms,
		         (unsigned long)mesh_v2_node_ack_age_ms(),
		         (unsigned long)mesh_log_stream_root_ok_age_ms());
	}
	s_root_recovery_phase = ROOT_RECOVERY_OK;
	mesh_v2_node_set_recovery_phase((uint8_t)ROOT_RECOVERY_OK);
	s_root_unhealthy_since_ms = 0;
	s_last_root_burst_ms = 0;
	s_last_root_soft_reconnect_ms = 0;
	s_last_root_hard_restart_ms = 0;
	s_last_root_recovery_log_ms = 0;
	s_last_recovery_reason = MESH_V2_RECOVERY_REASON_NONE;
	if (was_recovering && is_mesh_connected) {
		update_v2_topology(true);
	}
}

static void root_recovery_log_throttled(uint32_t now, uint32_t down_ms,
                                        root_recovery_phase_t phase,
                                        const char *action,
                                        esp_err_t last_err)
{
	if (s_last_root_recovery_log_ms != 0 &&
	    (uint32_t)(now - s_last_root_recovery_log_ms) < ROOT_RECOVERY_LOG_MS) {
		return;
	}
	s_last_root_recovery_log_ms = now;
	mesh_v2_node_set_recovery_phase((uint8_t)phase);
	ESP_LOGW(MESH_TAG,
	         "root recovery: phase=%s down=%lu ms ack_age=%lu tx_age=%lu last_send=%s action=%s",
	         root_recovery_phase_name(phase),
	         (unsigned long)down_ms,
	         (unsigned long)mesh_v2_node_ack_age_ms(),
	         (unsigned long)mesh_log_stream_root_ok_age_ms(),
	         esp_err_to_name(last_err),
	         action ? action : "watch");
}

static void ota_mark_running_app_valid_if_pending(void)
{
	const esp_partition_t *running = esp_ota_get_running_partition();
	esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;

	if (!running) return;
	if (esp_ota_get_state_partition(running, &state) != ESP_OK) return;
	if (state != ESP_OTA_IMG_PENDING_VERIFY) return;

	esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
	if (err == ESP_OK) {
		ESP_LOGI(MESH_TAG, "OTA rollback: running app marked valid");
	} else {
		ESP_LOGE(MESH_TAG, "OTA rollback: mark valid failed: %s", esp_err_to_name(err));
	}
}

static esp_err_t send_reboot_status(uint16_t seq, uint8_t code, const char *msg)
{
	mesh_reboot_status_packet_t p;
	memset(&p, 0, sizeof(p));

	p.h.magic = MESH_PKT_MAGIC;
	p.h.version = MESH_PKT_VERSION;
	p.h.type = MESH_REBOOT_TYPE_STATUS;
	p.h.counter = tick_ms();
	esp_wifi_get_mac(WIFI_IF_STA, p.h.src_mac);
	p.code = code;
	p.seq = seq;
	if (msg) {
		strncpy(p.message, msg, sizeof(p.message) - 1);
		p.message[sizeof(p.message) - 1] = '\0';
	}

	mesh_data_t data;
	memset(&data, 0, sizeof(data));
	data.data = (uint8_t *)&p;
	data.size = sizeof(p);
	data.proto = MESH_PROTO_BIN;
	data.tos = MESH_TOS_P2P;

	mesh_addr_t dest;
	memset(&dest, 0, sizeof(dest));

	return esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

static void manual_reboot_task(void *arg)
{
	(void)arg;
	uint16_t delay_ms = s_manual_reboot_delay_ms;
	if (delay_ms < MANUAL_REBOOT_DELAY_MIN_MS ||
	    delay_ms > MANUAL_REBOOT_DELAY_MAX_MS) {
		delay_ms = MANUAL_REBOOT_DELAY_DEFAULT_MS;
	}

	vTaskDelay(pdMS_TO_TICKS(delay_ms));
	esp_restart();
}

static void handle_reboot_request(const mesh_reboot_request_packet_t *p,
                                  size_t pkt_len)
{
	if (!p || pkt_len < sizeof(*p)) {
		ESP_LOGW(MESH_TAG, "RX REBOOT short: %u bytes", (unsigned)pkt_len);
		return;
	}

	char reason[MESH_REBOOT_REASON_MAX + 1];
	memcpy(reason, p->reason, sizeof(p->reason));
	reason[sizeof(reason) - 1] = '\0';

	if (s_manual_reboot_pending) {
		(void)send_reboot_status(p->seq, MESH_REBOOT_STATUS_BUSY,
		                         "manual reboot already pending");
		return;
	}

	uint16_t delay_ms = p->delay_ms;
	if (delay_ms < MANUAL_REBOOT_DELAY_MIN_MS ||
	    delay_ms > MANUAL_REBOOT_DELAY_MAX_MS) {
		delay_ms = MANUAL_REBOOT_DELAY_DEFAULT_MS;
	}

	s_manual_reboot_pending = true;
	s_manual_reboot_delay_ms = delay_ms;

	BaseType_t task_ok = xTaskCreate(manual_reboot_task, "manual_reboot",
	                                 2048, NULL, 6, NULL);
	if (task_ok != pdPASS) {
		s_manual_reboot_pending = false;
		(void)send_reboot_status(p->seq, MESH_REBOOT_STATUS_ERROR,
		                         "reboot task failed");
		return;
	}

	ESP_LOGW(MESH_TAG, "manual reboot requested: seq=%u delay=%u reason=%s",
	         (unsigned)p->seq, (unsigned)delay_ms,
	         reason[0] ? reason : "none");
	(void)send_reboot_status(p->seq, MESH_REBOOT_STATUS_OK,
	                         "manual reboot accepted");
}

/* -------------------------------------------------------------------------- */
/*  RX task: receive packets from other nodes                                  */
/* -------------------------------------------------------------------------- */

static void mesh_rx_task(void *arg)
{
	uint8_t		rx_buf[256];

	mesh_data_t	data = {
		.data	= rx_buf,
		.size	= sizeof(rx_buf),
	};

	mesh_addr_t	from;
	int		flag = 0;
	esp_err_t	err;
	uint32_t last_error_log_ms = 0;

	while (is_running) {
		data.size = sizeof(rx_buf);

		err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
		if (err != ESP_OK) {
			uint32_t now = tick_ms();
			if (last_error_log_ms == 0 ||
			    (uint32_t)(now - last_error_log_ms) >= 5000U) {
				last_error_log_ms = now;
				ESP_LOGE(MESH_TAG, "esp_mesh_recv failed: 0x%x (%s)",
					 err, esp_err_to_name(err));
			}
			vTaskDelay(pdMS_TO_TICKS(250));
			continue;
		}
		last_error_log_ms = 0;

		if (data.size < sizeof(mesh_pkt_hdr_t)) {
			ESP_LOGW(MESH_TAG, "RX too short: %d bytes", (int)data.size);
			continue;
		}

		const mesh_pkt_hdr_t *h = (const mesh_pkt_hdr_t *)rx_buf;

		if (h->magic == MESH_PKT_MAGIC && h->version == MESH_PKT_VERSION_V2) {
			mesh_v2_node_handle_rx(rx_buf, data.size);
			continue;
		}

		// Not our protocol; ignore it.
		if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION) {
			ESP_LOGW(MESH_TAG, "RX unknown packet from " MACSTR " (%d bytes)", MAC2STR(from.addr), (int)data.size);
			continue;
		}

		// Packet type dispatcher.
		if (h->type == MESH_TIME_SYNC_TYPE_TIME) {
			mesh_time_sync_handle_rx(rx_buf, data.size);
			continue;
		}

		if (h->type == MESH_LOG_TYPE_CTRL) {
			mesh_log_stream_handle_rx(rx_buf, data.size);
			continue;
		}

		if (h->type == MESH_REBOOT_TYPE_REQUEST) {
			handle_reboot_request((const mesh_reboot_request_packet_t *)rx_buf,
			                      data.size);
			continue;
		}

		if (h->type == MESH_OTA_TYPE_BEGIN ||
		    h->type == MESH_OTA_TYPE_DATA ||
		    h->type == MESH_OTA_TYPE_END ||
		    h->type == MESH_OTA_TYPE_ABORT) {
			mesh_ota_receiver_handle_rx(rx_buf, data.size);
			continue;
		}

		if (h->type == MESH_PKT_TYPE_TEXT) {
			if (data.size < sizeof(mesh_packet_t)) {
				ESP_LOGW(MESH_TAG, "RX TEXT short: %d bytes", (int)data.size);
				continue;
			}

			const mesh_packet_t *p = (const mesh_packet_t *)rx_buf;

			// Guarantee trailing NUL.
			char payload[sizeof(p->payload)];
			memcpy(payload, p->payload, sizeof(payload));
			payload[sizeof(payload) - 1] = '\0';

			ESP_LOGI(MESH_TAG, "RX TEXT from " MACSTR " (src=" MACSTR "): \"%s\"",
				MAC2STR(from.addr),
				MAC2STR(p->src_mac),
				payload
			);

			// Replace this call if the legacy text handler changes.
			legacy_handle_text(payload);

			continue;
		}

		ESP_LOGI(MESH_TAG, "RX type=%u from " MACSTR " (%d bytes)",
			(unsigned)h->type, MAC2STR(from.addr), (int)data.size);
	}

	vTaskDelete(NULL);
}


/* -------------------------------------------------------------------------- */
/*  Start TX/RX tasks once                                                     */
/* -------------------------------------------------------------------------- */

static esp_err_t mesh_comm_start(void)
{
	static bool started = false;

	if (!started) {
		started = true;
		xTaskCreate(mesh_rx_task, "mesh_rx", 4096, NULL, 5, NULL);
		stack_monitor_start(3);
	}
	return ESP_OK;
}

static esp_err_t mesh_service_init_and_start(void)
{
	esp_err_t err = esp_mesh_init();
	if (err != ESP_OK) return err;

	err = esp_mesh_fix_root(false);
	if (err != ESP_OK) return err;
	err = esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY);
	if (err != ESP_OK) return err;

	mesh_max_layer = CONFIG_MESH_MAX_LAYER;
#if CONFIG_CHOINKA_DIRECT_ROOT_ONLY
	if (mesh_max_layer > CONFIG_CHOINKA_DIRECT_MAX_LAYER) {
		mesh_max_layer = CONFIG_CHOINKA_DIRECT_MAX_LAYER;
	}
#endif
	err = esp_mesh_set_max_layer(mesh_max_layer);
	if (err != ESP_OK) return err;
	err = esp_mesh_set_vote_percentage(1);
	if (err != ESP_OK) return err;
	err = esp_mesh_set_xon_qsize(128);
	if (err != ESP_OK) return err;
	err = esp_mesh_disable_ps();
	if (err != ESP_OK) return err;
	err = esp_mesh_set_ap_assoc_expire(10);
	if (err != ESP_OK) return err;

	mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
	memcpy(cfg.mesh_id.addr, MESH_ID, sizeof(MESH_ID));
	cfg.channel = CONFIG_MESH_CHANNEL;
	size_t router_ssid_len = 0;
	err = copy_mesh_config_string(cfg.router.ssid, sizeof(cfg.router.ssid),
				      CONFIG_MESH_ROUTER_SSID, &router_ssid_len,
				      "router ssid");
	if (err != ESP_OK) return err;
	cfg.router.ssid_len = (uint8_t)router_ssid_len;
	err = copy_mesh_config_string(cfg.router.password,
				      sizeof(cfg.router.password),
				      CONFIG_MESH_ROUTER_PASSWD, NULL,
				      "router password");
	if (err != ESP_OK) return err;

	err = esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE);
	if (err != ESP_OK) return err;
	cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
	cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
	err = copy_mesh_config_string(cfg.mesh_ap.password,
				      sizeof(cfg.mesh_ap.password),
				      CONFIG_MESH_AP_PASSWD, NULL,
				      "mesh ap password");
	if (err != ESP_OK) return err;
	err = esp_mesh_set_config(&cfg);
	if (err != ESP_OK) return err;

	ESP_LOGI(MESH_TAG,
		 "mesh service configured max_layer:%u direct_root_only:%u",
		 (unsigned)mesh_max_layer,
		 (unsigned)CONFIG_CHOINKA_DIRECT_ROOT_ONLY);
	return esp_mesh_start();
}

static esp_err_t mesh_service_request_restart(uint32_t now)
{
	if (s_mesh_service_recovery != MESH_SERVICE_RECOVERY_IDLE) {
		return ESP_OK;
	}

	s_mesh_service_action_ms = now;
	if (!s_mesh_service_started) {
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
		return ESP_OK;
	}

	s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STOPPING;
	s_mesh_service_stop_started_ms = now;
	esp_err_t err = esp_mesh_stop();
	diag_note_send_err(err);
	if (err != ESP_OK) {
		ESP_LOGW(MESH_TAG, "mesh service stop request failed: %s",
			 esp_err_to_name(err));
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
		s_mesh_service_action_ms = 0;
		s_mesh_service_stop_started_ms = 0;
	}
	return err;
}

static bool mesh_service_recovery_step(uint32_t now)
{
	mesh_service_recovery_t state = s_mesh_service_recovery;
	if (state == MESH_SERVICE_RECOVERY_IDLE) {
		if (s_mesh_service_started) return false;
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
		s_mesh_service_action_ms = 0;
		state = MESH_SERVICE_RECOVERY_STARTING;
	}

	if (state == MESH_SERVICE_RECOVERY_STOPPING) {
		if (!s_mesh_service_started) {
			s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
			s_mesh_service_action_ms = 0;
			s_mesh_service_stop_started_ms = 0;
		} else if (s_mesh_service_stop_started_ms != 0 &&
		           (uint32_t)(now - s_mesh_service_stop_started_ms) >=
		           MESH_SERVICE_STOP_TIMEOUT_MS) {
			ESP_LOGW(MESH_TAG,
			         "mesh service stop timeout after %lu ms; forcing local restart state",
			         (unsigned long)(uint32_t)(now - s_mesh_service_stop_started_ms));
			mesh_v2_node_on_mesh_disconnected();
			mesh_log_stream_on_mesh_disconnected();
			note_mesh_disconnected();
			s_mesh_service_started = false;
			s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
			s_mesh_service_action_ms = 0;
			s_mesh_service_stop_started_ms = 0;
		} else if ((uint32_t)(now - s_mesh_service_action_ms) >=
		           MESH_SERVICE_STOP_RETRY_MS) {
			s_mesh_service_action_ms = now;
			esp_err_t err = esp_mesh_stop();
			diag_note_send_err(err);
			if (err != ESP_OK) {
				ESP_LOGW(MESH_TAG, "mesh service stop retry failed: %s",
					 esp_err_to_name(err));
				if (err == ESP_ERR_MESH_NOT_START ||
				    err == ESP_ERR_INVALID_STATE) {
					s_mesh_service_started = false;
					s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
					s_mesh_service_action_ms = 0;
					s_mesh_service_stop_started_ms = 0;
				}
			}
		}
		return true;
	}

	if (s_mesh_service_started) {
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_IDLE;
		s_mesh_service_action_ms = 0;
		return false;
	}
	if (s_mesh_service_action_ms == 0 ||
	    (uint32_t)(now - s_mesh_service_action_ms) >= 5000U) {
		s_mesh_service_action_ms = now;
		esp_err_t err = esp_mesh_start();
		if (err == ESP_ERR_MESH_NOT_INIT) {
			err = mesh_service_init_and_start();
		}
		diag_note_send_err(err);
		s_mesh_service_stop_started_ms = 0;
		if (err != ESP_OK) {
			ESP_LOGW(MESH_TAG, "mesh service start retry failed: %s",
				 esp_err_to_name(err));
		}
	}
	return true;
}

static void root_liveness_watchdog_step(void)
{
	uint32_t now = tick_ms();

	if (!is_mesh_connected) {
		return;
	}

	if (root_ack_health_fresh()) {
		root_recovery_reset_ok(now);
		return;
	}

	if (s_root_unhealthy_since_ms == 0) {
		s_root_unhealthy_since_ms = now;
		diag_note_ack_stale(now);
	}

	uint32_t down_ms = (uint32_t)(now - s_root_unhealthy_since_ms);
	root_recovery_phase_t phase = ROOT_RECOVERY_WAIT_ACK;
	bool ota_active = mesh_ota_receiver_active();

	if (!ota_active && down_ms >= ROOT_RECOVERY_HARD_MS) {
		phase = ROOT_RECOVERY_MESH_RESTART;
	} else if (!ota_active && down_ms >= ROOT_RECOVERY_SOFT_MS) {
		phase = ROOT_RECOVERY_MESH_RECONNECT;
	} else if (down_ms >= ROOT_RECOVERY_BURST_MS) {
		phase = ROOT_RECOVERY_NODEINFO_BURST;
	}
	s_root_recovery_phase = phase;
	mesh_v2_node_set_recovery_phase((uint8_t)phase);

	if (s_last_root_burst_ms == 0 ||
	    (uint32_t)(now - s_last_root_burst_ms) >= ROOT_RECOVERY_BURST_MS) {
		s_last_root_burst_ms = now;
		bool reset_v2_session = !ota_active && down_ms >= ROOT_RECOVERY_RESET_MS;
		esp_err_t hello_err = mesh_v2_node_force_hello(reset_v2_session);
		diag_note_send_err(hello_err);
		update_v2_topology(false);
		esp_err_t err = mesh_log_stream_send_nodeinfo_now();
		diag_note_send_err(err);
		sync_v2_diagnostics();
		esp_err_t topo_err = mesh_v2_node_send_topology();
		diag_note_send_err(topo_err);
		root_recovery_log_throttled(now, down_ms, phase,
		                            reset_v2_session
		                            ? "hello_reset_nodeinfo_burst"
		                            : "nodeinfo_burst",
		                            hello_err != ESP_OK ? hello_err :
		                            err != ESP_OK ? err : topo_err);
	}

	if (!ota_active &&
	    down_ms >= ROOT_RECOVERY_HARD_MS &&
	    (s_last_root_hard_restart_ms == 0 ||
	     (uint32_t)(now - s_last_root_hard_restart_ms) >= ROOT_RECOVERY_HARD_MS)) {
		s_last_root_hard_restart_ms = now;
		s_mesh_restart_count++;
		s_last_recovery_reason = MESH_V2_RECOVERY_REASON_MESH_RESTART;
		diag_note_recovery_action(now, ESP_OK);
		sync_v2_diagnostics();
		root_recovery_log_throttled(now, down_ms, ROOT_RECOVERY_MESH_RESTART,
		                            "mesh_stop_start",
		                            mesh_log_stream_last_send_err());
		mesh_v2_node_on_mesh_disconnected();
		mesh_log_stream_on_mesh_disconnected();
		note_mesh_disconnected();
		(void)mesh_service_request_restart(now);
		return;
	}

	if (!ota_active &&
	    down_ms >= ROOT_RECOVERY_SOFT_MS &&
	    (s_last_root_soft_reconnect_ms == 0 ||
	     (uint32_t)(now - s_last_root_soft_reconnect_ms) >= ROOT_RECOVERY_SOFT_MS)) {
		s_last_root_soft_reconnect_ms = now;
		s_soft_reconnect_count++;
		s_last_recovery_reason = MESH_V2_RECOVERY_REASON_SOFT_RECONNECT;
		diag_note_recovery_action(now, ESP_OK);
		sync_v2_diagnostics();
		root_recovery_log_throttled(now, down_ms, ROOT_RECOVERY_MESH_RECONNECT,
		                            "mesh_disconnect_connect",
		                            mesh_log_stream_last_send_err());
		esp_err_t disc_err = esp_mesh_disconnect();
		diag_note_send_err(disc_err);
		if (disc_err != ESP_OK) {
			ESP_LOGW(MESH_TAG, "root recovery: esp_mesh_disconnect failed: %s",
			         esp_err_to_name(disc_err));
		} else {
			mesh_v2_node_on_mesh_disconnected();
			mesh_log_stream_on_mesh_disconnected();
		}
		vTaskDelay(pdMS_TO_TICKS(250));
		(void)esp_mesh_flush_scan_result();
		esp_err_t conn_err = esp_mesh_connect();
		diag_note_send_err(conn_err);
		if (conn_err != ESP_OK) {
			ESP_LOGW(MESH_TAG, "root recovery: esp_mesh_connect failed: %s",
			         esp_err_to_name(conn_err));
		}
		return;
	}
}

static void mesh_reconnect_watchdog_task(void *arg)
{
	(void)arg;

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(MESH_RECONNECT_CHECK_MS));
		uint32_t now = tick_ms();

		if (mesh_service_recovery_step(now)) {
			continue;
		}

		if (is_mesh_connected) {
			root_liveness_watchdog_step();
			continue;
		}

		if (s_disconnected_since_ms == 0) {
			s_disconnected_since_ms = now;
			continue;
		}

		uint32_t down_ms = (uint32_t)(now - s_disconnected_since_ms);
		if (down_ms >= MESH_RECONNECT_SOFT_MS &&
		    (uint32_t)(now - s_last_reconnect_attempt_ms) >= MESH_RECONNECT_SOFT_MS) {
			s_last_reconnect_attempt_ms = now;
			s_soft_reconnect_count++;
			s_last_recovery_reason = MESH_V2_RECOVERY_REASON_SOFT_RECONNECT;
			diag_note_recovery_action(now, ESP_OK);
			ESP_LOGW(MESH_TAG,
			         "mesh reconnect watchdog: no parent for %lu ms, calling esp_mesh_connect()",
			         (unsigned long)down_ms);
			esp_err_t flush_err = esp_mesh_flush_scan_result();
			diag_note_send_err(flush_err);
			if (flush_err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: scan flush failed: %s",
				         esp_err_to_name(flush_err));
			}
			esp_err_t err = esp_mesh_connect();
			diag_note_send_err(err);
			if (err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: esp_mesh_connect failed: %s",
				         esp_err_to_name(err));
			}
		}

		if (down_ms >= MESH_RECONNECT_HARD_MS &&
		    (uint32_t)(now - s_last_mesh_restart_ms) >= MESH_RECONNECT_HARD_MS) {
			s_last_mesh_restart_ms = now;
			s_last_reconnect_attempt_ms = now;
			s_mesh_restart_count++;
			s_last_recovery_reason = MESH_V2_RECOVERY_REASON_MESH_RESTART;
			diag_note_recovery_action(now, ESP_OK);
			ESP_LOGW(MESH_TAG,
			         "mesh reconnect watchdog: no parent for %lu ms, restarting mesh service",
			         (unsigned long)down_ms);

			mesh_v2_node_on_mesh_disconnected();
			mesh_log_stream_on_mesh_disconnected();
			note_mesh_disconnected();
			(void)mesh_service_request_restart(now);
		}
	}
}

/* -------------------------------------------------------------------------- */
/*  MESH events                                                               */
/* -------------------------------------------------------------------------- */

static void mesh_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
	mesh_addr_t id = {0};
	static uint16_t last_layer = 0;

	switch (event_id) {
	case MESH_EVENT_STARTED: {
		s_mesh_service_started = true;
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_IDLE;
		s_mesh_service_action_ms = 0;
		s_mesh_service_stop_started_ms = 0;
		esp_mesh_get_id(&id);
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_STARTED> ID:" MACSTR,
		         MAC2STR(id.addr));
		note_mesh_disconnected();
		mesh_layer = esp_mesh_get_layer();
	}
	break;

	case MESH_EVENT_STOPPED: {
		s_mesh_service_started = false;
		s_mesh_service_recovery = MESH_SERVICE_RECOVERY_STARTING;
		s_mesh_service_action_ms = 0;
		s_mesh_service_stop_started_ms = 0;
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
		note_mesh_disconnected();
		mesh_layer = esp_mesh_get_layer();
		mesh_v2_node_on_mesh_disconnected();
		mesh_log_stream_on_mesh_disconnected();
	}
	break;

	case MESH_EVENT_CHILD_CONNECTED: {
		mesh_event_child_connected_t *child =
		    (mesh_event_child_connected_t *)event_data;
		if (mesh_child_count < UINT8_MAX) {
			mesh_child_count++;
		}
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_CHILD_CONNECTED> aid:%d, " MACSTR,
		         child->aid, MAC2STR(child->mac));
		update_v2_topology(true);
	}
	break;

	case MESH_EVENT_CHILD_DISCONNECTED: {
		mesh_event_child_disconnected_t *child =
		    (mesh_event_child_disconnected_t *)event_data;
		if (mesh_child_count > 0) {
			mesh_child_count--;
		}
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_CHILD_DISCONNECTED> aid:%d, " MACSTR,
		         child->aid, MAC2STR(child->mac));
		update_v2_topology(true);
	}
	break;

	case MESH_EVENT_ROUTING_TABLE_ADD: {
		mesh_event_routing_table_change_t *rt =
		    (mesh_event_routing_table_change_t *)event_data;
		ESP_LOGW(MESH_TAG,
		         "<MESH_EVENT_ROUTING_TABLE_ADD> add %d, new:%d, layer:%d",
		         rt->rt_size_change, rt->rt_size_new, mesh_layer);
	}
	break;

	case MESH_EVENT_ROUTING_TABLE_REMOVE: {
		mesh_event_routing_table_change_t *rt =
		    (mesh_event_routing_table_change_t *)event_data;
		ESP_LOGW(MESH_TAG,
		         "<MESH_EVENT_ROUTING_TABLE_REMOVE> remove %d, new:%d, layer:%d",
		         rt->rt_size_change, rt->rt_size_new, mesh_layer);
	}
	break;

	case MESH_EVENT_NO_PARENT_FOUND: {
		mesh_event_no_parent_found_t *np =
		    (mesh_event_no_parent_found_t *)event_data;
		uint32_t now = tick_ms();
		s_no_parent_count++;
		diag_note_recovery_reason(now, MESH_V2_RECOVERY_REASON_NO_PARENT);
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_NO_PARENT_FOUND> scan times:%d",
		         np->scan_times);
		note_mesh_disconnected();
		mesh_v2_node_on_mesh_disconnected();
		mesh_log_stream_on_mesh_disconnected();
		if (now > MESH_RECONNECT_SOFT_MS) {
			s_disconnected_since_ms = now - MESH_RECONNECT_SOFT_MS;
		} else {
			s_disconnected_since_ms = 1;
		}
		s_last_reconnect_attempt_ms = 0;
		(void)esp_mesh_flush_scan_result();
	}
	break;

	case MESH_EVENT_PARENT_CONNECTED: {
		mesh_event_connected_t *conn =
		    (mesh_event_connected_t *)event_data;
		esp_mesh_get_id(&id);
		mesh_layer = conn->self_layer;
		memcpy(mesh_parent_addr.addr, conn->connected.bssid, 6);
		note_mesh_connected();
		update_v2_topology(false);

		mesh_v2_node_on_mesh_connected();
		mesh_log_stream_on_mesh_connected();

		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_PARENT_CONNECTED> layer:%d -> %d, parent:" MACSTR
		         " %s, ID:" MACSTR ", duty:%d",
		         last_layer, mesh_layer,
		         MAC2STR(mesh_parent_addr.addr),
		         esp_mesh_is_root() ? "<ROOT>" :
		         (mesh_layer == 2) ? "<layer2>" : "",
		         MAC2STR(id.addr),
		         conn->duty);
		last_layer = mesh_layer;

		if (esp_mesh_is_root()) {
			esp_netif_dhcpc_stop(netif_sta);
			esp_netif_dhcpc_start(netif_sta);
		}
		mesh_comm_start();
	}
	break;

	case MESH_EVENT_PARENT_DISCONNECTED: {
		mesh_event_disconnected_t *disc =
		    (mesh_event_disconnected_t *)event_data;
		s_parent_disconnect_count++;
		s_last_parent_disconnect_reason = disc->reason;
		diag_note_recovery_reason(tick_ms(), MESH_V2_RECOVERY_REASON_PARENT_DISC);
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_PARENT_DISCONNECTED> reason:%d",
		         disc->reason);
		note_mesh_disconnected();
		mesh_layer = esp_mesh_get_layer();
		mesh_v2_node_on_mesh_disconnected();
		mesh_log_stream_on_mesh_disconnected();
		update_v2_topology(false);
	}
	break;

	case MESH_EVENT_LAYER_CHANGE: {
		mesh_event_layer_change_t *lc =
		    (mesh_event_layer_change_t *)event_data;
		mesh_layer = lc->new_layer;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_LAYER_CHANGE> layer:%d -> %d %s",
		         last_layer, mesh_layer,
		         esp_mesh_is_root() ? "<ROOT>" :
		         (mesh_layer == 2) ? "<layer2>" : "");
		last_layer = mesh_layer;
		(void)mesh_v2_node_force_hello(false);
		update_v2_topology(true);
		mesh_log_stream_kick_nodeinfo_burst();
	}
	break;

	case MESH_EVENT_ROOT_ADDRESS: {
		mesh_event_root_address_t *ra =
		    (mesh_event_root_address_t *)event_data;
		memcpy(mesh_root_addr, ra->addr, sizeof(mesh_root_addr));
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_ROOT_ADDRESS> root:" MACSTR,
		         MAC2STR(ra->addr));
		mesh_v2_node_on_mesh_connected();
		update_v2_topology(true);
		(void)mesh_log_stream_send_nodeinfo_now();
		mesh_log_stream_kick_nodeinfo_burst();
	}
	break;

	case MESH_EVENT_TODS_STATE: {
		mesh_event_toDS_state_t *st =
		    (mesh_event_toDS_state_t *)event_data;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_TODS_STATE> state:%d",
		         *st);
	}
	break;

	case MESH_EVENT_NETWORK_STATE: {
		mesh_event_network_state_t *ns =
		    (mesh_event_network_state_t *)event_data;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_NETWORK_STATE> is_rootless:%d",
		         ns->is_rootless);
		if (ns->is_rootless) {
			uint32_t now = tick_ms();
			s_rootless_count++;
			diag_note_recovery_reason(now, MESH_V2_RECOVERY_REASON_ROOTLESS);
			mesh_log_stream_clear_root_ok();
			if (s_root_unhealthy_since_ms == 0) {
				s_root_unhealthy_since_ms = now;
			}
			s_root_recovery_phase = ROOT_RECOVERY_WAIT_ACK;
			mesh_v2_node_set_recovery_phase((uint8_t)s_root_recovery_phase);
			mesh_v2_node_on_mesh_disconnected();
			s_last_root_burst_ms = 0;
			if (is_mesh_connected) {
				(void)mesh_v2_node_force_hello(true);
				update_v2_topology(false);
				(void)mesh_log_stream_send_nodeinfo_now();
				mesh_log_stream_kick_nodeinfo_burst();
			}
			root_recovery_log_throttled(now,
			                            (uint32_t)(now - s_root_unhealthy_since_ms),
			                            s_root_recovery_phase,
			                            "rootless_hello_nodeinfo",
			                            mesh_log_stream_last_send_err());
		}
	}
	break;

	default:
		ESP_LOGI(MESH_TAG,
		         "unknown mesh event id:%" PRId32,
		         event_id);
		break;
	}
}

/* -------------------------------------------------------------------------- */
/*  app_main: mesh + Wi-Fi init                                                */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());
	diag_boot_init();
	ota_mark_running_app_valid_if_pending();
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Create mesh netifs; keep the STA netif handle.
	ESP_ERROR_CHECK(
	    esp_netif_create_default_wifi_mesh_netifs(&netif_sta, NULL));

	// Wi-Fi
	wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
	ESP_ERROR_CHECK(esp_wifi_start());

	// MESH
	ESP_ERROR_CHECK(
	    esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID,
	                               &mesh_event_handler, NULL));
	mesh_time_sync_init();
	log_time_vprintf_start();
	mesh_v2_node_init(MESH_TAG);
	mesh_log_stream_init(MESH_TAG);
	ESP_ERROR_CHECK(mesh_ota_receiver_start());

	ESP_ERROR_CHECK(mesh_service_init_and_start());
	if (!s_mesh_reconnect_task) {
		xTaskCreate(mesh_reconnect_watchdog_task,
		            "mesh_reconn",
		            5120,
		            NULL,
		            4,
		            &s_mesh_reconnect_task);
	}

	ESP_LOGI(MESH_TAG,
	         "mesh started, heap:%" PRId32 ", root_fixed:%d, topo:%d %s, ps:%d",
	         esp_get_minimum_free_heap_size(),
	         esp_mesh_is_root_fixed(),
	         esp_mesh_get_topology(),
	         esp_mesh_get_topology() ? "(chain)" : "(tree)",
	         esp_mesh_is_ps_enabled());

		// -------- Pump node wiring --------
	pump_node_pins_t pump_pins = {
		.level_a_gpio = GPIO_NUM_32,   // LEVEL_A_PIN
		.level_b_gpio = GPIO_NUM_33,   // LEVEL_B_PIN
		.pump_gpio    = GPIO_NUM_26,   // PUMP_PIN
	};

	ESP_ERROR_CHECK(pump_node_init(&pump_pins));
	pump_node_start_task(5);
}
