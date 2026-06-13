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
#define ROOT_ACK_FRESH_MS       45000U
#define ROOT_RECOVERY_BURST_MS  5000U
#define ROOT_RECOVERY_SOFT_MS   30000U
#define ROOT_RECOVERY_HARD_MS   90000U
#define ROOT_RECOVERY_LOG_MS    15000U

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

static uint32_t tick_ms(void)
{
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int8_t current_parent_rssi(void)
{
	wifi_ap_record_t ap;
	if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
		return ap.rssi;
	}
	return -127;
}

static void update_v2_topology(bool send_now)
{
	mesh_v2_node_update_topology(mesh_parent_addr.addr,
	                             mesh_root_addr,
	                             (mesh_layer > 0) ? (uint16_t)mesh_layer : 0,
	                             mesh_max_layer,
	                             current_parent_rssi(),
	                             mesh_child_count);
	if (send_now && is_mesh_connected) {
		(void)mesh_v2_node_send_topology();
	}
}

static void note_mesh_disconnected(void)
{
	if (is_mesh_connected || s_disconnected_since_ms == 0) {
		s_disconnected_since_ms = tick_ms();
	}
	is_mesh_connected = false;
	s_root_recovery_phase = ROOT_RECOVERY_OK;
	s_root_unhealthy_since_ms = 0;
	s_last_root_burst_ms = 0;
}

static void note_mesh_connected(void)
{
	is_mesh_connected = true;
	s_disconnected_since_ms = 0;
	s_last_reconnect_attempt_ms = 0;
	s_root_recovery_phase = ROOT_RECOVERY_WAIT_ACK;
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
	if (s_root_recovery_phase != ROOT_RECOVERY_OK && s_root_unhealthy_since_ms != 0) {
		uint32_t down_ms = (uint32_t)(now - s_root_unhealthy_since_ms);
		ESP_LOGI(MESH_TAG,
		         "root recovery: restored after %lu ms, ack_age=%lu",
		         (unsigned long)down_ms,
		         (unsigned long)mesh_v2_node_ack_age_ms());
	}
	s_root_recovery_phase = ROOT_RECOVERY_OK;
	s_root_unhealthy_since_ms = 0;
	s_last_root_burst_ms = 0;
	s_last_root_soft_reconnect_ms = 0;
	s_last_root_hard_restart_ms = 0;
	s_last_root_recovery_log_ms = 0;
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
	ESP_LOGW(MESH_TAG,
	         "root recovery: phase=%s down=%lu ms ack_age=%lu last_send=%s action=%s",
	         root_recovery_phase_name(phase),
	         (unsigned long)down_ms,
	         (unsigned long)mesh_v2_node_ack_age_ms(),
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

	while (is_running) {
		data.size = sizeof(rx_buf);

		err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
		if (err != ESP_OK) {
			ESP_LOGE(MESH_TAG, "esp_mesh_recv failed: 0x%x (%s)", err, esp_err_to_name(err));
			continue;
		}

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

static void root_liveness_watchdog_step(void)
{
	uint32_t now = tick_ms();

	if (!is_mesh_connected) {
		return;
	}

	if (mesh_v2_node_ack_fresh(ROOT_ACK_FRESH_MS)) {
		root_recovery_reset_ok(now);
		return;
	}

	if (s_root_unhealthy_since_ms == 0) {
		s_root_unhealthy_since_ms = now;
	}

	uint32_t down_ms = (uint32_t)(now - s_root_unhealthy_since_ms);
	root_recovery_phase_t phase = ROOT_RECOVERY_WAIT_ACK;

	if (down_ms >= ROOT_RECOVERY_HARD_MS) {
		phase = ROOT_RECOVERY_MESH_RESTART;
	} else if (down_ms >= ROOT_RECOVERY_SOFT_MS) {
		phase = ROOT_RECOVERY_MESH_RECONNECT;
	} else if (down_ms >= ROOT_RECOVERY_BURST_MS) {
		phase = ROOT_RECOVERY_NODEINFO_BURST;
	}
	s_root_recovery_phase = phase;

	if (s_last_root_burst_ms == 0 ||
	    (uint32_t)(now - s_last_root_burst_ms) >= ROOT_RECOVERY_BURST_MS) {
		s_last_root_burst_ms = now;
		mesh_v2_node_kick_root();
		update_v2_topology(false);
		esp_err_t err = mesh_log_stream_send_nodeinfo_now();
		(void)mesh_v2_node_send_topology();
		root_recovery_log_throttled(now, down_ms, phase, "nodeinfo_burst", err);
	}

	if (down_ms >= ROOT_RECOVERY_HARD_MS &&
	    (s_last_root_hard_restart_ms == 0 ||
	     (uint32_t)(now - s_last_root_hard_restart_ms) >= ROOT_RECOVERY_HARD_MS)) {
		s_last_root_hard_restart_ms = now;
		root_recovery_log_throttled(now, down_ms, ROOT_RECOVERY_MESH_RESTART,
		                            "mesh_stop_start",
		                            mesh_log_stream_last_send_err());
		mesh_v2_node_on_mesh_disconnected();
		mesh_log_stream_on_mesh_disconnected();

		esp_err_t stop_err = esp_mesh_stop();
		if (stop_err != ESP_OK) {
			ESP_LOGW(MESH_TAG, "root recovery: esp_mesh_stop failed: %s",
			         esp_err_to_name(stop_err));
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
		note_mesh_disconnected();
		(void)esp_mesh_flush_scan_result();
		esp_err_t start_err = esp_mesh_start();
		if (start_err != ESP_OK) {
			ESP_LOGW(MESH_TAG, "root recovery: esp_mesh_start failed: %s",
			         esp_err_to_name(start_err));
		}
		return;
	}

	if (down_ms >= ROOT_RECOVERY_SOFT_MS &&
	    (s_last_root_soft_reconnect_ms == 0 ||
	     (uint32_t)(now - s_last_root_soft_reconnect_ms) >= ROOT_RECOVERY_SOFT_MS)) {
		s_last_root_soft_reconnect_ms = now;
		root_recovery_log_throttled(now, down_ms, ROOT_RECOVERY_MESH_RECONNECT,
		                            "mesh_disconnect_connect",
		                            mesh_log_stream_last_send_err());
		esp_err_t disc_err = esp_mesh_disconnect();
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

		if (is_mesh_connected) {
			root_liveness_watchdog_step();
			continue;
		}

		uint32_t now = tick_ms();
		if (s_disconnected_since_ms == 0) {
			s_disconnected_since_ms = now;
			continue;
		}

		uint32_t down_ms = (uint32_t)(now - s_disconnected_since_ms);
		if (down_ms >= MESH_RECONNECT_SOFT_MS &&
		    (uint32_t)(now - s_last_reconnect_attempt_ms) >= MESH_RECONNECT_SOFT_MS) {
			s_last_reconnect_attempt_ms = now;
			ESP_LOGW(MESH_TAG,
			         "mesh reconnect watchdog: no parent for %lu ms, calling esp_mesh_connect()",
			         (unsigned long)down_ms);
			esp_err_t flush_err = esp_mesh_flush_scan_result();
			if (flush_err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: scan flush failed: %s",
				         esp_err_to_name(flush_err));
			}
			esp_err_t err = esp_mesh_connect();
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
			ESP_LOGW(MESH_TAG,
			         "mesh reconnect watchdog: no parent for %lu ms, restarting mesh service",
			         (unsigned long)down_ms);

			mesh_v2_node_on_mesh_disconnected();
			mesh_log_stream_on_mesh_disconnected();

			esp_err_t stop_err = esp_mesh_stop();
			if (stop_err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: esp_mesh_stop failed: %s",
				         esp_err_to_name(stop_err));
			}

			vTaskDelay(pdMS_TO_TICKS(1000));
			note_mesh_disconnected();
			esp_err_t flush_err = esp_mesh_flush_scan_result();
			if (flush_err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: scan flush before restart failed: %s",
				         esp_err_to_name(flush_err));
			}

			esp_err_t start_err = esp_mesh_start();
			if (start_err != ESP_OK) {
				ESP_LOGW(MESH_TAG,
				         "mesh reconnect watchdog: esp_mesh_start failed: %s",
				         esp_err_to_name(start_err));
			}
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
		esp_mesh_get_id(&id);
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_STARTED> ID:" MACSTR,
		         MAC2STR(id.addr));
		note_mesh_disconnected();
		mesh_layer = esp_mesh_get_layer();
	}
	break;

	case MESH_EVENT_STOPPED: {
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
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_NO_PARENT_FOUND> scan times:%d",
		         np->scan_times);
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
		update_v2_topology(true);
	}
	break;

	case MESH_EVENT_ROOT_ADDRESS: {
		mesh_event_root_address_t *ra =
		    (mesh_event_root_address_t *)event_data;
		memcpy(mesh_root_addr, ra->addr, sizeof(mesh_root_addr));
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_ROOT_ADDRESS> root:" MACSTR,
		         MAC2STR(ra->addr));
		update_v2_topology(true);
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
	ESP_ERROR_CHECK(esp_mesh_init());
	ESP_ERROR_CHECK(
	    esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID,
	                               &mesh_event_handler, NULL));
	mesh_time_sync_init();
	log_time_vprintf_start();
	mesh_v2_node_init(MESH_TAG);
	mesh_log_stream_init(MESH_TAG);

	// Network uses fixed root; only node0 designates itself as MESH_ROOT.
	ESP_ERROR_CHECK(esp_mesh_fix_root(true));

	ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
	mesh_max_layer = CONFIG_MESH_MAX_LAYER;
#if CONFIG_CHOINKA_DIRECT_ROOT_ONLY
	if (mesh_max_layer > CONFIG_CHOINKA_DIRECT_MAX_LAYER) {
		mesh_max_layer = CONFIG_CHOINKA_DIRECT_MAX_LAYER;
	}
#endif
	ESP_ERROR_CHECK(esp_mesh_set_max_layer(mesh_max_layer));
	ESP_LOGI(MESH_TAG,
	         "mesh max layer:%u (configured:%u, direct_root_only:%u)",
	         (unsigned)mesh_max_layer,
	         (unsigned)CONFIG_MESH_MAX_LAYER,
	         (unsigned)CONFIG_CHOINKA_DIRECT_ROOT_ONLY);
	ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
	ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

	ESP_ERROR_CHECK(esp_mesh_disable_ps());
	ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));


	mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();

	// mesh_id
	memcpy(cfg.mesh_id.addr, MESH_ID, 6);

	// Router-facing Wi-Fi credentials from menuconfig.
	cfg.channel        = CONFIG_MESH_CHANNEL;
	cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
	memcpy(cfg.router.ssid,
	       CONFIG_MESH_ROUTER_SSID,
	       cfg.router.ssid_len);
	memcpy(cfg.router.password,
	       CONFIG_MESH_ROUTER_PASSWD,
	       strlen(CONFIG_MESH_ROUTER_PASSWD));

	// Mesh AP for child nodes.
	ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
	cfg.mesh_ap.max_connection        = CONFIG_MESH_AP_CONNECTIONS;
	cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
	memcpy(cfg.mesh_ap.password,
	       CONFIG_MESH_AP_PASSWD,
	       strlen(CONFIG_MESH_AP_PASSWD));

	ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

	ESP_ERROR_CHECK(esp_mesh_start());
	if (!s_mesh_reconnect_task) {
		xTaskCreate(mesh_reconnect_watchdog_task,
		            "mesh_reconn",
		            4096,
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
