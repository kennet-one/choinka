#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_mesh_internal.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "legacy_proto.h"
#include "pump_node.h"
#include "stack_monitor.h"
#include "log_time_vprintf.h"
#include "mesh_proto.h"
#include "mesh_time_sync.h"
#include "mesh_log_stream.h"

/* -------------------------------------------------------------------------- */
/*  Константи / глобальні змінні                                              */
/* -------------------------------------------------------------------------- */

#define RX_SIZE          (256)
#define TX_INTERVAL_MS   (5000)

static const char *MESH_TAG = "choinka";

/* Один і той самий MESH_ID на всі ноди в цій мережі */
static const uint8_t MESH_ID[6] = { 0x77, 0x77, 0x77, 0x77, 0x77, 0x77 };

static bool       is_running        = true;
static bool       is_mesh_connected = false;
static mesh_addr_t mesh_parent_addr;
static int        mesh_layer        = -1;
static esp_netif_t *netif_sta       = NULL;

/* -------------------------------------------------------------------------- */
/*  Мінімальний власний протокол                                              */
/* -------------------------------------------------------------------------- */
/*
 * Формат нашого пакету:
 *  magic    - 0xA5 (для перевірки, що це "наш" пакет)
 *  version  - версія протоколу (1)
 *  type     - тип (1 = просто текстове "Hello N")
 *  reserved - вирівнювання, запас
 *  counter  - лічильник пакета від цієї ноди
 *  src_mac  - MAC відправника
 *  payload  - невеликий текст (рядок з '\0' в кінці)
 */

/* -------------------------------------------------------------------------- */
/*  Прототипи                                                                 */
/* -------------------------------------------------------------------------- */

static void mesh_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data);

static void mesh_rx_task(void *arg);
static esp_err_t mesh_comm_start(void);

/* -------------------------------------------------------------------------- */
/*  RX task – слухаємо пакети від інших нод                                   */
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

		// не наш протокол? ігноруємо
		if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION) {
			ESP_LOGW(MESH_TAG, "RX unknown packet from " MACSTR " (%d bytes)", MAC2STR(from.addr), (int)data.size);
			continue;
		}

		// диспетчер типів
		if (h->type == MESH_TIME_SYNC_TYPE_TIME) {
			mesh_time_sync_handle_rx(rx_buf, data.size);
			continue;
		}

		if (h->type == MESH_LOG_TYPE_CTRL) {
			mesh_log_stream_handle_rx(rx_buf, data.size);
			continue;
		}

		if (h->type == MESH_PKT_TYPE_TEXT) {
			if (data.size < sizeof(mesh_packet_t)) {
				ESP_LOGW(MESH_TAG, "RX TEXT short: %d bytes", (int)data.size);
				continue;
			}

			const mesh_packet_t *p = (const mesh_packet_t *)rx_buf;

			// гарантуємо '\0'
			char payload[sizeof(p->payload)];
			memcpy(payload, p->payload, sizeof(payload));
			payload[sizeof(payload) - 1] = '\0';

			ESP_LOGI(MESH_TAG, "RX TEXT from " MACSTR " (src=" MACSTR "): \"%s\"",
				MAC2STR(from.addr),
				MAC2STR(p->src_mac),
				payload
			);

			// <-- ОЦЕ МІСЦЕ: якщо в тебе не legacy_handle_text(), заміни на свій handler
			legacy_handle_text(payload);

			continue;
		}

		ESP_LOGI(MESH_TAG, "RX type=%u from " MACSTR " (%d bytes)",
			(unsigned)h->type, MAC2STR(from.addr), (int)data.size);
	}

	vTaskDelete(NULL);
}


/* -------------------------------------------------------------------------- */
/*  Запуск задач TX/RX один раз                                               */
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
		is_mesh_connected = false;
		mesh_layer = esp_mesh_get_layer();
	}
	break;

	case MESH_EVENT_STOPPED: {
		ESP_LOGI(MESH_TAG, "<MESH_EVENT_STOPPED>");
		is_mesh_connected = false;
		mesh_layer = esp_mesh_get_layer();
	}
	break;

	case MESH_EVENT_CHILD_CONNECTED: {
		mesh_event_child_connected_t *child =
		    (mesh_event_child_connected_t *)event_data;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_CHILD_CONNECTED> aid:%d, " MACSTR,
		         child->aid, MAC2STR(child->mac));
	}
	break;

	case MESH_EVENT_CHILD_DISCONNECTED: {
		mesh_event_child_disconnected_t *child =
		    (mesh_event_child_disconnected_t *)event_data;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_CHILD_DISCONNECTED> aid:%d, " MACSTR,
		         child->aid, MAC2STR(child->mac));
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
		is_mesh_connected = true;

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
		is_mesh_connected = false;
		mesh_layer = esp_mesh_get_layer();
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
	}
	break;

	case MESH_EVENT_ROOT_ADDRESS: {
		mesh_event_root_address_t *ra =
		    (mesh_event_root_address_t *)event_data;
		ESP_LOGI(MESH_TAG,
		         "<MESH_EVENT_ROOT_ADDRESS> root:" MACSTR,
		         MAC2STR(ra->addr));
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
/*  app_main – ініціалізація mesh + Wi-Fi                                     */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	// Створюємо netif для mesh (sta + softAP, але зберігаємо тільки sta)
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
        // Звичайний вузол, root’ом не стає
    ESP_ERROR_CHECK(esp_mesh_fix_root(false));      // можна й не викликати, але так явно

	ESP_ERROR_CHECK(esp_mesh_set_topology(CONFIG_MESH_TOPOLOGY));
	ESP_ERROR_CHECK(esp_mesh_set_max_layer(CONFIG_MESH_MAX_LAYER));
	ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
	ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

	ESP_ERROR_CHECK(esp_mesh_disable_ps());
	ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));


	mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();

	// mesh_id
	memcpy(cfg.mesh_id.addr, MESH_ID, 6);

	// роутер (твій домашній Wi-Fi з menuconfig)
	cfg.channel        = CONFIG_MESH_CHANNEL;
	cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
	memcpy(cfg.router.ssid,
	       CONFIG_MESH_ROUTER_SSID,
	       cfg.router.ssid_len);
	memcpy(cfg.router.password,
	       CONFIG_MESH_ROUTER_PASSWD,
	       strlen(CONFIG_MESH_ROUTER_PASSWD));

	// mesh AP (для дітей)
	ESP_ERROR_CHECK(esp_mesh_set_ap_authmode(CONFIG_MESH_AP_AUTHMODE));
	cfg.mesh_ap.max_connection        = CONFIG_MESH_AP_CONNECTIONS;
	cfg.mesh_ap.nonmesh_max_connection = CONFIG_MESH_NON_MESH_AP_CONNECTIONS;
	memcpy(cfg.mesh_ap.password,
	       CONFIG_MESH_AP_PASSWD,
	       strlen(CONFIG_MESH_AP_PASSWD));

	ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

	ESP_ERROR_CHECK(esp_mesh_start());

	ESP_LOGI(MESH_TAG,
	         "mesh started, heap:%" PRId32 ", root_fixed:%d, topo:%d %s, ps:%d",
	         esp_get_minimum_free_heap_size(),
	         esp_mesh_is_root_fixed(),
	         esp_mesh_get_topology(),
	         esp_mesh_get_topology() ? "(chain)" : "(tree)",
	         esp_mesh_is_ps_enabled());

		// -------- Підключення вузла помпи --------
	pump_node_pins_t pump_pins = {
		.level_a_gpio = GPIO_NUM_32,   // LEVEL_A_PIN
		.level_b_gpio = GPIO_NUM_33,   // LEVEL_B_PIN
		.pump_gpio    = GPIO_NUM_26,   // PUMP_PIN
	};

	ESP_ERROR_CHECK(pump_node_init(&pump_pins));
	pump_node_start_task(5);     
	mesh_time_sync_init();
	log_time_vprintf_start();     
	mesh_log_stream_init(MESH_TAG);
}
