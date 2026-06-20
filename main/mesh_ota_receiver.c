#include "mesh_ota_receiver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "mesh_proto.h"

static const char *TAG = "mesh_ota";

#define MESH_OTA_RX_TIMEOUT_MS		60000U

typedef struct {
	bool active;
	esp_ota_handle_t handle;
	const esp_partition_t *partition;
	uint32_t image_size;
	uint32_t written;
	uint16_t last_seq;
	TickType_t last_rx_tick;
} mesh_ota_rx_state_t;

static mesh_ota_rx_state_t s_ota = {0};
static uint32_t s_status_counter = 0;
static bool s_reboot_pending = false;
static uint16_t s_reboot_end_seq = 0;
static uint32_t s_reboot_total = 0;
static SemaphoreHandle_t s_ota_lock = NULL;
static TaskHandle_t s_ota_watchdog_task = NULL;

static void copy_packet_text(char *dst, size_t dst_sz, const char *src, size_t src_sz)
{
	size_t n = 0;

	if (!dst || dst_sz == 0) {
		return;
	}

	if (src && src_sz > 0) {
		n = strnlen(src, src_sz);
		if (n >= dst_sz) {
			n = dst_sz - 1;
		}
		memcpy(dst, src, n);
	}
	dst[n] = '\0';
}

static bool packet_text_equals(const char *field, size_t field_sz, const char *expected)
{
	char clean[MESH_OTA_PROJECT_MAX + 1];

	copy_packet_text(clean, sizeof(clean), field, field_sz);
	return expected && strcmp(clean, expected) == 0;
}

static bool ota_rx_stale(void)
{
	if (!s_ota.active) {
		return false;
	}

	return (xTaskGetTickCount() - s_ota.last_rx_tick) >
	       pdMS_TO_TICKS(MESH_OTA_RX_TIMEOUT_MS);
}

static void ota_rx_touch(void)
{
	s_ota.last_rx_tick = xTaskGetTickCount();
}

static void fill_ota_slot_info(char running_label[MESH_OTA_SLOT_LABEL_MAX],
                               char update_label[MESH_OTA_SLOT_LABEL_MAX],
                               uint32_t *running_size,
                               uint32_t *update_size)
{
	const esp_partition_t *running = esp_ota_get_running_partition();
	const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);

	if (running_label) {
		memset(running_label, 0, MESH_OTA_SLOT_LABEL_MAX);
		if (running) {
			strncpy(running_label, running->label, MESH_OTA_SLOT_LABEL_MAX - 1);
		}
	}
	if (update_label) {
		memset(update_label, 0, MESH_OTA_SLOT_LABEL_MAX);
		if (update) {
			strncpy(update_label, update->label, MESH_OTA_SLOT_LABEL_MAX - 1);
		}
	}
	if (running_size) {
		*running_size = running ? (uint32_t)running->size : 0;
	}
	if (update_size) {
		*update_size = update ? (uint32_t)update->size : 0;
	}
}

static void send_status(uint8_t op, uint8_t code, uint16_t seq,
                        uint32_t offset, uint32_t total, const char *msg)
{
	mesh_ota_status_v2_packet_t p;
	memset(&p, 0, sizeof(p));

	p.base.h.magic = MESH_PKT_MAGIC;
	p.base.h.version = MESH_PKT_VERSION;
	p.base.h.type = MESH_OTA_TYPE_STATUS;
	p.base.h.counter = ++s_status_counter;
	esp_wifi_get_mac(WIFI_IF_STA, p.base.h.src_mac);

	p.base.op = op;
	p.base.code = code;
	p.base.seq = seq;
	p.base.offset = offset;
	p.base.total = total;
	if (msg) {
		strncpy(p.base.message, msg, sizeof(p.base.message) - 1);
		p.base.message[sizeof(p.base.message) - 1] = '\0';
	}
	uint32_t running_size = 0;
	uint32_t update_size = 0;
	fill_ota_slot_info(p.running_label, p.update_label,
	                   &running_size, &update_size);
	p.running_size = running_size;
	p.update_size = update_size;

	mesh_data_t data;
	memset(&data, 0, sizeof(data));
	data.data = (uint8_t *)&p;
	data.size = sizeof(p);
	data.proto = MESH_PROTO_BIN;
	data.tos = MESH_TOS_P2P;

	mesh_addr_t dest;
	memset(&dest, 0, sizeof(dest)); // root

	esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

static void finish_abort(void)
{
	if (s_ota.active) {
		esp_ota_abort(s_ota.handle);
	}
	memset(&s_ota, 0, sizeof(s_ota));
}

static void finish_stale_abort(void)
{
	if (!ota_rx_stale()) {
		return;
	}

	ESP_LOGW(TAG, "remote OTA timed out after %u ms, aborting partial image",
	         (unsigned)MESH_OTA_RX_TIMEOUT_MS);
	send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_ERROR, s_ota.last_seq,
	            s_ota.written, s_ota.image_size, "OTA timeout abort");
	finish_abort();
}

static void ota_watchdog_task(void *arg)
{
	(void)arg;

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(5000));
		if (!s_ota_lock) {
			continue;
		}
		if (xSemaphoreTake(s_ota_lock, pdMS_TO_TICKS(1000)) != pdTRUE) {
			continue;
		}
		finish_stale_abort();
		xSemaphoreGive(s_ota_lock);
	}
}

esp_err_t mesh_ota_receiver_start(void)
{
	if (!s_ota_lock) {
		s_ota_lock = xSemaphoreCreateMutex();
		if (!s_ota_lock) {
			return ESP_ERR_NO_MEM;
		}
	}

	if (s_ota_watchdog_task) {
		return ESP_OK;
	}

	if (xTaskCreate(ota_watchdog_task, "ota_rx_watch", 3072, NULL, 4,
	                &s_ota_watchdog_task) != pdPASS) {
		s_ota_watchdog_task = NULL;
		return ESP_ERR_NO_MEM;
	}

	return ESP_OK;
}

static void reboot_task(void *arg)
{
	(void)arg;
	vTaskDelay(pdMS_TO_TICKS(1500));
	esp_restart();
}

static esp_err_t handle_begin(const mesh_ota_begin_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

	if (s_reboot_pending) {
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_BUSY, p->seq,
		            s_reboot_total, s_reboot_total, "OTA reboot pending");
		return ESP_ERR_INVALID_STATE;
	}

	finish_stale_abort();

	if (s_ota.active) {
		if (p->seq == s_ota.last_seq && s_ota.written == 0 &&
		    p->image_size == s_ota.image_size && s_ota.partition) {
			ota_rx_touch();
			send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_OK, p->seq,
			            0, s_ota.image_size, s_ota.partition->label);
			return ESP_OK;
		}
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_BUSY, p->seq,
		            s_ota.written, s_ota.image_size, "OTA already active");
		return ESP_ERR_INVALID_STATE;
	}

	const esp_app_desc_t *running = esp_app_get_description();
	const char *expected_project = running ? running->project_name : "";
	if (!packet_text_equals(p->project_name, sizeof(p->project_name), expected_project)) {
		char msg[64];
		char got[MESH_OTA_PROJECT_MAX + 1];
		copy_packet_text(got, sizeof(got), p->project_name, sizeof(p->project_name));
		snprintf(msg, sizeof(msg), "wrong project %s", got);
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_ERROR, p->seq, 0, p->image_size, msg);
		return ESP_ERR_INVALID_ARG;
	}

	if (p->chunk_size == 0 || p->chunk_size > MESH_OTA_CHUNK_MAX || p->image_size == 0) {
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_ERROR, p->seq, 0,
		            p->image_size, "bad OTA begin");
		return ESP_ERR_INVALID_ARG;
	}

	const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
	if (!partition) {
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_ERROR, p->seq, 0,
		            p->image_size, "no OTA partition");
		return ESP_ERR_NOT_FOUND;
	}

	if (p->image_size > partition->size) {
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_ERROR, p->seq, 0,
		            p->image_size, "image too large");
		return ESP_ERR_INVALID_SIZE;
	}

	esp_ota_handle_t handle = 0;
	esp_err_t err = esp_ota_begin(partition, p->image_size, &handle);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "esp_ota_begin %s", esp_err_to_name(err));
		send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_ERROR, p->seq, 0, p->image_size, msg);
		return err;
	}

	memset(&s_ota, 0, sizeof(s_ota));
	s_ota.active = true;
	s_ota.handle = handle;
	s_ota.partition = partition;
	s_ota.image_size = p->image_size;
	s_ota.last_seq = p->seq;
	ota_rx_touch();
	s_reboot_pending = false;

	char version[MESH_OTA_VERSION_MAX + 1];
	copy_packet_text(version, sizeof(version), p->version, sizeof(p->version));
	ESP_LOGI(TAG, "remote OTA begin: size=%lu target=%s version=%s",
	         (unsigned long)p->image_size, partition->label, version);

	send_status(MESH_OTA_OP_BEGIN, MESH_OTA_STATUS_OK, p->seq, 0,
	            p->image_size, partition->label);
	return ESP_OK;
}

static esp_err_t handle_data(const mesh_ota_data_packet_t *p, size_t pkt_len)
{
	const size_t header_len = offsetof(mesh_ota_data_packet_t, data);

	finish_stale_abort();

	if (!s_ota.active) {
		send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_ERROR, p->seq, 0, 0, "OTA not active");
		return ESP_ERR_INVALID_STATE;
	}

	if (p->len == 0 || p->len > MESH_OTA_CHUNK_MAX || pkt_len < header_len + p->len) {
		send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, "bad chunk");
		return ESP_ERR_INVALID_SIZE;
	}

	if (p->offset != s_ota.written || p->offset + p->len > s_ota.image_size) {
		if (p->seq == s_ota.last_seq &&
		    p->offset < s_ota.written &&
		    p->offset + p->len == s_ota.written) {
			ota_rx_touch();
			send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_OK, p->seq,
			            s_ota.written, s_ota.image_size, "duplicate chunk OK");
			return ESP_OK;
		}
		send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, "offset mismatch");
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = esp_ota_write(s_ota.handle, p->data, p->len);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "esp_ota_write %s", esp_err_to_name(err));
		send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, msg);
		finish_abort();
		return err;
	}

	s_ota.written += p->len;
	s_ota.last_seq = p->seq;
	ota_rx_touch();
	send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_OK, p->seq,
	            s_ota.written, s_ota.image_size, "chunk OK");
	return ESP_OK;
}

static esp_err_t handle_end(const mesh_ota_end_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

	if (s_reboot_pending &&
	    p->seq == s_reboot_end_seq &&
	    p->image_size == s_reboot_total) {
		send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_OK, p->seq,
		            s_reboot_total, s_reboot_total, "OTA OK rebooting");
		return ESP_OK;
	}

	finish_stale_abort();

	if (!s_ota.active) {
		send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_ERROR, p->seq, 0, 0, "OTA not active");
		return ESP_ERR_INVALID_STATE;
	}

	if (p->image_size != s_ota.image_size || s_ota.written != s_ota.image_size) {
		send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, "size mismatch");
		finish_abort();
		return ESP_ERR_INVALID_SIZE;
	}

	esp_err_t err = esp_ota_end(s_ota.handle);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "esp_ota_end %s", esp_err_to_name(err));
		send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, msg);
		finish_abort();
		return err;
	}

	err = esp_ota_set_boot_partition(s_ota.partition);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "set boot %s", esp_err_to_name(err));
		send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_ERROR, p->seq,
		            s_ota.written, s_ota.image_size, msg);
		memset(&s_ota, 0, sizeof(s_ota));
		return err;
	}

	const char *label = s_ota.partition ? s_ota.partition->label : "ota";
	ESP_LOGI(TAG, "remote OTA complete: boot=%s size=%lu",
	         label, (unsigned long)s_ota.image_size);

	s_reboot_pending = true;
	s_reboot_end_seq = p->seq;
	s_reboot_total = s_ota.image_size;

	send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_OK, p->seq,
	            s_ota.written, s_ota.image_size, "OTA OK rebooting");
	memset(&s_ota, 0, sizeof(s_ota));

	if (xTaskCreate(reboot_task, "ota_reboot", 2048, NULL, 5, NULL) != pdPASS) {
		ESP_LOGW(TAG, "failed to create OTA reboot task");
		vTaskDelay(pdMS_TO_TICKS(1500));
		esp_restart();
	}
	return ESP_OK;
}

static esp_err_t handle_abort(const mesh_ota_abort_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

	if (s_reboot_pending) {
		send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_BUSY, p->seq,
		            s_reboot_total, s_reboot_total, "OTA reboot pending");
		return ESP_ERR_INVALID_STATE;
	}

	finish_abort();
	send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_OK, p->seq, 0, 0, "OTA aborted");
	return ESP_OK;
}

esp_err_t mesh_ota_receiver_handle_rx(const void *pkt_buf, size_t pkt_len)
{
	if (!pkt_buf || pkt_len < sizeof(mesh_pkt_hdr_t)) {
		return ESP_ERR_INVALID_SIZE;
	}
	if (!s_ota_lock) {
		return ESP_ERR_INVALID_STATE;
	}

	const mesh_pkt_hdr_t *h = (const mesh_pkt_hdr_t *)pkt_buf;
	if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION) {
		return ESP_ERR_INVALID_ARG;
	}

	if (xSemaphoreTake(s_ota_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}

	esp_err_t ret = ESP_ERR_INVALID_ARG;
	switch (h->type) {
	case MESH_OTA_TYPE_BEGIN:
		ret = (pkt_len < sizeof(mesh_ota_begin_packet_t))
		      ? ESP_ERR_INVALID_SIZE
		      : handle_begin((const mesh_ota_begin_packet_t *)pkt_buf, pkt_len);
		break;
	case MESH_OTA_TYPE_DATA:
		ret = (pkt_len < offsetof(mesh_ota_data_packet_t, data))
		      ? ESP_ERR_INVALID_SIZE
		      : handle_data((const mesh_ota_data_packet_t *)pkt_buf, pkt_len);
		break;
	case MESH_OTA_TYPE_END:
		ret = (pkt_len < sizeof(mesh_ota_end_packet_t))
		      ? ESP_ERR_INVALID_SIZE
		      : handle_end((const mesh_ota_end_packet_t *)pkt_buf, pkt_len);
		break;
	case MESH_OTA_TYPE_ABORT:
		ret = (pkt_len < sizeof(mesh_ota_abort_packet_t))
		      ? ESP_ERR_INVALID_SIZE
		      : handle_abort((const mesh_ota_abort_packet_t *)pkt_buf, pkt_len);
		break;
	default:
		ret = ESP_ERR_INVALID_ARG;
		break;
	}

	xSemaphoreGive(s_ota_lock);
	return ret;
}
