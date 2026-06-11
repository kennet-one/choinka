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
#include "freertos/task.h"

#include "mesh_proto.h"

static const char *TAG = "mesh_ota";

typedef struct {
	bool active;
	esp_ota_handle_t handle;
	const esp_partition_t *partition;
	uint32_t image_size;
	uint32_t written;
	uint16_t last_seq;
} mesh_ota_rx_state_t;

static mesh_ota_rx_state_t s_ota = {0};
static uint32_t s_status_counter = 0;

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

static void send_status(uint8_t op, uint8_t code, uint16_t seq,
                        uint32_t offset, uint32_t total, const char *msg)
{
	mesh_ota_status_packet_t p;
	memset(&p, 0, sizeof(p));

	p.h.magic = MESH_PKT_MAGIC;
	p.h.version = MESH_PKT_VERSION;
	p.h.type = MESH_OTA_TYPE_STATUS;
	p.h.counter = ++s_status_counter;
	esp_wifi_get_mac(WIFI_IF_STA, p.h.src_mac);

	p.op = op;
	p.code = code;
	p.seq = seq;
	p.offset = offset;
	p.total = total;
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

static void reboot_task(void *arg)
{
	(void)arg;
	vTaskDelay(pdMS_TO_TICKS(1500));
	esp_restart();
}

static esp_err_t handle_begin(const mesh_ota_begin_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

	if (s_ota.active) {
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
	send_status(MESH_OTA_OP_DATA, MESH_OTA_STATUS_OK, p->seq,
	            s_ota.written, s_ota.image_size, "chunk OK");
	return ESP_OK;
}

static esp_err_t handle_end(const mesh_ota_end_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

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

	send_status(MESH_OTA_OP_END, MESH_OTA_STATUS_OK, p->seq,
	            s_ota.written, s_ota.image_size, "OTA OK rebooting");
	memset(&s_ota, 0, sizeof(s_ota));

	if (xTaskCreate(reboot_task, "ota_reboot", 2048, NULL, 5, NULL) != pdPASS) {
		ESP_LOGW(TAG, "failed to create OTA reboot task");
	}
	return ESP_OK;
}

static esp_err_t handle_abort(const mesh_ota_abort_packet_t *p, size_t pkt_len)
{
	(void)pkt_len;

	finish_abort();
	send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_OK, p->seq, 0, 0, "OTA aborted");
	return ESP_OK;
}

esp_err_t mesh_ota_receiver_handle_rx(const void *pkt_buf, size_t pkt_len)
{
	if (!pkt_buf || pkt_len < sizeof(mesh_pkt_hdr_t)) {
		return ESP_ERR_INVALID_SIZE;
	}

	const mesh_pkt_hdr_t *h = (const mesh_pkt_hdr_t *)pkt_buf;
	if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION) {
		return ESP_ERR_INVALID_ARG;
	}

	switch (h->type) {
	case MESH_OTA_TYPE_BEGIN:
		if (pkt_len < sizeof(mesh_ota_begin_packet_t)) return ESP_ERR_INVALID_SIZE;
		return handle_begin((const mesh_ota_begin_packet_t *)pkt_buf, pkt_len);
	case MESH_OTA_TYPE_DATA:
		if (pkt_len < offsetof(mesh_ota_data_packet_t, data)) return ESP_ERR_INVALID_SIZE;
		return handle_data((const mesh_ota_data_packet_t *)pkt_buf, pkt_len);
	case MESH_OTA_TYPE_END:
		if (pkt_len < sizeof(mesh_ota_end_packet_t)) return ESP_ERR_INVALID_SIZE;
		return handle_end((const mesh_ota_end_packet_t *)pkt_buf, pkt_len);
	case MESH_OTA_TYPE_ABORT:
		if (pkt_len < sizeof(mesh_ota_abort_packet_t)) return ESP_ERR_INVALID_SIZE;
		return handle_abort((const mesh_ota_abort_packet_t *)pkt_buf, pkt_len);
	default:
		return ESP_ERR_INVALID_ARG;
	}
}
