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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "mbedtls/md.h"

#include "mesh_v2_link.h"
#include "mesh_proto.h"

static const char *TAG = "mesh_ota";

#define MESH_OTA_RX_TIMEOUT_MS		60000U
#define MESH_OTA_V2_RX_TIMEOUT_MS	300000U
#define MESH_OTA_V2_QUEUE_LEN		2U
#define MESH_OTA_V2_WORKER_STACK	6144U
#define MESH_OTA_V2_WORKER_PRIO		5U

typedef struct {
	bool active;
	esp_ota_handle_t handle;
	const esp_partition_t *partition;
	uint32_t image_size;
	uint32_t written;
	uint32_t op_id;
	uint16_t last_seq;
	TickType_t last_rx_tick;
	bool v2_active;
	bool sha_active;
	mbedtls_md_context_t sha_ctx;
	uint8_t sha256[MESH_V2_OTA_SHA256_LEN];
} mesh_ota_rx_state_t;

typedef struct {
	size_t payload_len;
	uint8_t payload[sizeof(mesh_v2_ota_data_payload_t)];
} mesh_ota_v2_work_item_t;

static mesh_ota_rx_state_t s_ota = {0};
static char s_last_abort_reason[32] = "boot";
static uint32_t s_status_counter = 0;
static bool s_reboot_pending = false;
static uint16_t s_reboot_end_seq = 0;
static uint32_t s_reboot_v2_op_id = 0;
static uint32_t s_reboot_total = 0;
static SemaphoreHandle_t s_ota_lock = NULL;
static QueueHandle_t s_ota_v2_queue = NULL;
static TaskHandle_t s_ota_v2_worker_task = NULL;
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

	uint32_t timeout_ms = s_ota.v2_active ?
	                      MESH_OTA_V2_RX_TIMEOUT_MS :
	                      MESH_OTA_RX_TIMEOUT_MS;
	return (xTaskGetTickCount() - s_ota.last_rx_tick) >
	       pdMS_TO_TICKS(timeout_ms);
}

static void ota_rx_touch(void)
{
	s_ota.last_rx_tick = xTaskGetTickCount();
}

static void ota_note_abort_reason(const char *reason)
{
	if (!reason || !reason[0]) {
		reason = "unknown";
	}
	strncpy(s_last_abort_reason, reason, sizeof(s_last_abort_reason) - 1);
	s_last_abort_reason[sizeof(s_last_abort_reason) - 1] = '\0';
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

static void send_status_v2(uint8_t op, uint8_t code, uint32_t op_id,
                           uint32_t offset, uint32_t total, const char *msg)
{
	mesh_v2_ota_status_payload_t p;
	memset(&p, 0, sizeof(p));

	p.c.op = MESH_V2_OTA_OP_STATUS;
	p.c.status = code;
	p.c.op_id = op_id;
	p.c.image_size = total;
	p.c.offset = offset;
	p.c.len = offset;
	if (msg) {
		strncpy(p.c.message, msg, sizeof(p.c.message) - 1);
		p.c.message[sizeof(p.c.message) - 1] = '\0';
	}
	if (op != 0) {
		p.c.rsv = op;
	}
	uint32_t running_size = 0;
	uint32_t update_size = 0;
	fill_ota_slot_info(p.running_label, p.update_label,
	                   &running_size, &update_size);
	p.running_size = running_size;
	p.update_size = update_size;

	(void)mesh_v2_node_send_ota_status(&p);
}

static bool sha_is_zero(const uint8_t sha[MESH_V2_OTA_SHA256_LEN])
{
	uint8_t acc = 0;
	for (size_t i = 0; i < MESH_V2_OTA_SHA256_LEN; i++) {
		acc |= sha[i];
	}
	return acc == 0;
}

static void ota_hash_free(void)
{
	if (s_ota.sha_active) {
		mbedtls_md_free(&s_ota.sha_ctx);
		s_ota.sha_active = false;
	}
}

static esp_err_t ota_hash_start(const uint8_t expected[MESH_V2_OTA_SHA256_LEN])
{
	const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
	if (!info) {
		return ESP_ERR_INVALID_ARG;
	}

	ota_hash_free();
	mbedtls_md_init(&s_ota.sha_ctx);
	int ret = mbedtls_md_setup(&s_ota.sha_ctx, info, 0);
	if (ret == 0) {
		ret = mbedtls_md_starts(&s_ota.sha_ctx);
	}
	if (ret != 0) {
		mbedtls_md_free(&s_ota.sha_ctx);
		return ESP_FAIL;
	}

	if (expected && !sha_is_zero(expected)) {
		memcpy(s_ota.sha256, expected, MESH_V2_OTA_SHA256_LEN);
	}
	s_ota.sha_active = true;
	return ESP_OK;
}

static esp_err_t ota_hash_update(const uint8_t *data, size_t len)
{
	if (!s_ota.sha_active || !data || len == 0) {
		return ESP_ERR_INVALID_STATE;
	}
	return mbedtls_md_update(&s_ota.sha_ctx, data, len) == 0 ? ESP_OK : ESP_FAIL;
}

static esp_err_t ota_hash_finish_and_check(void)
{
	uint8_t got[MESH_V2_OTA_SHA256_LEN] = {0};
	if (!s_ota.sha_active) {
		return ESP_ERR_INVALID_STATE;
	}
	int ret = mbedtls_md_finish(&s_ota.sha_ctx, got);
	ota_hash_free();
	if (ret != 0) {
		return ESP_FAIL;
	}
	return memcmp(got, s_ota.sha256, sizeof(got)) == 0 ? ESP_OK : ESP_ERR_INVALID_CRC;
}

static void finish_abort(void)
{
	if (s_ota.active) {
		esp_ota_abort(s_ota.handle);
	}
	ota_hash_free();
	memset(&s_ota, 0, sizeof(s_ota));
}

static void finish_stale_abort(void)
{
	if (!ota_rx_stale()) {
		return;
	}

	uint32_t timeout_ms = s_ota.v2_active ?
	                      MESH_OTA_V2_RX_TIMEOUT_MS :
	                      MESH_OTA_RX_TIMEOUT_MS;
	ESP_LOGW(TAG, "remote OTA timed out after %u ms, aborting partial image",
	         (unsigned)timeout_ms);
	if (s_ota.v2_active) {
		send_status_v2(MESH_V2_OTA_OP_ABORT, MESH_V2_OTA_STATUS_ERROR,
		               s_ota.op_id, s_ota.written, s_ota.image_size,
		               "OTA timeout abort");
	} else {
		send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_ERROR, s_ota.last_seq,
		            s_ota.written, s_ota.image_size, "OTA timeout abort");
	}
	ota_note_abort_reason("timeout");
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

static esp_err_t process_v2_payload_locked(const void *payload, size_t payload_len);

static void ota_v2_worker_task(void *arg)
{
	(void)arg;
	mesh_ota_v2_work_item_t item;

	for (;;) {
		if (xQueueReceive(s_ota_v2_queue, &item, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		if (!s_ota_lock) {
			continue;
		}
		if (xSemaphoreTake(s_ota_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
			const mesh_v2_ota_common_payload_t *c =
				(const mesh_v2_ota_common_payload_t *)item.payload;
			if (item.payload_len >= sizeof(*c)) {
				send_status_v2(c->op, MESH_V2_OTA_STATUS_ERROR,
				               c->op_id, 0, c->image_size,
				               "OTA worker lock timeout");
			}
			continue;
		}
		(void)process_v2_payload_locked(item.payload, item.payload_len);
		xSemaphoreGive(s_ota_lock);
	}
}

static esp_err_t start_v2_worker(void)
{
	if (!s_ota_v2_queue) {
		s_ota_v2_queue = xQueueCreate(MESH_OTA_V2_QUEUE_LEN,
		                              sizeof(mesh_ota_v2_work_item_t));
		if (!s_ota_v2_queue) return ESP_ERR_NO_MEM;
	}
	if (!s_ota_v2_worker_task &&
	    xTaskCreate(ota_v2_worker_task, "ota_v2_rx",
	                MESH_OTA_V2_WORKER_STACK, NULL,
	                MESH_OTA_V2_WORKER_PRIO,
	                &s_ota_v2_worker_task) != pdPASS) {
		return ESP_ERR_NO_MEM;
	}
	return ESP_OK;
}
esp_err_t mesh_ota_receiver_start(void)
{
	if (!s_ota_lock) {
		s_ota_lock = xSemaphoreCreateMutex();
		if (!s_ota_lock) {
			return ESP_ERR_NO_MEM;
		}
	}

	esp_err_t err = start_v2_worker();
	if (err != ESP_OK) {
		return err;
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

bool mesh_ota_receiver_active(void)
{
	bool active = false;
	if (!s_ota_lock) {
		return false;
	}
	if (xSemaphoreTake(s_ota_lock, pdMS_TO_TICKS(20)) != pdTRUE) {
		return true;
	}
	active = s_ota.active;
	xSemaphoreGive(s_ota_lock);
	return active;
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
	ota_note_abort_reason("v1 prepare");
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
	ota_note_abort_reason("v1 abort");
	send_status(MESH_OTA_OP_ABORT, MESH_OTA_STATUS_OK, p->seq, 0, 0, "OTA aborted");
	return ESP_OK;
}

static esp_err_t handle_v2_prepare(const mesh_v2_ota_prepare_payload_t *p,
                                   size_t payload_len)
{
	(void)payload_len;
	const mesh_v2_ota_common_payload_t *c = &p->c;

	if (s_reboot_pending) {
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_BUSY,
		               c->op_id, s_reboot_total, s_reboot_total,
		               "OTA reboot pending");
		return ESP_ERR_INVALID_STATE;
	}

	finish_stale_abort();

	if (s_ota.active) {
		if (s_ota.v2_active && c->op_id == s_ota.op_id &&
		    s_ota.written == 0 && c->image_size == s_ota.image_size &&
		    s_ota.partition) {
			ota_rx_touch();
			send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_OK,
			               c->op_id, 0, s_ota.image_size,
			               s_ota.partition->label);
			return ESP_OK;
		}
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_BUSY,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "OTA already active");
		return ESP_ERR_INVALID_STATE;
	}

	const esp_app_desc_t *running = esp_app_get_description();
	const char *expected_project = running ? running->project_name : "";
	if (!packet_text_equals(c->project_name, sizeof(c->project_name), expected_project)) {
		char msg[64];
		char got[MESH_OTA_PROJECT_MAX + 1];
		copy_packet_text(got, sizeof(got), c->project_name, sizeof(c->project_name));
		snprintf(msg, sizeof(msg), "wrong project %s", got);
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, msg);
		return ESP_ERR_INVALID_ARG;
	}

	if (c->op_id == 0 || c->image_size == 0 ||
	    c->len == 0 || c->len > MESH_V2_OTA_CHUNK_MAX) {
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, "bad OTA prepare");
		return ESP_ERR_INVALID_ARG;
	}

	const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
	if (!partition) {
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, "no OTA partition");
		return ESP_ERR_NOT_FOUND;
	}

	if (c->image_size > partition->size) {
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, "image too large");
		return ESP_ERR_INVALID_SIZE;
	}

	esp_ota_handle_t handle = 0;
	esp_err_t err = esp_ota_begin(partition, c->image_size, &handle);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "esp_ota_begin %s", esp_err_to_name(err));
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, msg);
		return err;
	}

	memset(&s_ota, 0, sizeof(s_ota));
	ota_note_abort_reason("v2 prepare");
	s_ota.active = true;
	s_ota.v2_active = true;
	s_ota.handle = handle;
	s_ota.partition = partition;
	s_ota.image_size = c->image_size;
	s_ota.op_id = c->op_id;
	ota_rx_touch();
	s_reboot_pending = false;

	err = ota_hash_start(c->sha256);
	if (err != ESP_OK) {
		finish_abort();
		send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, 0, c->image_size, "sha256 init failed");
		return err;
	}

	char version[MESH_OTA_VERSION_MAX + 1];
	copy_packet_text(version, sizeof(version), c->version, sizeof(c->version));
	ESP_LOGI(TAG, "remote OTA v2 prepare: size=%lu target=%s version=%s",
	         (unsigned long)c->image_size, partition->label, version);

	send_status_v2(MESH_V2_OTA_OP_PREPARE, MESH_V2_OTA_STATUS_OK,
	               c->op_id, 0, c->image_size, partition->label);
	return ESP_OK;
}

static esp_err_t handle_v2_data(const mesh_v2_ota_data_payload_t *p,
                                size_t payload_len)
{
	const size_t header_len = offsetof(mesh_v2_ota_data_payload_t, data);
	const mesh_v2_ota_common_payload_t *c = &p->c;

	finish_stale_abort();

	if (!s_ota.active || !s_ota.v2_active || c->op_id != s_ota.op_id) {
		char msg[64];
		snprintf(msg, sizeof(msg), "not active a=%u v=%u cur=%lu r=%s",
		         s_ota.active ? 1U : 0U, s_ota.v2_active ? 1U : 0U,
		         (unsigned long)s_ota.op_id, s_last_abort_reason);
		send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, msg);
		return ESP_ERR_INVALID_STATE;
	}

	if (c->len == 0 || c->len > MESH_V2_OTA_CHUNK_MAX ||
	    payload_len < header_len + c->len ||
	    c->offset + c->len > s_ota.image_size) {
		send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, "bad chunk");
		return ESP_ERR_INVALID_SIZE;
	}

	if (c->offset != s_ota.written) {
		if (c->offset < s_ota.written && c->offset + c->len <= s_ota.written) {
			ota_rx_touch();
			send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_OK,
			               c->op_id, s_ota.written, s_ota.image_size,
			               "duplicate chunk OK");
			return ESP_OK;
		}
		send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "offset mismatch");
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t err = esp_ota_write(s_ota.handle, p->data, c->len);
	if (err == ESP_OK) {
		err = ota_hash_update(p->data, c->len);
	}
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "OTA write/hash %s", esp_err_to_name(err));
		send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, msg);
		ota_note_abort_reason("write/hash");
		finish_abort();
		return err;
	}

	s_ota.written += c->len;
	ota_rx_touch();
	send_status_v2(MESH_V2_OTA_OP_DATA, MESH_V2_OTA_STATUS_OK,
	               c->op_id, s_ota.written, s_ota.image_size, "chunk OK");
	return ESP_OK;
}

static esp_err_t handle_v2_commit(const mesh_v2_ota_commit_payload_t *p,
                                  size_t payload_len)
{
	(void)payload_len;
	const mesh_v2_ota_common_payload_t *c = &p->c;

	if (s_reboot_pending && c->op_id == s_reboot_v2_op_id) {
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_OK,
		               c->op_id, s_reboot_total, s_reboot_total,
		               "OTA OK rebooting");
		return ESP_OK;
	}

	finish_stale_abort();

	if (!s_ota.active || !s_ota.v2_active || c->op_id != s_ota.op_id) {
		char msg[64];
		snprintf(msg, sizeof(msg), "not active a=%u v=%u cur=%lu r=%s",
		         s_ota.active ? 1U : 0U, s_ota.v2_active ? 1U : 0U,
		         (unsigned long)s_ota.op_id, s_last_abort_reason);
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, msg);
		return ESP_ERR_INVALID_STATE;
	}

	if (c->image_size != s_ota.image_size || s_ota.written != s_ota.image_size) {
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "size mismatch");
		ota_note_abort_reason("size mismatch");
		finish_abort();
		return ESP_ERR_INVALID_SIZE;
	}

	if (sha_is_zero(c->sha256)) {
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "missing sha256");
		ota_note_abort_reason("missing sha");
		finish_abort();
		return ESP_ERR_INVALID_ARG;
	}
	if (!sha_is_zero(s_ota.sha256) &&
	    memcmp(c->sha256, s_ota.sha256, MESH_V2_OTA_SHA256_LEN) != 0) {
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "sha256 mismatch");
		ota_note_abort_reason("sha mismatch");
		finish_abort();
		return ESP_ERR_INVALID_CRC;
	}
	memcpy(s_ota.sha256, c->sha256, MESH_V2_OTA_SHA256_LEN);
	if (ota_hash_finish_and_check() != ESP_OK) {
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size,
		               "sha256 mismatch");
		ota_note_abort_reason("sha mismatch");
		finish_abort();
		return ESP_ERR_INVALID_CRC;
	}

	esp_err_t err = esp_ota_end(s_ota.handle);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "esp_ota_end %s", esp_err_to_name(err));
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, msg);
		ota_note_abort_reason("ota_end");
		finish_abort();
		return err;
	}

	err = esp_ota_set_boot_partition(s_ota.partition);
	if (err != ESP_OK) {
		char msg[64];
		snprintf(msg, sizeof(msg), "set boot %s", esp_err_to_name(err));
		send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_ERROR,
		               c->op_id, s_ota.written, s_ota.image_size, msg);
		memset(&s_ota, 0, sizeof(s_ota));
		return err;
	}

	const char *label = s_ota.partition ? s_ota.partition->label : "ota";
	uint32_t total = s_ota.image_size;
	uint32_t op_id = s_ota.op_id;
	ESP_LOGI(TAG, "remote OTA v2 complete: boot=%s size=%lu",
	         label, (unsigned long)total);

	s_reboot_pending = true;
	s_reboot_end_seq = 0;
	s_reboot_v2_op_id = op_id;
	s_reboot_total = total;

	send_status_v2(MESH_V2_OTA_OP_COMMIT, MESH_V2_OTA_STATUS_OK,
	               op_id, total, total, "OTA OK rebooting");
	memset(&s_ota, 0, sizeof(s_ota));

	if (xTaskCreate(reboot_task, "ota_reboot", 2048, NULL, 5, NULL) != pdPASS) {
		ESP_LOGW(TAG, "failed to create OTA reboot task");
		vTaskDelay(pdMS_TO_TICKS(1500));
		esp_restart();
	}
	return ESP_OK;
}

static esp_err_t handle_v2_abort(const mesh_v2_ota_abort_payload_t *p,
                                 size_t payload_len)
{
	(void)payload_len;
	uint32_t op_id = p ? p->c.op_id : 0;

	if (s_reboot_pending) {
		send_status_v2(MESH_V2_OTA_OP_ABORT, MESH_V2_OTA_STATUS_BUSY,
		               op_id, s_reboot_total, s_reboot_total,
		               "OTA reboot pending");
		return ESP_ERR_INVALID_STATE;
	}

	if (s_ota.active && s_ota.v2_active && op_id != s_ota.op_id) {
		send_status_v2(MESH_V2_OTA_OP_ABORT, MESH_V2_OTA_STATUS_OK,
		               op_id, s_ota.written, s_ota.image_size,
		               "stale abort ignored");
		return ESP_OK;
	}

	ota_note_abort_reason("v2 abort");
	finish_abort();
	send_status_v2(MESH_V2_OTA_OP_ABORT, MESH_V2_OTA_STATUS_OK,
	               op_id, 0, 0, "OTA aborted");
	return ESP_OK;
}

static esp_err_t process_v2_payload_locked(const void *payload, size_t payload_len)
{
	const mesh_v2_ota_common_payload_t *c = payload;

	switch (c->op) {
	case MESH_V2_OTA_OP_PREPARE:
		return (payload_len < sizeof(mesh_v2_ota_prepare_payload_t))
		       ? ESP_ERR_INVALID_SIZE
		       : handle_v2_prepare((const mesh_v2_ota_prepare_payload_t *)payload,
		                           payload_len);
	case MESH_V2_OTA_OP_DATA:
		return (payload_len < offsetof(mesh_v2_ota_data_payload_t, data))
		       ? ESP_ERR_INVALID_SIZE
		       : handle_v2_data((const mesh_v2_ota_data_payload_t *)payload,
		                        payload_len);
	case MESH_V2_OTA_OP_COMMIT:
		return (payload_len < sizeof(mesh_v2_ota_commit_payload_t))
		       ? ESP_ERR_INVALID_SIZE
		       : handle_v2_commit((const mesh_v2_ota_commit_payload_t *)payload,
		                          payload_len);
	case MESH_V2_OTA_OP_ABORT:
		return (payload_len < sizeof(mesh_v2_ota_abort_payload_t))
		       ? ESP_ERR_INVALID_SIZE
		       : handle_v2_abort((const mesh_v2_ota_abort_payload_t *)payload,
		                         payload_len);
	default:
		return ESP_ERR_INVALID_ARG;
	}
}
esp_err_t mesh_ota_receiver_handle_v2(const void *payload, size_t payload_len)
{
	if (!payload || payload_len < sizeof(mesh_v2_ota_common_payload_t)) {
		return ESP_ERR_INVALID_SIZE;
	}
	if (!s_ota_lock || !s_ota_v2_queue) {
		return ESP_ERR_INVALID_STATE;
	}
	if (payload_len > sizeof(((mesh_ota_v2_work_item_t *)0)->payload)) {
		return ESP_ERR_INVALID_SIZE;
	}

	const mesh_v2_ota_common_payload_t *c = payload;
	size_t min_len = sizeof(*c);
	switch (c->op) {
	case MESH_V2_OTA_OP_PREPARE:
		min_len = sizeof(mesh_v2_ota_prepare_payload_t);
		break;
	case MESH_V2_OTA_OP_DATA:
		min_len = offsetof(mesh_v2_ota_data_payload_t, data);
		break;
	case MESH_V2_OTA_OP_COMMIT:
		min_len = sizeof(mesh_v2_ota_commit_payload_t);
		break;
	case MESH_V2_OTA_OP_ABORT:
		min_len = sizeof(mesh_v2_ota_abort_payload_t);
		break;
	default:
		return ESP_ERR_INVALID_ARG;
	}
	if (payload_len < min_len) {
		return ESP_ERR_INVALID_SIZE;
	}

	mesh_ota_v2_work_item_t item;
	memset(&item, 0, sizeof(item));
	item.payload_len = payload_len;
	memcpy(item.payload, payload, payload_len);
	if (xQueueSend(s_ota_v2_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
		send_status_v2(c->op, MESH_V2_OTA_STATUS_BUSY, c->op_id,
		               0, c->image_size, "OTA worker busy");
		return ESP_ERR_TIMEOUT;
	}
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
