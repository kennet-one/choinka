#include "mesh_v2_link.h"

#include <stddef.h>
#include <string.h>

#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#include "mesh_proto.h"

static const char *TAG = "mesh_v2";

typedef struct {
	bool used;
	uint32_t seq;
	size_t len;
	uint8_t bytes[MESH_V2_PACKET_MAX];
} replay_slot_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static replay_slot_t s_ring[MESH_V2_REPLAY_RING_SIZE];
static char s_tag[MESH_V2_TAG_MAX] = "node";
static uint32_t s_session_id = 0;
static uint32_t s_next_seq = 1;
static bool s_inited = false;
static bool s_root_ready = false;
static uint32_t s_last_ack_ms = 0;
static uint32_t s_last_hello_ms = 0;

#ifndef MESH_V2_ACK_STALE_MS
#define MESH_V2_ACK_STALE_MS 30000
#endif

#ifndef MESH_V2_HELLO_RETRY_MS
#define MESH_V2_HELLO_RETRY_MS 5000
#endif

static uint32_t ms_now(void)
{
	return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void copy_tag(char *dst, size_t dst_sz, const char *tag)
{
	if (!dst || dst_sz == 0) {
		return;
	}

	const char *src = (tag && tag[0]) ? tag : "node";
	size_t n = strnlen(src, dst_sz - 1);
	memcpy(dst, src, n);
	dst[n] = '\0';
}

static uint16_t crc16_update(uint16_t crc, const uint8_t *data, size_t len)
{
	while (len--) {
		crc ^= (uint16_t)(*data++) << 8;
		for (int i = 0; i < 8; i++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

static uint16_t packet_crc(const mesh_v2_hdr_t *h, const uint8_t *payload)
{
	mesh_v2_hdr_t tmp = *h;
	tmp.crc16 = 0;

	uint16_t crc = 0xffff;
	crc = crc16_update(crc, (const uint8_t *)&tmp, sizeof(tmp));
	if (tmp.payload_len > 0 && payload) {
		crc = crc16_update(crc, payload, tmp.payload_len);
	}
	return crc;
}

static bool validate_packet(const void *pkt_buf, size_t pkt_len, const mesh_v2_hdr_t **out_h,
                            const uint8_t **out_payload)
{
	if (!pkt_buf || pkt_len < sizeof(mesh_v2_hdr_t)) {
		return false;
	}

	const mesh_v2_hdr_t *h = (const mesh_v2_hdr_t *)pkt_buf;
	if (h->magic != MESH_PKT_MAGIC || h->version != MESH_PKT_VERSION_V2) {
		return false;
	}
	if (h->payload_len > MESH_V2_PAYLOAD_MAX ||
	    pkt_len != sizeof(mesh_v2_hdr_t) + h->payload_len ||
	    pkt_len > MESH_V2_PACKET_MAX) {
		return false;
	}

	const uint8_t *payload = (const uint8_t *)pkt_buf + sizeof(mesh_v2_hdr_t);
	if (packet_crc(h, payload) != h->crc16) {
		ESP_LOGW(TAG, "CRC mismatch type=%u seq=%lu",
		         (unsigned)h->type, (unsigned long)h->seq);
		return false;
	}

	if (out_h) {
		*out_h = h;
	}
	if (out_payload) {
		*out_payload = payload;
	}
	return true;
}

static esp_err_t send_raw_to_root(const uint8_t *bytes, size_t len)
{
	mesh_addr_t dest = {0};
	mesh_data_t data = {
		.data = (uint8_t *)bytes,
		.size = len,
		.proto = MESH_PROTO_BIN,
		.tos = MESH_TOS_P2P,
	};

	return esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

static void ring_store_locked(uint32_t seq, const uint8_t *bytes, size_t len)
{
	replay_slot_t *slot = &s_ring[seq % MESH_V2_REPLAY_RING_SIZE];
	slot->used = true;
	slot->seq = seq;
	slot->len = len;
	memcpy(slot->bytes, bytes, len);
}

static void ring_ack_locked(uint32_t ack_seq)
{
	for (uint32_t i = 0; i < MESH_V2_REPLAY_RING_SIZE; i++) {
		if (s_ring[i].used && s_ring[i].seq <= ack_seq) {
			memset(&s_ring[i], 0, sizeof(s_ring[i]));
		}
	}
}

static bool ring_copy_locked(uint32_t seq, uint8_t *out, size_t *out_len)
{
	replay_slot_t *slot = &s_ring[seq % MESH_V2_REPLAY_RING_SIZE];
	if (!slot->used || slot->seq != seq || !out || !out_len) {
		return false;
	}

	memcpy(out, slot->bytes, slot->len);
	*out_len = slot->len;
	return true;
}

static void new_session_locked(void)
{
	s_session_id = esp_random();
	if (s_session_id == 0) {
		s_session_id = 1;
	}
	s_next_seq = 1;
	memset(s_ring, 0, sizeof(s_ring));
	s_root_ready = false;
	s_last_ack_ms = 0;
}

static esp_err_t send_packet(uint8_t type, const void *payload, size_t payload_len, bool reliable)
{
	if (payload_len > MESH_V2_PAYLOAD_MAX ||
	    sizeof(mesh_v2_hdr_t) + payload_len > MESH_V2_PACKET_MAX) {
		return ESP_ERR_INVALID_SIZE;
	}

	if (reliable && !mesh_v2_node_ready()) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t buf[MESH_V2_PACKET_MAX];
	memset(buf, 0, sizeof(buf));

	mesh_v2_hdr_t *h = (mesh_v2_hdr_t *)buf;
	h->magic = MESH_PKT_MAGIC;
	h->version = MESH_PKT_VERSION_V2;
	h->type = type;
	h->payload_len = (uint16_t)payload_len;
	esp_wifi_get_mac(WIFI_IF_STA, h->src_mac);

	portENTER_CRITICAL(&s_lock);
	h->session_id = s_session_id;
	if (reliable) {
		h->seq = s_next_seq++;
	}
	portEXIT_CRITICAL(&s_lock);

	if (payload && payload_len > 0) {
		memcpy(buf + sizeof(mesh_v2_hdr_t), payload, payload_len);
	}
	h->crc16 = packet_crc(h, buf + sizeof(mesh_v2_hdr_t));

	size_t pkt_len = sizeof(mesh_v2_hdr_t) + payload_len;

	if (reliable) {
		portENTER_CRITICAL(&s_lock);
		ring_store_locked(h->seq, buf, pkt_len);
		portEXIT_CRITICAL(&s_lock);
	}

	return send_raw_to_root(buf, pkt_len);
}

static esp_err_t send_lost(uint32_t missing_seq)
{
	mesh_v2_lost_payload_t p = {
		.missing_seq = missing_seq,
	};
	return send_packet(MESH_V2_TYPE_LOST, &p, sizeof(p), false);
}

static esp_err_t send_hello(bool reset_session)
{
	mesh_v2_hello_payload_t p;
	memset(&p, 0, sizeof(p));

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	if (reset_session) {
		new_session_locked();
	}
	s_last_hello_ms = ms_now();
	portEXIT_CRITICAL(&s_lock);

	p.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
	p.mtu = MESH_V2_PACKET_MAX;
	p.ring_size = MESH_V2_REPLAY_RING_SIZE;
	p.capabilities = 0;

	esp_err_t err = send_packet(MESH_V2_TYPE_HELLO, &p, sizeof(p), false);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "HELLO send failed: %s", esp_err_to_name(err));
	}
	return err;
}

static void recover_root_link_if_needed(void)
{
	uint32_t now = ms_now();
	bool send = false;
	bool reset = false;

	portENTER_CRITICAL(&s_lock);
	{
		bool hello_due = s_last_hello_ms == 0 ||
		                 (uint32_t)(now - s_last_hello_ms) >= MESH_V2_HELLO_RETRY_MS;
		bool ack_stale = s_root_ready &&
		                 (s_last_ack_ms == 0 ||
		                  (uint32_t)(now - s_last_ack_ms) >= MESH_V2_ACK_STALE_MS);

		if ((!s_root_ready && hello_due) || (ack_stale && hello_due)) {
			send = true;
			reset = ack_stale;
		}
	}
	portEXIT_CRITICAL(&s_lock);

	if (send) {
		if (reset) {
			ESP_LOGW(TAG, "root ACK stale, restarting V2 session");
		}
		send_hello(reset);
	}
}

void mesh_v2_node_init(const char *tag)
{
	portENTER_CRITICAL(&s_lock);
	if (!s_inited) {
		new_session_locked();
		s_inited = true;
	}
	copy_tag(s_tag, sizeof(s_tag), tag);
	portEXIT_CRITICAL(&s_lock);
}

void mesh_v2_node_on_mesh_connected(void)
{
	send_hello(true);
}

void mesh_v2_node_on_mesh_disconnected(void)
{
	portENTER_CRITICAL(&s_lock);
	s_root_ready = false;
	s_last_ack_ms = 0;
	portEXIT_CRITICAL(&s_lock);
}

bool mesh_v2_node_ready(void)
{
	bool ready;
	portENTER_CRITICAL(&s_lock);
	ready = s_root_ready;
	portEXIT_CRITICAL(&s_lock);
	return ready;
}

esp_err_t mesh_v2_node_send_nodeinfo(void)
{
	mesh_v2_nodeinfo_payload_t p;
	memset(&p, 0, sizeof(p));

	recover_root_link_if_needed();

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	portEXIT_CRITICAL(&s_lock);

	p.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
	return send_packet(MESH_V2_TYPE_NODEINFO, &p, sizeof(p), true);
}

esp_err_t mesh_v2_node_send_log_line(const char *line)
{
	if (!line) {
		return ESP_ERR_INVALID_ARG;
	}

	mesh_v2_log_line_payload_t p;
	memset(&p, 0, sizeof(p));

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	portEXIT_CRITICAL(&s_lock);

	size_t n = strnlen(line, sizeof(p.line) - 1);
	memcpy(p.line, line, n);
	p.line[n] = '\0';

	return send_packet(MESH_V2_TYPE_LOG_LINE, &p, sizeof(p), true);
}

static void handle_ack(uint32_t session_id, uint32_t ack_seq)
{
	bool became_ready = false;

	portENTER_CRITICAL(&s_lock);
	if (session_id == s_session_id) {
		s_last_ack_ms = ms_now();
		if (!s_root_ready) {
			s_root_ready = true;
			became_ready = true;
		}
		ring_ack_locked(ack_seq);
	}
	portEXIT_CRITICAL(&s_lock);

	if (became_ready) {
		ESP_LOGI(TAG, "root confirmed V2 session=%lu", (unsigned long)session_id);
		mesh_v2_node_send_nodeinfo();
	}
}

static void handle_nack(const mesh_v2_hdr_t *h, const uint8_t *payload)
{
	if (h->payload_len < sizeof(mesh_v2_nack_payload_t)) {
		return;
	}

	const mesh_v2_nack_payload_t *p = (const mesh_v2_nack_payload_t *)payload;
	uint8_t replay[MESH_V2_PACKET_MAX];
	size_t replay_len = 0;
	bool found = false;

	portENTER_CRITICAL(&s_lock);
	if (h->session_id == s_session_id) {
		s_root_ready = true;
		ring_ack_locked(h->ack_seq);
		found = ring_copy_locked(p->missing_seq, replay, &replay_len);
	}
	portEXIT_CRITICAL(&s_lock);

	if (found) {
		mesh_v2_hdr_t *rh = (mesh_v2_hdr_t *)replay;
		rh->flags |= MESH_V2_FLAG_REPLAY;
		rh->crc16 = packet_crc(rh, replay + sizeof(mesh_v2_hdr_t));
		ESP_LOGI(TAG, "replay seq=%lu", (unsigned long)p->missing_seq);
		send_raw_to_root(replay, replay_len);
	} else {
		ESP_LOGW(TAG, "missing seq=%lu is no longer in replay ring",
		         (unsigned long)p->missing_seq);
		send_lost(p->missing_seq);
	}
}

esp_err_t mesh_v2_node_handle_rx(const void *pkt_buf, size_t pkt_len)
{
	const mesh_v2_hdr_t *h = NULL;
	const uint8_t *payload = NULL;

	if (!validate_packet(pkt_buf, pkt_len, &h, &payload)) {
		return ESP_ERR_INVALID_ARG;
	}

	if (h->type == MESH_V2_TYPE_ACK) {
		handle_ack(h->session_id, h->ack_seq);
		return ESP_OK;
	}

	if (h->type == MESH_V2_TYPE_NACK) {
		handle_nack(h, payload);
		return ESP_OK;
	}

	return ESP_OK;
}
