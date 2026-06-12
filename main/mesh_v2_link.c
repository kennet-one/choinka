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

#include "mesh_log_stream.h"
#include "mesh_proto.h"

static const char *TAG = "mesh_v2";

typedef struct {
	bool used;
	uint32_t seq;
	size_t len;
	uint8_t bytes[MESH_V2_PACKET_MAX];
} replay_slot_t;

typedef struct {
	uint32_t next_seq;
	replay_slot_t ring[MESH_V2_TUNNEL_REPLAY_RING_SIZE];
} tunnel_tx_channel_t;

typedef struct {
	uint32_t expected_seq;
	uint32_t highest_seen_seq;
	uint32_t last_ack_seq;
} tunnel_rx_channel_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static replay_slot_t s_ring[MESH_V2_REPLAY_RING_SIZE];
static tunnel_tx_channel_t s_tunnel_tx[MESH_V2_TUNNEL_CHANNEL_MAX + 1];
static tunnel_rx_channel_t s_tunnel_rx[MESH_V2_TUNNEL_CHANNEL_MAX + 1];
static char s_tag[MESH_V2_TAG_MAX] = "node";
static uint32_t s_session_id = 0;
static uint32_t s_next_seq = 1;
static bool s_inited = false;
static bool s_root_ready = false;
static uint32_t s_last_ack_ms = 0;
static uint32_t s_last_hello_ms = 0;
static uint8_t s_parent_mac[6] = {0};
static uint8_t s_root_mac[6] = {0};
static uint16_t s_layer = 0;
static uint16_t s_max_layer = 0;
static int8_t s_parent_rssi = -127;
static uint8_t s_child_count = 0;
static uint32_t s_tunnel_gap_count = 0;
static uint32_t s_tunnel_lost_count = 0;
static uint32_t s_tunnel_replay_count = 0;

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

static bool mac_eq(const uint8_t a[6], const uint8_t b[6])
{
	return memcmp(a, b, 6) == 0;
}

static bool mac_is_zero(const uint8_t mac[6])
{
	static const uint8_t zero[6] = {0};
	return mac_eq(mac, zero);
}

static void mac_copy(uint8_t dst[6], const uint8_t src[6])
{
	memcpy(dst, src, 6);
}

static void local_mac(uint8_t mac[6])
{
	esp_wifi_get_mac(WIFI_IF_STA, mac);
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

static void tunnel_ring_store_locked(uint8_t channel, uint32_t seq, const uint8_t *bytes, size_t len)
{
	if (channel == 0 || channel > MESH_V2_TUNNEL_CHANNEL_MAX) {
		return;
	}

	replay_slot_t *slot = &s_tunnel_tx[channel].ring[seq % MESH_V2_TUNNEL_REPLAY_RING_SIZE];
	slot->used = true;
	slot->seq = seq;
	slot->len = len;
	memcpy(slot->bytes, bytes, len);
}

static void tunnel_ring_ack_locked(uint8_t channel, uint32_t ack_seq)
{
	if (channel == 0 || channel > MESH_V2_TUNNEL_CHANNEL_MAX) {
		return;
	}

	for (uint32_t i = 0; i < MESH_V2_TUNNEL_REPLAY_RING_SIZE; i++) {
		replay_slot_t *slot = &s_tunnel_tx[channel].ring[i];
		if (slot->used && slot->seq <= ack_seq) {
			memset(slot, 0, sizeof(*slot));
		}
	}
}

static bool tunnel_ring_copy_locked(uint8_t channel, uint32_t seq, uint8_t *out, size_t *out_len)
{
	if (channel == 0 || channel > MESH_V2_TUNNEL_CHANNEL_MAX) {
		return false;
	}

	replay_slot_t *slot = &s_tunnel_tx[channel].ring[seq % MESH_V2_TUNNEL_REPLAY_RING_SIZE];
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
	memset(s_tunnel_tx, 0, sizeof(s_tunnel_tx));
	memset(s_tunnel_rx, 0, sizeof(s_tunnel_rx));
	for (uint32_t ch = 0; ch <= MESH_V2_TUNNEL_CHANNEL_MAX; ch++) {
		s_tunnel_tx[ch].next_seq = 1;
		s_tunnel_rx[ch].expected_seq = 1;
	}
	s_tunnel_gap_count = 0;
	s_tunnel_lost_count = 0;
	s_tunnel_replay_count = 0;
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
	local_mac(h->src_mac);

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

static esp_err_t send_tunnel_control_packet(uint8_t type, const void *payload, size_t payload_len)
{
	return send_packet(type, payload, payload_len, false);
}

static esp_err_t send_tunnel_lost(uint8_t channel, uint32_t missing_seq)
{
	mesh_v2_tunnel_lost_payload_t p;
	memset(&p, 0, sizeof(p));
	p.channel_id = channel;
	p.missing_seq = missing_seq;
	local_mac(p.origin_mac);
	memset(p.target_mac, 0, sizeof(p.target_mac));
	return send_tunnel_control_packet(MESH_V2_TYPE_TUNNEL_LOST, &p, sizeof(p));
}

static esp_err_t send_tunnel_ack(const mesh_v2_hdr_t *h, const mesh_v2_tunnel_hdr_t *t,
                                 uint32_t ack_seq)
{
	(void)h;
	mesh_v2_tunnel_ack_payload_t p;
	memset(&p, 0, sizeof(p));
	p.channel_id = t->channel_id;
	p.flags = MESH_V2_TUNNEL_FLAG_E2E_ACK | MESH_V2_TUNNEL_FLAG_HOP_ACK;
	p.seq = t->seq;
	p.ack_seq = ack_seq;
	p.sack_bitmap = 0;
	mac_copy(p.origin_mac, t->origin_mac);
	mac_copy(p.target_mac, t->target_mac);
	return send_tunnel_control_packet(MESH_V2_TYPE_TUNNEL_ACK, &p, sizeof(p));
}

static esp_err_t send_tunnel_nack(const mesh_v2_hdr_t *h, const mesh_v2_tunnel_hdr_t *t,
                                  uint32_t missing_seq)
{
	(void)h;
	mesh_v2_tunnel_nack_payload_t p;
	memset(&p, 0, sizeof(p));
	p.channel_id = t->channel_id;
	p.missing_seq = missing_seq;
	mac_copy(p.origin_mac, t->origin_mac);
	mac_copy(p.target_mac, t->target_mac);
	return send_tunnel_control_packet(MESH_V2_TYPE_TUNNEL_NACK, &p, sizeof(p));
}

static esp_err_t send_tunnel_packet(uint8_t channel, const void *inner_payload,
                                    size_t inner_len, bool reliable)
{
	if (channel == 0 || channel > MESH_V2_TUNNEL_CHANNEL_MAX ||
	    inner_len > MESH_V2_TUNNEL_INNER_MAX ||
	    sizeof(mesh_v2_tunnel_hdr_t) + inner_len > MESH_V2_PAYLOAD_MAX) {
		return ESP_ERR_INVALID_ARG;
	}

	if (reliable && !mesh_v2_node_ready()) {
		return ESP_ERR_INVALID_STATE;
	}

	uint8_t payload[MESH_V2_PAYLOAD_MAX];
	memset(payload, 0, sizeof(payload));

	mesh_v2_tunnel_hdr_t *t = (mesh_v2_tunnel_hdr_t *)payload;
	t->channel_id = channel;
	t->flags = MESH_V2_TUNNEL_FLAG_E2E_ACK | MESH_V2_TUNNEL_FLAG_HOP_ACK;
	t->ttl = MESH_V2_TUNNEL_TTL_DEFAULT;
	t->fragment_count = 1;
	t->payload_len = (uint16_t)inner_len;
	t->fragment_index = 0;
	t->stream_id = channel;
	t->ack_seq = 0;
	t->sack_bitmap = 0;
	local_mac(t->origin_mac);
	memset(t->target_mac, 0, sizeof(t->target_mac)); // zero means root.

	portENTER_CRITICAL(&s_lock);
	if (reliable) {
		t->seq = s_tunnel_tx[channel].next_seq++;
	}
	portEXIT_CRITICAL(&s_lock);

	if (inner_payload && inner_len > 0) {
		memcpy(payload + sizeof(*t), inner_payload, inner_len);
	}

	uint8_t buf[MESH_V2_PACKET_MAX];
	memset(buf, 0, sizeof(buf));

	mesh_v2_hdr_t *h = (mesh_v2_hdr_t *)buf;
	h->magic = MESH_PKT_MAGIC;
	h->version = MESH_PKT_VERSION_V2;
	h->type = MESH_V2_TYPE_TUNNEL_DATA;
	h->payload_len = (uint16_t)(sizeof(*t) + inner_len);
	local_mac(h->src_mac);

	portENTER_CRITICAL(&s_lock);
	h->session_id = s_session_id;
	portEXIT_CRITICAL(&s_lock);

	memcpy(buf + sizeof(*h), payload, h->payload_len);
	h->crc16 = packet_crc(h, buf + sizeof(*h));

	size_t pkt_len = sizeof(*h) + h->payload_len;

	if (reliable) {
		portENTER_CRITICAL(&s_lock);
		tunnel_ring_store_locked(channel, t->seq, buf, pkt_len);
		portEXIT_CRITICAL(&s_lock);
	}

	return send_raw_to_root(buf, pkt_len);
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
	p.ring_size = MESH_V2_TUNNEL_REPLAY_RING_SIZE;
	p.capabilities = MESH_V2_CAP_TUNNEL | MESH_V2_CAP_RELAY | MESH_V2_CAP_TOPOLOGY;

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

void mesh_v2_node_update_topology(const uint8_t parent_mac[6],
                                  const uint8_t root_mac[6],
                                  uint16_t layer,
                                  uint16_t max_layer,
                                  int8_t parent_rssi,
                                  uint8_t child_count)
{
	portENTER_CRITICAL(&s_lock);
	if (parent_mac) {
		mac_copy(s_parent_mac, parent_mac);
	}
	if (root_mac) {
		mac_copy(s_root_mac, root_mac);
	}
	s_layer = layer;
	s_max_layer = max_layer;
	s_parent_rssi = parent_rssi;
	s_child_count = child_count;
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
	mesh_v2_tunnel_nodeinfo_payload_t p;
	memset(&p, 0, sizeof(p));

	recover_root_link_if_needed();

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	portEXIT_CRITICAL(&s_lock);

	p.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
	return send_tunnel_packet(MESH_V2_TUNNEL_CHANNEL_NODEINFO, &p, sizeof(p), true);
}

esp_err_t mesh_v2_node_send_log_line(const char *line)
{
	if (!line) {
		return ESP_ERR_INVALID_ARG;
	}

	mesh_v2_tunnel_log_payload_t p;
	memset(&p, 0, sizeof(p));

	recover_root_link_if_needed();

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	portEXIT_CRITICAL(&s_lock);

	size_t n = strnlen(line, sizeof(p.line) - 1);
	memcpy(p.line, line, n);
	p.line[n] = '\0';

	return send_tunnel_packet(MESH_V2_TUNNEL_CHANNEL_LOG, &p, sizeof(p), true);
}

esp_err_t mesh_v2_node_send_topology(void)
{
	mesh_v2_topology_payload_t p;
	memset(&p, 0, sizeof(p));

	recover_root_link_if_needed();

	portENTER_CRITICAL(&s_lock);
	copy_tag(p.tag, sizeof(p.tag), s_tag);
	mac_copy(p.parent_mac, s_parent_mac);
	mac_copy(p.root_mac, s_root_mac);
	p.layer = s_layer;
	p.max_layer = s_max_layer;
	p.parent_rssi = s_parent_rssi;
	p.child_count = s_child_count;
	p.gap_count = s_tunnel_gap_count;
	p.lost_count = s_tunnel_lost_count;
	p.replay_count = s_tunnel_replay_count;
	portEXIT_CRITICAL(&s_lock);

	p.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
	p.capabilities = MESH_V2_CAP_TUNNEL | MESH_V2_CAP_RELAY | MESH_V2_CAP_TOPOLOGY;

	return send_tunnel_packet(MESH_V2_TUNNEL_CHANNEL_TOPOLOGY, &p, sizeof(p), true);
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
		mesh_v2_node_send_topology();
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

static void handle_tunnel_ack(const mesh_v2_hdr_t *h, const uint8_t *payload)
{
	if (h->payload_len < sizeof(mesh_v2_tunnel_ack_payload_t)) {
		return;
	}

	const mesh_v2_tunnel_ack_payload_t *p =
		(const mesh_v2_tunnel_ack_payload_t *)payload;
	if (p->channel_id == 0 || p->channel_id > MESH_V2_TUNNEL_CHANNEL_MAX) {
		return;
	}

	portENTER_CRITICAL(&s_lock);
	if (h->session_id == s_session_id) {
		s_root_ready = true;
		s_last_ack_ms = ms_now();
		tunnel_ring_ack_locked(p->channel_id, p->ack_seq);
	}
	portEXIT_CRITICAL(&s_lock);
}

static void handle_tunnel_nack(const mesh_v2_hdr_t *h, const uint8_t *payload)
{
	if (h->payload_len < sizeof(mesh_v2_tunnel_nack_payload_t)) {
		return;
	}

	const mesh_v2_tunnel_nack_payload_t *p =
		(const mesh_v2_tunnel_nack_payload_t *)payload;
	uint8_t replay[MESH_V2_PACKET_MAX];
	size_t replay_len = 0;
	bool found = false;

	portENTER_CRITICAL(&s_lock);
	if (h->session_id == s_session_id) {
		s_root_ready = true;
		s_last_ack_ms = ms_now();
		found = tunnel_ring_copy_locked(p->channel_id, p->missing_seq, replay, &replay_len);
		if (found) {
			s_tunnel_replay_count++;
		} else {
			s_tunnel_lost_count++;
		}
	}
	portEXIT_CRITICAL(&s_lock);

	if (found) {
		mesh_v2_hdr_t *rh = (mesh_v2_hdr_t *)replay;
		mesh_v2_tunnel_hdr_t *rt =
			(mesh_v2_tunnel_hdr_t *)(replay + sizeof(mesh_v2_hdr_t));
		rt->flags |= MESH_V2_TUNNEL_FLAG_REPLAY;
		rh->crc16 = packet_crc(rh, replay + sizeof(mesh_v2_hdr_t));
		ESP_LOGI(TAG, "tunnel replay ch=%u seq=%lu",
		         (unsigned)p->channel_id, (unsigned long)p->missing_seq);
		send_raw_to_root(replay, replay_len);
	} else {
		ESP_LOGW(TAG, "tunnel missing ch=%u seq=%lu is no longer in replay ring",
		         (unsigned)p->channel_id, (unsigned long)p->missing_seq);
		send_tunnel_lost(p->channel_id, p->missing_seq);
	}
}

static void deliver_tunnel_payload(const mesh_v2_tunnel_hdr_t *t, const uint8_t *payload)
{
	if (t->channel_id != MESH_V2_TUNNEL_CHANNEL_CONTROL) {
		return;
	}

	if (t->payload_len < sizeof(mesh_v2_tunnel_log_ctrl_payload_t)) {
		return;
	}

	const mesh_v2_tunnel_log_ctrl_payload_t *ctrl =
		(const mesh_v2_tunnel_log_ctrl_payload_t *)payload;
	mesh_log_ctrl_packet_t v1;
	memset(&v1, 0, sizeof(v1));
	v1.h.magic = MESH_PKT_MAGIC;
	v1.h.version = MESH_PKT_VERSION;
	v1.h.type = MESH_LOG_TYPE_CTRL;
	v1.enable = ctrl->enable ? 1 : 0;
	mac_copy(v1.h.src_mac, t->origin_mac);
	mesh_log_stream_handle_rx(&v1, sizeof(v1));
}

static void handle_tunnel_data(const mesh_v2_hdr_t *h, const uint8_t *payload)
{
	if (h->session_id != s_session_id || h->payload_len < sizeof(mesh_v2_tunnel_hdr_t)) {
		recover_root_link_if_needed();
		return;
	}

	const mesh_v2_tunnel_hdr_t *t = (const mesh_v2_tunnel_hdr_t *)payload;
	const uint8_t *inner = payload + sizeof(mesh_v2_tunnel_hdr_t);
	size_t inner_space = h->payload_len - sizeof(mesh_v2_tunnel_hdr_t);
	if (t->channel_id == 0 || t->channel_id > MESH_V2_TUNNEL_CHANNEL_MAX ||
	    t->payload_len > inner_space ||
	    t->payload_len > MESH_V2_TUNNEL_INNER_MAX) {
		return;
	}

	uint8_t self[6];
	local_mac(self);
	if (!mac_is_zero(t->target_mac) && !mac_eq(t->target_mac, self)) {
		return;
	}

	bool deliver = false;
	bool send_nack = false;
	uint32_t ack_seq = 0;
	uint32_t missing_seq = 0;

	portENTER_CRITICAL(&s_lock);
	tunnel_rx_channel_t *rx = &s_tunnel_rx[t->channel_id];
	if (t->seq > rx->highest_seen_seq) {
		rx->highest_seen_seq = t->seq;
	}
	if (t->seq == rx->expected_seq) {
		deliver = true;
		rx->expected_seq++;
		rx->last_ack_seq = t->seq;
		ack_seq = t->seq;
		if (rx->expected_seq <= rx->highest_seen_seq) {
			s_tunnel_gap_count++;
			missing_seq = rx->expected_seq;
			send_nack = true;
		}
	} else if (t->seq < rx->expected_seq) {
		ack_seq = rx->expected_seq - 1;
	} else {
		s_tunnel_gap_count++;
		missing_seq = rx->expected_seq;
		ack_seq = rx->expected_seq ? rx->expected_seq - 1 : 0;
		send_nack = true;
	}
	portEXIT_CRITICAL(&s_lock);

	if (send_nack) {
		send_tunnel_nack(h, t, missing_seq);
	} else {
		send_tunnel_ack(h, t, ack_seq);
	}

	if (deliver) {
		deliver_tunnel_payload(t, inner);
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

	if (h->type == MESH_V2_TYPE_TUNNEL_ACK) {
		handle_tunnel_ack(h, payload);
		return ESP_OK;
	}

	if (h->type == MESH_V2_TYPE_TUNNEL_NACK) {
		handle_tunnel_nack(h, payload);
		return ESP_OK;
	}

	if (h->type == MESH_V2_TYPE_TUNNEL_DATA) {
		handle_tunnel_data(h, payload);
		return ESP_OK;
	}

	return ESP_OK;
}
