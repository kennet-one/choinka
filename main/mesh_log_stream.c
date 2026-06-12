#include "mesh_log_stream.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_mesh.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mesh_proto.h"
#include "mesh_v2_link.h"

static const char *TAG = "mesh_log";

typedef int (*vprintf_like_t)(const char *fmt, va_list ap);

static vprintf_like_t	s_prev_vprintf = NULL;

static bool		s_inited = false;
static bool		s_stream_enabled = false;
static bool		s_mesh_connected = false;
static bool		s_in_hook = false;

static char		s_tag[16] = "node";

static uint32_t		s_cnt = 0;
static TaskHandle_t	s_nodeinfo_task = NULL;

#ifndef LOG_STREAM_STACK_TMP
	#define LOG_STREAM_STACK_TMP	128
#endif

#ifndef LOG_STREAM_HEAP_MAX
	#define LOG_STREAM_HEAP_MAX	256
#endif

#ifndef LOG_STREAM_NODEINFO_PERIOD_MS
	#define LOG_STREAM_NODEINFO_PERIOD_MS	15000
#endif

static void build_time_prefix(char *out, size_t out_sz)
{
	if (!out || out_sz == 0) return;

	time_t now = time(NULL);
	if (now <= 0) {
		snprintf(out, out_sz, "[no-time] ");
		return;
	}

	struct tm tm_now;
	if (!localtime_r(&now, &tm_now)) {
		snprintf(out, out_sz, "[no-time] ");
		return;
	}

	size_t n = strftime(out, out_sz, "[%Y-%m-%d %H:%M:%S] ", &tm_now);
	if (n == 0) snprintf(out, out_sz, "[no-time] ");
}

static void send_nodeinfo_to_root(void)
{
	if (!s_mesh_connected) {
		return;
	}

	(void)mesh_v2_node_send_nodeinfo();

	mesh_nodeinfo_v2_packet_t p;
	memset(&p, 0, sizeof(p));

	p.h.magic = MESH_PKT_MAGIC;
	p.h.version = MESH_PKT_VERSION;
	p.h.type = MESH_LOG_TYPE_NODEINFO;
	p.h.counter = ++s_cnt;

	esp_wifi_get_mac(WIFI_IF_STA, p.h.src_mac);
	strncpy(p.tag, s_tag, sizeof(p.tag) - 1);
	p.tag[sizeof(p.tag) - 1] = '\0';
	p.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);

	mesh_data_t data;
	memset(&data, 0, sizeof(data));
	data.data = (uint8_t *)&p;
	data.size = sizeof(p);
	data.proto = MESH_PROTO_BIN;
	data.tos = MESH_TOS_P2P;

	mesh_addr_t dest;
	memset(&dest, 0, sizeof(dest)); // root

	// Keep V1 NODEINFO alive as a compatibility beacon. It makes the node
	// visible even while V2 is recovering after a root reboot.
	(void)esp_mesh_send(&dest, &data, MESH_DATA_P2P, NULL, 0);
}

static void send_logline_to_root(const char *line)
{
	if (!line) return;
	if (!s_mesh_connected) return;

	mesh_log_line_packet_t p;
	memset(&p, 0, sizeof(p));

	p.h.magic = MESH_PKT_MAGIC;
	p.h.version = MESH_PKT_VERSION;
	p.h.type = MESH_LOG_TYPE_LINE;
	p.h.counter = ++s_cnt;

	esp_wifi_get_mac(WIFI_IF_STA, p.h.src_mac);
	strncpy(p.tag, s_tag, sizeof(p.tag) - 1);
	p.tag[sizeof(p.tag) - 1] = '\0';

	// line already includes time prefix we build here
	strncpy(p.line, line, sizeof(p.line) - 1);
	p.line[sizeof(p.line) - 1] = '\0';

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

static void send_stream_status_to_root(bool enabled)
{
	char tprefix[40];
	char line[96];

	build_time_prefix(tprefix, sizeof(tprefix));
	snprintf(line, sizeof(line),
	         "%sI (0) mesh_log: remote log stream %s",
	         tprefix,
	         enabled ? "ready" : "disabled");
	send_logline_to_root(line);
}

static void nodeinfo_heartbeat_task(void *arg)
{
	(void)arg;

	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(LOG_STREAM_NODEINFO_PERIOD_MS));
		send_nodeinfo_to_root();
	}
}

static int mesh_log_vprintf(const char *fmt, va_list ap)
{
	// 1) Print to UART through the previous sink.
	int ret = 0;
	if (s_prev_vprintf) {
		va_list ap_copy;
		va_copy(ap_copy, ap);
		ret = s_prev_vprintf(fmt, ap_copy);
		va_end(ap_copy);
	}

	// 2) Stop here when remote streaming is disabled.
	if (!s_stream_enabled) return ret;

	// 3) Recursion guard.
	if (s_in_hook) return ret;
	s_in_hook = true;

	char tprefix[40];
	build_time_prefix(tprefix, sizeof(tprefix));

	char stack_buf[LOG_STREAM_STACK_TMP];
	size_t cap = sizeof(stack_buf);

	// prefix
	size_t tlen = strnlen(tprefix, sizeof(tprefix));
	size_t copy_t = (tlen < (cap - 1)) ? tlen : (cap - 1);
	memcpy(stack_buf, tprefix, copy_t);
	stack_buf[copy_t] = '\0';

	// message
	va_list ap_copy2;
	va_copy(ap_copy2, ap);
	int w = vsnprintf(stack_buf + copy_t, cap - copy_t, fmt, ap_copy2);
	va_end(ap_copy2);

	if (w < 0) {
		send_logline_to_root(stack_buf);
		s_in_hook = false;
		return ret;
	}

	if ((size_t)w < (cap - copy_t)) {
		send_logline_to_root(stack_buf);
		s_in_hook = false;
		return ret;
	}

	// Truncated heap fallback.
	size_t need = copy_t + (size_t)w + 1;
	if (need > LOG_STREAM_HEAP_MAX) need = LOG_STREAM_HEAP_MAX;

	char *heap_buf = (char *)malloc(need);
	if (!heap_buf) {
		stack_buf[cap - 2] = '\n';
		stack_buf[cap - 1] = '\0';
		send_logline_to_root(stack_buf);
		s_in_hook = false;
		return ret;
	}

	memcpy(heap_buf, tprefix, copy_t);
	heap_buf[copy_t] = '\0';

	va_list ap_copy3;
	va_copy(ap_copy3, ap);
	vsnprintf(heap_buf + copy_t, need - copy_t, fmt, ap_copy3);
	va_end(ap_copy3);

	send_logline_to_root(heap_buf);
	free(heap_buf);

	s_in_hook = false;
	return ret;
}

void mesh_log_stream_init(const char *tag)
{
	if (s_inited) return;
	s_inited = true;

	if (tag && tag[0]) {
		strncpy(s_tag, tag, sizeof(s_tag) - 1);
		s_tag[sizeof(s_tag) - 1] = '\0';
	}

	s_prev_vprintf = (vprintf_like_t)esp_log_set_vprintf(&mesh_log_vprintf);
	if (!s_nodeinfo_task) {
		xTaskCreate(nodeinfo_heartbeat_task, "nodeinfo_hb", 3072, NULL, 4, &s_nodeinfo_task);
	}
	ESP_LOGI(TAG, "mesh log stream inited (waiting CTRL)");
}

void mesh_log_stream_on_mesh_connected(void)
{
	s_mesh_connected = true;
	send_nodeinfo_to_root();
}

void mesh_log_stream_on_mesh_disconnected(void)
{
	s_mesh_connected = false;
	s_stream_enabled = false;
}

esp_err_t mesh_log_stream_handle_rx(const void *pkt_buf, size_t pkt_len)
{
	if (!pkt_buf || pkt_len < sizeof(mesh_log_ctrl_packet_t)) {
		return ESP_ERR_INVALID_SIZE;
	}

	const mesh_log_ctrl_packet_t *p = (const mesh_log_ctrl_packet_t *)pkt_buf;

	if (p->h.magic != MESH_PKT_MAGIC || p->h.version != MESH_PKT_VERSION) {
		return ESP_ERR_INVALID_ARG;
	}
	if (p->h.type != MESH_LOG_TYPE_CTRL) {
		return ESP_ERR_INVALID_ARG;
	}

	bool enable = (p->enable != 0);
	s_stream_enabled = enable;

	if (enable) {
		send_nodeinfo_to_root();
		send_stream_status_to_root(true);
	}

	// Do not log here: this path is reached from the vprintf hook.
	return ESP_OK;
}
