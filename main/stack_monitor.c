#include "stack_monitor.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_ota_ops.h"
#include "nvs.h"

#define STACK_MONITOR_MAX_TASKS	25
#define STACK_MONITOR_PERIOD_MS	60000	// Once per 60 seconds.

static const char *TAG = "[STACKMON]";

static void log_ram_snapshot(void)
{
	size_t free_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	size_t min_free_bytes = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
	size_t total_bytes = heap_caps_get_total_size(MALLOC_CAP_8BIT);

	ESP_LOGI(TAG,
			"RAM: free=%u min=%u total=%u bytes",
			(unsigned)free_bytes,
			(unsigned)min_free_bytes,
			(unsigned)total_bytes);
}

static void log_flash_snapshot(void)
{
	uint32_t flash_size = 0;
	uint32_t app_used = 0;
	uint32_t app_slot = 0;
	uint32_t nvs_used = 0;
	uint32_t nvs_free = 0;
	uint32_t nvs_avail = 0;
	uint32_t nvs_total = 0;

	esp_flash_get_size(NULL, &flash_size);

	const esp_partition_t *app_partition = esp_ota_get_running_partition();
	if (app_partition) {
		app_slot = (uint32_t)app_partition->size;

		esp_partition_pos_t app_pos = {
			.offset = app_partition->address,
			.size = app_partition->size,
		};
		esp_image_metadata_t app_meta = {0};
		if (esp_image_get_metadata(&app_pos, &app_meta) == ESP_OK) {
			app_used = (uint32_t)app_meta.image_len;
		}
	}

	nvs_stats_t nvs_stats = {0};
	if (nvs_get_stats(NULL, &nvs_stats) == ESP_OK) {
		nvs_used = (uint32_t)nvs_stats.used_entries;
		nvs_free = (uint32_t)nvs_stats.free_entries;
		nvs_avail = (uint32_t)nvs_stats.available_entries;
		nvs_total = (uint32_t)nvs_stats.total_entries;
	}

	ESP_LOGI(TAG,
			"FLASH: chip=%u app_used=%u app_slot=%u nvs_used=%u nvs_free=%u nvs_avail=%u nvs_total=%u",
			(unsigned)flash_size,
			(unsigned)app_used,
			(unsigned)app_slot,
			(unsigned)nvs_used,
			(unsigned)nvs_free,
			(unsigned)nvs_avail,
			(unsigned)nvs_total);
}

// Main monitoring task.
static void stack_monitor_task(void *arg)
{
	(void)arg;

	// Previous snapshot.
	static TaskStatus_t	prev[STACK_MONITOR_MAX_TASKS];
	static UBaseType_t	prev_count   = 0;
	static bool			have_prev    = false;

	for (;;) {
		TaskStatus_t	cur[STACK_MONITOR_MAX_TASKS];
		UBaseType_t		count       = 0;
		uint32_t		total_time  = 0;

		// Capture all task states.
		count = uxTaskGetSystemState(cur,
				STACK_MONITOR_MAX_TASKS,
				&total_time);

		ESP_LOGI(TAG, "===== STACK MONITOR: %u task(s), slots=%u =====",
				(unsigned)count,
				(unsigned)STACK_MONITOR_MAX_TASKS);

		if (!have_prev) {
			// First pass has no previous snapshot, so CPU is unknown.
			for (UBaseType_t i = 0; i < count; ++i) {
				const char *name = cur[i].pcTaskName;
				if (!name || !name[0]) {
					name = "noname";
				}

				size_t free_words = cur[i].usStackHighWaterMark;
				size_t free_bytes = free_words * sizeof(StackType_t);

				ESP_LOGI(TAG,
						"\"%s\" prio=%u free=%u words (%u bytes), cpu=?",
						name,
						(unsigned)cur[i].uxCurrentPriority,
						(unsigned)free_words,
						(unsigned)free_bytes);

				prev[i] = cur[i];
			}
			prev_count = count;
			have_prev  = true;
		} else {
			// Later passes use ulRunTimeCounter deltas.
			uint64_t dt_total = 0;
			uint64_t dt_idle  = 0;
			uint32_t dt_arr[STACK_MONITOR_MAX_TASKS] = {0};

			// First pass: calculate per-task and total runtime deltas.
			for (UBaseType_t i = 0; i < count; ++i) {
				TaskStatus_t *c = &cur[i];
				uint32_t prev_run = 0;

				// Match this task in the previous snapshot by xTaskNumber.
				for (UBaseType_t j = 0; j < prev_count; ++j) {
					if (prev[j].xTaskNumber == c->xTaskNumber) {
						prev_run = prev[j].ulRunTimeCounter;
						break;
					}
				}

				uint32_t dt = c->ulRunTimeCounter - prev_run;
				dt_arr[i]   = dt;
				dt_total   += dt;

				const char *name = c->pcTaskName ? c->pcTaskName : "";
				if (strcmp(name, "IDLE0") == 0 || strcmp(name, "IDLE1") == 0) {
					dt_idle += dt;
				}
			}

			// Second pass: print stack and CPU percentage for each task.
			for (UBaseType_t i = 0; i < count; ++i) {
				TaskStatus_t *c = &cur[i];
				const char *name = c->pcTaskName;
				if (!name || !name[0]) {
					name = "noname";
				}

				size_t free_words = c->usStackHighWaterMark;
				float cpu_pct = 0.0f;
				if (dt_total > 0) {
					cpu_pct = (float)dt_arr[i] * 100.0f / (float)dt_total;
				}

				ESP_LOGI(TAG,
						"\"%s\" prio=%u free=%u words, cpu=%.1f%%",
						name,
						(unsigned)c->uxCurrentPriority,
						(unsigned)free_words,
						cpu_pct);

				// Update previous snapshot.
				prev[i] = cur[i];
			}
			prev_count = count;

			// Total CPU load excluding idle tasks.
			float cpu_load = 0.0f;
			if (dt_total > 0) {
				cpu_load = (float)(dt_total - dt_idle) * 100.0f / (float)dt_total;
			}

			ESP_LOGI(TAG,
					"CPU: CPU load ~ %.1f%%  (dt_total=%" PRIu64 ", dt_idle=%" PRIu64 ")",
					cpu_load, dt_total, dt_idle);
		}

		log_ram_snapshot();
		log_flash_snapshot();
		ESP_LOGI(TAG, "===== END STACK MONITOR =====");

		vTaskDelay(pdMS_TO_TICKS(STACK_MONITOR_PERIOD_MS));
	}
}

// Public monitor start.
void stack_monitor_start(UBaseType_t priority)
{
	static bool started = false;

	if (started) {
		return;
	}
	started = true;

	BaseType_t ok = xTaskCreate(
			stack_monitor_task,
			"stack_mon",
			6096,			// Monitor stack.
			NULL,
			priority,
			NULL);

	if (ok != pdPASS) {
		ESP_LOGE(TAG, "failed to create stack_monitor task");
	}
}
