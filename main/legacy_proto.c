#include "legacy_proto.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>	// atof

static const char *LEG_TAG = "legacy";

/**
 * Прості хелпери для порівняння рядків
 */
static bool streq(const char *a, const char *b)
{
	if (!a || !b) return false;
	return strcmp(a, b) == 0;
}

static bool starts_with(const char *s, const char *prefix)
{
	if (!s || !prefix) return false;
	size_t l = strlen(prefix);
	return strncmp(s, prefix, l) == 0;
}

bool legacy_is_sensor_value(const char *msg)
{
	if (!msg || !msg[0]) return false;

	// старі формати: "TDSB123", "TDS456", "ttds25" і т.п.
	if (starts_with(msg, "TDSB")) return true;
	if (starts_with(msg, "TDS"))  return true;
	if (starts_with(msg, "ttds")) return true;

	return false;
}

void legacy_handle_text(const char *msg)
{
	if (!msg) return;

	// зробимо копію, щоб трохи “почистити” (trim)
	char buf[64];
	strncpy(buf, msg, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	// прибираємо пробіли з кінця
	for (int i = strlen(buf) - 1; i >= 0; --i) {
		if (buf[i] == ' ' || buf[i] == '\r' || buf[i] == '\n' || buf[i] == '\t') {
			buf[i] = '\0';
		} else {
			break;
		}
	}

	if (!buf[0]) return;

	// ---- 1) Команди типу "readtds", "pm1", "pomp", "140" ... ----
	if (streq(buf, "readtds")) {
		ESP_LOGI(LEG_TAG, "[CMD] readtds  (тут включиш TDS state-machine)");
		// TODO: виклик твоєї функції, яка стартує вимір TDS + відправку
		// start_tds_measurement(true);
		return;
	}

	if (streq(buf, "pm1")) {
		ESP_LOGI(LEG_TAG, "[CMD] pm1  (старий режим pmm.state = WAKE)");
		// TODO: виклик твого нового коду, який робить те саме, що старий handleBody("pm1")
		return;
	}

	if (streq(buf, "pomp")) {
		ESP_LOGI(LEG_TAG, "[CMD] pomp  (включити помпу)");
		// TODO: relControl.pimp() еквівалент
		return;
	}

	if (streq(buf, "140") ||
	    streq(buf, "141") ||
	    streq(buf, "142") ||
	    streq(buf, "143")) {
		ESP_LOGI(LEG_TAG, "[CMD] turbo mode = %s", buf);
		// TODO: виставити ту ж туurbo-логіку, що й раніше
		return;
	}

	if (streq(buf, "flow")) {
		ESP_LOGI(LEG_TAG, "[CMD] flow");
		// TODO: relControl.flo();
		return;
	}

	if (streq(buf, "ion")) {
		ESP_LOGI(LEG_TAG, "[CMD] ion");
		// TODO: relControl.ionn();
		return;
	}

	if (streq(buf, "echo_turb")) {
		ESP_LOGI(LEG_TAG, "[CMD] echo_turb");
		// TODO: eho();
		return;
	}

	if (streq(buf, "huOn")) {
		ESP_LOGI(LEG_TAG, "[CMD] huOn (power)");
		// TODO: relControl.power();
		return;
	}

	// ---- 2) Сенсорні значення типу "TDSB123", "TDS456", "ttds25" ----
	if (starts_with(buf, "TDSB")) {
		const char *p = buf + 4;
		float broth_ppm = atof(p);
		ESP_LOGI(LEG_TAG, "[SENSOR] TDS broth @25°C ≈ %.1f ppm", broth_ppm);
		// TODO: зберегти кудись broth_ppm
		return;
	}

	if (starts_with(buf, "TDS")) {
		const char *p = buf + 3;
		float tds_ppm = atof(p);
		ESP_LOGI(LEG_TAG, "[SENSOR] TDS@25°C ≈ %.1f ppm", tds_ppm);
		// TODO: зберегти tds_ppm
		return;
	}

	if (starts_with(buf, "ttds")) {
		const char *p = buf + 4;
		float temp_c = atof(p);
		ESP_LOGI(LEG_TAG, "[SENSOR] temp for TDS ≈ %.2f °C", temp_c);
		// TODO: зберегти temp_c (калібровка / лог)
		return;
	}

	// ---- 3) Якщо нічого не підійшло ----
	ESP_LOGW(LEG_TAG, "unknown legacy msg: \"%s\"", buf);
}
