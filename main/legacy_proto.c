#include "legacy_proto.h"

#include <string.h>

#include "esp_log.h"

static const char *TAG = "legacy";

static bool starts_with(const char *text, const char *prefix)
{
	return text && prefix && strncmp(text, prefix, strlen(prefix)) == 0;
}

bool legacy_is_sensor_value(const char *message)
{
	return starts_with(message, "TDSB") || starts_with(message, "TDS") ||
	       starts_with(message, "ttds");
}

bool legacy_handle_command(const char *message)
{
	if (!message) {
		return false;
	}

	char text[64];
	strncpy(text, message, sizeof(text) - 1);
	text[sizeof(text) - 1] = '\0';
	for (size_t length = strlen(text); length > 0; --length) {
		char c = text[length - 1];
		if (c != ' ' && c != '\r' && c != '\n' && c != '\t') {
			break;
		}
		text[length - 1] = '\0';
	}
	if (text[0] == '\0') {
		return false;
	}

	/*
	 * This firmware has no safe legacy actuator contract. Returning false is
	 * intentional: the reliable control layer must not cache a fake success.
	 */
	ESP_LOGW(TAG, "unsupported legacy message: \"%s\"", text);
	return false;
}

void legacy_handle_text(const char *message)
{
	(void)legacy_handle_command(message);
}
