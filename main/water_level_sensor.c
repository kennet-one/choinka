#include "water_level_sensor.h"

#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef ADC_ATTEN_DB_11
#define ADC_ATTEN_DB_11 ADC_ATTEN_DB_12
#endif

#define SENSOR_SAMPLE_COUNT 10
#define SENSOR_APPROX_VREF_MV 3300
#define SENSOR_ADC_MAX_RAW 4095

static const char *TAG = "water_level";

typedef struct {
	bool initialized;
	bool calibration_available;
	bool conversion_warning_logged;
	water_level_sensor_config_t config;
	adc_unit_t unit;
	adc_channel_t channel_a;
	adc_channel_t channel_b;
	adc_oneshot_unit_handle_t adc;
	adc_cali_handle_t calibration;
} water_level_context_t;

static water_level_context_t s_sensor;

static void electrodes_high_impedance(void)
{
	gpio_set_direction(s_sensor.config.electrode_a_gpio, GPIO_MODE_INPUT);
	gpio_set_direction(s_sensor.config.electrode_b_gpio, GPIO_MODE_INPUT);
}

static void electrodes_discharge(void)
{
	gpio_set_direction(s_sensor.config.electrode_a_gpio, GPIO_MODE_OUTPUT);
	gpio_set_direction(s_sensor.config.electrode_b_gpio, GPIO_MODE_OUTPUT);
	gpio_set_level(s_sensor.config.electrode_a_gpio, 0);
	gpio_set_level(s_sensor.config.electrode_b_gpio, 0);
	vTaskDelay(pdMS_TO_TICKS(2));
	electrodes_high_impedance();
}

static void calibration_try_init(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	adc_cali_curve_fitting_config_t curve_config = {
		.unit_id = s_sensor.unit,
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = ADC_BITWIDTH_12,
	};
	if (adc_cali_create_scheme_curve_fitting(&curve_config,
						 &s_sensor.calibration) == ESP_OK) {
		s_sensor.calibration_available = true;
		ESP_LOGI(TAG, "ADC calibration: curve fitting");
		return;
	}
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	adc_cali_line_fitting_config_t line_config = {
		.unit_id = s_sensor.unit,
		.atten = ADC_ATTEN_DB_11,
		.bitwidth = ADC_BITWIDTH_12,
	};
	if (adc_cali_create_scheme_line_fitting(&line_config,
						&s_sensor.calibration) == ESP_OK) {
		s_sensor.calibration_available = true;
		ESP_LOGI(TAG, "ADC calibration: line fitting");
		return;
	}
#endif

	ESP_LOGW(TAG,
		 "ADC calibration unavailable; approximate raw-to-mV fallback is active");
}

static esp_err_t measure_direction(bool drive_a, int *voltage_mv,
				   bool *used_calibration)
{
	if (!voltage_mv || !used_calibration) {
		return ESP_ERR_INVALID_ARG;
	}

	electrodes_high_impedance();
	gpio_num_t drive_gpio = drive_a ? s_sensor.config.electrode_a_gpio
					 : s_sensor.config.electrode_b_gpio;
	adc_channel_t sense_channel = drive_a ? s_sensor.channel_b
					      : s_sensor.channel_a;

	esp_err_t err = gpio_set_direction(drive_gpio, GPIO_MODE_OUTPUT);
	if (err != ESP_OK) {
		return err;
	}
	err = gpio_set_level(drive_gpio, 1);
	if (err != ESP_OK) {
		electrodes_high_impedance();
		return err;
	}
	vTaskDelay(pdMS_TO_TICKS(5));

	int64_t raw_sum = 0;
	for (int i = 0; i < SENSOR_SAMPLE_COUNT; ++i) {
		int raw = 0;
		err = adc_oneshot_read(s_sensor.adc, sense_channel, &raw);
		if (err != ESP_OK) {
			electrodes_discharge();
			return err;
		}
		raw_sum += raw;
		vTaskDelay(pdMS_TO_TICKS(2));
	}

	electrodes_discharge();
	int average_raw = (int)(raw_sum / SENSOR_SAMPLE_COUNT);
	if (s_sensor.calibration_available && s_sensor.calibration) {
		err = adc_cali_raw_to_voltage(s_sensor.calibration, average_raw,
					      voltage_mv);
		if (err == ESP_OK) {
			*used_calibration = true;
			return ESP_OK;
		}
		if (!s_sensor.conversion_warning_logged) {
			s_sensor.conversion_warning_logged = true;
			ESP_LOGW(TAG,
				 "ADC calibration conversion failed: %s; using fallback",
				 esp_err_to_name(err));
		}
	}

	*voltage_mv = (average_raw * SENSOR_APPROX_VREF_MV) / SENSOR_ADC_MAX_RAW;
	*used_calibration = false;
	return ESP_OK;
}

static pump_level_state_t classify_voltage(int voltage_mv)
{
	if (voltage_mv >= s_sensor.config.wet_threshold_mv) {
		return PUMP_LEVEL_WET;
	}
	if (voltage_mv <= s_sensor.config.dry_threshold_mv) {
		return PUMP_LEVEL_DRY;
	}
	return PUMP_LEVEL_UNKNOWN;
}

esp_err_t water_level_sensor_init(const water_level_sensor_config_t *config)
{
	if (!config || !GPIO_IS_VALID_OUTPUT_GPIO(config->electrode_a_gpio) ||
	    !GPIO_IS_VALID_OUTPUT_GPIO(config->electrode_b_gpio) ||
	    config->electrode_a_gpio == config->electrode_b_gpio ||
	    config->dry_threshold_mv >= config->wet_threshold_mv) {
		return ESP_ERR_INVALID_ARG;
	}
	if (s_sensor.initialized) {
		return s_sensor.config.electrode_a_gpio == config->electrode_a_gpio &&
		       s_sensor.config.electrode_b_gpio == config->electrode_b_gpio &&
		       s_sensor.config.dry_threshold_mv == config->dry_threshold_mv &&
		       s_sensor.config.wet_threshold_mv == config->wet_threshold_mv
			       ? ESP_OK
			       : ESP_ERR_INVALID_STATE;
	}

	memset(&s_sensor, 0, sizeof(s_sensor));
	s_sensor.config = *config;
	adc_unit_t unit_a;
	adc_unit_t unit_b;
	esp_err_t err = adc_oneshot_io_to_channel(config->electrode_a_gpio,
						  &unit_a, &s_sensor.channel_a);
	if (err != ESP_OK) {
		return err;
	}
	err = adc_oneshot_io_to_channel(config->electrode_b_gpio, &unit_b,
					&s_sensor.channel_b);
	if (err != ESP_OK) {
		return err;
	}
	if (unit_a != unit_b || unit_a != ADC_UNIT_1) {
		return ESP_ERR_INVALID_ARG;
	}
	s_sensor.unit = unit_a;

	electrodes_high_impedance();
	adc_oneshot_unit_init_cfg_t unit_config = {
		.unit_id = s_sensor.unit,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	err = adc_oneshot_new_unit(&unit_config, &s_sensor.adc);
	if (err != ESP_OK) {
		return err;
	}
	adc_oneshot_chan_cfg_t channel_config = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten = ADC_ATTEN_DB_11,
	};
	err = adc_oneshot_config_channel(s_sensor.adc, s_sensor.channel_a,
					 &channel_config);
	if (err == ESP_OK) {
		err = adc_oneshot_config_channel(s_sensor.adc, s_sensor.channel_b,
						 &channel_config);
	}
	if (err != ESP_OK) {
		adc_oneshot_del_unit(s_sensor.adc);
		s_sensor.adc = NULL;
		return err;
	}

	calibration_try_init();
	s_sensor.initialized = true;
	return ESP_OK;
}

esp_err_t water_level_sensor_read(water_level_snapshot_t *snapshot)
{
	if (!snapshot) {
		return ESP_ERR_INVALID_ARG;
	}
	memset(snapshot, 0, sizeof(*snapshot));
	snapshot->state = PUMP_LEVEL_UNKNOWN;
	if (!s_sensor.initialized) {
		snapshot->error = ESP_ERR_INVALID_STATE;
		return snapshot->error;
	}

	bool calibrated_ab = false;
	bool calibrated_ba = false;
	esp_err_t err = measure_direction(true, &snapshot->voltage_ab_mv,
					  &calibrated_ab);
	if (err == ESP_OK) {
		err = measure_direction(false, &snapshot->voltage_ba_mv,
					&calibrated_ba);
	}
	if (err != ESP_OK) {
		electrodes_high_impedance();
		snapshot->error = err;
		return err;
	}

	pump_level_state_t state_ab = classify_voltage(snapshot->voltage_ab_mv);
	pump_level_state_t state_ba = classify_voltage(snapshot->voltage_ba_mv);
	if (state_ab == PUMP_LEVEL_WET || state_ba == PUMP_LEVEL_WET) {
		snapshot->state = PUMP_LEVEL_WET;
	} else if (state_ab == PUMP_LEVEL_DRY && state_ba == PUMP_LEVEL_DRY) {
		snapshot->state = PUMP_LEVEL_DRY;
	}
	snapshot->calibrated = calibrated_ab && calibrated_ba;
	snapshot->approximate_fallback = !snapshot->calibrated;
	snapshot->error = ESP_OK;
	return ESP_OK;
}
