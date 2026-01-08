#include "pump_node.h"

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

/* -------------------------------------------------------------------------- */
/*  Константи                                                                 */
/* -------------------------------------------------------------------------- */

static const char *TAG = "pump_node";

/*  таймінги й пороги */

#define CHECK_PERIOD_MS			1000UL		// раз на секунду
#define MAX_PUMP_TIME_MS		3000UL		// помпа максимум 3 c
#define MIN_PAUSE_MS			60000UL		// 1 хв пауза між поливами

#define DRY_VOLTAGE				1.10f		// нижче ≈ сухо
#define WET_VOLTAGE				1.90f		// вище ≈ точно вода

#define DRY_CONFIRM_CYCLES		3			// скільки разів підряд "сухо"
#define WET_CONFIRM_CYCLES		2			// скільки разів підряд "мокро"

#define ADC_VREF				3.3f
#define ADC_MAX_RAW				4095.0f		// 12-біт

/* ---- Захист: якщо ОБИДВА напрямки ≈ 0V — НЕ поливаємо ---- */
#define ZERO_VOLTAGE			0.02f		// 20мВ ~ "реально 0"
#define ZERO_CONFIRM_CYCLES		3			// скільки циклів підряд, щоб вважати fault

/* Для класичного ESP32: GPIO32->ADC1_CH4, GPIO33->ADC1_CH5 */
#define PUMP_ADC_UNIT			ADC_UNIT_1
#define LEVEL_A_CHANNEL			ADC_CHANNEL_4	// GPIO32
#define LEVEL_B_CHANNEL			ADC_CHANNEL_5	// GPIO33

/* -------------------------------------------------------------------------- */
/*  Типи / глобальний контекст                                               */
/* -------------------------------------------------------------------------- */

typedef enum {
	WATER_DRY = 0,
	WATER_WET,
	WATER_UNKNOWN
} water_state_t;

typedef struct {
	gpio_num_t	level_a_gpio;
	gpio_num_t	level_b_gpio;
	gpio_num_t	pump_gpio;

	bool		inited;

	// стан помпи
	bool		pump_on;
	uint32_t	pump_start_ms;
	uint32_t	last_water_ms;

	// гістерезис рівня
	bool		stored_is_full;
	uint8_t		dry_streak;
	uint8_t		wet_streak;

	// fault: обидва виміри ~0V
	uint8_t		zero_streak;

	int			last_level_percent;
} pump_ctx_t;

static pump_ctx_t					s_pump;
static adc_oneshot_unit_handle_t	s_adc = NULL;

/* -------------------------------------------------------------------------- */
/*  Хелпери часу / класифікації                                              */
/* -------------------------------------------------------------------------- */

static inline uint32_t now_ms(void)
{
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static water_state_t classify_voltage(float u)
{
	if (isnan(u)) {
		return WATER_UNKNOWN;
	}
	if (u >= WET_VOLTAGE) {
		return WATER_WET;
	}
	if (u <= DRY_VOLTAGE) {
		return WATER_DRY;
	}
	return WATER_UNKNOWN;
}

/* -------------------------------------------------------------------------- */
/*  Вимірювання напруги A->B та B->A через новий ADC                          */
/* -------------------------------------------------------------------------- */

/**
 * drive_a = true  -> A = 3.3V, міряємо B
 * drive_a = false -> B = 3.3V, міряємо A
 */
static float measure_voltage(bool drive_a)
{
	const int	samples = 10;
	int			sum = 0;

	// Обидва електроди спочатку в hi-Z
	gpio_set_direction(s_pump.level_a_gpio, GPIO_MODE_INPUT);
	gpio_set_direction(s_pump.level_b_gpio, GPIO_MODE_INPUT);

	gpio_num_t	drive_gpio	= drive_a ? s_pump.level_a_gpio : s_pump.level_b_gpio;
	adc_channel_t sense_ch	= drive_a ? LEVEL_B_CHANNEL      : LEVEL_A_CHANNEL;

	// drive -> вихід HIGH
	gpio_set_direction(drive_gpio, GPIO_MODE_OUTPUT);
	gpio_set_level(drive_gpio, 1);

	vTaskDelay(pdMS_TO_TICKS(5));	// стабілізація

	for (int i = 0; i < samples; ++i) {
		int raw = 0;
		esp_err_t err = adc_oneshot_read(s_adc, sense_ch, &raw);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
			gpio_set_direction(drive_gpio, GPIO_MODE_INPUT);
			return NAN;
		}
		sum += raw;
		vTaskDelay(pdMS_TO_TICKS(2));
	}

	// Відпускаємо драйвер назад в hi-Z
	gpio_set_direction(drive_gpio, GPIO_MODE_INPUT);

	float avg_raw  = (float)sum / (float)samples;
	float voltage  = (avg_raw / ADC_MAX_RAW) * ADC_VREF;
	return voltage;
}

/**
 * Читаємо обидва напрями й отримуємо:
 * any_water = хоч один напрям явно "мокрий"
 * all_dry   = обидва напрями явно "сухі"
 * all_zero  = обидва напрями ≈ 0V (fault / обрив / не той пін / ADC=0)
 */
static void get_water_state(bool *any_water, bool *all_dry, bool *all_zero)
{
	float u_ab = measure_voltage(true);		// A -> 3.3V, міряємо B
	float u_ba = measure_voltage(false);	// B -> 3.3V, міряємо A

	water_state_t s_ab = classify_voltage(u_ab);
	water_state_t s_ba = classify_voltage(u_ba);

	bool water_ab = (s_ab == WATER_WET);
	bool water_ba = (s_ba == WATER_WET);
	bool dry_ab   = (s_ab == WATER_DRY);
	bool dry_ba   = (s_ba == WATER_DRY);

	bool zero_ab = (!isnan(u_ab) && (u_ab <= ZERO_VOLTAGE));
	bool zero_ba = (!isnan(u_ba) && (u_ba <= ZERO_VOLTAGE));

	*any_water = (water_ab || water_ba);
	*all_dry   = (dry_ab   && dry_ba);
	*all_zero  = (zero_ab  && zero_ba);

	ESP_LOGI(TAG,
	         "getWaterState(): U_AB=%.3fV(%s)  U_BA=%.3fV(%s)  anyWater=%s allDry=%s allZero=%s",
	         u_ab,
	         s_ab == WATER_WET ? "WET" :
	         s_ab == WATER_DRY ? "DRY" : "UNK",
	         u_ba,
	         s_ba == WATER_WET ? "WET" :
	         s_ba == WATER_DRY ? "DRY" : "UNK",
	         *any_water ? "YES" : "NO",
	         *all_dry   ? "YES" : "NO",
	         *all_zero  ? "YES" : "NO");
}

/* -------------------------------------------------------------------------- */
/*  Один крок логіки автополиву                                              */
/* -------------------------------------------------------------------------- */

static void pump_step(void)
{
	uint32_t now = now_ms();

	bool any_water = false;
	bool all_dry   = false;
	bool all_zero  = false;
	get_water_state(&any_water, &all_dry, &all_zero);

	// ---- Обробка "all_zero" fault ----
	if (all_zero) {
		if (s_pump.zero_streak < 255) s_pump.zero_streak++;
	} else {
		s_pump.zero_streak = 0;
	}

	bool sensor_fault_zero = (s_pump.zero_streak >= ZERO_CONFIRM_CYCLES);

	// Якщо fault — не даємо помпі стартувати (і бажано гасимо, якщо вона вже ON)
	if (sensor_fault_zero) {
		ESP_LOGW(TAG,
		         "SENSOR FAULT: both directions ~0V for %u cycles -> inhibit watering",
		         (unsigned)s_pump.zero_streak);

		// якщо помпа вже ON — вимикаємо з безпеки
		if (s_pump.pump_on) {
			ESP_LOGW(TAG, "Pump OFF due to sensor fault");
			s_pump.pump_on = false;
			gpio_set_level(s_pump.pump_gpio, 0);
			s_pump.last_water_ms = now;		// щоб не дергало часто
		}
		return;
	}

	// оновлюємо стріки рівня (тільки якщо НЕ fault)
	if (any_water) {
		s_pump.wet_streak++;
		s_pump.dry_streak = 0;
	} else if (all_dry) {
		s_pump.dry_streak++;
		s_pump.wet_streak = 0;
	} else {
		s_pump.wet_streak = 0;
		s_pump.dry_streak = 0;
	}

	bool is_full = s_pump.stored_is_full;

	if (s_pump.wet_streak >= WET_CONFIRM_CYCLES) {
		is_full = true;
		s_pump.stored_is_full = true;
		s_pump.wet_streak = WET_CONFIRM_CYCLES;
	}
	if (s_pump.dry_streak >= DRY_CONFIRM_CYCLES) {
		is_full = false;
		s_pump.stored_is_full = false;
		s_pump.dry_streak = DRY_CONFIRM_CYCLES;
	}

	s_pump.last_level_percent = is_full ? 100 : 0;

	ESP_LOGI(TAG,
	         "checkLevel(): isFull=%s pumpOn=%s dtSinceLast=%lu wet=%u dry=%u zero=%u",
	         is_full ? "YES" : "NO",
	         s_pump.pump_on ? "YES" : "NO",
	         (unsigned long)(now - s_pump.last_water_ms),
	         (unsigned)s_pump.wet_streak,
	         (unsigned)s_pump.dry_streak,
	         (unsigned)s_pump.zero_streak);

	// ---- Якщо помпа ВЖЕ увімкнена ----
	if (s_pump.pump_on) {
		// тайм-аут
		if (now - s_pump.pump_start_ms > MAX_PUMP_TIME_MS) {
			ESP_LOGW(TAG, "Pump TIMEOUT -> OFF");
			s_pump.pump_on = false;
			gpio_set_level(s_pump.pump_gpio, 0);
			s_pump.last_water_ms = now;
			return;
		}

		// рівень став FULL -> вимикаємо
		if (is_full) {
			ESP_LOGI(TAG, "Level FULL -> pump OFF");
			s_pump.pump_on = false;
			gpio_set_level(s_pump.pump_gpio, 0);
			s_pump.last_water_ms = now;
		}
		return;
	}

	// ---- Помпа вимкнена – вирішуємо, чи запускати ----

	// 1) Пауза між поливами
	if (now - s_pump.last_water_ms < MIN_PAUSE_MS) {
		ESP_LOGI(TAG, "Too soon since last watering, skip.");
		return;
	}

	// 2) Включаємо помпу, якщо НЕ full
	if (!is_full) {
		ESP_LOGI(TAG, "Level LOW -> pump ON");
		s_pump.pump_on = true;
		s_pump.pump_start_ms = now;
		gpio_set_level(s_pump.pump_gpio, 1);
	} else {
		ESP_LOGI(TAG, "Level FULL by hysteresis, no watering.");
	}
}

/* -------------------------------------------------------------------------- */
/*  Таска                                                                     */
/* -------------------------------------------------------------------------- */

static void pump_node_task(void *arg)
{
	(void)arg;
	for (;;) {
		pump_step();
		vTaskDelay(pdMS_TO_TICKS(CHECK_PERIOD_MS));
	}
}

/* -------------------------------------------------------------------------- */
/*  Публічний API                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t pump_node_init(const pump_node_pins_t *pins)
{
	if (s_pump.inited) {
		return ESP_OK;
	}
	if (!pins) {
		return ESP_ERR_INVALID_ARG;
	}

	memset(&s_pump, 0, sizeof(s_pump));
	s_pump.level_a_gpio = pins->level_a_gpio;
	s_pump.level_b_gpio = pins->level_b_gpio;
	s_pump.pump_gpio    = pins->pump_gpio;

	// Конфіг GPIO помпи
	gpio_config_t pump_conf = {
		.pin_bit_mask = 1ULL << s_pump.pump_gpio,
		.mode         = GPIO_MODE_OUTPUT,
		.pull_up_en   = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&pump_conf));
	gpio_set_level(s_pump.pump_gpio, 0);

	// ADC oneshot unit
	adc_oneshot_unit_init_cfg_t unit_cfg = {
		.unit_id = PUMP_ADC_UNIT,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

	adc_oneshot_chan_cfg_t chan_cfg = {
		.bitwidth = ADC_BITWIDTH_12,
		.atten    = ADC_ATTEN_DB_11,
	};
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, LEVEL_A_CHANNEL, &chan_cfg));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, LEVEL_B_CHANNEL, &chan_cfg));

	// Початковий стан автополиву
	uint32_t now = now_ms();
	s_pump.pump_on        = false;
	s_pump.pump_start_ms  = 0;
	s_pump.last_water_ms  = now - MIN_PAUSE_MS;	// щоби одразу дозволити полив
	s_pump.stored_is_full = false;
	s_pump.wet_streak     = 0;
	s_pump.dry_streak     = DRY_CONFIRM_CYCLES;
	s_pump.zero_streak    = 0;
	s_pump.last_level_percent = 0;

	// Легкий "блінк" помпою як у Arduino
	gpio_set_level(s_pump.pump_gpio, 1);
	vTaskDelay(pdMS_TO_TICKS(200));
	gpio_set_level(s_pump.pump_gpio, 0);

	s_pump.inited = true;

	ESP_LOGI(TAG,
	         "init done (A=GPIO%d, B=GPIO%d, pump=GPIO%d)",
	         (int)s_pump.level_a_gpio,
	         (int)s_pump.level_b_gpio,
	         (int)s_pump.pump_gpio);

	return ESP_OK;
}

void pump_node_start_task(UBaseType_t prio)
{
	static bool started = false;
	if (started) {
		return;
	}
	started = true;

	BaseType_t ok = xTaskCreate(
		pump_node_task,
		"pump_node",
		4096,
		NULL,
		prio,
		NULL
	);
	if (ok != pdPASS) {
		ESP_LOGE(TAG, "failed to create pump_node task");
	}
}

int pump_node_get_last_level_percent(void)
{
	return s_pump.last_level_percent;
}

bool pump_node_is_pump_on(void)
{
	return s_pump.pump_on;
}
