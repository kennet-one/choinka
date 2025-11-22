// Node ID: 635036282
#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh mesh;

// -------- Піни ESP8266 --------
const uint8_t LEVEL_A_PIN = 5;   // D1 -> через 100k на електрод 1 (+ 1M на GND)
const uint8_t LEVEL_B_PIN = 0;   // D3 -> через 100k на електрод 2 (+ 1M на GND) [boot-пін]
const uint8_t PUMP_PIN    = 4;   // D2: реле / MOSFET помпи

// -------- Таймінги --------
const unsigned long CHECK_PERIOD    = 3000;    // як часто перевіряти рівень, мс
const unsigned long MAX_PUMP_TIME   = 2000;    // макс. час безперервної роботи помпи, мс (ПІДГОНЯЄШ ПІД СЕБЕ!)
const unsigned long MIN_PAUSE_TIME  = 600000;  // мін. пауза між поливами, мс (10 хв)

// -------- Змінні стану --------
bool          pumpIsOn        = false;
unsigned long pumpStart       = 0;      // коли включили помпу
unsigned long lastWater       = 0;      // коли востаннє рівень був "повний"
int           lastLevelPercent= 0;      // 0 або 100 для tomat0

// -------- Прототипи --------
void   feedback();
void   checkLevel();
void   receivedCallback(uint32_t from, String &msg);
bool   isWaterPresent();
bool   measureOnce(uint8_t drivePin, uint8_t sensePin);

// -------- Таск перевірки рівня --------
Task taskCheckLevel(CHECK_PERIOD, TASK_FOREVER, &checkLevel);

// ----------------------------------------------------
// Вимірювання з AC-чергуванням
// ----------------------------------------------------
// Фізика:
// D1 ---[100k]--- E1 ~~~ вода ~~~ E2 ---[100k]--- D3
// На кожному піні ще ~1M до GND (підстроєчник).
// Логіка:
// - drivePin = OUTPUT HIGH
// - sensePin = INPUT (з підтяжкою 1M до GND в залізі)
// Якщо вода є -> через 100k + вода + 100k підтягне sensePin вгору -> HIGH.
// Якщо води нема -> sensePin стягується своїм 1M на GND -> LOW.
// Ми міряємо в обидва боки й вважаємо воду присутньою, якщо обидва заміри дали HIGH.

bool measureOnce(uint8_t drivePin, uint8_t sensePin) {
	// На старті обидва піни у hi-Z
	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	// drivePin -> вихід HIGH (3.3V)
	pinMode(drivePin, OUTPUT);
	digitalWrite(drivePin, HIGH);

	// sensePin -> вхід, підтяжка до GND є в залізі (1M)
	pinMode(sensePin, INPUT);

	delay(5); // дати стабілізуватись

	int highCount = 0;
	const int samples = 10;

	for (int i = 0; i < samples; i++) {
		int v = digitalRead(sensePin); // 1 = вода, 0 = сухо
		highCount += (v == HIGH) ? 1 : 0;
		delay(2);
	}

	// Відпускаємо drivePin назад в hi-Z
	pinMode(drivePin, INPUT);

	// Якщо більшість читань HIGH -> вода є
	bool water = (highCount >= (samples / 2));

	return water;
}

// Чергуємо A->B, B->A, щоб міняти полярність
bool isWaterPresent() {
	bool w1 = measureOnce(LEVEL_A_PIN, LEVEL_B_PIN); // D1 дає 3.3V, D3 міряє
	bool w2 = measureOnce(LEVEL_B_PIN, LEVEL_A_PIN); // D3 дає 3.3V, D1 міряє

	bool water = (w1 && w2);

	return water;
}

// Повертає останній відомий відсоток (0 або 100) для сумісності з tomat0
int readLevelPercent() {
	return lastLevelPercent;
}

// ----------------------------------------------------
// Mesh feedback
// ----------------------------------------------------
void feedback() {
	String msg = "tomat0=";
	msg += lastLevelPercent;

	// сюди підстав ті ID, які в тебе реально є в мережі
	mesh.sendSingle(624409705, msg);
	mesh.sendSingle(1127818912, msg);
}

// ----------------------------------------------------
// Основна логіка автополиву
// ----------------------------------------------------
void checkLevel() {
	unsigned long now   = millis();
	bool         isFull = isWaterPresent();  // вода дійшла до електродів = "рівень ОК / повний"

	lastLevelPercent = isFull ? 100 : 0;

	// Якщо помпа вже вмикнена – стежимо за таймаутом і рівнем
	if (pumpIsOn) {
		// Аварійне відключення, якщо щось пішло не так (датчик здох, шланг злетів і т.п.)
		if (now - pumpStart > MAX_PUMP_TIME) {
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;  // вважаємо, що робили спробу поливу
			return;
		}

		// Якщо бачимо, що рівень досягнутий – вимикаємо помпу
		if (isFull) {
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;
		}

		return;
	}

	// Якщо помпа вимкнена – вирішуємо, чи запускати

	// 1. Не поливаємо занадто часто
	if (now - lastWater < MIN_PAUSE_TIME) {
		//Serial.println("Too soon since last watering, skip.");
		return;
	}

	// 2. Якщо рівень НЕ повний (води мало біля датчика) – запускаємо помпу
	if (!isFull) {
		pumpIsOn  = true;
		pumpStart = now;
		digitalWrite(PUMP_PIN, HIGH);
	}
}

// ----------------------------------------------------
// Обробка повідомлень Mesh
// ----------------------------------------------------
void receivedCallback(uint32_t from, String &msg) {

	if (msg == "tomat0") {
		feedback();
	}
}

// ----------------------------------------------------
// SETUP
// ----------------------------------------------------
void setup() {
	Serial.begin(115200);
	delay(200);

	// Піна помпи
	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);  // помпа спочатку вимкнена

	// Піни рівня – у високому опорі, все робимо в measureOnce()
	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	// Mesh
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);
	// mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // можна увімкнути для дебагу

	// Таски
	userScheduler.addTask(taskCheckLevel);
	taskCheckLevel.enable();

	// стартові значення, щоб при запуску можна було одразу поливати, якщо треба
	lastWater = millis() - MIN_PAUSE_TIME;

	// Короткий "блік" реле при старті (видно, що живе)
	digitalWrite(PUMP_PIN, HIGH);
	delay(200);
	digitalWrite(PUMP_PIN, LOW);
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------
void loop() {
	mesh.update();
}
