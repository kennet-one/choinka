// Node ID: 635036282
#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh mesh;

// -------- Піни ESP8266 --------
const uint8_t LEVEL_A_PIN = 5;   // D1 -> через 100k на електрод 1 (+ ~1M на GND)
const uint8_t LEVEL_B_PIN = 4;   // D2 -> через 100k на електрод 2 (+ ~1M на GND)
const uint8_t PUMP_PIN    = 14;  // D5: реле / MOSFET помпи (HIGH = помпа ON)

// -------- Таймінги --------
const unsigned long CHECK_PERIOD    = 1000;    // перевірка 1 раз/сек
const unsigned long MAX_PUMP_TIME   = 3000;    // макс. 3 сек безперервної роботи помпи
const unsigned long MIN_PAUSE_TIME  = 60000;   // 1 хв пауза між поливами (для тестів можна менше)

// Поріг "води" по кількості HIGH з 10 вимірів
const int WATER_THRESHOLD = 6;    // якщо >=6 з 10 HIGH -> вважаємо, що вода є

// -------- Змінні стану --------
bool          pumpIsOn         = false;
unsigned long pumpStart        = 0;      // коли включили помпу
unsigned long lastWater        = 0;      // коли востаннє рівень був "повний" / полив завершився
int           lastLevelPercent = 0;      // 0 або 100 для tomat0

// -------- Прототипи --------
void feedback();
void checkLevel();
void receivedCallback(uint32_t from, String &msg);
int  measureHighCount(uint8_t drivePin, uint8_t sensePin);

// -------- Таск перевірки рівня --------
Task taskCheckLevel(CHECK_PERIOD, TASK_FOREVER, &checkLevel);

// ----------------------------------------------------
// Вимірювання з AC-чергуванням на D1/D2
// ----------------------------------------------------
// D1 ---[100k]--- Е1 ~~~ вода ~~~ Е2 ---[100k]--- D2
// На кожному піні ще ~1M до GND (підстроєчник).
//
// drivePin = OUTPUT HIGH
// sensePin = INPUT (підтяжка 1M до GND в залізі)
//
// Якщо вода Є -> через 100k + вода + 100k піднімає sensePin -> читаємо HIGH.
// Якщо води НЕМає -> sensePin тягнеться своїм 1M до GND -> LOW.
// ----------------------------------------------------

int measureHighCount(uint8_t drivePin, uint8_t sensePin) {
	// На старті обидва піни у hi-Z
	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	// drivePin -> вихід HIGH (3.3V)
	pinMode(drivePin, OUTPUT);
	digitalWrite(drivePin, HIGH);

	// sensePin -> вхід
	pinMode(sensePin, INPUT);

	delay(5); // дати стабілізуватись

	const int samples = 10;
	int highCount = 0;

	for (int i = 0; i < samples; i++) {
		int v = digitalRead(sensePin); // 1 = вода, 0 = сухо
		if (v == HIGH) {
			highCount++;
		}
		delay(2);
	}

	// Відпускаємо drivePin назад в hi-Z
	pinMode(drivePin, INPUT);

	return highCount;
}

// ----------------------------------------------------
// Основна логіка визначення "вода/сухо" (обидва датчики)
// ----------------------------------------------------
void getWaterState(bool &anyWater, bool &allDry) {
	int highAB = measureHighCount(LEVEL_A_PIN, LEVEL_B_PIN); // D1 -> 3.3V, D2 міряє
	int highBA = measureHighCount(LEVEL_B_PIN, LEVEL_A_PIN); // D2 -> 3.3V, D1 міряє

	bool waterAB = (highAB >= WATER_THRESHOLD);
	bool waterBA = (highBA >= WATER_THRESHOLD);

	anyWater = (waterAB || waterBA);          // хоч один каже "вода"
	allDry   = (!waterAB && !waterBA);        // обидва кажуть "сухо"

	Serial.print("getWaterState(): highAB=");
	Serial.print(highAB);
	Serial.print(" (");
	Serial.print(waterAB ? "WATER" : "DRY");
	Serial.print(")  highBA=");
	Serial.print(highBA);
	Serial.print(" (");
	Serial.print(waterBA ? "WATER" : "DRY");
	Serial.print(")  -> anyWater=");
	Serial.print(anyWater ? "YES" : "NO");
	Serial.print("  allDry=");
	Serial.println(allDry ? "YES" : "NO");
}

// Повертає 0 або 100 для сумісності з tomat0
int readLevelPercent() {
	return lastLevelPercent;
}

// ----------------------------------------------------
// Mesh feedback
// ----------------------------------------------------
void feedback() {
	String msg = "tomat0=";
	msg += lastLevelPercent;

	Serial.print("Sending feedback: ");
	Serial.println(msg);

	mesh.sendSingle(624409705, msg);
	mesh.sendSingle(1127818912, msg);
}

// ----------------------------------------------------
// Основна логіка автополиву
// ----------------------------------------------------
void checkLevel() {
	unsigned long now = millis();

	bool anyWater = false;
	bool allDry   = false;
	getWaterState(anyWater, allDry);

	// isFull = бак вважаємо повним, якщо ХОЧ ДЕ-НЕБУДЬ бачимо воду
	bool isFull = anyWater;
	lastLevelPercent = isFull ? 100 : 0;

	Serial.print("checkLevel(): isFull=");
	Serial.print(isFull ? "YES" : "NO");
	Serial.print("  pumpIsOn=");
	Serial.print(pumpIsOn ? "YES" : "NO");
	Serial.print("  now=");
	Serial.print(now);
	Serial.print("  lastWater=");
	Serial.print(lastWater);
	Serial.print("  dtSinceLastWater=");
	Serial.println(now - lastWater);

	// --------- Якщо помпа ВЖЕ увімкнена ---------
	if (pumpIsOn) {
		// Аварійне відключення, якщо щось пішло не так
		if (now - pumpStart > MAX_PUMP_TIME) {
			Serial.println("Pump TIMEOUT -> OFF");
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;
			return;
		}

		// Якщо ДЕ-НЕБУДЬ бачимо воду – виключаємо помпу (щоб не перелити)
		if (anyWater) {
			Serial.println("Any sensor sees WATER -> pump OFF");
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;
		}

		return;
	}

	// --------- Помпа ВИМКНЕНА – вирішуємо, чи запускати ---------

	// 1. Не поливаємо занадто часто
	if (now - lastWater < MIN_PAUSE_TIME) {
		Serial.println("Too soon since last watering, skip.");
		return;
	}

	// 2. Включаємо помпу ТІЛЬКИ якщо ОБИДВА кажуть "сухо"
	if (allDry) {
		Serial.println("Both sensors DRY -> pump ON");
		pumpIsOn  = true;
		pumpStart = now;
		digitalWrite(PUMP_PIN, HIGH);
	} else {
		Serial.println("Not all sensors DRY yet, no watering.");
	}
}

// ----------------------------------------------------
// Обробка повідомлень Mesh
// ----------------------------------------------------
void receivedCallback(uint32_t from, String &msg) {
	Serial.print("Got from ");
	Serial.print(from);
	Serial.print(": ");
	Serial.println(msg);

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

	Serial.println();
	Serial.println("=== Mint auto-watering node (ESP8266, D1/D2 sensor, D5 pump) start ===");

	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);  // помпа спочатку вимкнена

	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	Serial.println("Pins configured. Initial states:");
	Serial.print("LEVEL_A_PIN = ");
	Serial.println(LEVEL_A_PIN);
	Serial.print("LEVEL_B_PIN = ");
	Serial.println(LEVEL_B_PIN);
	Serial.print("PUMP_PIN    = ");
	Serial.println(PUMP_PIN);

	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);

	Serial.println("Mesh initialized.");

	userScheduler.addTask(taskCheckLevel);
	userScheduler.addTask(taskCheckLevel);
	taskCheckLevel.enable();

	// щоб одразу можна було полити, якщо треба
	lastWater = millis() - MIN_PAUSE_TIME;

	Serial.println("Blink pump relay.");
	digitalWrite(PUMP_PIN, HIGH);
	delay(200);
	digitalWrite(PUMP_PIN, LOW);

	Serial.println("Setup done.");
}

// ----------------------------------------------------
// LOOP
// ----------------------------------------------------
void loop() {
	mesh.update();
}
