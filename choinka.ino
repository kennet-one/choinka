#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh mesh;

// -------- Піни ESP32 --------
const uint8_t LEVEL_A_PIN = 32;   // GPIO32 -> через 100k на електрод A (+ ~100k до GND)
const uint8_t LEVEL_B_PIN = 33;   // GPIO33 -> через 100k на електрод B (+ ~100k до GND)
const uint8_t PUMP_PIN    = 26;   // GPIO25: реле / MOSFET помпи (HIGH = помпа ON)

// -------- Таймінги --------
const unsigned long CHECK_PERIOD    = 1000;    // перевірка 1 раз/сек
const unsigned long MAX_PUMP_TIME   = 3000;    // макс. 3 сек безперервної роботи помпи
const unsigned long MIN_PAUSE_TIME  = 60000;   // 1 хв пауза між поливами (потім підкрутиш)

// -------- Пороги для класифікації по напрузі (вольти) --------
//   U <= DRY_VOLTAGE  -> DRY
//   U >= WET_VOLTAGE  -> WET
//   між ними          -> UNKNOWN
const float DRY_VOLTAGE = 0.25f;   // нижче ≈ сухо
const float WET_VOLTAGE = 0.90f;   // вище ≈ точно вода

// -------- Гістерезис по часу --------
const uint8_t DRY_CONFIRM_CYCLES  = 3;   // скільки разів підряд "сухо", щоб вважати LOW
const uint8_t WET_CONFIRM_CYCLES  = 2;   // скільки разів підряд "мокро", щоб вважати FULL

// -------- Змінні стану --------
bool          pumpIsOn         = false;
unsigned long pumpStart        = 0;
unsigned long lastWater        = 0;
int           lastLevelPercent = 0;

// гістерезис рівня
bool     storedIsFull = false;  // останній підтверджений стан (FULL/LOW)
uint8_t  dryStreak    = 0;
uint8_t  wetStreak    = 0;

// -------- ENUM НАВЕРХУ (важливо!) --------
enum WaterState {
	STATE_DRY,
	STATE_WET,
	STATE_UNKNOWN
};

// -------- Прототипи --------
void feedback();
void checkLevel();
void receivedCallback(uint32_t from, String &msg);
float measureVoltage(uint8_t drivePin, uint8_t sensePin);
WaterState classifyVoltage(float u);
void getWaterState(bool &anyWater, bool &allDry);
int readLevelPercent();

// -------- Таск --------
Task taskCheckLevel(CHECK_PERIOD, TASK_FOREVER, &checkLevel);

// ----------------------------------------------------
// АЦП-функція: міряє напругу на sensePin, коли drivePin тримаємо в HIGH
// ----------------------------------------------------
float measureVoltage(uint8_t drivePin, uint8_t sensePin) {
	// Обидва в hi-Z
	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	// drivePin -> вихід HIGH (3.3V)
	pinMode(drivePin, OUTPUT);
	digitalWrite(drivePin, HIGH);

	// sensePin -> вхід (ADC)
	pinMode(sensePin, INPUT);

	delay(5); // стабілізація

	const int samples = 10;
	long sum = 0;

	for (int i = 0; i < samples; i++) {
		int raw = analogRead(sensePin); // 0..4095 на ESP32
		sum += raw;
		delay(2);
	}

	// Відпускаємо драйвер назад в hi-Z
	pinMode(drivePin, INPUT);

	float avgRaw = (float)sum / samples;
	float voltage = (avgRaw / 4095.0f) * 3.3f;
	return voltage;
}

WaterState classifyVoltage(float u) {
	if (u >= WET_VOLTAGE) {
		return STATE_WET;
	}
	if (u <= DRY_VOLTAGE) {
		return STATE_DRY;
	}
	return STATE_UNKNOWN;
}

// ----------------------------------------------------
// Читаємо обидва напрями, видаємо агрегований стан:
// anyWater = хоч один явно "мокрий"
// allDry   = обидва явно "сухі"
// ----------------------------------------------------
void getWaterState(bool &anyWater, bool &allDry) {
	float uAB = measureVoltage(LEVEL_A_PIN, LEVEL_B_PIN);  // A->3.3V, міряємо B
	float uBA = measureVoltage(LEVEL_B_PIN, LEVEL_A_PIN);  // B->3.3V, міряємо A

	WaterState sAB = classifyVoltage(uAB);
	WaterState sBA = classifyVoltage(uBA);

	bool waterAB = (sAB == STATE_WET);
	bool waterBA = (sBA == STATE_WET);
	bool dryAB   = (sAB == STATE_DRY);
	bool dryBA   = (sBA == STATE_DRY);

	anyWater = (waterAB || waterBA);
	allDry   = (dryAB   && dryBA);

	Serial.print("getWaterState(): U_AB=");
	Serial.print(uAB, 3);
	Serial.print("V (");
	Serial.print(sAB == STATE_WET ? "WET" :
	             sAB == STATE_DRY ? "DRY" : "UNK");
	Serial.print(")  U_BA=");
	Serial.print(uBA, 3);
	Serial.print("V (");
	Serial.print(sBA == STATE_WET ? "WET" :
	             sBA == STATE_DRY ? "DRY" : "UNK");
	Serial.print(")  -> anyWater=");
	Serial.print(anyWater ? "YES" : "NO");
	Serial.print("  allDry=");
	Serial.println(allDry ? "YES" : "NO");
}

// ----------------------------------------------------
// tomat0 сумісність
// ----------------------------------------------------
int readLevelPercent() {
	return lastLevelPercent;
}

void feedback() {
	String msg = "tomat0=";
	msg += lastLevelPercent;

	Serial.print("Sending feedback: ");
	Serial.println(msg);

	// Підстав свої реальні ID нод
	mesh.sendSingle(624409705, msg);
	mesh.sendSingle(1127818912, msg);
}

// ----------------------------------------------------
// Автополив з аналоговим датчиком і гістерезисом
// ----------------------------------------------------
void checkLevel() {
	unsigned long now = millis();

	bool anyWater = false;
	bool allDry   = false;
	getWaterState(anyWater, allDry);

	// оновлюємо стріки
	if (anyWater) {
		wetStreak++;
		dryStreak = 0;
	} else if (allDry) {
		dryStreak++;
		wetStreak = 0;
	} else {
		// UNKNOWN на будь-якому каналі — стріки не ростуть
		wetStreak = 0;
		dryStreak = 0;
	}

	// новий isFull з гістерезисом
	bool isFull = storedIsFull;

	if (wetStreak >= WET_CONFIRM_CYCLES) {
		isFull       = true;
		storedIsFull = true;
		wetStreak    = WET_CONFIRM_CYCLES; // обмежуємо
	}
	if (dryStreak >= DRY_CONFIRM_CYCLES) {
		isFull       = false;
		storedIsFull = false;
		dryStreak    = DRY_CONFIRM_CYCLES;
	}

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
	Serial.print(now - lastWater);
	Serial.print("  wetStreak=");
	Serial.print(wetStreak);
	Serial.print("  dryStreak=");
	Serial.println(dryStreak);

	// --------- Якщо помпа ВЖЕ увімкнена ---------
	if (pumpIsOn) {
		// Аварійне відключення
		if (now - pumpStart > MAX_PUMP_TIME) {
			Serial.println("Pump TIMEOUT -> OFF");
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;
			return;
		}

		// Вимикаємо помпу, якщо підтверджено FULL
		if (isFull) {
			Serial.println("Level confirmed FULL -> pump OFF");
			pumpIsOn = false;
			digitalWrite(PUMP_PIN, LOW);
			lastWater = now;
		}

		return;
	}

	// --------- Помпа ВИМКНЕНА – вирішуємо, чи запускати ---------

	// 1. Пауза між поливами
	if (now - lastWater < MIN_PAUSE_TIME) {
		Serial.println("Too soon since last watering, skip.");
		return;
	}

	// 2. Включаємо помпу, якщо підтверджено, що не FULL
	if (!isFull) {
		Serial.println("Level confirmed LOW -> pump ON");
		pumpIsOn  = true;
		pumpStart = now;
		digitalWrite(PUMP_PIN, HIGH);
	} else {
		Serial.println("Level still FULL by hysteresis, no watering.");
	}
}

// ----------------------------------------------------
// Mesh callback
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

	// Налаштування АЦП тільки для ESP32
	#ifdef ARDUINO_ARCH_ESP32
		analogReadResolution(12);           // 0..4095
		analogSetAttenuation(ADC_11db);     // до ~3.3V
	#endif

	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);

	pinMode(LEVEL_A_PIN, INPUT);
	pinMode(LEVEL_B_PIN, INPUT);

	Serial.println("Pins configured. Initial states:");
	Serial.print("LEVEL_A_PIN = ");
	Serial.println(LEVEL_A_PIN);
	Serial.print("LEVEL_B_PIN = ");
	Serial.println(LEVEL_B_PIN);
	Serial.print("PUMP_PIN    = ");
	Serial.println(PUMP_PIN);

	// Mesh
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);

	Serial.println("Mesh initialized.");

	userScheduler.addTask(taskCheckLevel);
	taskCheckLevel.enable();

	// щоб одразу можна було полити, якщо треба
	lastWater    = millis() - MIN_PAUSE_TIME;
	storedIsFull = false;
	wetStreak    = 0;
	dryStreak    = DRY_CONFIRM_CYCLES; // стартово "дозволяємо" полив

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
