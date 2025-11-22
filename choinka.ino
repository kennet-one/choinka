// Node ID: 635036282
#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh  mesh;

// ----- Піни ESP8266 -----
const uint8_t LEVEL_PIN = 5;   // D1 на NodeMCU: контакт "бак повний" -> GND
const uint8_t PUMP_PIN  = 4;   // D2 на NodeMCU: реле / MOSFET помпи

// ----- Таймінги -----
const unsigned long CHECK_PERIOD = 500;    // як часто перевіряти рівень, мс
const unsigned long MAX_PUMP_TIME = 60000; // макс. час безперервної роботи помпи, мс

bool pumpIsOn = false;

// ----- Прототипи -----
void feedback();
void checkLevel();
void turnOffPump();
void receivedCallback(uint32_t from, String &msg);

// ----- Таски -----
Task taskCheckLevel(CHECK_PERIOD, TASK_FOREVER, &checkLevel);
Task taskTurnOffPump(MAX_PUMP_TIME, TASK_ONCE, &turnOffPump);

// повертає 0 або 100, щоб залишити старий формат tomat0
int readLevelPercent() {
	bool isFull = (digitalRead(LEVEL_PIN) == LOW); // LOW = замкнуто на GND = бак повний
	return isFull ? 100 : 0;
}

// відправка стану бака на центральні ноди
void feedback() {
	int pct = readLevelPercent();
	String msg = "tomat0=";
	msg += pct;

	// сюди підстав ті ID, які в тебе реально є в мережі
	mesh.sendSingle(624409705, msg);
	mesh.sendSingle(1127818912, msg);
}

// основна логіка автополиву
void checkLevel() {
	bool isFull = (digitalRead(LEVEL_PIN) == LOW);

	// бак НЕ повний -> треба долити
	if (!isFull) {
		if (!pumpIsOn) {
			pumpIsOn = true;
			digitalWrite(PUMP_PIN, HIGH); // вмикаємо помпу
			// запускаємо таймер безпеки
			taskTurnOffPump.restartDelayed();
		}
	}
	// бак повний
	else {
		if (pumpIsOn) {
			turnOffPump();          // вимикаємо помпу
			taskTurnOffPump.disable(); // таймер більше не потрібен
		}
	}
}

// викликається, якщо помпа працює надто довго
void turnOffPump() {
	pumpIsOn = false;
	digitalWrite(PUMP_PIN, LOW); // вимикаємо помпу
}

void receivedCallback(uint32_t from, String &msg) {
	if (msg == "tomat0") {
		feedback();
	}
}

void setup() {
	Serial.begin(115200);

	// Піни
	pinMode(PUMP_PIN, OUTPUT);
	digitalWrite(PUMP_PIN, LOW);      // помпа спочатку вимкнена

	pinMode(LEVEL_PIN, INPUT_PULLUP); // контакт до GND = "бак повний"
	// якщо підеш на інший пін без pull-up — додай зовнішній резистор 10–100 кОм на 3.3V

	// Mesh
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);
	// mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // якщо треба дебаг

	// Таски
	userScheduler.addTask(taskCheckLevel);
	userScheduler.addTask(taskTurnOffPump);

	taskCheckLevel.enable();

	// короткий "блік" помпою / реле при старті
	digitalWrite(PUMP_PIN, HIGH);
	delay(200);
	digitalWrite(PUMP_PIN, LOW);
}

void loop() {
	mesh.update();
}
