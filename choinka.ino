// Node ID: 635036282
#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh  mesh;

const uint8_t    SENSOR_PIN   = A0;
const int        DRY_VALUE    = 1023;                       
const int        WET_VALUE    = 0; 
const int        THRESHOLD   = 23;    // поріг вологості

void receivedCallback( uint32_t from, String &msg ) {

  if (msg == "tomat0") feedback();
}

int readsens () {
  int raw = analogRead(SENSOR_PIN);                      // 0…1023
  int pct = map(raw, DRY_VALUE, WET_VALUE, 0, 100);      // конвертуємо у % вологості
  pct = constrain(pct, 0, 100);

  return pct;
}

void feedback () {
  int pctt = readsens();
  String pct = "tomat0=";
  pct += pctt;
  mesh.sendSingle(624409705,pct);
  mesh.sendSingle(1127818912,pct);
}

Task taskTurnOffRelay(
  0,               
  TASK_ONCE,          // одноразово
  []() {
    digitalWrite(4, LOW);   // вимикаємо (HIGH — залежить від того, активне LOW у тебе чи HIGH)
    //Serial.println("gsdssf");
  },
  &userScheduler
);

Task taskReadSensor(
  600000,                    // через 10 мін
  TASK_FOREVER,            // нескінченно
  []() {
    int pct = readsens();
    Serial.println(pct);

    if (pct < THRESHOLD) {
      //Serial.println("🌱 Вологості замало — вмикаю реле та запускаю таймер на вимкнення");
      digitalWrite(4, HIGH);
      taskTurnOffRelay.restart();
      taskTurnOffRelay.enableDelayed(2000);
    }
  },
  &userScheduler
);

void setup() {
  Serial.begin(115200);
  
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(5);
  digitalWrite(4, LOW);

  userScheduler.addTask(taskReadSensor);
  userScheduler.addTask(taskTurnOffRelay);
  
  taskReadSensor.enable();
}

void loop() {
  mesh.update();
}
