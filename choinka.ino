// Node ID: 635036282
#include "painlessMesh.h"
#include "mash_parameter.h"

Scheduler userScheduler;
painlessMesh  mesh;


void receivedCallback( uint32_t from, String &msg ) {
  String str1 = msg.c_str();
  String str2 = "tomat0";
  String str3 = "tomat1";

  if (str1.equals(str2)) {
    digitalWrite(4, LOW);
  }

  if (str1.equals(str3)) {
    digitalWrite(4, HIGH);
  }
}

void setup() {
  Serial.begin(115200);
  
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(5);
  digitalWrite(4, LOW);
}

void loop() {
  mesh.update();

}
