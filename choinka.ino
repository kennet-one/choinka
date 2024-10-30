#include "painlessMesh.h"

#define   MESH_PREFIX     "kennet"
#define   MESH_PASSWORD   "kennet123"
#define   MESH_PORT       5555

Scheduler userScheduler; 
painlessMesh  mesh;

void receivedCallback( uint32_t from, String &msg ) {

  String str1 = msg.c_str();
  String str2 = "fito";

  if (str1.equals(str2)) {
    String x = "H9" + String(analogRead(A0)); 
    mesh.sendSingle(624409705,x);
  }
}

void setup() {
  pinMode(A0, INPUT); 

  Serial.begin(9600); 

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
}

void loop() {

  mesh.update();

  Serial.println (analogRead(A0)); 
}


