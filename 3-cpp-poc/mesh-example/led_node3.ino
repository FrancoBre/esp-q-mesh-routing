// Node 3

#include "painlessMesh.h"

// WiFi Credentials
#define   MESH_PREFIX     "AccordingToYou"
#define   MESH_PASSWORD   "somethingSecret"
#define   MESH_PORT       5555 // default number and should be an integer

#define LED D5

// As we want to read both the messages
int Bstate;
float h, t;


// Scheduler is used to synchronize the connection bw other nodes
Scheduler userScheduler; 
painlessMesh  mesh;


// Deserialize the message
void receivedCallback( uint32_t from, String &msg ) {

  Serial.println("Message from Node3");
  Serial.println("Message ="); Serial.println(msg);
  String json = msg.c_str();;
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
  Bstate = doc["Button"]; 
  t = doc["TEMP"];
  h = doc["HUM"];

  if (t > 27 || h > 50)  
  digitalWrite(LED,HIGH);
  else(digitalWrite(LED,LOW));
  
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}
void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}
void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void setup() {
 
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  
  mesh.setDebugMsgTypes( ERROR | STARTUP );  
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
 
}

void loop() {
  
  mesh.update();
}