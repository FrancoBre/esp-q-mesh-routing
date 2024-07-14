// Node 2
#include "painlessMesh.h"
// WiFi Credentials
#define   MESH_PREFIX     "AccordingToYou"
#define   MESH_PASSWORD   "somethingSecret"
#define   MESH_PORT       5555 // default number and should be an integer

#define button D4
#define LED D5

bool Bstate = 0; 

// Scheduler is used to synchronize the connection bw other nodes
Scheduler userScheduler; 
painlessMesh  mesh;

void sendMessage() ; // callback func
Task sendTask( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

// everthying should be done in callback func
void sendMessage()
{
  digitalWrite(LED,HIGH); // when start sending
  Serial.println();
  Serial.println("Start Sending....");
  
  if(digitalRead(button) == LOW)
     Bstate = !Bstate; 
  
  
  
  Serial.println("Button State");Serial.println(Bstate);
  
  // Serialize the message
  DynamicJsonDocument doc(1024);
  doc["Button"] = Bstate; 
  String msg ;
  serializeJson(doc, msg); //
  mesh.sendBroadcast( msg );
  Serial.println("Message from Node2");
  Serial.println(msg);
  digitalWrite(LED,LOW); 
}

// Deserialize the message
void receivedCallback( uint32_t from, String &msg ) {

  Serial.println("Message ="); Serial.println(msg);
  String json = msg.c_str();;
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, json);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
  }
  
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
  pinMode(button, INPUT);
  pinMode(LED, OUTPUT);
  
  mesh.setDebugMsgTypes( ERROR | STARTUP );  
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  userScheduler.addTask( sendTask );
  sendTask.enable();
}

void loop() {
  
  mesh.update();
}