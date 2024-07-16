#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

// Q-Learning Parameters (only for node 1 to start the learning)
String q_alpha = "0.1";  // Learning rate
String q_gamma = "0.9";  // Discount factor
String q_epsilon = "0.1";  // Exploration rate
String q_epsilonDecay = "0.1";  // Exploration rate

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;

// User stub
void sendMessage() ; // Prototype so PlatformIO doesn't complain
void printNeighbors();
String getNetworkDataToSend();

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

void sendMessage() {
  StaticJsonDocument<1024> doc;
  doc["node_id"] = mesh.getNodeId();

  JsonObject q_parameters = doc.createNestedObject("q_parameters");
  q_parameters["alpha"] = q_alpha;
  q_parameters["gamma"] = q_gamma;
  q_parameters["epsilon"] = q_epsilon;
  q_parameters["epsilon_decay"] = q_epsilonDecay;

  doc["current_episode"] = 1;
  JsonArray episodesArray = doc.createNestedArray("episodes");
  JsonObject episode = episodesArray.createNestedObject();
  episode["episode_number"] = 1;
  episode["reward"] = 0;

  JsonArray stepsArray = episode.createNestedArray("steps");
  JsonObject step = stepsArray.createNestedObject();
  step["hop"] = 0;
  step["node_visited"] = mesh.getNodeId();

  String jsonString;
  serializeJsonPretty(doc, jsonString);

  mesh.sendBroadcast( jsonString );
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void printNeighbors() {
    Serial.print("NODE 1: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }

    Serial.println();
}

void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
