#include "painlessMesh.h"

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

// Q-Learning Parameters (only for node 1 to start the learning)
String q_alpha = "0.1";  // Learning rate
String q_gamma = "0.9";  // Discount factor
String q_epsilon = "0.1";  // Exploration rate
String q_epsilonDecay = "0.1";  // Exploration rate

painlessMesh mesh;

// User stub
void sendMessage(); // Prototype so PlatformIO doesn't complain
String getNetworkDataToSend();

void sendMessage() {
  mesh.subConnectionJson(true);
  StaticJsonDocument<1024> doc;
  doc["current_node_id"] = String(mesh.getNodeId());

  JsonObject q_parameters = doc.createNestedObject("q_parameters");
  q_parameters["alpha"] = q_alpha;
  q_parameters["gamma"] = q_gamma;
  q_parameters["epsilon"] = q_epsilon;
  q_parameters["epsilon_decay"] = q_epsilonDecay;

  doc["current_episode"] = 1;
  doc["accumulated_reward"] = 0.0;
  doc["total_time"] = 0.0;

  JsonArray episodesArray = doc.createNestedArray("episodes");
  JsonObject episode = episodesArray.createNestedObject();
  episode["episode_number"] = 1;
  episode["reward"] = 0.0;
  episode["time"] = 0.0;

  JsonArray stepsArray = episode.createNestedArray("steps");
  JsonObject step = stepsArray.createNestedObject();
  step["hop"] = 0;
  step["node_from"] = String(mesh.getNodeId());
  // step["node_to"] = String(mesh.getNodeId());

  JsonObject q_table = doc.createNestedObject("q_table");
  JsonObject action = q_table.createNestedObject(String(mesh.getNodeId()));

  String jsonString;
  serializeJsonPretty(doc, jsonString);

  // Send message to first encountered neighbor, for simplicity
  auto nodes = mesh.getNodeList();
  for (auto &&id : nodes) {
    mesh.sendSingle(id, jsonString);
    return;
  }
}

// Needed for painless library
void receivedCallback(uint32_t from, String &msg) {
  Serial.print("startHere: Received from ");
  Serial.print(from);
  Serial.print(" msg=");
  Serial.println(msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.print("--> startHere: New Connection, nodeId = ");
  Serial.println(nodeId);
}

void changedConnectionCallback() {
  Serial.println("Changed connections");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.print("Adjusted time ");
  Serial.print(mesh.getNodeTime());
  Serial.print(". Offset = ");
  Serial.println(offset);
}

void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes(ERROR | STARTUP);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
