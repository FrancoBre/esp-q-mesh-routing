/*
* MASTER NODE
*/
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <random>
#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>

ESP8266WiFiMulti WiFiMulti;

//#define   STATION_SSID     "Telecentro-40d5"     
//#define   STATION_PASSWORD "RTMHNMMHJMNK"        
#define   STATION_SSID     "Speedy-Fibra-C19C2E" 
#define   STATION_PASSWORD "qazwsxed"            
#define   STATION_PORT     5555
// 192.168.1.56
uint8_t   station_ip[4] =  {192,168,1,56}; // IP of the server

#define   MESH_PREFIX         "whateverYouLike"
#define   MESH_PASSWORD       "somethingSneaky"
#define   MESH_PORT           5555

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
WiFiUDP udp;

// save the most recent q table for sending data
// with the latest learning data
JsonDocument qTable;

String path = "";

void receivedCallback(uint32_t from, String &msg);
void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc);
void sendMessageToServer(String &msg);
void formatPath(String& p);

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

// TODO aca tendríamos que setear la recompensa a +100 y mandar un broadcast a todos los
//  nodos de la red para que actualicen su q table
//  para eso tenemos que implementar que cuando los nodos reciban un mensaje, puedan di-
//  ferenciar si es un mensaje normal, de otro nodo, o un mensaje broadcast del nodo master
void receivedCallback(uint32_t from, String &msg) {
  Serial.print("startHere: Received from ");
  Serial.print(from);
  Serial.print(" msg=");
  Serial.println(msg);
  Serial.flush();

  // Deserialize the JSON message
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    Serial.flush();
    return;
  }

  // Extract q-learning parameters and state information
  float q_alpha = doc["q_parameters"]["alpha"];
  float q_gamma = doc["q_parameters"]["gamma"];
  float q_epsilon = doc["q_parameters"]["epsilon"];
  float q_epsilonDecay = doc["q_parameters"]["epsilon_decay"];

  Serial.println("Extracted Q-learning parameters:");
  Serial.print("alpha: ");
  Serial.print(q_alpha);
  Serial.print(", gamma: ");
  Serial.print(q_gamma);
  Serial.print(", epsilon: ");
  Serial.print(q_epsilon);
  Serial.print(", epsilonDecay: ");
  Serial.println(q_epsilonDecay);
  Serial.flush();

  int current_episode = doc["current_episode"];
  float accumulated_reward = doc["accumulated_reward"];
  float total_time = doc["total_time"];

  Serial.println("Extracted episode information:");
  Serial.print("Current episode: ");
  Serial.print(current_episode);
  Serial.print(", Accumulated reward: ");
  Serial.print(accumulated_reward);
  Serial.print(", Total time: ");
  Serial.println(total_time);
  Serial.flush();

  JsonArray episodes = doc["episodes"];
  for (JsonObject episode : episodes) {
    int episode_number = episode["episode_number"];
    float reward = episode["reward"];
    float time = episode["time"];

    Serial.println("Processing episode:");
    Serial.print("Episode number: ");
    Serial.print(episode_number);
    Serial.print(", Reward: ");
    Serial.print(reward);
    Serial.print(", Time: ");
    Serial.println(time);
    Serial.flush();

    JsonArray steps = episode["steps"];
    for (JsonObject step : steps) {
      int hop = step["hop"];
      String node_from = step["node_from"];
      String node_to = String(mesh.getNodeId());

      Serial.println("Processing step:");
      Serial.print("Hop: ");
      Serial.print(hop);
      Serial.print(", Node from: ");
      Serial.print(node_from);
      Serial.print(", Node to: ");
      Serial.println(node_to);
      Serial.flush();

      // Add reward to episode
      episode["reward"] = episode["reward"] + 100; // Master node found!
      Serial.println("Master node found! Reward increased by 100");
      Serial.flush();

      // Update Q-Table
      updateQTable(node_from, node_to, episode["reward"], q_alpha, q_gamma, doc);

      // TODO broadcast updated q table
      String updatedJsonString;
      serializeJson(doc, updatedJsonString);
      qTable = doc["q_table"];

      // print learning results so the middleware catches them and sends it over 
      // to the server
      Serial.print("Log message with structure: ");
      Serial.println(updatedJsonString);
      //formatPath(path);
      //sendLearningDataToServer(path, updatedJsonString);
    }
  }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc) {
  auto nodes = mesh.getNodeList(true);
  std::vector<int> neighbors;
  String nodesStr;
  int num_neighbors = 0;

  JsonObject q_table = doc["q_table"];
  for (auto &&id : nodes) {
    String node_from = String(id);
    for (auto &&id_2 : nodes) {
      String node_to = String(id_2);

      if (id_2 != id) {
        if (!q_table.containsKey(node_from)) {
          q_table.createNestedObject(node_from);
        }

        if (!q_table[node_to].containsKey(node_to)) {
          q_table[node_from][node_to] = 0.0; // Initialize Q-value if not present
        }
      }
    }
  }

  Serial.println("Q-table:");
  serializeJsonPretty(q_table, Serial);
  Serial.flush();

  // Ensure state_from exists in q_table
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
  }

  // Ensure state_to exists in q_table[state_from]
  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = 0.0; // Initialize Q-value if not present
  }

  // Retrieve Q-value to update
  float currentQ = q_table[state_from][state_to].as<float>();

  // Calculate maxQ for state_to
  float maxQ = -9999.0; // Start with a very low value
  if (q_table.containsKey(state_to)) {
    JsonObject actions = q_table[state_to];
    for (JsonPair kv : actions) {
      float value = kv.value().as<float>();
      if (value > maxQ) {
        maxQ = value;
      }
    }
  }

  // Apply Bellman equation to update Q-value
  float updatedQ = currentQ + alpha * (reward + gamma * maxQ - currentQ);

  // Update Q-value in q_table
  q_table[state_from][state_to] = updatedQ;

  // Log updated Q-table
  Serial.println("Updated Q-table:");
  serializeJsonPretty(q_table, Serial);
  Serial.println();
  Serial.flush();
}

void sendLearningDataToServer(String path, String learningData) {
  Serial.print("Sending final q-learning information to server ");
  Serial.print(": ");
  Serial.println(path);
  Serial.flush();

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  Serial.println(wifiConnected);
  
  if (WiFi.status() == WL_CONNECTED) {
    // IPAddress subnetMask = WiFi.subnetMask();
    // IPAddress gatewayIP = WiFi.gatewayIP();

    // uint32_t subnetMaskRaw = (uint32_t)subnetMask;
    // uint32_t gatewayIPRaw = (uint32_t)gatewayIP;

    // uint32_t broadcastIPRaw = ~subnetMaskRaw | gatewayIPRaw;
    // IPAddress broadcastIp(broadcastIPRaw);
    //
    //IPAddress broadcastIp = ~WiFi.subnetMask() | WiFi.gatewayIP();
    IPAddress broadcastIp;
    broadcastIp = (~uint32_t(WiFi.subnetMask())) | uint32_t(WiFi.gatewayIP());

    Serial.print("Calculated broadcast IP: ");
    Serial.println(broadcastIp.toString());

    udp.beginPacket(broadcastIp, 12345);
    //udp.write(reinterpret_cast<const uint8_t *>(learningData.c_str()), learningData.length());
    udp.write("Hello World");
    udp.endPacket();
  }
}

void formatPath(String& p) {
  // String serverUrl = "https://api.thingspeak.com/update?api_key=" + String(THING_SPEAK_API_KEY) + "&field1=" + tsData;
  p = "192.168.1.56:5000/data";
}

void setup() {
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("Initializing MASTER NODE");

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6 );
  // Setup over the air update support
  mesh.initOTAReceive("bridge");

  mesh.stationManual(STATION_SSID, STATION_PASSWORD, STATION_PORT, station_ip);
  // Bridge node, should (in most cases) be a root node. See [the wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation) for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  mesh.onReceive(&receivedCallback);
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
