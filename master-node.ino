/*
* MASTER NODE
*/
#include "IPAddress.h"
#ifdef ESP8266
#include "Hash.h"
#include <ESPAsyncTCP.h>
#else
#include <AsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <random>
#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>

#define   STATION_SSID     "whateverYouLike"     
#define   STATION_PASSWORD "somethingSneaky"        
//#define   STATION_SSID     "Telecentro-40d5"     
//#define   STATION_PASSWORD "RTMHNMMHJMNK"        
//#define   STATION_SSID     "Speedy-Fibra-C19C2E" 
//#define   STATION_PASSWORD "qazwsxed"   
//#define STATION_SSID "TCH-3490222"
//#define STATION_PASSWORD "5yG2nSZ3uuCC9MTBgY"
#define STATION_PORT 5555

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define HOSTNAME "HTTP_BRIDGE"

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
AsyncWebServer server(80);
IPAddress myIP(0,0,0,0);
IPAddress myAPIP(0,0,0,0);
IPAddress getlocalIP();

// save the most recent q table for sending data
// with the latest learning data
JsonDocument qTable;

String path = "";

void receivedCallback(uint32_t from, String &msg);
void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument &doc);
void sendMessageToServer(String &msg);
void handleReceiveQParameters();
IPAddress getlocalIP();

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

void receivedCallback(uint32_t from, String &msg) {
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
      float updatedReward = ((float) episode["reward"]) + 100.00; // Master node found!
      episode["reward"] = String(updatedReward);
      Serial.println("Master node found! Reward increased by 100");
      Serial.print("Updated reward: ");
      Serial.println(String(episode["reward"]));
      Serial.flush();

      // Update Q-Table
      updateQTable(node_from, node_to, episode["reward"], q_alpha, q_gamma, doc);

      String updatedJsonString;
      serializeJson(doc, updatedJsonString);

      // print learning results so the middleware catches them and sends it over
      // to the server
      Serial.print("Log message with structure: ");
      Serial.println(updatedJsonString);

      // broadcast the final learning data for this episode to all nodes
      qTable = doc["q_table"];
      /*
      String qTableString;
      serializeJsonPretty(qTable, qTableString);
      mesh.sendBroadcast(qTableString);
      */
    }
  }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument &doc) {
    auto nodes = mesh.getNodeList(true);

    JsonObject q_table = doc["q_table"];
    for (auto &&id : nodes) {
        String node_from = String(id);
        for (auto &&id_2 : nodes) {
            String node_to = String(id_2);

            if (id_2 != id) {
                if (!q_table.containsKey(node_from)) {
                    q_table.createNestedObject(node_from);
                }

                if (!q_table[node_from].containsKey(node_to)) {
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
    float maxQ = 0.0; // Start with zero if no actions are found
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

void setup() {
  Serial.begin(9600);
  // Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 10; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  Serial.println("Initializing MASTER NODE");

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);

  mesh.onReceive(&receivedCallback);

  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  // Bridge node, should (in most cases) be a root node. See [the wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation) for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  myAPIP = IPAddress(mesh.getAPIP());

  Serial.println("");
  Serial.println("----------------------------");
  Serial.print("IP address: ");
  Serial.println(myAPIP.toString());
  Serial.println("Make sure to connect to the AP:");
  Serial.print("STATION_SSID: ");
  Serial.println(STATION_SSID);
  Serial.print("STATION_PASSWORD: ");
  Serial.println(STATION_PASSWORD);
  Serial.println("----------------------------");

  server.on("/update-q-parameters", HTTP_POST, [](AsyncWebServerRequest *request){
      Serial.println("Received new Q-learning parameters:");

      StaticJsonDocument<1024> doc;

      if (request->hasParam("alpha", true)) {
        String alpha;
        alpha = request->getParam("alpha", true)->value();
        doc["alpha"] = alpha;
        Serial.print("alpha: ");
        Serial.print(alpha);
      } else {
        request->send(400, "text/plain", "Error reading alpha from request");
        return;
      }

      if (request->hasParam("gamma", true)) {
        String gamma;
        gamma = request->getParam("gamma", true)->value();
        doc["gamma"] = gamma;
        Serial.print(", gamma: ");
        Serial.print(gamma);
      } else {
        request->send(400, "text/plain", "Error reading gamma from request");
        return;
      }

      if (request->hasParam("epsilon", true)) {
        String epsilon;
        epsilon = request->getParam("epsilon", true)->value();
        doc["epsilon"] = epsilon;
        Serial.print(", epsilon: ");
        Serial.println(epsilon);
      } else {
        request->send(400, "text/plain", "Error reading epsilon from request");
        return;
      }

      // send over new calculated q parameters along with q table
      doc["q_table"] = qTable;

      String qParametersString;
      serializeJsonPretty(doc, qParametersString);
      mesh.sendBroadcast(qParametersString);

      request->send(200, "application/json", "{\"status\":\"Parameters updated\"}");
  });
  server.begin();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
