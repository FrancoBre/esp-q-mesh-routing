/*
 * MASTER NODE
 */
#include "IPAddress.h"

#ifdef ESP8266#include "Hash.h"

#include <ESPAsyncTCP.h>

#else#include <AsyncTCP.h>

#endif#include <ESPAsyncWebServer.h>

#include <ArduinoJson.h>

#include <map>

#include <vector>

#include <random>

#include "painlessMesh.h"

#include <ESP8266WiFi.h>

#include <ESPAsyncTCP.h>

#include <ESPAsyncWebServer.h>

#define STATION_SSID "whateverYouLike"
#define STATION_PASSWORD "somethingSneaky"
#define STATION_PORT 5555

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define HOSTNAME "HTTP_BRIDGE"

#ifdef ENABLE_FEEDBACK_SERVER
AsyncWebServer server(80);
#endif

Scheduler userScheduler; // to control your personal task
painlessMesh mesh;
IPAddress myIP(0, 0, 0, 0);
IPAddress myAPIP(0, 0, 0, 0);
IPAddress getlocalIP();

// save the most recent q table for sending data
// with the latest learning data
JsonDocument broadcastMessage;
bool broadcastMessageToBeSent;

bool initialMessageReceived = false;
unsigned long lastSentMessage = 0;
unsigned long twentySecondsInMillis = 20000;

JsonDocument qTable;
JsonArray episodes;
float accumulatedReward = 0.0;
int currentEpisode = 1;

void sendBroadcastMessage();
void receivedCallback(uint32_t from, String & msg);
void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument & doc);
void sendMessageToServer(String & msg);
void handleReceiveQParameters();
void refreshQTableOnConnectionChange();
IPAddress getlocalIP();

Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, & sendBroadcastMessage);

void logBroadcastMessageDetails() {
  // Measure the size of the broadcastMessage JSON object
  size_t messageSize = measureJsonPretty(broadcastMessage);

  // Get the available heap memory
  size_t freeHeap = ESP.getFreeHeap();
  size_t maxFreeBlock = ESP.getMaxFreeBlockSize();

  // Print the details to the Serial Monitor
  Serial.print("Broadcast message size: ");
  Serial.print(messageSize);
  Serial.println(" bytes");

  Serial.print("Free heap memory: ");
  Serial.print(freeHeap);
  Serial.println(" bytes");

  Serial.print("Max free block size: ");
  Serial.print(maxFreeBlock);
  Serial.println(" bytes");

  Serial.println();
}

void sendBroadcastMessage() {
  // broadcast message from the middleware may take some time to get to the network
  // I'm implementing this as a fallback for that, we'd need to think about a 
  // mechanism for blocking a new send until the sender gets a broadcastMessage
  broadcastMessageToBeSent = (broadcastMessageToBeSent || (millis() - lastSentMessage >= twentySecondsInMillis)) && initialMessageReceived;

  if (broadcastMessageToBeSent) {
    logBroadcastMessageDetails();
    String broadcastMessageString;
    serializeJsonPretty(broadcastMessage, broadcastMessageString);
    Serial.println();
    Serial.println("About to send broadcast");
    Serial.println(broadcastMessageString);

    int retryCount = 0;
    const int maxRetries = 20;

    while (retryCount < maxRetries) {
      if (mesh.sendBroadcast(broadcastMessageString)) {
        broadcastMessageToBeSent = false;
        return;
      }
      retryCount++;
      delay(100);
    }

    Serial.println("Failed to send broadcast message after max retries.");
    broadcastMessageToBeSent = false;
    lastSentMessage = millis();
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.print("New Connection, nodeId = ");
  Serial.println(nodeId);
  refreshQTableOnConnectionChange();
}

void changedConnectionCallback() {
  Serial.println("Connections Changed");
  refreshQTableOnConnectionChange();
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.print("Node Time Adjusted by offset = ");
  Serial.println(offset);
  refreshQTableOnConnectionChange();
}

void refreshQTableOnConnectionChange() {
  auto nodes = mesh.getNodeList(true);
  JsonObject q_table = qTable["q_table"];

  for (auto && id: nodes) {
    bool nodeIsUp = mesh.sendSingle(id, "healthcheck");

    String node_from = String(id);

    if (nodeIsUp) {
      // Si el nodo está activo, verificamos si ya está en la Q-table
      if (!q_table.containsKey(node_from)) {
        q_table.createNestedObject(node_from);
      }

      for (auto && id_2: nodes) {
        String node_to = String(id_2);

        if (id_2 != id) {
          if (!q_table[node_from].containsKey(node_to)) {
            q_table[node_from][node_to] = 0.0;
          }
        }
      }
    } else {
      // Remove node references from q table
      if (q_table.containsKey(node_from)) {
        q_table.remove(node_from);
      }

      for (auto && id_2: nodes) {
        String node_to = String(id_2);

        if (q_table.containsKey(node_to)) {
          if (q_table[node_to].containsKey(node_from)) {
            q_table[node_to].remove(node_from);
          }
        }
      }
    }
  }
}

void receivedCallback(uint32_t from, String & msg) {
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("----------------------------");
  Serial.println("RECEIVED CALLBACK");
  Serial.println("----------------------------");
  Serial.println(ESP.getFreeHeap());
  initialMessageReceived = true;
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());

  // Deserialize the JSON message
  StaticJsonDocument < 1024 > doc;
  DeserializationError error = deserializeJson(doc, msg);

  Serial.println("Received message:");
  serializeJsonPretty(doc, Serial);

  if (error) {
    return;
  }

  // Extract q-learning parameters and state information
  float q_alpha = doc["q_parameters"]["alpha"];
  float q_gamma = doc["q_parameters"]["gamma"];
  float q_epsilon = doc["q_parameters"]["epsilon"];
  float q_epsilonDecay = doc["q_parameters"]["epsilon_decay"];

  int current_episode = doc["current_episode"];
  float accumulated_reward = doc["accumulated_reward"];

  Serial.println("Extracted episode information:");
  serializeJsonPretty(doc["episodes"], Serial);
  Serial.print("Current episode: ");
  Serial.print(current_episode);
  Serial.print(", Accumulated reward: ");
  Serial.print(accumulated_reward);
  Serial.flush();

  episodes = doc["episodes"];
  for (JsonObject episode: episodes) {
    int episode_number = episode["episode_number"];
    if (episode_number == current_episode) {
      float reward = episode["reward"];

      Serial.println("Processing episode:");
      Serial.print("Episode number: ");
      Serial.print(episode_number);
      Serial.print(", Reward: ");
      Serial.print(reward);
      Serial.flush();

      JsonArray steps = episode["steps"];
      for (JsonObject step: steps) {
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

        float accumulated_reward = doc["accumulated_reward"];
        doc["accumulated_reward"] = String(accumulated_reward + 100.0);
        accumulatedReward = doc["accumulated_reward"];

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
        Serial.print("Episode results: ");
        Serial.println(updatedJsonString);

        // flush episodes from json to save memory
        if (current_episode > 10) {
          broadcastMessage["episodes"] = doc.createNestedArray("episodes");
        } else {
          broadcastMessage["episodes"] = doc["episodes"];
        }

        broadcastMessage["q_table"] = doc["q_table"];
        broadcastMessage["alpha"] = doc["q_parameters"]["alpha"];
        broadcastMessage["gamma"] = doc["q_parameters"]["gamma"];
        broadcastMessage["epsilon"] = doc["q_parameters"]["epsilon"];
        broadcastMessage["accumulated_reward"] = accumulatedReward;
        broadcastMessage["broadcast"] = true;
        broadcastMessage["current_episode"] = currentEpisode;
        broadcastMessageToBeSent = true;

        Serial.println("MF current episode to be broadcasted:");
        Serial.println(String(currentEpisode));
        Serial.println("MF message to be broadcasted:");
        serializeJsonPretty(broadcastMessage, Serial);

        qTable = doc["q_table"];

        Serial.println("About to increase current episode counter");
        currentEpisode++;
        Serial.println(String(currentEpisode));
      }
    }
  }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument & doc) {
  auto nodes = mesh.getNodeList(true);

  JsonObject q_table = doc["q_table"];
  for (auto && id: nodes) {
    String node_from = String(id);
    for (auto && id_2: nodes) {
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
  float currentQ = q_table[state_from][state_to].as < float > ();

  // Calculate maxQ for state_to
  float maxQ = 0.0; // Start with zero if no actions are found
  if (q_table.containsKey(state_to)) {
    JsonObject actions = q_table[state_to];
    for (JsonPair kv: actions) {
      float value = kv.value().as < float > ();
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

  /*
    broadcastMessage["q_table"] = q_table;
    broadcastMessage["episodes"] = doc["episodes"];
    broadcastMessage["alpha"] = doc["q_parameters"]["alpha"];
    broadcastMessage["gamma"] = doc["q_parameters"]["gamma"];
    broadcastMessage["epsilon"] = doc["q_parameters"]["epsilon"];
    broadcastMessage["accumulated_reward"] = accumulatedReward;
    broadcastMessage["broadcast"] = true;
    broadcastMessageToBeSent = true;
  */
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

  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, & userScheduler, MESH_PORT);

  mesh.onReceive( & receivedCallback);

  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

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

  #ifdef ENABLE_FEEDBACK_SERVER
  server.on("/update-q-parameters", HTTP_POST, [](AsyncWebServerRequest * request) {
    Serial.println("Received new Q-learning parameters:");

    StaticJsonDocument < 4096 > doc;

    if (request -> hasParam("alpha", true)) {
      String alpha;
      alpha = request -> getParam("alpha", true) -> value();
      doc["alpha"] = alpha;
      Serial.print("alpha: ");
      Serial.print(alpha);
    } else {
      request -> send(400, "text/plain", "Error reading alpha from request");
      return;
    }

    if (request -> hasParam("gamma", true)) {
      String gamma;
      gamma = request -> getParam("gamma", true) -> value();
      doc["gamma"] = gamma;
      Serial.print(", gamma: ");
      Serial.print(gamma);
    } else {
      request -> send(400, "text/plain", "Error reading gamma from request");
      return;
    }

    if (request -> hasParam("epsilon", true)) {
      String epsilon;
      epsilon = request -> getParam("epsilon", true) -> value();
      doc["epsilon"] = epsilon;
      Serial.print(", epsilon: ");
      Serial.println(epsilon);
    } else {
      request -> send(400, "text/plain", "Error reading epsilon from request");
      return;
    }

    doc["episodes"] = episodes;
    Serial.println("Episodes: ");
    serializeJsonPretty(episodes, Serial);

    doc["q_table"] = qTable;
    Serial.println("Q-Table: ");
    serializeJsonPretty(qTable, Serial);

    doc["accumulated_reward"] = accumulatedReward;
    Serial.print("Accumulated reward: ");
    Serial.println(accumulatedReward);

    doc["broadcast"] = true;

    // send over new calculated q parameters along with q table
    String qParametersString;
    serializeJsonPretty(doc, qParametersString);
    Serial.println("About to send mf broadcast");
    mesh.sendBroadcast(qParametersString);

    request -> send(200, "application/json", "{\"status\":\"Parameters updated\"}");
  });
  server.begin();
  Serial.println("Feedback server started on port 80");
  #else
  Serial.println("Feedback server is disabled");
  #endif
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
