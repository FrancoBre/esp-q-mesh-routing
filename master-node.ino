/*
 * MASTER NODE
 *
 * Copyright (c) 2024 Franco Brégoli, Pablo Torres,
 * Universidad Nacional de General Sarmiento (UNGS), Buenos Aires, Argentina.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Franco Brégoli <bregolif.fb@gmail.com>,
 *          Pablo Torres <ptorres@campus.ungs.edu.ar>
 *
 * This project is part of our thesis at Universidad Nacional de General
 * Sarmiento (UNGS), and is part of a research initiative to apply reinforcement
 * learning for optimized packet routing in ESP-based mesh networks.
 *
 * The source code for this project is available at:
 * https://github.com/FrancoBre/esp-q-mesh-routing
 */

#include <ESPAsyncWebServer.h>

#include "painlessMesh.h"

// Logging macro
#define LOG(msg)                       \
  Serial.print("[MASTER NODE - Id: "); \
  Serial.print(g_nodeId);              \
  Serial.print(" - ");                 \
  Serial.print(__FUNCTION__);          \
  Serial.print("] ");                  \
  Serial.println(msg);                 \
  Serial.flush();

// Network and mesh configuration
#define MESH_PREFIX "ESP_Q_MESH_ROUTING"
#define MESH_PASSWORD "ESP_Q_MESH_ROUTING"
#define MESH_PORT 5555

#ifdef ENABLE_FEEDBACK_SERVER
#define WIFI_SSID "whateverYouLike"
#define WIFI_PASSWORD "somethingSneaky"
#define HOSTNAME "HTTP_BRIDGE"
#endif

// Timing constants
const unsigned long TWENTY_SECONDS_MILLIS = 20000;
const int MAX_RETRIES = 20;

enum MessageType { PACKET_HOP, BROADCAST, HEALTHCHECK };

enum MasterState { WAITING_FOR_INITIAL_MESSAGE, BROADCAST_MESSAGE_TO_BE_SENT };

// Global variables
MasterState g_masterState = WAITING_FOR_INITIAL_MESSAGE;
StaticJsonDocument<4096> g_broadcastMessage;
StaticJsonDocument<4096> g_qTable;
JsonArray g_episodes;
float g_accumulatedReward = 0.0;
int g_currentEpisode = 1;
unsigned long g_lastSentMessage = 0;
String g_nodeId = "MESH NOT INITIALIZED YET";

// Object declarations
Scheduler userScheduler;
painlessMesh mesh;

#ifdef ENABLE_FEEDBACK_SERVER
IPAddress myAPIP(0, 0, 0, 0);
AsyncWebServer server(80);
#endif

// Function declarations
bool isTimeToSendBroadcast();
void sendInitialBroadcast();
void sendBroadcastMessage();
bool sendMessageWithRetries(const String &msg);
void handleEpisodeFinalization(StaticJsonDocument<1024> &doc);
void updateQTable(const String &state_from, const String &state_to,
                  float reward, float alpha, float gamma,
                  StaticJsonDocument<1024> &doc);
void buildBroadcastMessage(StaticJsonDocument<1024> &doc);
MessageType getMessageType(const String &typeStr);
void refreshQTableOnConnectionChange();
void createNewHop(JsonObject &episode, const String &node_from,
                  const String &next_action, float reward);

Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendBroadcastMessage);

// Main logic functions
void setup() {
  Serial.begin(9600);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing MASTER NODE");

  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);

#ifdef ENABLE_FEEDBACK_SERVER
  mesh.setHostname(HOSTNAME);
#endif
  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
  g_nodeId = String(mesh.getNodeId());

  LOG("Mesh initialized successfully");

#ifdef ENABLE_FEEDBACK_SERVER
  myAPIP = mesh.getAPIP();
  LOG("IP address: " + myAPIP.toString());

  server.on("/update-q-parameters", HTTP_POST,
            [](AsyncWebServerRequest *request) {
              LOG("Received new Q-learning parameters");
              StaticJsonDocument<4096> doc;

              if (request->hasParam("alpha", true))
                doc["alpha"] = request->getParam("alpha", true)->value();
              if (request->hasParam("gamma", true))
                doc["gamma"] = request->getParam("gamma", true)->value();
              if (request->hasParam("epsilon", true))
                doc["epsilon"] = request->getParam("epsilon", true)->value();

              doc["episodes"] = g_episodes;
              doc["q_table"] = g_qTable;
              doc["accumulated_reward"] = g_accumulatedReward;
              doc["broadcast"] = true;

              String qParametersString;
              serializeJsonPretty(doc, qParametersString);
              mesh.sendBroadcast(qParametersString);

              request->send(200, "application/json",
                            "{\"status\":\"Parameters updated\"}");
            });
  server.begin();
  LOG("Feedback server started on port 80");
#else
  LOG("Feedback server is disabled");
#endif
}

void loop() { mesh.update(); }

// Messaging and episode handling
void receivedCallback(uint32_t from, String &msg) {
  LOG("Received message from node " + String(from));

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    LOG("Failed to parse message.");
    return;
  }

  if (getMessageType(doc["type"]) != PACKET_HOP) {
    LOG("Received an unprocessable message type. Ignoring.");
    return;
  }

  handleEpisodeFinalization(doc);
}

void handleEpisodeFinalization(StaticJsonDocument<1024> &doc) {
  float q_alpha, q_gamma, q_epsilon, q_epsilonDecay, accumulated_reward;
  int current_episode;

  extractHyperparameters(doc, q_alpha, q_gamma, q_epsilon, q_epsilonDecay,
                         current_episode, accumulated_reward);

  JsonArray receivedEpisodes = doc["episodes"];
  JsonObject episode = findCurrentEpisode(receivedEpisodes, current_episode);

  if (!episode.isNull()) {
    processEpisodeFinalization(episode, doc, q_alpha, q_gamma,
                               accumulated_reward);
  } else {
    LOG("No matching episode found for current_episode: " +
        String(current_episode));
  }
}

void createNewHop(JsonObject &episode, const String &node_from,
                  const String &next_action, float reward) {
  JsonArray steps;
  if (episode.containsKey("steps")) {
    steps = episode["steps"].as<JsonArray>();
  } else {
    steps = episode.createNestedArray("steps");
    LOG("Steps array was missing. Created new steps array.");
  }

  int hop = steps.size();
  JsonObject newHop = steps.createNestedObject();

  newHop["hop"] = hop;
  newHop["node_from"] = node_from;
  newHop["node_to"] = String(next_action);
  newHop["reward"] = reward;

  LOG("Created new hop from " + node_from + " to " + next_action +
      " with reward: " + String(reward));
}

void processEpisodeFinalization(JsonObject &episode,
                                StaticJsonDocument<1024> &doc, float q_alpha,
                                float q_gamma, float &accumulated_reward) {
  String node_from = doc["current_node_id"];
  String node_to = String(mesh.getNodeId());

  float reward = episode["reward"].as<float>();
  reward += 100.0;

  createNewHop(episode, node_from, node_to, reward);

  String serializedDoc;
  serializeJsonPretty(doc, serializedDoc);
  LOG("aca debería estar el nuevo hop: " + serializedDoc);

  accumulated_reward += reward;
  g_accumulatedReward = accumulated_reward;

  updateQTable(node_from, node_to, reward, q_alpha, q_gamma, doc);

  LOG("Updated reward for step from " + node_from + " to " + node_to + ": " +
      String(reward));

  buildBroadcastMessage(doc);
  g_masterState = BROADCAST_MESSAGE_TO_BE_SENT;

  LOG("Master node found, next broadcast will end the episode and start a new "
      "one");
}

bool isTimeToSendBroadcast() {
  return (g_masterState == BROADCAST_MESSAGE_TO_BE_SENT &&
          millis() - g_lastSentMessage >= TWENTY_SECONDS_MILLIS);
}

void sendBroadcastMessage() {
  if (g_masterState == WAITING_FOR_INITIAL_MESSAGE) {
    sendInitialBroadcast();
    return;
  }

  if (isTimeToSendBroadcast()) {
    String broadcastMessageString;
    serializeJsonPretty(g_broadcastMessage, broadcastMessageString);
    LOG("Sending broadcast message " + broadcastMessageString);

    if (sendMessageWithRetries(broadcastMessageString)) {
      LOG("Broadcast message sent successfully.");
      g_lastSentMessage = millis();
    } else {
      LOG("Failed to send broadcast message after max retries.");
    }
  }
}

void sendInitialBroadcast() {
  LOG("Sending initial broadcast to start episode 1");

  String initialMessage = "{\"type\": \"INITIAL_BROADCAST\"}";
  if (sendMessageWithRetries(initialMessage)) {
    LOG("Initial broadcast sent successfully.");
    g_lastSentMessage = millis();
  } else {
    LOG("Failed to send initial broadcast after max retries.");
  }
}

void buildBroadcastMessage(StaticJsonDocument<1024> &doc) {
  g_broadcastMessage["type"] = "BROADCAST";
  g_broadcastMessage["episodes"] = doc["episodes"];
  g_broadcastMessage["q_table"] = doc["q_table"];
  g_broadcastMessage["alpha"] = doc["hyperparameters"]["alpha"];
  g_broadcastMessage["gamma"] = doc["hyperparameters"]["gamma"];
  g_broadcastMessage["epsilon"] = doc["hyperparameters"]["epsilon"];
  g_broadcastMessage["accumulated_reward"] = g_accumulatedReward;
  g_broadcastMessage["current_episode"] = g_currentEpisode++;

  String broadcastMessageString;
  serializeJsonPretty(g_broadcastMessage, broadcastMessageString);
  LOG("Broadcast message built " + broadcastMessageString);
}

// Hyperparameter and episode management
void extractHyperparameters(StaticJsonDocument<1024> &doc, float &q_alpha,
                            float &q_gamma, float &q_epsilon,
                            float &q_epsilonDecay, int &current_episode,
                            float &accumulated_reward) {
  q_alpha = doc["hyperparameters"]["alpha"].as<float>();
  q_gamma = doc["hyperparameters"]["gamma"].as<float>();
  q_epsilon = doc["hyperparameters"]["epsilon"].as<float>();
  q_epsilonDecay = doc["hyperparameters"]["epsilon_decay"].as<float>();
  current_episode = doc["current_episode"].as<int>();
  accumulated_reward = doc["accumulated_reward"].as<float>();
}

JsonObject findCurrentEpisode(JsonArray &episodes, int current_episode) {
  for (JsonObject episode : episodes) {
    if (episode["episode_number"] == current_episode) {
      return episode;
    }
  }
  return JsonObject();
}

bool sendMessageWithRetries(const String &msg) {
  for (int retryCount = 0; retryCount < MAX_RETRIES; retryCount++) {
    if (mesh.sendBroadcast(msg)) return true;
    delay(100);
  }
  return false;
}

void refreshQTableOnConnectionChange() {
  auto nodes = mesh.getNodeList(true);
  JsonObject q_table = g_qTable["q_table"];

  for (const auto &id : nodes) {
    String node_from = String(id);
    bool nodeIsUp = mesh.sendSingle(id, "healthcheck");

    if (nodeIsUp) {
      if (!q_table.containsKey(node_from)) {
        q_table.createNestedObject(node_from);
      }

      for (const auto &id_2 : nodes) {
        String node_to = String(id_2);
        if (id_2 != id && !q_table[node_from].containsKey(node_to)) {
          q_table[node_from][node_to] = 0.0;
        }
      }
    } else {
      if (q_table.containsKey(node_from)) {
        q_table.remove(node_from);
      }
      for (const auto &id_2 : nodes) {
        String node_to = String(id_2);
        if (q_table.containsKey(node_to) &&
            q_table[node_to].containsKey(node_from)) {
          q_table[node_to].remove(node_from);
        }
      }
    }
  }

  LOG("Q-table refreshed based on connection changes");
}

MessageType getMessageType(const String &typeStr) {
  if (typeStr == "PACKET_HOP") return PACKET_HOP;
  if (typeStr == "BROADCAST") return BROADCAST;
  if (typeStr == "HEALTHCHECK") return HEALTHCHECK;
  return PACKET_HOP;
}

// Q-Learning functions
void updateQTable(const String &state_from, const String &state_to,
                  float reward, float alpha, float gamma,
                  StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  initializeOrUpdateQTable(q_table);

  ensureStateExists(q_table, state_from, state_to);

  updateQValue(q_table, state_from, state_to, reward, alpha, gamma);
}

void initializeOrUpdateQTable(JsonObject &q_table) {
  auto nodes = mesh.getNodeList(true);

  for (auto &&id : nodes) {
    String node_from = String(id);
    for (auto &&id_2 : nodes) {
      String node_to = String(id_2);

      if (id_2 != id) {
        if (!q_table.containsKey(node_from)) {
          q_table.createNestedObject(node_from);
        }

        if (!q_table[node_from].containsKey(node_to)) {
          q_table[node_from][node_to] = 0.0f;
        }
      }
    }
  }
}

void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to) {
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
    LOG("State from " + state_from + " not found in Q-table, initializing...");
  }

  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = 0.0f;
    LOG("State to " + state_to + " not found in Q-table[" + state_from +
        "], initializing with 0.0");
  }
}

float getMaxQValue(JsonObject &q_table, const String &state_to) {
  float maxQ = 0.0f;

  if (q_table.containsKey(state_to)) {
    JsonObject actions = q_table[state_to];
    for (JsonPair kv : actions) {
      float value = kv.value().as<float>();
      if (value > maxQ) {
        maxQ = value;
      }
    }
  }
  LOG("Max Q-value for state_to " + state_to + ": " + String(maxQ));
  return maxQ;
}

void updateQValue(JsonObject &q_table, const String &state_from,
                  const String &state_to, float reward, float alpha,
                  float gamma) {
  float currentQ = q_table[state_from][state_to].as<float>();
  LOG("Current Q-value for Q[" + state_from + "][" + state_to +
      "]: " + String(currentQ));

  float maxQ = getMaxQValue(q_table, state_to);

  float updatedQ = currentQ + alpha * (reward + gamma * maxQ - currentQ);
  LOG("Updated Q-value using Bellman equation: " + String(updatedQ));

  q_table[state_from][state_to] = updatedQ;
  LOG("Q-table updated for Q[" + state_from + "][" + state_to +
      "] = " + String(updatedQ));
}

float getMaxQValue(const String &state) {
  if (!g_qTable.containsKey(state)) {
    return 0.0f;
  }

  float max_q = 0.0f;
  for (JsonPair kv : g_qTable[state].as<JsonObject>()) {
    float q_value = kv.value().as<float>();
    if (q_value > max_q) {
      max_q = q_value;
    }
  }

  return max_q;
}
