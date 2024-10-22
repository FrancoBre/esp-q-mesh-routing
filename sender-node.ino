/*
 * SENDER NODE
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

#include "painlessMesh.h"

// Logging macro
#define LOG(msg)                       \
  Serial.print("[SENDER NODE - Id: "); \
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

// Constants and Hyperparameters
const unsigned long TWENTY_SECONDS_MILLIS = 20000;
const int MAX_RETRIES = 10;
const int MAX_EPISODES = 100;
float g_alpha = 0.1f;         // Learning rate
float g_gamma = 0.9f;         // Discount factor
float g_epsilon = 0.1f;       // Exploration rate
float g_epsilonDecay = 0.1f;  // Exploration decay rate
int g_currentEpisode = 1;

enum MessageType {
  PACKET_HOP,
  BROADCAST,
  HEALTHCHECK,
  CALLBACK,
  UNKNOWN
};

enum NodeState {
  FIRST_TIME,
  PROCESSING_EPISODE,
  EPISODE_FINISHED,
  STUCK_EPISODE,
  EXPLOITATION_PHASE
};

// Global variables
NodeState g_nodeState = FIRST_TIME;
JsonObject g_qTable;
StaticJsonDocument<4096> g_persistentDoc;
JsonArray g_episodes = g_persistentDoc.createNestedArray("episodes");
String g_episodesString;
float g_accumulatedReward = 0.0;
unsigned long g_lastSentMessage = 0;
String g_nodeId = "MESH NOT INITIALIZED YET";

// For implementing full echo
uint32_t g_sendTimestamp;
String g_previousNode;
float g_estimatedTimeForCallback;

// Object declarations
Scheduler userScheduler;
painlessMesh mesh;

// Function declarations
void setup();
void loop();
void receivedMessage(uint32_t from, String &msg);
void maybeStartNewEpisode();
bool isTimeToManuallyStartNewEpisode();
void startNewEpisode();
void buildMessage(StaticJsonDocument<1024> &doc, String next_action);
void handlePacketHop(StaticJsonDocument<1024> &doc);
void handleBroadcast(StaticJsonDocument<1024> &doc);
void handleEchoCallback(uint32_t from, StaticJsonDocument<1024> &doc);
void handleHealthCheck();
String getMessageTypeString(MessageType type);
MessageType getMessageType(const String &typeStr);
int chooseAction();
int chooseBestAction(const JsonObject &actions,
                     const std::vector<int> &neighbors);
bool sendMessageWithRetries(uint32_t next_hop, String &msg);
unsigned long getSyncedTimeInMs();
void extractHyperparameters(StaticJsonDocument<1024> &doc);
void createNewHop(JsonObject &episode, const String &node_from,
                  const String &next_action);
bool prepareAndSendMessage(StaticJsonDocument<1024> &doc,
                           const String &next_action);
JsonObject findCurrentEpisode(JsonArray &episodes, int current_episode);
void processEpisode(JsonObject &episode, StaticJsonDocument<1024> &doc);
void updateQTableWithIncompleteInformation(int next_action,
                                           const String &node_from,
                                           const String &node_to,
                                           float estimated_time_remaining,
                                           float alpha, float gamma,
                                           StaticJsonDocument<1024> &doc);
float estimateRemainingTime(const String &current_node,
                            StaticJsonDocument<1024> &doc);

// Main logic functions
Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &maybeStartNewEpisode);

void setup() {
  Serial.begin(9600);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing SENDER NODE");

  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedMessage);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  g_nodeId = String(mesh.getNodeId());

  LOG("Mesh initialized successfully");
}

void loop() { mesh.update(); }

void startNewEpisode() {
  LOG("Starting episode: " + String(g_currentEpisode));
  mesh.subConnectionJson(true);

  int next_action = chooseAction();
  if (next_action == -1) {
    LOG("No valid next action");
    return;
  }

  StaticJsonDocument<1024> doc;
  buildMessage(doc, String(next_action));

  String jsonString;
  serializeJsonPretty(doc, jsonString);
  LOG("Data to send: " + jsonString);

  if (!sendMessageWithRetries(next_action, jsonString)) {
    LOG("Failed to send hop after max retries");
  } else {
    LOG("Message sent successfully");
    g_nodeState = PROCESSING_EPISODE;
  }
}

void maybeStartNewEpisode() {
  if (g_nodeState == FIRST_TIME || isTimeToManuallyStartNewEpisode()) {
    startNewEpisode();
  }
}

bool isTimeToManuallyStartNewEpisode() {
  return (getSyncedTimeInMs() - g_lastSentMessage >= TWENTY_SECONDS_MILLIS);
}


// Messaging and episode handling
void receivedMessage(uint32_t from, String &msg) {
  LOG("Received message from " + String(from) + ": " + msg);

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    LOG("Failed to parse message: ");
    Serial.println(error.c_str());
    return;
  }

  MessageType msgType = getMessageType(doc["type"]);
  switch (msgType) {
    case PACKET_HOP:
      handlePacketHop(doc);
      break;
    case BROADCAST:
      handleBroadcast(doc);
      break;
    case HEALTHCHECK:
      handleHealthCheck();
      break;
    case CALLBACK:
      handleEchoCallback(from, doc);
      break;
    default:
      LOG("Received an unprocessable message type. Ignoring.");
      break;
  }
}

void handlePacketHop(StaticJsonDocument<1024> &doc) {
  LOG("Processing PACKET_HOP");

  extractHyperparameters(doc);

  int current_episode = doc["current_episode"];
  g_accumulatedReward = doc["accumulated_reward"];

  JsonArray receivedEpisodes = doc["episodes"];
  JsonObject episode = findCurrentEpisode(receivedEpisodes, current_episode);

  if (!episode.isNull()) {
    processEpisode(episode, doc);
  } else {
    LOG("No matching episode found for current_episode: " +
        String(current_episode));
  }
}

void processEpisode(JsonObject &episode, StaticJsonDocument<1024> &doc) {
  String node_from = doc["current_node_id"];
  String node_to = String(g_nodeId);

  int next_action = chooseAction();
  if (next_action == -1) {
    return;
  }

  float estimated_time_remaining = estimateRemainingTime(String(next_action), doc);

  updateQTableWithIncompleteInformation(next_action, node_from, node_to,
                                        estimated_time_remaining, g_alpha,
                                        g_gamma, doc);

  createNewHop(episode, node_from, node_to);

  doc["type"] = "PACKET_HOP";
  if (prepareAndSendMessage(doc, String(next_action))) {
    g_previousNode = String(node_from);
    g_sendTimestamp = getSyncedTimeInMs();
    g_estimatedTimeForCallback = estimated_time_remaining;
  }
}

void handleEchoCallback(uint32_t from, StaticJsonDocument<1024> &doc) {
  float transmission_time = getSyncedTimeInMs() - g_sendTimestamp;

  String node_from = g_nodeId;
  String node_to = String(from);

  updateQValueWithLatency(node_from, node_to, transmission_time,
                          g_estimatedTimeForCallback, g_alpha, doc);

  // Save episodes for new episode
  if (doc.containsKey("episodes")) {
    serializeJsonPretty(doc["episodes"], g_episodesString);
    deserializeJson(g_episodes, g_episodesString);
  }

  g_currentEpisode++;
  startNewEpisode();
}

void handleBroadcast(StaticJsonDocument<1024> &doc) {
  LOG("Broadcast processed successfully");
}

void buildMessage(StaticJsonDocument<1024> &doc, String next_action) {
  doc["type"] = getMessageTypeString(PACKET_HOP);

  doc["current_node_id"] = String(g_nodeId);

  JsonObject hyperparameters = doc.createNestedObject("hyperparameters");
  hyperparameters["alpha"] = g_alpha;
  hyperparameters["gamma"] = g_gamma;
  hyperparameters["epsilon"] = g_epsilon;
  hyperparameters["epsilon_decay"] = g_epsilonDecay;

  doc["current_episode"] = g_currentEpisode;

  if (g_currentEpisode != 1 && g_currentEpisode % 10 == 0) {
    deserializeJson(g_episodes, g_episodesString);
  } else {
    serializeJson(g_episodes, g_episodesString);
  }

  JsonObject episode = g_episodes.createNestedObject();
  episode["episode_number"] = g_currentEpisode;

  if (g_currentEpisode != 1) {
    String serializedEpisodes;
    serializeJson(g_episodes, serializedEpisodes);
    LOG("Episodes after adding new episode: " + serializedEpisodes);
  }

  JsonObject q_table = doc.createNestedObject("q_table");
  for (JsonPair kv : g_qTable) {
    q_table[kv.key()] = kv.value();
  }

  initializeOrUpdateQTable(q_table);

  String serializedQTable;
  serializeJson(q_table, serializedQTable);
  LOG("Q-table added to the message: " + serializedQTable);

  doc["episodes"] = g_episodes;
}

void extractHyperparameters(StaticJsonDocument<1024> &doc) {
  g_alpha = doc["hyperparameters"]["alpha"];
  g_gamma = doc["hyperparameters"]["gamma"];
  g_epsilon = doc["hyperparameters"]["epsilon"];
  g_epsilonDecay = doc["hyperparameters"]["epsilon_decay"];
}

JsonObject findCurrentEpisode(JsonArray &episodes, int current_episode) {
  for (JsonObject episode : episodes) {
    if (episode["episode_number"] == current_episode) {
      return episode;
    }
  }
  return JsonObject();
}

void createNewHop(JsonObject &episode, const String &node_from,
                  const String &next_action) {
  JsonArray steps = episode["steps"];
  int hop = steps.size();
  JsonObject newHop = steps.createNestedObject();
  newHop["hop"] = hop;
  newHop["node_from"] = node_from;
  newHop["node_to"] = String(next_action);
}

bool prepareAndSendMessage(StaticJsonDocument<1024> &doc,
                           const String &next_action) {
  String updatedJsonString;
  serializeJson(doc, updatedJsonString);
  g_qTable = doc["q_table"];
  doc["type"] = "PACKET_HOP";

  uint32_t next_action_int = next_action.toInt();
  return sendMessageWithRetries(next_action_int, updatedJsonString);
}

void handleHealthCheck() { LOG("healthy, Node ID: " + String(g_nodeId)); }

// Q-Learning update for latency-based routing
void updateQValueWithLatency(const String &state_from, const String &state_to,
                             float transmission_time,
                             float estimated_time_remaining, float alpha,
                             StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  float current_q = q_table[state_from][state_to].as<float>();
  LOG("Current Q-value for Q[" + state_from + "][" + state_to +
      "]: " + String(current_q));
  LOG("Transmission time: " + String(transmission_time));
  LOG("Estimated remaining time: " + String(estimated_time_remaining));

  float new_estimate = transmission_time + estimated_time_remaining;

  float updated_q = current_q + alpha * (new_estimate - current_q);

  q_table[state_from][state_to] = updated_q;

  g_qTable = q_table;
  LOG("Updated Q-value using Latency with Bellman equation: " +
      String(updated_q));
}

void updateQTableWithIncompleteInformation(int next_action,
                                           const String &node_from,
                                           const String &node_to,
                                           float estimated_time_remaining,
                                           float alpha, float gamma,
                                           StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  initializeOrUpdateQTable(q_table);

  ensureStateExists(q_table, node_from, node_to);

  // Estimate approximation of new q value with incomplete information
  float current_q = q_table[node_from][node_to].as<float>();
  LOG("Current Q-value for Q[" + node_from + "][" + node_to +
      "]: " + String(current_q));

  float updated_q = current_q + alpha * (estimated_time_remaining - current_q);

  q_table[node_from][node_to] = updated_q;

  LOG("Updated Q-value (incomplete info) for Q[" + node_from + "][" + node_to +
      "] = " + String(updated_q));
}

float estimateRemainingTime(const String &current_node,
                            StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  if (!q_table.containsKey(current_node)) {
    return 99999.0;  // Valor arbitrario grande si no hay información disponible
  }

  float min_time = 99999.0;  // Valor grande para empezar la búsqueda del mínimo

  // Recorrer todos los vecinos del nodo actual para encontrar el tiempo mínimo
  // estimado
  JsonObject actions = q_table[current_node];
  for (JsonPair kv : actions) {
    float q_value = kv.value().as<float>();

    // Buscar el valor de Q más bajo, que representa el menor tiempo estimado
    // hacia el destino
    if (q_value < min_time) {
      min_time = q_value;
    }
  }

  // Devuelve el menor tiempo estimado para llegar al destino desde este nodo
  LOG("Estimated remaining time from node " + current_node + ": " +
      String(min_time));
  return min_time;
}

String getMessageTypeString(MessageType type) {
  switch (type) {
    case PACKET_HOP:
      return "PACKET_HOP";
    case BROADCAST:
      return "BROADCAST";
    case HEALTHCHECK:
      return "HEALTHCHECK";
    default:
      return "UNKNOWN";
  }
}

MessageType getMessageType(const String &typeStr) {
  if (typeStr == "PACKET_HOP") return PACKET_HOP;
  if (typeStr == "BROADCAST") return BROADCAST;
  if (typeStr == "HEALTHCHECK") return HEALTHCHECK;
  if (typeStr == "CALLBACK") return CALLBACK;
  return UNKNOWN;
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
          LOG("Initializing state_from: " + node_from);
        }

        if (!q_table[node_from].containsKey(node_to)) {
          q_table[node_from][node_to] = 0.0f;
          LOG("Initializing state_to: " + node_to +
              " for state_from: " + node_from);
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

// Choose action using epsilon-greedy strategy
int chooseAction() {
  auto nodes = mesh.getNodeList(false);
  std::vector<int> neighbors;
  for (const auto &id : nodes) {
    neighbors.push_back(id);
  }

  if (neighbors.empty()) {
    LOG("No neighbors found");
    return -1;
  }

  if (random(0, 100) < g_epsilon * 100) {
    int action_index = random(0, neighbors.size());
    return neighbors[action_index];  // Explore
  } else {
    // Exploit: choose action with the highest Q-value
    return chooseBestAction(g_qTable[g_nodeId], neighbors);
  }
}

// Helper function to choose the best action (exploit)
int chooseBestAction(const JsonObject &actions,
                     const std::vector<int> &neighbors) {
  float best_value = -1.0;
  int best_action = -1;

  for (const auto &neighbor : neighbors) {
    float value = actions[String(neighbor)].as<float>();
    if (value > best_value) {
      best_value = value;
      best_action = neighbor;
    }
  }

  if (best_action == -1) {
    LOG("No valid actions for exploitation found. Defaulting to exploration.");
    return neighbors[random(0, neighbors.size())];
  }

  return best_action;
}

// Retry logic
bool sendMessageWithRetries(uint32_t next_hop, String &msg) {
  int retryCount = 0;

  while (retryCount < MAX_RETRIES) {
    if (mesh.sendSingle(next_hop, msg)) {
      g_lastSentMessage = getSyncedTimeInMs();
      return true;
      break;
    } else {
      retryCount++;
      delay(100);
    }
  }

  return false;
}

unsigned long getSyncedTimeInMs() {
  unsigned long nodeTimeMicroseconds = mesh.getNodeTime();
  unsigned long nodeTimeMilliseconds = nodeTimeMicroseconds / 1000;

  unsigned long scaledTimeMilliseconds = nodeTimeMilliseconds / 100;

  return scaledTimeMilliseconds;
}
