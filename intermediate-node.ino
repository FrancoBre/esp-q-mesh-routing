/*
 * INTERMEDIATE NODE
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

#include <ArduinoJson.h>
#include <vector>

#include "painlessMesh.h"

// Logging macro
#define LOG(msg)                             \
  Serial.print("[INTERMEDIATE NODE - Id: "); \
  Serial.print(g_nodeId);                    \
  Serial.print(" - ");                       \
  Serial.print(__FUNCTION__);                \
  Serial.print("] ");                        \
  Serial.println(msg);                       \
  Serial.flush();

#define MESH_PREFIX "ESP_Q_MESH_ROUTING"
#define MESH_PASSWORD "ESP_Q_MESH_ROUTING"
#define MESH_PORT 5555

// Constants and Hyperparameters
const int MAX_RETRIES = 10;
const float STEP_TIME = 1.0f;      // Fallback when send_timestamp missing
const float INITIAL_Q = 0.0f;
float g_eta = 0.7f;               // Learning rate

enum MessageType { PACKET_HOP, UNKNOWN };

enum NodeState { PROCESSING_EPISODE, EXPLOITATION_PHASE };

// Global variables
NodeState g_nodeState = PROCESSING_EPISODE;
float g_accumulatedReward = 0.0;
String g_nodeId = "MESH NOT INITIALIZED YET";

// Objects declarations
Scheduler userScheduler;
painlessMesh mesh;

// Function declarations
void setup();
void loop();
void receivedCallback(uint32_t from, String &msg);
void handlePacketHop(StaticJsonDocument<1024> &doc);
MessageType getMessageType(const String &typeStr);
std::vector<int> getNeighbors();
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
float estimateRemainingTime(const String &current_node,
                            StaticJsonDocument<1024> &doc);
void updateQTableForwardOnly(int next_action,
                             const String &node_from,
                             const String &node_to,
                             float time_in_queue,
                             float step_time,
                             float t,
                             StaticJsonDocument<1024> &doc);

// Main logic functions
void setup() {
  Serial.begin(9600);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing INTERMEDIATE NODE");

  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  g_nodeId = String(mesh.getNodeId());

  LOG("Mesh initialized successfully");
}

void loop() { mesh.update(); }

// Messaging and episode handling
void receivedCallback(uint32_t from, String &msg) {
  LOG("Received message from " + String(from) + ": " + msg);

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    LOG("Failed to parse message: ");
    Serial.println(error.c_str());
    return;
  }

  if (getMessageType(doc["type"]) == PACKET_HOP) {
    handlePacketHop(doc);
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

  std::vector<int> neighbors = getNeighbors();
  if (neighbors.empty()) return;
  int next_action = chooseBestAction(doc["q_table"][g_nodeId], neighbors);
  if (next_action == -1) {
    return;
  }

  // ΔQ_x(d,y) = η((q + s + t) - Q_x(d,y))
  // We update Q(node_from, us) = link that was traversed. t = our estimate (we're receiver y)
  float t = estimateRemainingTime(g_nodeId, doc);
  // Queues are handled by the TCP stack; we don't have access to that data
  float time_in_queue = 0.0f;

  float step_time = STEP_TIME;
  if (doc.containsKey("send_timestamp")) {
    step_time = (mesh.getNodeTime() - doc["send_timestamp"].as<uint32_t>()) /
                1000000.0f;  // seconds
  }

  updateQTableForwardOnly(next_action, node_from, node_to, time_in_queue,
                          step_time, t, doc);

  createNewHop(episode, node_from, node_to);

  doc["type"] = "PACKET_HOP";
  prepareAndSendMessage(doc, String(next_action));
}

// Hyperparameter and episode management
void extractHyperparameters(StaticJsonDocument<1024> &doc) {
  if (doc["hyperparameters"].containsKey("eta")) {
    g_eta = doc["hyperparameters"]["eta"];
  } else if (doc["hyperparameters"].containsKey("alpha")) {
    g_eta = doc["hyperparameters"]["alpha"];  // Backward compat
  }
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
  doc["send_timestamp"] = mesh.getNodeTime();  // For next hop to compute step_time
  String updatedJsonString;
  serializeJson(doc, updatedJsonString);

  uint32_t next_action_int = next_action.toInt();
  return sendMessageWithRetries(next_action_int, updatedJsonString);
}

// Q-Routing update: ΔQ_x(d,y) = η((q + s + t) - Q_x(d,y))
// We update Q(node_from, node_to) = link traversed. t = receiver's estimate.
void updateQTableForwardOnly(int next_action,
                             const String &node_from,
                             const String &node_to,
                             float time_in_queue,
                             float step_time,
                             float t,
                             StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  ensureStateExists(q_table, node_from, node_to);

  float old_q = q_table[node_from][node_to].as<float>();
  float target = time_in_queue + step_time + t;
  float delta = g_eta * (target - old_q);
  float new_q = old_q + delta;

  q_table[node_from][node_to] = new_q;

  LOG("Q-update [from=" + node_from + " to=" + node_to + "] old=" +
      String(old_q) + " target=" + String(target) + " new=" + String(new_q));
}

float estimateRemainingTime(const String &current_node,
                            StaticJsonDocument<1024> &doc) {
  JsonObject q_table = doc["q_table"];

  if (!q_table.containsKey(current_node)) {
    return 99999.0;  // Large number when no information available
  }

  float min_time = 99999.0;  // Start high to find minimum

  JsonObject actions = q_table[current_node];
  for (JsonPair kv : actions) {
    float q_value = kv.value().as<float>();
    if (q_value < min_time) {
      min_time = q_value;
    }
  }

  LOG("Estimated remaining time from node " + current_node + ": " +
      String(min_time));
  return min_time;
}

MessageType getMessageType(const String &typeStr) {
  return (typeStr == "PACKET_HOP") ? PACKET_HOP : UNKNOWN;
}

void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to) {
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
  }
  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = INITIAL_Q;
  }
}

std::vector<int> getNeighbors() {
  auto nodes = mesh.getNodeList(false);
  std::vector<int> neighbors;
  for (const auto &id : nodes) {
    neighbors.push_back(id);
  }
  return neighbors;
}

// Choose neighbor with minimum Q-value (greedy)
// Q-Routing: lower Q = better path (faster delivery), so choose minimum
int chooseBestAction(const JsonObject &actions,
                     const std::vector<int> &neighbors) {
  float best_value = 999999.0f;  // Start high, we want minimum
  int best_action = -1;

  for (const auto &neighbor : neighbors) {
    float value = actions[String(neighbor)].as<float>();
    if (value < best_value) {
      best_value = value;
      best_action = neighbor;
    }
  }

  if (best_action == -1) {
    LOG("No valid actions found.");
    return -1;
  }

  return best_action;
}

// Retry logic
bool sendMessageWithRetries(uint32_t next_hop, String &msg) {
  int retryCount = 0;

  while (retryCount < MAX_RETRIES) {
    if (mesh.sendSingle(next_hop, msg)) {
      return true;
    }
    retryCount++;
    delay(100);
  }

  return false;
}

unsigned long getSyncedTimeInMs() {
  unsigned long nodeTimeMicroseconds = mesh.getNodeTime();
  unsigned long nodeTimeMilliseconds = nodeTimeMicroseconds / 1000;

  unsigned long scaledTimeMilliseconds = nodeTimeMilliseconds / 100;

  return scaledTimeMilliseconds;
}
