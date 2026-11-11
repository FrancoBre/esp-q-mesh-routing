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

#include <ArduinoJson.h>
#include <vector>

#include "injection_config.h"
#include "painlessMesh.h"
#include <q_mesh_utils.h>

// Logging macro
#define LOG(msg)                       \
  Serial.print("[SENDER NODE - Id: "); \
  Serial.print(g_nodeId);              \
  Serial.print(" - ");                 \
  Serial.print(__FUNCTION__);          \
  Serial.print("] ");                  \
  Serial.println(msg);

// Network and mesh configuration
#define MESH_PREFIX "ESP_Q_MESH_ROUTING"
#define MESH_PASSWORD "ESP_Q_MESH_ROUTING"
#define MESH_PORT 5555

#ifndef PACKET_JSON_CAPACITY
#define PACKET_JSON_CAPACITY 8192
#endif

// NodeMCU / most ESP8266 devkits: FLASH button → GPIO0 (active LOW). Short press injects a packet.
#ifndef FLASH_BUTTON_PIN
#define FLASH_BUTTON_PIN 0
#endif
const unsigned long FLASH_DEBOUNCE_MS = 300;
const int MAX_INJECT_PER_TICK = 32;

InjectionConfigContext g_injection_config;

// Constants and Hyperparameters
const int MAX_RETRIES = 3;
const float STEP_TIME = 1.0f;      // Fallback when send_timestamp missing
const float INITIAL_Q = 0.0f;
float g_eta = 0.7f;               // Learning rate
uint32_t g_packetId = 1;

enum MessageType {
  PACKET_HOP,
  Q_FEEDBACK,
  DIAG_REQUEST,
  DIAG_RESPONSE,
  UNKNOWN
};

enum NodeState {
  FIRST_TIME,
  PROCESSING_PACKET,
  PACKET_FINISHED,
  STUCK_PACKET,
  EXPLOITATION_PHASE
};

// Global variables
NodeState g_nodeState = FIRST_TIME;
StaticJsonDocument<8192> g_persistentDoc;
JsonObject g_qTable = g_persistentDoc.createNestedObject("q_table");
NodeId g_nodeId = 0;
DiagnosticCounters g_diagnostics;
PendingDiagnosticResponse g_pendingDiagnostic;
DiagnosticRequestHistory g_diagnosticRequestHistory;

// Object declarations
Scheduler userScheduler;
painlessMesh mesh;

// Function declarations
void setup();
void loop();
void receivedMessage(uint32_t from, String &msg);
void pollFlashButton();
void pollScheduledInjection();
void logInjectionConfig();
void applyInjectionRngSeed();
void sendNewPacket();
void buildMessage(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
String getMessageTypeString(MessageType type);
MessageType getMessageType(const String &typeStr);
std::vector<NodeId> getNeighbors();
NodeId chooseBestAction(const std::vector<NodeId> &neighbors);
bool sendMessageWithRetries(NodeId next_hop, String &msg,
                            bool direct_only = false);
void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void createNewHop(JsonObject &packetRecord, const String &node_from,
                  const String &next_action);
bool prepareAndSendMessage(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc,
                           NodeId next_action);
JsonObject findPacketById(JsonArray &packets, uint32_t current_packet_id);
void processPacket(JsonObject &packetRecord, StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void updatePacketQTable(const String &node_from,
                        const String &node_to,
                        float time_in_queue,
                        float step_time,
                        float t,
                        StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
float estimateRemainingTime(const std::vector<NodeId> &neighbors);
void initializeLocalActions(const std::vector<NodeId> &neighbors);
void copyLocalActionsToPacket(JsonObject q_table);
void handleQFeedback(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc,
                     NodeId transport_from);
void sendQFeedback(NodeId to_node, NodeId via_node,
                   float estimate_t, float step_time);
void scheduleDiagnosticResponse(JsonDocument &doc);
void processPendingDiagnosticResponse();

void setup() {
  Serial.begin(MESH_SERIAL_BAUD);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing SENDER NODE");
  pinMode(FLASH_BUTTON_PIN, INPUT_PULLUP);

  if (!loadInjectionConfigFromLittleFS(g_injection_config)) {
    g_injection_config = InjectionConfigContext::defaults();
    LOG("Injection schedule: using defaults (LittleFS missing or invalid " +
        String(INJECTION_SCHEDULE_JSON_PATH) + ")");
  } else {
    LOG("Injection schedule: loaded from LittleFS");
  }
  applyInjectionRngSeed();
  logInjectionConfig();

  // mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedMessage);

  g_nodeId = mesh.getNodeId();

  if (g_injection_config.mode == InjectionMode::PhysicalButtonDriven) {
    LOG("Mesh ready — press FLASH (GPIO " + String(FLASH_BUTTON_PIN) +
        ") to inject a packet");
  } else {
    LOG("Mesh ready — scheduled injection (tick_ms=" +
        String(g_injection_config.tick_ms) + ")");
  }
}

void loop() {
  mesh.update();
  if (g_injection_config.mode == InjectionMode::PhysicalButtonDriven) {
    pollFlashButton();
  } else {
    pollScheduledInjection();
  }
  processPendingDiagnosticResponse();
}

void applyInjectionRngSeed() {
  if (g_injection_config.seed != 0) {
    randomSeed(g_injection_config.seed);
  } else {
    randomSeed(ESP.getChipId() ^ (uint32_t)micros());
  }
}

void logInjectionConfig() {
  const char *modeStr = "?";
  switch (g_injection_config.mode) {
    case InjectionMode::PhysicalButtonDriven:
      modeStr = "PHYSICAL_BUTTON_DRIVEN";
      break;
    case InjectionMode::LoadLevel:
      modeStr = "LOAD_LEVEL";
      break;
    case InjectionMode::Periodic:
      modeStr = "PERIODIC";
      break;
  }
  LOG(String("injection_schedule: mode=") + modeStr +
      " tick_ms=" + String(g_injection_config.tick_ms) +
      " load_level=" + String(g_injection_config.load_level) +
      " seed=" + String(g_injection_config.seed));
}

void pollScheduledInjection() {
  static unsigned long last_tick_ms = 0;
  static bool primed = false;
  unsigned long now = millis();
  if (!primed) {
    last_tick_ms = now;
    primed = true;
    return;
  }
  if (now - last_tick_ms < g_injection_config.tick_ms) {
    return;
  }
  last_tick_ms = now;

  if (g_injection_config.mode == InjectionMode::Periodic) {
    LOG("Periodic tick — injecting one packet");
    sendNewPacket();
    return;
  }
  if (g_injection_config.mode == InjectionMode::LoadLevel) {
    int n = computeStochasticInjectCount(g_injection_config.load_level);
    if (n > MAX_INJECT_PER_TICK) {
      n = MAX_INJECT_PER_TICK;
    }
    LOG("Load-level tick — injecting " + String(n) + " packet(s)");
    for (int i = 0; i < n; i++) {
      sendNewPacket();
    }
  }
}

void pollFlashButton() {
  static int prevLevel = HIGH;
  static unsigned long lastFireMs = 0;

  int level = digitalRead(FLASH_BUTTON_PIN);
  unsigned long now = millis();
  if (prevLevel == HIGH && level == LOW) {
    if (now - lastFireMs >= FLASH_DEBOUNCE_MS) {
      lastFireMs = now;
      LOG("FLASH pressed — injecting packet");
      sendNewPacket();
    }
  }
  prevLevel = level;
}

void sendNewPacket() {
  LOG("Sending new packet, packet_id=" + String(g_packetId));

  StaticJsonDocument<PACKET_JSON_CAPACITY> doc;
  buildMessage(doc);
  std::vector<NodeId> neighbors = getNeighbors();
  if (neighbors.empty()) {
    LOG("No neighbors found");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_no_neighbor++;
    return;
  }
  NodeId next_action = chooseBestAction(neighbors);
  if (next_action == 0) {
    LOG("No valid next action");
    return;
  }

  if (!consumeHopBudget(doc)) {
    LOG("Packet hop budget exhausted before first send");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_ttl++;
    return;
  }

  String jsonString;
  if (!serializePacketDocument(doc, jsonString)) {
    LOG("Packet JSON is invalid, too large, or out of memory");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_json++;
    return;
  }
#if VERBOSE_MESH_LOGS
  LOG("Data queued: " + jsonString);
#endif

  if (!sendMessageWithRetries(next_action, jsonString, true)) {
    LOG("Failed to send hop after max retries");
    g_diagnostics.packets_dropped++;
  } else {
    LOG("Message accepted by painlessMesh queue");
    g_nodeState = PROCESSING_PACKET;
    g_packetId++;
  }
}

// Messaging
void receivedMessage(uint32_t from, String &msg) {
#if VERBOSE_MESH_LOGS
  LOG("Received message from " + String(from) + ": " + msg);
#endif

  if (msg.length() > MAX_PACKET_JSON_SIZE) {
    LOG("Dropping oversized mesh message");
    g_diagnostics.drop_json++;
    return;
  }

  StaticJsonDocument<PACKET_JSON_CAPACITY> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    LOG("Failed to parse message: ");
    Serial.println(error.c_str());
    g_diagnostics.drop_json++;
    return;
  }

  MessageType msgType = getMessageType(doc["type"]);
  if (msgType == PACKET_HOP) {
    NodeId claimedSender = 0;
    if (!parseNodeId(doc["current_node_id"], claimedSender) ||
        claimedSender != from || !isDirectMeshNeighbor(mesh, claimedSender)) {
      LOG("Dropping PACKET_HOP not received from a direct Q neighbor");
      g_diagnostics.packets_dropped++;
      g_diagnostics.drop_invalid_packet++;
      return;
    }
    handlePacketHop(doc);
  } else if (msgType == Q_FEEDBACK) {
    handleQFeedback(doc, from);
  } else if (msgType == DIAG_REQUEST) {
    scheduleDiagnosticResponse(doc);
  }
}

void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  LOG("Processing PACKET_HOP");
  g_diagnostics.packets_received++;

  if (!validatePacketDocument(doc)) {
    LOG("Dropping invalid PACKET_HOP");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_invalid_packet++;
    return;
  }

  extractHyperparameters(doc);

  uint32_t current_packet_id = doc["current_packet_id"];

  JsonArray receivedPackets = doc["packets"];
  JsonObject packetRecord = findPacketById(receivedPackets, current_packet_id);

  if (!packetRecord.isNull()) {
    processPacket(packetRecord, doc);
  } else {
    LOG("No matching packet for current_packet_id: " +
        String(current_packet_id));
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_invalid_packet++;
  }
}

void processPacket(JsonObject &packetRecord, StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  String node_from = doc["current_node_id"];
  String node_to = nodeIdKey(g_nodeId);
  createNewHop(packetRecord, node_from, node_to);

  if (doc["hops_remaining"].as<uint32_t>() == 0) {
    LOG("Dropping packet: Q-hop limit reached");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_ttl++;
    return;
  }

  std::vector<NodeId> neighbors = getNeighbors();
  if (neighbors.empty()) {
    LOG("Dropping packet: no direct neighbors");
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_no_neighbor++;
    return;
  }
  initializeLocalActions(neighbors);
  copyLocalActionsToPacket(doc["q_table"]);

  NodeId next_action = chooseBestAction(neighbors);
  if (next_action == 0) {
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_no_neighbor++;
    return;
  }

  // ΔQ_x(d,y) = η((q + s + t) - Q_x(d,y))
  // We update Q(node_from, us) = link that was traversed. t = our estimate (we're receiver y)
  float t = estimateRemainingTime(neighbors);
  // Queues are handled by the TCP stack; we don't have access to that data
  float time_in_queue = 0.0f;

  float step_time = STEP_TIME;
  if (doc.containsKey("send_timestamp")) {
    step_time = (mesh.getNodeTime() - doc["send_timestamp"].as<uint32_t>()) /
                1000000.0f;  // seconds
  }

  // This records the observed update for telemetry. Persistent local learning
  // is applied only by handleQFeedback() in the previous node.
  updatePacketQTable(node_from, node_to, time_in_queue, step_time, t, doc);

  NodeId previousNode = 0;
  if (parseNodeId(doc["current_node_id"], previousNode)) {
    sendQFeedback(previousNode, g_nodeId, t, step_time);
  }

  doc["type"] = "PACKET_HOP";
  if (!consumeHopBudget(doc)) {
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_ttl++;
    return;
  }
  if (prepareAndSendMessage(doc, next_action)) {
    g_diagnostics.packets_forwarded++;
  } else {
    LOG("Dropping packet: failed to queue forward");
    g_diagnostics.packets_dropped++;
  }
}

void buildMessage(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  doc["type"] = getMessageTypeString(PACKET_HOP);

  doc["current_node_id"] = nodeIdKey(g_nodeId);

  JsonObject hyperparameters = doc.createNestedObject("hyperparameters");
  hyperparameters["eta"] = g_eta;

  doc["current_packet_id"] = g_packetId;
  doc["hops_remaining"] = MAX_Q_HOPS;

  JsonArray packets = doc.createNestedArray("packets");
  JsonObject packetRecord = packets.createNestedObject();
  packetRecord["packet_id"] = g_packetId;
  packetRecord.createNestedArray("steps");

  JsonObject q_table = doc.createNestedObject("q_table");
  initializeLocalActions(getNeighbors());
  for (JsonPair kv_from : g_qTable) {
    JsonObject inner = q_table.createNestedObject(kv_from.key().c_str());
    for (JsonPair kv_to : kv_from.value().as<JsonObject>()) {
      inner[kv_to.key().c_str()] = kv_to.value().as<float>();
    }
  }

#if VERBOSE_MESH_LOGS
  String serializedQTable;
  serializeJson(q_table, serializedQTable);
  LOG("Q-table added to the message: " + serializedQTable);
#endif

  doc["send_timestamp"] = mesh.getNodeTime();  // For next hop to compute step_time
}

void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  if (doc["hyperparameters"].containsKey("eta")) {
    g_eta = doc["hyperparameters"]["eta"];
  } else if (doc["hyperparameters"].containsKey("alpha")) {
    g_eta = doc["hyperparameters"]["alpha"];  // Backward compat
  }
}

JsonObject findPacketById(JsonArray &packets, uint32_t current_packet_id) {
  for (JsonObject packetRecord : packets) {
    if (packetRecord["packet_id"] == current_packet_id) {
      return packetRecord;
    }
  }
  return JsonObject();
}

void createNewHop(JsonObject &packetRecord, const String &node_from,
                  const String &next_action) {
  if (!packetRecord.containsKey("steps")) {
    packetRecord.createNestedArray("steps");
  }
  JsonArray steps = packetRecord["steps"];
  int hop = steps.size();
  JsonObject newHop = steps.createNestedObject();
  newHop["hop"] = hop;
  newHop["node_from"] = node_from;
  newHop["node_to"] = String(next_action);
}

bool prepareAndSendMessage(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc,
                           NodeId next_action) {
  doc["current_node_id"] = nodeIdKey(g_nodeId);
  doc["send_timestamp"] = mesh.getNodeTime();  // For next hop to compute step_time
  String updatedJsonString;
  if (!serializePacketDocument(doc, updatedJsonString)) {
    g_diagnostics.drop_json++;
    return false;
  }

  return sendMessageWithRetries(next_action, updatedJsonString, true);
}

// Telemetry mirror of the Q-Routing update that Q_FEEDBACK applies at
// node_from. This does not mutate this node's persistent local table.
void updatePacketQTable(const String &node_from,
                        const String &node_to,
                        float time_in_queue,
                        float step_time,
                        float t,
                        StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  JsonObject q_table = doc["q_table"];

  ensureStateExists(q_table, node_from, node_to);

  float old_q = q_table[node_from][node_to].as<float>();
  float target = time_in_queue + step_time + t;
  float delta = g_eta * (target - old_q);
  float new_q = old_q + delta;

  q_table[node_from][node_to] = new_q;

  LOG("Packet telemetry Q-update [from=" + node_from + " to=" + node_to + "] old=" +
      String(old_q) + " target=" + String(target) + " new=" + String(new_q));
}

float estimateRemainingTime(const std::vector<NodeId> &neighbors) {
  initializeLocalActions(neighbors);
  float min_time = 99999.0;  // Start high to find minimum
  JsonObject actions = g_qTable[nodeIdKey(g_nodeId)];
  for (const auto &neighbor : neighbors) {
    float q_value = actions[nodeIdKey(neighbor)].as<float>();
    if (q_value < min_time) {
      min_time = q_value;
    }
  }

  LOG("Estimated remaining time from local Q-table at node " + nodeIdKey(g_nodeId) + ": " +
      String(min_time));
  return min_time;
}

String getMessageTypeString(MessageType type) {
  if (type == PACKET_HOP) return "PACKET_HOP";
  if (type == Q_FEEDBACK) return "Q_FEEDBACK";
  return "UNKNOWN";
}

MessageType getMessageType(const String &typeStr) {
  if (typeStr == "PACKET_HOP") return PACKET_HOP;
  if (typeStr == "Q_FEEDBACK") return Q_FEEDBACK;
  if (typeStr == "DIAG_REQUEST") return DIAG_REQUEST;
  if (typeStr == "DIAG_RESPONSE") return DIAG_RESPONSE;
  return UNKNOWN;
}

void scheduleDiagnosticResponse(JsonDocument &doc) {
  NodeId collector = 0;
  uint32_t requestId = doc["request_id"] | 0;
  if (!parseNodeId(doc["collector_id"], collector) ||
      !g_diagnosticRequestHistory.accept(requestId)) {
    return;
  }
  g_pendingDiagnostic.pending = true;
  g_pendingDiagnostic.request_id = requestId;
  g_pendingDiagnostic.collector_id = collector;
  g_pendingDiagnostic.send_at_ms = millis() + 50 + (g_nodeId % 750);
}

void processPendingDiagnosticResponse() {
  if (!g_pendingDiagnostic.pending ||
      !timeReached(millis(), g_pendingDiagnostic.send_at_ms)) {
    return;
  }
  g_pendingDiagnostic.pending = false;

  StaticJsonDocument<2048> doc;
  JsonObject localActions = g_qTable[nodeIdKey(g_nodeId)];
  buildDiagnosticResponse(doc, mesh, g_nodeId, "SENDER", localActions,
                          g_diagnostics, g_pendingDiagnostic.request_id);
  String message;
  if (!serializePacketDocument(doc, message)) {
    g_diagnostics.drop_json++;
    return;
  }
  if (!sendMessageWithRetries(g_pendingDiagnostic.collector_id, message)) {
    LOG("Failed to queue DIAG_RESPONSE");
  }
}

void initializeLocalActions(const std::vector<NodeId> &neighbors) {
  String localNode = nodeIdKey(g_nodeId);
  if (!g_qTable.containsKey(localNode)) {
    g_qTable.createNestedObject(localNode);
  }
  for (const auto &neighbor : neighbors) {
    String neighborId = nodeIdKey(neighbor);
    if (!g_qTable[localNode].containsKey(neighborId)) {
      g_qTable[localNode][neighborId] = INITIAL_Q;
    }
  }
}

// q_table in PACKET_HOP remains wire-compatible telemetry: each visited node
// contributes its local row. It is never used to overwrite local learning.
void copyLocalActionsToPacket(JsonObject q_table) {
  String localNode = nodeIdKey(g_nodeId);
  if (!q_table.containsKey(localNode)) {
    q_table.createNestedObject(localNode);
  }
  JsonObject packetActions = q_table[localNode];
  for (JsonPair action : g_qTable[localNode].as<JsonObject>()) {
    packetActions[action.key().c_str()] = action.value().as<float>();
  }
}

void handleQFeedback(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc,
                     NodeId transport_from) {
  g_diagnostics.feedback_received++;
  if (!doc["estimate_t"].is<float>() || !doc["step_time"].is<float>()) {
    LOG("Ignoring malformed Q_FEEDBACK");
    return;
  }
  NodeId fromNodeId = 0;
  NodeId viaNodeId = 0;
  if (!parseNodeId(doc["from_node"], fromNodeId) ||
      !parseNodeId(doc["via_node"], viaNodeId)) {
    LOG("Ignoring Q_FEEDBACK with invalid node ID");
    return;
  }
  String from_node = nodeIdKey(fromNodeId);
  String via_node = nodeIdKey(viaNodeId);
  float estimate_t = doc["estimate_t"].as<float>();
  float step_time = doc["step_time"].as<float>();
  float time_in_queue = 0.0f;

  if (fromNodeId != g_nodeId) {
    LOG("Ignoring Q_FEEDBACK addressed to " + from_node);
    return;
  }
  if (viaNodeId != transport_from ||
      !isDirectMeshNeighbor(mesh, transport_from)) {
    LOG("Ignoring Q_FEEDBACK not received from the direct via node");
    return;
  }

  if (!g_qTable.containsKey(from_node)) {
    g_qTable.createNestedObject(from_node);
  }
  if (!g_qTable[from_node].containsKey(via_node)) {
    g_qTable[from_node][via_node] = INITIAL_Q;
  }

  float old_q = g_qTable[from_node][via_node].as<float>();
  float target = time_in_queue + step_time + estimate_t;
  float new_q = old_q + g_eta * (target - old_q);
  g_qTable[from_node][via_node] = new_q;
  if (g_persistentDoc.overflowed()) {
    LOG("Local Q-table allocation failed");
    g_diagnostics.drop_json++;
    return;
  }
  g_diagnostics.feedback_applied++;

  LOG("Q_FEEDBACK applied [from=" + from_node + " via=" + via_node +
      "] old=" + String(old_q) + " t=" + String(estimate_t) +
      " step_time=" + String(step_time) + " new=" + String(new_q));
}

void sendQFeedback(NodeId to_node, NodeId via_node,
                   float estimate_t, float step_time) {
  StaticJsonDocument<256> feedbackDoc;
  feedbackDoc["type"] = "Q_FEEDBACK";
  feedbackDoc["from_node"] = nodeIdKey(to_node);
  feedbackDoc["via_node"] = nodeIdKey(via_node);
  feedbackDoc["estimate_t"] = estimate_t;
  feedbackDoc["step_time"] = step_time;

  String feedbackStr;
  if (!serializePacketDocument(feedbackDoc, feedbackStr)) {
    g_diagnostics.drop_json++;
    return;
  }

  if (sendMessageWithRetries(to_node, feedbackStr, true)) {
    LOG("Q_FEEDBACK queued to " + nodeIdKey(to_node) + " (via=" +
        nodeIdKey(via_node) +
        " t=" + String(estimate_t) + " s=" + String(step_time) + ")");
  } else {
    LOG("Failed to queue Q_FEEDBACK to " + nodeIdKey(to_node));
  }
}

void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to) {
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
    LOG("State from " + state_from + " not found in Q-table, initializing...");
  }

  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = INITIAL_Q;
    LOG("State to " + state_to + " not found in Q-table[" + state_from +
        "], initializing with 0.0");
  }
}

std::vector<NodeId> getNeighbors() {
  return getDirectMeshNeighbors(mesh);
}

// Choose neighbor with minimum Q-value (greedy)
// Q-Routing: lower Q = better path (faster delivery), so choose minimum
NodeId chooseBestAction(const std::vector<NodeId> &neighbors) {
  initializeLocalActions(neighbors);
  JsonObject actions = g_qTable[nodeIdKey(g_nodeId)];
  if (g_persistentDoc.overflowed() || actions.isNull()) {
    LOG("Local Q-table allocation failed");
    g_diagnostics.drop_json++;
    return 0;
  }
  float best_value = 999999.0f;  // Start high, we want minimum
  NodeId best_action = 0;

  for (const auto &neighbor : neighbors) {
    float value = actions[nodeIdKey(neighbor)].as<float>();
    if (value < best_value) {
      best_value = value;
      best_action = neighbor;
    }
  }

  if (best_action == 0) {
    LOG("No valid actions found.");
    return 0;
  }

  return best_action;
}

// Bounded retries cover immediate route/enqueue rejection only. A true return
// means queued by painlessMesh, not confirmed delivery.
bool sendMessageWithRetries(NodeId next_hop, String &msg, bool direct_only) {
  int retryCount = 0;

  while (retryCount < MAX_RETRIES) {
    if (direct_only && !isDirectMeshNeighbor(mesh, next_hop)) return false;
    if (mesh.sendSingle(next_hop, msg)) {
      return true;
    }
    g_diagnostics.send_enqueue_failures++;
    retryCount++;
    yield();
  }

  g_diagnostics.retry_exhausted++;
  return false;
}
