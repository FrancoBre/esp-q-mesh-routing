/*
 * RECEIVER NODE (Destination)
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

#include "painlessMesh.h"
#include <q_mesh_utils.h>

#define LOG(msg)                         \
  Serial.print("[RECEIVER NODE - Id: "); \
  Serial.print(g_nodeId);                \
  Serial.print(" - ");                   \
  Serial.print(__FUNCTION__);            \
  Serial.print("] ");                    \
  Serial.println(msg);

#define MESH_PREFIX "ESP_Q_MESH_ROUTING"
#define MESH_PASSWORD "ESP_Q_MESH_ROUTING"
#define MESH_PORT 5555

#ifndef PACKET_JSON_CAPACITY
#define PACKET_JSON_CAPACITY 8192
#endif

const float STEP_TIME = 1.0f;
const float INITIAL_Q = 0.0f;
float g_eta = 0.7f;

enum MessageType { PACKET_HOP, Q_FEEDBACK, DIAG_REQUEST, DIAG_RESPONSE, UNKNOWN };

const int MAX_RETRIES = 3;

NodeId g_nodeId = 0;
DiagnosticCounters g_diagnostics;
PendingDiagnosticResponse g_pendingDiagnostic;
DiagnosticRequestHistory g_diagnosticRequestHistory;
uint32_t g_diagnosticRequestSequence = 0;

Scheduler userScheduler;
painlessMesh mesh;

void setup();
void loop();
void receivedCallback(uint32_t from, String &msg);
void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
MessageType getMessageType(const String &typeStr);
void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
JsonObject findPacketById(JsonArray &packets, uint32_t current_packet_id);
void createNewHop(JsonObject &packetRecord, const String &node_from,
                  const String &node_to);
void updatePacketQTable(const String &node_from,
                        const String &node_to,
                        float time_in_queue,
                        float step_time,
                        float t,
                        StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to);
void emitDeliveryData(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void sendQFeedback(NodeId to_node, NodeId via_node,
                   float estimate_t, float step_time);
bool sendMessageWithRetries(NodeId next_hop, String &msg,
                            bool direct_only = false);
void pollDiagnosticCommand();
void requestDiagnostics();
void scheduleDiagnosticResponse(JsonDocument &doc);
void processPendingDiagnosticResponse();
void emitDiagnosticData(JsonDocument &doc);

void setup() {
  Serial.begin(MESH_SERIAL_BAUD);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing RECEIVER NODE (destination)");

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  g_nodeId = mesh.getNodeId();
  g_diagnosticRequestSequence =
      mesh.getNodeTime() ^ ESP.getCycleCount() ^ g_nodeId;
  if (g_diagnosticRequestSequence == 0) g_diagnosticRequestSequence = 1;

  LOG("Mesh initialized successfully");
}

void loop() {
  mesh.update();
  pollDiagnosticCommand();
  processPendingDiagnosticResponse();
}

void receivedCallback(uint32_t from, String &msg) {
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

  MessageType type = getMessageType(doc["type"]);
  if (type == PACKET_HOP) {
    NodeId claimedSender = 0;
    if (!parseNodeId(doc["current_node_id"], claimedSender) ||
        claimedSender != from || !isDirectMeshNeighbor(mesh, claimedSender)) {
      LOG("Dropping PACKET_HOP not received from a direct Q neighbor");
      g_diagnostics.packets_dropped++;
      g_diagnostics.drop_invalid_packet++;
      return;
    }
    handlePacketHop(doc);
  } else if (type == DIAG_REQUEST) {
    scheduleDiagnosticResponse(doc);
  } else if (type == DIAG_RESPONSE) {
    emitDiagnosticData(doc);
  }
}

void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  LOG("Processing PACKET_HOP — delivery at destination");
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

  if (packetRecord.isNull()) {
    LOG("No matching packet for current_packet_id: " +
        String(current_packet_id));
    g_diagnostics.packets_dropped++;
    g_diagnostics.drop_invalid_packet++;
    return;
  }

  String node_from = doc["current_node_id"].as<String>();
  String node_to = nodeIdKey(g_nodeId);

  float t = 0.0f;
  float time_in_queue = 0.0f;

  float step_time = STEP_TIME;
  if (doc.containsKey("send_timestamp")) {
    step_time = (mesh.getNodeTime() - doc["send_timestamp"].as<uint32_t>()) /
                1000000.0f;
  }

  // Destination remaining time is exactly zero. Keep the corresponding
  // observed update in packet telemetry, then report it to the previous node.
  updatePacketQTable(node_from, node_to, time_in_queue, step_time, t, doc);

  NodeId previousNode = 0;
  if (parseNodeId(doc["current_node_id"], previousNode)) {
    sendQFeedback(previousNode, g_nodeId, t, step_time);
  }

  createNewHop(packetRecord, node_from, node_to);

  doc["current_node_id"] = nodeIdKey(g_nodeId);
  doc["type"] = "PACKET_DELIVERED";
  emitDeliveryData(doc);
}

void emitDeliveryData(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  String jsonStr;
  if (!serializePacketDocument(doc, jsonStr)) {
    LOG("Failed to serialize delivery data");
    g_diagnostics.drop_json++;
    return;
  }
  Serial.println("DELIVERY_DATA:" + jsonStr);
  LOG("Packet marked received; DELIVERY_DATA emitted (no forward)");
}

void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  if (doc["hyperparameters"].containsKey("eta")) {
    g_eta = doc["hyperparameters"]["eta"];
  } else if (doc["hyperparameters"].containsKey("alpha")) {
    g_eta = doc["hyperparameters"]["alpha"];
  }
}

JsonObject findPacketById(JsonArray &packets, uint32_t current_packet_id) {
  for (JsonObject pkt : packets) {
    if (pkt["packet_id"] == current_packet_id) {
      return pkt;
    }
  }
  return JsonObject();
}

void createNewHop(JsonObject &packetRecord, const String &node_from,
                  const String &node_to) {
  if (!packetRecord.containsKey("steps")) {
    packetRecord.createNestedArray("steps");
  }
  JsonArray steps = packetRecord["steps"];
  int hop = steps.size();
  JsonObject newHop = steps.createNestedObject();
  newHop["hop"] = hop;
  newHop["node_from"] = node_from;
  newHop["node_to"] = node_to;
}

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

void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to) {
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
  }
  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = INITIAL_Q;
  }
}

MessageType getMessageType(const String &typeStr) {
  if (typeStr == "PACKET_HOP") return PACKET_HOP;
  if (typeStr == "Q_FEEDBACK") return Q_FEEDBACK;
  if (typeStr == "DIAG_REQUEST") return DIAG_REQUEST;
  if (typeStr == "DIAG_RESPONSE") return DIAG_RESPONSE;
  return UNKNOWN;
}

void pollDiagnosticCommand() {
  while (Serial.available() > 0) {
    if (Serial.read() == 'D') requestDiagnostics();
  }
}

void requestDiagnostics() {
  uint32_t requestId = ++g_diagnosticRequestSequence;
  StaticJsonDocument<256> request;
  request["type"] = "DIAG_REQUEST";
  request["request_id"] = requestId;
  request["collector_id"] = nodeIdKey(g_nodeId);

  scheduleDiagnosticResponse(request);

  String message;
  if (!serializePacketDocument(request, message) ||
      !mesh.sendBroadcast(message, false)) {
    LOG("Failed to queue DIAG_REQUEST broadcast");
    g_diagnostics.send_enqueue_failures++;
    return;
  }
  LOG("Diagnostic snapshot requested: " + String(requestId));
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

  StaticJsonDocument<2048> response;
  StaticJsonDocument<32> emptyTable;
  JsonObject emptyRow = emptyTable.to<JsonObject>();
  buildDiagnosticResponse(response, mesh, g_nodeId, "RECEIVER", emptyRow,
                          g_diagnostics, g_pendingDiagnostic.request_id);

  if (g_pendingDiagnostic.collector_id == g_nodeId) {
    emitDiagnosticData(response);
    return;
  }

  String message;
  if (!serializePacketDocument(response, message)) {
    g_diagnostics.drop_json++;
    return;
  }
  if (!sendMessageWithRetries(g_pendingDiagnostic.collector_id, message)) {
    LOG("Failed to queue DIAG_RESPONSE");
  }
}

void emitDiagnosticData(JsonDocument &doc) {
  if (doc["type"] != "DIAG_RESPONSE" ||
      !doc["request_id"].is<uint32_t>() ||
      !doc["node_id"].is<const char *>()) {
    g_diagnostics.drop_invalid_packet++;
    return;
  }
  String output;
  if (!serializePacketDocument(doc, output)) {
    g_diagnostics.drop_json++;
    return;
  }
  Serial.println("DIAG_DATA:" + output);
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
