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

#define LOG(msg)                         \
  Serial.print("[RECEIVER NODE - Id: "); \
  Serial.print(g_nodeId);                \
  Serial.print(" - ");                   \
  Serial.print(__FUNCTION__);            \
  Serial.print("] ");                    \
  Serial.println(msg);                   \
  Serial.flush();

#define MESH_PREFIX "ESP_Q_MESH_ROUTING"
#define MESH_PASSWORD "ESP_Q_MESH_ROUTING"
#define MESH_PORT 5555

#ifndef PACKET_JSON_CAPACITY
#define PACKET_JSON_CAPACITY 8192
#endif

const float STEP_TIME = 1.0f;
const float INITIAL_Q = 0.0f;
float g_eta = 0.7f;

enum MessageType { PACKET_HOP, UNKNOWN };

String g_nodeId = "MESH NOT INITIALIZED YET";

Scheduler userScheduler;
painlessMesh mesh;

void setup();
void loop();
void receivedCallback(uint32_t from, String &msg);
void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
MessageType getMessageType(const String &typeStr);
void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
JsonObject findPacketById(JsonArray &packets, int current_packet_id);
void createNewHop(JsonObject &packetRecord, const String &node_from,
                  const String &node_to);
void updateQTableForwardOnly(const String &node_from,
                             const String &node_to,
                             float time_in_queue,
                             float step_time,
                             float t,
                             StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);
void ensureStateExists(JsonObject &q_table, const String &state_from,
                       const String &state_to);
void initializeOrUpdateQTable(JsonObject &q_table);
void emitDeliveryData(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc);

void setup() {
  Serial.begin(9600);

  for (uint8_t t = 10; t > 0; t--) {
    LOG("WAIT " + String(t) + "...");
    delay(1000);
  }

  LOG("Initializing RECEIVER NODE (destination)");

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  g_nodeId = String(mesh.getNodeId());

  LOG("Mesh initialized successfully");
}

void loop() { mesh.update(); }

void receivedCallback(uint32_t from, String &msg) {
  LOG("Received message from " + String(from) + ": " + msg);

  StaticJsonDocument<PACKET_JSON_CAPACITY> doc;
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

void handlePacketHop(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  LOG("Processing PACKET_HOP — delivery at destination");

  extractHyperparameters(doc);

  int current_packet_id = doc["current_packet_id"];
  JsonArray receivedPackets = doc["packets"];
  JsonObject packetRecord = findPacketById(receivedPackets, current_packet_id);

  if (packetRecord.isNull()) {
    LOG("No matching packet for current_packet_id: " +
        String(current_packet_id));
    return;
  }

  String node_from = doc["current_node_id"].as<String>();
  String node_to = String(g_nodeId);

  float t = 0.0f;
  float time_in_queue = 0.0f;

  float step_time = STEP_TIME;
  if (doc.containsKey("send_timestamp")) {
    step_time = (mesh.getNodeTime() - doc["send_timestamp"].as<uint32_t>()) /
                1000000.0f;
  }

  updateQTableForwardOnly(node_from, node_to, time_in_queue, step_time, t, doc);

  createNewHop(packetRecord, node_from, node_to);

  doc["current_node_id"] = g_nodeId;
  doc["type"] = "PACKET_DELIVERED";
  emitDeliveryData(doc);
}

void emitDeliveryData(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  String jsonStr;
  serializeJson(doc, jsonStr);
  Serial.println("DELIVERY_DATA:" + jsonStr);
  Serial.flush();
  LOG("Packet marked received; DELIVERY_DATA emitted (no forward)");
}

void extractHyperparameters(StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  if (doc["hyperparameters"].containsKey("eta")) {
    g_eta = doc["hyperparameters"]["eta"];
  } else if (doc["hyperparameters"].containsKey("alpha")) {
    g_eta = doc["hyperparameters"]["alpha"];
  }
}

JsonObject findPacketById(JsonArray &packets, int current_packet_id) {
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

void updateQTableForwardOnly(const String &node_from,
                             const String &node_to,
                             float time_in_queue,
                             float step_time,
                             float t,
                             StaticJsonDocument<PACKET_JSON_CAPACITY> &doc) {
  JsonObject q_table = doc["q_table"];

  initializeOrUpdateQTable(q_table);
  ensureStateExists(q_table, node_from, node_to);

  float old_q = q_table[node_from][node_to].as<float>();
  float target = time_in_queue + step_time + t;
  float delta = g_eta * (target - old_q);
  float new_q = old_q + delta;

  q_table[node_from][node_to] = new_q;

  LOG("Q-update [from=" + node_from + " to=" + node_to + "] old=" +
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

void initializeOrUpdateQTable(JsonObject &q_table) {
  auto nodes = mesh.getNodeList(true);

  for (auto &&id : nodes) {
    String from = String(id);
    for (auto &&id_2 : nodes) {
      String to = String(id_2);

      if (id_2 != id) {
        if (!q_table.containsKey(from)) {
          q_table.createNestedObject(from);
        }
        if (!q_table[from].containsKey(to)) {
          q_table[from][to] = INITIAL_Q;
        }
      }
    }
  }
}

MessageType getMessageType(const String &typeStr) {
  return (typeStr == "PACKET_HOP") ? PACKET_HOP : UNKNOWN;
}
