#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <vector>

using NodeId = uint32_t;

#ifndef MAX_Q_HOPS
#define MAX_Q_HOPS 24
#endif

#ifndef MAX_PACKET_JSON_SIZE
#define MAX_PACKET_JSON_SIZE 6144
#endif

#ifndef MESH_SERIAL_BAUD
#define MESH_SERIAL_BAUD 115200
#endif

#ifndef VERBOSE_MESH_LOGS
#define VERBOSE_MESH_LOGS 0
#endif

inline String nodeIdKey(NodeId id) { return String(id); }

inline bool parseNodeId(JsonVariantConst value, NodeId &nodeId) {
  if (value.is<uint32_t>()) {
    nodeId = value.as<uint32_t>();
    return true;
  }

  const char *text = value.as<const char *>();
  if (text == nullptr || *text == '\0') return false;

  uint32_t parsed = 0;
  for (const char *cursor = text; *cursor != '\0'; ++cursor) {
    if (*cursor < '0' || *cursor > '9') return false;
    uint8_t digit = static_cast<uint8_t>(*cursor - '0');
    if (parsed > (UINT32_MAX - digit) / 10) return false;
    parsed = parsed * 10 + digit;
  }

  nodeId = parsed;
  return true;
}

// asNodeTree().subs is built from Mesh::subs, whose entries are the active
// direct painlessMesh TCP connections. Descendants inside each sub-tree are
// reachable nodes, but are deliberately not Q-Routing actions.
inline std::vector<NodeId> getDirectMeshNeighbors(painlessMesh &mesh) {
  auto topology = mesh.asNodeTree();
  std::vector<NodeId> neighbors;
  neighbors.reserve(topology.subs.size());
  for (const auto &connection : topology.subs) {
    if (connection.nodeId != 0) neighbors.push_back(connection.nodeId);
  }
  return neighbors;
}

inline bool isDirectMeshNeighbor(painlessMesh &mesh, NodeId nodeId) {
  for (NodeId neighbor : getDirectMeshNeighbors(mesh)) {
    if (neighbor == nodeId) return true;
  }
  return false;
}

struct DiagnosticCounters {
  uint32_t packets_received = 0;
  uint32_t packets_forwarded = 0;
  uint32_t packets_dropped = 0;
  uint32_t drop_no_neighbor = 0;
  uint32_t drop_invalid_packet = 0;
  uint32_t drop_ttl = 0;
  uint32_t drop_json = 0;
  uint32_t send_enqueue_failures = 0;
  uint32_t retry_exhausted = 0;
  uint32_t feedback_received = 0;
  uint32_t feedback_applied = 0;
};

struct PendingDiagnosticResponse {
  bool pending = false;
  uint32_t request_id = 0;
  NodeId collector_id = 0;
  unsigned long send_at_ms = 0;
};

struct DiagnosticRequestHistory {
  uint32_t request_ids[4] = {0, 0, 0, 0};
  uint8_t next = 0;

  bool accept(uint32_t requestId) {
    if (requestId == 0) return false;
    for (uint32_t seen : request_ids) {
      if (seen == requestId) return false;
    }
    request_ids[next] = requestId;
    next = (next + 1) % 4;
    return true;
  }
};

inline bool timeReached(unsigned long now, unsigned long deadline) {
  return static_cast<long>(now - deadline) >= 0;
}

template <typename TDocument>
inline void buildDiagnosticResponse(TDocument &doc, painlessMesh &mesh,
                                    NodeId nodeId, const char *role,
                                    JsonObject localActions,
                                    const DiagnosticCounters &counters,
                                    uint32_t requestId) {
  doc["type"] = "DIAG_RESPONSE";
  doc["request_id"] = requestId;
  doc["node_id"] = nodeIdKey(nodeId);
  doc["role"] = role;
  bool receiverIdentityKnown = strcmp(role, "RECEIVER") == 0;
  doc["receiver_identity_known"] = receiverIdentityKnown;
  if (receiverIdentityKnown) {
    doc["receiver_id"] = nodeIdKey(nodeId);
  } else {
    doc["receiver_id"] = nullptr;
  }
  doc["uptime_ms"] = millis();
  doc["free_heap"] = ESP.getFreeHeap();
  doc["known_nodes"] = mesh.getNodeList(false).size();

  JsonArray direct = doc.createNestedArray("direct_neighbors");
  for (NodeId neighbor : getDirectMeshNeighbors(mesh)) {
    direct.add(nodeIdKey(neighbor));
  }

  JsonObject qRow = doc.createNestedObject("q_row");
  for (JsonPair action : localActions) {
    qRow[action.key().c_str()] = action.value().as<float>();
  }

  JsonObject values = doc.createNestedObject("counters");
  values["received"] = counters.packets_received;
  values["forwarded"] = counters.packets_forwarded;
  values["dropped"] = counters.packets_dropped;
  values["drop_no_neighbor"] = counters.drop_no_neighbor;
  values["drop_invalid"] = counters.drop_invalid_packet;
  values["drop_ttl"] = counters.drop_ttl;
  values["drop_json"] = counters.drop_json;
  values["send_enqueue_failures"] = counters.send_enqueue_failures;
  values["retry_exhausted"] = counters.retry_exhausted;
  values["feedback_received"] = counters.feedback_received;
  values["feedback_applied"] = counters.feedback_applied;
}

template <typename TDocument>
inline bool validatePacketDocument(TDocument &doc) {
  if (doc.overflowed() || doc["type"] != "PACKET_HOP" ||
      !doc["current_packet_id"].template is<uint32_t>() ||
      !doc["hops_remaining"].template is<uint32_t>() ||
      doc["hops_remaining"].template as<uint32_t>() > MAX_Q_HOPS ||
      !doc["send_timestamp"].template is<uint32_t>() ||
      !doc["q_table"].template is<JsonObjectConst>() ||
      !doc["hyperparameters"]["eta"].template is<float>()) {
    return false;
  }

  NodeId currentNode = 0;
  if (!parseNodeId(doc["current_node_id"], currentNode)) return false;

  JsonArrayConst packets = doc["packets"].template as<JsonArrayConst>();
  if (packets.size() != 1) return false;
  JsonObjectConst packet = packets[0];
  if (packet["packet_id"].template as<uint32_t>() !=
      doc["current_packet_id"].template as<uint32_t>()) {
    return false;
  }

  JsonArrayConst steps = packet["steps"].template as<JsonArrayConst>();
  return !steps.isNull() && steps.size() <= MAX_Q_HOPS;
}

template <typename TDocument>
inline bool consumeHopBudget(TDocument &doc) {
  uint32_t remaining = doc["hops_remaining"].template as<uint32_t>();
  if (remaining == 0) return false;
  doc["hops_remaining"] = remaining - 1;
  return true;
}

template <typename TDocument>
inline bool serializePacketDocument(TDocument &doc, String &output) {
  if (doc.overflowed()) return false;
  size_t length = measureJson(doc);
  if (length == 0 || length > MAX_PACKET_JSON_SIZE) return false;
  output.reserve(length + 1);
  return serializeJson(doc, output) == length;
}
