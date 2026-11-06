#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <vector>

using NodeId = uint32_t;

inline String nodeIdKey(NodeId id) { return String(id); }

inline bool parseNodeId(JsonVariantConst value, NodeId &nodeId) {
  if (value.is<uint32_t>()) {
    nodeId = value.as<uint32_t>();
    return true;
  }

  const char *text = value.as<const char *>();
  if (text == nullptr || *text == '\0') return false;

  char *end = nullptr;
  unsigned long parsed = strtoul(text, &end, 10);
  if (end == text || *end != '\0') return false;

  nodeId = static_cast<NodeId>(parsed);
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
