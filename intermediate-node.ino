/*
* INTERMEDIATE NODE
*/
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <random>
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

painlessMesh  mesh;

// save the most recent q table for sending data
// with the latest learning data
JsonDocument qTable;
bool isFirstTime = true;

void receivedCallback(uint32_t from, String &msg);
void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc);
int chooseAction(int state, JsonDocument& doc, float epsilon);
void sendMessageToNextHop(uint32_t next_hop, String &msg);

void newConnectionCallback(uint32_t nodeId) {
  Serial.print("New Connection, nodeId = ");
  Serial.println(nodeId);
}

void changedConnectionCallback() {
}

void nodeTimeAdjustedCallback(int32_t offset) {
}

bool isHopMessage(StaticJsonDocument<1024> doc) {
  return doc.containsKey("hop");
}

bool isBroadcastMessage(StaticJsonDocument<1024> doc) {
  return doc.containsKey("broadcast");
}

// Needed for painless library
void receivedCallback(uint32_t from, String &msg) {
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());

    Serial.print("startHere: Received from ");
    Serial.print(from);
    Serial.print(" msg=");
    Serial.println(msg.c_str());

    // Deserialize the JSON message
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        Serial.flush();
        return;
    }

    // Check if the message has all the learning data
    if (isHopMessage(doc)) {
        // Extract q-learning parameters and state information
        float q_alpha = doc["q_parameters"]["alpha"];
        float q_gamma = doc["q_parameters"]["gamma"];
        float q_epsilon = doc["q_parameters"]["epsilon"];
        float q_epsilonDecay = doc["q_parameters"]["epsilon_decay"];

        int current_episode = doc["current_episode"];
        float accumulated_reward = doc["accumulated_reward"];

        JsonArray episodes = doc["episodes"];
        for (JsonObject episode : episodes) {
            int episode_number = episode["episode_number"];
            if (episode_number == current_episode) {
              float reward = episode["reward"];

              // Add reward to episode
              episode["reward"] = String(reward - 1.0); // This node is not master!
              doc["accumulated_reward"] = String(reward - 1.0);
              Serial.println("This node is not master! Reduce episode reward in 1");
              Serial.println(String(episode["reward"]));
              Serial.flush();

              // Choose next action using epsilon-greedy policy
              int next_action = chooseAction(mesh.getNodeId(), doc, q_epsilon);

              if (next_action == -1) {
                return;
              }

              JsonArray steps = episode["steps"];
              int hop = steps.size();
              
              JsonObject newHop = steps.createNestedObject();
              newHop["hop"] = hop;
              newHop["node_from"] = String(mesh.getNodeId());
              newHop["node_to"] = String(next_action);

              // Update Q-Table for the current hop
              updateQTable(newHop["node_to"], newHop["node_from"], episode["reward"], q_alpha, q_gamma, doc);

              // Send updated message to the next hop
              String updatedJsonString;
              serializeJson(doc, updatedJsonString);
              qTable = doc["q_table"];
              doc["hop"] = true;
              sendMessageToNextHop(next_action, updatedJsonString);
            }
        }
    }
    // Message is a q_table update broadcast
    else if (isBroadcastMessage(doc)) {
      Serial.println("Received Q-Table update:");
      serializeJsonPretty(doc, Serial);
      qTable = doc["q_table"];
    } else {
      Serial.println("Unknown message structure");
      Serial.flush();
    }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc) {
  auto nodes = mesh.getNodeList(true);
  std::vector<int> neighbors;
  String nodesStr;
  int num_neighbors = 0;

  //JsonObject q_table = doc["q_table"];
  for (auto &&id : nodes) {
    String node_from = String(id);
    for (auto &&id_2 : nodes) {
      String node_to = String(id_2);

      if (id_2 != id) {
        if (!doc["q_table"].containsKey(node_from)) {
          doc["q_table"].createNestedObject(node_from);
        }

        if (!doc["q_table"][node_to].containsKey(node_to)) {
          doc["q_table"][node_from][node_to] = 0.0; // Initialize Q-value if not present
        }
      }
    }
  }

  Serial.println("Q-table:");
  serializeJsonPretty(doc["q_table"], Serial);
  Serial.flush();

  // Ensure state_from exists in q_table
  if (!doc["q_table"].containsKey(state_from)) {
    doc["q_table"].createNestedObject(state_from);
  }

  // Ensure state_to exists in q_table[state_from]
  if (!doc["q_table"][state_from].containsKey(state_to)) {
    doc["q_table"][state_from][state_to] = 0.0; // Initialize Q-value if not present
  }

  // Retrieve Q-value to update
  float currentQ = doc["q_table"][state_from][state_to].as<float>();
  Serial.print("Current Q:");
  Serial.println(String(currentQ));

  // Calculate maxQ for state_to
  float maxQ = -9999.0; // Start with a very low value
  if (doc["q_table"].containsKey(state_to)) {
    JsonObject actions = doc["q_table"][state_to];
    for (JsonPair kv : actions) {
      float value = kv.value().as<float>();
      if (value > maxQ) {
        maxQ = value;
      }
    }
  }

  // Apply Bellman equation to update Q-value
  float updatedQ = currentQ + alpha * (reward + gamma * maxQ - currentQ);
  Serial.print("updatedQ :");
  Serial.println(String(updatedQ));

  // Update Q-value in q_table
  doc["q_table"][state_from][state_to] = updatedQ;

  Serial.print("q_table with updated q:");
  serializeJsonPretty(doc["q_table"], Serial);

  // Log updated Q-table
  Serial.println("Updated Q-table:");
  serializeJsonPretty(doc["q_table"], Serial);
  Serial.println();
  Serial.flush();
}

int chooseAction(int state, JsonDocument& doc, float epsilon) {
    auto nodes = mesh.getNodeList(false);
    std::vector<int> neighbors;
    String nodesStr;
    int num_neighbors = 0;

    Serial.print("iterating over node list ");
    for (auto &&id : nodes) {
        neighbors.push_back(id);
        nodesStr += String(id) + String(" ");
        Serial.print(nodesStr);
        num_neighbors++;
    }
    Serial.flush();

    if (num_neighbors == 0) {
        Serial.println("No neighbors found");
        return -1;
    }

    if (isFirstTime) {
        Serial.println("Populating QTable for the first time");
        StaticJsonDocument<1024> doc;
        JsonObject q_table = doc.createNestedObject("q_table");

        // Asegurarse de que cada estado tiene un objeto dentro de la Q-table
        for (auto &&id : nodes) {
            JsonObject stateObj = q_table.createNestedObject(String(state)); // Cambio aquí
            for (auto &&action : nodes) {
                stateObj[String(action)] = 0.0; // Inicializar valores Q para cada acción
            }
        }

        String jsonString;
        serializeJsonPretty(doc, jsonString);
        Serial.println(jsonString);
        qTable = doc;
        isFirstTime = false;
    } else {
        Serial.println("");
        Serial.println("Updating QTable with missing neighbors");

        // Asegurarse de que el estado actual tiene un objeto dentro de la Q-table
        if (!qTable["q_table"].containsKey(String(state))) {
            JsonObject stateObj = qTable["q_table"].createNestedObject(String(state));
            for (auto &&action : nodes) {
                stateObj[String(action)] = 0.0; // Inicializar valores Q para cada acción
            }
        }

        // Verificar si los vecinos están en la Q-table para el estado actual
        JsonObject stateObj = qTable["q_table"][String(state)];
        for (auto &&id : nodes) {
            if (!stateObj.containsKey(String(id))) {
                Serial.print("Adding missing action for neighbor ");
                Serial.println(String(id));
                stateObj[String(id)] = 0.0; // Inicializar valor Q para la acción faltante
            }
        }

        serializeJsonPretty(qTable, Serial);
    }

    Serial.flush();

    Serial.print("Neighbors for state ");
    Serial.print(String(state));
    Serial.print(" are ");
    Serial.println(nodesStr);
    Serial.flush();

    if (random() < epsilon) {
        // Explorar
        int action_index = random(0, num_neighbors - 1);
        int action = neighbors[action_index];
        Serial.println("Exploring action");
        Serial.println(action);
        Serial.flush();
        return action;
    } else {
        // Explotar: elegir la acción con el valor Q más alto
        if (!qTable["q_table"].containsKey(String(state))) {
            Serial.print("No Q-values found for state ");
            Serial.println(String(state));

            Serial.println("in qTable:");
            serializeJsonPretty(qTable, Serial);

            Serial.println("in qTable['q_table'");
            serializeJsonPretty(qTable["q_table"], Serial);
            Serial.flush();
            return -1;
        }

        // Asegurarse de que todos los vecinos están en la Q-table para este estado
        for (auto &&neighbor : neighbors) {
            if (!qTable["q_table"][String(state)].containsKey(String(neighbor))) {
                Serial.print("Adding missing action for neighbor ");
                Serial.println(neighbor);
                qTable["q_table"][String(state)][String(neighbor)] = 0.0; // Inicializar Q-value para la acción faltante
            }
        }

        Serial.println("Available actions are:");
        serializeJsonPretty(qTable["q_table"][String(state)], Serial);

        float best_value = -1.0;
        String best_action = "";
        Serial.println("Starting exploitation phase...");
        Serial.flush();
        JsonObject actions = qTable["q_table"][String(state)];
        for (JsonPair kv : actions) {
            String action = kv.key().c_str();
            float value = kv.value().as<float>();
            uint32_t action_int = action.toInt();
            Serial.print("Checking action: ");
            Serial.print(action.c_str());
            Serial.print(" with value ");
            Serial.println(value);
            Serial.flush();
            if (value > best_value && std::find(neighbors.begin(), neighbors.end(), action_int) != neighbors.end()) {
                Serial.print("Action ");
                Serial.print(action.c_str());
                Serial.print(" is a valid neighbor and has a better value ");
                Serial.println(value);
                Serial.flush();
                best_value = value;
                best_action = action;
            } else {
                Serial.print("Action ");
                Serial.print(action.c_str());
                Serial.println(" is not valid or does not have a better value");
                Serial.flush();
            }
        }
        Serial.print("Exploiting best action: ");
        Serial.print(best_action.c_str());
        Serial.print(" with value ");
        Serial.println(best_value);
        Serial.flush();
        return best_action.toInt();
    }
}

void sendMessageToNextHop(uint32_t next_hop, String &msg) {
  Serial.print("Sending message to next hop ");
  Serial.print(next_hop);
  Serial.print(": ");
  Serial.println(msg);
  Serial.flush();
  mesh.sendSingle(next_hop, msg);
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("Initializing INTERMEDIATE NODE");

  for (uint8_t t = 10; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
