#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <random>
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

void receivedCallback(uint32_t from, String &msg);
void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc);
int chooseAction(int state, JsonDocument& doc, float epsilon);
void sendMessageToNextHop(uint32_t next_hop, String &msg);

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());

  // Deserialize the JSON message
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  // Extract q-learning parameters and state information
  float q_alpha = doc["q_parameters"]["alpha"];
  float q_gamma = doc["q_parameters"]["gamma"];
  float q_epsilon = doc["q_parameters"]["epsilon"];
  float q_epsilonDecay = doc["q_parameters"]["epsilon_decay"];

  Serial.println("Extracted Q-learning parameters:");
  Serial.printf("alpha: %f, gamma: %f, epsilon: %f, epsilonDecay: %f\n", q_alpha, q_gamma, q_epsilon, q_epsilonDecay);

  int current_episode = doc["current_episode"];
  float accumulated_reward = doc["accumulated_reward"];
  float total_time = doc["total_time"];

  Serial.println("Extracted episode information:");
  Serial.printf("Current episode: %d, Accumulated reward: %f, Total time: %f\n", current_episode, accumulated_reward, total_time);

  JsonArray episodes = doc["episodes"];
  for (JsonObject episode : episodes) {
    int episode_number = episode["episode_number"];
    float reward = episode["reward"];
    float time = episode["time"];

    Serial.println("Processing episode:");
    Serial.printf("Episode number: %d, Reward: %f, Time: %f\n", episode_number, reward, time);

    JsonArray steps = episode["steps"];
    for (JsonObject step : steps) {
      int hop = step["hop"];
      String node_from = step["node_from"];
      String node_to = String(mesh.getNodeId());

      Serial.println("Processing step:");
      Serial.printf("Hop: %d, Node from: %s, Node to: %s\n", hop, node_from.c_str(), node_to.c_str());

      // Add reward to episode
      episode["reward"] = -1; // This node is not master!
      Serial.println("Updated episode reward to -1");

      // Update Q-Table
      updateQTable(node_from, node_to, episode["reward"], q_alpha, q_gamma, doc);

      // Choose next action using epsilon-greedy policy
      int next_action = chooseAction(mesh.getNodeId(), doc, q_epsilon);

      Serial.printf("Chosen next action: %d\n", next_action);

      // Send updated message to the next hop
      String updatedJsonString;
      serializeJson(doc, updatedJsonString);
      sendMessageToNextHop(next_action, updatedJsonString);
    }
  }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument& doc) {
  JsonObject q_table = doc["q_table"];
  
  // Ensure state_from exists in q_table
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
  }

  // Ensure state_to exists in q_table[state_from]
  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = 0.0; // Initialize Q-value if not present
  }

  // Retrieve Q-value to update
  float currentQ = q_table[state_from][state_to].as<float>();

  // Calculate maxQ for state_to
  float maxQ = -9999.0; // Start with a very low value
  if (q_table.containsKey(state_to)) {
    JsonObject actions = q_table[state_to];
    for (JsonPair kv : actions) {
      float value = kv.value().as<float>();
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
}

int chooseAction(int state, JsonDocument& doc, float epsilon) {
  auto nodes = mesh.getNodeList(true);
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

  if (num_neighbors == 0) {
    Serial.println("No neighbors found");
    return -1;
  }

  Serial.printf("Neighbors for state %s are %s\n", String(state), nodesStr);

  if (random() < epsilon) {
    // Explore
    int action_index = random(0, num_neighbors - 1);
    int action = neighbors[action_index];
    Serial.println("Exploring action");
    Serial.println(action);
    return action;
  } else {
    // Exploit: Choose the action with the highest Q-value
    JsonObject q_table = doc["q_table"];
    if (!q_table.containsKey(String(state))) {
      Serial.println("No Q-values found for state");
      return -1;
    }

    JsonObject actions = q_table[String(state)];
    float best_value = -1.0;
    String best_action = "";
    Serial.println("Starting exploitation phase...");
    for (JsonPair kv : actions) {
      String action = kv.key().c_str();
      float value = kv.value().as<float>();
      uint32_t action_int = action.toInt();
      Serial.printf("Checking action: %s with value %f\n", action.c_str(), value);
      if (value > best_value && std::find(neighbors.begin(), neighbors.end(), action_int) != neighbors.end()) {
        Serial.printf("Action %s is a valid neighbor and has a better value %f\n", action.c_str(), value);
        best_value = value;
        best_action = action;
      } else {
        Serial.printf("Action %s is not valid or does not have a better value\n", action.c_str());
      }
    }
    Serial.printf("Exploiting best action: %s with value %f\n", best_action.c_str(), best_value);
    return best_action.toInt();
  }
}

void sendMessageToNextHop(uint32_t next_hop, String &msg) {
  Serial.printf("Sending message to next hop %u: %s\n", next_hop, msg.c_str());
  mesh.sendSingle(next_hop, msg);
}

void setup() {
  Serial.begin(115200);

//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
