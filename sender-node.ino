/*
 * SENDER NODE
 */
#include "painlessMesh.h"

#include "DHT.h"

#define MESH_PREFIX "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT 5555

#define DPIN 4 //Pin to connect DHT sensor (GPIO number) D2
#define DTYPE DHT11 // Define DHT 11 or DHT22 sensor type

// Hyperparameters (only for node 1 to start the learning)
String g_alpha = "0.1"; // Learning rate
String g_gamma = "0.9"; // Discount factor
String g_epsilon = "0.1"; // Exploration rate
String g_epsilonDecay = "0.1"; // Exploration rate
int currentEpisode = 1;

JsonDocument qTable;
StaticJsonDocument < 4096 > persistentDoc;
JsonArray episodes = persistentDoc.createNestedArray("episodes");
String episodesString;
float accumulatedReward = 0.0;

bool isFirstTime = true;
bool initialMessageSent = false;
bool episodeFinishedReceived = true;
unsigned long lastSentMessage = 0;
unsigned long twentySecondsInMillis = 20000;

void sendMessage();
void sendInitialMessage();
void refreshQTableOnConnectionChange();
String getNetworkDataToSend();

//Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, & sendInitialMessage);
Scheduler userScheduler;
painlessMesh mesh;
DHT dht(DPIN, DTYPE);

void sendInitialMessage() {
  if (!initialMessageSent) {
    sendMessage();
  }
}

void sendMessage() {
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("----------------------------");
  Serial.print("STARTING EPISODE: ");
  Serial.println(String(currentEpisode));
  Serial.println("----------------------------");
  if (episodeFinishedReceived || currentEpisode > 100) { // 100 episodes for learning phase
    mesh.subConnectionJson(true);
    StaticJsonDocument < 1024 > doc;

    doc["hop"] = true;
    JsonObject payload = doc.createNestedObject("payload");
    payload["tem"] = dht.readTemperature(false); //Read temperature in C
    payload["hum"] = dht.readHumidity(); //Read Humidity

    doc["current_node_id"] = String(mesh.getNodeId());

    JsonObject q_parameters = doc.createNestedObject("q_parameters");
    q_parameters["alpha"] = g_alpha;
    q_parameters["gamma"] = g_gamma;
    q_parameters["epsilon"] = g_epsilon;
    q_parameters["epsilon_decay"] = g_epsilonDecay;

    doc["current_episode"] = currentEpisode;
    doc["accumulated_reward"] = accumulatedReward;

    // if episode is greater than 10, flush the episodes array for it
    // not to grow so much that it occupies a lot of memory
    // (middleware already has them anyway)
    if (currentEpisode != 1 && currentEpisode < 10) {
      deserializeJson(episodes, episodesString);
    } else {
      episodes = persistentDoc.createNestedArray("episodes");
      serializeJson(episodes, episodesString);
    }

    JsonObject episode = episodes.createNestedObject();
    episode["episode_number"] = currentEpisode;
    episode["reward"] = 0.0;

    JsonArray stepsArray = episode.createNestedArray("steps");
    JsonObject step = stepsArray.createNestedObject();
    step["hop"] = 0;
    step["node_from"] = String(mesh.getNodeId());

    JsonObject q_table = doc.createNestedObject("q_table");
    q_table = qTable["q_table"];

    int next_action = chooseAction(mesh.getNodeId(), doc, g_epsilon.toFloat());

    if (next_action == -1) {
      return;
    }

    step["node_to"] = String(next_action);

    doc["episodes"] = episodes;

    String jsonString;
    serializeJsonPretty(doc, jsonString);

    Serial.println("Data to send : ");
    Serial.println(jsonString);

    Serial.print("To node with id: ");
    Serial.println(next_action);

    int retryCount = 0;
    const int maxRetries = 10;

    while (retryCount < maxRetries) {
      if (currentEpisode == 5) {
        Serial.println();
        Serial.println();
        Serial.println("Current episode is mf 5, looking for mf node to send again");
        Serial.println();
        Serial.println();

        auto nodes = mesh.getNodeList(true);

        for (auto && id: nodes) {
          Serial.print("ORIGINAL NODE ID: ");
          Serial.println(next_action);

          String new_next_action = String(id);
          Serial.print("NEW NODE ID: ");
          Serial.println(new_next_action);
          mesh.sendSingle(next_action, jsonString);
        }

        //mesh.sendSingle(next_action, jsonString);
        //mesh.sendSingle(next_action, jsonString);
      }

      if (mesh.sendSingle(next_action, jsonString)) {
        initialMessageSent = true;
        episodeFinishedReceived = false;
        lastSentMessage = millis();
        Serial.println("Hop sent successfully");
        return;
      }
      retryCount++;
      delay(100);
    }

    // TODO: hay un problema con esto, me parece que va a haber que agarrar
    //  el número de episodio que nos llegue desde el broadcast y subir ese
    /*
    currentEpisode++;
    Serial.println("");
    Serial.print("Episode number increased to ");
    Serial.println(currentEpisode);
    */
    Serial.println("Failed to send hop!");
    return;
  }
}

bool isHopMessage(StaticJsonDocument < 1024 > doc) {
  return doc.containsKey("hop");
}

bool isBroadcastMessage(StaticJsonDocument < 1024 > doc) {
  return doc.containsKey("broadcast");
}

// Needed for painless library
void receivedCallback(uint32_t from, String & msg) {
  if (isFirstTime) {
    isFirstTime = false;
    sendInitialMessage();
  }
  //Serial.print("startHere: Received from ");
  //Serial.print(from);
  //Serial.print(" msg=");
  Serial.println(msg.c_str());
  //Serial.println();

  // Deserialize the JSON message
  StaticJsonDocument < 1024 > doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    return;
  }

  if (isHopMessage(doc)) {
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("----------------------------");
    Serial.println("RECEIVED HOP");
    Serial.println("----------------------------");
    // Extract q-learning parameters and state information
    float g_alpha = doc["q_parameters"]["alpha"];
    float g_gamma = doc["q_parameters"]["gamma"];
    float g_epsilon = doc["q_parameters"]["epsilon"];
    float g_epsilonDecay = doc["q_parameters"]["epsilon_decay"];

    int current_episode = doc["current_episode"];
    float accumulated_reward = doc["accumulated_reward"];

    JsonArray receivedEpisodes = doc["episodes"];
    for (JsonObject episode: receivedEpisodes) {
      int episode_number = episode["episode_number"];

      // Add reward to episode
      float updatedReward = ((float) episode["reward"]) - 1.00; // This node is not master!
      episode["reward"] = String(updatedReward);

      float accumulated_reward = ((float) episode["accumulated_reward"]) - 1.00;
      episode["accumulated_reward"] = String(accumulated_reward);

      //Serial.println("This node is not master! Reduce episode reward in 1");
      //Serial.print("Updated reward: ");
      //Serial.println(String(episode["reward"]));
      //Serial.flush();

      // Choose next action using epsilon-greedy policy
      int next_action = chooseAction(mesh.getNodeId(), doc, g_epsilon);

      if (next_action == -1) {
        return;
      }

      JsonArray steps = episode["steps"];
      int hop = steps.size();

      // Crear el nuevo hop
      JsonObject newHop = steps.createNestedObject();
      newHop["hop"] = hop;
      newHop["node_from"] = String(mesh.getNodeId());
      newHop["node_to"] = String(next_action);

      // Update Q-Table
      updateQTable(newHop["node_from"], newHop["node_to"], episode["reward"], g_alpha, g_gamma, doc);

      // Send updated message to the next hop
      String updatedJsonString;
      serializeJson(doc, updatedJsonString);
      qTable = doc["q_table"];
      doc["hop"] = true;
      sendMessageToNextHop(next_action, updatedJsonString);
    }
  }
  // Message is a q_table update broadcast
  else if (isBroadcastMessage(doc)) {
    //Serial.println("Received Q-Table update:");
    //serializeJsonPretty(doc, Serial);
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println("----------------------------");
    Serial.println("RECEIVED BROADCAST");
    Serial.println("----------------------------");

    if (doc.containsKey("q_table")) {
      JsonDocument qtabledoc = doc["q_table"];
      qTable = qtabledoc;
    } else {
      //Serial.println("Unable to get q table from q table update");
    }

    if (doc.containsKey("alpha")) {
      g_alpha = String(doc["alpha"]);
    } else {
      //Serial.println("Unable to get alpha from q table update");
    }

    if (doc.containsKey("gamma")) {
      g_gamma = String(doc["gamma"]);
    } else {
      //Serial.println("Unable to get gamma from q table update");
    }

    if (doc.containsKey("epsilon")) {
      g_epsilon = String(doc["epsilon"]);
      //Serial.println("Got mf epsilon from q table update successfully");
    } else {
      //Serial.println("Unable to get epsilon from q table update");
    }

    if (doc.containsKey("episodes")) {
      serializeJsonPretty(doc["episodes"], episodesString);

      Serial.println();
    } else {
      //Serial.println("Unable to get episodes from q table update");
    }

    if (doc.containsKey("accumulated_reward")) {
      accumulatedReward = doc["accumulated_reward"];
    } else {
      //Serial.println("Unable to get accumulated_reward from q table update");
    }

    if (doc.containsKey("current_episode")) {
      int newCurrentEpisode = ((int) doc["current_episode"]) + 1;
      if (newCurrentEpisode == currentEpisode) {
        Serial.println("OH NO! episode is stuck!");
        Serial.println("Mf restarting the node...");
        ESP.restart();
      }
      currentEpisode = newCurrentEpisode;
    } else {
      //Serial.println("Unable to get current_episode from q table update");
    }

    episodeFinishedReceived = true;
    sendMessage();
  } else {
    //Serial.println("Unknown message structure");
    Serial.flush();
  }
}

void updateQTable(String state_from, String state_to, float reward, float alpha, float gamma, JsonDocument & doc) {
  auto nodes = mesh.getNodeList(true);

  JsonObject q_table = doc["q_table"];
  for (auto && id: nodes) {
    String node_from = String(id);
    for (auto && id_2: nodes) {
      String node_to = String(id_2);

      if (id_2 != id) {
        if (!q_table.containsKey(node_from)) {
          q_table.createNestedObject(node_from);
        }

        if (!q_table[node_from].containsKey(node_to)) {
          q_table[node_from][node_to] = 0.0; // Initialize Q-value if not present
        }
      }
    }
  }

  // Ensure state_from exists in q_table
  if (!q_table.containsKey(state_from)) {
    q_table.createNestedObject(state_from);
  }

  // Ensure state_to exists in q_table[state_from]
  if (!q_table[state_from].containsKey(state_to)) {
    q_table[state_from][state_to] = 0.0; // Initialize Q-value if not present
  }

  // Retrieve Q-value to update
  float currentQ = q_table[state_from][state_to].as < float > ();

  // Calculate maxQ for state_to
  float maxQ = 0.0; // Start with zero if no actions are found
  if (q_table.containsKey(state_to)) {
    JsonObject actions = q_table[state_to];
    for (JsonPair kv: actions) {
      float value = kv.value().as < float > ();
      if (value > maxQ) {
        maxQ = value;
      }
    }
  }

  // Apply Bellman equation to update Q-value
  float updatedQ = currentQ + alpha * (reward + gamma * maxQ - currentQ);

  // Update Q-value in q_table
  q_table[state_from][state_to] = updatedQ;
}

void refreshQTableOnConnectionChange() {
  auto nodes = mesh.getNodeList(true);
  JsonObject q_table = qTable["q_table"];

  for (auto && id: nodes) {
    bool nodeIsUp = mesh.sendSingle(id, "healthcheck");

    String node_from = String(id);

    if (nodeIsUp) {
      // Si el nodo está activo, verificamos si ya está en la Q-table
      if (!q_table.containsKey(node_from)) {
        q_table.createNestedObject(node_from);
      }

      for (auto && id_2: nodes) {
        String node_to = String(id_2);

        if (id_2 != id) {
          if (!q_table[node_from].containsKey(node_to)) {
            q_table[node_from][node_to] = 0.0; // Inicializar Q-value si no está presente
          }
        }
      }
    } else {
      // Si el nodo no está activo, lo removemos de la Q-table
      if (q_table.containsKey(node_from)) {
        q_table.remove(node_from);
      }

      // También eliminar las referencias a este nodo en las demás entradas de la Q-table
      for (auto && id_2: nodes) {
        String node_to = String(id_2);

        if (q_table.containsKey(node_to)) {
          if (q_table[node_to].containsKey(node_from)) {
            q_table[node_to].remove(node_from);
          }
        }
      }
    }
  }
}

int chooseAction(int state, JsonDocument & doc, float epsilon) {
  auto nodes = mesh.getNodeList(false);
  std::vector < int > neighbors;
  String nodesStr;
  int num_neighbors = 0;

  Serial.println();
  Serial.print("Iterating over node list ");
  for (auto && id: nodes) {
    neighbors.push_back(id);
    nodesStr += String(id) + String(" ");
    Serial.print(nodesStr);
    num_neighbors++;
  }
  Serial.println();
  Serial.flush();

  if (num_neighbors == 0) {
    Serial.println("No neighbors found");
    return -1;
  }

  if (isFirstTime) {
    //Serial.println("Populating QTable for the first time");

    // Asegurarse de que cada estado tiene un objeto dentro de la Q-table
    for (auto && id: nodes) {
      if (!qTable["q_table"].containsKey(String(state))) {
        JsonObject stateObj = qTable["q_table"].createNestedObject(String(state));
        for (auto && action: nodes) {
          stateObj[String(action)] = 0.0; // Inicializar valores Q para cada acción
        }
      }
    }

    serializeJsonPretty(qTable["q_table"], Serial);
    isFirstTime = false;
  } else {
    //Serial.println("Updating QTable with missing neighbors");

    // Verificar si los vecinos están en la Q-table para el estado actual
    if (!qTable.containsKey(String(state))) {
      for (auto && id: nodes) {
        if (!qTable[String(state)].containsKey(String(id))) {
          //Serial.print("Adding missing action for neighbor ");
          //Serial.println(String(id));
          qTable[String(state)][String(id)] = 0.0; // Inicializar valor Q para la acción faltante
        }
      }
    }

    //serializeJsonPretty(qTable, Serial);
  }

  Serial.flush();

  //Serial.print("Neighbors for state ");
  //Serial.print(String(state));
  //Serial.print(" are ");
  //Serial.println(nodesStr);
  //Serial.flush();

  if (random(0, 100) < epsilon * 100) {
    // Explorar
    int action_index = random(0, num_neighbors);
    int action = neighbors[action_index];
    //Serial.println("Exploring action");
    //Serial.println(action);
    //Serial.flush();
    return action;
  } else {
    // Explotar: elegir la acción con el valor Q más alto
    //Serial.println("Starting exploitation phase...");
    //Serial.flush();

    float best_value = -1.0;
    String best_action = "";

    JsonObject actions = qTable[String(state)];

    for (JsonPair kv: actions) {
      String action = kv.key().c_str();
      float value = kv.value().as < float > ();
      uint32_t action_int = action.toInt();

      //Serial.print("Checking action: ");
      //Serial.print(action.c_str());
      //Serial.print(" with value ");
      //Serial.println(value);
      //Serial.flush();

      if (value > best_value && std::find(neighbors.begin(), neighbors.end(), action_int) != neighbors.end()) {
        best_value = value;
        best_action = action;
      }
    }

    if (best_action != "") {
      //Serial.print("Exploiting best action: ");
      //Serial.print(best_action.c_str());
      //Serial.print(" with value ");
      //Serial.println(best_value);
      //Serial.flush();
      int bestActionInt = best_action.toInt();
      return bestActionInt;
    } else {
      //Serial.println("No valid action found");
      return -1;
    }
  }
}

void sendMessageToNextHop(uint32_t next_hop, String & msg) {
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("----------------------------");
  Serial.print("SENDING MESSAGE TO NEXT HOP ");
  Serial.println(next_hop);
  Serial.println("----------------------------");
  Serial.flush();

  Serial.println(msg);
  mesh.sendSingle(next_hop, msg);
}

void newConnectionCallback(uint32_t nodeId) {
  //Serial.print("New Connection, nodeId = ");
  //Serial.println(nodeId);
  refreshQTableOnConnectionChange();
}

void changedConnectionCallback() {
  //Serial.println("Connections Changed");
  refreshQTableOnConnectionChange();
}

void nodeTimeAdjustedCallback(int32_t offset) {
  //Serial.print("Node Time Adjusted by offset = ");
  //Serial.println(offset);
  refreshQTableOnConnectionChange();
}

void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("Initializing SENDER NODE");

  for (uint8_t t = 10; t > 0; t--) {
    Serial.printf("[SETUP] WAIT %d...\n", t);
    Serial.flush();
    delay(1000);
  }

  dht.begin();
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | COMMUNICATION); // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, & userScheduler, MESH_PORT);

  mesh.onReceive( & receivedCallback);
  mesh.onNewConnection( & newConnectionCallback);
  mesh.onChangedConnections( & changedConnectionCallback);
  mesh.onNodeTimeAdjusted( & nodeTimeAdjustedCallback);

  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  Serial.println("Mesh initialized successfully");

  //userScheduler.addTask(taskSendMessage);
  //taskSendMessage.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
