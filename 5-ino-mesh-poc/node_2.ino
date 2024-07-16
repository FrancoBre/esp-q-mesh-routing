// This is to be flashed in node 2
#include <ESP8266WiFi.h>
#include <vector>
#include <unordered_map>
#include <painlessMesh.h>

// Mesh configuration
#define MESH_PREFIX     "Q_LEARNING_MESH"
#define MESH_PASSWORD   "1234"
#define MESH_PORT       5555 // default port

// Q-Table
std::unordered_map<int, std::vector<float>> qTable;

// Function prototypes
float generate_random_number();
int chooseAction(int state, const std::vector<int>& neighbors);
void updateQTable(int state, int action, float reward, int nextState);
void sendPacket(int& currentState);
void onReceive(uint32_t from, String &msg);
void sendMessage();

// Mesh network parameters
painlessMesh mesh;
Scheduler userScheduler;

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );

void sendMessage() {
    Serial.print("NODE 1: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }

    Serial.println();
    Serial.println("SENDING MESSAGE FROM NODE 2");
    String msg = "Hello from node ";
    msg += mesh.getNodeId();
    mesh.sendBroadcast(msg);
    taskSendMessage.setInterval(random(TASK_SECOND * 1, TASK_SECOND * 5));
}

// Needed for painless library
void receivedCallback(uint32_t from, String &msg) {
    Serial.printf("NODE 2: startHere: Received from %u msg=%s\n", from, msg.c_str());
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("NODE 2: --> startHere: New Connection, nodeId = %u\n", nodeId);
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }
}

void changedConnectionCallback() {
    Serial.printf("NODE 2: Changed connections\n");
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("NODE 2: Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }
}

void setup() {
    Serial.begin(115200);

    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    userScheduler.addTask(taskSendMessage);
    taskSendMessage.enable();
  
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }
    Serial.println();

  /*
    // TODO get neighbors to initialize q table
    // Initialize Q-Table
    for (int i = 0; i < 6; ++i) {
        qTable[i] = std::vector<float>(4, 0.0);  // Assuming max 4 neighbors
    }

    Serial.println("Q-Table initialized:");
    for (const auto& pair : qTable) {
        Serial.print("State ");
        Serial.print(pair.first);
        Serial.print(": ");
        for (float value : pair.second) {
            Serial.print(value);
            Serial.print(" ");
        }
        Serial.println();
    }
    */
}

void loop() {
    mesh.update();

    // choose_action();
    /*
    while (true) {
        sendPacket(currentState);

        if (amIMaster()) {
            Serial.println("Master server found, episode finished");
            ESP.deepSleep(0);  // 0 indicates it wakes up with a reset
            delay(100);  // Wait a bit to ensure deepSleep() starts correctly
        }
    }
    */
}

void onReceive(uint32_t from, String &msg) {
    Serial.print("NODE 2: ");
    Serial.println(mesh.getNodeId());

    std::list<uint32_t> nodes = mesh.getNodeList();
    Serial.println("CONNECTED NODES: ");
    Serial.print("Num nodes: ");
    Serial.println(nodes.size());
    Serial.println("Connection list:");

    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      Serial.print(*node);
      node++;
    }

    Serial.println();
    Serial.print("Received packet from: ");
    Serial.println(from);

    // Process received data
    Serial.print("Packet data: ");
    Serial.println(msg);

    // Parse Q-Table and parameters from packet (if present)
    // This is just a dummy example, adjust parsing logic as necessary
    int alphaPos = msg.indexOf("ALPHA:");
    int gammaPos = msg.indexOf("GAMMA:");
    int epsilonPos = msg.indexOf("EPSILON:");

    if (alphaPos != -1 && gammaPos != -1 && epsilonPos != -1) {
        float alpha = msg.substring(alphaPos + 6, gammaPos).toFloat();
        float gamma = msg.substring(gammaPos + 6, epsilonPos).toFloat();
        float epsilon = msg.substring(epsilonPos + 8).toFloat();
    }

  /*
    // Update dynamic neighbors based on received packet
    int fromNode = 0;  // Extract node ID from packet or address
    std::vector<int> neighbors = {1, 2};  // Extract neighbor IDs from packet
    // dynamic_neighbors[fromNode] = neighbors; // Placeholder for dynamic neighbors update
  // */
}
