// This is to be flashed in a master node
#include <ESP8266WiFi.h>
#include <vector>
#include <unordered_map>
#include <random>
#include <esp_mesh.h>

// Mesh configuration
#define   MESH_PREFIX     "Q_LEARNING_MESH"
#define   MESH_PASSWORD   "1234" 
#define   MESH_PORT       5555 // default port

// Q-Learning Parameters
#define ALPHA 0.1  // Learning rate
#define GAMMA 0.9  // Discount factor
#define EPSILON 0.1  // Exploration rate

// Q-Table
std::unordered_map<int, std::vector<float>> qTable;

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0.0, 1.0);

// Function prototypes
float generate_random_number();
int chooseAction(int state, const std::vector<int>& neighbors);
void updateQTable(int state, int action, float reward, int nextState);
void sendPacket(int& currentState);
bool am_i_master();
void onReceive(mesh_addr_t *from, uint8_t *data, uint16_t len);

// Mesh network parameters
mesh_addr_t masterAddr;
std::unordered_map<int, std::vector<int>> dynamic_neighbors;

void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialize Wi-Fi and Mesh
    WiFi.mode(WIFI_STA);
    esp_mesh_init();
    esp_mesh_set_receive_cb(onReceive);

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
}

void loop() {
    while (true) {
        sendPacket(currentState);

        if (amIMaster()) {
            Serial.println("Master server found, episode finished");
            ESP.deepSleep(0);  // 0 indicates it wakes up with a reset
            delay(100);  // Wait a bit to ensure deepSleep() starts correctly
        }
    }
}

void sendPacket(int& state) {
    Serial.print("Current state: ");
    Serial.println(state);

    std::vector<int> neighbors = dynamic_neighbors[state];
    if (neighbors.empty()) {
        Serial.println("No neighbors found, cannot send packet");
        return;
    }

    int action = chooseAction(state, neighbors);
    Serial.print("CHOSEN ACTION INDEX ");
    Serial.println(action);

    int nextState = neighbors[action];
    Serial.print("ACTUAL HOP WILL BE TO NODE ");
    Serial.println(nextState);

    float reward = am_i_master() ? 1.0 : -0.1;  // Reward for reaching master, penalty otherwise
    Serial.print("Reward: ");
    Serial.println(reward);

    updateQTable(state, action, reward, nextState);

    Serial.println("Updated Q-Table:");
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

    // Prepare packet
    String packet = "hello world";
    packet += "|";
    packet += "ALPHA:" + String(ALPHA);
    packet += " GAMMA:" + String(GAMMA);
    packet += " EPSILON:" + String(EPSILON);

    uint8_t data[packet.length() + 1];
    packet.toCharArray((char*)data, packet.length() + 1);

    // Send packet
    mesh_addr_t nextAddr;
    esp_mesh_get_parent_bssid(&nextAddr);
    esp_mesh_send(&nextAddr, data, packet.length() + 1, NULL, 0);

    state = nextState;  // Update the current state

    delay(1000);  // Delay to simulate packet transmission
}

int chooseAction(int state, const std::vector<int>& neighbors) {
    float randNum = generate_random_number();
    Serial.print("Generated random number: ");
    Serial.println(randNum);

    if (randNum < EPSILON) {
        int action = std::uniform_int_distribution<>(0, neighbors.size() - 1)(gen);  // Explore
        Serial.println("Exploring action");
        return action;
    } else {
        // Exploit: Choose the action with the highest Q-value
        std::vector<float>& qValues = qTable[state];
        int bestAction = 0;
        for (int i = 1; i < qValues.size(); ++i) {
            if (qValues[i] > qValues[bestAction]) {
                bestAction = i;
            }
        }
        Serial.println("Exploiting best action");
        return bestAction;
    }
}

void updateQTable(int state, int action, float reward, int nextState) {
    std::vector<float>& qValues = qTable[state];
    float maxQ = *std::max_element(qTable[nextState].begin(), qTable[nextState].end());
    qValues[action] = qValues[action] + ALPHA * (reward + GAMMA * maxQ - qValues[action]);

    Serial.print("Updated Q-value for state ");
    Serial.print(state);
    Serial.print(", action ");
    Serial.print(action);
    Serial.print(": ");
    Serial.println(qValues[action]);
    Serial.println();
}

float generate_random_number() {
    return dis(gen);
}

bool amIMaster() {
    return false;
}

void onReceive(mesh_addr_t *from, uint8_t *data, uint16_t len) {
    Serial.print("Received packet from: ");
    for (int i = 0; i < 6; ++i) {
        Serial.print(from->addr[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();

    // Process received data
    String packet = String((char*)data);
    Serial.print("Packet data: ");
    Serial.println(packet);

    // Parse Q-Table and parameters from packet (if present)
    // This is just a dummy example, adjust parsing logic as necessary
    int alphaPos = packet.indexOf("ALPHA:");
    int gammaPos = packet.indexOf("GAMMA:");
    int epsilonPos = packet.indexOf("EPSILON:");

    if (alphaPos != -1 && gammaPos != -1 && epsilonPos != -1) {
        ALPHA = packet.substring(alphaPos + 6, gammaPos).toFloat();
        GAMMA = packet.substring(gammaPos + 6, epsilonPos).toFloat();
        EPSILON = packet.substring(epsilonPos + 8).toFloat();
    }

    // Update dynamic neighbors based on received packet
    int fromNode = 0;  // Extract node ID from packet or address
    std::vector<int> neighbors = {1, 2};  // Extract neighbor IDs from packet
    dynamic_neighbors[fromNode] = neighbors;
}
