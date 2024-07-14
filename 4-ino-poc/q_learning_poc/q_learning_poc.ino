#include <ESP8266WiFi.h>
#include <vector>
#include <unordered_map>
#include <random>

// Q-Learning Parameters
#define ALPHA 0.1  // Learning rate
#define GAMMA 0.9  // Discount factor
#define EPSILON 0.1  // Exploration rate

// Q-Table
std::unordered_map<int, std::vector<float>> qTable;

// Network topology
std::unordered_map<int, std::vector<int>> neighbors_dict = {
    {0, {1, 2}},
    {1, {0, 3, 4}},
    {2, {0, 4}},
    {3, {1, 5}},
    {4, {1, 2, 5}},
    {5, {3, 4}}  // Master server
};

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(0.0, 1.0);

// Function prototypes
float generate_random_number();
int chooseAction(int state);
void updateQTable(int state, int action, float reward, int nextState);
void sendPacket(int& currentState);

void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialize Q-Table
    for (const auto& pair : neighbors_dict) {
        int state = pair.first;
        qTable[state] = std::vector<float>(pair.second.size(), 0.0);
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
    int currentState = 0;  // Start from node 0

    while (true) {
        sendPacket(currentState);

        if (currentState == 5) {
            Serial.println("Master server found, episode ended");
            ESP.deepSleep(0);  // 0 indica que se despierta con un reinicio
            delay(100);  // Esperar un poco para asegurar que deepSleep() se inicie correctamente
        }
    }
}

void sendPacket(int& state) {
    Serial.println();
    Serial.print("Current state: ");
    Serial.println(state);

    int action = chooseAction(state);
    Serial.print("CHOSEN ACTION INDEX ");
    Serial.println(action);

    Serial.print("HOP OPTIONS FOR STATE ");
    Serial.print(state);
    Serial.print(" ARE: ");

    // NEIGHBORS DICT PRINT
    Serial.print("neighbors_dict[");
    Serial.print(state);
    Serial.print("]: ");

    // Check if the state exists in the map
    if (neighbors_dict.find(state) != neighbors_dict.end()) {
      // Get the vector associated with the state
      std::vector<int> neighbors = neighbors_dict[state];

      // Print each element in the vector
      Serial.print("[");
      for (size_t i = 0; i < neighbors.size(); ++i) {
          Serial.print(neighbors[i]);
          if (i < neighbors.size() - 1) {
              Serial.print(", ");
          }
      }
      Serial.println("]");
    }

    int nextState = neighbors_dict[state][action];
    Serial.print("ACTUAL HOP WILL BE TO NODE ");
    Serial.println(nextState);

    float reward = (nextState == 5) ? 1.0 : -0.1;  // Reward for reaching master, penalty otherwise
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

    state = nextState;  // Update the current state

    delay(1000);  // Delay to simulate packet transmission
}

int chooseAction(int state) {
    const double random_number = dis(gen);
    Serial.print("random number: ");
    Serial.println(random_number);

    if (dis(gen) < EPSILON) {
        int action = std::uniform_int_distribution<>(0, neighbors_dict[state].size() - 1)(gen);  // Explore
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
