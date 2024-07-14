#include <iostream>
#include <vector>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <thread>  // For sleep functionality
#include <chrono>  // For time duration
#include <numeric> // For std::accumulate

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

// Function Prototypes
int chooseAction(int state);
void updateQTable(int state, int action, float reward, int nextState);
std::vector<int> sendPacket(int& currentState, int& steps, float& totalReward);

int main() {
    // Initialize Q-Table
    for (const auto& pair : neighbors_dict) {
        int state = pair.first;
        qTable[state] = std::vector<float>(pair.second.size(), 0.0);
    }

    // Metrics
    std::vector<int> stepsPerEpisode;
    std::vector<float> rewardsPerEpisode;

    // Simulate packet sending
    for (int episode = 0; episode < 100; ++episode) {  // Run for 100 episodes
        int currentState = 0;  // Start from node 0
        int steps = 0;
        float totalReward = 0.0;
        std::vector<int> path = sendPacket(currentState, steps, totalReward);

        // Store metrics
        stepsPerEpisode.push_back(steps);
        rewardsPerEpisode.push_back(totalReward);

        // Print the path taken in the current episode
        std::cout << "Episode " << episode + 1 << " path: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;

        // Print the Q-Table
        std::cout << "Q-Table after episode " << episode + 1 << ":" << std::endl;
        for (const auto& pair : qTable) {
            std::cout << "State " << pair.first << ": ";
            for (float value : pair.second) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }

        // Add a new line and sleep for 1 second
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Print performance metrics
    std::cout << "Performance Metrics:" << std::endl;
    std::cout << "Steps per episode: ";
    for (int steps : stepsPerEpisode) {
        std::cout << steps << " ";
    }
    std::cout << std::endl;

    std::cout << "Rewards per episode: ";
    for (float reward : rewardsPerEpisode) {
        std::cout << reward << " ";
    }
    std::cout << std::endl;

    // Average steps and rewards
    float avgSteps = std::accumulate(stepsPerEpisode.begin(), stepsPerEpisode.end(), 0) / static_cast<float>(stepsPerEpisode.size());
    float avgRewards = std::accumulate(rewardsPerEpisode.begin(), rewardsPerEpisode.end(), 0.0) / static_cast<float>(rewardsPerEpisode.size());
    
    std::cout << "Average steps per episode: " << avgSteps << std::endl;
    std::cout << "Average rewards per episode: " << avgRewards << std::endl;

    return 0;
}

std::vector<int> sendPacket(int& state, int& steps, float& totalReward) {
    std::vector<int> path;  // To store the path for the current episode
    while (state != 5) {  // Until reaching the master server
        path.push_back(state);
        int action = chooseAction(state);
        int nextState = neighbors_dict[state][action];

        float reward = (nextState == 5) ? 1.0 : -0.1;  // Reward for reaching master, penalty otherwise

        updateQTable(state, action, reward, nextState);

        state = nextState;  // Update the current state
        totalReward += reward;
        steps++;
    }
    path.push_back(state);  // Add the final state (master server) to the path
    return path;
}

int chooseAction(int state) {
    if (dis(gen) < EPSILON) {
        return std::uniform_int_distribution<>(0, neighbors_dict[state].size() - 1)(gen);  // Explore
    } else {
        // Exploit: Choose the action with the highest Q-value
        std::vector<float>& qValues = qTable[state];
        int bestAction = 0;
        for (int i = 1; i < qValues.size(); ++i) {
            if (qValues[i] > qValues[bestAction]) {
                bestAction = i;
            }
        }
        return bestAction;
    }
}

void updateQTable(int state, int action, float reward, int nextState) {
    std::vector<float>& qValues = qTable[state];
    float maxQ = *std::max_element(qTable[nextState].begin(), qTable[nextState].end());
    qValues[action] = qValues[action] + ALPHA * (reward + GAMMA * maxQ - qValues[action]);
}
