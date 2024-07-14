# Mesh Routing with Q-Learning

This project implements a mesh routing simulation using Q-Learning. The goal is to find the optimal path to route a packet from an initial node to a master server in a mesh network.

## Project Structure

The project is organized into sections, each demonstrating different incremental approaches to implementing the Q-Learning logic and packet routing simulation.

### 1. Vanilla Python
This script implements a basic network routing simulation using Q-Learning in Python. It does not rely on any external libraries and serves as an introductory implementation of the Q-Learning algorithm applied to packet routing.

### 2. Gymnasium
This program uses the gymnasium library (a derivative of OpenAI Gym) to create a more robust and structured simulation environment. The scripts define a custom environment for the routing problem and an agent that uses Q-Learning to learn the optimal path.

- main.py: The main script that runs the simulation and training loop.
- mesh_routing_agent.py: Defines the agent that uses Q-Learning.
- mesh_routing_env.py: Contains the definition of the custom Gym environment.
- utils.py: Utility functions, including one to print the Q-table.

For more details, you can consult the specific README within this folder.

### 3. C++ POC (Proof of Concept)
This program features a standalone (that is, that can run on a PC instead of an ESP device, like >4 sections) proof of concept in C++ for implementing the Q-Learning algorithm. It simulates 100 episodes in which the learning occurs. 

To run:
```bash
$ g++ q_learning_poc.cpp -o q_learning_poc
$ ./q_learning_poc
```

### 4. Arduino POC (Proof of Concept)
This Arduino file (.ino) is a port of the C++ POC to be run in an ESP8266. It features a single episode, until finding the master server, in which the device enters deep sleep and awaits for restart to simulate a new episode.

To run, flash on an ESP8266.

## Next Steps
 
 1. *Create a Mesh Network*: Implement a mesh network using ESP8266.
 2. *Incorporate Q-Learning*: Integrate Q-Learning logic into the mesh network to optimize packet routing.
 
The aim is to move towards practical testing on real hardware to demonstrate the applicability of these POCs in real-world scenarios.
