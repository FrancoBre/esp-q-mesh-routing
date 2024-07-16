# Mesh Routing with Q-Learning

This project implements a mesh routing simulation using Q-Learning. The goal is to solve a routing problem in a mesh network using reinforcement learning, implemented with ESP devices. The main objective is to optimize the route to a master node (which is the node that's connected to the router), minimizing the number of hops required to reach it.

![9ecb6138-5f85-4ea1-809f-09d864d480a9](https://github.com/user-attachments/assets/b4e368f2-b956-4bad-a0d7-8ac717b5eb83)

Each ESP device will be flashed with software to handle packet reception and transmission in the mesh network, and to implement the Q-Learning algorithm. Each time an ESP receives a packet, it will update the Q-table, which has states defined by the current node and the set of neighboring nodes (obtained through the ESP-MESH library).

The actions in the Q-table will consist of sending the packet to one of the neighboring nodes. At each hop, the Q-table will be updated using an ε-greedy algorithm. The reward for the algorithm will be -1 for each hop that does not reach the master node and +100 when the node is reached. This way, we aim to optimize the use of the mesh network by minimizing the number of hops required to reach the central server.

![8a0 (1)](https://github.com/user-attachments/assets/06467555-cd01-4b8c-bec0-57996327c314)


## Project Structure

The project is organized into sections, each demonstrating different incremental approaches to implementing the Q-Learning logic and packet routing simulation.

### 1. Vanilla Python
This script implements a basic network routing simulation using Q-Learning in Python. It does not rely on any external libraries and serves as an introductory implementation of the Q-Learning algorithm applied to packet routing.

To run:
```bash
$ python3 q_learning_mesh.py
```

### 2. Gymnasium
This program uses the gymnasium library (a derivative of OpenAI Gym) to create a more robust and structured simulation environment. The scripts define a custom environment for the routing problem and an agent that uses Q-Learning to learn the optimal path.

- main.py: The main script that runs the simulation and training loop.
- mesh_routing_agent.py: Defines the agent that uses Q-Learning.
- mesh_routing_env.py: Contains the definition of the custom Gym environment.
- utils.py: Utility functions, including one to print the Q-table.

Make sure to install required dependencies:
```bash
$ pip install matplotlib
$ pip install numpy
$ pip install seaborn
$ pip install tqdm
$ pip install gymnasium==0.27.0
$ pip install pandas
$ pip install networkx
```

And run with:
```bash
$ python3 main.py
```

![2024-07-1120-58-55online-video-cutter com1-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/8c051953-d5f7-49ff-8958-199edd62eedd)


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

To run, flash on an ESP8266 using arduino IDE or sm.

## Next Steps
 
 1. *Create a Mesh Network*: Implement a mesh network using ESP8266.
 2. *Incorporate Q-Learning*: Integrate Q-Learning logic into the mesh network to optimize packet routing.
 3. *Integrate with gymnasium*: Make somehow gymnasium intercept the functioning of the actual mesh network to provide metrics and allow to tweak parameters based on results.
 
The aim is to move towards practical testing on real hardware to demonstrate the applicability of these POCs in real-world scenarios.
