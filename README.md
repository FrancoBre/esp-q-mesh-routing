# Mesh Routing with Q-Learning

This project implements a mesh network routing system using Q-Learning. The goal is to solve a routing problem in a mesh network using reinforcement learning, implemented with ESP devices. The main objective is to optimize the route from a `sender node`, streaming sensor data obtained with a DHT11 to a `master node` (which is the node that's connected to a server, which is connected to a router), minimizing the number of hops required to reach it.

![infrastructure (1)](https://github.com/user-attachments/assets/3d4fac86-66f3-4d0a-aa06-6f3a00470c4f)

Each ESP device will be flashed with software to handle packet reception and transmission in the mesh network, and to implement the Q-Learning algorithm. Each time an ESP receives a packet, it will update the Q-table, which has states defined by the current node and the set of neighboring nodes (obtained through the ESP-MESH library).

The actions in the Q-table will consist of sending the packet to one of the neighboring nodes. At each hop, the Q-table will be updated using an ε-greedy algorithm. The reward for the algorithm will be -1 for each hop that does not reach the master node and +100 when the node is reached. This way, we aim to optimize the use of the mesh network by minimizing the number of hops required to reach the central server.

## How does the learning works

Here’s a step-by-step explanation of how the learning process works:

1. **Initialization**: 
   - Each node starts with an empty Q-table. The Q-table maps states (neighboring nodes) to actions (hops to neighboring nodes) with corresponding Q-values.
   - The Q-learning parameters include the learning rate (alpha), discount factor (gamma), and exploration rate (epsilon).

2. **Receiving a Hop**: 
   - When a node receives a hop, it uses its Q-table to decide the next hop.
   - The decision is based on two strategies:
     - **Exploit (1 - epsilon)**: Choose the neighboring node with the highest Q-value.
     - **Explore (epsilon)**: Randomly choose a neighboring node to explore new paths.

3. **Updating the Q-table**: 
   - Each time a hop is forwarded, the Q-table is updated.
   - The Q-value is updated using the formula: 
     \[
     Q(s, a) = Q(s, a) + \alpha \left( r + \gamma \max_a Q(s', a') - Q(s, a) \right)
     \]
     where \( s \) is the current state, \( a \) is the action taken, \( r \) is the reward, \( s' \) is the next state, and \( a' \) is the next action.

4. **Reward System**: 
   - For every hop that does not reach the master node, the node receives a reward of -1.
   - When the hop reaches the master node, the node receives a reward of +100.

5. **Broadcasting the Q-Table**:
   - When the hop reaches the master node, the master node broadcasts the updated Q-table to all nodes in the network.
   - This ensures all nodes have the latest learning results, allowing them to make informed decisions on the best hop.

![sender-intermediate-master](https://github.com/user-attachments/assets/af65e433-a2b9-45f8-bbc7-f9211c1d41ca)

6. **Middleware and Server**:
   - A middleware script running on a PC reads the serial monitor output from the master node and sends relevant learning results to a server.
   - The server receives learning data, logs it, and visualizes it in a web interface for analysis.

![master-middleware-server](https://github.com/user-attachments/assets/2738826d-d479-40a9-b36d-fa9a73e2d3a7)

This setup ensures that all nodes in the network have the latest learning results, allowing them to make informed decisions on the best hop to optimize packet routing.

## Setup

\>1 ESP8266 devices are required for the mesh network:
 - One `sender node`.
 - One or more than one `intermediate nodes`.
 - And a `master node`.

Flash `sender-node.ino`, `intermediate-node.ino` and `master-node.ino` respectively using Arduino IDE. Also, required dependencies are `painlessMesh`, `TaskScheduler`, `ArduinoJson` and `AsyncTCP`.
There are plenty of tutorials online on how to program an ESP8266, but if you are too lazy to search, [here](https://www.youtube.com/watch?v=lQm3YKkXPNc)'s one.

Once you have flashed the nodes, do the following:

1. Run the server in which the learning data is received to be visualized and analyzed.
```bash
$ cd learning-visualization-server
$ python3 server.py
```
This will run a server in `localhost:5000`. Required pip dependencies are `Flask` `plotly` and `pandas`.

2. Run the middleware that grabs the results of the learning process.
```bash
$ cd ..
$ cd learning-results-grabbing
$ python3 middleware.py
```

3. Plug the master node (the serial port device in the code is assumed to be `/dev/ttyUSB0`, change as needed).

4. Plug the sender node and the intermediate nodes for the learning to start.

Demo:
https://github.com/user-attachments/assets/84d4e89f-cd16-4f19-8628-ede290551f67

## Next Steps

Test with more nodes to tweak learning parameters, or create a simulation with more nodes to do so.
