# Mesh Routing with Q-Learning

This project implements a mesh network routing system using Q-Learning. The goal is to solve a routing problem in a mesh network using reinforcement learning, implemented with ESP devices. The main objective is to optimize the route from a `sender node`, streaming sensor data obtained with a DHT11 to a `master node` (which is the node that's connected to a server, which is connected to a router), minimizing the number of hops required to reach it.

![infrastructure (1)](https://github.com/user-attachments/assets/3d4fac86-66f3-4d0a-aa06-6f3a00470c4f)

Each ESP device will be flashed with software to handle packet reception and transmission in the mesh network, and to implement the Q-Learning algorithm. Each time an ESP receives a packet, it will update the Q-table, which has states defined by the current node and the set of neighboring nodes (obtained through the painlessMesh library).

The actions in the Q-table will consist of sending the packet to one of the neighboring nodes. At each hop, the Q-table will be updated using an ε-greedy algorithm. The reward for the algorithm will be -1 for each hop that does not reach the master node and +100 when the node is reached. This way, we aim to optimize the use of the mesh network by minimizing the number of hops required to reach the central server.

## How does the learning work

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
   - Each time a hop is forwarded, the Q-table is updated using the bellman equation.

4. **Reward System**: 
   - For every hop that does not reach the master node, the node receives a reward of -1.
   - When the hop reaches the master node, the node receives a reward of +100.

5. **Finding and Broadcasting of Optimal Q-Parameters**:
   - When the hop reaches the master node, the middleware reads the episode results and sends the current Q parameters (alpha, gamma, epsilon) to a genetic algorithm, in order to find the optimal parameters.
   - Then, the middleware sends the Q parameters back to the master node, which then broadcasts the updated Q-table and the Q parameters to all nodes in the network.
   - This is done for a number of episodes, to separate the learning phase from the exploitation phase.
   - This ensures all nodes have the latest learning results, allowing them to make informed decisions on the best hop.

![sender-intermediate-master](https://github.com/user-attachments/assets/af65e433-a2b9-45f8-bbc7-f9211c1d41ca)

6. **Middleware and Server**:
   - A middleware script running on a PC reads the serial monitor output from the master node, tries to find the optimal Q-Parameters with a genetic algorithm and sends relevant learning results to a server.
   - The server receives learning data, logs it, and visualizes it in a web interface for analysis.

![master-middleware-server](https://github.com/user-attachments/assets/2738826d-d479-40a9-b36d-fa9a73e2d3a7)

This setup ensures that all nodes in the network have the latest learning results, allowing them to make informed decisions on the best hop to optimize packet routing.

## Message structure

Messages sent across nodes have the following structure:

```json
{
    "payload": {
       "tem": 28.1,
       "hum": 70.2
    },
    "current_node_id": "434960473",
    "q_parameters": {
        "alpha": "0.1",
        "gamma": "0.9",
        "epsilon": "0.1",
        "epsilon_decay": "0.1"
    },
    "current_episode": 1,
    "accumulated_reward": 0,
    "total_time": 0,
    "episodes": [
        {
            "episode_number": 1,
            "reward": "0100",
            "time": 0,
            "steps": [
                {
                    "hop": 0,
                    "node_from": "434939008"
                    "node_to": "434960473"
                }
            ]
        }
    ],
    "q_table": {
        "434939008": {
            "434960473": 10
        },
        "434960473": {
            "434939008": 0
        }
    }
}
```

Where `434939008` and `434960473` are the nodes in the network, and the actions are to hop to node `434939008` and `434960473` respectively.

## Setup

2 ESP8266 devices are required for the mesh network:
 - One `sender node`.
 - And a `master node`.

Also, incorporate one or more than one `intermediate nodes` to the network as needed. The more intermediate nodes you have, the more useful the learning results will be.

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

4. Connect to the AP lifted by the master node with the following credentials: `STATION_SSID: whateverYouLike`, `STATION_PASSWORD: somethingSneaky`.

5. Plug the sender node and the intermediate nodes for the learning to start.

6. Enter the learning visualization server to analyze results: `http://localhost:5000`

Demo 1:
[![Watch the video](https://raw.githubusercontent.com/FrancoBre/q-mesh-routing/master/assets/thumbnail.jpeg)](https://youtu.be/WYOyJp7k9bQ)

Demo 2:
[![Watch the video](https://github.com/user-attachments/assets/88dddd2a-fdf8-4cbd-a961-f1f22c596134)](https://youtu.be/fHp0AZggZRo)

## Next Steps

Test with more nodes to tweak learning parameters, or create a simulation with more nodes to do so.

Create a better demo.
