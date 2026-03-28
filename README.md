# Mesh Routing with Q-Routing (TD(0))

This project implements a mesh network routing system using **Q-Routing** as described in Boyan & Littman (1994), *Packet Routing in Dynamically Changing Networks: A Reinforcement Learning Approach*. The algorithm uses **TD(0) temporal-difference updates** to minimize packet delivery time. Implemented with ESP devices and the painlessMesh library.

Each ESP device handles packet reception and transmission in the mesh network. The routing logic minimizes **estimated delivery time**. Q-values represent the estimated time to deliver a packet to the destination via each neighbor—**lower Q means a better path**.

## Project layout

| Path | Contents |
|------|----------|
| `firmware/` | ESP8266 sketches—one folder per node (folder name = main `.ino` basename for Arduino CLI) |
| `tools/` | Host-side Python: serial middleware and visualization server |
| `docs/` | Additional documentation |

## Node Roles

| Role | File | Behavior |
|------|------|----------|
| **Sender** | `firmware/sender-node/sender-node.ino` | Sends packets on a timer, chooses first hop, forwards packets |
| **Intermediate** | `firmware/intermediate-node/intermediate-node.ino` | Forwards packets, updates Q-table at each hop |
| **Receiver** | `firmware/receiver-node/receiver-node.ino` | Packet destination; receives and logs delivery |

```mermaid
sequenceDiagram
    participant Sender
    participant Intermediate
    participant Receiver
    participant Middleware
    participant Server

    Note over Sender: Choose neighbor (greedy, min Q)
    Sender->>Sender: Build PACKET_HOP with q_table, send_timestamp
    Sender->>Intermediate: Send data + Q-learning data

    Note over Intermediate: Compute step_time from send_timestamp
    Note over Intermediate: Update Q(node_from, us): ΔQ = η((q+s+t)−Q)
    Note over Intermediate: Choose next hop (greedy, min Q)
    Intermediate->>Receiver: Forward PACKET_HOP

    Note over Receiver: Packet delivered
    Receiver->>Receiver: Serial: DELIVERY_DATA: + JSON
    Receiver->>Middleware: USB Serial (receiver connected to PC)
    Middleware->>Middleware: Extract JSON, parse
    Middleware->>Server: POST /data (JSON)
    Note over Server: Store, serve topology, hop count, delivery data
    Note over Server: http://localhost:5000

    Note over Sender: Timeout → send next packet
```

**Forward-only flow:** No backward propagation. Q-updates happen at each hop when forwarding. Receiver does not broadcast; sender sends packets on a timer.

**Visualization (optional):** When the receiver is connected via USB to a PC, it outputs `DELIVERY_DATA:` + JSON on Serial. The middleware reads this, extracts the JSON, and forwards it to the visualization server. Open http://localhost:5000 for topology, hop count per delivered packet, and delivery data.

## How the Learning Works

### Q-Values

- **Q(from, to)** = estimated time to deliver a packet from the current node to the destination via neighbor `to`
- **Lower Q** = faster path = better choice

### Update Rule (Boyan & Littman 1994)

When a packet arrives from node `x` and we forward it, we update the link that was traversed:

```
ΔQ_x(d,y) = η × ((q + s + t) - Q_x(d,y))
```

Where:
- **q** = time in queue (0). AsyncTCP exposes only send-buffer info (`space()`, `canSend()`), not receive queue size or per-packet queue delay. painlessMesh does not expose the underlying TCP connections. We cannot measure queue time without an application-level queue.
- **s** = transmission time (measured via `send_timestamp` in the packet)
- **t** = our estimate of remaining time = min over our neighbors of Q(us, neighbor)
- **η** = learning rate (eta, default 0.7)

### Action Selection

- **Greedy:** Always choose the neighbor with the **minimum** Q-value

### Flow

1. **Sender** sends a packet, picks first hop, sends `PACKET_HOP` with `send_timestamp`
2. **Intermediate** receives, computes step_time from `send_timestamp`, updates Q(node_from, us), picks next hop, forwards
3. **Receiver** receives; packet is delivered (no further action)
4. **Sender** sends the next packet after a timeout (no callback)

## Message Structure

Only **PACKET_HOP** messages are used. Structure:

```json
{
    "type": "PACKET_HOP",
    "current_node_id": "434960473",
    "send_timestamp": 12345678,
    "hyperparameters": {
        "eta": 0.7
    },
    "current_episode": 1,
    "episodes": [
        {
            "episode_number": 1,
            "steps": [
                {
                    "hop": 0,
                    "node_from": "434939008",
                    "node_to": "434960473"
                }
            ]
        }
    ],
    "q_table": {
        "434939008": { "434960473": 0.5 },
        "434960473": { "434939008": 0.3 }
    }
}
```

- **send_timestamp** (µs): Used by the receiver to compute actual transmission time
- **q_table**: Q(from, to) = estimated delivery time; lower = better
- **episodes** (legacy naming): Tracks delivered packets; each entry = one path (steps) from sender to receiver

## Setup

### Hardware

- 2+ ESP8266 devices
  - `firmware/sender-node/` → sender
  - `firmware/intermediate-node/` → intermediate
  - `firmware/receiver-node/` → receiver
- Micro-USB data cable
- Linux machine with `arduino-cli`

### Arduino CLI setup on Linux

Install `arduino-cli` and configure the ESP8266 core:

```bash
$ arduino-cli config init
```

Edit `~/.arduino15/arduino-cli.yaml` and add:

```yaml
board_manager:
  additional_urls:
    - https://arduino.esp8266.com/stable/package_esp8266com_index.json
```

Then install the ESP8266 core:

```bash
$ arduino-cli core update-index
$ arduino-cli core install esp8266:esp8266
```

Install required libraries:

```bash
$ arduino-cli lib update-index
$ arduino-cli lib install "Painless Mesh"
$ arduino-cli lib install TaskScheduler
$ arduino-cli lib install ESPAsyncTCP
$ arduino-cli lib install ArduinoJson
```

## Detect the board

Connect the ESP8266 through USB and verify that the OS detects it:

```bash
$ lsusb
$ arduino-cli board list
```

A NodeMCU board typically appears as something like:

```bash
$ /dev/ttyUSB0 serial Serial Port (USB)
```

## Compile

For NodeMCU ESP8266, use:

```bash
$ arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 ./firmware/sender-node
$ arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 ./firmware/intermediate-node
$ arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 ./firmware/receiver-node
```

## Upload

Upload each sketch to the corresponding board:

```bash
$ arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 ./firmware/sender-node
$ arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 ./firmware/intermediate-node
$ arduino-cli upload -p /dev/ttyUSB0 --fqbn esp8266:esp8266:nodemcuv2 ./firmware/receiver-node
```

## Serial monitoring

To inspect logs from a node, use arduino-cli monitor.

Example for baudrate 9600:

```bash
$ arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=9600
```
## Network bring-up order

1. Power on the receiver
2. Power on any intermediate nodes
3. Power on the sender

The sender transmits packets periodically, intermediate nodes forward them and update Q-values, and the receiver logs deliveries.

### Visualization (optional)

Connect the **receiver** via USB to a PC to visualize learning progress:

```bash
# Terminal 1: Start visualization server
cd tools/learning-visualization-server && pip install -r requirements.txt && python server.py

# Terminal 2: Start middleware (receiver connected via USB)
cd tools/learning-results-grabbing && pip install -r requirements.txt && python middleware.py --verbose
```

Open http://localhost:5000 for topology, hop count per delivered packet, and delivery data.

<img width="1693" height="1782" alt="image" src="https://github.com/user-attachments/assets/d002593c-ff9e-46f1-88b0-68cf9f6cd054" />


## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `eta` | 0.7 | Learning rate |
| `INITIAL_Q` | 0.0 | Initial Q-value (lower = better) |

## References

- Boyan, J. A., & Littman, M. L. (1994). *Packet Routing in Dynamically Changing Networks: A Reinforcement Learning Approach*
- [meshroutingframework](https://github.com/yourusername/meshroutingframework) — Java simulation with the same Q-routing logic

## Demos

Demo 1:
[![Watch the video](https://raw.githubusercontent.com/FrancoBre/q-mesh-routing/master/assets/thumbnail.jpeg)](https://youtu.be/WYOyJp7k9bQ)

Demo 2:
[![Watch the video](https://github.com/user-attachments/assets/88dddd2a-fdf8-4cbd-a961-f1f22c596134)](https://youtu.be/fHp0AZggZRo)
