# Mesh Routing with Q-Learning

This project implements a mesh routing simulation using Q-Learning. It leverages the `gymnasium` library to create a custom environment for routing packets in a mesh network. The agent uses an epsilon-greedy policy to explore and exploit the network to find the optimal path from an initial node to a goal node.

## Installation

Before running the program, you need to install the required packages. You can install them using pip:
```bash
pip install matplotlib numpy seaborn tqdm gymnasium==0.27.0 pandas networkx frozendict
```
## Project Structure
The project is organized into several Python files:

 - `main.py`: The main script that runs the simulation and training loop.
 - `mesh_routing_env.py`: Contains the custom Gym environment for the mesh routing problem.
 - `mesh_routing_agent.py`: Defines the Q-learning agent.
 - `utils.py`: Utility functions, including a function to print the Q-table.

## Files Description
`main.py`
This is the main script that orchestrates the training of the Q-learning agent in the custom mesh routing environment. It initializes the environment, agent, and runs the training loop.

`mesh_routing_env.py`
This file contains the definition of the MeshRoutingEnv class, which extends the gym.Env class. It defines the environment dynamics including state and action spaces, the reward mechanism, and how the state transitions occur.

`mesh_routing_agent.py`
This file contains the MeshRoutingAgent class, which implements the Q-learning algorithm. It includes methods to select actions (epsilon-greedy policy), update Q-values, and decay the exploration rate.

`utils.py`
This file contains utility functions, such as print_q_table, which prints the Q-table for the agent.

## Usage
### Step 1: Define the Environment
In main.py, we define the initial node, goal node, and the neighbors dictionary that describes the mesh network.

```python
initial_node = 0
goal_node = 5
neighbors_dict = frozendict({
    0: [1, 2],
    1: [0, 3, 4],
    2: [0, 4],
    3: [1, 5],
    4: [1, 2, 5],
    5: [3, 4]  # Master server
})
```
### Step 2: Initialize the Environment and Agent
```python
env = MeshRoutingEnv(initial_node, neighbors_dict, goal_node)

# Hyperparameters
learning_rate = 0.01
n_episodes = 100
start_epsilon = 1.0
epsilon_decay = start_epsilon / (n_episodes / 2)
final_epsilon = 0.1

agent = MeshRoutingAgent(
    neighbors_dict=neighbors_dict,
    learning_rate=learning_rate,
    initial_epsilon=start_epsilon,
    epsilon_decay=epsilon_decay,
    final_epsilon=final_epsilon
)

env = gym.wrappers.RecordEpisodeStatistics(env, deque_size=n_episodes)
```

### Step 3: Run the Training Loop

```python
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

for episode in tqdm(range(n_episodes)):
    print_q_table(agent, neighbors_dict)
    obs, info = env.reset()
    done = False

    while not done:
        action_space_size = env.action_space.n  # Get the current action space size
        action_index = agent.get_action(obs, action_space_size)
        next_action, reward, terminated, truncated, info = env.step(action_index)
        agent.update(obs, action_index, reward, terminated, next_action)

        # Render the environment
        env.render()
        plt.pause(0.01)  # Pause to update the plot
        print("")
        print("Actual node is: ", next_action)
        print("Node's neighbors are: ", neighbors_dict[next_action])

        done = terminated or truncated
        obs = next_action
        agent.decay_epsilon()

rolling_length = 1
fig, axs = plt.subplots(ncols=3, figsize=(12, 5))

# Episode Rewards
reward_moving_average = (
    np.convolve(
        np.array(env.length_queue).flatten(), np.ones(rolling_length), mode="valid"
    ) / rolling_length
)

axs[0].set_title("Episode Rewards")
axs[0].plot(range(len(reward_moving_average)), reward_moving_average)

# Episode Lengths
length_moving_average = (
    np.convolve(
        np.array(env.length_queue).flatten(), np.ones(rolling_length), mode="valid"
    ) / rolling_length
)
axs[1].set_title("Episode Lengths")
axs[1].plot(range(len(length_moving_average)), length_moving_average)

# Training Error
training_error_moving_average = (
    np.convolve(agent.training_error, np.ones(rolling_length), mode="valid") / rolling_length
)
axs[2].set_title("Training Error")
axs[2].plot(range(len(training_error_moving_average)), training_error_moving_average)

plt.tight_layout()
plt.show()
```

### Explanation
#### MeshRoutingEnv
 - Initialization: The environment is initialized with the initial node, neighbors dictionary, and the goal node.
 - reset: Resets the environment to the initial state.
 - _get_observation: Returns the current observation.
 - step: Executes one step in the environment by taking an action.

### MeshRoutingAgent
 - render: Visualizes the mesh network and the current routing path.
 - Initialization: Initializes the Q-table, learning rate, epsilon values, and discount factor.
 - get_action: Selects an action based on the epsilon-greedy policy.
 - update: Updates the Q-values based on the received reward and the next state.
 - decay_epsilon: Decays the exploration rate.

### Visualizations
The script includes code to visualize:

 - The Q-table at each episode.
 - The routing path on the mesh network.
 - The episode rewards, lengths, and training errors.
