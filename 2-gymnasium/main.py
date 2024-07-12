from mesh_routing_agent import MeshRoutingAgent
from mesh_routing_env import MeshRoutingEnv
from utils import print_q_table
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from tqdm import tqdm
import gymnasium as gym

# Topologia de ejemplo
initial_node = 0
goal_node = 5
neighbors_dict = {
    0: [1, 2],
    1: [0, 3, 4],
    2: [0, 4],
    3: [1, 5],
    4: [1, 2, 5],
    5: [3, 4]  # Master server
}

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

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

# Main training loop
for episode in tqdm(range(n_episodes)):
    print_q_table(agent, neighbors_dict)
    obs, info = env.reset()
    done = False

    while not done:
        action_space_size = env.action_space.n  # Obtener el tamaño actual del espacio de acción
        action_index = agent.get_action(obs, action_space_size)
        next_action, reward, terminated, truncated, info = env.step(action_index)
        agent.update(obs, action_index, reward, terminated, next_action)

        # Render the environment
        env.render()
        plt.pause(0.01)  # Pause to update the plot
        print("")
        print("actual node is: ", next_action)
        print("node's neighbors are: ", neighbors_dict[next_action])

        done = terminated or truncated
        obs = next_action
        agent.decay_epsilon()

# Plotting results at the end of training
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
