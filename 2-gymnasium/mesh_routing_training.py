# Required imports and package installations
# pip install matplotlib
# pip install numpy
# pip install seaborn
# pip install tqdm
# pip install gymnasium==0.27.0
# pip install pandas
# pip install networkx
# pip install frozendict

from __future__ import annotations
from collections import defaultdict, deque
# from copy import deepcopy
from frozendict import frozendict

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import seaborn as sns
import time

from tqdm import tqdm
import gymnasium as gym

def print_q_table(agent, neighbors_dict):
    q_table = {}
    for state in range(len(neighbors_dict)):
        q_table[state] = agent.q_values[state]

    df = pd.DataFrame(q_table)
    print()
    print(df.T)

# Definir el entorno de ruteo de paquetes
class MeshRoutingEnv(gym.Env):
    def __init__(self, initial_node, neighbors_dict, goal_node):
        super(MeshRoutingEnv, self).__init__()
        
        self.initial_node = initial_node
        self.current_node = initial_node
        self.neighbors_dict = neighbors_dict
        self.goal_node = goal_node
        
        self.observation_space = gym.spaces.Discrete(len(neighbors_dict))
        self.action_space = gym.spaces.Discrete(len(neighbors_dict))  # Será actualizado dinámicamente

        self.route = [self.current_node]  # Para almacenar el ruteo actual

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.current_node = self.initial_node
        self.route = [self.current_node]
        return self._get_observation(), {}
        
    def _get_observation(self):
        self.action_space = gym.spaces.Discrete(len(self.neighbors_dict[self.current_node]))
        return self.current_node
    
    def step(self, action):
        # neighbors = self.neighbors_dict[self.current_node]
        neighbors = neighbors_dict[obs]
        # if action < 0 or action >= len(neighbors):
        #     raise ValueError(f"Invalid action: {action}")
        #
        # print("DEBUG action: ", action)
        # print("DEBUG neighbors: ", neighbors)
        # print("DEBUG neighbors[action]: ", neighbors[action])
        self.current_node = neighbors[action]
        self.route.append(self.current_node)
        
        observation = self._get_observation()
        reward = -1 if self.current_node != self.goal_node else 100
        terminated = self.current_node == self.goal_node or len(self.neighbors_dict[self.current_node]) == 0
        truncated = False  # No hay condición de truncado en este entorno
        info = {}
        
        # if terminated and self.current_node != self.goal_node:
        #     reward = -100  # Penalización fuerte por terminar en un nodo que no es el objetivo
        
        return observation, reward, terminated, truncated, info
    
    def render(self, mode='human'):
        G = nx.Graph()

        for node, neighbors in self.neighbors_dict.items():
            for neighbor in neighbors:
                G.add_edge(node, neighbor)

        pos = nx.spring_layout(G)
        plt.clf()  # Clear the current figure

        nx.draw(G, pos, with_labels=True, node_size=500, node_color="skyblue", font_weight='bold', edge_color='gray')

        route_edges = [(self.route[i], self.route[i+1]) for i in range(len(self.route)-1)]
        nx.draw_networkx_edges(G, pos, edgelist=route_edges, width=2, edge_color='r')

        if len(self.route) > 1:
            start_node = self.route[-2]
            end_node = self.route[-1]
            start_pos = pos[start_node]
            end_pos = pos[end_node]

            plt.annotate(
                '', 
                xy=end_pos, 
                xytext=start_pos,
                arrowprops=dict(facecolor='blue', shrink=0.05)
            )

        plt.title("Network Routing Visualization")
        plt.pause(0.01)  # Pause to update the plot

# Ejemplo de uso
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

# env = MeshRoutingEnv(initial_node, deepcopy(neighbors_dict), goal_node)
env = MeshRoutingEnv(initial_node, neighbors_dict, goal_node)

# epsilon-greedy implementation for the new environment
class MeshRoutingAgent:
    def __init__(
        self,
        learning_rate: float,
        initial_epsilon: float,
        epsilon_decay: float,
        final_epsilon: float,
        discount_factor: float = 0.95
    ):
        self.q_values = defaultdict(lambda: np.zeros(len(neighbors_dict)))
        self.lr = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = initial_epsilon
        self.epsilon_decay = epsilon_decay
        self.final_epsilon = final_epsilon
        self.training_error = []

    def get_action(self, obs, action_space_size):
        if np.random.random() < self.epsilon:
            return np.random.choice(action_space_size)  # Explore: choose a random action
        else:
            return int(np.argmax(self.q_values[obs][:action_space_size]))  # Exploit: choose the best action based on Q-values

    def update(self, obs, action, reward, terminated, next_obs):
        future_q_values = (not terminated) * np.max(self.q_values[next_obs])
        temporal_difference = (
            reward + self.discount_factor * future_q_values - self.q_values[obs][action]
        )
        self.q_values[obs][action] += self.lr * temporal_difference
        self.training_error.append(temporal_difference)

    def decay_epsilon(self):
        self.epsilon = max(self.final_epsilon, self.epsilon - self.epsilon_decay)

# Training loop

# Hyperparameters
learning_rate = 0.01
# n_episodes = 10
n_episodes = 100
start_epsilon = 1.0
epsilon_decay = start_epsilon / (n_episodes / 2)
final_epsilon = 0.1

agent = MeshRoutingAgent(
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
        # time.sleep(3)
        print("")
        print("actual node is: ", next_action)
        print("node's neighbors are: ", neighbors_dict[next_action])

        done = terminated or truncated
        obs = next_action
        agent.decay_epsilon()

rolling_length = 1  # Tamaño de la ventana para la media móvil

fig, axs = plt.subplots(ncols=3, figsize=(12, 5))

# Calcular medias móviles para las métricas de rendimiento

# Episode Rewards
reward_moving_average = (
    np.convolve(
        np.array(env.return_queue).flatten(), np.ones(rolling_length), mode="valid"
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
