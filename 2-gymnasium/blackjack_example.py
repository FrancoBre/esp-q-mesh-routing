# Required imports and package installations
# pip install matplotlib
# pip install numpy
# pip install seaborn
# pip install tqdm
# pip install gymnasium==0.27.0
# pip install pandas

from __future__ import annotations
from collections import defaultdict, deque

import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np
import pandas as pd
import seaborn as sns

from tqdm import tqdm
import gymnasium as gym

# Create the Blackjack environment
env = gym.make('Blackjack-v1', sab=True, render_mode="rgb_array")

# epsilon-greedy implementation for blackjack
class BlackjackAgent:
    def __init__(
        self,
        learning_rate: float,
        initial_epsilon: float,
        epsilon_decay: float,
        final_epsilon: float,
        discount_factor: float = 0.95
    ):
        self.q_values = defaultdict(lambda: np.zeros(env.action_space.n))
        self.lr = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = initial_epsilon
        self.epsilon_decay = epsilon_decay
        self.final_epsilon = final_epsilon
        self.training_error = []

    def get_action(self, obs: tuple[int, int, bool]) -> int:
        if np.random.random() > self.epsilon:
            return env.action_space.sample()  # Explore: choose a random action
        else:
            return int(np.argmax(self.q_values[obs]))  # Exploit: choose the best action based on Q-values

    def update(
        self,
        obs: tuple[int, int, bool],
        action: int,
        reward: float,
        terminated: bool,
        next_obs: tuple[int, int, bool]
    ):
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
n_episodes = 100000  # Number of episodes
start_epsilon = 1.0
epsilon_decay = start_epsilon / (n_episodes / 2)
final_epsilon = 0.1

agent = BlackjackAgent(
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
    obs, info = env.reset()
    done = False

    while not done:
        action = agent.get_action(obs)
        next_obs, reward, terminated, truncated, info = env.step(action)
        agent.update(obs, action, reward, terminated, next_obs)

        # Render the environment
        frame = env.render()
        ax.imshow(frame)
        plt.draw()
        plt.pause(0.01)  # Pause to update the plot

        done = terminated or truncated
        obs = next_obs
        agent.decay_epsilon()

rolling_length = 500
fig, axs = plt.subplots(ncols=3, figsize=(12, 5))

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
