# pip install matplotlib
# pip install numpy
# pip install seaborn
# pip install tqdm
# pip install gymnasium==0.27.0

from __future__ import annotations

from collections import defaultdict # allows access to keys that do not exist yet

import matplotlib.pyplot as plt # drawing plots
from matplotlib.patches import Patch
import numpy as np # data and array manipulation
import seaborn as sns

from tqdm import tqdm # progress bar
import gymnasium as gym

# Create the Blackjack environment
env = gym.make('Blackjack-v1', sab=True, render_mode="rgb_array")

# Observing the environment

# reset the environment to get the first observation
done = False
observation, info = env.reset()

# our observation is a tuple consisting of 3 values:
# 1. the player's current sum
# 2. values of the dealer's face-up card
# 3. boolean whether the player holds a usable ace 
#    (it is usable if it counts as 11 without busting)
# e.g. observation = (16, 9, False)

# we interact with the environment by doing
# env.step(action)
# this returns the following:
#   - next_state
#   - reward
#   - terminated
#   - truncated
#   - info (extra info about the env or the agent)

# sample a random action from all valid actions
action = env.action_space.sample()

# Take a step in the environment using the sampled action
observation, reward, terminated, truncated, info = env.step(action)

# if terminated = true we should stop the current episode and begin a new one
# if terminated:
#     env.reset()

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
        # Initialize Q-values with a defaultdict that returns arrays of zeros for each action
        self.q_values = defaultdict(lambda: np.zeros(env.action_space.n))
        self.lr = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = initial_epsilon
        self.epsilon_decay = epsilon_decay
        self.final_epsilon = final_epsilon

        # List to keep track of the training error over time
        # This list stores the temporal difference errors, which indicate
        # how much the Q-values are being updated in each step
        # This helps understanding how quickly the agent is learning
        self.training_error = []

    def get_action(self, obs: tuple[int, int, bool]) -> int:
        # Epsilon-greedy action selection
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
        # Update the Q-value of an action using the Q-learning algorithm
        future_q_values = (not terminated) * np.max(self.q_values[next_obs])
        temporal_difference = (
            reward + self.discount_factor * future_q_values - self.q_values[obs][action]
        )

        # Update the Q-value with the temporal difference
        self.q_values[obs][action] += self.lr * temporal_difference

        # Append the temporal difference to the training error list
        self.training_error.append(temporal_difference)

    def decay_epsilon(self):
        # Decay the epsilon value to reduce exploration over time
        self.epsilon = max(self.final_epsilon, self.epsilon - self.epsilon_decay)

# Training loop

# Hyperparameters
learning_rate = 0.01
n_episodes = 100_000
start_epsilon = 1.0
epsilon_decay = start_epsilon / (n_episodes / 2)  # this is to reduce exploration over time
final_epsilon = 0.1

# Instantiate the BlackjackAgent with the specified hyperparameters
agent = BlackjackAgent(
    learning_rate=learning_rate,
    initial_epsilon=start_epsilon,
    epsilon_decay=epsilon_decay,
    final_epsilon=final_epsilon
)

from IPython.display import clear_output

# Wrap the environment to record episode statistics
env = gym.wrappers.RecordEpisodeStatistics(env, deque_size=n_episodes)

plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots()

# Main training loop
for episode in tqdm(range(n_episodes)):
    obs, info = env.reset()
    done = False

    while not done:
        # Choose an action using the agent's policy
        action = agent.get_action(obs)
        
        # Take a step in the environment
        next_obs, reward, terminated, truncated, info = env.step(action)

        # Update the agent with the observed transition
        agent.update(obs, action, reward, terminated, next_obs)

        # Render the environment
        frame = env.render()
        ax.imshow(frame)
        plt.draw()
        plt.pause(0.01)  # Pause to update the plot

        # Check if the episode is done
        done = terminated or truncated
        
        # Update the observation
        obs = next_obs

        # Decay the epsilon value
        agent.decay_epsilon()

plt.ioff()  # Turn off interactive mode

# Plot the training error over episodes
plt.figure()
plt.plot(agent.training_error)
plt.xlabel('Episode')
plt.ylabel('Training Error')
plt.title('Training Error over Episodes')
plt.show()
