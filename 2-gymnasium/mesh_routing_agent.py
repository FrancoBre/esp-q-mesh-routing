from collections import defaultdict
import numpy as np

class MeshRoutingAgent:
    def __init__(
        self,
        neighbors_dict: dict,
        learning_rate: float,
        initial_epsilon: float,
        epsilon_decay: float,
        final_epsilon: float,
        discount_factor: float = 0.95
    ):
        self.neighbors_dict = neighbors_dict,
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
