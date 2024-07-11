import gymnasium as gym
from gymnasium import spaces
import numpy as np

class MeshRoutingEnv(gym.Env):
    def __init__(self, initial_node, neighbors_dict):
        super(MeshRoutingEnv, self).__init__()
        
        self.initial_node = initial_node
        self.current_node = initial_node
        self.neighbors_dict = neighbors_dict
        
        self.action_space = spaces.Discrete(len(neighbors_dict[initial_node]))  # número de vecinos del nodo inicial
        self.observation_space = spaces.Dict({
            'current_node': spaces.Discrete(len(neighbors_dict)),
            'neighbors': spaces.MultiDiscrete([len(neighbors_dict) for _ in range(len(neighbors_dict[initial_node]))])
        })
        
    def reset(self):
        self.current_node = self.initial_node
        return self._get_observation()
        
    def _get_observation(self):
        neighbors = self.neighbors_dict[self.current_node]
        return {'current_node': self.current_node, 'neighbors': np.array(neighbors)}
    
    def step(self, action):
        neighbors = self.neighbors_dict[self.current_node]
        if action < 0 or action >= len(neighbors):
            raise ValueError(f"Invalid action: {action}")
        
        self.current_node = neighbors[action]
        
        observation = self._get_observation()
        reward = -1  # Penalty for each step
        done = self.current_node == self.goal_node  # Define tu condición de finalización
        info = {}
        
        return observation, reward, done, info
    
    def render(self, mode='human'):
        pass  # Opcional, para visualizar el entorno

# Ejemplo de uso
initial_node = 0
neighbors_dict = {
    0: [1, 2],
    1: [0, 3],
    2: [0, 3],
    3: [1, 2]
}

env = MeshRoutingEnv(initial_node, neighbors_dict)
obs = env.reset()
print(obs)

action = 0  # Por ejemplo, mover al primer vecino
obs, reward, done, info = env.step(action)
print(obs, reward, done, info)
