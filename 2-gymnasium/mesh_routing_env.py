import gym
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np

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
        neighbors = self.neighbors_dict[self.current_node]
        self.current_node = neighbors[action]
        self.route.append(self.current_node)
        
        observation = self._get_observation()
        reward = -1 if self.current_node != self.goal_node else 100
        terminated = self.current_node == self.goal_node or len(self.neighbors_dict[self.current_node]) == 0
        truncated = False
        info = {}
        
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
