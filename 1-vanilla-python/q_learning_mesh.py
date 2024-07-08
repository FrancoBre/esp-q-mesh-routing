import random
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

# Parámetros del algoritmo
alpha = 0.1  # Tasa de aprendizaje
gamma = 1  # Factor de descuento
epsilon = 0.1  # Parámetro de exploración-explotación
num_nodes = 6  # Número de nodos en la red mesh (incluyendo el servidor central)
server_node = 5  # Índice del servidor central

# Inicializar la tabla Q, suponiendo que el agente 
# conoce todos los estados al principio del episodio
Q = np.zeros((num_nodes, num_nodes - 1))

# Definir las conexiones de la red mesh
neighbors = {
    0: [1, 2],
    1: [0, 3, 4],
    2: [0, 4],
    3: [1, 5],
    4: [1, 2, 5],
    5: []  # El servidor no tiene vecinos
}

# Función para obtener el índice de una acción (vecino)
def get_action_index(state, action):
    return neighbors[state].index(action)

# Función para obtener la recompensa
def get_reward(current_node, next_node):
    if next_node == server_node:
        return 100  # Recompensa por alcanzar el servidor
    else:
        return -1  # Penalización por cada hop

# Función para elegir una acción (vecino)
def choose_action(state):
    if random.uniform(0, 1) < epsilon:
        # Exploración: elegir un vecino aleatorio
        return random.choice(neighbors[state])
    else:
        # Explotación: elegir la mejor acción conocida
        action_index = np.argmax([Q[state, get_action_index(state, a)] for a in neighbors[state]])
        return neighbors[state][action_index]

# Simulación del proceso de aprendizaje
for episode in range(10000):  # Número de episodios de entrenamiento
    current_node = 0  # Empezar desde el nodo 0
    while current_node != server_node:
        state = current_node
        action = choose_action(state)
        next_state = action
        reward = get_reward(current_node, next_state)
        action_index = get_action_index(state, action)
        Q[state, action_index] = Q[state, action_index] + alpha * (reward + gamma * np.max([Q[next_state, get_action_index(next_state, a)] for a in neighbors[next_state]] if neighbors[next_state] else [0]) - Q[state, action_index])
        current_node = next_state

# Imprimir la tabla Q aprendida
print("Tabla Q aprendida:")
for state in range(num_nodes):
    print(f"Estado {state}: {Q[state]}")

# Función para encontrar la ruta óptima desde el nodo 0 al servidor central
def find_optimal_path():
    current_node = 0
    path = [current_node]
    while current_node != server_node:
        action = choose_action(current_node)
        current_node = action
        path.append(current_node)
    return path

# Imprimir la ruta óptima
optimal_path = find_optimal_path()
print("Ruta óptima desde el nodo 0 al servidor central:")
print(optimal_path)

# Visualización de la red mesh y la tabla Q
def visualize_network(Q, optimal_path):
    G = nx.Graph()

    # Añadir nodos y bordes al grafo
    for node, neighbors_list in neighbors.items():
        for neighbor in neighbors_list:
            G.add_edge(node, neighbor, weight=round(Q[node, get_action_index(node, neighbor)], 2))

    pos = nx.spring_layout(G)  # Posición de los nodos
    edge_labels = {(u, v): d['weight'] for u, v, d in G.edges(data=True)}

    # Dibujar el grafo
    plt.figure(figsize=(10, 8))
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=12)

    # Dibujar el servidor central con otro color y forma
    nx.draw_networkx_nodes(G, pos, nodelist=[server_node], node_color='red', node_size=700, node_shape='s')

    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color='red')

    # Resaltar la ruta óptima
    path_edges = [(optimal_path[i], optimal_path[i + 1]) for i in range(len(optimal_path) - 1)]
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='blue', width=2)

    plt.title('Red Mesh con Q-Table y Ruta Óptima')
    plt.show()

visualize_network(Q, optimal_path)
