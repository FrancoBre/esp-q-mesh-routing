import json
from flask import Flask, request, render_template_string
import plotly.express as px
import plotly.io as pio
import plotly.graph_objs as go
import networkx as nx

LOG_FILE = 'received_data.log'

app = Flask(__name__)
data = []  # Store the received JSON data

@app.route('/')
def index():
    global data
    G = nx.Graph()
    edge_trace = []

    # Prepare data for the network topology plot
    topology_html = "<p>No topology data available</p>"
    if data:
        # Añadir nodos y aristas basados en la q_table
        q_table =  data[-1]['q_table']
        for node_from, connections in q_table.items():
            G.add_node(node_from)
            for node_to in connections:
                G.add_edge(node_from, node_to)

        # Asignar posiciones arbitrarias para los nodos
        pos = nx.spring_layout(G)  # Usa un layout automático de NetworkX

        for edge in G.edges():
            x0, y0 = pos[edge[0]]
            x1, y1 = pos[edge[1]]
            edge_trace.append(go.Scatter(
                x=[x0, x1, None], y=[y0, y1, None],
                line=dict(width=2, color='#888'),
                hoverinfo='none',
                mode='lines'))
            
        node_trace = go.Scatter(
        x=[pos[node][0] for node in G.nodes()],
        y=[pos[node][1] for node in G.nodes()],
        text=list(G.nodes()),
        mode='markers+text',
        hoverinfo='text',
        marker=dict(
            showscale=False,
            color='blue',
            size=10,
            line=dict(width=2)))

        # Crear la figura y añadir las trazas
        fig2 = go.Figure(data=edge_trace + [node_trace])

        # Convert the network topology graph to HTML
        topology_html = pio.to_html(fig2, full_html=False)

    # Prepare data for the rewards plot
    rewards_html = "<p>No reward data available</p>"
    if data:
        # Extract rewards and episode numbers
        rewards = []
        episode_numbers = []
        for entry in data:
            episodes = entry.get('episodes', [])
            for ep in episodes:
                episode_numbers.append(ep['episode_number'])
                rewards.append(ep['reward'])
        
        # Create the line plot
        if rewards and episode_numbers:
            fig = px.line(x=episode_numbers, y=rewards, labels={'x': 'Episode Number', 'y': 'Reward'}, title="Rewards per Episode")
            rewards_html = pio.to_html(fig, full_html=False)

    # Prepare data for the table
    table_html = "<p>No data available</p>"
    if data:
        table_html = '<table border="1"><tr><th>Node ID</th><th>Current Episode</th><th>Accumulated Reward</th><th>Total Time</th><th>Episodes</th></tr>'
        for entry in data:
            node_id = entry.get('current_node_id', 'N/A')
            current_episode = entry.get('current_episode', 'N/A')
            accumulated_reward = entry.get('accumulated_reward', 'N/A')
            total_time = entry.get('total_time', 'N/A')
            episodes = entry.get('episodes', [])
            episodes_html = '<br>'.join([f"Episode Number: {ep['episode_number']}, Reward: {ep['reward']}, Time: {ep['time']}" for ep in episodes])
            
            table_html += f'<tr><td>{node_id}</td><td>{current_episode}</td><td>{accumulated_reward}</td><td>{total_time}</td><td>{episodes_html}</td></tr>'
        table_html += '</table>'

    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Learning Progress</title>
        <style>
            table {
                width: 100%;
                border-collapse: collapse;
            }
            th, td {
                border: 1px solid black;
                padding: 8px;
                text-align: left;
            }
            th {
                background-color: #f2f2f2;
            }
        </style>
    </head>
    <body>
        <h1>Learning Progress</h1>
        {{ graph_html|safe }}
        <h2>Network Topology</h2>
        {{ topology_html|safe }}
        <h2>Rewards per Episode</h2>
        {{ rewards_html|safe }}
        <h2>Received Data</h2>
        {{ table_html|safe }}
    </body>
    </html>
    ''', topology_html=topology_html, rewards_html=rewards_html, table_html=table_html)

@app.route('/data', methods=['POST'])
def receive_data():
    global data
    received_data = request.json
    data.append(received_data)

    # Log the received data for further analysis
    with open(LOG_FILE, 'a') as f:
        json.dump(received_data, f)
        f.write('\n')

    print("Received data:", received_data)
    return "Data received", 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
