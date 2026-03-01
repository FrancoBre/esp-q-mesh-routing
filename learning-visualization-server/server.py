"""
Visualization server for Q-routing (Boyan & Littman 1993).
Receives delivery data from middleware, shows topology, hop count, and table.
"""
import json

from flask import Flask, request, render_template_string
import plotly.express as px
import plotly.io as pio
import plotly.graph_objs as go
import networkx as nx

LOG_FILE = 'received_data.log'

app = Flask(__name__)
data = []  # Store received JSON (delivery events)


def hop_count(ep):
    """Extract hop count from delivered packet (steps length)."""
    steps = ep.get('steps', [])
    return len(steps)


def path_to_str(ep):
    """Format path as node_from -> node_to -> ..."""
    steps = ep.get('steps', [])
    parts = []
    for s in steps:
        parts.append(str(s.get('node_from', '?')))
    if steps:
        last = steps[-1].get('node_to', '?')
        parts.append(str(last))
    return ' -> '.join(parts) if parts else '-'


@app.route('/')
def index():
    global data
    G = nx.Graph()
    edge_trace = []

    # Network topology from q_table
    topology_html = "<p>No topology data available</p>"
    if data:
        q_table = data[-1].get('q_table', {})
        for node_from, connections in q_table.items():
            G.add_node(node_from)
            for node_to in connections:
                G.add_edge(node_from, node_to)

        if G.number_of_nodes() > 0:
            pos = nx.spring_layout(G)
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

            fig2 = go.Figure(data=edge_trace + [node_trace])
            topology_html = pio.to_html(fig2, full_html=False)

    # Hop count per delivered packet
    hops_html = "<p>No delivery data available</p>"
    if data:
        packet_numbers = []
        hop_counts = []
        for entry in data:
            packets = entry.get('episodes', [])  # legacy field: delivered packets
            for ep in packets:
                packet_numbers.append(ep.get('episode_number', 0))  # legacy field
                hop_counts.append(hop_count(ep))

        if packet_numbers and hop_counts:
            fig = px.line(
                x=packet_numbers, y=hop_counts,
                labels={'x': 'Delivered Packet #', 'y': 'Hop Count'},
                title="Hop Count per Delivered Packet (path length)"
            )
            hops_html = pio.to_html(fig, full_html=False)

    # Table: packet #, hop_count, path, accumulated_reward
    table_html = "<p>No data available</p>"
    if data:
        table_html = (
            '<table border="1"><tr><th>Node ID</th><th>Packet #</th>'
            '<th>Hop Count</th><th>Path</th><th>Accumulated Reward</th></tr>'
        )
        for entry in data:
            node_id = entry.get('current_node_id', 'N/A')
            accumulated_reward = entry.get('accumulated_reward', 'N/A')
            packets = entry.get('episodes', [])  # legacy field: delivered packets
            for ep in packets:
                pkt_num = ep.get('episode_number', 'N/A')  # legacy field
                hc = hop_count(ep)
                path = path_to_str(ep)
                table_html += (
                    f'<tr><td>{node_id}</td><td>{pkt_num}</td>'
                    f'<td>{hc}</td><td>{path}</td><td>{accumulated_reward}</td></tr>'
                )
        table_html += '</table>'

    return render_template_string('''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Q-Routing Learning Progress</title>
        <style>
            table { width: 100%; border-collapse: collapse; }
            th, td { border: 1px solid black; padding: 8px; text-align: left; }
            th { background-color: #f2f2f2; }
        </style>
    </head>
    <body>
        <h1>Q-Routing Learning Progress</h1>
        <h2>Network Topology</h2>
        {{ topology_html|safe }}
        <h2>Hop Count per Delivered Packet</h2>
        {{ hops_html|safe }}
        <h2>Received Data</h2>
        {{ table_html|safe }}
    </body>
    </html>
    ''', topology_html=topology_html, hops_html=hops_html, table_html=table_html)


@app.route('/data', methods=['POST'])
def receive_data():
    global data
    received_data = request.json
    data.append(received_data)

    with open(LOG_FILE, 'a') as f:
        json.dump(received_data, f)
        f.write('\n')

    print("Received data:", received_data)
    return "Data received", 200


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
