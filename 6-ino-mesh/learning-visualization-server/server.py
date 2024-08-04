import json
from flask import Flask, request, render_template_string
import plotly.express as px
import plotly.io as pio

LOG_FILE = 'received_data.log'

app = Flask(__name__)
data = []  # Store the received JSON data

@app.route('/')
def index():
    global data

    # Prepare data for the plot
    graph_html = "<p>No data available</p>"
    if data:
        episodes = data[-1]['episodes']
        episode_numbers = [ep['episode_number'] for ep in episodes]
        rewards = [ep['reward'] for ep in episodes]

        fig = px.line(x=episode_numbers, y=rewards, labels={'x': 'Episode', 'y': 'Reward'}, title='Rewards per Episode')
        graph_html = pio.to_html(fig, full_html=False)

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
        <h2>Received Data</h2>
        {{ table_html|safe }}
    </body>
    </html>
    ''', graph_html=graph_html, table_html=table_html)

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
