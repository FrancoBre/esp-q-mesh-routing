"""
Visualization server for Q-routing (Boyan & Littman 1993).
Receives delivery data from middleware, provides real-time visualization
of learning dynamics: topology, Q-values, hop counts, convergence, loops.
"""
import json
import time
import queue

from flask import Flask, request, Response, jsonify, render_template_string

LOG_FILE = 'received_data.log'

app = Flask(__name__)


# ---------------------------------------------------------------------------
# Data model
# ---------------------------------------------------------------------------

class SessionState:
    """Incrementally built state from DELIVERY_DATA events."""

    def __init__(self):
        self.nodes = set()
        self.node_labels = {}           # node_id -> short label ("A", "B", ...)
        self.q_snapshots = []           # [(event_idx, timestamp, q_table)]
        self.packets = {}               # packet_id -> packet record dict
        self.events = []                # raw events, append-only
        self.hop_counts = []            # per packet in delivery order
        self.rolling_avg_hops = []      # window=5
        self.q_deltas = []              # max |delta Q| per event
        self.loop_count = 0
        self._label_counter = 0

    def _assign_labels(self, q_table):
        """Assign short labels to nodes deterministically (sorted IDs)."""
        all_ids = sorted(q_table.keys())
        for nid in all_ids:
            if nid not in self.node_labels:
                label = chr(ord('A') + self._label_counter)
                self.node_labels[nid] = label
                self._label_counter += 1
            self.nodes.add(nid)

    def _detect_loops(self, steps):
        """Return (has_loop, loop_nodes) from a packet's step list."""
        visited = []
        loop_nodes = []
        for step in steps:
            node = step.get('node_from', '')
            if node in visited:
                loop_nodes.append(node)
            visited.append(node)
        return len(loop_nodes) > 0, loop_nodes

    def _compute_q_delta(self, new_q):
        """Max |delta Q| between latest snapshot and new Q-table."""
        if not self.q_snapshots:
            # First event: delta is max absolute Q-value
            max_delta = 0.0
            for frm in new_q:
                for to in new_q[frm]:
                    max_delta = max(max_delta, abs(new_q[frm][to]))
            return max_delta

        old_q = self.q_snapshots[-1][2]
        max_delta = 0.0
        for frm in new_q:
            for to in new_q.get(frm, {}):
                old_val = old_q.get(frm, {}).get(to, 0.0)
                new_val = new_q[frm][to]
                max_delta = max(max_delta, abs(new_val - old_val))
        return max_delta

    def _rolling_avg(self, window=5):
        """Compute rolling average of last `window` hop counts."""
        if not self.hop_counts:
            return 0.0
        recent = self.hop_counts[-window:]
        return sum(recent) / len(recent)

    def ingest_event(self, event, timestamp=None):
        """Process a DELIVERY_DATA event. Returns the new packet record."""
        if timestamp is None:
            timestamp = time.time()

        event_idx = len(self.events)
        self.events.append(event)

        q_table = event.get('q_table', {})
        self._assign_labels(q_table)

        # Q-table snapshot and delta
        q_delta = self._compute_q_delta(q_table)
        self.q_deltas.append(q_delta)
        self.q_snapshots.append((event_idx, timestamp, q_table))

        # Extract only the current packet (dedup)
        current_pid = event.get('current_packet_id', 0)
        all_packets = event.get('packets', [])
        current_pkt = None
        for p in all_packets:
            pid = p.get('packet_id', p.get('episode_number', 0))
            if pid == current_pid:
                current_pkt = p
                break

        packet_record = None
        if current_pkt:
            steps = current_pkt.get('steps', [])
            has_loop, loop_nodes = self._detect_loops(steps)
            hc = len(steps)

            packet_record = {
                'packet_id': current_pid,
                'steps': steps,
                'hop_count': hc,
                'has_loop': has_loop,
                'loop_nodes': loop_nodes,
                'event_index': event_idx,
                'timestamp': timestamp,
                'esp_timestamp': event.get('send_timestamp', 0),
                'receiver_node': event.get('current_node_id', ''),
            }
            self.packets[current_pid] = packet_record
            self.hop_counts.append(hc)
            self.rolling_avg_hops.append(self._rolling_avg())
            if has_loop:
                self.loop_count += 1

        return packet_record

    def get_latest_q_table(self):
        if not self.q_snapshots:
            return {}
        return self.q_snapshots[-1][2]

    def get_latest_packet(self):
        if not self.packets:
            return None
        max_pid = max(self.packets.keys())
        return self.packets[max_pid]

    def to_api_state(self):
        """Serialize full state for /api/state endpoint."""
        sorted_packets = sorted(self.packets.values(), key=lambda p: p['packet_id'])
        return {
            'nodes': sorted(self.nodes),
            'node_labels': self.node_labels,
            'q_table': self.get_latest_q_table(),
            'q_snapshots': [
                {'event_idx': s[0], 'timestamp': s[1], 'q_table': s[2]}
                for s in self.q_snapshots
            ],
            'packets': sorted_packets,
            'hop_counts': self.hop_counts,
            'rolling_avg_hops': self.rolling_avg_hops,
            'q_deltas': self.q_deltas,
            'loop_count': self.loop_count,
            'total_packets': len(self.packets),
            'total_events': len(self.events),
        }


# Global state
state = SessionState()
sse_listeners = []  # list of queue.Queue, one per connected browser


def load_session():
    """Replay events from log file to rebuild state on startup."""
    try:
        with open(LOG_FILE, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                    state.ingest_event(event)
                except json.JSONDecodeError:
                    continue
        if state.events:
            print(f"Loaded {len(state.events)} events from {LOG_FILE} "
                  f"({len(state.packets)} packets, {state.loop_count} loops)")
    except FileNotFoundError:
        print(f"No existing log file ({LOG_FILE}), starting fresh")


def notify_listeners(data):
    """Push SSE event to all connected browsers."""
    dead = []
    for q in sse_listeners:
        try:
            q.put_nowait(data)
        except queue.Full:
            dead.append(q)
    for q in dead:
        sse_listeners.remove(q)


# ---------------------------------------------------------------------------
# Helpers (kept for backward-compat with GET / rendering)
# ---------------------------------------------------------------------------

def path_to_str(steps, labels=None):
    """Format path as A -> B -> C using short labels."""
    if not steps:
        return '-'
    parts = []
    for s in steps:
        nf = s.get('node_from', '?')
        parts.append(labels.get(nf, nf) if labels else nf)
    if steps:
        last = steps[-1].get('node_to', '?')
        parts.append(labels.get(last, last) if labels else last)
    return ' -> '.join(parts)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.route('/data', methods=['POST'])
def receive_data():
    received = request.json
    timestamp = time.time()

    packet_record = state.ingest_event(received, timestamp)

    with open(LOG_FILE, 'a') as f:
        json.dump(received, f)
        f.write('\n')

    # Push SSE update (delta only)
    sse_data = {
        'type': 'new_event',
        'event_index': len(state.events) - 1,
        'packet': packet_record,
        'q_table': state.get_latest_q_table(),
        'q_delta': state.q_deltas[-1] if state.q_deltas else 0,
        'hop_counts': state.hop_counts,
        'rolling_avg_hops': state.rolling_avg_hops,
        'node_labels': state.node_labels,
        'nodes': sorted(state.nodes),
        'total_packets': len(state.packets),
        'loop_count': state.loop_count,
    }
    notify_listeners(sse_data)

    pid = packet_record['packet_id'] if packet_record else '?'
    loop = ' [LOOP]' if packet_record and packet_record['has_loop'] else ''
    print(f"Packet #{pid}: {packet_record['hop_count']} hops{loop}" if packet_record else "Event received (no packet)")

    return "Data received", 200


@app.route('/api/state')
def api_state():
    return jsonify(state.to_api_state())


@app.route('/api/stream')
def api_stream():
    def generate():
        q = queue.Queue(maxsize=100)
        sse_listeners.append(q)
        try:
            while True:
                data = q.get()
                yield f"data: {json.dumps(data)}\n\n"
        except GeneratorExit:
            sse_listeners.remove(q)

    return Response(generate(), mimetype='text/event-stream',
                    headers={'Cache-Control': 'no-cache',
                             'X-Accel-Buffering': 'no'})


@app.route('/')
def index():
    """Serve the visualization page."""
    return render_template_string(PAGE_HTML)


# ---------------------------------------------------------------------------
# HTML template (client-side Plotly.js rendering)
# ---------------------------------------------------------------------------

PAGE_HTML = r'''
<!DOCTYPE html>
<html>
<head>
    <title>Q-Routing Debugger</title>
    <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { font-family: 'Courier New', monospace; background: #1a1a2e; color: #e0e0e0; padding: 16px; }
        h1 { color: #00d4ff; margin-bottom: 8px; font-size: 1.4em; }
        h2 { color: #7ec8e3; margin: 16px 0 8px; font-size: 1.1em; border-bottom: 1px solid #333; padding-bottom: 4px; }
        .status-bar { background: #16213e; padding: 8px 12px; border-radius: 4px; margin-bottom: 12px;
                      display: flex; gap: 24px; font-size: 0.9em; }
        .status-bar .stat { color: #aaa; }
        .status-bar .val { color: #00d4ff; font-weight: bold; }
        .status-bar .warn { color: #ff6b6b; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
        .panel { background: #16213e; border: 1px solid #2a2a4a; border-radius: 4px; padding: 12px; }
        .panel.full { grid-column: 1 / -1; }
        .chart { width: 100%; height: 350px; }
        .chart.tall { height: 450px; }
        table { width: 100%; border-collapse: collapse; font-size: 0.85em; }
        th, td { border: 1px solid #2a2a4a; padding: 6px 8px; text-align: left; }
        th { background: #0f3460; color: #7ec8e3; }
        td { background: #16213e; }
        .loop-yes { color: #ff6b6b; font-weight: bold; }
        .loop-no { color: #4ecca3; }
        .event-log { max-height: 200px; overflow-y: auto; font-size: 0.8em; background: #0a0a1a;
                     padding: 8px; border-radius: 4px; white-space: pre-wrap; word-break: break-all; }
        .event-log .entry { border-bottom: 1px solid #1a1a2e; padding: 2px 0; }
        .connected { color: #4ecca3; }
        .disconnected { color: #ff6b6b; }
    </style>
</head>
<body>
    <h1>Q-Routing Debugger</h1>
    <div class="status-bar">
        <span class="stat">Packets: <span class="val" id="stat-packets">0</span></span>
        <span class="stat">Loops: <span id="stat-loops" class="val">0</span></span>
        <span class="stat">Latest hops: <span class="val" id="stat-hops">-</span></span>
        <span class="stat">Avg hops: <span class="val" id="stat-avg">-</span></span>
        <span class="stat">Max |dQ|: <span class="val" id="stat-dq">-</span></span>
        <span class="stat">Stream: <span id="stat-stream" class="disconnected">connecting...</span></span>
    </div>

    <div class="grid">
        <div class="panel full">
            <h2>Network Topology</h2>
            <div id="topology-chart" class="chart tall"></div>
        </div>

        <div class="panel">
            <h2>Hop Count (Learning Curve)</h2>
            <div id="hops-chart" class="chart"></div>
        </div>

        <div class="panel">
            <h2>Convergence (max |delta Q| per event)</h2>
            <div id="convergence-chart" class="chart"></div>
        </div>

        <div class="panel">
            <h2>Q-Table Heatmap</h2>
            <div id="heatmap-chart" class="chart"></div>
        </div>

        <div class="panel">
            <h2>Packet Details</h2>
            <div style="max-height:350px; overflow-y:auto;">
                <table id="packet-table">
                    <thead><tr><th>#</th><th>Hops</th><th>Loop</th><th>Path</th><th>ESP ts</th></tr></thead>
                    <tbody></tbody>
                </table>
            </div>
        </div>

        <div class="panel full">
            <h2>Event Log</h2>
            <div id="event-log" class="event-log">Waiting for data...</div>
        </div>
    </div>

<script>
// ---- State ----
let S = {
    nodes: [], node_labels: {}, q_table: {}, packets: [],
    hop_counts: [], rolling_avg_hops: [], q_deltas: [],
    q_snapshots: [], loop_count: 0, total_packets: 0, total_events: 0
};
let eventLog = [];

// ---- Init ----
async function init() {
    try {
        const resp = await fetch('/api/state');
        S = await resp.json();
        renderAll();
        connectSSE();
    } catch(e) {
        document.getElementById('stat-stream').textContent = 'fetch failed';
    }
}

function connectSSE() {
    const es = new EventSource('/api/stream');
    es.onopen = () => {
        const el = document.getElementById('stat-stream');
        el.textContent = 'connected';
        el.className = 'connected';
    };
    es.onmessage = (e) => {
        const d = JSON.parse(e.data);
        // Update state incrementally
        S.nodes = d.nodes || S.nodes;
        S.node_labels = d.node_labels || S.node_labels;
        S.q_table = d.q_table || S.q_table;
        S.hop_counts = d.hop_counts || S.hop_counts;
        S.rolling_avg_hops = d.rolling_avg_hops || S.rolling_avg_hops;
        S.total_packets = d.total_packets || S.total_packets;
        S.loop_count = d.loop_count != null ? d.loop_count : S.loop_count;
        if (d.q_delta != null) S.q_deltas.push(d.q_delta);
        if (d.packet) S.packets.push(d.packet);
        if (d.q_table) S.q_snapshots.push({q_table: d.q_table, event_idx: d.event_index});
        eventLog.unshift(JSON.stringify(d.packet || d, null, 0));
        if (eventLog.length > 20) eventLog.pop();
        renderAll();
    };
    es.onerror = () => {
        const el = document.getElementById('stat-stream');
        el.textContent = 'disconnected';
        el.className = 'disconnected';
    };
}

// ---- Render all views ----
function renderAll() {
    renderStatusBar();
    renderTopology();
    renderHopsChart();
    renderConvergenceChart();
    renderHeatmap();
    renderPacketTable();
    renderEventLog();
}

// ---- Status bar ----
function renderStatusBar() {
    document.getElementById('stat-packets').textContent = S.total_packets;
    const loopEl = document.getElementById('stat-loops');
    loopEl.textContent = S.loop_count;
    loopEl.className = S.loop_count > 0 ? 'warn' : 'val';
    const hc = S.hop_counts;
    document.getElementById('stat-hops').textContent = hc.length ? hc[hc.length-1] : '-';
    const ra = S.rolling_avg_hops;
    document.getElementById('stat-avg').textContent = ra.length ? ra[ra.length-1].toFixed(1) : '-';
    document.getElementById('stat-dq').textContent = S.q_deltas.length ? S.q_deltas[S.q_deltas.length-1].toFixed(4) : '-';
}

// ---- Topology ----
function renderTopology() {
    const q = S.q_table;
    const labels = S.node_labels;
    const nodes = S.nodes;
    if (!nodes.length) return;

    // Fixed circular layout
    const pos = {};
    nodes.forEach((n, i) => {
        const angle = (2 * Math.PI * i) / nodes.length - Math.PI/2;
        pos[n] = [Math.cos(angle), Math.sin(angle)];
    });

    // Find preferred next-hop per node (argmin Q > 0)
    const preferred = {};
    for (const frm in q) {
        let bestTo = null, bestQ = Infinity;
        for (const to in q[frm]) {
            const v = q[frm][to];
            if (v > 0 && v < bestQ) { bestQ = v; bestTo = to; }
        }
        if (bestTo) preferred[frm] = bestTo;
    }

    // Latest packet path edges
    const latestPkt = S.packets.length ? S.packets[S.packets.length - 1] : null;
    const pathEdges = new Set();
    if (latestPkt && latestPkt.steps) {
        latestPkt.steps.forEach(s => pathEdges.add(s.node_from + '->' + s.node_to));
    }

    // Loop edges from latest packet
    const loopEdges = new Set();
    if (latestPkt && latestPkt.has_loop && latestPkt.steps) {
        const visited = [];
        latestPkt.steps.forEach(s => {
            if (visited.includes(s.node_from)) {
                // Mark edges into and out of loop nodes
                loopEdges.add(s.node_from + '->' + s.node_to);
            }
            visited.push(s.node_from);
        });
    }

    const traces = [];
    const annotations = [];

    // Draw edges with Q-values
    for (const frm in q) {
        for (const to in q[frm]) {
            const v = q[frm][to];
            if (v === 0 && !pathEdges.has(frm+'->'+to)) continue;
            if (frm === to) continue;
            if (!pos[frm] || !pos[to]) continue;

            const edgeKey = frm + '->' + to;
            const isPref = preferred[frm] === to;
            const isPath = pathEdges.has(edgeKey);
            const isLoop = loopEdges.has(edgeKey);

            let color = '#444';
            let width = 1.5;
            if (isPref) { color = '#4ecca3'; width = 2.5; }
            if (isPath) { color = '#00d4ff'; width = 3; }
            if (isLoop) { color = '#ff6b6b'; width = 3; }

            // Offset for bidirectional edges
            const dx = pos[to][0] - pos[frm][0];
            const dy = pos[to][1] - pos[frm][1];
            const len = Math.sqrt(dx*dx + dy*dy) || 1;
            const perpX = -dy/len * 0.03;
            const perpY = dx/len * 0.03;

            const x0 = pos[frm][0] + perpX, y0 = pos[frm][1] + perpY;
            const x1 = pos[to][0] + perpX, y1 = pos[to][1] + perpY;

            traces.push({
                x: [x0, x1], y: [y0, y1],
                mode: 'lines', line: {width, color},
                hoverinfo: 'text',
                text: `${labels[frm]||frm} -> ${labels[to]||to}: Q=${v.toFixed(3)}`,
                showlegend: false
            });

            // Arrow annotation
            annotations.push({
                ax: x0, ay: y0, x: x1, y: y1,
                xref: 'x', yref: 'y', axref: 'x', ayref: 'y',
                showarrow: true, arrowhead: 3, arrowsize: 1.2, arrowwidth: width * 0.8,
                arrowcolor: color, opacity: 0.6
            });

            // Q-value label on edge midpoint
            if (v > 0) {
                const mx = (x0 + x1) / 2 + perpX * 3;
                const my = (y0 + y1) / 2 + perpY * 3;
                annotations.push({
                    x: mx, y: my, text: v.toFixed(2),
                    showarrow: false, font: {size: 9, color: color},
                    xref: 'x', yref: 'y'
                });
            }
        }
    }

    // Sender detection: node with outgoing Q > 0 but no incoming Q > 0
    // Receiver: current_node_id from latest event
    const receiver = latestPkt ? latestPkt.receiver_node : null;
    const hasIncoming = new Set();
    for (const frm in q) {
        for (const to in q[frm]) {
            if (q[frm][to] > 0) hasIncoming.add(to);
        }
    }
    const sender = nodes.find(n => {
        const row = q[n] || {};
        const hasOut = Object.values(row).some(v => v > 0);
        return hasOut && !hasIncoming.has(n);
    });

    // Node trace
    const nodeColors = nodes.map(n => {
        if (n === sender) return '#ff9f43';
        if (n === receiver) return '#4ecca3';
        return '#7ec8e3';
    });
    const nodeSymbols = nodes.map(n => {
        if (n === sender) return 'diamond';
        if (n === receiver) return 'square';
        return 'circle';
    });

    traces.push({
        x: nodes.map(n => pos[n][0]),
        y: nodes.map(n => pos[n][1]),
        text: nodes.map(n => labels[n] || n),
        customdata: nodes.map(n => n),
        hovertemplate: '%{text} (%{customdata})<extra></extra>',
        mode: 'markers+text', textposition: 'top center',
        textfont: {size: 14, color: '#fff', family: 'Courier New'},
        marker: {size: 20, color: nodeColors, symbol: nodeSymbols,
                 line: {width: 2, color: '#fff'}},
        showlegend: false
    });

    const layout = {
        paper_bgcolor: '#16213e', plot_bgcolor: '#16213e',
        xaxis: {visible: false, range: [-1.5, 1.5]},
        yaxis: {visible: false, range: [-1.5, 1.5], scaleanchor: 'x'},
        margin: {l: 20, r: 20, t: 10, b: 10},
        annotations: annotations,
        showlegend: false
    };
    Plotly.react('topology-chart', traces, layout, {displayModeBar: false});
}

// ---- Hop count chart ----
function renderHopsChart() {
    if (!S.hop_counts.length) return;
    const x = S.hop_counts.map((_, i) => i + 1);

    // Color markers by loop status
    const colors = S.packets.map(p => p.has_loop ? '#ff6b6b' : '#4ecca3');

    const traces = [
        {
            x, y: S.hop_counts, mode: 'lines+markers', name: 'Hop count',
            line: {color: '#00d4ff', width: 1.5},
            marker: {color: colors, size: 8, line: {width: 1, color: '#fff'}},
        },
        {
            x, y: S.rolling_avg_hops, mode: 'lines', name: 'Rolling avg (5)',
            line: {color: '#ff9f43', width: 2, dash: 'dot'},
        },
    ];
    // Optimal line at y=1
    const shapes = [{
        type: 'line', x0: 1, x1: x.length, y0: 1, y1: 1,
        line: {color: '#4ecca3', width: 1, dash: 'dash'},
    }];

    const layout = {
        paper_bgcolor: '#16213e', plot_bgcolor: '#0a0a1a',
        xaxis: {title: 'Packet #', color: '#aaa', gridcolor: '#1a1a2e'},
        yaxis: {title: 'Hops', color: '#aaa', gridcolor: '#1a1a2e', rangemode: 'tozero'},
        margin: {l: 50, r: 20, t: 10, b: 40},
        legend: {font: {color: '#aaa'}, x: 0.7, y: 0.95},
        shapes,
    };
    Plotly.react('hops-chart', traces, layout, {displayModeBar: false});
}

// ---- Convergence chart ----
function renderConvergenceChart() {
    if (!S.q_deltas.length) return;
    const x = S.q_deltas.map((_, i) => i + 1);

    const traces = [{
        x, y: S.q_deltas, mode: 'lines+markers', name: 'max |dQ|',
        line: {color: '#e94560', width: 1.5},
        marker: {size: 5, color: '#e94560'},
    }];

    const shapes = [{
        type: 'line', x0: 1, x1: x.length, y0: 0.01, y1: 0.01,
        line: {color: '#4ecca3', width: 1, dash: 'dash'},
    }];

    const layout = {
        paper_bgcolor: '#16213e', plot_bgcolor: '#0a0a1a',
        xaxis: {title: 'Event #', color: '#aaa', gridcolor: '#1a1a2e'},
        yaxis: {title: 'max |delta Q|', color: '#aaa', gridcolor: '#1a1a2e', type: 'log'},
        margin: {l: 60, r: 20, t: 10, b: 40},
        legend: {font: {color: '#aaa'}},
        shapes,
    };
    Plotly.react('convergence-chart', traces, layout, {displayModeBar: false});
}

// ---- Q-table heatmap ----
function renderHeatmap() {
    const q = S.q_table;
    const labels = S.node_labels;
    const nodes = S.nodes;
    if (!nodes.length) return;

    const shortLabels = nodes.map(n => labels[n] || n);
    const z = nodes.map(frm =>
        nodes.map(to => {
            const v = (q[frm] || {})[to] || 0;
            return v;
        })
    );
    const text = nodes.map(frm =>
        nodes.map(to => {
            const v = (q[frm] || {})[to] || 0;
            return v > 0 ? v.toFixed(2) : '';
        })
    );

    const traces = [{
        z, x: shortLabels, y: shortLabels, text, texttemplate: '%{text}',
        type: 'heatmap',
        colorscale: [[0, '#0a0a1a'], [0.01, '#16213e'], [0.5, '#0f3460'], [1, '#e94560']],
        showscale: true,
        colorbar: {tickfont: {color: '#aaa'}, title: {text: 'Q', font: {color: '#aaa'}}},
    }];
    const layout = {
        paper_bgcolor: '#16213e', plot_bgcolor: '#16213e',
        xaxis: {title: 'To', color: '#aaa', side: 'bottom'},
        yaxis: {title: 'From', color: '#aaa', autorange: 'reversed'},
        margin: {l: 50, r: 20, t: 10, b: 50},
    };
    Plotly.react('heatmap-chart', traces, layout, {displayModeBar: false});
}

// ---- Packet table ----
function renderPacketTable() {
    const tbody = document.querySelector('#packet-table tbody');
    const labels = S.node_labels;
    // Show newest first
    const pkts = [...S.packets].reverse();
    tbody.innerHTML = pkts.map(p => {
        const pathStr = (p.steps || []).map((s, i) => {
            const from = labels[s.node_from] || s.node_from;
            const to = labels[s.node_to] || s.node_to;
            return i === 0 ? from + ' -> ' + to : to;
        }).join(' -> ') || '-';
        const loopClass = p.has_loop ? 'loop-yes' : 'loop-no';
        const loopText = p.has_loop ? 'YES' : 'no';
        return `<tr>
            <td>${p.packet_id}</td>
            <td>${p.hop_count}</td>
            <td class="${loopClass}">${loopText}</td>
            <td>${pathStr}</td>
            <td>${p.esp_timestamp}</td>
        </tr>`;
    }).join('');
}

// ---- Event log ----
function renderEventLog() {
    const el = document.getElementById('event-log');
    if (!eventLog.length && !S.packets.length) return;
    // On initial load, populate from packets
    if (!eventLog.length && S.packets.length) {
        S.packets.slice(-20).reverse().forEach(p => {
            const lbl = S.node_labels;
            const pathStr = (p.steps || []).map(s => (lbl[s.node_from]||s.node_from)).join('->');
            const last = p.steps && p.steps.length ? (lbl[p.steps[p.steps.length-1].node_to]||'?') : '';
            eventLog.push(`Pkt #${p.packet_id}: ${p.hop_count} hops [${pathStr}->${last}]${p.has_loop ? ' LOOP' : ''}`);
        });
    }
    el.innerHTML = eventLog.map(e => `<div class="entry">${e}</div>`).join('');
}

// Start
init();
</script>
</body>
</html>
'''


# ---------------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    load_session()
    app.run(host='0.0.0.0', port=5000, threaded=True)
