#!/usr/bin/env python3
"""
N8N-Style Workflow Visualization for Dora Dataflows
Creates an interactive workflow diagram similar to n8n's interface.
Uses jsPlumb for connection rendering and draggable nodes.
"""

import yaml
import json
import sys
from pathlib import Path

def parse_yaml(yaml_file):
    """Parse the dataflow YAML file."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    nodes = []
    connections = []
    
    # Define node positions in a more compact grid layout
    grid_positions = [
        (80, 80),     # 0: microphone
        (80, 180),    # 1: speech-monitor
        (80, 280),    # 2: asr
        (280, 180),   # 3: chat-controller
        (480, 180),   # 4: llm
        (480, 280),   # 5: text-segmenter
        (280, 380),   # 6: conversation-controller
        (480, 380),   # 7: primespeech
        (480, 480),   # 8: audio-player
        (680, 280),   # 9: log-display
    ]
    
    # Node type configurations with high contrast colors
    node_configs = {
        # INPUT - Bright Red/Orange (High visibility for data entry points)
        'microphone': {'icon': '🎤', 'color': '#FF1744', 'category': 'Input'},
        
        # PROCESSING - Deep Blue (Core processing nodes)
        'speech': {'icon': '🔊', 'color': '#2962FF', 'category': 'Processing'},
        'asr': {'icon': '📝', 'color': '#0D47A1', 'category': 'Processing'},
        'segment': {'icon': '✂️', 'color': '#1565C0', 'category': 'Processing'},
        
        # CONTROL - Vibrant Green (Control flow nodes)
        'controller': {'icon': '🎮', 'color': '#00C853', 'category': 'Control'},
        'conversation-controller': {'icon': '🔄', 'color': '#00E676', 'category': 'Control'},
        'chat-controller': {'icon': '💬', 'color': '#00C853', 'category': 'Control'},
        
        # AI/LLM - Deep Purple (AI processing)
        'llm': {'icon': '🤖', 'color': '#AA00FF', 'category': 'AI'},
        'qwen': {'icon': '🤖', 'color': '#AA00FF', 'category': 'AI'},
        
        # TTS - Bright Orange (Text to Speech)
        'tts': {'icon': '🗣️', 'color': '#FF6D00', 'category': 'Synthesis'},
        'primespeech': {'icon': '🗣️', 'color': '#FF6D00', 'category': 'Synthesis'},
        
        # OUTPUT - Hot Pink/Magenta (High visibility for data exit points)
        'player': {'icon': '🔈', 'color': '#FF0080', 'category': 'Output'},
        'audio-player': {'icon': '🔈', 'color': '#FF0080', 'category': 'Output'},
        
        # LOG/MONITORING - Dark Gray/Black (Utility nodes)
        'display': {'icon': '📋', 'color': '#424242', 'category': 'Monitoring'},
        'log': {'icon': '📋', 'color': '#424242', 'category': 'Monitoring'},
    }
    
    def get_node_config(node_id):
        """Get configuration for a node based on its ID."""
        node_lower = node_id.lower()
        
        # Check for exact matches first (for compound names)
        if 'chat-controller' in node_lower:
            return node_configs['chat-controller']
        if 'conversation-controller' in node_lower:
            return node_configs['conversation-controller']
        if 'audio-player' in node_lower:
            return node_configs['audio-player']
            
        # Then check for partial matches
        for key, config in node_configs.items():
            if key in node_lower:
                return config
        return {'icon': '📦', 'color': '#7f8c8d', 'category': 'Other'}
    
    # Parse nodes
    node_map = {}
    for idx, node in enumerate(data.get('nodes', [])):
        node_id = node['id']
        config = get_node_config(node_id)
        
        # Get position
        if idx < len(grid_positions):
            x, y = grid_positions[idx]
        else:
            x = 100 + (idx % 4) * 300
            y = 100 + (idx // 4) * 150
        
        # Shorten node names for display
        short_names = {
            'microphone': 'Mic',
            'speech-monitor': 'Speech',
            'asr': 'ASR',
            'chat-controller': 'Chat Ctrl',
            'qwen3-llm': 'LLM',
            'text-segmenter': 'Segment',
            'conversation-controller': 'Conv Ctrl',
            'primespeech': 'TTS',
            'audio-player': 'Player',
            'log-display': 'Logs'
        }
        
        display_name = short_names.get(node_id, node_id.replace('-', ' ').title()[:10])
        
        node_data = {
            'id': node_id,
            'name': display_name,
            'type': config['category'],
            'icon': config['icon'],
            'color': config['color'],
            'position': {'x': x, 'y': y},
            'inputs': [],
            'outputs': node.get('outputs', []),
            'env': node.get('env', {})
        }
        
        nodes.append(node_data)
        node_map[node_id] = node_data
        
        # Parse inputs
        inputs = node.get('inputs', {})
        if isinstance(inputs, dict):
            for input_name, input_source in inputs.items():
                if isinstance(input_source, dict):
                    source = input_source.get('source', '')
                else:
                    source = input_source
                
                if '/' in str(source):
                    source_node, source_output = source.split('/', 1)
                    
                    node_data['inputs'].append({
                        'name': input_name,
                        'from_node': source_node,
                        'from_output': source_output
                    })
                    
                    connections.append({
                        'from': source_node,
                        'to': node_id,
                        'from_output': source_output,
                        'to_input': input_name
                    })
    
    return {'nodes': nodes, 'connections': connections}

def generate_html(workflow_data):
    """Generate HTML with n8n-style workflow visualization."""
    
    html = f'''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Workflow Editor - N8N Style</title>
    
    <!-- jsPlumb for connections -->
    <script src="https://cdn.jsdelivr.net/npm/jsplumb@2.15.6/dist/js/jsplumb.min.js"></script>
    
    <style>
        * {{
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }}
        
        body {{
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #f3f4f6;
            overflow: hidden;
            height: 100vh;
        }}
        
        /* Header */
        .header {{
            background: white;
            border-bottom: 1px solid #e5e7eb;
            padding: 1rem 2rem;
            display: flex;
            align-items: center;
            justify-content: space-between;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }}
        
        .header h1 {{
            font-size: 1.5rem;
            color: #1f2937;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }}
        
        .header .subtitle {{
            color: #6b7280;
            font-size: 0.875rem;
        }}
        
        /* Canvas */
        #canvas {{
            position: relative;
            width: 100%;
            height: calc(100vh - 60px);
            background: #f9fafb;
            background-image: 
                linear-gradient(0deg, transparent 24%, rgba(209, 213, 219, 0.3) 25%, rgba(209, 213, 219, 0.3) 26%, transparent 27%, transparent 74%, rgba(209, 213, 219, 0.3) 75%, rgba(209, 213, 219, 0.3) 76%, transparent 77%, transparent),
                linear-gradient(90deg, transparent 24%, rgba(209, 213, 219, 0.3) 25%, rgba(209, 213, 219, 0.3) 26%, transparent 27%, transparent 74%, rgba(209, 213, 219, 0.3) 75%, rgba(209, 213, 219, 0.3) 76%, transparent 77%, transparent);
            background-size: 50px 50px;
            overflow: auto;
        }}
        
        /* Workflow nodes */
        .workflow-node {{
            position: absolute;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            min-width: 120px;
            cursor: move;
            transition: transform 0.1s, box-shadow 0.2s;
            border: 2px solid transparent;
        }}
        
        .workflow-node:hover {{
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            transform: translateY(-2px);
        }}
        
        .workflow-node.selected {{
            border-color: #3b82f6;
            box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
        }}
        
        .node-header {{
            padding: 8px 10px;
            border-radius: 6px 6px 0 0;
            color: white;
            font-weight: 600;
            font-size: 0.75rem;
            display: flex;
            align-items: center;
            gap: 6px;
        }}
        
        .node-icon {{
            font-size: 1rem;
            filter: drop-shadow(0 1px 2px rgba(0,0,0,0.1));
        }}
        
        .node-content {{
            padding: 6px 8px;
            font-size: 0.65rem;
            color: #4b5563;
        }}
        
        .node-ports {{
            display: flex;
            justify-content: space-between;
            padding: 0 6px 6px;
        }}
        
        .input-ports, .output-ports {{
            display: flex;
            flex-direction: column;
            gap: 2px;
            font-size: 0.6rem;
            color: #6b7280;
        }}
        
        .port {{
            display: flex;
            align-items: center;
            gap: 4px;
        }}
        
        .port-dot {{
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background: #d1d5db;
            border: 2px solid white;
            box-shadow: 0 1px 2px rgba(0,0,0,0.1);
        }}
        
        .input-ports .port-dot {{
            background: #60a5fa;
        }}
        
        .output-ports .port-dot {{
            background: #34d399;
        }}
        
        /* Connection styling */
        .jtk-connector {{
            stroke: #9ca3af;
            stroke-width: 2;
            fill: none;
        }}
        
        .jtk-connector:hover {{
            stroke: #3b82f6;
            stroke-width: 3;
        }}
        
        .jtk-endpoint {{
            fill: #3b82f6;
        }}
        
        /* Controls */
        .controls {{
            position: fixed;
            bottom: 2rem;
            right: 2rem;
            display: flex;
            gap: 1rem;
            z-index: 100;
        }}
        
        .control-btn {{
            background: white;
            border: 1px solid #e5e7eb;
            padding: 0.75rem 1.5rem;
            border-radius: 8px;
            cursor: pointer;
            font-size: 0.875rem;
            color: #374151;
            transition: all 0.2s;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }}
        
        .control-btn:hover {{
            background: #f9fafb;
            transform: translateY(-1px);
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        
        /* Sidebar */
        .sidebar {{
            position: fixed;
            right: 0;
            top: 60px;
            width: 300px;
            height: calc(100vh - 60px);
            background: white;
            border-left: 1px solid #e5e7eb;
            padding: 1.5rem;
            overflow-y: auto;
            transform: translateX(100%);
            transition: transform 0.3s;
            z-index: 50;
        }}
        
        .sidebar.open {{
            transform: translateX(0);
        }}
        
        .sidebar h2 {{
            font-size: 1.125rem;
            color: #1f2937;
            margin-bottom: 1rem;
        }}
        
        .node-info {{
            background: #f9fafb;
            padding: 1rem;
            border-radius: 6px;
            margin-bottom: 1rem;
        }}
        
        .node-info h3 {{
            font-size: 0.875rem;
            color: #374151;
            margin-bottom: 0.5rem;
        }}
        
        .node-info ul {{
            list-style: none;
            font-size: 0.75rem;
            color: #6b7280;
        }}
        
        .node-info li {{
            padding: 0.25rem 0;
            display: flex;
            align-items: center;
            gap: 0.5rem;
        }}
        
        /* Zoom controls */
        .zoom-controls {{
            position: fixed;
            bottom: 2rem;
            left: 2rem;
            display: flex;
            flex-direction: column;
            gap: 0.5rem;
            z-index: 100;
        }}
        
        .zoom-btn {{
            width: 40px;
            height: 40px;
            background: white;
            border: 1px solid #e5e7eb;
            border-radius: 8px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 1.25rem;
            color: #374151;
            transition: all 0.2s;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }}
        
        .zoom-btn:hover {{
            background: #f9fafb;
            transform: scale(1.1);
        }}
    </style>
</head>
<body>
    <div class="header">
        <div>
            <h1>🔄 Workflow Editor</h1>
            <div class="subtitle">N8N-Style Dataflow Visualization</div>
        </div>
        <div>
            <span style="color: #6b7280; font-size: 0.875rem;">
                {len(workflow_data['nodes'])} nodes • {len(workflow_data['connections'])} connections
            </span>
        </div>
    </div>
    
    <div id="canvas">
        <!-- Nodes will be generated here -->
    </div>
    
    <div class="controls">
        <button class="control-btn" onclick="autoLayout()">📐 Auto Layout</button>
        <button class="control-btn" onclick="toggleInfo()">ℹ️ Node Info</button>
        <button class="control-btn" onclick="exportWorkflow()">💾 Export</button>
    </div>
    
    <div class="zoom-controls">
        <button class="zoom-btn" onclick="zoomIn()">+</button>
        <button class="zoom-btn" onclick="zoomOut()">−</button>
        <button class="zoom-btn" onclick="zoomReset()">⟲</button>
    </div>
    
    <div class="sidebar" id="sidebar">
        <h2>Workflow Information</h2>
        <div id="node-details"></div>
    </div>
    
    <script>
        // Workflow data
        const workflowData = {json.dumps(workflow_data)};
        
        let jsPlumbInstance = null;
        let currentZoom = 1;
        let selectedNode = null;
        
        // Initialize jsPlumb
        jsPlumb.ready(function() {{
            jsPlumbInstance = jsPlumb.getInstance({{
                Endpoint: ["Dot", {{ radius: 5 }}],
                Connector: ["Bezier", {{ curviness: 50 }}],
                HoverPaintStyle: {{ stroke: "#3b82f6", strokeWidth: 3 }},
                ConnectionOverlays: [
                    ["Arrow", {{ location: 1, width: 10, length: 10 }}]
                ],
                Container: "canvas"
            }});
            
            // Create nodes
            createNodes();
            
            // Create connections
            createConnections();
            
            // Make nodes draggable
            jsPlumbInstance.draggable(document.querySelectorAll(".workflow-node"), {{
                grid: [25, 25],
                containment: false
            }});
            
            // Update sidebar with initial info
            updateSidebar();
        }});
        
        function createNodes() {{
            const canvas = document.getElementById('canvas');
            
            workflowData.nodes.forEach(node => {{
                const nodeEl = document.createElement('div');
                nodeEl.className = 'workflow-node';
                nodeEl.id = 'node-' + node.id;
                nodeEl.style.left = node.position.x + 'px';
                nodeEl.style.top = node.position.y + 'px';
                
                // Node header with custom color
                const header = document.createElement('div');
                header.className = 'node-header';
                header.style.background = node.color;
                header.innerHTML = `
                    <span class="node-icon">${{node.icon}}</span>
                    <span>${{node.name}}</span>
                `;
                
                // Node content
                const content = document.createElement('div');
                content.className = 'node-content';
                
                // Show inputs and outputs
                const ports = document.createElement('div');
                ports.className = 'node-ports';
                
                // Input ports
                if (node.inputs && node.inputs.length > 0) {{
                    const inputPorts = document.createElement('div');
                    inputPorts.className = 'input-ports';
                    inputPorts.innerHTML = '<div class="port"><span class="port-dot"></span>In</div>';
                    ports.appendChild(inputPorts);
                }}
                
                // Output ports
                if (node.outputs && node.outputs.length > 0) {{
                    const outputPorts = document.createElement('div');
                    outputPorts.className = 'output-ports';
                    outputPorts.innerHTML = '<div class="port"><span class="port-dot"></span>Out</div>';
                    ports.appendChild(outputPorts);
                }}
                
                nodeEl.appendChild(header);
                if (ports.children.length > 0) {{
                    nodeEl.appendChild(ports);
                }}
                
                // Click handler
                nodeEl.addEventListener('click', function() {{
                    selectNode(node);
                }});
                
                canvas.appendChild(nodeEl);
                
                // Add endpoints for connections
                if (node.inputs && node.inputs.length > 0) {{
                    jsPlumbInstance.addEndpoint(nodeEl, {{
                        anchor: "Left",
                        isTarget: true,
                        maxConnections: -1,
                        paintStyle: {{ fill: "#60a5fa", radius: 6 }}
                    }});
                }}
                
                if (node.outputs && node.outputs.length > 0) {{
                    jsPlumbInstance.addEndpoint(nodeEl, {{
                        anchor: "Right",
                        isSource: true,
                        maxConnections: -1,
                        paintStyle: {{ fill: "#34d399", radius: 6 }}
                    }});
                }}
            }});
        }}
        
        function createConnections() {{
            workflowData.connections.forEach(conn => {{
                const sourceEl = document.getElementById('node-' + conn.from);
                const targetEl = document.getElementById('node-' + conn.to);
                
                if (sourceEl && targetEl) {{
                    jsPlumbInstance.connect({{
                        source: sourceEl,
                        target: targetEl,
                        anchor: ["Right", "Left"],
                        paintStyle: {{ stroke: "#9ca3af", strokeWidth: 2 }},
                        overlays: [
                            ["Label", {{ 
                                label: conn.from_output,
                                location: 0.5,
                                cssClass: "connection-label"
                            }}]
                        ]
                    }});
                }}
            }});
        }}
        
        function selectNode(node) {{
            // Clear previous selection
            document.querySelectorAll('.workflow-node').forEach(el => {{
                el.classList.remove('selected');
            }});
            
            // Select new node
            const nodeEl = document.getElementById('node-' + node.id);
            nodeEl.classList.add('selected');
            selectedNode = node;
            
            // Update sidebar
            updateNodeDetails(node);
        }}
        
        function updateNodeDetails(node) {{
            const detailsEl = document.getElementById('node-details');
            
            let inputsHtml = '';
            if (node.inputs && node.inputs.length > 0) {{
                inputsHtml = '<h3>📥 Inputs:</h3><ul>';
                node.inputs.forEach(input => {{
                    inputsHtml += `<li>• ${{input.name}} ← ${{input.from_node}}/${{input.from_output}}</li>`;
                }});
                inputsHtml += '</ul>';
            }}
            
            let outputsHtml = '';
            if (node.outputs && node.outputs.length > 0) {{
                outputsHtml = '<h3>📤 Outputs:</h3><ul>';
                node.outputs.forEach(output => {{
                    outputsHtml += `<li>• ${{output}}</li>`;
                }});
                outputsHtml += '</ul>';
            }}
            
            detailsEl.innerHTML = `
                <div class="node-info">
                    <h3 style="color: ${{node.color}};">${{node.icon}} ${{node.name}}</h3>
                    <ul>
                        <li>Type: ${{node.type}}</li>
                        <li>ID: ${{node.id}}</li>
                    </ul>
                </div>
                <div class="node-info">
                    ${{inputsHtml}}
                </div>
                <div class="node-info">
                    ${{outputsHtml}}
                </div>
            `;
        }}
        
        function updateSidebar() {{
            const detailsEl = document.getElementById('node-details');
            detailsEl.innerHTML = `
                <div class="node-info">
                    <h3>📊 Workflow Statistics</h3>
                    <ul>
                        <li>Total Nodes: ${{workflowData.nodes.length}}</li>
                        <li>Total Connections: ${{workflowData.connections.length}}</li>
                    </ul>
                </div>
                <div class="node-info">
                    <h3>💡 Tips</h3>
                    <ul>
                        <li>• Drag nodes to rearrange</li>
                        <li>• Click nodes to see details</li>
                        <li>• Use Auto Layout to organize</li>
                        <li>• Scroll to pan the canvas</li>
                    </ul>
                </div>
            `;
        }}
        
        function autoLayout() {{
            // Compact grid layout
            const nodes = workflowData.nodes;
            const cols = 4;
            const spacing = 150;
            const startX = 80;
            const startY = 80;
            
            nodes.forEach((node, index) => {{
                const row = Math.floor(index / cols);
                const col = index % cols;
                const x = startX + col * spacing;
                const y = startY + row * spacing;
                
                const nodeEl = document.getElementById('node-' + node.id);
                nodeEl.style.left = x + 'px';
                nodeEl.style.top = y + 'px';
                
                node.position.x = x;
                node.position.y = y;
            }});
            
            jsPlumbInstance.repaintEverything();
        }}
        
        function toggleInfo() {{
            const sidebar = document.getElementById('sidebar');
            sidebar.classList.toggle('open');
        }}
        
        function exportWorkflow() {{
            const dataStr = JSON.stringify(workflowData, null, 2);
            const dataUri = 'data:application/json;charset=utf-8,' + encodeURIComponent(dataStr);
            
            const exportFileDefaultName = 'workflow.json';
            
            const linkElement = document.createElement('a');
            linkElement.setAttribute('href', dataUri);
            linkElement.setAttribute('download', exportFileDefaultName);
            linkElement.click();
        }}
        
        function zoomIn() {{
            currentZoom = Math.min(currentZoom + 0.1, 2);
            applyZoom();
        }}
        
        function zoomOut() {{
            currentZoom = Math.max(currentZoom - 0.1, 0.5);
            applyZoom();
        }}
        
        function zoomReset() {{
            currentZoom = 1;
            applyZoom();
        }}
        
        function applyZoom() {{
            const canvas = document.getElementById('canvas');
            canvas.style.transform = `scale(${{currentZoom}})`;
            canvas.style.transformOrigin = '0 0';
            jsPlumbInstance.setZoom(currentZoom);
        }}
    </script>
</body>
</html>'''
    
    return html

def main():
    if len(sys.argv) < 2:
        yaml_file = "../voice-chatbot/voice_chatbot.yml"
    else:
        yaml_file = sys.argv[1]
    
    if not Path(yaml_file).exists():
        print(f"Error: File '{yaml_file}' not found")
        sys.exit(1)
    
    print(f"🔄 Parsing {yaml_file}...")
    workflow_data = parse_yaml(yaml_file)
    
    print(f"📊 Found {len(workflow_data['nodes'])} nodes and {len(workflow_data['connections'])} connections")
    
    html_content = generate_html(workflow_data)
    
    output_file = "dataflow_n8n_workflow.html"
    with open(output_file, 'w') as f:
        f.write(html_content)
    
    print(f"✅ N8N-style workflow visualization saved to: {output_file}")
    print(f"📂 Open this file in your browser: {Path(output_file).absolute()}")
    print("\n🎯 Features:")
    print("  • Drag nodes to rearrange (snaps to grid)")
    print("  • Click nodes to see detailed information")
    print("  • Auto Layout button for automatic arrangement")
    print("  • Zoom controls for better navigation")
    print("  • Export workflow as JSON")
    print("\nTo open: ")
    print(f"  open {output_file}")

if __name__ == "__main__":
    main()