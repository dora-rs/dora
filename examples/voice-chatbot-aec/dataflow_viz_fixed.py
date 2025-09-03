#!/usr/bin/env python3
"""
Fixed Layout Dataflow Visualizer
Creates an HTML visualization with predetermined node positions to avoid overlap.
"""

import yaml
import json
import sys
from pathlib import Path

def parse_yaml(yaml_file):
    """Parse the dataflow YAML file and assign fixed positions."""
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)
    
    nodes = []
    links = []
    
    # Define fixed positions for common nodes in a vertical flow
    positions = {
        'microphone': {'x': 700, 'y': 50},
        'speech-monitor': {'x': 700, 'y': 150},
        'asr': {'x': 700, 'y': 250},
        'chat-controller': {'x': 700, 'y': 350},
        'qwen3-llm': {'x': 700, 'y': 450},
        'qwen3': {'x': 700, 'y': 450},
        'text-segmenter': {'x': 700, 'y': 550},
        'primespeech': {'x': 700, 'y': 650},
        'audio-player': {'x': 700, 'y': 750},
        'conversation-controller': {'x': 400, 'y': 500},
        'log-display': {'x': 1000, 'y': 400},
    }
    
    # Parse nodes
    for idx, node in enumerate(data.get('nodes', [])):
        node_id = node['id']
        
        # Determine color based on node type
        color = '#4CAF50'  # default green
        if 'microphone' in node_id.lower():
            color = '#F44336'  # red
        elif 'speech' in node_id.lower():
            color = '#2196F3'  # blue
        elif 'asr' in node_id.lower():
            color = '#FFEB3B'  # yellow
        elif 'controller' in node_id.lower():
            color = '#4CAF50'  # green
        elif 'llm' in node_id.lower() or 'qwen' in node_id.lower():
            color = '#9C27B0'  # purple
        elif 'segment' in node_id.lower():
            color = '#00BCD4'  # cyan
        elif 'tts' in node_id.lower() or 'primespeech' in node_id.lower():
            color = '#FF9800'  # orange
        elif 'player' in node_id.lower():
            color = '#F44336'  # red
        elif 'display' in node_id.lower() or 'log' in node_id.lower():
            color = '#9E9E9E'  # gray
        
        # Get position or calculate default
        if node_id in positions:
            x = positions[node_id]['x']
            y = positions[node_id]['y']
        else:
            # Place unknown nodes on the side
            x = 100
            y = 100 + (idx * 100)
        
        nodes.append({
            'id': node_id,
            'name': node_id,
            'color': color,
            'x': x,
            'y': y,
            'fx': x,  # Fixed x position
            'fy': y   # Fixed y position
        })
        
        # Parse inputs and create links
        inputs = node.get('inputs', {})
        if isinstance(inputs, dict):
            for input_name, input_source in inputs.items():
                if isinstance(input_source, dict):
                    source = input_source.get('source', '')
                else:
                    source = input_source
                
                if '/' in str(source):
                    source_node, source_output = source.split('/', 1)
                    links.append({
                        'source': source_node,
                        'target': node_id,
                        'label': source_output + ' → ' + input_name
                    })
    
    return {'nodes': nodes, 'links': links}

def generate_html(graph_data):
    """Generate HTML with fixed node positions."""
    
    html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Dataflow Visualization - Fixed Layout</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        }}
        
        h1 {{
            text-align: center;
            color: white;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }}
        
        #viz-container {{
            background: white;
            border-radius: 10px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            margin: 0 auto;
            max-width: 1400px;
            padding: 20px;
        }}
        
        #viz {{
            background: #fafafa;
            border: 1px solid #ddd;
            border-radius: 5px;
        }}
        
        .node {{
            cursor: move;
        }}
        
        .node rect {{
            stroke: #fff;
            stroke-width: 3px;
            rx: 10;
            ry: 10;
        }}
        
        .node text {{
            font-size: 12px;
            font-weight: bold;
            fill: white;
            text-anchor: middle;
            pointer-events: none;
        }}
        
        .link {{
            stroke: #999;
            stroke-opacity: 0.6;
            stroke-width: 2px;
            fill: none;
        }}
        
        .link-label {{
            font-size: 10px;
            fill: #666;
        }}
        
        marker {{
            fill: #999;
        }}
        
        #info {{
            position: fixed;
            top: 10px;
            right: 10px;
            background: white;
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
            max-width: 200px;
        }}
        
        #legend {{
            display: flex;
            justify-content: center;
            flex-wrap: wrap;
            margin-top: 20px;
            gap: 15px;
        }}
        
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 5px;
            font-size: 12px;
        }}
        
        .legend-color {{
            width: 20px;
            height: 20px;
            border-radius: 3px;
            border: 1px solid #ddd;
        }}
    </style>
</head>
<body>
    <h1>📊 Dataflow Visualization - Voice Chatbot</h1>
    
    <div id="info">
        <strong>Instructions:</strong><br>
        • Drag nodes to reposition<br>
        • Scroll to zoom<br>
        • Pan by dragging background<br>
        • Double-click to reset
    </div>
    
    <div id="viz-container">
        <svg id="viz" width="1350" height="850"></svg>
        
        <div id="legend">
            <div class="legend-item">
                <div class="legend-color" style="background: #F44336;"></div>
                <span>Input/Output</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #2196F3;"></div>
                <span>Speech Processing</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #FFEB3B;"></div>
                <span>ASR</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #4CAF50;"></div>
                <span>Controller</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #9C27B0;"></div>
                <span>LLM</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #00BCD4;"></div>
                <span>Text Processing</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #FF9800;"></div>
                <span>TTS</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #9E9E9E;"></div>
                <span>Logging</span>
            </div>
        </div>
    </div>
    
    <script src="https://d3js.org/d3.v7.min.js"></script>
    <script>
        // Data with fixed positions
        const data = {json.dumps(graph_data)};
        
        console.log("Loading graph with", data.nodes.length, "nodes and", data.links.length, "links");
        
        // Set up SVG
        const svg = d3.select("#viz");
        const width = +svg.attr("width");
        const height = +svg.attr("height");
        
        // Create container for zoom
        const g = svg.append("g");
        
        // Add white background for panning
        g.append("rect")
            .attr("width", width)
            .attr("height", height)
            .attr("fill", "transparent");
        
        // Set up zoom behavior
        const zoom = d3.zoom()
            .scaleExtent([0.5, 3])
            .on("zoom", (event) => {{
                g.attr("transform", event.transform);
            }});
        
        svg.call(zoom);
        
        // Reset on double click
        svg.on("dblclick.zoom", () => {{
            svg.transition().duration(750).call(
                zoom.transform,
                d3.zoomIdentity
            );
        }});
        
        // Create arrow markers
        svg.append("defs").append("marker")
            .attr("id", "arrowhead")
            .attr("viewBox", "-0 -5 10 10")
            .attr("refX", 50)
            .attr("refY", 0)
            .attr("orient", "auto")
            .attr("markerWidth", 8)
            .attr("markerHeight", 8)
            .append("path")
            .attr("d", "M0,-5L10,0L0,5")
            .attr("fill", "#999");
        
        // Draw links first (so they appear under nodes)
        const link = g.append("g")
            .selectAll("path")
            .data(data.links)
            .enter().append("path")
            .attr("class", "link")
            .attr("marker-end", "url(#arrowhead)")
            .attr("d", d => {{
                const source = data.nodes.find(n => n.id === d.source);
                const target = data.nodes.find(n => n.id === d.target);
                if (source && target) {{
                    // Create curved path
                    const dx = target.x - source.x;
                    const dy = target.y - source.y;
                    const dr = Math.sqrt(dx * dx + dy * dy) / 2;
                    return `M${{source.x}},${{source.y}}A${{dr}},${{dr}} 0 0,1 ${{target.x}},${{target.y}}`;
                }}
                return "";
            }});
        
        // Add link labels (optional - uncomment if needed)
        /*
        const linkLabel = g.append("g")
            .selectAll("text")
            .data(data.links)
            .enter().append("text")
            .attr("class", "link-label")
            .attr("x", d => {{
                const source = data.nodes.find(n => n.id === d.source);
                const target = data.nodes.find(n => n.id === d.target);
                return source && target ? (source.x + target.x) / 2 : 0;
            }})
            .attr("y", d => {{
                const source = data.nodes.find(n => n.id === d.source);
                const target = data.nodes.find(n => n.id === d.target);
                return source && target ? (source.y + target.y) / 2 : 0;
            }})
            .text(d => d.label);
        */
        
        // Create node groups
        const node = g.append("g")
            .selectAll("g")
            .data(data.nodes)
            .enter().append("g")
            .attr("class", "node")
            .attr("transform", d => `translate(${{d.x}},${{d.y}})`)
            .call(d3.drag()
                .on("start", dragstarted)
                .on("drag", dragged)
                .on("end", dragended));
        
        // Add rectangles to nodes
        node.append("rect")
            .attr("x", -60)
            .attr("y", -20)
            .attr("width", 120)
            .attr("height", 40)
            .attr("fill", d => d.color);
        
        // Add text to nodes
        node.append("text")
            .attr("dy", 5)
            .text(d => d.name);
        
        // Add tooltips
        node.append("title")
            .text(d => d.name);
        
        // Drag functions
        function dragstarted(event, d) {{
            d3.select(this).raise().attr("cursor", "grabbing");
        }}
        
        function dragged(event, d) {{
            d.x = event.x;
            d.y = event.y;
            d3.select(this).attr("transform", `translate(${{d.x}},${{d.y}})`);
            
            // Update connected links
            link.attr("d", linkData => {{
                const source = data.nodes.find(n => n.id === linkData.source);
                const target = data.nodes.find(n => n.id === linkData.target);
                if (source && target) {{
                    const dx = target.x - source.x;
                    const dy = target.y - source.y;
                    const dr = Math.sqrt(dx * dx + dy * dy) / 2;
                    return `M${{source.x}},${{source.y}}A${{dr}},${{dr}} 0 0,1 ${{target.x}},${{target.y}}`;
                }}
                return "";
            }});
        }}
        
        function dragended(event, d) {{
            d3.select(this).attr("cursor", "move");
        }}
        
        // Initial zoom to fit
        setTimeout(() => {{
            const bounds = g.node().getBBox();
            const fullWidth = width;
            const fullHeight = height;
            const width0 = bounds.width;
            const height0 = bounds.height;
            const midX = bounds.x + width0 / 2;
            const midY = bounds.y + height0 / 2;
            
            if (width0 > 0 && height0 > 0) {{
                const scale = 0.95 / Math.max(width0 / fullWidth, height0 / fullHeight);
                const translate = [fullWidth / 2 - scale * midX, fullHeight / 2 - scale * midY];
                
                svg.call(
                    zoom.transform,
                    d3.zoomIdentity.translate(translate[0], translate[1]).scale(scale)
                );
            }}
        }}, 100);
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
    
    print(f"Parsing {yaml_file}...")
    graph_data = parse_yaml(yaml_file)
    
    print(f"Found {len(graph_data['nodes'])} nodes and {len(graph_data['links'])} connections")
    print("Nodes are positioned in a vertical flow layout")
    
    html_content = generate_html(graph_data)
    
    output_file = "dataflow_fixed_layout.html"
    with open(output_file, 'w') as f:
        f.write(html_content)
    
    print(f"✅ Visualization saved to: {output_file}")
    print(f"📂 Open this file in your browser: {Path(output_file).absolute()}")
    print("\nTo open: ")
    print(f"  open {output_file}")

if __name__ == "__main__":
    main()