#!/usr/bin/env python3
"""
Simple HTML Dataflow Visualizer with inline D3.js
Creates a self-contained HTML file with the visualization.
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
    links = []
    
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
        
        nodes.append({
            'id': node_id,
            'name': node_id,
            'color': color,
            'group': idx
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
                        'value': 1
                    })
    
    return {'nodes': nodes, 'links': links}

def generate_html(graph_data):
    """Generate a simple HTML file with D3.js visualization."""
    
    html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Dataflow Visualization</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: #f0f0f0;
        }}
        
        h1 {{
            text-align: center;
            color: #333;
        }}
        
        #viz {{
            background: white;
            border: 1px solid #ddd;
            border-radius: 5px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        
        .node {{
            stroke: #fff;
            stroke-width: 2px;
            cursor: pointer;
        }}
        
        .node:hover {{
            stroke: #000;
            stroke-width: 3px;
        }}
        
        .link {{
            stroke: #999;
            stroke-opacity: 0.6;
            stroke-width: 2px;
        }}
        
        .node-label {{
            font-size: 12px;
            pointer-events: none;
        }}
        
        #info {{
            position: fixed;
            top: 10px;
            right: 10px;
            background: white;
            padding: 10px;
            border: 1px solid #ddd;
            border-radius: 5px;
            max-width: 200px;
        }}
    </style>
</head>
<body>
    <h1>Dataflow Visualization</h1>
    <div id="info">
        <strong>Instructions:</strong><br>
        • Drag nodes to move<br>
        • Scroll to zoom<br>
        • Click to fix position
    </div>
    <svg id="viz" width="1400" height="900"></svg>
    
    <script src="https://d3js.org/d3.v7.min.js"></script>
    <script>
        // Data
        const data = {json.dumps(graph_data)};
        
        // Set up SVG
        const svg = d3.select("#viz");
        const width = +svg.attr("width");
        const height = +svg.attr("height");
        
        // Create a group for zooming
        const g = svg.append("g");
        
        // Set up zoom
        const zoom = d3.zoom()
            .scaleExtent([0.1, 4])
            .on("zoom", (event) => {{
                g.attr("transform", event.transform);
            }});
        
        svg.call(zoom);
        
        // Create arrow markers
        svg.append("defs").selectAll("marker")
            .data(["end"])
            .enter().append("marker")
            .attr("id", String)
            .attr("viewBox", "0 -5 10 10")
            .attr("refX", 15)
            .attr("refY", -1.5)
            .attr("markerWidth", 6)
            .attr("markerHeight", 6)
            .attr("orient", "auto")
            .append("path")
            .attr("d", "M0,-5L10,0L0,5")
            .attr("fill", "#999");
        
        // Create force simulation with better spacing
        const simulation = d3.forceSimulation()
            .force("link", d3.forceLink().id(d => d.id).distance(250))
            .force("charge", d3.forceManyBody().strength(-2000))
            .force("center", d3.forceCenter(width / 2, height / 2))
            .force("x", d3.forceX(width / 2).strength(0.05))
            .force("y", d3.forceY(height / 2).strength(0.05))
            .force("collision", d3.forceCollide().radius(80));
        
        // Create links
        const link = g.append("g")
            .attr("class", "links")
            .selectAll("line")
            .data(data.links)
            .enter().append("line")
            .attr("class", "link")
            .attr("marker-end", "url(#end)");
        
        // Create nodes
        const node = g.append("g")
            .attr("class", "nodes")
            .selectAll("g")
            .data(data.nodes)
            .enter().append("g")
            .call(d3.drag()
                .on("start", dragstarted)
                .on("drag", dragged)
                .on("end", dragended));
        
        // Add circles to nodes (larger for better visibility)
        node.append("circle")
            .attr("class", "node")
            .attr("r", 30)
            .attr("fill", d => d.color);
        
        // Add labels to nodes
        node.append("text")
            .attr("class", "node-label")
            .attr("dx", 35)
            .attr("dy", 5)
            .text(d => d.name);
        
        // Add title for hover
        node.append("title")
            .text(d => d.name);
        
        // Set up simulation
        simulation
            .nodes(data.nodes)
            .on("tick", ticked);
        
        simulation.force("link")
            .links(data.links);
        
        // Update positions
        function ticked() {{
            link
                .attr("x1", d => d.source.x)
                .attr("y1", d => d.source.y)
                .attr("x2", d => d.target.x)
                .attr("y2", d => d.target.y);
            
            node
                .attr("transform", d => "translate(" + d.x + "," + d.y + ")");
        }}
        
        // Drag functions
        function dragstarted(event, d) {{
            if (!event.active) simulation.alphaTarget(0.3).restart();
            d.fx = d.x;
            d.fy = d.y;
        }}
        
        function dragged(event, d) {{
            d.fx = event.x;
            d.fy = event.y;
        }}
        
        function dragended(event, d) {{
            if (!event.active) simulation.alphaTarget(0);
            if (!event.sourceEvent.shiftKey) {{
                d.fx = null;
                d.fy = null;
            }}
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
                const scale = 0.9 / Math.max(width0 / fullWidth, height0 / fullHeight);
                const translate = [fullWidth / 2 - scale * midX, fullHeight / 2 - scale * midY];
                
                svg.transition().duration(750).call(
                    zoom.transform,
                    d3.zoomIdentity.translate(translate[0], translate[1]).scale(scale)
                );
            }}
        }}, 500);
        
        console.log("Graph loaded with", data.nodes.length, "nodes and", data.links.length, "links");
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
    
    html_content = generate_html(graph_data)
    
    output_file = Path(yaml_file).stem + "_simple_viz.html"
    with open(output_file, 'w') as f:
        f.write(html_content)
    
    print(f"✅ Visualization saved to: {output_file}")
    print(f"📂 Open this file in your browser: {Path(output_file).absolute()}")
    print("\nTo open: ")
    print(f"  open {output_file}")

if __name__ == "__main__":
    main()