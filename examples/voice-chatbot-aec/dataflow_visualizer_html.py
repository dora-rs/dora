#!/usr/bin/env python3
"""
HTML Dataflow Visualizer using D3.js
Creates an interactive, beautiful web-based visualization of Dora dataflow YAML files.
"""

import yaml
import json
import sys
import webbrowser
import tempfile
import os
from pathlib import Path
from collections import defaultdict

class HTMLDataflowVisualizer:
    def __init__(self):
        self.nodes = {}
        self.links = []
        self.node_positions = {}
        
        # Node categories and styling
        self.node_categories = {
            'input': {'color': '#ff6b6b', 'icon': '🎤', 'shape': 'rect'},
            'speech': {'color': '#4ecdc4', 'icon': '🔊', 'shape': 'rect'},
            'asr': {'color': '#ffe66d', 'icon': '📝', 'shape': 'rect'},
            'controller': {'color': '#95e77e', 'icon': '🎮', 'shape': 'rect'},
            'llm': {'color': '#a8e6cf', 'icon': '🤖', 'shape': 'rect'},
            'segmenter': {'color': '#7eb3ff', 'icon': '✂️', 'shape': 'rect'},
            'tts': {'color': '#ffb347', 'icon': '🗣️', 'shape': 'rect'},
            'player': {'color': '#ff6b6b', 'icon': '🔈', 'shape': 'rect'},
            'display': {'color': '#d3d3d3', 'icon': '📋', 'shape': 'rect'},
            'default': {'color': '#b19cd9', 'icon': '📦', 'shape': 'rect'}
        }
    
    def get_node_category(self, node_id):
        """Determine node category based on ID."""
        node_lower = node_id.lower()
        if 'microphone' in node_lower:
            return 'input'
        elif 'speech' in node_lower:
            return 'speech'
        elif 'asr' in node_lower:
            return 'asr'
        elif 'controller' in node_lower:
            return 'controller'
        elif 'llm' in node_lower or 'qwen' in node_lower:
            return 'llm'
        elif 'segment' in node_lower:
            return 'segmenter'
        elif 'tts' in node_lower or 'primespeech' in node_lower:
            return 'tts'
        elif 'player' in node_lower:
            return 'player'
        elif 'display' in node_lower or 'log' in node_lower:
            return 'display'
        else:
            return 'default'
    
    def parse_yaml(self, yaml_file):
        """Parse the dataflow YAML file."""
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        # Parse nodes
        for idx, node in enumerate(data.get('nodes', [])):
            node_id = node['id']
            category = self.get_node_category(node_id)
            
            self.nodes[node_id] = {
                'id': node_id,
                'label': node_id.replace('-', ' ').title(),
                'category': category,
                'color': self.node_categories[category]['color'],
                'icon': self.node_categories[category]['icon'],
                'shape': self.node_categories[category]['shape'],
                'inputs': [],
                'outputs': node.get('outputs', []),
                'env': node.get('env', {}),
                'index': idx
            }
            
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
                        
                        # Store input information
                        self.nodes[node_id]['inputs'].append({
                            'name': input_name,
                            'from': source_node,
                            'output': source_output
                        })
                        
                        # Create link
                        self.links.append({
                            'source': source_node,
                            'target': node_id,
                            'sourceOutput': source_output,
                            'targetInput': input_name,
                            'label': f"{source_output} → {input_name}"
                        })
    
    def generate_html(self):
        """Generate the HTML with D3.js visualization."""
        
        # Convert nodes dict to list for D3
        nodes_list = list(self.nodes.values())
        
        # Prepare data for JavaScript
        graph_data = {
            'nodes': nodes_list,
            'links': self.links
        }
        
        html_content = f"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dataflow Visualization</title>
    <script src="https://d3js.org/d3.v7.min.js"></script>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 0;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            overflow: hidden;
        }}
        
        #container {{
            width: 100vw;
            height: 100vh;
            position: relative;
        }}
        
        #header {{
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            background: rgba(255, 255, 255, 0.95);
            padding: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            z-index: 1000;
        }}
        
        h1 {{
            margin: 0;
            color: #333;
            font-size: 24px;
        }}
        
        .subtitle {{
            color: #666;
            font-size: 14px;
            margin-top: 5px;
        }}
        
        #graph {{
            width: 100%;
            height: 100%;
        }}
        
        .node {{
            cursor: pointer;
            filter: drop-shadow(0px 3px 3px rgba(0,0,0,0.2));
            transition: all 0.3s ease;
        }}
        
        .node:hover {{
            filter: drop-shadow(0px 5px 5px rgba(0,0,0,0.3));
            transform: scale(1.05);
        }}
        
        .node rect {{
            stroke-width: 2;
            stroke: #fff;
            rx: 10;
            ry: 10;
        }}
        
        .node text {{
            font-size: 12px;
            font-weight: 600;
            fill: #fff;
            text-anchor: middle;
            dominant-baseline: middle;
            pointer-events: none;
            text-shadow: 0 1px 2px rgba(0,0,0,0.3);
        }}
        
        .link {{
            fill: none;
            stroke: #999;
            stroke-opacity: 0.6;
            stroke-width: 2;
            marker-end: url(#arrowhead);
        }}
        
        .link:hover {{
            stroke-opacity: 1;
            stroke-width: 3;
        }}
        
        .link-label {{
            font-size: 10px;
            fill: #666;
            text-anchor: middle;
            dominant-baseline: middle;
            background: white;
            padding: 2px 4px;
            border-radius: 3px;
        }}
        
        #tooltip {{
            position: absolute;
            padding: 10px;
            background: rgba(0, 0, 0, 0.8);
            color: white;
            border-radius: 5px;
            pointer-events: none;
            opacity: 0;
            transition: opacity 0.3s;
            font-size: 12px;
            max-width: 300px;
            z-index: 2000;
        }}
        
        #controls {{
            position: absolute;
            bottom: 20px;
            left: 20px;
            background: rgba(255, 255, 255, 0.95);
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            z-index: 1000;
        }}
        
        button {{
            background: #667eea;
            color: white;
            border: none;
            padding: 8px 16px;
            margin: 0 5px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 14px;
            transition: background 0.3s;
        }}
        
        button:hover {{
            background: #5a67d8;
        }}
        
        #legend {{
            position: absolute;
            top: 80px;
            right: 20px;
            background: rgba(255, 255, 255, 0.95);
            padding: 15px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            z-index: 1000;
        }}
        
        .legend-item {{
            display: flex;
            align-items: center;
            margin: 5px 0;
            font-size: 12px;
        }}
        
        .legend-color {{
            width: 20px;
            height: 20px;
            margin-right: 10px;
            border-radius: 3px;
            border: 1px solid #fff;
        }}
    </style>
</head>
<body>
    <div id="container">
        <div id="header">
            <h1>🔄 Dataflow Visualization</h1>
            <div class="subtitle">Interactive Node Graph powered by D3.js</div>
        </div>
        
        <svg id="graph"></svg>
        
        <div id="tooltip"></div>
        
        <div id="controls">
            <button onclick="resetZoom()">Reset View</button>
            <button onclick="centerGraph()">Center</button>
            <button onclick="toggleLabels()">Toggle Labels</button>
        </div>
        
        <div id="legend">
            <strong>Node Types</strong>
            <div class="legend-item">
                <div class="legend-color" style="background: #ff6b6b;"></div>
                <span>🎤 Input</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #4ecdc4;"></div>
                <span>🔊 Speech Processing</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #ffe66d;"></div>
                <span>📝 ASR</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #95e77e;"></div>
                <span>🎮 Controller</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #a8e6cf;"></div>
                <span>🤖 LLM</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #7eb3ff;"></div>
                <span>✂️ Text Segmenter</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #ffb347;"></div>
                <span>🗣️ TTS</span>
            </div>
            <div class="legend-item">
                <div class="legend-color" style="background: #ff6b6b;"></div>
                <span>🔈 Audio Player</span>
            </div>
        </div>
    </div>
    
    <script>
        // Graph data
        const graphData = {json.dumps(graph_data)};
        
        // SVG setup
        const svg = d3.select("#graph");
        const width = window.innerWidth;
        const height = window.innerHeight;
        
        svg.attr("width", width).attr("height", height);
        
        // Create container for zoom
        const container = svg.append("g");
        
        // Define arrow marker
        svg.append("defs").append("marker")
            .attr("id", "arrowhead")
            .attr("viewBox", "-0 -5 10 10")
            .attr("refX", 30)
            .attr("refY", 0)
            .attr("orient", "auto")
            .attr("markerWidth", 8)
            .attr("markerHeight", 8)
            .append("svg:path")
            .attr("d", "M 0,-5 L 10,0 L 0,5")
            .attr("fill", "#999");
        
        // Create force simulation
        const simulation = d3.forceSimulation(graphData.nodes)
            .force("link", d3.forceLink(graphData.links)
                .id(d => d.id)
                .distance(200))
            .force("charge", d3.forceManyBody().strength(-500))
            .force("center", d3.forceCenter(width / 2, height / 2))
            .force("collision", d3.forceCollide().radius(60));
        
        // Create links
        const link = container.append("g")
            .selectAll("path")
            .data(graphData.links)
            .enter().append("path")
            .attr("class", "link")
            .attr("id", (d, i) => "link-" + i);
        
        // Create link labels
        const linkLabel = container.append("g")
            .selectAll("text")
            .data(graphData.links)
            .enter().append("text")
            .attr("class", "link-label")
            .style("display", "none")
            .append("textPath")
            .attr("href", (d, i) => "#link-" + i)
            .attr("startOffset", "50%")
            .text(d => d.label);
        
        // Create nodes
        const node = container.append("g")
            .selectAll(".node")
            .data(graphData.nodes)
            .enter().append("g")
            .attr("class", "node")
            .call(d3.drag()
                .on("start", dragStarted)
                .on("drag", dragged)
                .on("end", dragEnded));
        
        // Add rectangles to nodes
        node.append("rect")
            .attr("width", 120)
            .attr("height", 50)
            .attr("x", -60)
            .attr("y", -25)
            .attr("fill", d => d.color);
        
        // Add icons and text to nodes
        node.append("text")
            .attr("y", -5)
            .text(d => d.icon);
        
        node.append("text")
            .attr("y", 10)
            .style("font-size", "10px")
            .text(d => d.label);
        
        // Tooltip
        const tooltip = d3.select("#tooltip");
        
        node.on("mouseover", function(event, d) {{
            let tooltipContent = '<strong>' + d.icon + ' ' + d.label + '</strong><br/>';
            tooltipContent += '<em>Category: ' + d.category + '</em><br/>';
            
            if (d.outputs && d.outputs.length > 0) {{
                tooltipContent += '<br/><strong>Outputs:</strong><br/>';
                tooltipContent += d.outputs.map(o => '• ' + o).join('<br/>');
            }}
            
            if (d.inputs && d.inputs.length > 0) {{
                tooltipContent += '<br/><strong>Inputs:</strong><br/>';
                tooltipContent += d.inputs.map(i => '• ' + i.name + ' ← ' + i.from + '/' + i.output).join('<br/>');
            }}
            
            tooltip.html(tooltipContent)
                .style("left", (event.pageX + 10) + "px")
                .style("top", (event.pageY - 10) + "px")
                .style("opacity", 1);
        }})
        .on("mouseout", function() {{
            tooltip.style("opacity", 0);
        }});
        
        // Update positions on tick
        simulation.on("tick", () => {{
            link.attr("d", d => {{
                const dx = d.target.x - d.source.x;
                const dy = d.target.y - d.source.y;
                const dr = Math.sqrt(dx * dx + dy * dy) * 2;
                return 'M' + d.source.x + ',' + d.source.y + 'A' + dr + ',' + dr + ' 0 0,1 ' + d.target.x + ',' + d.target.y;
            }});
            
            node.attr("transform", d => 'translate(' + d.x + ',' + d.y + ')');
        }});
        
        // Zoom behavior
        const zoom = d3.zoom()
            .scaleExtent([0.1, 4])
            .on("zoom", (event) => {{
                container.attr("transform", event.transform);
            }});
        
        svg.call(zoom);
        
        // Control functions
        function resetZoom() {{
            svg.transition().duration(750).call(
                zoom.transform,
                d3.zoomIdentity
            );
        }}
        
        function centerGraph() {{
            const bounds = container.node().getBBox();
            const fullWidth = width;
            const fullHeight = height;
            const width0 = bounds.width;
            const height0 = bounds.height;
            const midX = bounds.x + width0 / 2;
            const midY = bounds.y + height0 / 2;
            
            const scale = 0.8 / Math.max(width0 / fullWidth, height0 / fullHeight);
            const translate = [fullWidth / 2 - scale * midX, fullHeight / 2 - scale * midY];
            
            svg.transition().duration(750).call(
                zoom.transform,
                d3.zoomIdentity.translate(translate[0], translate[1]).scale(scale)
            );
        }}
        
        let labelsVisible = false;
        function toggleLabels() {{
            labelsVisible = !labelsVisible;
            d3.selectAll(".link-label").style("display", labelsVisible ? "block" : "none");
        }}
        
        // Drag functions
        function dragStarted(event, d) {{
            if (!event.active) simulation.alphaTarget(0.3).restart();
            d.fx = d.x;
            d.fy = d.y;
        }}
        
        function dragged(event, d) {{
            d.fx = event.x;
            d.fy = event.y;
        }}
        
        function dragEnded(event, d) {{
            if (!event.active) simulation.alphaTarget(0);
            d.fx = null;
            d.fy = null;
        }}
        
        // Initial center
        setTimeout(centerGraph, 1000);
    </script>
</body>
</html>
"""
        return html_content
    
    def visualize(self, yaml_file, open_browser=True):
        """Generate and open the visualization."""
        self.parse_yaml(yaml_file)
        html_content = self.generate_html()
        
        # Save to current directory with meaningful name
        output_file = Path(yaml_file).stem + "_visualization.html"
        output_path = Path.cwd() / output_file
        
        with open(output_path, 'w') as f:
            f.write(html_content)
        
        print(f"✅ Visualization saved to: {output_path}")
        print(f"📂 File location: {output_path.absolute()}")
        
        if open_browser:
            webbrowser.open(f"file://{output_path.absolute()}")
            print("🌐 Attempting to open in browser...")
            print(f"\n💡 If browser didn't open, manually open: {output_path.absolute()}")
        
        return str(output_path)

def main():
    if len(sys.argv) < 2:
        yaml_file = "../voice-chatbot/voice_chatbot.yml"
        if not Path(yaml_file).exists():
            print("Usage: python dataflow_visualizer_html.py <dataflow.yml>")
            sys.exit(1)
    else:
        yaml_file = sys.argv[1]
    
    if not Path(yaml_file).exists():
        print(f"Error: File '{yaml_file}' not found")
        sys.exit(1)
    
    visualizer = HTMLDataflowVisualizer()
    visualizer.visualize(yaml_file)
    
    print("\n📊 Interactive features:")
    print("  • Drag nodes to rearrange")
    print("  • Scroll to zoom in/out")
    print("  • Hover over nodes for details")
    print("  • Click buttons to control view")
    print("  • Toggle labels to show/hide connection details")

if __name__ == "__main__":
    main()