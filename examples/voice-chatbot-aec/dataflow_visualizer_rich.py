#!/usr/bin/env python3
"""
Enhanced Dataflow YAML Visualizer with Beautiful ASCII Art
Uses rich library for beautiful terminal output and advanced formatting.
"""

import yaml
import sys
from pathlib import Path
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Set
import math

try:
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    from rich.columns import Columns
    from rich.tree import Tree
    from rich import box
    from rich.align import Align
    from rich.style import Style
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Installing rich library for beautiful visualization...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "rich"])
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
    from rich.columns import Columns
    from rich.tree import Tree
    from rich import box
    from rich.align import Align
    from rich.style import Style

class BeautifulDataflowVisualizer:
    def __init__(self):
        self.console = Console()
        self.nodes = {}
        self.connections = defaultdict(list)
        self.node_categories = {
            'input': [],
            'processing': [],
            'output': [],
            'control': [],
            'display': []
        }
        
        # Color scheme for different node types
        self.node_colors = {
            'microphone': 'bright_red',
            'speech-monitor': 'bright_cyan',
            'asr': 'bright_yellow',
            'chat-controller': 'bright_green',
            'llm': 'bright_magenta',
            'qwen': 'bright_magenta',
            'text-segmenter': 'bright_blue',
            'tts': 'orange1',
            'primespeech': 'orange1',
            'audio-player': 'bright_red',
            'controller': 'green',
            'display': 'bright_white',
            'log': 'dim white'
        }
        
        # Icons for node types
        self.node_icons = {
            'microphone': '🎤',
            'speech-monitor': '🔊',
            'asr': '📝',
            'chat-controller': '🎮',
            'llm': '🤖',
            'qwen': '🤖',
            'text-segmenter': '✂️',
            'primespeech': '🗣️',
            'audio-player': '🔈',
            'conversation-controller': '🔄',
            'log-display': '📋'
        }
    
    def get_node_color(self, node_id: str) -> str:
        """Get color for a node based on its type."""
        node_lower = node_id.lower()
        for key, color in self.node_colors.items():
            if key in node_lower:
                return color
        return 'white'
    
    def get_node_icon(self, node_id: str) -> str:
        """Get icon for a node based on its type."""
        node_lower = node_id.lower()
        for key, icon in self.node_icons.items():
            if key in node_lower:
                return icon
        return '📦'
    
    def parse_yaml(self, yaml_file: str):
        """Parse the dataflow YAML file."""
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        
        # Parse nodes
        for node in data.get('nodes', []):
            node_id = node['id']
            self.nodes[node_id] = {
                'inputs': {},
                'outputs': node.get('outputs', []),
                'path': node.get('path', ''),
                'build': node.get('build', ''),
                'env': node.get('env', {})
            }
            
            # Categorize nodes
            if not node.get('inputs'):
                self.node_categories['input'].append(node_id)
            elif 'display' in node_id.lower() or 'log' in node_id.lower():
                self.node_categories['display'].append(node_id)
            elif 'controller' in node_id.lower():
                self.node_categories['control'].append(node_id)
            elif not any(self.connections[node_id]):
                self.node_categories['output'].append(node_id)
            else:
                self.node_categories['processing'].append(node_id)
            
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
                        self.nodes[node_id]['inputs'][input_name] = (source_node, source_output)
                        self.connections[source_node].append((node_id, source_output, input_name))
    
    def create_flow_diagram(self):
        """Create a beautiful flow diagram using rich."""
        # Main title
        title = Panel.fit(
            "[bold bright_cyan]DATAFLOW VISUALIZATION[/bold bright_cyan]\n[dim]Beautiful ASCII Art Representation[/dim]",
            border_style="bright_blue",
            box=box.DOUBLE
        )
        self.console.print(title)
        self.console.print()
        
        # Create the main pipeline flow
        if 'microphone' in self.nodes:
            self.draw_main_pipeline()
        
        # Draw auxiliary components
        self.draw_auxiliary_components()
        
        # Draw connection matrix
        self.draw_connection_matrix()
    
    def draw_main_pipeline(self):
        """Draw the main conversation pipeline."""
        pipeline_nodes = [
            'microphone',
            'speech-monitor', 
            'asr',
            'chat-controller',
            'qwen3-llm',
            'text-segmenter',
            'primespeech',
            'audio-player'
        ]
        
        self.console.print(Panel("[bold]MAIN CONVERSATION PIPELINE[/bold]", style="bright_green", box=box.HEAVY))
        
        for i, node_id in enumerate(pipeline_nodes):
            if node_id not in self.nodes:
                # Try alternative names
                if node_id == 'qwen3-llm' and 'qwen3' in self.nodes:
                    node_id = 'qwen3'
                else:
                    continue
            
            # Get node info
            icon = self.get_node_icon(node_id)
            color = self.get_node_color(node_id)
            
            # Create node box
            node_info = self.nodes[node_id]
            
            # Build content
            content_lines = []
            content_lines.append(f"[bold]{icon} {node_id.upper()}[/bold]")
            
            if node_info['outputs']:
                outputs = ", ".join(node_info['outputs'][:3])
                if len(node_info['outputs']) > 3:
                    outputs += "..."
                content_lines.append(f"[dim]→ {outputs}[/dim]")
            
            # Create panel
            panel = Panel(
                "\n".join(content_lines),
                style=color,
                box=box.ROUNDED,
                width=50,
                padding=(0, 2)
            )
            
            # Center the panel
            self.console.print(Align.center(panel))
            
            # Draw connector
            if i < len(pipeline_nodes) - 1:
                connector = Text("║\n▼", style=f"bold {color}")
                self.console.print(Align.center(connector))
    
    def draw_auxiliary_components(self):
        """Draw auxiliary components in a grid."""
        auxiliary = []
        
        # Collect auxiliary nodes
        for node_id in self.nodes:
            if node_id not in ['microphone', 'speech-monitor', 'asr', 'chat-controller', 
                              'qwen3-llm', 'qwen3', 'text-segmenter', 'primespeech', 'audio-player']:
                auxiliary.append(node_id)
        
        if not auxiliary:
            return
        
        self.console.print()
        self.console.print(Panel("[bold]AUXILIARY COMPONENTS[/bold]", style="bright_yellow", box=box.HEAVY))
        
        # Create panels for auxiliary nodes
        panels = []
        for node_id in auxiliary:
            icon = self.get_node_icon(node_id)
            color = self.get_node_color(node_id)
            
            content = f"[{color}]{icon} {node_id}[/{color}]"
            
            # Add connection info
            connections = []
            for target, output, input_name in self.connections[node_id]:
                connections.append(f"→ {target}")
            
            if connections:
                content += f"\n[dim]{', '.join(connections[:2])}[/dim]"
            
            panel = Panel(content, box=box.SIMPLE, width=30)
            panels.append(panel)
        
        # Display in columns
        if panels:
            columns = Columns(panels, equal=True, expand=False)
            self.console.print(columns)
    
    def draw_connection_matrix(self):
        """Draw a connection matrix showing all connections."""
        self.console.print()
        self.console.print(Panel("[bold]CONNECTION MATRIX[/bold]", style="bright_blue", box=box.HEAVY))
        
        # Create table
        table = Table(box=box.ROUNDED, show_header=True, header_style="bold bright_cyan")
        table.add_column("Source Node", style="bright_yellow", width=20)
        table.add_column("Output", style="bright_green", width=20)
        table.add_column("→", width=3, justify="center")
        table.add_column("Target Node", style="bright_blue", width=20)
        table.add_column("Input", style="bright_magenta", width=20)
        
        # Add connections
        for source_node in sorted(self.nodes.keys()):
            for target, output_name, input_name in self.connections[source_node]:
                source_icon = self.get_node_icon(source_node)
                target_icon = self.get_node_icon(target)
                
                table.add_row(
                    f"{source_icon} {source_node}",
                    output_name,
                    "→",
                    f"{target_icon} {target}",
                    input_name
                )
        
        self.console.print(table)
    
    def create_tree_view(self):
        """Create a tree view of the dataflow."""
        tree = Tree("[bold bright_cyan]📊 DATAFLOW TREE[/bold bright_cyan]")
        
        # Input nodes
        if self.node_categories['input']:
            input_branch = tree.add("[bold bright_red]🔴 INPUT NODES[/bold bright_red]")
            for node in self.node_categories['input']:
                icon = self.get_node_icon(node)
                node_branch = input_branch.add(f"{icon} [bright_yellow]{node}[/bright_yellow]")
                
                # Add outputs
                for target, output, _ in self.connections[node]:
                    node_branch.add(f"→ {output} to {target}", style="dim")
        
        # Processing nodes
        if self.node_categories['processing']:
            proc_branch = tree.add("[bold bright_green]🟢 PROCESSING NODES[/bold bright_green]")
            for node in self.node_categories['processing']:
                icon = self.get_node_icon(node)
                color = self.get_node_color(node)
                node_branch = proc_branch.add(f"{icon} [{color}]{node}[/{color}]")
                
                # Add inputs and outputs
                node_info = self.nodes[node]
                for input_name, (source, output) in node_info['inputs'].items():
                    node_branch.add(f"← {input_name} from {source}/{output}", style="dim cyan")
                for target, output, _ in self.connections[node]:
                    node_branch.add(f"→ {output} to {target}", style="dim yellow")
        
        # Control nodes
        if self.node_categories['control']:
            ctrl_branch = tree.add("[bold bright_blue]🔵 CONTROL NODES[/bold bright_blue]")
            for node in self.node_categories['control']:
                icon = self.get_node_icon(node)
                ctrl_branch.add(f"{icon} [bright_blue]{node}[/bright_blue]")
        
        # Display nodes
        if self.node_categories['display']:
            disp_branch = tree.add("[bold bright_white]⚪ DISPLAY NODES[/bold bright_white]")
            for node in self.node_categories['display']:
                icon = self.get_node_icon(node)
                disp_branch.add(f"{icon} [bright_white]{node}[/bright_white]")
        
        self.console.print()
        self.console.print(tree)
    
    def create_stats_panel(self):
        """Create statistics panel."""
        stats = []
        stats.append(f"[bright_cyan]Total Nodes:[/bright_cyan] {len(self.nodes)}")
        stats.append(f"[bright_red]Input Nodes:[/bright_red] {len(self.node_categories['input'])}")
        stats.append(f"[bright_green]Processing Nodes:[/bright_green] {len(self.node_categories['processing'])}")
        stats.append(f"[bright_blue]Control Nodes:[/bright_blue] {len(self.node_categories['control'])}")
        stats.append(f"[bright_white]Display Nodes:[/bright_white] {len(self.node_categories['display'])}")
        
        total_connections = sum(len(conns) for conns in self.connections.values())
        stats.append(f"[bright_yellow]Total Connections:[/bright_yellow] {total_connections}")
        
        stats_panel = Panel(
            "\n".join(stats),
            title="[bold]📈 DATAFLOW STATISTICS[/bold]",
            border_style="bright_magenta",
            box=box.DOUBLE
        )
        
        self.console.print()
        self.console.print(stats_panel)

def main():
    if len(sys.argv) < 2:
        yaml_file = "../voice-chatbot/voice_chatbot.yml"
        if not Path(yaml_file).exists():
            print("Usage: python dataflow_visualizer_rich.py <dataflow.yml>")
            sys.exit(1)
    else:
        yaml_file = sys.argv[1]
    
    if not Path(yaml_file).exists():
        print(f"Error: File '{yaml_file}' not found")
        sys.exit(1)
    
    visualizer = BeautifulDataflowVisualizer()
    visualizer.parse_yaml(yaml_file)
    
    # Clear screen for better presentation
    visualizer.console.clear()
    
    # Create visualizations
    visualizer.create_flow_diagram()
    visualizer.create_tree_view()
    visualizer.create_stats_panel()
    
    # Footer
    visualizer.console.print()
    footer = Panel(
        f"[dim]Visualization of: {yaml_file}[/dim]",
        style="dim white",
        box=box.SIMPLE
    )
    visualizer.console.print(footer)

if __name__ == "__main__":
    main()