#!/usr/bin/env python3
"""
Dataflow YAML Visualizer with ASCII Art
Parses Dora dataflow YAML files and creates ASCII art diagrams showing node connections.
"""

import yaml
import sys
from pathlib import Path
from collections import defaultdict, OrderedDict
from typing import Dict, List, Tuple, Set

class DataflowVisualizer:
    def __init__(self):
        self.nodes = {}
        self.connections = defaultdict(list)
        self.node_positions = {}
        self.layers = []
        
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
                'build': node.get('build', '')
            }
            
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
    
    def detect_layers(self):
        """Organize nodes into layers based on dependencies."""
        # Find nodes with no inputs (source nodes)
        source_nodes = []
        for node_id, node_data in self.nodes.items():
            if not node_data['inputs']:
                source_nodes.append(node_id)
        
        # Build layers using BFS
        visited = set()
        layers = []
        current_layer = source_nodes
        
        while current_layer:
            layers.append(current_layer)
            visited.update(current_layer)
            
            next_layer = []
            for node in current_layer:
                for target, _, _ in self.connections[node]:
                    if target not in visited and target not in next_layer:
                        # Check if all dependencies are in visited
                        deps_satisfied = True
                        for input_name, (source_node, _) in self.nodes[target]['inputs'].items():
                            if source_node not in visited:
                                deps_satisfied = False
                                break
                        
                        if deps_satisfied:
                            next_layer.append(target)
            
            current_layer = next_layer
        
        # Add any remaining nodes
        remaining = set(self.nodes.keys()) - visited
        if remaining:
            layers.append(list(remaining))
        
        self.layers = layers
    
    def create_ascii_diagram(self):
        """Create ASCII art diagram of the dataflow."""
        if not self.nodes:
            return "No nodes to visualize"
        
        diagram_lines = []
        
        # Header
        diagram_lines.append("=" * 80)
        diagram_lines.append("DATAFLOW VISUALIZATION - NODE GRAPH")
        diagram_lines.append("=" * 80)
        diagram_lines.append("")
        
        # Create a simplified vertical flow diagram
        # First, identify the main pipeline
        processed = set()
        
        # Start with microphone if it exists
        if 'microphone' in self.nodes:
            current = 'microphone'
            diagram_lines.append(f"┌{'─' * 30}┐")
            diagram_lines.append(f"│ {'🎤 MICROPHONE':^28} │")
            diagram_lines.append(f"└{'─' * 30}┘")
            diagram_lines.append("              ║")
            diagram_lines.append("              ▼")
            processed.add(current)
            
            # Follow the main path
            if 'speech-monitor' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'🔊 SPEECH-MONITOR':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add('speech-monitor')
            
            if 'asr' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'📝 ASR':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add('asr')
            
            if 'chat-controller' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'🎮 CHAT-CONTROLLER':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add('chat-controller')
            
            if 'qwen3-llm' in self.nodes or 'qwen3' in self.nodes:
                llm_name = 'qwen3-llm' if 'qwen3-llm' in self.nodes else 'qwen3'
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'🤖 LLM (QWEN3)':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add(llm_name)
            
            if 'text-segmenter' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'✂️ TEXT-SEGMENTER':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add('text-segmenter')
            
            if 'primespeech' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'🗣️ PRIMESPEECH (TTS)':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                diagram_lines.append("              ║")
                diagram_lines.append("              ▼")
                processed.add('primespeech')
            
            if 'audio-player' in self.nodes:
                diagram_lines.append(f"┌{'─' * 30}┐")
                diagram_lines.append(f"│ {'🔈 AUDIO-PLAYER':^28} │")
                diagram_lines.append(f"└{'─' * 30}┘")
                processed.add('audio-player')
        
        # Add auxiliary nodes
        auxiliary = []
        for node_id in self.nodes:
            if node_id not in processed:
                auxiliary.append(node_id)
        
        if auxiliary:
            diagram_lines.append("")
            diagram_lines.append("AUXILIARY NODES:")
            diagram_lines.append("─" * 40)
            for node in auxiliary:
                diagram_lines.append(f"  • {node}")
        
        # Add connection details
        diagram_lines.append("")
        diagram_lines.append("=" * 80)
        diagram_lines.append("CONNECTION DETAILS:")
        diagram_lines.append("─" * 80)
        
        # Group connections by source
        for source_node in sorted(self.nodes.keys()):
            connections = self.connections[source_node]
            if connections:
                diagram_lines.append(f"\n{source_node}:")
                for target, output_name, input_name in connections:
                    diagram_lines.append(f"  └─ {output_name} → {target}/{input_name}")
        
        return "\n".join(diagram_lines)
    
    def create_detailed_diagram(self):
        """Create a detailed flow diagram with all inputs/outputs."""
        lines = []
        lines.append("=" * 100)
        lines.append("DETAILED DATAFLOW DIAGRAM")
        lines.append("=" * 100)
        lines.append("")
        
        # Group nodes by category
        input_nodes = []
        processing_nodes = []
        output_nodes = []
        
        for node_id, node_data in self.nodes.items():
            if not node_data['inputs']:
                input_nodes.append(node_id)
            elif not self.connections[node_id]:
                output_nodes.append(node_id)
            else:
                processing_nodes.append(node_id)
        
        # Draw input layer
        if input_nodes:
            lines.append("INPUT LAYER:")
            lines.append("┌" + "─" * 98 + "┐")
            for node in input_nodes:
                outputs = self.nodes[node]['outputs']
                output_str = ", ".join(outputs) if outputs else "none"
                lines.append(f"│ [{node:20}] outputs: {output_str:73} │")
            lines.append("└" + "─" * 98 + "┘")
            lines.append("          ║")
            lines.append("          ▼")
        
        # Draw processing layer
        if processing_nodes:
            lines.append("PROCESSING LAYER:")
            lines.append("┌" + "─" * 98 + "┐")
            for node in processing_nodes:
                # Show inputs
                inputs = self.nodes[node]['inputs']
                input_str = ", ".join([f"{k}←{v[0]}/{v[1]}" for k, v in inputs.items()])[:40]
                
                # Show outputs
                outputs = self.nodes[node]['outputs']
                output_str = ", ".join(outputs) if outputs else "none"
                
                lines.append(f"│ [{node:20}] {input_str:35} → {output_str:35} │")
            lines.append("└" + "─" * 98 + "┘")
            
            if output_nodes:
                lines.append("          ║")
                lines.append("          ▼")
        
        # Draw output layer
        if output_nodes:
            lines.append("OUTPUT LAYER:")
            lines.append("┌" + "─" * 98 + "┐")
            for node in output_nodes:
                inputs = self.nodes[node]['inputs']
                input_str = ", ".join([f"{k}←{v[0]}/{v[1]}" for k, v in inputs.items()])[:70]
                lines.append(f"│ [{node:20}] inputs: {input_str:73} │")
            lines.append("└" + "─" * 98 + "┘")
        
        return "\n".join(lines)
    
    def create_conversation_flow(self):
        """Create a specialized view showing the conversation flow."""
        lines = []
        lines.append("=" * 100)
        lines.append("TWO-WAY CONVERSATION FLOW WITH LLM")
        lines.append("=" * 100)
        lines.append("")
        
        # User to LLM flow
        lines.append("USER → LLM FLOW:")
        lines.append("─" * 50)
        lines.append("")
        lines.append("  🎤 [microphone]")
        lines.append("       ↓ audio")
        lines.append("  🔊 [speech-monitor] ← control (pause/resume)")
        lines.append("       ↓ audio_segment")
        lines.append("  📝 [asr] (Speech-to-Text)")
        lines.append("       ↓ transcription")
        lines.append("  🎮 [chat-controller] (Orchestrator)")
        lines.append("       ↓ question_to_llm")
        lines.append("  🤖 [qwen3-llm] (AI Processing)")
        lines.append("       ↓ text")
        lines.append("")
        
        # LLM to User flow
        lines.append("LLM → USER FLOW:")
        lines.append("─" * 50)
        lines.append("")
        lines.append("  🤖 [qwen3-llm]")
        lines.append("       ↓ text")
        lines.append("  ✂️  [text-segmenter] ← backpressure control")
        lines.append("       ↓ text_segment")
        lines.append("  🗣️  [primespeech] (Text-to-Speech)")
        lines.append("       ↓ audio")
        lines.append("  🔈 [audio-player] → buffer_status")
        lines.append("       ↓")
        lines.append("  👂 User hears response")
        lines.append("")
        
        # Control flow
        lines.append("CONTROL MECHANISMS:")
        lines.append("─" * 50)
        lines.append("")
        lines.append("1. PAUSE/RESUME CONTROL:")
        lines.append("   [chat-controller] → speech_control → [speech-monitor]")
        lines.append("   • Pauses microphone during TTS playback")
        lines.append("   • Resumes when buffer is nearly empty")
        lines.append("")
        lines.append("2. BACKPRESSURE CONTROL:")
        lines.append("   [audio-player] → buffer_status → [conversation-controller]")
        lines.append("                                  → segment_control → [text-segmenter]")
        lines.append("   • Prevents audio buffer overflow")
        lines.append("   • Pauses text generation when buffer > 70%")
        lines.append("   • Resumes when buffer < 50%")
        lines.append("")
        lines.append("3. QUESTION DETECTION:")
        lines.append("   [speech-monitor] → question_ended → [chat-controller]")
        lines.append("   • Detects 3 seconds of silence after speech")
        lines.append("   • Triggers LLM processing")
        lines.append("")
        
        # Key features
        lines.append("KEY FEATURES:")
        lines.append("─" * 50)
        lines.append("• Full-duplex conversation (with pause/resume)")
        lines.append("• Automatic language detection (Chinese/English)")
        lines.append("• Streaming TTS with backpressure control")
        lines.append("• Conversation history management")
        lines.append("• Buffer-based flow control")
        lines.append("")
        
        return "\n".join(lines)

def main():
    if len(sys.argv) < 2:
        # Default to voice_chatbot.yml
        yaml_file = "../voice-chatbot/voice_chatbot.yml"
        if not Path(yaml_file).exists():
            print("Usage: python dataflow_visualizer.py <dataflow.yml>")
            sys.exit(1)
    else:
        yaml_file = sys.argv[1]
    
    if not Path(yaml_file).exists():
        print(f"Error: File '{yaml_file}' not found")
        sys.exit(1)
    
    visualizer = DataflowVisualizer()
    visualizer.parse_yaml(yaml_file)
    visualizer.detect_layers()
    
    print("\n" + "=" * 100)
    print(f"VISUALIZING: {yaml_file}")
    print("=" * 100 + "\n")
    
    # Show simplified diagram
    print(visualizer.create_ascii_diagram())
    print("\n")
    
    # Show detailed diagram
    print(visualizer.create_detailed_diagram())
    print("\n")
    
    # Show conversation flow
    print(visualizer.create_conversation_flow())

if __name__ == "__main__":
    main()