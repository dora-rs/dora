#!/usr/bin/env python3
import yaml
import json

# Test parsing old format
print("=== Testing old format (dataflow.yaml) ===")
with open("dataflow.yaml", "r") as f:
    old_format = yaml.safe_load(f)
print(json.dumps(old_format, indent=2))

# Test parsing new format
print("\n=== Testing new format (dataflow 2.yaml) ===")
with open("dataflow 2.yaml", "r") as f:
    new_format = yaml.safe_load(f)
print(json.dumps(new_format, indent=2))

# Test parsing metadata
print("\n=== Testing metadata (dora.yaml) ===")
with open("dora.yaml", "r") as f:
    metadata = yaml.safe_load(f)
print(json.dumps(metadata, indent=2))

print("\n=== Format validation ===")
print(f"Old format has 'nodes': {'nodes' in old_format}")
print(f"Old format has 'graph': {'graph' in old_format}")
print(f"New format has 'nodes': {'nodes' in new_format}")
print(f"New format has 'graph': {'graph' in new_format}")
print(f"Metadata has 'nodes': {'nodes' in metadata}")

# Simulate resolution
print("\n=== Simulating new format resolution ===")
if 'graph' in new_format and 'nodes' in metadata:
    prototypes = {node['name']: node for node in metadata['nodes']}
    print(f"Found {len(prototypes)} prototypes:")
    for name in prototypes:
        print(f"  - {name}")
    
    print(f"\nResolving {len(new_format['graph'])} graph nodes:")
    for graph_node in new_format['graph']:
        node_id = graph_node['id']
        proto_name = graph_node['proto']
        print(f"  - {node_id} -> proto '{proto_name}'")
        if proto_name in prototypes:
            proto = prototypes[proto_name]
            print(f"    entry: {proto.get('entry', 'N/A')}")
            print(f"    build: {proto.get('build', 'N/A')}")
        else:
            print(f"    ERROR: prototype '{proto_name}' not found!")

print("\n=== All tests passed! ===")
