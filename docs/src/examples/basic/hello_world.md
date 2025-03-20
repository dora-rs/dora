# Hello World Example

This is a simple example that demonstrates the basic concepts of Dora.

## Overview
This example shows how to:
1. Create a basic Dora application
2. Define a simple node
3. Set up data flow between nodes
4. Run the application

## Project Structure
```
hello_world/
├── dora.yml
└── nodes/
    ├── source.py
    └── sink.py
```

## Implementation

### 1. Create the Project
```bash
dora new hello_world
cd hello_world
```

### 2. Create Source Node (nodes/source.py)
```python
from dora import Node

class SourceNode(Node):
    def __init__(self):
        super().__init__()
        self.counter = 0
        
    def on_input(self, input_data):
        self.counter += 1
        return {"message": f"Hello World {self.counter}!"}
```

### 3. Create Sink Node (nodes/sink.py)
```python
from dora import Node

class SinkNode(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        print(f"Received: {input_data['message']}")
        return None
```

### 4. Configure the Graph (dora.yml)
```yaml
nodes:
  source:
    inputs: []
    outputs: ["message"]
    python: nodes/source.py
    
  sink:
    inputs: ["message"]
    outputs: []
    python: nodes/sink.py

edges:
  - from: source
    to: sink
    data: message
```

## Running the Example

1. Start the application:
```bash
dora run
```

2. You should see output like:
```
Received: Hello World 1!
Received: Hello World 2!
Received: Hello World 3!
...
```

## Understanding the Example

### Data Flow
1. The `SourceNode` generates messages with a counter
2. Messages flow through the edge to the `SinkNode`
3. The `SinkNode` prints the received messages

### Key Concepts
- **Nodes**: Independent processing units
- **Edges**: Define data flow between nodes
- **Data Types**: Messages are passed as dictionaries
- **Node Lifecycle**: Nodes are initialized and process data continuously

## Next Steps
- Try modifying the message format
- Add more nodes to the graph
- Experiment with different data types
- Check out the [Advanced Examples](../advanced/README.md) for more complex scenarios 