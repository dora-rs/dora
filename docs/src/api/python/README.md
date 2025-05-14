# Python API Documentation

## Overview
The Dora Python API provides a high-level interface for creating and managing Dora nodes and dataflows.

[See the full Python API documentation here](python/README.md)

## Core Components

### Node Class
```python
from typing import Any, Dict, Optional
from dora import Node

class MyNode(Node):
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.counter = 0

    async def on_input(self, input_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process incoming data from other nodes.
        
        Args:
            input_data: Dictionary containing input data from connected nodes
            
        Returns:
            Optional[Dict[str, Any]]: Output data to be sent to connected nodes
        """
        self.counter += 1
        return {"count": self.counter}
```

### DataFlow Class
```python
from dora import DataFlow

# Create a new dataflow
flow = DataFlow()

# Add nodes
node1 = flow.add_node("counter", MyNode, {"initial_value": 0})
node2 = flow.add_node("processor", ProcessorNode, {"threshold": 100})

# Connect nodes
flow.connect("counter.output", "processor.input")

# Run the dataflow
flow.run()
```

## Type Hints and Return Values

### Common Types
```python
from typing import TypedDict, List, Union

class NodeConfig(TypedDict):
    name: str
    type: str
    parameters: Dict[str, Any]

class NodeOutput(TypedDict):
    data: Union[Dict[str, Any], List[Any]]
    metadata: Dict[str, Any]
```

### Node Methods
```python
class Node:
    async def on_input(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Process input data and return output."""
        pass

    async def on_start(self) -> None:
        """Called when the node starts."""
        pass

    async def on_stop(self) -> None:
        """Called when the node stops."""
        pass
```

## Performance Considerations

### Memory Management
- Use streaming for large datasets
- Implement proper cleanup in `on_stop`
- Monitor memory usage with built-in metrics

### Async Operations
- Use `async/await` for I/O operations
- Implement proper error handling
- Use connection pooling for external services

## Best Practices

### Error Handling
```python
class MyNode(Node):
    async def on_input(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        try:
            result = await self.process_data(data)
            return result
        except ValueError as e:
            self.logger.error(f"Invalid input: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            raise
```

### Logging
```python
class MyNode(Node):
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.logger.info("Node initialized")
        self.logger.debug(f"Config: {config}")
```

### Configuration
```python
class MyNode(Node):
    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.threshold = config.get("threshold", 100)
        self.timeout = config.get("timeout", 30)
```

## Examples

### Basic Node
```python
from dora import Node
from typing import Dict, Any, Optional

class EchoNode(Node):
    async def on_input(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        return {"echo": data}
```

### Data Transformation
```python
class TransformNode(Node):
    async def on_input(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        transformed = {
            "timestamp": data.get("time"),
            "value": float(data.get("value", 0)),
            "metadata": data.get("metadata", {})
        }
        return transformed
```

### External Service Integration
```python
import aiohttp

class APINode(Node):
    async def on_input(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        async with aiohttp.ClientSession() as session:
            async with session.get(self.config["api_url"]) as response:
                result = await response.json()
                return {"api_response": result}
``` 