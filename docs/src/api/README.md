# Dora-RS API Documentation

## Overview
This document provides comprehensive documentation for the Dora-RS API, including both Python and Rust interfaces.

## Python API

### Core Components

#### Node Class
```python
from dora import Node

class MyNode(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        # Process input data
        return processed_data
```

#### Data Types
```python
from dora import DataType

# Supported data types
class SupportedTypes:
    STRING = DataType.STRING
    INT = DataType.INT
    FLOAT = DataType.FLOAT
    BYTES = DataType.BYTES
    JSON = DataType.JSON
```

#### Configuration
```python
from dora import Config

config = Config(
    name="my_node",
    inputs=["input1", "input2"],
    outputs=["output1"],
    parameters={
        "param1": "value1",
        "param2": 42
    }
)
```

### Advanced Features

#### Async Support
```python
from dora import AsyncNode

class AsyncMyNode(AsyncNode):
    async def on_input(self, input_data):
        # Async processing
        return processed_data
```

#### Error Handling
```python
from dora import NodeError

class SafeNode(Node):
    def on_input(self, input_data):
        try:
            # Processing
            return result
        except Exception as e:
            raise NodeError(f"Processing failed: {str(e)}")
```

## Rust API

### Core Components

#### Node Trait
```rust
use dora_node_api::Node;

struct MyNode {
    // Node state
}

impl Node for MyNode {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Process input
        Ok(output)
    }
}
```

#### Data Types
```rust
use dora_node_api::DataType;

// Supported data types
enum SupportedTypes {
    String,
    Int,
    Float,
    Bytes,
    Json,
}
```

#### Configuration
```rust
use dora_node_api::Config;

let config = Config::new("my_node")
    .with_inputs(vec!["input1", "input2"])
    .with_outputs(vec!["output1"])
    .with_parameters(hashmap! {
        "param1".to_string() => "value1".to_string(),
        "param2".to_string() => "42".to_string(),
    });
```

### Advanced Features

#### Async Support
```rust
use dora_node_api::AsyncNode;

struct AsyncMyNode {
    // Node state
}

#[async_trait]
impl AsyncNode for AsyncMyNode {
    async fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Async processing
        Ok(output)
    }
}
```

#### Error Handling
```rust
use dora_node_api::{Node, NodeError};

struct SafeNode {
    // Node state
}

impl Node for SafeNode {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Processing
        Ok(output)
    }
}
```

## Common Patterns

### 1. Data Transformation
```python
# Python
def transform_data(input_data):
    # Transform data
    return transformed_data

# Rust
fn transform_data(input: Input) -> Result<Output, Error> {
    // Transform data
    Ok(transformed_data)
}
```

### 2. State Management
```python
# Python
class StatefulNode(Node):
    def __init__(self):
        super().__init__()
        self.state = {}
        
    def on_input(self, input_data):
        # Update state
        self.state.update(input_data)
        return processed_data

# Rust
struct StatefulNode {
    state: HashMap<String, Value>,
}

impl Node for StatefulNode {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Update state
        self.state.extend(input.iter());
        Ok(processed_data)
    }
}
```

### 3. Resource Management
```python
# Python
class ResourceNode(Node):
    def __init__(self):
        super().__init__()
        self.resource = None
        
    def setup(self):
        self.resource = acquire_resource()
        
    def cleanup(self):
        if self.resource:
            release_resource(self.resource)

# Rust
struct ResourceNode {
    resource: Option<Resource>,
}

impl Node for ResourceNode {
    fn setup(&mut self) -> Result<(), Error> {
        self.resource = Some(acquire_resource()?);
        Ok(())
    }
    
    fn cleanup(&mut self) {
        if let Some(resource) = self.resource.take() {
            release_resource(resource);
        }
    }
}
```

## Performance Optimization

### 1. Memory Management
```python
# Python
class OptimizedNode(Node):
    def __init__(self):
        super().__init__()
        self.buffer = bytearray(1024)  # Pre-allocate buffer
        
    def on_input(self, input_data):
        # Reuse buffer
        return processed_data

# Rust
struct OptimizedNode {
    buffer: Vec<u8>,
}

impl Node for OptimizedNode {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Reuse buffer
        Ok(processed_data)
    }
}
```

### 2. Batch Processing
```python
# Python
class BatchNode(Node):
    def on_input(self, input_data):
        # Process batch
        return processed_batch

# Rust
struct BatchNode;

impl Node for BatchNode {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Process batch
        Ok(processed_batch)
    }
}
```

## Best Practices

### 1. Error Handling
- Use specific error types
- Provide meaningful error messages
- Implement proper error recovery
- Log errors appropriately

### 2. Resource Management
- Clean up resources properly
- Use RAII patterns
- Implement proper shutdown
- Handle resource exhaustion

### 3. Performance
- Minimize allocations
- Use efficient data structures
- Implement proper buffering
- Optimize hot paths

### 4. Testing
- Write unit tests
- Implement integration tests
- Test error cases
- Benchmark performance 