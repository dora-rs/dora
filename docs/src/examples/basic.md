# Basic Examples

This guide provides a collection of basic examples to help you get started with Dora-RS.

## Simple Data Processing Node

### Python Example
```python
from dora import Node, Config

class SimpleProcessor(Node):
    def __init__(self):
        super().__init__()
        self.counter = 0
        
    def on_input(self, input_data):
        self.counter += 1
        return {
            "count": self.counter,
            "processed_data": input_data
        }

# Configuration
config = Config(
    name="simple_processor",
    inputs=["input_stream"],
    outputs=["output_stream"]
)

# Run the node
node = SimpleProcessor()
node.run(config)
```

### Rust Example
```rust
use dora_node_api::{Node, Config};

struct SimpleProcessor {
    counter: u64,
}

impl Node for SimpleProcessor {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        self.counter += 1;
        Ok(Output::new()
            .with("count", self.counter)
            .with("processed_data", input))
    }
}

// Configuration
let config = Config::new("simple_processor")
    .with_inputs(vec!["input_stream"])
    .with_outputs(vec!["output_stream"]);

// Run the node
let mut node = SimpleProcessor { counter: 0 };
node.run(config)?;
```

## Data Transformation Node

### Python Example
```python
from dora import Node, DataType

class DataTransformer(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        # Transform data
        transformed = {
            "uppercase": input_data["text"].upper(),
            "length": len(input_data["text"]),
            "words": len(input_data["text"].split())
        }
        return transformed

# Configuration
config = Config(
    name="data_transformer",
    inputs=["text_input"],
    outputs=["transformed_output"],
    input_types={
        "text_input": DataType.JSON
    }
)

# Run the node
node = DataTransformer()
node.run(config)
```

### Rust Example
```rust
use dora_node_api::{Node, Config, DataType};

struct DataTransformer;

impl Node for DataTransformer {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        let text = input.get::<String>("text")?;
        
        let transformed = json!({
            "uppercase": text.to_uppercase(),
            "length": text.len(),
            "words": text.split_whitespace().count()
        });
        
        Ok(Output::new().with("transformed_output", transformed))
    }
}

// Configuration
let config = Config::new("data_transformer")
    .with_inputs(vec!["text_input"])
    .with_outputs(vec!["transformed_output"])
    .with_input_types(hashmap! {
        "text_input".to_string() => DataType::Json,
    });

// Run the node
let node = DataTransformer;
node.run(config)?;
```

## Stateful Node

### Python Example
```python
from dora import Node
from collections import deque

class StatefulProcessor(Node):
    def __init__(self, window_size=5):
        super().__init__()
        self.window = deque(maxlen=window_size)
        
    def on_input(self, input_data):
        self.window.append(input_data["value"])
        
        # Calculate statistics
        stats = {
            "average": sum(self.window) / len(self.window),
            "min": min(self.window),
            "max": max(self.window),
            "count": len(self.window)
        }
        
        return stats

# Configuration
config = Config(
    name="stateful_processor",
    inputs=["value_stream"],
    outputs=["statistics"]
)

# Run the node
node = StatefulProcessor(window_size=5)
node.run(config)
```

### Rust Example
```rust
use dora_node_api::{Node, Config};
use std::collections::VecDeque;

struct StatefulProcessor {
    window: VecDeque<f64>,
}

impl Node for StatefulProcessor {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        let value = input.get::<f64>("value")?;
        self.window.push_back(value);
        
        // Calculate statistics
        let stats = json!({
            "average": self.window.iter().sum::<f64>() / self.window.len() as f64,
            "min": self.window.iter().fold(f64::INFINITY, |a, &b| a.min(b)),
            "max": self.window.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)),
            "count": self.window.len()
        });
        
        Ok(Output::new().with("statistics", stats))
    }
}

// Configuration
let config = Config::new("stateful_processor")
    .with_inputs(vec!["value_stream"])
    .with_outputs(vec!["statistics"]);

// Run the node
let mut node = StatefulProcessor {
    window: VecDeque::with_capacity(5)
};
node.run(config)?;
```

## Error Handling Node

### Python Example
```python
from dora import Node, NodeError

class SafeProcessor(Node):
    def __init__(self):
        super().__init__()
        
    def on_input(self, input_data):
        try:
            # Validate input
            if "value" not in input_data:
                raise NodeError("Missing required field: value")
                
            value = float(input_data["value"])
            if value < 0:
                raise NodeError("Value must be non-negative")
                
            # Process data
            result = {
                "square": value ** 2,
                "cube": value ** 3,
                "sqrt": value ** 0.5
            }
            
            return result
            
        except ValueError as e:
            raise NodeError(f"Invalid value format: {str(e)}")
        except Exception as e:
            raise NodeError(f"Processing error: {str(e)}")

# Configuration
config = Config(
    name="safe_processor",
    inputs=["value_input"],
    outputs=["results"]
)

# Run the node
node = SafeProcessor()
node.run(config)
```

### Rust Example
```rust
use dora_node_api::{Node, Config, NodeError};

struct SafeProcessor;

impl Node for SafeProcessor {
    fn on_input(&mut self, input: Input) -> Result<Output, Error> {
        // Validate input
        let value = input.get::<f64>("value")
            .map_err(|e| NodeError::new(format!("Missing required field: value: {}", e)))?;
            
        if value < 0.0 {
            return Err(NodeError::new("Value must be non-negative".to_string()));
        }
        
        // Process data
        let result = json!({
            "square": value * value,
            "cube": value * value * value,
            "sqrt": value.sqrt()
        });
        
        Ok(Output::new().with("results", result))
    }
}

// Configuration
let config = Config::new("safe_processor")
    .with_inputs(vec!["value_input"])
    .with_outputs(vec!["results"]);

// Run the node
let node = SafeProcessor;
node.run(config)?;
```

## Resource Management Node

### Python Example
```python
from dora import Node
import psutil

class ResourceMonitor(Node):
    def __init__(self):
        super().__init__()
        self.process = None
        
    def setup(self):
        self.process = psutil.Process()
        
    def on_input(self, input_data):
        # Monitor system resources
        stats = {
            "cpu_percent": self.process.cpu_percent(),
            "memory_percent": self.process.memory_percent(),
            "threads": self.process.num_threads(),
            "open_files": len(self.process.open_files())
        }
        
        return stats
        
    def cleanup(self):
        self.process = None

# Configuration
config = Config(
    name="resource_monitor",
    inputs=["trigger"],
    outputs=["stats"]
)

# Run the node
node = ResourceMonitor()
node.run(config)
```

### Rust Example
```rust
use dora_node_api::{Node, Config};
use sysinfo::{System, SystemExt, ProcessExt};

struct ResourceMonitor {
    sys: System,
}

impl Node for ResourceMonitor {
    fn setup(&mut self) -> Result<(), Error> {
        self.sys.refresh_all();
        Ok(())
    }
    
    fn on_input(&mut self, _input: Input) -> Result<Output, Error> {
        self.sys.refresh_all();
        
        let current_process = self.sys.get_current_pid()
            .ok_or_else(|| NodeError::new("Failed to get current process".to_string()))?;
            
        let process = self.sys.get_process(current_process)
            .ok_or_else(|| NodeError::new("Failed to get process info".to_string()))?;
            
        let stats = json!({
            "cpu_percent": process.cpu_usage(),
            "memory_percent": process.memory() as f64 / self.sys.total_memory() as f64 * 100.0,
            "threads": process.thread_count(),
            "open_files": process.open_files().len()
        });
        
        Ok(Output::new().with("stats", stats))
    }
}

// Configuration
let config = Config::new("resource_monitor")
    .with_inputs(vec!["trigger"])
    .with_outputs(vec!["stats"]);

// Run the node
let mut node = ResourceMonitor {
    sys: System::new_all()
};
node.run(config)?;
```

## Next Steps
1. Try modifying these examples to suit your needs
2. Explore the [Advanced Examples](./advanced.md)
3. Check out the [API Documentation](../api/README.md)
4. Join our [Community](../community.md) for more examples and support 