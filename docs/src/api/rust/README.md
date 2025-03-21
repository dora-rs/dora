# Rust API Documentation

## Overview
The Dora Rust API provides a low-level, high-performance interface for creating and managing Dora nodes and dataflows.

[See the full Rust API documentation here](rust/README.md)

## Core Components

### Node Trait
```rust
use dora::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
struct NodeConfig {
    threshold: f64,
    timeout: u64,
}

struct MyNode {
    config: NodeConfig,
    counter: u64,
}

impl Node for MyNode {
    type Config = NodeConfig;
    type Input = Value;
    type Output = Value;

    fn new(config: Self::Config) -> Self {
        Self {
            config,
            counter: 0,
        }
    }

    async fn on_input(&mut self, input: Self::Input) -> Result<Option<Self::Output>, Error> {
        self.counter += 1;
        Ok(Some(Value::Number(self.counter.into())))
    }
}
```

### DataFlow Builder
```rust
use dora::DataFlow;

let mut flow = DataFlow::new();

// Add nodes
let counter = flow.add_node("counter", MyNode::new(NodeConfig {
    threshold: 100.0,
    timeout: 30,
}));

let processor = flow.add_node("processor", ProcessorNode::new(ProcessorConfig {
    batch_size: 100,
    ..Default::default()
}));

// Connect nodes
flow.connect("counter.output", "processor.input")?;

// Run the dataflow
flow.run().await?;
```

## Type System

### Common Types
```rust
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct NodeMetadata {
    pub name: String,
    pub version: String,
    pub timestamp: DateTime<Utc>,
}

#[derive(Debug, Serialize, Deserialize)]
pub enum Value {
    Null,
    Boolean(bool),
    Number(f64),
    String(String),
    Array(Vec<Value>),
    Object(HashMap<String, Value>),
}
```

### Error Handling
```rust
use thiserror::Error;

#[derive(Error, Debug)]
pub enum NodeError {
    #[error("Invalid input: {0}")]
    InvalidInput(String),
    #[error("Processing error: {0}")]
    ProcessingError(String),
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
}

type Result<T> = std::result::Result<T, NodeError>;
```

## Performance Considerations

### Memory Management
- Use zero-copy data transfer where possible
- Implement proper cleanup in `Drop`
- Use memory pools for frequently allocated types

### Async Operations
- Use `tokio` for async runtime
- Implement proper backpressure handling
- Use connection pooling for external services

## Best Practices

### Error Handling
```rust
impl Node for MyNode {
    async fn on_input(&mut self, input: Self::Input) -> Result<Option<Self::Output>, Error> {
        match self.process_data(input).await {
            Ok(result) => Ok(Some(result)),
            Err(e) => {
                self.logger.error(&format!("Processing error: {}", e));
                Err(NodeError::ProcessingError(e.to_string()))
            }
        }
    }
}
```

### Logging
```rust
use tracing::{info, debug, error};

impl Node for MyNode {
    fn new(config: Self::Config) -> Self {
        info!("Initializing node with config: {:?}", config);
        Self {
            config,
            counter: 0,
        }
    }
}
```

### Configuration
```rust
#[derive(Debug, Serialize, Deserialize)]
struct NodeConfig {
    #[serde(default = "default_threshold")]
    threshold: f64,
    #[serde(default = "default_timeout")]
    timeout: u64,
}

fn default_threshold() -> f64 { 100.0 }
fn default_timeout() -> u64 { 30 }
```

## Examples

### Basic Node
```rust
struct EchoNode;

impl Node for EchoNode {
    type Config = ();
    type Input = Value;
    type Output = Value;

    fn new(_config: Self::Config) -> Self {
        Self
    }

    async fn on_input(&mut self, input: Self::Input) -> Result<Option<Self::Output>, Error> {
        Ok(Some(input))
    }
}
```

### Data Transformation
```rust
struct TransformNode;

impl Node for TransformNode {
    type Config = TransformConfig;
    type Input = RawData;
    type Output = TransformedData;

    async fn on_input(&mut self, input: Self::Input) -> Result<Option<Self::Output>, Error> {
        let transformed = TransformedData {
            timestamp: input.time,
            value: input.value.parse()?,
            metadata: input.metadata,
        };
        Ok(Some(transformed))
    }
}
```

### External Service Integration
```rust
struct APINode {
    client: reqwest::Client,
    config: APIConfig,
}

impl Node for APINode {
    async fn on_input(&mut self, input: Self::Input) -> Result<Option<Self::Output>, Error> {
        let response = self.client
            .get(&self.config.api_url)
            .send()
            .await?
            .json()
            .await?;
        Ok(Some(response))
    }
}
``` 