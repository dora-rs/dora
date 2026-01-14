# Error Propagation Example

This example demonstrates the automatic error event propagation feature, which notifies downstream nodes when a node fails.

## Overview

The dataflow consists of two nodes:
- **Producer**: Simulates a processing error and exits with a non-zero exit code
- **Consumer**: Automatically receives an error event when the producer fails and handles it gracefully

## Key Features Demonstrated

1. **Automatic Error Propagation**: When a node exits with a non-zero exit code, the daemon automatically sends `NodeFailed` events to all downstream nodes
2. **Error Reception**: The consumer receives `Event::NodeFailed` and can handle it appropriately
3. **Fault Tolerance**: Downstream nodes are notified of upstream failures and can implement recovery strategies

## Running the Example

```bash
# Build the example
cargo build --release -p error-propagation-producer -p error-propagation-consumer

# Run the dataflow
dora up
dora start examples/error-propagation/dataflow.yml
```

## Expected Output

**Producer:**
```
[Producer] Starting...
[Producer] Processing message 0
[Producer] Processing message 1
[Producer] Processing message 2
[Producer] Encountered an error! Exiting with error code...
Error: Simulated processing error: invalid data format
```

**Consumer:**
```
[Consumer] Starting...
[Consumer] ⚠️  Received error from node 'producer' affecting inputs ["data"]: exited with code 1
[Consumer] Handling error gracefully - using cached data...
[Consumer] Input data closed
[Consumer] Exiting
```

## Use Cases

This feature is useful for:
- **Fault Tolerance**: Nodes can recover from errors without crashing the entire dataflow
- **Graceful Degradation**: Downstream nodes can switch to backup data sources or cached data
- **Error Logging**: Centralized error handling and monitoring
- **Retry Logic**: Downstream nodes can implement retry strategies

