# Error Propagation Example

This example demonstrates the error event propagation feature, which allows nodes to send errors to downstream nodes instead of crashing.

## Overview

The dataflow consists of two nodes:
- **Producer**: Simulates a processing error and sends an error event instead of crashing
- **Consumer**: Receives the error event and handles it gracefully

## Key Features Demonstrated

1. **Graceful Error Handling**: The producer node encounters an error but continues running
2. **Error Propagation**: The error is sent to downstream nodes via `node.send_error()`
3. **Error Reception**: The consumer receives `Event::InputError` and can handle it appropriately

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
[Producer] Sending message 0
[Producer] Sending message 1
[Producer] Sending message 2
[Producer] Encountered an error! Sending error event instead of crashing...
[Producer] Continuing after error...
[Producer] Exiting gracefully
```

**Consumer:**
```
[Consumer] Starting...
[Consumer] ⚠️  Received error from node 'producer' on input 'data': Simulated processing error: invalid data format
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

