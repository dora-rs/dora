# Error Propagation Example

This example demonstrates the automatic error event propagation feature, which notifies downstream nodes when a node fails.

## Overview

The dataflow consists of two nodes:
- **Producer**: Sends a few messages, then simulates a processing error by exiting with a non-zero exit code
- **Consumer**: Receives the messages, then automatically receives a `NodeFailed` event when the producer crashes and handles it gracefully

## Running the Example

```bash
# Build the example
cargo build --release -p error-propagation-producer -p error-propagation-consumer

# Run the dataflow
dora up
dora build examples/error-propagation/dataflow.yml
dora run examples/error-propagation/dataflow.yml
```

## Expected Output

**Producer:**
```
Starting...
Sent message 0
Sent message 1
Sent message 2
Encountered an error! Exiting with error code...
Error: Simulated processing failure
```

**Consumer:**
```
Starting...
Received input on data
Received input on data
Received input on data
⚠️  Received error from node 'producer' affecting inputs ["data"]: exited with code 1
Handling error in some way...
Input data closed
Exiting
```
