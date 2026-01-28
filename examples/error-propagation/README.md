# Error Propagation Example

This example demonstrates the automatic error event propagation feature, which notifies downstream nodes when a node fails.

## Overview

The dataflow consists of two nodes:
- **Producer**: Simulates a processing error and exits with a non-zero exit code
- **Consumer**: Automatically receives an error event when the producer fails and handles it gracefully

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
Processing message 0
Processing message 1
Processing message 2
Encountered an error! Exiting with error code...
Error: Simulated error: exiting with exit code 1
```

**Consumer:**
```
[Consumer] Starting...
[Consumer] ⚠️  Received error from node 'producer' affecting inputs ["data"]: exited with code 1
[Consumer] Handling error in some way...
[Consumer] Input data closed
[Consumer] Exiting
```

