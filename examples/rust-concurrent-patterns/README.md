# Rust Concurrent Patterns for Dora

This example demonstrates safe and efficient concurrent patterns in Dora using Rust threads and mpsc channels, as recommended by the Dora maintainers as an alternative to direct callbacks.

## Overview

Instead of using traditional callback functions (which are difficult to handle safely across language boundaries), this example shows how to:

1. **Spawn worker threads** for compute-heavy operations
2. **Use mpsc channels** to safely communicate results back to the main event loop
3. **Handle concurrent operations** with sensor data processing
4. **Implement patterns** for handling emergency stops and reactive conditions

## Why Not Callbacks?

Callbacks are avoided in Dora because:
- They're difficult to handle safely across language boundaries (Python, C++, Rust)
- Callbacks to memory-unsafe languages can lead to undefined behavior
- Thread safety and synchronization become complex

## Pattern Overview

The recommended pattern uses:

```rust
fn main() {
    // Initialize Dora node
    let (dora, events) = DoraNode::init_from_env();
    
    // Create channels for thread communication
    let (outputs_tx, outputs_rx) = mpsc::channel();
    
    // Spawn output handler thread
    thread::spawn(|| output_handler(dora, outputs_rx));
    
    // Main event loop
    for event in events {
        match event_type {
            Type::Input => {
                // Spawn worker thread for expensive computation
                thread::spawn(|| handle_input(event, outputs_tx.clone()));
            }
        }
    }
}
```

## Files in This Example

- **Cargo.toml** - Project configuration
- **src/main.rs** - Main example demonstrating the pattern
- **src/patterns.rs** - Helper utilities and abstractions
- **README.md** - This file

## Running the Example

```bash
cd examples/rust-concurrent-patterns
cargo build
dora run dataflow.yaml
```

## Key Concepts

### 1. Worker Threads
Compute-heavy operations run in separate threads to avoid blocking the main event loop.

### 2. MPSC Channels
Multiple Producer, Single Consumer channels safely pass data between threads without shared memory.

### 3. Event Routing
Custom event routing utilities help map events to their appropriate handlers.

### 4. Managed Cleanup
Helper utilities ensure proper thread cleanup and resource management.

## Common Patterns Covered

- Background data processing
- Real-time status updates
- Emergency stop handling
- Concurrent sensor data processing
- Result aggregation from multiple threads

## Performance Considerations

- Minimal overhead compared to callbacks
- Predictable thread management
- No garbage collection issues
- Safe across all language boundaries

## Further Reading

- [Dora Documentation](https://www.dora-rs.ai/docs/guides/)
- [Rust Concurrency](https://doc.rust-lang.org/book/ch16-00-concurrency.html)
- [Crossbeam (Advanced Threading)](https://docs.rs/crossbeam/latest/crossbeam/)

## Contributing

Feel free to extend this example with your own patterns and use cases!
