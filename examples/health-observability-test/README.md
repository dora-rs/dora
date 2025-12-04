# Health Observability Test Example

This example demonstrates the Node Health Introspection and Observability API (Issue #1232).

## Features Demonstrated

1. **Health Status Management** - `set_health_status()`
2. **Lifecycle Event Subscription** - `subscribe_node_events()`
3. **Error Propagation** - `send_error()`
4. **Input Health Queries** - `query_input_health()`

## Running

```bash
cargo build
dora up
dora start dataflow.yml
```

## API Methods Used

- `node.subscribe_node_events(&[NodeId])`
- `node.set_health_status(HealthStatus)`
- `node.send_error(output_id, error)`
- `node.query_input_health(&input_id)`

