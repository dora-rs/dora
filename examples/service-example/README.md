# Service Example

Demonstrates the **service** communication pattern with **multiple clients**: two clients send structured requests to a single server and receive correlated responses via `REQUEST_ID` metadata. Each client filters responses by its own tracked request IDs.

## Architecture

```
timer (500ms) --> service-client-1 --> request --> service-server --> response --> service-client-1
timer (700ms) --> service-client-2 --> request -->                --> response --> service-client-2
```

Both clients use the same binary -- the YAML wires them as separate nodes with different timers. Responses fan out to all clients; each filters by its own tracked `REQUEST_ID`.

## Nodes

**service-client-1 / service-client-2** (Rust, same binary) -- Sends requests at 500ms / 700ms intervals containing a `StructArray` with fields `a` (sequential counter) and `b` (counter + 10). Uses `node.send_service_request()` which automatically generates a `REQUEST_ID` and embeds it in metadata. Tracks pending requests by ID in a HashMap and matches responses. Logs are prefixed with the node ID (`ADORA_NODE_ID`) to distinguish output.

**service-server** (Rust) -- Receives requests from both clients (via separate named inputs), extracts fields `a` and `b`, computes `sum = a + b`, and returns a response `StructArray` with the `sum` field. Uses `node.send_service_response()` which propagates the `REQUEST_ID` from the incoming metadata.

## Key Concepts

- **Multi-client fan-in/fan-out**: multiple clients share one server; responses fan out, clients correlate by ID
- **Request ID**: unique UUID v7 identifier, generated internally by `send_service_request()`
- **Metadata key**: `REQUEST_ID` automatically managed by helper methods
- **Arrow StructArray**: typed structured data for request/response payloads
- **Same binary, multiple nodes**: YAML declares distinct nodes pointing to the same executable

See [docs/patterns.md](../../docs/patterns.md#2-service-requestreply) for the full pattern specification.

## Run

```bash
cargo build -p service-example-client -p service-example-server
adora run dataflow.yml
```

Or in distributed mode:

```bash
adora up
adora start dataflow.yml --attach
# Ctrl+C to detach, then:
adora stop --all && adora down
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Multi-client service pattern | Two clients, one server |
| `send_service_request()` / `send_service_response()` | Client / Server |
| `REQUEST_ID` metadata correlation | Automatic via helpers |
| Arrow `StructArray` for typed payloads | Both nodes |
| Pending request tracking | Client HashMap |
| Same binary, multiple nodes | YAML dataflow |
