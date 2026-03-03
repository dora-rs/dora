# Service Example

Demonstrates the **service** communication pattern: a client sends structured requests and receives correlated responses via `REQUEST_ID` metadata.

## Architecture

```
timer (500ms) --> service-client --> request  --> service-server
                                  <- response <--
```

## Nodes

**service-client** (Rust) -- Every 500ms, sends a request containing a `StructArray` with fields `a` (sequential counter) and `b` (counter + 10). Uses `node.send_service_request()` which automatically generates a `REQUEST_ID` and embeds it in metadata. Tracks pending requests by ID in a HashMap and matches responses.

**service-server** (Rust) -- Receives requests, extracts fields `a` and `b`, computes `sum = a + b`, and returns a response `StructArray` with the `sum` field. Uses `node.send_service_response()` which propagates the `REQUEST_ID` from the incoming metadata.

## Key Concepts

- **Request ID**: unique UUID v7 identifier, generated internally by `send_service_request()`
- **Metadata key**: `REQUEST_ID` automatically managed by helper methods
- **Arrow StructArray**: typed structured data for request/response payloads

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
| Service pattern (request/response) | Both nodes |
| `send_service_request()` / `send_service_response()` | Client / Server |
| `REQUEST_ID` metadata correlation | Automatic via helpers |
| Arrow `StructArray` for typed payloads | Both nodes |
| Pending request tracking | Client HashMap |
