# ADR-002: Protocol Between UI Clients and Dora Core

- **Date:** 2025-03-21
- **Status:** Draft
- **Related Issues:** #111, #112, #113, #114, #115, #116, #117

## Context

We want to allow multiple UI clients (TUI, legacy CLI, future GUI/web) to interact with the Dora
runtime without linking directly against CLI internals. Today, the TUI shells out to the legacy CLI
or hits coordinator RPC helpers directly, which prevents remote use and increases coupling.

ADR-001 defined a local trait-based boundary (`tui-interface`). This ADR specifies a network
protocol so that those traits can be implemented over the wire. The protocol will be the consistent
contract for launching dataflows, inspecting state, retrieving logs/telemetry, and handling
preferences.

## Goals

- Provide a transport-agnostic, evolvable interface between UI clients and Dora core.
- Cover the key operations identified in the Phase P0 audit (list/start/stop/destroy, logs,
  telemetry, preferences, inspect, etc.).
- Support streaming telemetry/logs and asynchronous command execution.
- Enable remote clients (no local filesystem access) while retaining backward compatibility for
  in-process callers.

## Non-Goals

- Complete replacement of the legacy CLI in this ADR (migration tracked separately).
- Authentication/authorization policies (initially out of scope; placeholders will be provided).

## Proposed Architecture

```
[TUI / CLI / Future UI]
          |
          v
- [Protocol Client SDK]
         |
         v
 +------------------------+
 | Dora Protocol Gateway  |  <-- runs alongside coordinator (same process or sidecar)
 +------------------------+
         |
         v
    [Coordinator]
         |
         v
  [Daemon / Runtime]
```

- The **protocol gateway** exposes HTTP/JSON endpoints with Server-Sent Events (SSE) channels for
  streaming. gRPC remains a future option, but HTTP+JSON keeps tooling simple and is easily
  consumable from multiple languages.
- The **client SDK** (initially Rust) wraps these endpoints and implements the service traits defined
  in `tui-interface`.
- The legacy CLI/TUI register their service implementation using the SDK. During migration they can
  opt for local implementations when running on the same machine.

## Transport & Encoding

- **Transport:** HTTP 1.1/2 with JSON payloads.
- **Streaming:** Use Server-Sent Events (SSE) for log/telemetry streams; fallback to long-polling for
  environments that cannot keep persistent streams open.
- **Error handling:** Standard HTTP status codes with JSON error envelopes (see below).
- **Authentication:** Placeholder headers (`Authorization`) to be wired in later. Credentials are
  optional in the initial rollout.

## Core Endpoints (MVP)

| Endpoint | Verb | Description | Response |
| --- | --- | --- | --- |
| `/v1/dataflows` | `GET` | List dataflows with optional query params (status, name, pagination). | `DataflowSummary[]` |
| `/v1/dataflows` | `POST` | Start new dataflow (`up`, `start`, or config payload). | `OperationHandle` |
| `/v1/dataflows/{id}:start` | `POST` | Start an existing dataflow. | `OperationHandle` |
| `/v1/dataflows/{id}:stop` | `POST` | Stop a running dataflow. | `OperationHandle` |
| `/v1/dataflows/{id}:destroy` | `POST` | Destroy a dataflow. | `OperationHandle` |
| `/v1/dataflows/{id}` | `GET` | Fetch detailed info including nodes/metrics. | `DataflowDetail` |
| `/v1/logs/{id}` | `GET` | Fetch recent logs (query params for tail count, since). | `LogChunk[]` |
| `/v1/logs/{id}/stream` | `SSE` | Stream logs live over SSE. | `LogEvent` stream |
| `/v1/telemetry/system` | `GET` | Snapshot of system metrics. | `SystemMetrics` |
| `/v1/telemetry/system/stream` | `SSE` | Stream periodic system metrics via SSE. | `SystemMetrics` stream |
| `/v1/preferences/ui` | `GET` | Retrieve UI preference snapshot. | `UserPreferencesSnapshot` |
| `/v1/preferences/ui` | `PUT` | Persist UI preferences. | `204 No Content` |
| `/v1/commands/legacy` | `POST` | Execute unsupported legacy CLI command (optional escape hatch). | `OperationHandle` |
| `/v1/operations/{handle}` | `GET` | Poll result of async operation (start/stop). | `OperationStatus` + optional payload |

### Data Structures (JSON)

#### `DataflowSummary`
```json
{
  "id": "8f7e…",
  "name": "camera-pipeline",
  "status": "running",
  "updated_at": "2025-03-21T12:34:56Z"
}
```

#### `DataflowDetail`
```json
{
  "summary": { …summary fields… },
  "nodes": [
    {
      "id": "node1",
      "name": "camera",
      "status": "running",
      "kind": "runtime",
      "inputs": ["cam/raw"],
      "outputs": ["cam/processed"],
      "description": null,
      "source": { "type": "local", "path": "…" }
    }
  ]
}
```

#### `OperationHandle`
```json
{
  "handle": "op-1234",
  "submitted_at": "2025-03-21T12:34:56Z"
}
```

#### `OperationStatus`
```json
{
  "handle": "op-1234",
  "state": "completed",      // pending | running | completed | failed
  "message": "Start completed",
  "result": { /* optional payload depending on command */ }
}
```

#### `LogEvent`
```json
{
  "timestamp": "2025-03-21T12:35:00Z",
  "level": "INFO",
  "node": "camera",
  "line": "Camera initialized"
}
```

#### `SystemMetrics`
(as already defined in `tui-interface`, expressed in JSON)

### Error Envelope

Errors from the gateway respond with standard HTTP status codes and the following JSON body:

```json
{
  "error": {
    "code": "RESOURCE_NOT_FOUND",
    "message": "Dataflow 123 not found",
    "details": {
      "resource": "dataflow",
      "id": "123"
    }
  }
}
```

Possible `code` values (non-exhaustive): `RESOURCE_NOT_FOUND`, `INVALID_ARGUMENT`,
`ALREADY_EXISTS`, `FAILED_PRECONDITION`, `INTERNAL_ERROR`, `NOT_IMPLEMENTED`, `UNAVAILABLE`.

## Versioning & Compatibility

- Base path includes major version (`/v1`).
- Backward-compatible changes (new fields, endpoints) allowed within same major version.
- Breaking changes require bump to `/v2` with deprecation policy.
- Clients advertise supported ranges via header `X-Dora-Client-Version: 1.x`.
- Gateway may expose `/v1/status` endpoint returning `{"serverVersion":"1.2.0"}` for diagnostics.

## Migration Strategy

1. Implement gateway endpoints in coordinator and leverage existing runtime APIs.
2. Build Rust protocol client that satisfies the `tui-interface` service traits.
3. Switch TUI to use protocol client behind feature flag; CLI continues using direct bridge until
   commands migrate.
4. Gradually replace legacy CLI operations with protocol calls.
5. Eventually retire direct coordinator RPC helpers and the legacy command executor.

## Open Questions

- Authentication/authorization model (API keys? OAuth? integration with existing infra?).
- Should `build/new/check` operations be protocolized early or left to CLI bridge?
- How to handle large log retention or pagination beyond simple tail operations?
- Streaming telemetry interval negotiation (client-specified vs server default?).

## Alternatives Considered

- **gRPC**: Offers strong typing and streaming out of the box, but increases tooling requirements
  and raises adoption barriers for non-Rust clients. We can add gRPC in future if needed.
- **Keep CLI-only traits**: Would not enable remote clients; leaves us dependent on shelling out.
- **Expose coordinator RPC directly**: Maintains current coupling and lacks standardized responses.

## References

- ADR-001: TUI ↔ Core Interface Boundary
- Touchpoint Inventory (docs/tui-boundary/touchpoints.md)
- Roadmap tracker (docs/tui-boundary/protocol-boundary-issues.md)
