# WebSocket Control Plane

Dora's control plane uses WebSocket connections for all communication between the CLI, coordinator, and daemons. A single Axum server exposes three routes on one port, replacing the previous multi-port TCP design. JSON text frames carry a UUID-correlated request-reply protocol with fire-and-forget events for log streaming.

## Features at a Glance

| Feature | Detail |
|---------|--------|
| Routes | `/api/control` (CLI), `/api/daemon` (daemons), `/health` |
| Wire format | JSON text frames + binary frames for topic data |
| Protocol | UUID-correlated request-reply + fire-and-forget events |
| Message size limit | 1 MiB (`MAX_CONTROL_MESSAGE_BYTES`) |
| Concurrency limit | 256 connections (`MAX_WS_CONNECTIONS`) |
| Server framework | Axum + Tower middleware |
| Client library | `tokio-tungstenite` (integration tests, daemon), custom `WsSession` (CLI) |
| Security | Re-register guard, daemon ID verification, machine ID length limit |

---

## Architecture

```
                        Single Axum server (one port)
                       ┌────────────────────────────┐
                       │  /api/control   (CLI)       │
  CLI ──── WS ────────>│  /api/daemon    (Daemons)   │
                       │  /health        (HTTP GET)  │
  Daemon ── WS ───────>│                             │
                       └──────────┬─────────────────┘
                                  │ mpsc::Sender<Event>
                                  v
                            Coordinator
                          (event loop)
```

The coordinator binds a single `TcpListener` and serves an Axum router. Each WebSocket upgrade spawns a handler task that communicates with the coordinator's main event loop through an `mpsc::Sender<Event>` channel.

### Key source files

| File | Role |
|------|------|
| `binaries/coordinator/src/ws_server.rs` | Router, `serve()`, constants, `ShutdownTrigger` |
| `binaries/coordinator/src/ws_control.rs` | `/api/control` handler |
| `binaries/coordinator/src/ws_daemon.rs` | `/api/daemon` handler, security, event translation |
| `binaries/cli/src/ws_client.rs` | `WsSession` synchronous client wrapper |
| `libraries/message/src/ws_protocol.rs` | `WsRequest`, `WsResponse`, `WsEvent`, `WsMessage` types |

---

## Wire Protocol

All messages are JSON text frames. Three message shapes exist:

### WsRequest (client -> server)

```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "method": "control",
  "params": { "List": null }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Unique request identifier for reply correlation |
| `method` | string | `"control"` for CLI requests, `"daemon_event"` / `"daemon_command"` for daemon |
| `params` | object | Serialized `ControlRequest` or `Timestamped<CoordinatorRequest>` |

### WsResponse (server -> client)

Success:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "result": { "DataflowList": [] }
}
```

Error:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "error": "no running dataflow with id ..."
}
```

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | Matches the originating request `id` |
| `result` | object? | Present on success (serialized `ControlRequestReply`) |
| `error` | string? | Present on failure |

### WsEvent (either direction)

```json
{
  "event": "log",
  "payload": { "message": "sensor started", "level": "info" }
}
```

Used for log streaming after a `LogSubscribe`/`BuildLogSubscribe` is acknowledged.

### Dispatch

Each handler parses incoming frames with its own strategy to preserve u128 fidelity (see [u128 serialization](#u128-serialization-workaround)):

- **CLI (`ws_client.rs`)**: Uses a flat `IncomingFrame` struct with `serde_json::value::RawValue` for the `result`/`payload` fields, avoiding `serde_json::Value` entirely. Discriminates by presence of `event` (log push) or `id` (response).
- **Coordinator control handler (`ws_control.rs`)**: Parses as `WsRequest` (always a request from CLI).
- **Coordinator daemon handler (`ws_daemon.rs`)**: Checks for `"method"` key to distinguish requests vs responses. Uses `DaemonWsRequestRaw` helper for requests.
- **Daemon (`coordinator.rs`)**: Uses `CoordinatorCommandRaw` / `RegisterReplyRaw` helper structs to parse directly from raw JSON text.

A `WsMessage` untagged enum is defined in `ws_protocol.rs` for generic dispatch but is not used by the production handlers:

```rust
#[serde(untagged)]
pub enum WsMessage {
    Request(WsRequest),
    Response(WsResponse),
    Event(WsEvent),
}
```

---

## CLI Control Plane (`/api/control`)

The CLI connects to `/api/control` to send `ControlRequest` commands and receive `ControlRequestReply` responses.

### Connection lifecycle

1. **Connect** -- HTTP upgrade to WebSocket
2. **Request-reply** -- CLI sends `WsRequest`, coordinator processes the `ControlRequest`, sends `WsResponse`
3. **Log subscribe** (optional) -- CLI sends `LogSubscribe`/`BuildLogSubscribe`, coordinator acks with `WsResponse`, then pushes `WsEvent{event:"log"}` frames
4. **Close** -- CLI sends `Close` frame or drops connection

### Supported ControlRequest variants

| Variant | Description |
|---------|-------------|
| `List` | List all running dataflows |
| `Build` | Trigger a dataflow build |
| `WaitForBuild` | Block until build completes |
| `Start` | Start a dataflow |
| `WaitForSpawn` | Block until nodes are spawned |
| `Stop` / `StopByName` | Stop a running dataflow |
| `Reload` | Hot-reload a node/operator |
| `Check` | Check dataflow status |
| `Destroy` | Tear down all daemons |
| `Logs` | Retrieve historical logs |
| `Info` | Get dataflow details |
| `DaemonConnected` | Check if any daemon is connected |
| `ConnectedMachines` | List connected daemons |
| `LogSubscribe` | Subscribe to live dataflow logs |
| `BuildLogSubscribe` | Subscribe to live build logs |
| `CliAndDefaultDaemonOnSameMachine` | Check co-location |
| `GetNodeInfo` | Get node metadata |
| `TopicSubscribe` | Subscribe to live topic data via binary WS frames ([details](websocket-topic-data-channel.md)) |
| `TopicUnsubscribe` | Cancel a topic subscription |

### Log subscription flow

```
CLI                         Coordinator
 │                              │
 │─── WsRequest{LogSubscribe} ─>│
 │                              │  (check dataflow exists)
 │<── WsResponse{subscribed} ───│
 │                              │
 │<── WsEvent{event:"log"} ────│  (repeated)
 │<── WsEvent{event:"log"} ────│
 │                              │
 │─── Close ───────────────────>│  (log_subscribers dropped)
```

If the dataflow is not found, the coordinator returns `WsResponse` with an error and no events are sent.

### WsSession (CLI client)

`WsSession` is a synchronous wrapper that bridges blocking CLI code to the async WebSocket connection. It creates an internal `tokio::runtime::Runtime` (current-thread) and spawns an async `session_loop` task.

```
CLI thread (sync)                       session_loop (async)
     │                                        │
     │── SessionCommand::Request ────────────>│── WsRequest ──> server
     │                                        │<── WsResponse ──
     │<── oneshot reply ─────────────────────│
     │                                        │
     │── SessionCommand::SubscribeLogs ──────>│── WsRequest ──> server
     │                                        │<── WsResponse (ack)
     │<── oneshot ack ───────────────────────│
     │<── std_mpsc log events ───────────────│<── WsEvent ──
```

The session loop maintains:
- `pending_requests: HashMap<Uuid, oneshot::Sender>` -- for request-reply correlation
- `pending_subscribes: HashMap<Uuid, (ack_tx, log_tx)>` -- for subscribe ack routing
- `log_subscribers: Vec<std_mpsc::Sender>` -- for broadcasting log events
- `pending_topic_subscribes: HashMap<Uuid, (ack_tx, data_tx)>` -- for topic subscribe ack routing
- `topic_subscribers: HashMap<Uuid, std_mpsc::Sender>` -- for binary frame dispatch by subscription UUID

Binary WS frames (topic data) are dispatched separately from text frames. See [WebSocket Topic Data Channel](websocket-topic-data-channel.md) for details.

On disconnect, all pending requests receive an error via their oneshot channels.

---

## Daemon Plane (`/api/daemon`)

Daemons connect to `/api/daemon` for registration, event reporting, and receiving coordinator commands.

### Registration flow

```
Daemon                       Coordinator
  │                              │
  │── WsRequest{Register} ─────>│
  │                              │  (validate, assign daemon_id)
  │                              │  (track connection + cmd channel)
  │                              │
  │── WsRequest{Event{...}} ───>│  (subsequent events)
```

1. Daemon sends a `Register` request containing `DaemonRegisterRequest` (version + machine ID)
2. Coordinator validates version compatibility and machine ID length
3. Coordinator assigns a `DaemonId` and stores the `DaemonConnection` (includes `cmd_tx` channel for sending commands back to the daemon)
4. The connection is tracked via `tracked_daemon_id` for cleanup on disconnect

### Event translation

Daemon events are translated into coordinator-internal `Event` variants:

| DaemonEvent | Coordinator Event |
|-------------|-------------------|
| `AllNodesReady` | `Event::Dataflow { ReadyOnDaemon }` |
| `AllNodesFinished` | `Event::Dataflow { DataflowFinishedOnDaemon }` |
| `Heartbeat` | `Event::DaemonHeartbeat` |
| `Log(message)` | `Event::Log(message)` |
| `Exit` | `Event::DaemonExit` |
| `NodeMetrics` | `Event::NodeMetrics` |
| `BuildResult` | `Event::DataflowBuildResult` |
| `SpawnResult` | `Event::DataflowSpawnResult` |

### Bidirectional communication

The coordinator can send commands back to daemons via the `cmd_tx` channel stored in `DaemonConnection`. The daemon handler maintains a `pending_replies: HashMap<Uuid, oneshot::Sender>` to correlate daemon responses to coordinator-initiated requests.

Message routing on the daemon handler:
- Frame has `"method"` key -> daemon request (registration or event)
- Frame lacks `"method"` key -> daemon response to a coordinator command

### u128 serialization workaround

`uhlc::ID` contains a `NonZeroU128` which exceeds `serde_json::Value::Number` range (i64/u64/f64 only). Using `serde_json::to_value()` errors with "number out of range", and `serde_json::from_slice::<Value>()` silently loses precision by storing as f64.

All production code bypasses `serde_json::Value` for data containing `uhlc::Timestamp`:

| Component | Serialization | Deserialization |
|-----------|--------------|-----------------|
| Daemon (`coordinator.rs`) | `to_string` + `format!` | Helper structs (`RegisterReplyRaw`, `CoordinatorCommandRaw`) + `from_str` |
| Coordinator control (`ws_control.rs`) | `to_string` + `format!` for replies | N/A (CLI requests don't contain u128) |
| Coordinator daemon (`ws_daemon.rs`) | N/A | `DaemonWsRequestRaw` + `from_str` |
| Coordinator state (`state.rs`) | `str::from_utf8` + `format!` (raw bytes embedding) | N/A |
| CLI (`ws_client.rs`) | N/A (requests don't contain u128) | `IncomingFrame` with `serde_json::value::RawValue` |

Integration tests similarly construct WsRequest JSON strings manually via `format!()` + `serde_json::to_string()` (not `to_value()`) to match the real wire format.

---

## Security

### Re-register guard

Each daemon WebSocket connection allows exactly one `Register` request. If a connection attempts a second registration, the coordinator logs a warning and closes the connection:

```
daemon attempted re-register on same connection, rejecting
```

### Daemon ID verification

After registration, every `Event` message must include a `daemon_id` matching the one assigned during registration. Mismatched IDs cause connection termination:

```
daemon sent event with mismatched id: expected `X`, got `Y` -- closing connection
```

### Machine ID length validation

The `machine_id` field in `DaemonRegisterRequest` is limited to 256 bytes. Oversized values cause connection termination.

### Connection and message limits

| Limit | Value | Enforced by |
|-------|-------|-------------|
| Max message size | 1 MiB | `WebSocketUpgrade::max_message_size` |
| Max concurrent connections | 256 | Tower `ConcurrencyLimitLayer` |

---

## Connection Lifecycle & Keepalive

### Establishment

Both `/api/control` and `/api/daemon` use standard HTTP/1.1 WebSocket upgrade. The Axum `WebSocketUpgrade` extractor handles the handshake.

### Ping/pong

Both handlers respond to `Ping` frames with `Pong` frames containing the same payload:

```rust
Ok(Message::Ping(data)) => {
    let _ = ws_tx.send(Message::Pong(data)).await;
    continue;
}
```

### Graceful close

When a `Close` frame is received:
- **Control handler**: breaks the handler loop, dropping log subscriber channels
- **Daemon handler**: breaks the loop, then emits `Event::DaemonExit { daemon_id }` for immediate cleanup

### Cleanup on disconnect

**Control connections**:
- `log_tx` channel is dropped, stopping log forwarding to that client
- No coordinator state to clean up (control connections are stateless)

**Daemon connections**:
- `DaemonExit` event is emitted if a `daemon_id` was tracked
- `cmd_tx` and `pending_replies` are dropped
- Coordinator removes the daemon from its connection map

**WsSession (CLI client)**:
- All entries in `pending_requests` receive `Err("WS connection closed")`
- All entries in `pending_subscribes` receive `Err("WS connection closed")`

---

## Message Flow Examples

### CLI lists dataflows

```
CLI                          WsSession                    Coordinator
 │                              │                              │
 │── request(&List) ───────────>│                              │
 │                              │── WsRequest ────────────────>│
 │                              │   id: "abc-123"              │
 │                              │   method: "control"          │
 │                              │   params: "List"             │
 │                              │                              │
 │                              │                    ControlEvent::IncomingRequest
 │                              │                    reply via oneshot
 │                              │                              │
 │                              │<── WsResponse ──────────────│
 │                              │   id: "abc-123"              │
 │                              │   result: {DataflowList:[]}  │
 │                              │                              │
 │<── ControlRequestReply ─────│                              │
```

### Daemon registration

```
Daemon                                    Coordinator
  │                                           │
  │── WsRequest ─────────────────────────────>│
  │   method: "daemon_event"                  │
  │   params: {inner: Register{...},          │
  │            timestamp: ...}                │
  │                                           │  validate version
  │                                           │  validate machine_id
  │                                           │  assign daemon_id
  │                                           │  store DaemonConnection
  │                                           │
  │── WsRequest{Event{Heartbeat}} ──────────>│
  │                                           │  Event::DaemonHeartbeat
  │                                           │
  │                        (on WS close) ────>│  Event::DaemonExit
```

### Log subscription lifecycle

```
CLI                    WsSession              Coordinator
 │                        │                        │
 │── subscribe_logs() ───>│                        │
 │                        │── WsRequest ──────────>│
 │                        │   params: LogSubscribe │
 │                        │                        │  find dataflow
 │                        │<── WsResponse ────────│  {subscribed: true}
 │<── ack (Ok) ──────────│                        │
 │                        │                        │
 │                        │<── WsEvent{log} ──────│  (node produces log)
 │<── log_rx.recv() ─────│                        │
 │                        │<── WsEvent{log} ──────│
 │<── log_rx.recv() ─────│                        │
 │                        │                        │
 │   (drop session) ─────>│── Close ─────────────>│  (log_subscribers dropped)
```

---

## Test Coverage

### Test tiers

| Tier | Location | Tests | What's covered |
|------|----------|-------|----------------|
| Unit (protocol) | `libraries/message/src/ws_protocol.rs` | 10 | Roundtrip serialization, untagged dispatch, error cases |
| Unit (client) | `binaries/cli/src/ws_client.rs` | 6 | Response routing, subscribe ack, topic subscribe ack, orphan handling, disconnect |
| Integration (control) | `binaries/coordinator/tests/ws_control_tests.rs` | 11 | Health check, List, invalid JSON/params, Destroy, DaemonConnected, ping/pong, concurrent requests, connection close, log subscribe |
| Integration (daemon) | `binaries/coordinator/tests/ws_daemon_tests.rs` | 4 | Register, register-then-status, disconnect cleanup, ping/pong |
| E2E (WsSession) | `tests/ws-cli-e2e.rs` | 4 | WsSession + coordinator: list, status, stop, multi-request |
| **Total** | | **35** | |

### Key test patterns

**Poll-with-timeout**: Integration tests poll coordinator state (e.g., `DaemonConnected`) with a 2-second deadline and 20ms sleep intervals, avoiding flaky timing assumptions.

**No nested runtimes**: E2E tests run the coordinator on a background `std::thread` with its own tokio runtime, while `WsSession` (which creates its own current-thread runtime) runs on the test's main thread. This avoids the "cannot start a runtime from within a runtime" panic.

**u128 workaround in tests**: Daemon test helpers construct WsRequest JSON strings manually via `format!()` + `serde_json::to_string()` (not `serde_json::to_value()`) to preserve `uhlc::ID` u128 values on the wire.

**Test coordinator setup**: Both integration and E2E tests use `dora_coordinator::start_testing()` which binds to port 0 (OS-assigned) and accepts an empty external event stream.

---

## Configuration Reference

### Constants

| Constant | Value | File | Purpose |
|----------|-------|------|---------|
| `MAX_CONTROL_MESSAGE_BYTES` | 1 MiB (1,048,576) | `ws_server.rs` | Max WebSocket frame size |
| `MAX_WS_CONNECTIONS` | 256 | `ws_server.rs` | Tower concurrency limit |

### Server setup

```rust
// Production: called by coordinator's main startup
let (port, shutdown, future) = ws_server::serve(bind_addr, event_tx, clock).await?;
tokio::spawn(future);
// ...
shutdown.shutdown(); // graceful stop
```

### Test setup

```rust
// Binds to port 0, returns (port, future)
let (port, future) = dora_coordinator::start_testing(
    "127.0.0.1:0".parse().unwrap(),
    futures::stream::empty(),
).await?;
```

### Shutdown

`ShutdownTrigger` wraps a `oneshot::Sender<()>`. Calling `.shutdown()` sends the signal, which the Axum server receives via `with_graceful_shutdown`. In-flight requests complete; new connections are rejected.
