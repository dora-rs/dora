# WebSocket Topic Data Channel

The topic data channel extends the WebSocket control plane to proxy live dataflow messages from the coordinator to CLI clients. Instead of requiring direct Zenoh network access, CLI commands like `topic echo`, `topic hz`, and `topic info` receive message data over the existing WebSocket connection as binary frames.

## Motivation

| Scenario | Before (Zenoh direct) | After (WS proxy) |
|----------|----------------------|-------------------|
| CLI on same machine as daemon | Works | Works |
| CLI remote, Zenoh reachable | Works | Works |
| CLI remote, no Zenoh access | Fails | Works |
| Browser-based web UI | Impossible | Possible |
| Embedded target, no local disk | Cannot record locally | `--proxy` streams to CLI |

The key insight: CLI and future web UIs connect to the coordinator via WebSocket. By having the coordinator subscribe to Zenoh on their behalf and forward messages as binary frames, topic inspection works anywhere the WebSocket connection reaches.

---

## Architecture

```
CLI  ──── WS (binary frames) ────>  Coordinator  ──── Zenoh sub ────>  Daemon
                                    (Zenoh proxy)                      (debug publish)
```

The coordinator acts as a Zenoh proxy:

1. CLI sends a `TopicSubscribe` request over the existing text-frame WS protocol
2. Coordinator validates the dataflow and opens Zenoh subscribers
3. Coordinator forwards each Zenoh sample as a binary WS frame back to the CLI
4. CLI dispatches binary frames by subscription UUID to the appropriate consumer

### Key source files

| File | Role |
|------|------|
| `libraries/message/src/cli_to_coordinator.rs` | `TopicSubscribe`, `TopicUnsubscribe` request variants |
| `libraries/message/src/coordinator_to_cli.rs` | `TopicSubscribed` reply variant |
| `binaries/coordinator/src/ws_control.rs` | Zenoh proxy: subscribe, forward binary frames |
| `binaries/coordinator/src/control.rs` | `ControlEvent::TopicSubscribe` for validation |
| `binaries/cli/src/ws_client.rs` | `WsSession::subscribe_topics()`, binary frame dispatch |
| `binaries/cli/src/command/topic/echo.rs` | Topic echo via WS |
| `binaries/cli/src/command/topic/hz.rs` | Topic frequency measurement via WS |
| `binaries/cli/src/command/topic/info.rs` | Topic metadata/stats via WS |
| `binaries/cli/src/command/record.rs` | `--proxy` flag for WS-based recording |

---

## Wire Protocol

### Subscription handshake (JSON text frames)

The subscription uses the existing UUID-correlated request-reply protocol:

**Request** (CLI -> Coordinator):
```json
{
  "id": "abc-123",
  "method": "control",
  "params": {
    "TopicSubscribe": {
      "dataflow_id": "550e8400-...",
      "topics": [["camera_node", "image"], ["lidar_node", "points"]]
    }
  }
}
```

**Response** (Coordinator -> CLI):
```json
{
  "id": "abc-123",
  "result": {
    "TopicSubscribed": {
      "subscription_id": "7f1b3a00-..."
    }
  }
}
```

**Unsubscribe** (CLI -> Coordinator):
```json
{
  "id": "def-456",
  "method": "control",
  "params": {
    "TopicUnsubscribe": {
      "subscription_id": "7f1b3a00-..."
    }
  }
}
```

### Binary data frames

After the handshake, the coordinator pushes binary WS frames. Each frame has a fixed-size header:

```
 0                   16                              N
 ├───────────────────┼──────────────────────────────┤
 │  subscription_id  │  Timestamped<InterDaemonEvent>│
 │  (16 bytes UUID)  │  (bincode serialized)         │
 └───────────────────┴──────────────────────────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| `subscription_id` | 16 bytes | UUID matching the `TopicSubscribed` ack, for multiplexing |
| payload | variable | Raw `Timestamped<InterDaemonEvent>` bincode bytes from Zenoh |

The 16-byte UUID prefix allows multiplexing multiple subscriptions on a single WS connection without additional framing overhead.

---

## Data Flow

```
CLI                         WsSession                     Coordinator
 │                              │                              │
 │── subscribe_topics() ───────>│                              │
 │                              │── WsRequest{TopicSubscribe} >│
 │                              │                              │ validate dataflow
 │                              │                              │ open Zenoh session (lazy)
 │                              │                              │ spawn subscriber tasks
 │                              │<── WsResponse{TopicSubscribed}│
 │<── (sub_id, data_rx) ───────│                              │
 │                              │                              │
 │                              │       ┌── Zenoh sample ──────│ Daemon publishes
 │                              │<──────│ Binary frame         │
 │<── data_rx.recv() ──────────│       │ (sub_id + payload)   │
 │                              │       │                      │
 │                              │<──────│ Binary frame         │
 │<── data_rx.recv() ──────────│       │                      │
 │                              │       └                      │
 │                              │                              │
 │   (drop session) ───────────>│── Close ────────────────────>│ abort subscriber tasks
```

### Coordinator internals

1. **Validation**: `ControlEvent::TopicSubscribe` is sent to the coordinator event loop, which checks that the dataflow exists and has `enable_debug_inspection: true` enabled
2. **Lazy Zenoh**: The coordinator's Zenoh session is opened on the first `TopicSubscribe` request and reused for subsequent subscriptions on the same WS connection
3. **Per-topic tasks**: Each `(node_id, data_id)` pair spawns a tokio task that subscribes to the corresponding Zenoh topic and forwards samples to the binary frame channel
4. **Backpressure**: The binary frame channel has capacity 64. `try_send` is used -- if the channel is full (slow consumer), samples are silently dropped rather than blocking the Zenoh subscriber
5. **Cleanup**: When the WS connection closes, all subscriber tasks are aborted

### WsSession (CLI side)

The `WsSession::subscribe_topics()` method:

1. Serializes a `TopicSubscribe` request
2. Sends `SessionCommand::SubscribeTopics` through the internal command channel
3. The async `session_loop` wraps it as a `WsRequest` and sends it
4. On receiving the `TopicSubscribed` ack, registers the `data_tx` sender in `topic_subscribers` keyed by `subscription_id`
5. Binary frames are dispatched by extracting the first 16 bytes as UUID and sending the remainder to the matching `data_tx`

State maintained in `session_loop`:
- `pending_topic_subscribes: HashMap<Uuid, (ack_tx, data_tx)>` -- awaiting ack
- `topic_subscribers: HashMap<Uuid, Sender>` -- active subscriptions receiving binary data

---

## Prerequisites

The dataflow descriptor must enable debug message publishing:

```yaml
_unstable_debug:
  enable_debug_inspection: true
```

Without this, the coordinator rejects the `TopicSubscribe` with:
```
dataflow {id} not found or enable_debug_inspection not enabled
```

---

## CLI Commands

### `dora topic echo`

Stream topic data to the terminal in real-time.

```bash
# Echo a single topic
dora topic echo -d my-dataflow camera_node/image

# Echo multiple topics
dora topic echo -d my-dataflow robot1/pose robot2/vel

# JSON output for piping
dora topic echo -d my-dataflow robot1/pose --format json
```

Internally: calls `session.subscribe_topics()`, receives `Timestamped<InterDaemonEvent>` from the `data_rx` channel, deserializes Arrow data, and renders as table or JSON.

### `dora topic hz`

Interactive TUI displaying per-topic publish frequency statistics.

```bash
# All topics
dora topic hz -d my-dataflow --window 10

# Specific topics
dora topic hz -d my-dataflow robot1/pose robot2/vel --window 5
```

Uses ratatui for the TUI. A background `std::thread` receives events from `data_rx` and dispatches to per-topic `HzStats` trackers via a `BTreeMap<(node_id, data_id), index>` lookup.

### `dora topic info`

One-shot topic metadata and statistics.

```bash
dora topic info -d my-dataflow camera_node/image --duration 5
```

Collects messages for `--duration` seconds, then displays type information, publisher, subscribers (from descriptor), message count, and bandwidth.

### `dora record --proxy`

Stream dataflow data through WebSocket for local recording.

```bash
# Start dataflow first
dora start dataflow.yml --detach

# Record via proxy (data streams through coordinator to CLI)
dora record dataflow.yml --proxy -o capture.adorec

# Record specific topics
dora record dataflow.yml --proxy --topics sensor/image,lidar/points
```

Use case: the target machine (running the daemon) has no local disk or limited storage. The `--proxy` flag routes data through the coordinator WebSocket to the CLI machine, where the `.adorec` file is written locally.

Without `--proxy` (default), a record node is injected into the dataflow and records directly on the daemon's machine.

---

## Zenoh Topic Format

The coordinator subscribes to Zenoh topics using the format from `dora_core::topics::zenoh_output_publish_topic()`:

```
dora/{dataflow_id}/{node_id}/{data_id}
```

Each topic carries `Timestamped<InterDaemonEvent>` as its payload, serialized with bincode. The coordinator forwards these bytes as-is (prepended with subscription UUID) -- no re-serialization.

---

## Backpressure and Performance

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Binary frame channel capacity | 64 | Balance between latency and memory |
| Drop policy | Drop on full | Prefer freshness over completeness |
| Binary format | Raw bincode (no base64) | Avoid 33% overhead for large payloads |

For high-throughput topics (camera images, point clouds), the binary frame channel may fill up if the WS connection is slow. Dropped samples are silent -- the CLI will show reduced frequency in `topic hz` but won't stall.

---

## Error Handling

| Error | Source | Response |
|-------|--------|----------|
| Dataflow not found | Coordinator validation | WsResponse with error message |
| `enable_debug_inspection` not enabled | Coordinator validation | WsResponse with error message |
| Zenoh session open failure | Coordinator | WsResponse with error message |
| Zenoh subscriber failure | Per-topic task | Warning log, task exits |
| Binary frame too short (<16 bytes) | CLI session_loop | Warning log, frame dropped |
| Unknown subscription UUID | CLI session_loop | Frame dropped silently |
| WS connection closed | Either side | All tasks aborted, pending acks get error |

---

## Test Coverage

| Tier | Location | What's covered |
|------|----------|----------------|
| Unit (client) | `binaries/cli/src/ws_client.rs` | `handle_response_topic_subscribe_ack` -- verifies ack routing and subscriber registration |
| Unit (all existing) | `binaries/cli/src/ws_client.rs` | Updated to pass topic subscribe state through `handle_response` |

The `TopicSubscribe` / binary frame path is primarily validated via integration testing with a running coordinator and Zenoh session. See [Testing Guide](testing-guide.md) for smoke test instructions.
