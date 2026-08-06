# Communication Patterns

Dora is a dataflow framework based on pub/sub message passing. On top of
basic topics, the framework supports **service** (request/reply), **action**
(goal/feedback/result), and **streaming** (session/segment/chunk) patterns
using well-known metadata keys. No changes to the daemon, coordinator, or
YAML syntax are required -- the patterns are implemented as conventions at
the node API level.

## 1. Topic (pub/sub)

The default pattern. A node publishes data on an output, and any node that
subscribes to that output receives it.

```yaml
nodes:
  - id: publisher
    outputs:
      - data
  - id: subscriber
    inputs:
      data: publisher/data
```

**Use when**: streaming sensor data, periodic status, fire-and-forget events.

## 2. Service (request/reply)

A client sends a request and expects exactly one response, correlated by a
`request_id` metadata key.

### Well-known metadata keys

| Key | Constant | Description |
|-----|----------|-------------|
| `request_id` | `dora_node_api::REQUEST_ID` | UUID v7 correlating request and response |

### YAML

```yaml
nodes:
  - id: client
    inputs:
      tick: dora/timer/millis/500
      response: server/response
    outputs:
      - request

  - id: server
    inputs:
      request: client/request
    outputs:
      - response
```

### Node API helpers

```rust
// Client: send request with auto-generated request_id
let rid = node.send_service_request("request".into(), params, data)?;

// Server: pass through metadata.parameters (includes request_id)
node.send_service_response("response".into(), metadata.parameters, result)?;
```

The server MUST pass through the `request_id` from the incoming request's
metadata parameters into the response. The client matches responses to
requests using this key.

#### Waiting for a response with timeout + fault tolerance

Use [`EventStream::recv_service_response`](../apis/rust/node/src/event_stream/mod.rs)
to wait for a specific `request_id` with built-in handling of timeouts and
server restarts:

```rust
let rid = node.send_service_request("request".into(), params, data)?;
match events
    .recv_service_response(&rid, &server_node_id, Duration::from_secs(5))
    .await
{
    Ok(Event::Input { data, .. }) => handle_response(data),
    Err(PatternError::Timeout) => fallback_path(),
    Err(PatternError::ServerRestarted(server)) => {
        // The server instance crashed and was restarted by fault tolerance.
        // The in-flight request_id is orphaned; retry against the new instance.
        retry_with_new_instance()
    }
    Err(e) => return Err(e.into()),
    _ => unreachable!(),
}
```

Non-matching events arriving during the wait are buffered and replayed
on subsequent `recv()` calls, so your main event loop never loses
intermediate inputs, parameter updates, or lifecycle events.

**Example**: `examples/service-example/`

#### Clients that cannot block

`recv_service_response` waits. A single-threaded node with its own
schedule to keep — or several requests outstanding at once — cannot
afford that: the wait stalls everything else, and on a wedged server it
stalls for the whole timeout.

`try_recv_service_response` polls instead, returning `Ok(None)` when the
reply has not arrived (dora-rs/dora#3046). Buffering, restart detection,
correlation *and the deadline* are all handled as in the blocking form;
only the waiting is gone.

The framework owns the timeout: the first poll carrying one registers a
deadline for that `request_id`, and a later poll past it returns
`PatternError::Timeout` once. A caller passes the same timeout every
iteration and reacts to `Timeout` like any other outcome — it does not
sweep deadlines itself. Pass `None` to poll without one.

```rust
// once per loop iteration, for each outstanding request
let timeout = Some(Duration::from_secs(5));
match events.try_recv_service_response(&rid, ExpectedServers::One(&server), timeout) {
    Ok(Some(Event::Input { data, .. })) => complete(data),
    Ok(None) => {}                        // not ready — get on with the iteration
    Err(PatternError::Timeout) => give_up(),
    Err(e) => return Err(e.into()),
    _ => unreachable!(),
}
```

The clock starts at that first poll rather than at send time, and the
first deadline registered for an id wins.

A poll drops its own registration on a match, an error or expiry.
`cancel_correlation` releases one the node abandons and will never poll
again; without it that entry lives until the stream is dropped. It is
available in both Rust and C++ and is a no-op for an unknown id.

In C++ the timeout is `timeout_ms`, where `0` means *no deadline* rather
than "return immediately" — a poll never blocks, so there is nothing to
time out. That inverts the usual convention for a timeout argument.

> **Ordering matters.** The polls and your own `recv()` read the same
> stream, so whichever runs first consumes what is there. A poll
> correlates the reply it wants and buffers everything else for a later
> `recv()`, so polling first loses nothing. The reverse is not true: a
> reply consumed by `recv()` is gone, and no later poll can see it.
> Poll first, then drain your own events.

`try_recv_action_result` is the same for actions.

#### Fanning one request out to several servers

`send_service_request` mints a fresh `request_id` per call, so it cannot
express one logical request sent to several nodes. Use
`send_service_request_with_id` with a shared id, and await it with
`ExpectedServers::AnyOf`, which accepts whichever node answers first:

```rust
let request_id = DoraNode::new_request_id();
for server in &servers {
    node.send_service_request_with_id(
        output.clone(), params.clone(), data.clone(), request_id.clone(),
    )?;
}
let reply = events
    .recv_service_response_from(&request_id, ExpectedServers::AnyOf(&servers), timeout)
    .await?;
```

The server set only governs *restart* detection — which reply matches is
decided by `request_id` alone. `ExpectedServers::Any` skips restart
correlation entirely, for when the responder is not known up front.

A restart means different things to the two variants, and the deadline
follows that. For `One` it is terminal: the request is orphaned and its
deadline is dropped with it. For `AnyOf` it is only a notification — the
other candidates may still answer — so the correlation *and its
deadline* survive, still running on the original clock. That last part
matters: if a restart reset the clock, a node that keeps flapping would
hold the correlation open indefinitely and the caller would never see
`Timeout`.

## 3. Action (goal/feedback/result)

A client sends a goal and receives periodic feedback plus a final result.
Actions support cancellation.

### Well-known metadata keys

| Key | Constant | Description |
|-----|----------|-------------|
| `goal_id` | `dora_node_api::GOAL_ID` | UUID v7 identifying the goal |
| `goal_status` | `dora_node_api::GOAL_STATUS` | Final status of the goal |

Goal status values:

| Value | Constant | Meaning |
|-------|----------|---------|
| `succeeded` | `GOAL_STATUS_SUCCEEDED` | Goal completed successfully |
| `aborted` | `GOAL_STATUS_ABORTED` | Goal aborted by server |
| `canceled` | `GOAL_STATUS_CANCELED` | Goal canceled by client |

### YAML

```yaml
nodes:
  - id: client
    inputs:
      tick: dora/timer/millis/2000
      feedback: server/feedback
      result: server/result
    outputs:
      - goal
      - cancel

  - id: server
    inputs:
      goal: client/goal
      cancel: client/cancel
    outputs:
      - feedback
      - result
```

### Cancel pattern

The client sends a message on the `cancel` output with `goal_id` in the
metadata. The server checks for cancel requests between processing steps and
sends a result with `goal_status = "canceled"`.

### Waiting for a terminal result with timeout + fault tolerance

Use [`EventStream::recv_action_result`](../apis/rust/node/src/event_stream/mod.rs)
to wait for a terminal result (`goal_status` ∈ {`succeeded`, `aborted`,
`canceled`}) for a specific `goal_id`:

```rust
let goal_id = DoraNode::new_request_id();
let mut params = MetadataParameters::default();
params.insert(GOAL_ID.to_string(), Parameter::String(goal_id.clone()));
node.send_output("goal".into(), params, data)?;

match events
    .recv_action_result(&goal_id, &server_node_id, Duration::from_secs(30))
    .await
{
    Ok(Event::Input { metadata, data, .. }) => {
        // Inspect metadata.parameters for goal_status
        handle_terminal_result(metadata, data)
    }
    Err(PatternError::ServerRestarted(_)) => retry_with_new_instance(),
    Err(PatternError::Timeout) => give_up_and_cleanup(),
    Err(e) => return Err(e.into()),
    _ => unreachable!(),
}
```

Intermediate feedback events (matching `goal_id` without a terminal
`goal_status`) are passed through to the caller's main event loop, so
you can observe progress via `recv()` alongside the terminal wait.

**Example**: `examples/action-example/`

## 4. Streaming (session/segment/chunk)

For real-time pipelines (voice, video, sensor streams) where a user can
interrupt mid-stream and queued data must be discarded.

### Well-known metadata keys

| Key | Type | Constant | Description |
|-----|------|----------|-------------|
| `session_id` | String | `SESSION_ID` | Identifies the conversation/session |
| `segment_id` | Integer | `SEGMENT_ID` | Logical unit within a session (e.g. one utterance) |
| `seq` | Integer | `SEQ` | Chunk sequence number within a segment |
| `fin` | Bool | `FIN` | `true` on the last chunk of a segment |
| `flush` | Bool | `FLUSH` | `true` to discard older queued messages on this input |

### YAML

```yaml
nodes:
  - id: asr
    inputs:
      mic: mic-source/audio
    outputs:
      - text

  - id: llm
    inputs:
      text: asr/text
    outputs:
      - tokens

  - id: tts
    inputs:
      tokens: llm/tokens
    outputs:
      - audio
```

### Node API

```rust
use dora_node_api::{StreamSegment, DoraNode};

let mut seg = StreamSegment::new();

// Send chunks with auto-incrementing seq (e.g. inside an ASR node)
node.send_stream_chunk("text".into(), &mut seg, false, chunk_data)?;
// Mark final chunk of a segment
node.send_stream_chunk("text".into(), &mut seg, true, last_chunk)?;

// On user interruption: flush downstream queues and start a new segment.
// The prior segment ends without a fin=true signal -- old data is discarded.
let flush_params = seg.flush();
node.send_output("text".into(), flush_params, empty_data)?;
```

### Queue flush behavior

When a message arrives with `flush: true` in its metadata, the
receiver's input queue is cleared of all older messages before the
flush message is delivered. This enables instant interruption in
voice pipelines -- when the user speaks over TTS output, the ASR node
sends a new segment with `flush: true`, and the TTS node immediately
discards any queued audio chunks from the previous response.

**Note**: flush discards *all* queued messages on the input regardless of
`session_id`. Do not multiplex independent sessions on a single input
when using flush.

### Python

```python
# Streaming metadata is a plain dict
params = {
    "session_id": session_id,
    "segment_id": 1,
    "seq": 0,
    "fin": False,
    "flush": True,  # flush older queued messages
}
node.send_output("text", data, metadata={"parameters": params})
```

## 5. Choosing a pattern

| Need a response? | Long-running? | Cancelable? | Real-time stream? | Pattern |
|:-:|:-:|:-:|:-:|---------|
| No | - | - | No | **Topic** |
| Yes | No | No | No | **Service** |
| Yes | Yes | Optional | No | **Action** |
| No | Yes | Via flush | Yes | **Streaming** |

## 6. Important details

- **`goal_status` matching is case-sensitive.** Always use the exact lowercase
  values: `"succeeded"`, `"aborted"`, `"canceled"`. The ROS2 bridge defaults
  to `Aborted` for unrecognised values.

### Fault tolerance for correlated patterns

The fault tolerance system (`restart_policy`, `input_timeout`) restarts
crashed nodes, but it does **not** synthesise per-correlation cancellation
messages. When a service-server or action-server restarts:

- In-flight `request_id` correlations are orphaned. The restarted server
  has no knowledge of pre-crash requests, and no cancellation is sent
  to waiting clients.
- Active `goal_id` state machines are left in a non-terminal state.
  Clients never receive `"aborted"` or `"canceled"` for the orphaned goals.
- The daemon emits `NodeRestarted { id }` to all downstream nodes. Clients
  can use this signal to fail pending correlations against that server.

**Recommended**: use `recv_service_response` / `recv_action_result` (shown
in §2 and §3 above). They:

1. Take a `timeout` so waits are bounded.
2. Watch for `NodeRestarted { id: expected_server }` and return
   `PatternError::ServerRestarted` so you can retry against the new
   instance without hanging.
3. Buffer non-matching events so your main event loop keeps working.

Alternatively, handle the fault manually:

```rust
while let Some(event) = events.recv() {
    match event {
        Event::Input { metadata, .. } if matches_my_request(&metadata) => break,
        Event::NodeRestarted { id } if id == server_node_id => {
            // orphaned — retry or surface to caller
            break;
        }
        _ => continue,
    }
}
```

A future release may add daemon-side synthesis of per-correlation
cancellations so clients without the helpers still get explicit
terminal events (tracked in dora-rs/adora#148).

## 7. Python compatibility

Python nodes use the same metadata conventions. Parameters are plain dicts
with string keys:

```python
import uuid

# Service client (uuid7 for time-ordered IDs, matching Rust API)
params = {"request_id": str(uuid.uuid7())}
node.send_output("request", data, metadata={"parameters": params})

# Service server -- pass through parameters
node.send_output("response", result, metadata=event["metadata"])
```

> **Note**: `uuid.uuid7()` requires Python 3.13+. On older versions, use the
> `uuid_utils` package or `uuid.uuid4()` (random v4 also works for correlation,
> but loses time-ordering).

## 8. C++ compatibility

The C++ node binding exposes the same primitives as the Rust API, generated
into `dora-node-api.h` (dora-rs/dora#2686).

| Rust | C++ |
|------|-----|
| `DoraNode::new_request_id` / `new_goal_id` | `new_request_id()` / `new_goal_id()` |
| `DoraNode::send_service_request` | `send_service_request(...)` / `send_arrow_service_request(...)` |
| `DoraNode::send_service_response` | `send_service_response(...)` |
| `DoraNode::send_service_request_with_id` | `send_service_request_with_id(...)` |
| `EventStream::recv_service_response` | `recv_service_response(...)` |
| `EventStream::recv_action_result` | `recv_action_result(...)` |
| `EventStream::try_recv_service_response` | `try_recv_service_response(...)` |
| `EventStream::try_recv_action_result` | `try_recv_action_result(...)` |
| `EventStream::recv_service_response_from` | `recv_service_response_from(...)` |
| `EventStream::recv_action_result_from` | `recv_action_result_from(...)` |
| `ExpectedServers::AnyOf` / `::Any` | a `Vec<String>` of node ids / an empty one |
| `GOAL_STATUS_SUCCEEDED` / `_ABORTED` / `_CANCELED` | `goal_status_succeeded()` / `_aborted()` / `_canceled()` |
| `PatternError` | `DoraPatternStatus` |

Reading correlation keys off an incoming message needs
`event_as_input_with_metadata` — the older `event_as_input` returns the
payload only, which is not enough for the server side of an exchange.

```cpp
// Client: send and await the correlated reply.
auto request = send_service_request(
    node.send_output, "request", payload, new_metadata());
if (!std::string(request.error).empty()) { /* send failed */ }

auto reply = recv_service_response(
    node.events, std::string(request.request_id), "server", 5000);
switch (reply.status) {
case DoraPatternStatus::Matched:
    handle(event_as_input(std::move(reply.event)));
    break;
case DoraPatternStatus::Timeout:
    fallback_path();
    break;
case DoraPatternStatus::ServerRestarted:
    // in-flight request_id is orphaned; retry against the new instance
    retry_with_new_instance();
    break;
default:
    std::cerr << std::string(reply.error) << std::endl;
}

// Server: echo the request's metadata back so request_id survives.
auto input = event_as_input_with_metadata(std::move(event));
send_service_response(
    node.send_output, "response", result, std::move(input.metadata));
```

A C++ node that cannot block uses `try_recv_service_response`, which
returns `DoraPatternStatus::NotReady` instead of waiting — the same
convention `try_next_event` already uses for plain events:

```cpp
// per loop iteration, for each outstanding request
auto poll = try_recv_service_response(node.events, request_id, "server", 5000);
switch (poll.status) {
case DoraPatternStatus::Matched:
    complete(event_as_input(std::move(poll.event)));
    break;
case DoraPatternStatus::NotReady:
    break; // nothing to do, and no time spent
case DoraPatternStatus::Timeout:
    give_up(); // the registered deadline lapsed; reported once
    break;
default:
    std::cerr << std::string(poll.error) << std::endl;
}
```

`timeout_ms` works as in Rust — registered once per `request_id` by the
framework, reported as `Timeout` when it lapses; pass `0` for no
deadline. The same ordering rule applies: poll before consuming events
yourself, or a reply your own `next_event` picked up is lost. Passing an
empty `server_node_id` accepts a reply from any node.

For a request fanned out to several servers, mint the id once with
`new_request_id()`, send each copy with `send_service_request_with_id`,
and await them with `recv_service_response_from` /
`try_recv_service_response_from`, which take a `Vec<String>` of
acceptable responders (empty means any).

**Example**: `examples/c++-service-action/nodes/polling-client.cc`

For actions, set `goal_id` on the metadata (`metadata->set_goal_id(...)`),
send with `send_output_with_metadata`, and wait with `recv_action_result`.
The server tags feedback with `goal_id` only, and the terminal message with
`goal_id` plus `goal_status`. As in Rust, feedback buffered during a
`recv_action_result` wait is replayed by later `next_event` calls, so the
main event loop still sees it.

Both helpers return `DoraPatternResult`, whose `event` field also carries a
matching `DoraEventType` (`Timeout` / `AllInputsClosed` / `Empty`), so code
can branch on either `status` or the event type. A malformed
`server_node_id` yields `DoraPatternStatus::InvalidArgument` rather
than aborting the process.

**Example**: `examples/c++-service-action/`
