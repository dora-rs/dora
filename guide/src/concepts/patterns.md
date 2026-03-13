# Communication Patterns

Adora is a dataflow framework based on pub/sub message passing. On top of
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
| `request_id` | `adora_node_api::REQUEST_ID` | UUID v7 correlating request and response |

### YAML

```yaml
nodes:
  - id: client
    inputs:
      tick: adora/timer/millis/500
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

**Example**: `examples/service-example/`

## 3. Action (goal/feedback/result)

A client sends a goal and receives periodic feedback plus a final result.
Actions support cancellation.

### Well-known metadata keys

| Key | Constant | Description |
|-----|----------|-------------|
| `goal_id` | `adora_node_api::GOAL_ID` | UUID v7 identifying the goal |
| `goal_status` | `adora_node_api::GOAL_STATUS` | Final status of the goal |

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
      tick: adora/timer/millis/2000
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
use adora_node_api::{StreamSegment, AdoraNode};

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
