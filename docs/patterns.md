# Communication Patterns

Adora is a dataflow framework based on pub/sub message passing. On top of
basic topics, the framework supports **service** (request/reply) and **action**
(goal/feedback/result) patterns using well-known metadata keys. No changes to
the daemon, coordinator, or YAML syntax are required -- the patterns are
implemented as conventions at the node API level.

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

## 4. Choosing a pattern

| Need a response? | Long-running? | Cancelable? | Pattern |
|:-:|:-:|:-:|---------|
| No | - | - | **Topic** |
| Yes | No | No | **Service** |
| Yes | Yes | Optional | **Action** |

## 5. Python compatibility

Python nodes use the same metadata conventions. Parameters are plain dicts
with string keys:

```python
# Service client
params = {"request_id": str(uuid.uuid4())}
node.send_output("request", data, metadata={"parameters": params})

# Service server -- pass through parameters
node.send_output("response", result, metadata=event["metadata"])
```
