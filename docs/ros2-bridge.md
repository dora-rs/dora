# ROS2 Bridge

Adora provides a declarative YAML-based ROS2 bridge that lets any Adora node communicate with ROS2 topics, services, and actions without importing ROS2 libraries. You define the bridge in your dataflow YAML using the `ros2:` key, and the framework automatically spawns a bridge binary that converts between Apache Arrow (Adora's native format) and ROS2 CDR/DDS. Your user nodes stay ROS2-free -- they send and receive pure Arrow `StructArray` data.

## Features at a Glance

| Feature | Config | Description |
|---------|--------|-------------|
| Topic subscribe | `topic` + `direction: subscribe` | Receive from ROS2, forward as Arrow |
| Topic publish | `topic` + `direction: publish` | Receive Arrow, publish to ROS2 |
| Multi-topic | `topics` | Multiple topics on a single ROS2 node |
| Service client | `service` + `role: client` | Send requests, receive responses |
| Service server | `service` + `role: server` | Receive requests, send responses |
| Action client | `action` + `role: client` | Send goals, receive feedback + result |
| Action server | `action` + `role: server` | Receive goals, send feedback + result |
| QoS policies | `qos` | Reliability, durability, history, liveliness |
| Auto-spawn | Automatic | Bridge binary spawned by daemon as a Custom node |

---

## Architecture

When the Adora descriptor resolver encounters a `ros2:` key on a node, it converts it into a `Custom` node pointing to the `adora-ros2-bridge-node` binary. The bridge config is serialized as JSON into the `ADORA_ROS2_BRIDGE_CONFIG` environment variable.

```
User Node <--(Arrow/SharedMem)--> Bridge Binary <--(CDR/DDS)--> ROS2
```

The bridge binary:

1. Reads `AMENT_PREFIX_PATH` to locate installed ROS2 message packages
2. Parses message/service/action definitions at startup
3. Creates a `ros2_client` node and the appropriate publishers, subscribers, clients, or servers
4. Converts incoming ROS2 CDR messages to Arrow `StructArray` (subscribe/response/feedback)
5. Converts incoming Arrow `StructArray` to ROS2 CDR messages (publish/request/goal)

Your user nodes never link against ROS2 -- all ROS2 communication is isolated in the bridge binary.

---

## Prerequisites

- **ROS2 environment sourced**: `AMENT_PREFIX_PATH` must be set and point to a workspace containing the required message packages
- **Message packages installed**: e.g., `turtlesim`, `geometry_msgs`, `example_interfaces`
- **For service client**: A ROS2 service server must be running (or use a companion server dataflow)
- **For action client**: A ROS2 action server must be running *before* starting the dataflow (no `wait_for_action_server` mechanism)
- **For action server**: A ROS2 action client sends goals to the bridge (e.g., `ros2 action send_goal`)

---

## Topic Bridge

### Single Topic (Subscribe)

Subscribe to a ROS2 topic and forward messages as Arrow data to downstream Adora nodes.

```yaml
nodes:
  - id: pose_bridge
    ros2:
      topic: /turtle1/pose
      message_type: turtlesim/Pose
      direction: subscribe       # default, can be omitted
    outputs:
      - pose
```

The bridge creates a ROS2 subscription on `/turtle1/pose`, deserializes each incoming `turtlesim/Pose` message into an Arrow `StructArray`, and sends it on the `pose` output.

### Single Topic (Publish)

Receive Arrow data from Adora nodes and publish to a ROS2 topic.

```yaml
nodes:
  - id: cmd_bridge
    ros2:
      topic: /turtle1/cmd_vel
      message_type: geometry_msgs/Twist
      direction: publish
    inputs:
      cmd_vel: planner/cmd_vel
```

The bridge receives Arrow data on the `cmd_vel` input, serializes it to `geometry_msgs/Twist` CDR, and publishes to `/turtle1/cmd_vel`.

### Multi-Topic

Bridge multiple topics on a single ROS2 node context, mixing subscribe and publish directions.

```yaml
nodes:
  - id: turtle_bridge
    ros2:
      topics:
        - topic: /turtle1/pose
          message_type: turtlesim/Pose
          direction: subscribe
          output: pose
        - topic: /turtle1/cmd_vel
          message_type: geometry_msgs/Twist
          direction: publish
          input: velocity
      qos:
        reliable: true
        keep_last: 10
    inputs:
      velocity: planner/cmd_vel
    outputs:
      - pose
```

Multi-topic mode supports up to 64 topics per bridge node.

### Input/Output ID Mapping

By default, topic names are converted to Adora IDs by stripping the leading `/` and replacing remaining `/` with `_`:

| ROS2 Topic | Default Adora ID |
|------------|------------------|
| `/turtle1/pose` | `turtle1_pose` |
| `/camera/image_raw` | `camera_image_raw` |

In multi-topic mode, you can override this with explicit `output` (for subscribe) or `input` (for publish) fields. In single-topic mode, the node's declared `outputs` or `inputs` are used directly.

---

## Service Bridge

### Service Client

Send requests from Adora to an external ROS2 service and receive responses.

```yaml
nodes:
  - id: add_client
    ros2:
      service: /add_two_ints
      service_type: example_interfaces/AddTwoInts
      role: client
    inputs:
      request: requester/data
    outputs:
      - response
```

The bridge waits for the service to become available (up to 10 retries, 2 seconds each), then for each Arrow input it receives:

1. Serializes the Arrow data as an `AddTwoInts_Request` CDR message
2. Sends the request to the ROS2 service
3. Waits for a response (30-second timeout)
4. Deserializes the response into Arrow and sends it on the `response` output

### Service Server

Expose an Adora handler node as a ROS2 service that external ROS2 clients can call.

```yaml
nodes:
  - id: add_server
    ros2:
      service: /adora_add_two_ints
      service_type: example_interfaces/AddTwoInts
      role: server
    inputs:
      response: handler/result
    outputs:
      - request

  - id: handler
    path: path/to/handler-node
    inputs:
      request: add_server/request
    outputs:
      - result
```

The bridge receives ROS2 service requests, assigns each a unique `request_id` (UUID v7), forwards the request data as Arrow on the `request` output with `request_id` in metadata, and waits for the handler node to send a response back on the `response` input with the same `request_id`. The response is then returned to the correct ROS2 client.

See `examples/ros2-bridge/yaml-bridge-service/` for a working example.

### Request ID Correlation

Each incoming ROS2 request is assigned a `request_id` metadata parameter. The handler node **must include the same `request_id`** in metadata when sending the response. The simplest approach is to pass through `metadata.parameters`:

```rust
Event::Input { id, metadata, data } => {
    // metadata.parameters contains request_id
    let result = compute(data);
    node.send_service_response("response".into(), metadata.parameters, result)?;
}
```

Responses can arrive in any order -- the bridge correlates them by `request_id`, not by arrival order. Stale pending requests are evicted after 30 seconds. The maximum pending request queue is 64 -- additional requests are dropped when full.

### Service Wait and Timeouts

| Behavior | Value |
|----------|-------|
| Service client: wait for availability | 10 retries, 2s each (20s total) |
| Service client: response timeout | 30 seconds |
| Service server: pending request limit | 64 |

---

## Action Bridge

### Action Client

Send goals from Adora to an external ROS2 action server, receiving feedback and results.

```yaml
nodes:
  - id: fib_client
    ros2:
      action: /fibonacci
      action_type: example_interfaces/Fibonacci
      role: client
    inputs:
      goal: goal_sender/goal
    outputs:
      - feedback
      - result
```

For each Arrow goal input:

1. Serializes the Arrow data as a `Fibonacci_Goal` CDR message
2. Sends the goal to the action server (30-second timeout)
3. If accepted, spawns background threads for feedback and result
4. Feedback messages arrive on the `feedback` output as they stream in
5. The final result arrives on the `result` output (5-minute timeout)

### Feedback and Result Streams

The action bridge sends feedback and results on separate outputs:

- **`feedback`**: Streamed as each feedback message arrives from the action server. Contains the action's feedback message as Arrow (e.g., `{partial_sequence: int32[]}` for Fibonacci)
- **`result`**: Sent once when the action completes. Contains the action's result message as Arrow (e.g., `{sequence: int32[]}` for Fibonacci)

### Concurrent Goals

The bridge supports up to 8 concurrent in-flight goals (`MAX_CONCURRENT_GOALS`). Additional goals are dropped with a warning. Each goal spawns dedicated feedback and result reader threads.

### Timeouts

| Behavior | Value |
|----------|-------|
| Goal send timeout | 30 seconds |
| Result retrieval timeout | 5 minutes |
| Feedback | No timeout (streams until action completes) |

### Action Server

Expose an Adora handler node as a ROS2 action server that external ROS2 clients can call.

```yaml
nodes:
  - id: fib_server
    ros2:
      action: /fibonacci
      action_type: example_interfaces/Fibonacci
      role: server
    inputs:
      feedback: handler/feedback
      result: handler/result
    outputs:
      - goal

  - id: handler
    path: path/to/handler-node
    inputs:
      goal: fib_server/goal
    outputs:
      - feedback
      - result
```

The bridge receives goals from ROS2 clients, auto-accepts them, and forwards the goal data on the `goal` output. The handler computes feedback and results and sends them back on the `feedback` and `result` inputs.

See `examples/ros2-bridge/yaml-bridge-action-server/` for a working Fibonacci example.

### Goal ID Metadata

Each goal is identified by a UUID string passed as a `goal_id` metadata parameter. The bridge sets `goal_id` on every `goal` output. The handler **must include the same `goal_id`** in metadata when sending `feedback` and `result` so the bridge can correlate them to the correct goal.

The simplest approach is to pass through `metadata.parameters` from the goal event:

```rust
Event::Input { id, metadata, data } => match id.as_str() {
    "goal" => {
        let params = metadata.parameters; // contains goal_id
        // ... compute ...
        node.send_output("feedback".into(), params.clone(), feedback)?;
        node.send_output("result".into(), params, result)?;
    }
    // ...
}
```

### Action Server Lifecycle

1. ROS2 client sends a goal request
2. Bridge auto-accepts the goal and starts executing
3. Bridge sends goal data on `goal` output with `goal_id` in metadata
4. Handler sends `feedback` (zero or more times) with same `goal_id`
5. Handler sends `result` (once) with same `goal_id`; bridge returns it to the ROS2 client
6. Result send times out after 5 minutes if the client never requests it

Goals that contain no data or cannot be forwarded to the handler are automatically aborted -- the bridge sends `Aborted` status back to the ROS2 client so it does not hang indefinitely.

### Goal Status

By default, results are returned with `Succeeded` status. The handler can override this by setting a `goal_status` metadata parameter on the result output:

| `goal_status` value | ROS2 Status | Use case |
|---------------------|-------------|----------|
| `"succeeded"` (or omitted) | `Succeeded` | Goal completed successfully |
| `"aborted"` | `Aborted` | Goal failed during execution |
| `"canceled"` | `Canceled` | Goal was canceled by the handler |

Unrecognized `goal_status` values default to `Aborted` with a warning logged. Omitting `goal_status` entirely defaults to `Succeeded`.

Rust example:
```rust
use adora_node_api::{GOAL_STATUS, GOAL_STATUS_ABORTED, Parameter};

let mut params = metadata.parameters; // contains goal_id
params.insert(GOAL_STATUS.to_string(), Parameter::String(GOAL_STATUS_ABORTED.to_string()));
node.send_output("result".into(), params, error_result)?;
```

### Action Server Limits

| Behavior | Value |
|----------|-------|
| Max concurrent goals | 8 (additional goals receive `Aborted` status) |
| Auto-accept | All goals are auto-accepted |
| Result send timeout | 5 minutes |

### Python Action Server Handler

Python nodes receive goal data as PyArrow arrays with `goal_id` in the metadata dictionary. Pass it through on feedback/result outputs:

```python
for event in node:
    if event["type"] == "INPUT" and event["id"] == "goal":
        goal_id = event["metadata"]["goal_id"]
        order = event["value"]["order"][0].as_py()

        # Send feedback
        node.send_output("feedback", feedback_array, {"goal_id": goal_id})

        # Send result (with optional status)
        node.send_output("result", result_array, {
            "goal_id": goal_id,
            "goal_status": "succeeded",  # or "aborted", "canceled"
        })
```

### C++ Action Server Handler

C++ nodes access `goal_id` via type-safe metadata accessors:

```cpp
auto goal_id = metadata->get_str("goal_id");

// Send feedback with goal_id
auto fb_metadata = new_metadata();
fb_metadata->set_string("goal_id", goal_id);
send_arrow_output_with_metadata("feedback", feedback_data, fb_metadata);

// Send result with goal_id
auto res_metadata = new_metadata();
res_metadata->set_string("goal_id", goal_id);
send_arrow_output_with_metadata("result", result_data, res_metadata);
```

---

## Quality of Service (QoS)

### Configuration

Set QoS at the bridge level (applies to all topics/channels) or per-topic in multi-topic mode.

```yaml
nodes:
  - id: my_bridge
    ros2:
      topic: /sensor/data
      message_type: sensor_msgs/LaserScan
      qos:
        reliable: true
        durability: transient_local
        keep_last: 10
        liveliness: automatic
        lease_duration: 5.0
        max_blocking_time: 0.5
```

### Defaults

| Field | Default |
|-------|---------|
| `reliable` | `false` (best effort) |
| `durability` | `volatile` |
| `liveliness` | `automatic` |
| `lease_duration` | infinity |
| `max_blocking_time` | 100ms (only applies when `reliable: true`) |
| `keep_last` | `1` |
| `keep_all` | `false` |

### Per-Topic QoS Override

In multi-topic mode, each topic can override the bridge-level QoS:

```yaml
ros2:
  topics:
    - topic: /fast_sensor
      message_type: sensor_msgs/Imu
      direction: subscribe
      qos:
        reliable: false          # override: best effort for this topic
        keep_last: 1
    - topic: /cmd
      message_type: geometry_msgs/Twist
      direction: publish
      # inherits bridge-level QoS (reliable: true)
  qos:
    reliable: true               # default for all topics
    keep_last: 10
```

### Validation Rules

| Field | Valid Values |
|-------|-------------|
| `reliable` | `true`, `false` |
| `durability` | `"volatile"`, `"transient_local"` |
| `liveliness` | `"automatic"`, `"manual_by_participant"`, `"manual_by_topic"` |
| `keep_last` | `1` to `10000` |
| `keep_all` | `true`, `false` (mutually exclusive intent with `keep_last`) |
| `lease_duration` | Finite non-negative float (seconds) |
| `max_blocking_time` | Finite non-negative float (seconds) |

---

## Data Format: Arrow Structs

All data exchanged between your nodes and the bridge uses Arrow `StructArray` with a single row. Each field in the ROS2 message becomes a column in the struct.

### How to Build Arrow Messages

Rust example: building an `AddTwoInts_Request` (`{a: i64, b: i64}`):

```rust
use std::sync::Arc;
use arrow::array::{Array, Int64Array, StructArray};
use arrow::datatypes::{DataType, Field};

fn make_add_request(a: i64, b: i64) -> StructArray {
    let fields = vec![
        Arc::new(Field::new("a", DataType::Int64, false)),
        Arc::new(Field::new("b", DataType::Int64, false)),
    ];
    let arrays: Vec<Arc<dyn Array>> = vec![
        Arc::new(Int64Array::from(vec![a])),
        Arc::new(Int64Array::from(vec![b])),
    ];
    StructArray::try_new(fields.into(), arrays, None)
        .expect("failed to create struct array")
}
```

Reading a response (`{sum: i64}`):

```rust
use arrow::array::{Int64Array, StructArray};

fn read_response(data: &dyn arrow::array::Array) -> i64 {
    let struct_array = data
        .as_any()
        .downcast_ref::<StructArray>()
        .expect("expected struct array");
    struct_array
        .column_by_name("sum")
        .expect("missing 'sum' field")
        .as_any()
        .downcast_ref::<Int64Array>()
        .expect("expected Int64Array")
        .value(0)
}
```

### Mapping ROS2 Types to Arrow Types

| ROS2 Type | Arrow Type | Rust Arrow Array |
|-----------|-----------|-----------------|
| `bool` | `Boolean` | `BooleanArray` |
| `int8` | `Int8` | `Int8Array` |
| `int16` | `Int16` | `Int16Array` |
| `int32` | `Int32` | `Int32Array` |
| `int64` | `Int64` | `Int64Array` |
| `uint8` / `byte` / `char` | `UInt8` | `UInt8Array` |
| `uint16` | `UInt16` | `UInt16Array` |
| `uint32` | `UInt32` | `UInt32Array` |
| `uint64` | `UInt64` | `UInt64Array` |
| `float32` | `Float32` | `Float32Array` |
| `float64` | `Float64` | `Float64Array` |
| `string` | `Utf8` | `StringArray` |
| `wstring` | `Utf8` (encoded as UTF-16 on CDR side) | `StringArray` |
| Nested message | `Struct` | `StructArray` |

### Sequences and Arrays

| ROS2 Type | Arrow Type | Rust Arrow Array |
|-----------|-----------|-----------------|
| Variable-length sequence (`int32[]`) | `List` | `ListArray` |
| Bounded sequence (`int32[<=10]`) | `List` (length validated) | `ListArray` |
| Fixed-size array (`int32[3]`) | `FixedSizeList` | `FixedSizeListArray` |

Example: reading a `ListArray` from Fibonacci feedback (`{partial_sequence: int32[]}`):

```rust
use arrow::array::{Int32Array, ListArray, StructArray};

let struct_array = data.as_any().downcast_ref::<StructArray>().unwrap();
let list = struct_array
    .column_by_name("partial_sequence")
    .unwrap()
    .as_any()
    .downcast_ref::<ListArray>()
    .unwrap();
let values = list
    .value(0)
    .as_any()
    .downcast_ref::<Int32Array>()
    .unwrap()
    .values()
    .to_vec();
```

---

## Complete YAML Reference

```yaml
nodes:
  - id: my_bridge
    ros2:
      # --- Mode (exactly one required) ---

      # Single topic mode
      topic: /topic_name               # ROS2 topic name
      message_type: package/TypeName    # ROS2 message type
      direction: subscribe             # subscribe (default) | publish

      # Multi-topic mode (mutually exclusive with topic)
      topics:
        - topic: /topic_a
          message_type: package/TypeA
          direction: subscribe
          output: custom_output_id     # override default ID mapping
          qos:                         # per-topic QoS override
            reliable: true
        - topic: /topic_b
          message_type: package/TypeB
          direction: publish
          input: custom_input_id       # override default ID mapping

      # Service mode (mutually exclusive with topic/topics/action)
      service: /service_name           # ROS2 service name
      service_type: package/TypeName   # ROS2 service type
      role: client                     # client | server

      # Action mode (mutually exclusive with topic/topics/service)
      action: /action_name             # ROS2 action name
      action_type: package/TypeName    # ROS2 action type
      role: client                     # client | server

      # --- QoS (optional, applies to all channels) ---
      qos:
        reliable: false                # true | false (default: false = best effort)
        durability: volatile           # volatile (default) | transient_local
        liveliness: automatic          # automatic | manual_by_participant | manual_by_topic
        lease_duration: 5.0            # seconds (default: infinity)
        max_blocking_time: 0.1         # seconds (default: 0.1, reliable only)
        keep_last: 1                   # 1-10000 (default: 1)
        keep_all: false                # true | false (default: false)

      # --- Optional ROS2 node config ---
      namespace: /                     # ROS2 namespace (default: "/")
      node_name: my_ros_node           # ROS2 node name (default: adora node id)

    # --- Standard Adora node fields ---
    inputs:
      input_id: source_node/output_id
    outputs:
      - output_id
```

---

## Use Case Scenarios

### 1. Subscribe to Sensor Data (turtlesim pose)

```yaml
nodes:
  - id: pose_bridge
    ros2:
      topic: /turtle1/pose
      message_type: turtlesim/Pose
    outputs:
      - pose

  - id: my_processor
    path: ./target/debug/my-processor
    inputs:
      pose: pose_bridge/pose
```

```rust
// In my_processor: receive turtlesim/Pose as Arrow
Event::Input { id, data, .. } if id.as_str() == "pose" => {
    let s = data.as_any().downcast_ref::<StructArray>().unwrap();
    let x = s.column_by_name("x").unwrap()
        .as_any().downcast_ref::<Float32Array>().unwrap().value(0);
    let y = s.column_by_name("y").unwrap()
        .as_any().downcast_ref::<Float32Array>().unwrap().value(0);
    println!("Turtle at ({x}, {y})");
}
```

### 2. Publish Velocity Commands

```yaml
nodes:
  - id: planner
    path: ./target/debug/planner
    inputs:
      tick: adora/timer/millis/100
    outputs:
      - cmd_vel

  - id: cmd_bridge
    ros2:
      topic: /turtle1/cmd_vel
      message_type: geometry_msgs/Twist
      direction: publish
    inputs:
      cmd_vel: planner/cmd_vel
```

```rust
// In planner: send geometry_msgs/Twist as Arrow
// Twist has nested Vector3 fields: linear {x,y,z} and angular {x,y,z}
fn make_twist(linear_x: f64, angular_z: f64) -> StructArray {
    let vec3_fields = vec![
        Arc::new(Field::new("x", DataType::Float64, false)),
        Arc::new(Field::new("y", DataType::Float64, false)),
        Arc::new(Field::new("z", DataType::Float64, false)),
    ];
    let linear = StructArray::try_new(
        vec3_fields.clone().into(),
        vec![
            Arc::new(Float64Array::from(vec![linear_x])) as _,
            Arc::new(Float64Array::from(vec![0.0])) as _,
            Arc::new(Float64Array::from(vec![0.0])) as _,
        ],
        None,
    ).unwrap();
    let angular = StructArray::try_new(
        vec3_fields.into(),
        vec![
            Arc::new(Float64Array::from(vec![0.0])) as _,
            Arc::new(Float64Array::from(vec![0.0])) as _,
            Arc::new(Float64Array::from(vec![angular_z])) as _,
        ],
        None,
    ).unwrap();

    let fields = vec![
        Arc::new(Field::new("linear", linear.data_type().clone(), false)),
        Arc::new(Field::new("angular", angular.data_type().clone(), false)),
    ];
    StructArray::try_new(
        fields.into(),
        vec![Arc::new(linear) as _, Arc::new(angular) as _],
        None,
    ).unwrap()
}
```

### 3. Multi-Topic Bidirectional Bridge

Subscribe to pose and publish velocity on a single ROS2 node.

```yaml
nodes:
  - id: turtle_bridge
    ros2:
      topics:
        - topic: /turtle1/pose
          message_type: turtlesim/Pose
          direction: subscribe
          output: pose
        - topic: /turtle1/cmd_vel
          message_type: geometry_msgs/Twist
          direction: publish
          input: velocity
      qos:
        reliable: true
        keep_last: 10
    inputs:
      velocity: planner/cmd_vel
    outputs:
      - pose

  - id: planner
    path: ./target/debug/planner
    inputs:
      pose: turtle_bridge/pose
      tick: adora/timer/millis/100
    outputs:
      - cmd_vel
```

### 4. Service Client: Call an External ROS2 Service

```yaml
nodes:
  - id: requester
    path: ./target/debug/requester
    inputs:
      tick: adora/timer/millis/1000
      response: add_client/response
    outputs:
      - request

  - id: add_client
    ros2:
      service: /add_two_ints
      service_type: example_interfaces/AddTwoInts
      role: client
    inputs:
      request: requester/request
    outputs:
      - response
```

Prerequisites: run a ROS2 service first:
```bash
ros2 run examples_rclcpp_minimal_service service_main
```

### 5. Service Server: Expose an Adora Handler as ROS2 Service

```yaml
nodes:
  - id: add_server
    ros2:
      service: /add_two_ints
      service_type: example_interfaces/AddTwoInts
      role: server
    inputs:
      response: handler/response
    outputs:
      - request

  - id: handler
    path: ./target/debug/handler
    inputs:
      request: add_server/request
    outputs:
      - response
```

The handler receives `{a: i64, b: i64}` as Arrow, computes the result, and sends `{sum: i64}` back. External ROS2 clients can call this service:
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"
```

### 6. Action Client: Long-Running Fibonacci Goal

```yaml
nodes:
  - id: goal_sender
    path: ./target/debug/goal-sender
    inputs:
      tick: adora/timer/millis/5000
      feedback: fib_client/feedback
      result: fib_client/result
    outputs:
      - goal

  - id: fib_client
    ros2:
      action: /fibonacci
      action_type: example_interfaces/Fibonacci
      role: client
    inputs:
      goal: goal_sender/goal
    outputs:
      - feedback
      - result
```

Prerequisites: start the action server **before** the dataflow:
```bash
ros2 run examples_rclcpp_action_server fibonacci_action_server
```

The goal node sends `{order: int32}`, receives streamed `{partial_sequence: int32[]}` feedback, and a final `{sequence: int32[]}` result.

### 7. Action Server: Expose an Adora Handler as ROS2 Action

```yaml
nodes:
  - id: fib_server
    ros2:
      action: /fibonacci
      action_type: example_interfaces/Fibonacci
      role: server
    inputs:
      feedback: handler/feedback
      result: handler/result
    outputs:
      - goal

  - id: handler
    path: ./target/debug/handler
    inputs:
      goal: fib_server/goal
    outputs:
      - feedback
      - result
```

The handler receives `{order: int32}` goals with a `goal_id` in metadata, sends `{partial_sequence: int32[]}` feedback, and a final `{sequence: int32[]}` result -- all with the same `goal_id` in metadata. External ROS2 clients can send goals:
```bash
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}"
```

---

## Limitations and Known Constraints

- **Action server auto-accept**: All incoming goals are automatically accepted. The handler cannot reject goals before execution starts.
- **No action cancel support**: Neither client nor server handles ROS2 cancel requests.
- **No `wait_for_action_server`**: The `ros2_client` library does not provide this API. Start the action server before the dataflow. The first goal will time out (30s) if the server is unavailable.
- **Single-flight service client**: The service client processes requests sequentially -- each request blocks until the response arrives (or times out at 30s).
- **QoS uniform for service/action channels**: The `qos` config applies to all service/action sub-channels (goal, result, cancel, feedback, status). Per-channel QoS is not configurable.
- **`AMENT_PREFIX_PATH` required**: The bridge fails at startup if no ROS2 message definitions are found.
- **Max 64 topics**: Multi-topic mode supports at most 64 topics per bridge node.
- **Max 8 concurrent action goals**: Additional goals receive `Aborted` status when the limit is reached.
- **Max 64 pending service requests (server)**: Requests are dropped when the queue is full.

---

## Best Practices

**Source your ROS2 environment before running.** Ensure `AMENT_PREFIX_PATH` is set and includes all required message packages. The bridge logs an error if no definitions are found.

**Start action servers before the dataflow.** There is no wait mechanism for action servers. If the server is not ready, the first goal send will time out after 30 seconds.

**Use multi-topic mode for related topics.** Bridging `/turtle1/pose` (subscribe) and `/turtle1/cmd_vel` (publish) on the same bridge node reduces resource usage compared to two separate bridge nodes.

**Match Arrow field names exactly.** The bridge validates that Arrow struct field names match the ROS2 message definition. Missing fields use default values (zero for numbers, empty string). Extra fields cause an error.

**Use explicit `output`/`input` in multi-topic mode.** Default ID mapping (stripping `/`, replacing `/` with `_`) can be confusing for deep topic names. Explicit IDs make the dataflow YAML self-documenting.

**Set QoS to match the ROS2 publisher/subscriber.** QoS mismatches (e.g., reliable subscriber with best-effort publisher) cause silent communication failures. Check with `ros2 topic info -v /topic_name` to see the existing QoS settings.

**Pass through `request_id` in service responses.** The bridge correlates responses to requests using the `request_id` metadata parameter. If the handler does not include `request_id` in the response metadata, the bridge cannot match the response to the original ROS2 request.
