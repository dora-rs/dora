# Dora Node API for C++

Dora supports nodes written in C++ through this API crate.

## Build

- Clone the `dora` repository:
  ```bash
  > git clone https://github.com/dora-rs/dora.git
  > cd dora
  ```
- Build the `dora-node-api-cxx` package:
  ```bash
  cargo build --package dora-node-api-cxx
  ```
  - This will result in `dora-node-api.h` and `dora-node-api.cc` files in the `target/cxxbridge/dora-node-api-cxx` directory.
- Include the `dora-node-api.h` header file in your source file.
- Add the `dora-node-api.cc` file to your compile and link steps.

### Build with ROS2 Bridge

Dora features an experimental ROS2 Bridge that enables dora nodes to publish and subscribe to ROS2 topics.
To enable the bridge, use these steps:

- Clone the `dora` repository (see above).
- Source the ROS2 setup files (see [ROS2 docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files))
- Optional: Source package-specific ROS2 setup files if you want to use custom package-specific ROS2 messages in the bridge (see [ROS2 docs](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#source-the-setup-file))
- Build the `dora-node-api-cxx` package **with the `ros2-bridge` feature enabled**:
  ```bash
  cargo build --package dora-node-api-cxx --features ros2-bridge
  ```
  - In addition to the `dora-node-api.h` and `dora-node-api.cc` files, this will place a `dora-ros2-bindings.h` and a `dora-ros2-bindings.cc` file in the `target/cxxbridge/dora-node-api-cxx` directory.
- Include both the `dora-node-api.h` and the `dora-ros2-bindings.h` header files in your source file.
- Add the `dora-node-api.cc` and `dora-ros2-bindings.cc` files to your compile and link steps.

## Usage

The `dora-node-api.h` header provides various functions to interact with Dora.

### Init Dora Node

All nodes need to register themselves with Dora at startup.
To do that, call the `init_dora_node()` function.
The function returns a `DoraNode` instance, which gives access to dora events and enables sending Dora outputs.

```c++
auto dora_node = init_dora_node();
```

### Receiving Events

The `dora_node.events` field is a stream of incoming events.
To wait for the next incoming event, call `dora_node.events->next()`:

```c++
auto event = dora_node.events->next();
```

The `next` function returns an opaque `DoraEvent` type, which cannot be inspected from C++ directly.
Instead, use the following functions to read and destructure the event:

- `event_type(event)` returns a `DoraEventType`, which describes the kind of event. For example, an event could be an input or a stop instruction.
  - when receiving a `DoraEventType::AllInputsClosed`, the node should exit and not call `next` anymore
- Events of type `DoraEventType::Input` can be downcasted using `event_as_input`:
  ```c++
  auto input = event_as_input(std::move(event));
  ```
  The function returns a `DoraInput` instance, which has an `id` and `data` field.
  - The input `id` can be converted to a C++ string through `std::string(input.id)`.
  - The `data` of inputs is currently of type [`rust::Vec<uint8_t>`](https://cxx.rs/binding/vec.html). Use the provided methods for reading or converting the data.
    - **Note:** In the future, we plan to change the data type to the [Apache Arrow](https://arrow.apache.org/) data format to support typed inputs.

### Receiving Arrow Inputs

For Arrow-based inputs, you can use the following functions to receive typed data along with metadata:

- `event_as_arrow_input(event, out_array, out_schema)` - Returns only the Arrow array data (original function)
- `event_as_arrow_input_with_info(event, out_array, out_schema)` - Returns Arrow array data plus input ID and metadata as JSON

**Example using `event_as_arrow_input_with_info`:**

```c++
struct ArrowArray c_array;
struct ArrowSchema c_schema;

// Get Arrow array along with input ID and metadata
auto input_info = event_as_arrow_input_with_info(
    std::move(event),
    reinterpret_cast<uint8_t*>(&c_array),
    reinterpret_cast<uint8_t*>(&c_schema)
);

// Check for errors
if (!input_info.error.empty()) {
    std::cerr << "Error: " << input_info.error << std::endl;
    return;
}

// Access input ID
std::cout << "Input ID: " << input_info.id << std::endl;

// Access metadata via the provided helper type
auto metadata = std::move(input_info.metadata);
std::cout << "Metadata timestamp: " << metadata->timestamp() << std::endl;

auto keys = metadata->list_keys();
for (std::size_t i = 0; i < keys.size(); i++) {
    const std::string key(keys[i]);
    try {
        switch (metadata->type(key)) {
        case MetadataValueType::Float:
            std::cout << "Parameter " << key << " (float): "
                      << metadata->get_float(key) << std::endl;
            break;
        case MetadataValueType::Integer:
            std::cout << "Parameter " << key << " (int): "
                      << metadata->get_int(key) << std::endl;
            break;
        case MetadataValueType::String:
            std::cout << "Parameter " << key << " (string): "
                      << std::string(metadata->get_str(key)) << std::endl;
            break;
        case MetadataValueType::Bool:
            std::cout << "Parameter " << key << " (bool): "
                      << std::string(metadata->get_json(key)) << std::endl;
            break;
        default:
            std::cout << "Parameter " << key << " has unsupported type" << std::endl;
            break;
        }
    } catch (const std::exception& err) {
        std::cout << "Parameter " << key << " unavailable: " << err.what() << std::endl;
    }
}

std::cout << "Metadata as JSON: " << std::string(metadata->to_json()) << std::endl;

// Import the Arrow array for processing
auto result = arrow::ImportArray(&c_array, &c_schema);
std::shared_ptr<arrow::Array> input_array = result.ValueOrDie();
```

The metadata JSON contains:
- `timestamp` - Message timestamp with `secs` and `nanos` fields
- `type_info` - Arrow type information
- `parameters` - Custom metadata parameters (key-value pairs)

### Sending Outputs

Nodes can send outputs using the `send_output` output function and the `dora_node.send_output` field.
Note that all outputs need to be listed in the dataflow YAML declaration file, otherwise an error will occur.

**Example:**

```c++
// the data you want to send (NOTE: only byte vectors are supported right now)
std::vector<uint8_t> out_vec{42};
// create a Rust slice from the output vector
rust::Slice<const uint8_t> out_slice{out_vec.data(), out_vec.size()};
// send the slice as output
auto result = send_output(dora_node.send_output, "output_id", out_slice);

// check for errors
auto error = std::string(result.error);
if (!error.empty())
{
    std::cerr << "Error: " << error << std::endl;
    return -1;
}
```

## Using the ROS2 Bridge

The `dora-ros2-bindings.h` contains function and struct definitions that allow interacting with ROS2 nodes.
Currently, the bridge supports publishing and subscribing to ROS2 topics.
In the future, we plan to support ROS2 services and ROS2 actions as well.

### Initializing the ROS2 Context

The first step is to initialize a ROS2 context:

```c++
auto ros2_context = init_ros2_context();
```

### Creating Nodes

After initializing a ROS2 context, you can use it to create ROS2 nodes:

```c++
auto node = ros2_context->new_node("/ros2_demo", "turtle_teleop");
```

The first argument is the namespace of the node and the second argument is its name.

### Creating Topics

After creating a node, you can use one of the `create_topic_<TYPE>` functions to create a topic on it.
The `<TYPE>` describes the message type that will be sent on the topic.
The Dora ROS2 bridge automatically creates `create_topic_<TYPE>` functions for all messages types found in the sourced ROS2 environment.

```c++
auto vel_topic = node->create_topic_geometry_msgs_Twist("/turtle1", "cmd_vel", qos_default());
```

The first argument is the namespace of the topic and the second argument is its name.
The third argument is the QoS (quality of service) setting for the topic.
It can be adjusted as desired, for example:

```c++
auto qos = qos_default();
qos.durability = Ros2Durability::Volatile;
qos.liveliness = Ros2Liveliness::Automatic;
auto vel_topic = node->create_topic_geometry_msgs_Twist("/turtle1", "cmd_vel", qos);
```

### Publish

After creating a topic, it is possible to publish messages on it.
First, create a publisher:

```c++
auto vel_publisher = node->create_publisher(vel_topic, qos);
```

The returned publisher is typed by the chosen topic.
It will only accept messages of the topic's type, otherwise a compile error will occur.

After creating a publisher, you can use the `publish` function to publish one or more messages.
For example:

```c++
geometry_msgs::Twist twist = {
    .linear = {.x = 1, .y = 0, .z = 0},
    .angular = {.x = 0, .y = 0, .z = 0.5}
};
vel_publisher->publish(twist);
```

The `geometry_msgs::Twist` struct is automatically generated from the sourced ROS2 environment.
Since the publisher is typed, its `publish` method only accepts `geometry_msgs::Twist` messages.


### Subscriptions

Subscribing to a topic is possible through the `create_subscription` function on nodes:

```c++
auto pose_topic = node->create_topic_turtlesim_Pose("/turtle1", "pose", qos_default());
auto pose_subscription = node->create_subscription(pose_topic, qos_default(), event_stream);
```

The `topic` is the topic you want to subscribe to, created using a `create_topic_<TYPE>` function.
The second argument is the quality of service setting, which can be customized as described above.

The third parameter is the event stream that the received messages should be merged into.
Multiple subscriptions can be merged into the same event stream.

#### Combined Event Streams

Combined event streams enable the merging of multiple event streams into one.
The combined stream will then deliver messages from all sources, in order of arrival.

You can create such a event stream from Dora's event stream using the `dora_events_into_combined` function:

```c++
auto event_stream = dora_events_into_combined(std::move(dora_node.events));
```

Alternatively, if you don't want to use Dora, you can also create an empty event stream:

```c++
auto event_stream = empty_combined_events();
```

**Note:** You should only use `empty_combined_events` if you're running your executable independent of Dora.
Ignoring the events from the `dora_node.events` channel can result in unintended behavior.

#### Receiving Messages from Combined Event Stream

The merged event stream will receive all incoming events of the node, including Dora events and ROS2 messages.
To wait for the next incoming event, use its `next` method:

```c++
auto event = event_stream.next();
```

This returns a `event` instance of type `CombinedEvent`, which can be downcasted to Dora events or ROS2 messages.
To handle an event, you should check it's type and then downcast it:

- To check for a Dora event, you can use the `is_dora()` function. If it returns `true`, you can downcast the combined event to a Dora event using the `downcast_dora` function.
- ROS2 subscriptions support a `matches` function to check whether a combined event is an instance of the respective ROS2 subscription. If it returns true, you can downcast the event to the respective ROS2 message struct using the subscription's `downcast` function.

**Example:**

```c++
if (event.is_dora())
{
    auto dora_event = downcast_dora(std::move(event));
    // handle dora_event as described above
    auto ty = event_type(dora_event);
    if (ty == DoraEventType::Input)
    {
        auto input = event_as_input(std::move(dora_event));
        // etc
    }
    // .. else if
}
else if (pose_subscription->matches(event))
{
    auto pose = pose_subscription->downcast(std::move(event));
    std::cout << "Received pose x:" << pose.x << ", y:" << pose.y << std::endl;
}
else
{
    std::cout << "received unexpected event" << std::endl;
}
```

### Constants

Some ROS2 message definitions define constants, e.g. to specify the values of an enum-like integer field.
The Dora ROS2 bridge exposes these constants in the generated bindings as functions.

For example, the `STATUS_NO_FIX` constant of the [`NavSatStatus` message](https://docs.ros.org/en/jade/api/sensor_msgs/html/msg/NavSatStatus.html) can be accessed as follows:

```c++
assert((sensor_msgs::const_NavSatStatus_STATUS_NO_FIX() == -1));
```

(Note: Exposing them as C++ constants is not possible because it's [not supported by `cxx` yet](https://github.com/dtolnay/cxx/issues/1051).)

### Service Clients

To create a service client, use one of the `create_client_<TYPE>` functions.
The `<TYPE>` describes the service type, which specifies the request and response types.
The Dora ROS2 bridge automatically creates `create_client_<TYPE>` functions for all service types found in the sourced ROS2 environment.

```c++
auto add_two_ints = node->create_client_example_interfaces_AddTwoInts(
  "/",
  "add_two_ints",
  qos,
  merged_events
);
```

- The first argument is the namespace of the service and the second argument is its name.
- The third argument is the QoS (quality of service) setting for the service.
  It can be set to `qos_default()` or adjusted as desired, for example:
  ```c++
  auto qos = qos_default();
  qos.reliable = true;
  qos.max_blocking_time = 0.1;
  qos.keep_last = 1;
  ```
- The last argument is the [combined event stream](#combined-event-streams) that the received service responses should be merged into.

#### Waiting for the Service

In order to achieve reliable service communication, it is recommended to wait until the service is available before sending requests.
Use the `wait_for_service()` method for that, e.g.:

```c++
add_two_ints->wait_for_service(node)
```

The given `node` must be the node on which the service was created.

#### Sending Requests

To send a request, use the `send_request` method:

```c++
add_two_ints->send_request(request);
```

The method sends the request asynchronously without waiting for a response.
When the response is received, it is automatically sent to the [combined event stream](#combined-event-streams) that was given on client creation.

#### Receiving Responses

See the [_"Receiving Messages from Combined Event Stream"_](#receiving-messages-from-combined-event-stream) section for how to receive events from the combined event stream.
To check if a received event is a service response, use the `matches` method.
If it returns `true`, you can use the `downcast` method to convert the event to the correct service response type.

Example:

```c++
if (add_two_ints->matches(event))
{
    auto response = add_two_ints->downcast(std::move(event));
    std::cout << "Received sum response with value " << response.sum << std::endl;
    ...
}
```
