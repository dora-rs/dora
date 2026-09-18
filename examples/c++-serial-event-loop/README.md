# Serial Event Loop

A **task-responsive event loop** built on the Dora framework. It provides dedicated worker threads (`SerialWorker`), timer management, and thread-safe output delivery. The main event loop runs on a single thread and blocks on `dora_next_event` while waiting for input events. When an event arrives, the loop dispatches it to the dedicated `SerialWorker` for the corresponding `input_id`, then immediately resumes waiting for the next event. This design enables high-throughput, low-latency event dispatch.

### Requirements

- C++17
- pthread
- The Dora framework (providing `node_api.h` and the runtime library)

The test target uses the C API header from the current Dora checkout and provides a local fake runtime, so it does not link the Dora runtime library.

### Build and Test

```bash
mkdir build && cd build
cmake ..
make
./serial_event_loop_test
```

CMake derives `DORA_ROOT` from this example's location. For an out-of-tree checkout or a custom runtime build, override `DORA_ROOT` and/or `DORA_LIB_PATH` with `-D` options.

### Example

```cpp
#include "serial_event_loop.hpp"

using namespace dora_extensions;

int main() {
    SerialEventLoop loop("my_node");

    // 1. Register an input handler (one dedicated worker per input_id)
    loop.register_handler("camera", [](const InputEvent& e) {
        // Runs on its dedicated thread without blocking the main loop
        process_camera_frame(e.data);
    });

    // 2. Register a timer
    loop.register_timer("heartbeat", std::chrono::milliseconds(1000),
        []() { std::cout << "tick" << std::endl; });

    // 3. Start the event loop (blocking)
    loop.run();
}
```

## API Reference

### SerialWorker

Each `SerialWorker` owns an independent thread and FIFO queue, ensuring that events are processed **strictly in enqueue order**.

```cpp
SerialWorker worker("name", [](const InputEvent& e) {
    // Processing logic
});

InputEvent event;
event.id = "sensor_1";
event.data = {0x01, 0x02};
worker.enqueue(event);  // Non-blocking; returns immediately

worker.stop();  // Stops the thread after processing all queued events
```

### SerialEventLoop

#### 1. `register_handler(id, handler)`

Registers a dedicated handler for an input topic. Each `id` can have only one handler; registering another handler for the same `id` replaces the previous one.

| Parameter | Type | Description |
|-----------|------|-------------|
| `id` | `const std::string&` | Input topic ID corresponding to an input ID in the Dora dataflow |
| `handler` | `std::function<void(const InputEvent&)>` | Application logic executed on the dedicated thread |

```cpp
loop.register_handler("lidar", [](const InputEvent& e) {
    // e.id   -> "lidar"
    // e.data -> raw byte data
});
```

#### 2. `register_timer(id, interval, handler, repeat)`

Registers a timer. Timers run on a separate thread, so their callbacks do not block the main event loop.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `id` | `const std::string&` | - | Unique timer identifier |
| `interval` | `std::chrono::milliseconds` | - | Trigger interval |
| `handler` | `std::function<void()>` | - | Callback function |
| `repeat` | `bool` | `true` | `true` to trigger repeatedly; `false` to trigger only once |

```cpp
// Trigger every 100 ms
loop.register_timer("fast_tick", std::chrono::milliseconds(100), []() {
    publish_status();
});

// Trigger once after 5 seconds
loop.register_timer("delayed_init", std::chrono::seconds(5), []() {
    late_init();
}, false);
```

#### 3. `cancel_timer(id)`

Cancels a registered timer. Returns `true` if the timer was successfully cancelled, or `false` if the timer does not exist.

```cpp
if (loop.cancel_timer("fast_tick")) {
    std::cout << "timer cancelled" << std::endl;
}
```

#### 4. `send_output(output_id, data)`

Sends an output message. This method is **thread-safe** and can be called from any thread, including the main event loop, a worker, or a timer callback.

- When called from the main event-loop thread, the message is sent directly and synchronously.
- When called from another thread, the message is queued and sent by the main loop during its next iteration.

```cpp
std::vector<uint8_t> result = {0x00, 0x01, 0x02};
loop.send_output("processed_result", result);
```

#### 5. `run()`

Starts the event loop. This is a **blocking call** that performs the following steps internally:

1. Initializes the Dora context.
2. Starts the timer thread.
3. Waits for and dispatches Dora events in a loop.
4. Cleans up and exits after receiving a `Stop` event.

## Example Dataflow

The included dataflow demonstrates two C++ Dora nodes:

```text
dora/timer/millis/100 -> sender -> counter -> receiver
```

The sender publishes an incrementing `uint64` counter at 10 Hz. The receiver
uses `SerialEventLoop` to process each counter on its dedicated worker and
prints output such as:

```text
[cpp-receiver] received counter=0
[cpp-receiver] received counter=1
```

Build and run the example. Press `Ctrl+C` to stop it:

```bash
cmake -S . -B build
cmake --build build --target cpp_sender cpp_receiver
dora run dataflow.yml
```
