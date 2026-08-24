# Serial-Event-Loop

基于 Dora 框架的**任务响应式事件循环**，提供专属工作者线程（SerialWorker）、定时器管理和线程安全的输出发送。主事件循环运行在单线程中，通过 `dora_next_event` 阻塞等待输入事件。收到事件后，根据 `input_id` 将其分发到对应的 `SerialWorker` 专属线程处理，主循环立即返回继续等待下一个事件——从而实现高吞吐、低延迟的事件分发。

### 依赖
- C++17
- pthread
- Dora 框架（提供 `node_api.h` 和运行时库）

如果只需编译测试（不链接 Dora 运行时），CMake 会自动使用内置的 mock 头文件。

### 构建与测试

```bash
mkdir build && cd build
cmake ..
make
./serial_event_loop_test
```

### 示例

```cpp
#include "serial_event_loop.hpp"

using namespace dora_extensions;

int main() {
    SerialEventLoop loop("my_node");

    // 1. 注册输入处理器（每个 input_id 一个专属 worker）
    loop.register_handler("camera", [](const InputEvent& e) {
        // 在专属线程中执行，不会阻塞主循环
        process_camera_frame(e.data);
    });

    // 2. 注册定时器
    loop.register_timer("heartbeat", std::chrono::milliseconds(1000),
        []() { std::cout << "tick" << std::endl; });

    // 3. 启动事件循环（阻塞）
    loop.run();
}
```




## API 函数说明

### SerialWorker

每个 `SerialWorker` 拥有独立的线程和 FIFO 队列，保证事件**严格按入队顺序**处理。

```cpp
SerialWorker worker("name", [](const InputEvent& e) {
    // 处理逻辑
});

InputEvent event;
event.id = "sensor_1";
event.data = {0x01, 0x02};
worker.enqueue(event);  // 非阻塞，瞬间返回

worker.stop();  // 停止线程，等待处理完队列中剩余事件
```

### SerialEventLoop

#### 1 `register_handler(id, handler)`

为一个输入话题注册专属处理器。每个 `id` 只能注册一个 handler，后注册的会覆盖前者。

| 参数 | 类型 | 说明 |
|------|------|------|
| `id` | `const std::string&` | 输入话题 ID，与 Dora dataflow 中的 input ID 对应 |
| `handler` | `std::function<void(const InputEvent&)>` | 在专属线程中执行的业务逻辑 |

```cpp
loop.register_handler("lidar", [](const InputEvent& e) {
    // e.id   -> "lidar"
    // e.data -> 原始字节数据
});
```

#### 2 `register_timer(id, interval, handler, repeat)`

注册定时器。定时器在独立线程中运行，回调不会阻塞主事件循环。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `id` | `const std::string&` | - | 定时器唯一标识 |
| `interval` | `std::chrono::milliseconds` | - | 触发间隔 |
| `handler` | `std::function<void()>` | - | 回调函数 |
| `repeat` | `bool` | `true` | `true` 重复触发，`false` 仅触发一次 |

```cpp
// 每 100ms 触发一次
loop.register_timer("fast_tick", std::chrono::milliseconds(100), []() {
    publish_status();
});

// 5 秒后触发一次
loop.register_timer("delayed_init", std::chrono::seconds(5), []() {
    late_init();
}, false);
```

#### 3 `cancel_timer(id)`

取消已注册的定时器。返回 `true` 表示成功取消，`false` 表示定时器不存在。

```cpp
if (loop.cancel_timer("fast_tick")) {
    std::cout << "timer cancelled" << std::endl;
}
```

#### 4 `send_output(output_id, data)`

发送输出消息，**线程安全**，可在任意线程（主循环、Worker、定时器回调）中调用。

- 如果在主事件循环线程中调用：直接发送（同步）
- 如果在其他线程中调用：入队，由主循环在下一轮取出并发送

```cpp
std::vector<uint8_t> result = {0x00, 0x01, 0x02};
loop.send_output("processed_result", result);
```

#### 5 `run()`

启动事件循环（**阻塞调用**）。内部会：
1. 初始化 Dora 上下文
2. 启动定时器线程
3. 循环等待 Dora 事件并分发
4. 收到 `Stop` 事件后清理退出

## 示例数据流

示例包含两个 C++ Dora 节点：

```text
dora/timer/millis/100 -> sender -> counter -> receiver
```

发送节点以 10 Hz 发布递增的 `uint64` 计数器。接收节点通过
`SerialEventLoop` 的专属工作者处理并打印每个计数器，例如：

```text
[cpp-receiver] received counter=0
[cpp-receiver] received counter=1
```

构建并运行示例，按 `Ctrl+C` 停止：

```bash
cmake -S . -B build
cmake --build build --target cpp_sender cpp_receiver
dora run dataflow.yml
```
