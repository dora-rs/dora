[English](README.md) | [简体中文](README.zh-CN.md)

# Adora

**智能体数据流导向的机器人架构（Agentic Dataflow-Oriented Robotic Architecture）** -- 一个 100% Rust 编写的实时机器人与 AI 应用框架。

> 通过**智能体工程（agentic engineering）**方式构建和维护 -- 代码生成、审查、重构、测试和提交均由自主 AI 智能体驱动。

---

## 目录

- [特性](#特性)
- [安装](#安装)
- [快速开始](#快速开始)
- [CLI 命令](#cli-命令)
- [数据流配置](#数据流配置)
- [架构](#架构)
- [语言支持](#语言支持)
- [示例](#示例)
- [开发](#开发)
- [贡献](#贡献)
- [许可证](#许可证)

## 特性

### 性能

- **比 ROS2 Python 快 10-17 倍** -- 100% Rust 内核，对 >4KB 的消息使用零拷贝共享内存 IPC，从 4KB 到 4MB 有效载荷均保持平坦延迟
- **Apache Arrow 原生支持** -- 端到端列式内存格式，零序列化开销；所有语言绑定共享同一数据格式

### 开发者体验

- **单一 CLI，全生命周期管理** -- `adora run` 用于本地开发，`adora up/start` 用于分布式生产环境，加上构建、日志、监控、录制/回放，全部集成在一个工具中
- **声明式 YAML 数据流** -- 将流水线定义为有向图，通过类型化的输入/输出连接节点，支持环境变量覆盖
- **多语言节点** -- 使用 Rust、Python、C 或 C++ 编写节点，提供原生 API（非封装层）；可在同一数据流中自由混合语言
- **热重载** -- 无需重启数据流即可实时重载 Python 算子
- **编程式构建器** -- 作为 YAML 的替代方案，支持用 Python 代码构建数据流

### 生产就绪

- **容错** -- 逐节点重启策略（never/on-failure/always）、指数退避、健康监控、可配置输入超时的断路器
- **默认分布式** -- 同机节点间使用本地共享内存，跨机通信自动使用 [Zenoh](https://zenoh.io/) 发布-订阅，基于 SSH 的[集群管理](docs/distributed-deployment.md)支持标签调度、滚动升级和自动恢复
- **协调器持久化** -- 可选的 redb 后端状态存储，在协调器崩溃和重启后仍能恢复
- **OpenTelemetry** -- 内置结构化日志（支持轮转/路由）、指标、分布式追踪，以及通过 CLI 零配置查看追踪

### 调试与可观测性

- **录制/回放** -- 将数据流消息捕获到 `.adorec` 文件，支持以任意速度离线回放并替换节点，用于回归测试
- **主题检查** -- `topic echo` 打印实时数据，`topic hz` TUI 进行频率分析，`topic info` 查看 schema 和带宽
- **资源监控** -- `adora top` TUI 显示所有机器上每个节点的 CPU、内存、队列深度、网络 I/O、重启次数和健康状态；`--once` 标志可输出可脚本化的 JSON 快照
- **追踪检查** -- `trace list` 和 `trace view` 无需外部基础设施即可查看协调器 span
- **数据流可视化** -- 从 YAML 描述符生成交互式 HTML 或 Mermaid 图表

### 生态系统

- **通信模式** -- 内置[服务（请求/应答）](docs/patterns.md#2-service-requestreply)和[动作（目标/反馈/结果）](docs/patterns.md#3-action-goalfeedbackresult)模式，通过约定的元数据键实现；无需修改守护进程或 YAML
- **ROS2 桥接** -- 与 ROS2 主题、服务和动作的双向互操作；QoS 映射；Arrow 原生类型转换
- **预打包节点** -- [节点中心](https://github.com/dora-rs/dora-hub/)提供现成的摄像头、YOLO、LLM、TTS 等节点
- **进程内算子** -- 在共享运行时内运行的轻量级函数，避免简单变换的逐节点进程开销

## 安装

### 从 PyPI 安装（推荐）

```bash
pip install adora-rs-cli          # CLI（adora 命令）
pip install adora-rs              # Python 节点/算子 API
```

### 从 crates.io 安装

```bash
cargo install adora-cli
```

### 从源码构建

```bash
git clone https://github.com/adora-rs/adora.git
cd adora
cargo build --release -p adora-cli
PATH=$PATH:$(pwd)/target/release

# Python API（需要 maturin：pip install maturin）
maturin develop -m apis/python/node/Cargo.toml
maturin develop -m apis/python/operator/Cargo.toml
```

### 平台安装程序

**macOS / Linux：**

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.sh | sh
```

**Windows：**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.ps1 | iex"
```

### 构建特性

| 特性 | 描述 | 默认开启 |
|------|------|----------|
| `tracing` | OpenTelemetry 追踪支持 | 是 |
| `metrics` | OpenTelemetry 指标收集 | 否 |
| `python` | Python 算子支持（PyO3） | 否 |
| `redb-backend` | 持久化协调器状态（redb） | 否 |
| `prometheus` | 协调器上的 Prometheus `/metrics` 端点 | 否 |

```bash
cargo install adora-cli --features redb-backend
```

## 快速开始

### 1. 运行 Python 数据流

```bash
pip install adora-rs-cli adora-rs
git clone https://github.com/adora-rs/adora.git && cd adora
adora run examples/python-dataflow/dataflow.yml
```

这将运行一个 sender -> transformer -> receiver 流水线。以下是 Python 节点代码示例：

```python
# sender.py -- 发送 100 条消息
from adora import Node
import pyarrow as pa

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

```python
# receiver.py -- 接收并打印消息
from adora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT":
        print(f"Got {event['id']}: {event['value'].to_pylist()}")
    elif event["type"] == "STOP":
        break
```

参见 [Python 入门指南](docs/python-guide.md)获取完整教程，或查看 [Python API 参考](docs/api-python.md)获取完整 API 文档。

### 2. 运行 Rust 数据流

```bash
cd examples/rust-dataflow
adora run dataflow.yml
```

### 3. 分布式模式（临时部署）

```bash
# 终端 1：启动协调器 + 守护进程
adora up

# 终端 2：启动数据流（--debug 启用主题检查）
adora start dataflow.yml --attach --debug

# 终端 3：监控
adora list
adora logs <dataflow-id>
adora top

# 停止或重启
adora stop <dataflow-id>
adora restart --name <name>
adora down
```

### 4. 托管集群

```bash
# 从配置文件启动多机集群
adora cluster up cluster.yml

# 在集群上启动数据流
adora start dataflow.yml --name my-app --attach

# 检查集群健康状态
adora cluster status

# 关闭集群
adora cluster down
```

参见[分布式部署指南](docs/distributed-deployment.md)了解 cluster.yml 配置、标签调度、systemd 服务、滚动升级和运维手册。

## CLI 命令

### 生命周期

| 命令 | 描述 |
|------|------|
| `adora run <PATH>` | 本地运行数据流（无需协调器/守护进程） |
| `adora up` | 以本地模式启动协调器和守护进程 |
| `adora down` | 关闭协调器和守护进程 |
| `adora build <PATH>` | 从数据流描述符执行构建命令 |
| `adora start <PATH>` | 在运行中的协调器上启动数据流 |
| `adora stop <ID>` | 停止运行中的数据流 |
| `adora restart <ID>` | 重启运行中的数据流（停止 + 重新启动） |

### 监控

| 命令 | 描述 |
|------|------|
| `adora list` | 列出运行中的数据流（别名：`ps`） |
| `adora logs <ID>` | 显示数据流或节点的日志 |
| `adora top` | 实时资源监控（TUI）；也可使用 `adora inspect top` |
| `adora topic list` | 列出数据流中的主题 |
| `adora topic hz <TOPIC>` | 测量主题发布频率（TUI） |
| `adora topic echo <TOPIC>` | 将主题消息打印到标准输出 |
| `adora topic info <TOPIC>` | 显示主题类型和元数据 |
| `adora node list` | 列出数据流中的节点 |
| `adora trace list` | 列出协调器捕获的最近追踪 |
| `adora trace view <ID>` | 查看特定追踪的 span（支持前缀匹配） |
| `adora record <PATH>` | 将数据流消息录制到 `.adorec` 文件 |
| `adora replay <FILE>` | 从 `.adorec` 文件回放录制的消息 |

### 集群管理

| 命令 | 描述 |
|------|------|
| `adora cluster up <PATH>` | 从 cluster.yml 文件启动集群 |
| `adora cluster status` | 显示已连接的守护进程和活跃的数据流 |
| `adora cluster down` | 关闭集群 |
| `adora cluster install <PATH>` | 将守护进程安装为 systemd 服务 |
| `adora cluster uninstall <PATH>` | 移除 systemd 服务 |
| `adora cluster upgrade <PATH>` | 滚动升级：SCP 二进制文件 + 逐机重启 |
| `adora cluster restart <NAME>` | 按名称或 UUID 重启数据流 |

### 设置与工具

| 命令 | 描述 |
|------|------|
| `adora status` | 检查系统健康状态（别名：`check`） |
| `adora new` | 生成新项目或节点 |
| `adora graph <PATH>` | 可视化数据流（Mermaid 或 HTML） |
| `adora system` | 系统管理（守护进程/协调器控制） |
| `adora completion <SHELL>` | 生成 shell 补全脚本 |
| `adora self update` | 更新 adora CLI |

完整 CLI 文档请参见 [docs/cli.md](docs/cli.md)。分布式部署请参见 [docs/distributed-deployment.md](docs/distributed-deployment.md)。

## 数据流配置

数据流使用 YAML 定义。每个节点声明其二进制文件/脚本、输入和输出：

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: adora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install adora-yolo
    path: adora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install adora-rerun
    path: adora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

**内置定时器节点：** `adora/timer/millis/<N>` 和 `adora/timer/hz/<N>`。

**输入格式：** `<node-id>/<output-name>` 用于订阅另一个节点的输出。

## 架构

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
             (编排)          (每台机器)       (用户代码)
```

| 层级 | 协议 | 用途 |
|------|------|------|
| CLI <-> Coordinator | WebSocket（端口 6013） | 构建、运行、停止命令 |
| Coordinator <-> Daemon | TCP | 节点生成、数据流生命周期 |
| Daemon <-> Daemon | Zenoh | 分布式跨机通信 |
| Daemon <-> Node | 共享内存 / TCP | >4KB 数据零拷贝 IPC，小消息走 TCP |

### 核心组件

- **协调器（Coordinator）** -- 跨守护进程编排数据流生命周期。支持内存或持久化（redb）状态存储。
- **守护进程（Daemon）** -- 在单台机器上生成和管理节点。处理共享内存分配和消息路由。
- **运行时（Runtime）** -- 进程内算子执行引擎。算子在运行时进程内运行，避免逐算子的进程开销。
- **节点（Nodes）** -- 通过输入/输出通信的独立进程。可使用 Rust、Python、C 或 C++ 编写。
- **算子（Operators）** -- 在运行时内运行的轻量级函数。对于简单变换比节点更高效。

### 工作区布局

```
binaries/
  cli/                  # adora CLI 二进制文件
  coordinator/          # 编排服务
  daemon/               # 节点管理器 + IPC
  runtime/              # 进程内算子运行时
  ros2-bridge-node/     # ROS2 桥接二进制文件
  record-node/          # 数据流消息录制器
  replay-node/          # 录制消息回放器
libraries/
  core/                 # 描述符解析、构建工具
  message/              # 组件间消息类型（v0.7.0）
  shared-memory-server/ # 零拷贝 IPC
  arrow-convert/        # Arrow 数据转换
  recording/            # .adorec 录制格式
  log-utils/            # 日志解析、合并、格式化
  coordinator-store/    # 持久化协调器状态（redb）
  extensions/
    telemetry/          # OpenTelemetry 追踪 + 指标
    ros2-bridge/        # ROS2 互操作（桥接、消息生成、Arrow、Python）
    download/           # 下载工具
apis/
  rust/node/            # Rust 节点 API（adora-node-api）
  rust/operator/        # Rust 算子 API（adora-operator-api）
  python/node/          # Python 节点 API（PyO3）
  python/operator/      # Python 算子 API（PyO3）
  python/cli/           # Python CLI 接口
  c/node/               # C 节点 API
  c/operator/           # C 算子 API
  c++/node/             # C++ 节点 API（CXX bridge）
  c++/operator/         # C++ 算子 API（CXX bridge）
examples/               # 示例数据流
```

## 语言支持

| 语言 | 节点 API | 算子 API | 文档 | 状态 |
|------|----------|----------|------|------|
| Rust | `adora-node-api` | `adora-operator-api` | [API 参考](docs/api-rust.md) | 一等支持 |
| Python >= 3.8 | `pip install adora-rs` | 已包含 | [入门指南](docs/python-guide.md)、[API 参考](docs/api-python.md) | 一等支持 |
| C | `adora-node-api-c` | `adora-operator-api-c` | [API 参考](docs/api-c.md) | 支持 |
| C++ | `adora-node-api-cxx` | `adora-operator-api-cxx` | [API 参考](docs/api-cxx.md) | 支持 |
| ROS2 >= Foxy | `adora-ros2-bridge` | -- | [桥接指南](docs/ros2-bridge.md) | 实验性 |

### 平台支持

| 平台 | 状态 |
|------|------|
| Linux（x86_64、ARM64、ARM32） | 一等支持 |
| macOS（ARM64） | 一等支持 |
| Windows（x86_64） | 尽力支持 |
| WSL（x86_64） | 尽力支持 |

## 示例

### 核心语言示例

| 示例 | 语言 | 描述 |
|------|------|------|
| [rust-dataflow](examples/rust-dataflow) | Rust | 基础 Rust 节点流水线 |
| [python-dataflow](examples/python-dataflow) | Python | Python 发送/变换/接收 |
| [python-operator-dataflow](examples/python-operator-dataflow) | Python | Python 算子（进程内） |
| [python-dataflow-builder](examples/python-dataflow-builder) | Python | Python 编程式 API |
| [c-dataflow](examples/c-dataflow) | C | C 节点示例 |
| [c++-dataflow](examples/c++-dataflow) | C++ | C++ 节点示例 |
| [c++-arrow-dataflow](examples/c++-arrow-dataflow) | C++ | C++ Arrow 数据示例 |
| [cmake-dataflow](examples/cmake-dataflow) | C/C++ | 基于 CMake 的构建 |

### 通信模式

| 示例 | 语言 | 描述 |
|------|------|------|
| [service-example](examples/service-example) | Rust | 使用 `request_id` 关联的请求/应答 |
| [action-example](examples/action-example) | Rust | 带取消功能的目标/反馈/结果 |

完整指南请参见 [docs/patterns.md](docs/patterns.md)。

### 高级模式

| 示例 | 语言 | 描述 |
|------|------|------|
| [python-async](examples/python-async) | Python | 异步 Python 节点 |
| [python-concurrent-rw](examples/python-concurrent-rw) | Python | 并发读写模式 |
| [python-multiple-arrays](examples/python-multiple-arrays) | Python | 多数组处理 |
| [python-drain](examples/python-drain) | Python | 事件排空模式 |
| [multiple-daemons](examples/multiple-daemons) | Rust | 分布式多守护进程部署 |
| [rust-dataflow-git](examples/rust-dataflow-git) | Rust | 基于 Git 的数据流加载 |
| [rust-dataflow-url](examples/rust-dataflow-url) | Rust | 基于 URL 的数据流加载 |

### 日志

| 示例 | 语言 | 描述 |
|------|------|------|
| [python-logging](examples/python-logging) | Python | Python 日志集成 |
| [python-log](examples/python-log) | Python | 基础 Python 日志输出 |
| [log-sink-tcp](examples/log-sink-tcp) | YAML | 基于 TCP 的日志接收器 |
| [log-sink-file](examples/log-sink-file) | YAML | 基于文件的日志接收器 |
| [log-sink-alert](examples/log-sink-alert) | YAML | 基于告警的日志接收器 |

### 性能

| 示例 | 语言 | 描述 |
|------|------|------|
| [benchmark](examples/benchmark) | Rust | CPU 延迟基准测试 |
| [cuda-benchmark](examples/cuda-benchmark) | Rust/CUDA | GPU 零拷贝基准测试 |

### ROS2 集成

| 示例 | 描述 |
|------|------|
| [ros2-bridge/rust](examples/ros2-bridge/rust) | Rust ROS2 主题、服务、动作 |
| [ros2-bridge/python](examples/ros2-bridge/python) | Python ROS2 集成 |
| [ros2-bridge/c++](examples/ros2-bridge/c++) | C++ ROS2 集成 |
| [ros2-bridge/yaml-bridge](examples/ros2-bridge/yaml-bridge) | 基于 YAML 的 ROS2 主题桥接 |
| [ros2-bridge/yaml-bridge-service](examples/ros2-bridge/yaml-bridge-service) | YAML ROS2 服务桥接 |
| [ros2-bridge/yaml-bridge-action](examples/ros2-bridge/yaml-bridge-action) | YAML ROS2 动作客户端 |
| [ros2-bridge/yaml-bridge-action-server](examples/ros2-bridge/yaml-bridge-action-server) | YAML ROS2 动作服务器 |

## 开发

**Rust 版本 2024，最低支持 Rust 版本 1.85.0，工作区版本 0.4.1。**

### 构建

```bash
# 构建全部（排除需要 maturin 的 Python 包）
cargo build --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python

# 构建特定包
cargo build -p adora-cli
```

### 测试

```bash
# 运行所有测试
cargo test --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python

# 测试单个包
cargo test -p adora-core

# 冒烟测试（需要协调器/守护进程）
cargo test --test example-smoke -- --test-threads=1
```

### 代码检查与格式化

```bash
cargo clippy --all
cargo fmt --all -- --check
```

### 运行示例

```bash
cargo run --example rust-dataflow
cargo run --example python-dataflow
cargo run --example benchmark --release
```

## 贡献

我们欢迎各经验水平的贡献者。请参见[贡献指南](CONTRIBUTING.md)以开始。

### 交流

- [Discord](https://discord.gg/6eMGGutkfE)
- [GitHub Discussions](https://github.com/orgs/adora-rs/discussions)

## AI 辅助开发

本仓库通过 AI 辅助的智能体工程方式维护。代码生成、审查、重构、测试和提交均由自主 AI 智能体驱动 -- 实现更快的迭代速度和更高的大规模代码质量。

## 许可证

Apache-2.0。详情请参见 [NOTICE.md](NOTICE.md)。
