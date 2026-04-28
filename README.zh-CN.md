[English](README.md) | [简体中文](README.zh-CN.md)

<p align="center">
    <img src="https://raw.githubusercontent.com/dora-rs/dora/main/docs/src/logo.svg" width="400"/>
</p>

<h2 align="center">
  <a href="https://www.dora-rs.ai">官网</a>
  |
  <a href="https://dora-rs.ai/docs/guides/getting-started/conversation_py/">Python API</a>
  |
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/">Rust API</a>
  |
  <a href="https://www.dora-rs.ai/docs/guides/">指南</a>
  |
  <a href="https://discord.gg/6eMGGutkfE">Discord</a>
</h2>

<div align="center">
  <a href="https://github.com/dora-rs/dora/actions"><img src="https://github.com/dora-rs/dora/workflows/CI/badge.svg" alt="Build and test"/></a>
  <a href="https://crates.io/crates/dora-cli"><img src="https://img.shields.io/crates/v/dora-cli.svg" alt="crates.io"/></a>
  <a href="https://docs.rs/dora-node-api/latest/dora_node_api/"><img src="https://docs.rs/dora-node-api/badge.svg" alt="docs.rs"/></a>
  <a href="https://pypi.org/project/dora-rs/"><img src="https://img.shields.io/pypi/v/dora-rs.svg" alt="PyPI"/></a>
  <a href="https://github.com/dora-rs/dora/blob/main/LICENSE"><img src="https://img.shields.io/github/license/dora-rs/dora" alt="License"/></a>
</div>

# Dora

**智能体数据流导向的机器人架构（Agentic Dataflow-Oriented Robotic Architecture）** -- 一个 100% Rust 编写的实时机器人与 AI 应用框架。

[**User Guide**](https://dora-rs.ai/dora/) | [**用户指南 (中文)**](https://dora-rs.ai/dora/zh-CN/)

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

- **单一 CLI，全生命周期管理** -- `dora run` 用于本地开发，`dora up/start` 用于分布式生产环境，加上构建、日志、监控、录制/回放，全部集成在一个工具中
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

- **录制/回放** -- 将数据流消息捕获到 `.drec` 文件，支持以任意速度离线回放并替换节点，用于回归测试
- **主题检查** -- `topic echo` 打印实时数据，`topic hz` TUI 进行频率分析，`topic info` 查看 schema 和带宽
- **资源监控** -- `dora top` TUI 显示所有机器上每个节点的 CPU、内存、队列深度、网络 I/O、重启次数和健康状态；`--once` 标志可输出可脚本化的 JSON 快照
- **追踪检查** -- `trace list` 和 `trace view` 无需外部基础设施即可查看协调器 span
- **数据流可视化** -- 从 YAML 描述符生成交互式 HTML 或 Mermaid 图表

### 生态系统

- **通信模式** -- 内置[服务（请求/应答）](docs/patterns.md#2-service-requestreply)和[动作（目标/反馈/结果）](docs/patterns.md#3-action-goalfeedbackresult)模式，通过约定的元数据键实现；无需修改守护进程或 YAML
- **ROS2 桥接** -- 与 ROS2 主题、服务和动作的双向互操作；QoS 映射；Arrow 原生类型转换
- **预打包节点** -- [节点中心](https://github.com/dora-rs/dora-hub/)提供现成的摄像头、YOLO、LLM、TTS 等节点
- **进程内算子** -- 在共享运行时内运行的轻量级函数，避免简单变换的逐节点进程开销

## 安装

### 从 crates.io 安装（推荐）

```bash
cargo install dora-cli           # CLI（dora 命令）
pip install dora-rs              # Python 节点/算子 API
```

### 从源码构建

```bash
git clone https://github.com/dora-rs/dora.git
cd dora
cargo build --release -p dora-cli
PATH=$PATH:$(pwd)/target/release

# Python API（需要 maturin：pip install maturin）
maturin develop -m apis/python/node/Cargo.toml
maturin develop -m apis/python/operator/Cargo.toml
```

### 平台安装程序

**macOS / Linux：**

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.sh | sh
```

**Windows：**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/dora/releases/latest/download/dora-cli-installer.ps1 | iex"
```

### 构建特性

| 特性 | 描述 | 默认开启 |
|------|------|----------|
| `tracing` | OpenTelemetry 追踪支持 | 是 |
| `metrics` | OpenTelemetry 指标收集 | 是 |
| `python` | Python 算子支持（PyO3） | 否 |
| `redb-backend` | 持久化协调器状态（redb） | 是 |

```bash
cargo install dora-cli --features redb-backend
```

## 快速开始

### 1. 运行 Python 数据流

```bash
cargo install dora-cli            # 或使用下方安装脚本
pip install dora-rs
git clone https://github.com/dora-rs/dora.git && cd dora
dora run examples/python-dataflow/dataflow.yml
```

这将运行一个 sender -> transformer -> receiver 流水线。以下是 Python 节点代码示例：

```python
# sender.py -- 发送 100 条消息
from dora import Node
import pyarrow as pa

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

```python
# receiver.py -- 接收并打印消息
from dora import Node

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
dora run dataflow.yml
```

### 3. 分布式模式（临时部署）

```bash
# 终端 1：启动协调器 + 守护进程
dora up

# 终端 2：启动数据流（--debug 启用主题检查）
dora start dataflow.yml --attach --debug

# 终端 3：监控
dora list
dora logs <dataflow-id>
dora top

# 停止或重启
dora stop <dataflow-id>
dora restart --name <name>
dora down
```

### 4. 托管集群

```bash
# 从配置文件启动多机集群
dora cluster up cluster.yml

# 在集群上启动数据流
dora start dataflow.yml --name my-app --attach

# 检查集群健康状态
dora cluster status

# 关闭集群
dora cluster down
```

参见[分布式部署指南](docs/distributed-deployment.md)了解 cluster.yml 配置、标签调度、systemd 服务、滚动升级和运维手册。

## CLI 命令

### 生命周期

| 命令 | 描述 |
|------|------|
| `dora run <PATH>` | 本地运行数据流（无需协调器/守护进程） |
| `dora up` | 以本地模式启动协调器和守护进程 |
| `dora down` | 关闭协调器和守护进程 |
| `dora build <PATH>` | 从数据流描述符执行构建命令 |
| `dora start <PATH>` | 在运行中的协调器上启动数据流 |
| `dora stop <ID>` | 停止运行中的数据流 |
| `dora restart <ID>` | 重启运行中的数据流（停止 + 重新启动） |

### 监控

| 命令 | 描述 |
|------|------|
| `dora list` | 列出运行中的数据流（别名：`ps`） |
| `dora logs <ID>` | 显示数据流或节点的日志 |
| `dora top` | 实时资源监控（TUI）；也可使用 `dora inspect top` |
| `dora topic list` | 列出数据流中的主题 |
| `dora topic hz <TOPIC>` | 测量主题发布频率（TUI） |
| `dora topic echo <TOPIC>` | 将主题消息打印到标准输出 |
| `dora topic info <TOPIC>` | 显示主题类型和元数据 |
| `dora node list` | 列出数据流中的节点 |
| `dora trace list` | 列出协调器捕获的最近追踪 |
| `dora trace view <ID>` | 查看特定追踪的 span（支持前缀匹配） |
| `dora record <PATH>` | 将数据流消息录制到 `.drec` 文件 |
| `dora replay <FILE>` | 从 `.drec` 文件回放录制的消息 |

### 集群管理

| 命令 | 描述 |
|------|------|
| `dora cluster up <PATH>` | 从 cluster.yml 文件启动集群 |
| `dora cluster status` | 显示已连接的守护进程和活跃的数据流 |
| `dora cluster down` | 关闭集群 |
| `dora cluster install <PATH>` | 将守护进程安装为 systemd 服务 |
| `dora cluster uninstall <PATH>` | 移除 systemd 服务 |
| `dora cluster upgrade <PATH>` | 滚动升级：SCP 二进制文件 + 逐机重启 |
| `dora cluster restart <NAME>` | 按名称或 UUID 重启数据流 |

### 设置与工具

| 命令 | 描述 |
|------|------|
| `dora status` | 检查系统健康状态（别名：`check`） |
| `dora new` | 生成新项目或节点 |
| `dora graph <PATH>` | 可视化数据流（Mermaid 或 HTML） |
| `dora system` | 系统管理（守护进程/协调器控制） |
| `dora completion <SHELL>` | 生成 shell 补全脚本 |
| `dora self update` | 更新 dora CLI |

完整 CLI 文档请参见 [docs/cli.md](docs/cli.md)。分布式部署请参见 [docs/distributed-deployment.md](docs/distributed-deployment.md)。

## 数据流配置

数据流使用 YAML 定义。每个节点声明其二进制文件/脚本、输入和输出：

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install dora-yolo
    path: dora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install dora-rerun
    path: dora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

**内置定时器节点：** `dora/timer/millis/<N>` 和 `dora/timer/hz/<N>`。

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
  cli/                  # dora CLI 二进制文件
  coordinator/          # 编排服务
  daemon/               # 节点管理器 + IPC
  runtime/              # 进程内算子运行时
  ros2-bridge-node/     # ROS2 桥接二进制文件
  record-node/          # 数据流消息录制器
  replay-node/          # 录制消息回放器
libraries/
  core/                 # 描述符解析、构建工具
  message/              # 组件间消息类型
  shared-memory-server/ # 零拷贝 IPC
  arrow-convert/        # Arrow 数据转换
  recording/            # .drec 录制格式
  log-utils/            # 日志解析、合并、格式化
  coordinator-store/    # 持久化协调器状态（redb）
  extensions/
    telemetry/          # OpenTelemetry 追踪 + 指标
    ros2-bridge/        # ROS2 互操作（桥接、消息生成、Arrow、Python）
    download/           # 下载工具
apis/
  rust/node/            # Rust 节点 API（dora-node-api）
  rust/operator/        # Rust 算子 API（dora-operator-api）
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
| Rust | `dora-node-api` | `dora-operator-api` | [API 参考](docs/api-rust.md) | 一等支持 |
| Python >= 3.8 | `pip install dora-rs` | 已包含 | [入门指南](docs/python-guide.md)、[API 参考](docs/api-python.md) | 一等支持 |
| C | `dora-node-api-c` | `dora-operator-api-c` | [API 参考](docs/api-c.md) | 支持 |
| C++ | `dora-node-api-cxx` | `dora-operator-api-cxx` | [API 参考](docs/api-cxx.md) | 支持 |
| ROS2 >= Foxy | `dora-ros2-bridge` | -- | [桥接指南](docs/ros2-bridge.md) | 实验性 |

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
| [log-aggregator](examples/log-aggregator) | Python | 通过 `dora/logs` 集中日志聚合 |

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

**Rust 版本 2024；最低支持 Rust 版本（MSRV）及工作区默认包元数据以根
`Cargo.toml` 的 `[workspace.package]` 为准。** 多数 crate 通过
`version.workspace = true` 继承该版本；少数 crate（例如
`apis/rust/operator/types` 与 `examples/error-propagation/*` 样例）
自带独立版本号。

### 构建

```bash
# 构建全部（排除需要 maturin 的 Python 包）
cargo build --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python

# 构建特定包
cargo build -p dora-cli
```

### 测试

```bash
# 运行所有测试
cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python

# 测试单个包
cargo test -p dora-core

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
- [GitHub Discussions](https://github.com/orgs/dora-rs/discussions)

## AI 辅助开发

本仓库通过 AI 辅助的智能体工程方式维护。代码生成、审查、重构、测试和提交均由自主 AI 智能体驱动 -- 实现更快的迭代速度和更高的大规模代码质量。

## 许可证

Apache-2.0。详情请参见 [NOTICE.md](NOTICE.md)。
