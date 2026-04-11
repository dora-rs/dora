#!/usr/bin/env python3
"""Translate zh-CN.po file for Dora User Guide.

Usage: python3 translate.py
Reads po/zh-CN.po, applies translations, writes po/zh-CN.po.
Entries without translations keep empty msgstr (falls back to English).
"""
import re
from pathlib import Path


def parse_po(filepath):
    """Parse .po file into list of entries."""
    with open(filepath, 'r') as f:
        content = f.read()

    entries = []
    # Split on blank lines that separate entries
    blocks = re.split(r'\n\n+', content)

    for block in blocks:
        if not block.strip():
            continue
        entry = {'raw': block, 'comments': [], 'msgid': '', 'msgstr': ''}
        lines = block.split('\n')
        mode = None
        msgid_parts = []
        msgstr_parts = []

        for line in lines:
            if line.startswith('#'):
                entry['comments'].append(line)
            elif line.startswith('msgid "'):
                mode = 'msgid'
                msgid_parts.append(line[7:-1])
            elif line.startswith('msgstr "'):
                mode = 'msgstr'
                msgstr_parts.append(line[8:-1])
            elif line.startswith('"') and mode == 'msgid':
                msgid_parts.append(line[1:-1])
            elif line.startswith('"') and mode == 'msgstr':
                msgstr_parts.append(line[1:-1])

        entry['msgid'] = ''.join(msgid_parts)
        entry['msgstr'] = ''.join(msgstr_parts)
        entries.append(entry)

    return entries


PO_HEADER_LINES = [
    '"Project-Id-Version: Dora User Guide\\n"',
    '"POT-Creation-Date: 2026-03-03T12:08:31-06:00\\n"',
    '"PO-Revision-Date: 2026-03-03 00:00+0000\\n"',
    '"Last-Translator: Dora Contributors\\n"',
    '"Language-Team: Chinese (Simplified)\\n"',
    '"MIME-Version: 1.0\\n"',
    '"Content-Type: text/plain; charset=UTF-8\\n"',
    '"Content-Transfer-Encoding: 8bit\\n"',
    '"Language: zh_CN\\n"',
    '"Plural-Forms: nplurals=1; plural=0;\\n"',
]


def write_po(filepath, entries, translations):
    """Write .po file, applying translations."""
    with open(filepath, 'w') as f:
        for i, entry in enumerate(entries):
            if i > 0:
                f.write('\n')

            # Write comments
            for c in entry['comments']:
                f.write(c + '\n')

            msgid = entry['msgid']

            # Header entry (empty msgid) gets metadata
            if msgid == '' and i == 0:
                f.write('msgid ""\n')
                f.write('msgstr ""\n')
                for hline in PO_HEADER_LINES:
                    f.write(hline + '\n')
                continue

            # Write msgid (preserve original formatting)
            raw_lines = entry['raw'].split('\n')
            in_msgid = False
            in_msgstr = False
            for line in raw_lines:
                if line.startswith('#'):
                    continue
                if line.startswith('msgid '):
                    f.write(line + '\n')
                    in_msgid = True
                    in_msgstr = False
                elif line.startswith('msgstr '):
                    in_msgid = False
                    in_msgstr = True
                    # Write translated msgstr
                    if msgid in translations:
                        tr = translations[msgid]
                        write_msgstr(f, tr)
                    else:
                        f.write('msgstr ""\n')
                    break  # skip original msgstr lines
                elif line.startswith('"') and in_msgid:
                    f.write(line + '\n')


def write_msgstr(f, text):
    """Write msgstr with proper .po line wrapping."""
    if not text:
        f.write('msgstr ""\n')
        return

    # Check if text contains newlines
    if '\\n' in text:
        f.write('msgstr ""\n')
        parts = text.split('\\n')
        for i, part in enumerate(parts):
            suffix = '\\n' if i < len(parts) - 1 else ''
            f.write(f'"{part}{suffix}"\n')
    elif len(text) > 76:
        f.write('msgstr ""\n')
        # Wrap at ~76 chars
        words = text.split(' ')
        line = ''
        for word in words:
            if line and len(line) + len(word) + 1 > 76:
                f.write(f'"{line} "\n')
                line = word
            elif line:
                line += ' ' + word
            else:
                line = word
        if line:
            f.write(f'"{line}"\n')
    else:
        f.write(f'msgstr "{text}"\n')


def build_translations():
    """Build comprehensive translation dictionary."""
    t = {}

    # =============================================
    # SUMMARY.md - Navigation
    # =============================================
    t["Summary"] = "目录"
    t["Introduction"] = "简介"
    t["Getting Started"] = "快速入门"
    t["Installation"] = "安装"
    t["Quick Start"] = "快速开始"
    t["Concepts"] = "核心概念"
    t["Architecture"] = "架构"
    t["Dataflow YAML"] = "数据流 YAML"
    t["Communication Patterns"] = "通信模式"
    t["Language APIs"] = "语言 API"
    t["Rust"] = "Rust"
    t["Python"] = "Python"
    t["C"] = "C"
    t["C++"] = "C++"
    t["Operations"] = "运维"
    t["CLI Reference"] = "CLI 参考"
    t["Logging"] = "日志"
    t["Debugging"] = "调试"
    t["Fault Tolerance"] = "容错"
    t["Distributed Deployment"] = "分布式部署"
    t["Advanced"] = "进阶"
    t["ROS2 Bridge"] = "ROS2 桥接"
    t["WebSocket Control Plane"] = "WebSocket 控制面"
    t["WebSocket Topic Data"] = "WebSocket 主题数据"
    t["Development"] = "开发"
    t["Testing Guide"] = "测试指南"

    # =============================================
    # introduction.md
    # =============================================
    t["Dora"] = "Dora"
    t["**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications."] = \
        "**智能数据流导向机器人架构** (Agentic Dataflow-Oriented Robotic Architecture) -- 一个 100% Rust 的实时机器人与 AI 应用框架。"
    t["Why Dora?"] = "为什么选择 Dora？"
    t["**10-17x faster than ROS2 Python** -- zero-copy shared memory IPC, flat latency from 4KB to 4MB payloads"] = \
        "**比 ROS2 Python 快 10-17 倍** -- 零拷贝共享内存 IPC，4KB 到 4MB 载荷延迟平坦"
    t["**Apache Arrow native** -- columnar memory format end-to-end with zero serialization overhead"] = \
        "**Apache Arrow 原生** -- 端到端列式内存格式，零序列化开销"
    t["**Single CLI, full lifecycle** -- `dora run` for local dev, `dora up/start` for distributed prod, plus build, logs, monitoring, record/replay"] = \
        "**单一 CLI，全生命周期** -- `dora run` 本地开发，`dora up/start` 分布式生产，加上构建、日志、监控、录制/回放"
    t["**Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs"] = \
        "**声明式 YAML 数据流** -- 以有向图定义流水线，通过类型化输入/输出连接节点"
    t["**Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs; mix languages freely"] = \
        "**多语言节点** -- 用 Rust、Python、C 或 C++ 的原生 API 编写节点；可自由混合语言"
    t["**Fault tolerance** -- per-node restart policies, exponential backoff, health monitoring, circuit breakers"] = \
        "**容错机制** -- 每节点重启策略、指数退避、健康监测、熔断器"
    t["**Distributed by default** -- local shared memory between co-located nodes, automatic Zenoh pub-sub across machines"] = \
        "**天然分布式** -- 同机节点间共享内存，跨机自动 Zenoh 发布/订阅"
    t["**Record/replay** -- capture dataflow messages to `.adorec` files, replay offline at any speed"] = \
        "**录制/回放** -- 将数据流消息捕获到 `.adorec` 文件，可以任意速度离线回放"
    t["**Built-in observability** -- `dora top` TUI, topic inspection, trace viewing, structured logging with rotation"] = \
        "**内置可观测性** -- `dora top` TUI 监控、主题检查、追踪查看、结构化日志轮转"
    # v0.2 additions to introduction
    t["**Zenoh SHM data plane** -- nodes publish directly via Zenoh shared memory for zero-copy data transfer; automatic network fallback for cross-machine"] = \
        "**Zenoh SHM 数据面** -- 节点通过 Zenoh 共享内存直接发布，零拷贝数据传输；跨机自动网络回退"
    t["**Non-blocking event loop** -- Zenoh publishes offloaded to drain task; metrics collection runs in background"] = \
        "**非阻塞事件循环** -- Zenoh 发布卸载到排水任务；指标收集在后台运行"
    t["**Coordinator HA** -- persistent redb state store, daemon auto-reconnect, dataflow state reconstruction on coordinator restart"] = \
        "**协调器高可用** -- 持久化 redb 状态存储，守护进程自动重连，协调器重启时数据流状态重建"
    t["**Dynamic topology** -- add and remove nodes from running dataflows via CLI without restarting"] = \
        "**动态拓扑** -- 通过 CLI 在运行中的数据流上添加和删除节点，无需重启"
    t["**Soft real-time** -- optional `--rt` flag for mlockall + SCHED_FIFO; per-node `cpu_affinity` pinning"] = \
        "**软实时** -- 可选 `--rt` 标志启用 mlockall + SCHED_FIFO；每节点 `cpu_affinity` 绑核"
    t["Performance"] = "性能"
    t["Developer Experience"] = "开发者体验"
    t["Production Readiness"] = "生产就绪"
    t["Debugging and Observability"] = "调试和可观测性"
    t["Ecosystem"] = "生态系统"
    t["Type Annotations"] = "类型注解"
    t["Modules"] = "模块"
    t["Dynamic Topology"] = "动态拓扑"
    t["Real-Time Tuning"] = "实时调优"

    # concepts/dataflow-yaml.md additions
    t["Arrow IPC Framing"] = "Arrow IPC 帧格式"
    t["CPU Affinity"] = "CPU 亲和性"
    t["Per-Node CPU Affinity"] = "每节点 CPU 亲和性"

    t["Next Steps"] = "下一步"
    t["[Install Dora](getting-started/installation.md)"] = \
        "[安装 Dora](getting-started/installation.md)"
    t["[Quick Start tutorial](getting-started/quickstart.md)"] = \
        "[快速入门教程](getting-started/quickstart.md)"
    t["[Architecture overview](concepts/architecture.md)"] = \
        "[架构概览](concepts/architecture.md)"

    # =============================================
    # getting-started/installation.md
    # =============================================
    t["From PyPI (recommended)"] = "从 PyPI 安装（推荐）"
    t["From crates.io"] = "从 crates.io 安装"
    t["From source"] = "从源码安装"
    t["Platform installers"] = "平台安装器"
    t["Build features"] = "构建特性"
    t["Verify"] = "验证"
    t["Feature"] = "特性"
    t["Description"] = "描述"
    t["Default"] = "默认"
    t["OpenTelemetry tracing support"] = "OpenTelemetry 追踪支持"
    t["OpenTelemetry metrics collection"] = "OpenTelemetry 指标收集"
    t["Python operator support (PyO3)"] = "Python 算子支持（PyO3）"
    t["Persistent coordinator state (redb)"] = "持久化协调器状态（redb）"
    t["Prometheus `/metrics` endpoint on coordinator"] = "协调器上的 Prometheus `/metrics` 端点"
    t["Yes"] = "是"
    t["No"] = "否"
    t["**macOS / Linux:**"] = "**macOS / Linux：**"
    t["**Windows:**"] = "**Windows：**"
    t["# CLI (dora command)\\n"] = "# CLI（dora 命令）\\n"
    t["# Python node/operator API\\n"] = "# Python 节点/算子 API\\n"

    # =============================================
    # getting-started/quickstart.md
    # =============================================
    t["Getting Started with Python"] = "Python 快速入门"
    t["This guide walks you through writing Python nodes and operators for dora dataflows."] = \
        "本指南将引导你使用 Python 编写 dora 数据流的节点和算子。"
    t["Prerequisites"] = "前提条件"
    t["The `dora-rs` package includes `pyarrow` as a dependency."] = \
        "`dora-rs` 包已包含 `pyarrow` 作为依赖。"
    t["**Building from source** (instead of `pip install dora-rs`):"] = \
        "**从源码构建**（替代 `pip install dora-rs`）："
    t["Your First Dataflow"] = "你的第一个数据流"
    t["Step 1: Define the dataflow"] = "第 1 步：定义数据流"
    t["Create `dataflow.yml`:"] = "创建 `dataflow.yml`："
    t["This declares three nodes: a sender that fires every 100ms, an operator that transforms data, and a receiver that prints results."] = \
        "这声明了三个节点：一个每 100ms 触发的发送者，一个转换数据的算子，以及一个打印结果的接收者。"
    t["Step 2: Write a sender node"] = "第 2 步：编写发送节点"
    t["Step 3: Write an operator"] = "第 3 步：编写算子"
    t["Operators run in-process inside a shared runtime, avoiding the overhead of a separate process. They implement a simple class:"] = \
        "算子在共享运行时中进程内运行，避免了独立进程的开销。它们实现一个简单的类："
    t["Step 4: Write a receiver node"] = "第 4 步：编写接收节点"
    t["Step 5: Run the dataflow"] = "第 5 步：运行数据流"
    t["You should see the receiver printing transformed values."] = \
        "你应该会看到接收者打印出转换后的值。"
    t["Node API Basics"] = "节点 API 基础"
    t["Creating a node"] = "创建节点"
    t["Event loop"] = "事件循环"
    t["Event loop (iterator)"] = "事件循环（迭代器）"
    t["Sending output"] = "发送输出"
    t["You must send Arrow data. Common conversions:"] = \
        "你必须发送 Arrow 数据。常见转换方式："
    t["Sending output with metadata"] = "带元数据发送输出"
    t["Operator Basics"] = "算子基础"
    t["Operators are Python classes loaded by the runtime. Define an `Operator` class:"] = \
        "算子是由运行时加载的 Python 类。定义一个 `Operator` 类："
    t["Async Nodes"] = "异步节点"
    t["For I/O-bound workloads, use Python's `asyncio`:"] = \
        "对于 I/O 密集型工作负载，使用 Python 的 `asyncio`："
    t["**Key rules for async nodes:**"] = "**异步节点要点：**"
    t["Use `await node.async_next()` instead of the sync iterator"] = \
        "使用 `await node.async_next()` 代替同步迭代器"
    t["Each event must be awaited before processing the next"] = \
        "每个事件必须在处理下一个之前 await"
    t["Backpressure still works through the node's input queues"] = \
        "背压仍然通过节点的输入队列工作"
    t["Logging in nodes works via Python's `logging` module or direct `print()`:"] = \
        "节点中的日志通过 Python 的 `logging` 模块或直接 `print()` 工作："
    t["Log messages are captured and routed by the dora daemon according to the logging configuration in your dataflow descriptor."] = \
        "日志消息由 dora 守护进程根据数据流描述符中的日志配置捕获和路由。"
    t["Dataflow Builder (Programmatic API)"] = "数据流构建器（编程 API）"
    t["As an alternative to YAML, you can build dataflows in Python:"] = \
        "作为 YAML 的替代方案，你可以用 Python 构建数据流："
    t["This is useful for:"] = "这适用于："
    t["Dynamic dataflow construction (e.g., different configs per run)"] = \
        "动态数据流构建（例如每次运行不同配置）"
    t["IDE autocompletion and type checking"] = \
        "IDE 自动补全和类型检查"
    t["Programmatic testing of dataflow definitions"] = \
        "数据流定义的编程测试"
    t["[Python API Reference](../languages/python.md) -- full API docs for Node, Operator, DataflowBuilder, CUDA"] = \
        "[Python API 参考](../languages/python.md) -- Node、Operator、DataflowBuilder、CUDA 完整 API 文档"
    t["[Communication Patterns](../concepts/patterns.md) -- service (request/reply) and action (goal/feedback/result) patterns"] = \
        "[通信模式](../concepts/patterns.md) -- 服务（请求/应答）和动作（目标/反馈/结果）模式"
    t["[Examples](../../../examples/) -- python-dataflow, python-async, python-drain, python-concurrent-rw, python-multiple-arrays"] = \
        "[示例](../../../examples/) -- python-dataflow、python-async、python-drain、python-concurrent-rw、python-multiple-arrays"
    t["[Distributed Deployment](../operations/distributed.md) -- running across multiple machines with `dora up`"] = \
        "[分布式部署](../operations/distributed.md) -- 使用 `dora up` 跨多台机器运行"

    # =============================================
    # concepts/architecture.md
    # =============================================
    t["Overview"] = "概述"
    t["Layer"] = "层级"
    t["Protocol"] = "协议"
    t["Purpose"] = "用途"
    t["CLI \\<\\-> Coordinator"] = "CLI \\<\\-> 协调器"
    t["WebSocket (port 6013)"] = "WebSocket（端口 6013）"
    t["Build, run, stop commands"] = "构建、运行、停止命令"
    t["Coordinator \\<\\-> Daemon"] = "协调器 \\<\\-> 守护进程"
    t["TCP"] = "TCP"
    t["Node spawning, dataflow lifecycle"] = "节点生成、数据流生命周期"
    t["Daemon \\<\\-> Daemon"] = "守护进程 \\<\\-> 守护进程"
    t["Zenoh"] = "Zenoh"
    t["Distributed cross-machine communication"] = "分布式跨机器通信"
    t["Daemon \\<\\-> Node"] = "守护进程 \\<\\-> 节点"
    t["Shared memory / TCP"] = "共享内存 / TCP"
    t["Zero-copy IPC for data >4KB, TCP for small messages"] = "大于 4KB 的数据零拷贝 IPC，小消息用 TCP"
    t["Key Components"] = "关键组件"
    t["**Coordinator** -- orchestrates dataflow lifecycle across daemons. Supports in-memory or persistent (redb) state store."] = \
        "**协调器** -- 跨守护进程编排数据流生命周期。支持内存或持久化（redb）状态存储。"
    t["**Daemon** -- spawns and manages nodes on a single machine. Handles shared memory allocation and message routing."] = \
        "**守护进程** -- 在单台机器上生成和管理节点。处理共享内存分配和消息路由。"
    t["**Runtime** -- in-process operator execution engine. Operators run inside the runtime process, avoiding per-operator process overhead."] = \
        "**运行时** -- 进程内算子执行引擎。算子在运行时进程内运行，避免每个算子的进程开销。"
    t["**Nodes** -- standalone processes that communicate via inputs/outputs. Written in Rust, Python, C, or C++."] = \
        "**节点** -- 通过输入/输出通信的独立进程。可用 Rust、Python、C 或 C++ 编写。"
    t["**Operators** -- lightweight functions that run inside the runtime. Faster than nodes for simple transformations."] = \
        "**算子** -- 在运行时内运行的轻量级函数。对于简单转换比节点更快。"
    t["Data Format"] = "数据格式"
    t["All data flows through the system as **Apache Arrow** columnar arrays. This enables zero-copy shared memory transfer between co-located nodes and zero-serialization overhead."] = \
        "所有数据以 **Apache Arrow** 列式数组的形式在系统中流转。这使得同机节点间零拷贝共享内存传输和零序列化开销成为可能。"
    t["Workspace Layout"] = "工作空间布局"

    # =============================================
    # concepts/dataflow-yaml.md
    # =============================================
    t["Dataflows are defined in YAML files. Each node declares its binary/script, inputs, and outputs."] = \
        "数据流在 YAML 文件中定义。每个节点声明其二进制文件/脚本、输入和输出。"
    t["Minimal Example"] = "最小示例"
    t["Full Schema"] = "完整模式"
    t["Built-in Timer Nodes"] = "内置定时器节点"
    t["Timers are virtual nodes that emit ticks at fixed intervals:"] = \
        "定时器是以固定间隔发出 tick 的虚拟节点："
    t["Input Format"] = "输入格式"
    t["Inputs subscribe to another node's output using the format `<node-id>/<output-name>`:"] = \
        "输入使用 `<node-id>/<output-name>` 格式订阅另一个节点的输出："
    t["Operator Nodes"] = "算子节点"
    t["Operators run in-process inside a shared runtime (no separate process):"] = \
        "算子在共享运行时中进程内运行（无独立进程）："
    t["Assign nodes to specific machines using `_unstable_deploy`:"] = \
        "使用 `_unstable_deploy` 将节点分配到特定机器："
    t["When nodes are on different machines, communication automatically switches from shared memory to Zenoh pub/sub."] = \
        "当节点位于不同机器时，通信自动从共享内存切换到 Zenoh 发布/订阅。"

    # =============================================
    # concepts/patterns.md
    # =============================================
    t["Dora is a dataflow framework based on pub/sub message passing. On top of basic topics, the framework provides two higher-level communication patterns using well-known metadata keys. These patterns require **no daemon or YAML changes** -- they work over normal dataflow connections."] = \
        "Dora 是一个基于发布/订阅消息传递的数据流框架。在基本主题之上，框架使用预定义的元数据键提供了两种高级通信模式。这些模式**不需要修改守护进程或 YAML** -- 它们在普通数据流连接上运行。"
    t["1. Topic (pub/sub)"] = "1. 主题（发布/订阅）"
    t["The default pattern. A node publishes data on an output, and any node that subscribes to that output receives it. No metadata keys required."] = \
        "默认模式。节点在输出上发布数据，任何订阅该输出的节点都会接收到。不需要元数据键。"
    t["2. Service (request/reply)"] = "2. 服务（请求/应答）"
    t["For synchronous request/reply communication between nodes."] = \
        "用于节点间同步请求/应答通信。"
    t["3. Action (goal/feedback/result)"] = "3. 动作（目标/反馈/结果）"
    t["For long-running tasks that provide progress feedback and can be cancelled."] = \
        "用于提供进度反馈且可被取消的长时间运行任务。"
    t["publisher"] = "发布者"
    t["subscriber"] = "订阅者"
    t["client"] = "客户端"
    t["server"] = "服务端"
    t["How It Works"] = "工作原理"
    t["YAML Setup"] = "YAML 配置"
    t["Server Implementation (Rust)"] = "服务端实现（Rust）"
    t["Client Implementation (Rust)"] = "客户端实现（Rust）"
    t["Server Implementation (Python)"] = "服务端实现（Python）"
    t["Client Implementation (Python)"] = "客户端实现（Python）"
    t["Action Client (Rust)"] = "动作客户端（Rust）"
    t["Action Server (Rust)"] = "动作服务端（Rust）"
    t["Action Client (Python)"] = "动作客户端（Python）"
    t["Action Server (Python)"] = "动作服务端（Python）"
    t["Cancellation"] = "取消"
    t["Best Practices"] = "最佳实践"
    t["Pattern Summary"] = "模式总结"
    t["Pattern"] = "模式"
    t["Use case"] = "用例"
    t["Key metadata"] = "关键元数据"
    t["Topic"] = "主题"
    t["Service"] = "服务"
    t["Action"] = "动作"
    t["Streaming data, events"] = "流式数据、事件"
    t["Quick query, RPC"] = "快速查询、RPC"
    t["Long tasks with progress"] = "带进度的长时间任务"
    t["None"] = "无"

    # =============================================
    # Common table headers and terms used across files
    # =============================================
    t["Command"] = "命令"
    t["Flag"] = "标志"
    t["Example"] = "示例"
    t["Examples"] = "示例"
    t["Usage"] = "用法"
    t["Status"] = "状态"
    t["Notes"] = "备注"
    t["Type"] = "类型"
    t["Name"] = "名称"
    t["Value"] = "值"
    t["Parameter"] = "参数"
    t["Parameters"] = "参数"
    t["Returns"] = "返回值"
    t["Return value"] = "返回值"
    t["Method"] = "方法"
    t["Fields"] = "字段"
    t["Argument/Flag"] = "参数/标志"
    t["required"] = "必需"
    t["Table of Contents"] = "目录"
    t["See Also"] = "另请参阅"
    t["Troubleshooting"] = "故障排除"
    t["Common Issues"] = "常见问题"
    t["Debug Workflow"] = "调试工作流"
    t["Configuration"] = "配置"
    t["Requirements"] = "要求"
    t["Limitations"] = "限制"
    t["Environment Variables"] = "环境变量"
    t["Variable"] = "变量"
    t["Commands"] = "命令"
    t["Related"] = "相关"
    t["Summary"] = "目录"
    t["Node"] = "节点"

    # =============================================
    # operations/cli.md - Key prose
    # =============================================
    t["Dora CLI Reference"] = "Dora CLI 参考"
    t["Dora (AI-Dora, Dataflow-Oriented Robotic Architecture) is a 100% Rust framework for building real-time robotics and AI applications. This document covers the `dora` CLI from both an end-user and developer perspective."] = \
        "Dora（AI-Dora，数据流导向机器人架构）是一个 100% Rust 的实时机器人与 AI 应用框架。本文档从终端用户和开发者两个角度介绍 `dora` CLI。"
    t["Core Concepts"] = "核心概念"
    t["Dataflow"] = "数据流"
    t["Execution Modes"] = "执行模式"
    t["Component Roles"] = "组件角色"
    t["Data Format"] = "数据格式"
    t["Dataflow Descriptor"] = "数据流描述符"
    t["Command Reference"] = "命令参考"
    t["Lifecycle Commands"] = "生命周期命令"
    t["Monitoring Commands"] = "监控命令"
    t["Setup Commands"] = "设置命令"
    t["Utility Commands"] = "实用命令"
    t["Self-Management Commands"] = "自管理命令"
    t["Architecture Guide"] = "架构指南"
    t["Communication Stack"] = "通信栈"
    t["Protocol Layers"] = "协议层"
    t["Coordinator Internals"] = "协调器内部机制"
    t["Daemon Internals"] = "守护进程内部机制"
    t["Message Types"] = "消息类型"
    t["Timestamping"] = "时间戳"
    t["Zero-Copy Shared Memory"] = "零拷贝共享内存"
    t["Writing Nodes"] = "编写节点"
    t["Writing Operators"] = "编写算子"
    t["Rust Node"] = "Rust 节点"
    t["Python Node"] = "Python 节点"
    t["C Node"] = "C 节点"
    t["Node Logging"] = "节点日志"
    t["Rust Operator"] = "Rust 算子"
    t["Python Operator"] = "Python 算子"
    t["Distributed Deployments"] = "分布式部署"
    t["Setup"] = "设置"
    t["Dataflow with Machine Assignment"] = "带机器分配的数据流"
    t["Build and Start"] = "构建和启动"
    t["Monitor"] = "监控"
    t["Coordinator Persistence"] = "协调器持久化"
    t["Log File Locations"] = "日志文件位置"
    t["Mode"] = "模式"
    t["Infrastructure"] = "基础设施"
    t["Use case"] = "用例"
    t["**Local**"] = "**本地**"
    t["**Distributed**"] = "**分布式**"
    t["Development, testing, single-machine"] = "开发、测试、单机"
    t["Production, multi-machine"] = "生产、多机"
    t["A **dataflow** is a directed graph of nodes connected by typed data channels. Nodes produce **outputs** that other nodes consume as **inputs**. The framework handles data routing, serialization (Apache Arrow), and lifecycle management."] = \
        "**数据流**是由类型化数据通道连接的节点有向图。节点产生**输出**供其他节点作为**输入**消费。框架处理数据路由、序列化（Apache Arrow）和生命周期管理。"
    t["All data flows through the system as **Apache Arrow** columnar arrays. This enables zero-copy shared memory transfer between co-located nodes and zero-serialization overhead."] = \
        "所有数据以 **Apache Arrow** 列式数组的形式在系统中流转。这使得同机节点间零拷贝共享内存传输和零序列化开销成为可能。"
    t["**CLI**: User interface. Sends commands, displays logs."] = \
        "**CLI**：用户界面。发送命令、显示日志。"
    t["**Coordinator**: Orchestrates dataflow lifecycle across machines."] = \
        "**协调器**：跨机器编排数据流生命周期。"
    t["**Daemon**: Spawns node processes, manages IPC, collects metrics."] = \
        "**守护进程**：生成节点进程、管理 IPC、收集指标。"
    t["**Node**: A standalone process that produces and consumes Arrow data."] = \
        "**节点**：产生和消费 Arrow 数据的独立进程。"
    t["**Operator**: In-process code running inside a shared runtime (lower latency than nodes)."] = \
        "**算子**：在共享运行时内运行的进程内代码（比节点更低延迟）。"

    # CLI command descriptions
    t["Run a dataflow locally without coordinator or daemon. Best for development and testing."] = \
        "在本地运行数据流，无需协调器或守护进程。适合开发和测试。"
    t["Start coordinator and daemon in local mode."] = \
        "在本地模式下启动协调器和守护进程。"
    t["Tear down coordinator and daemon. Stops all running dataflows first."] = \
        "拆卸协调器和守护进程。首先停止所有运行中的数据流。"
    t["Run build commands defined in the dataflow descriptor."] = \
        "运行数据流描述符中定义的构建命令。"
    t["Start a dataflow on a running coordinator."] = \
        "在运行中的协调器上启动数据流。"
    t["Stop a running dataflow."] = "停止运行中的数据流。"
    t["Restart a running dataflow (stop + re-start with stored descriptor). No YAML path needed -- the coordinator retains the original descriptor."] = \
        "重启运行中的数据流（停止 + 使用存储的描述符重新启动）。无需 YAML 路径 -- 协调器保留了原始描述符。"
    t["List running dataflows with metrics."] = "列出运行中的数据流及指标。"
    t["Show and follow logs of a dataflow and node."] = "显示和跟踪数据流与节点的日志。"
    t["Check system health and connectivity."] = "检查系统健康状态和连接性。"
    t["Generate a new project or node from templates."] = "从模板生成新的项目或节点。"
    t["Visualize a dataflow as a graph."] = "以图形方式可视化数据流。"
    t["Generate shell completion scripts."] = "生成 shell 补全脚本。"
    t["Check for and install CLI updates."] = "检查并安装 CLI 更新。"
    t["Remove the CLI from the system."] = "从系统中移除 CLI。"

    # =============================================
    # operations/logging.md - Key terms
    # =============================================
    t["Dora provides a structured logging system for real-time robotics and AI dataflows. Logs are captured by daemons, routed to the coordinator, and viewable through the CLI."] = \
        "Dora 提供了一个用于实时机器人和 AI 数据流的结构化日志系统。日志由守护进程捕获、路由到协调器，并可通过 CLI 查看。"
    t["Features at a Glance"] = "功能一览"
    t["Scope"] = "范围"
    t["Config"] = "配置"
    t["Log level filtering"] = "日志级别过滤"
    t["Log rotation"] = "日志轮转"
    t["Log sinks"] = "日志输出目标"
    t["Structured format"] = "结构化格式"
    t["Node stdout capture"] = "节点标准输出捕获"
    t["Log streaming"] = "日志流"
    t["Log Levels"] = "日志级别"
    t["Log Rotation"] = "日志轮转"
    t["Log Sinks"] = "日志输出目标"
    t["Log Format"] = "日志格式"
    t["Viewing Logs"] = "查看日志"
    t["Filtering"] = "过滤"
    t["Language Integration"] = "语言集成"
    t["Internals"] = "内部机制"

    # =============================================
    # operations/debugging.md - Key terms
    # =============================================
    t["Debugging and Observability Guide"] = "调试与可观测性指南"
    t["This guide covers how to debug, record, replay, and monitor dora dataflows. It is written for new users who need to understand what's happening inside their dataflows."] = \
        "本指南介绍如何调试、录制、回放和监控 dora 数据流。面向需要了解数据流内部运行情况的新用户。"
    t["Quick Debugging Checklist"] = "快速调试清单"
    t["Record and Replay"] = "录制和回放"
    t["Topic Inspection"] = "主题检查"
    t["Resource Monitoring"] = "资源监控"
    t["Trace Inspection"] = "追踪检查"
    t["Log Analysis"] = "日志分析"
    t["End-to-End Scenarios"] = "端到端场景"
    t["[CLI Reference](cli.md) -- complete command reference"] = \
        "[CLI 参考](cli.md) -- 完整命令参考"
    t["[WebSocket Control Plane](../advanced/ws-control.md) -- how CLI communicates with coordinator"] = \
        "[WebSocket 控制面](../advanced/ws-control.md) -- CLI 如何与协调器通信"
    t["[WebSocket Topic Data Channel](../advanced/ws-topic.md) -- how topic data is proxied"] = \
        "[WebSocket 主题数据通道](../advanced/ws-topic.md) -- 主题数据如何代理"
    t["[Testing Guide](../development/testing.md) -- running smoke tests"] = \
        "[测试指南](../development/testing.md) -- 运行冒烟测试"

    # =============================================
    # operations/fault-tolerance.md - Key terms
    # =============================================
    t["Dora provides built-in fault tolerance for robotic and AI dataflows. Nodes can automatically restart on failure, inputs can detect upstream problems, and the coordinator persists state across crashes."] = \
        "Dora 为机器人和 AI 数据流提供内置容错。节点可以在故障时自动重启，输入可以检测上游问题，协调器可以跨崩溃持久化状态。"
    t["Restart policies"] = "重启策略"
    t["Restart Policies"] = "重启策略"
    t["Health Monitoring"] = "健康监测"
    t["Circuit Breaker Pattern"] = "熔断器模式"
    t["Input Timeout (Circuit Breaker)"] = "输入超时（熔断器）"
    t["Coordinator State Persistence"] = "协调器状态持久化"
    t["Recovery Behavior"] = "恢复行为"
    t["Operational Guide"] = "运维指南"

    # =============================================
    # operations/distributed.md - Key terms
    # =============================================
    t["Distributed Deployment Guide"] = "分布式部署指南"
    t["Dora supports deploying dataflows across multiple machines for multi-robot fleets, edge AI pipelines, and cloud-robot hybrid architectures."] = \
        "Dora 支持将数据流部署到多台机器上，用于多机器人集群、边缘 AI 流水线和云-机器人混合架构。"
    t["Cluster Configuration Reference"] = "集群配置参考"
    t["Cluster Lifecycle"] = "集群生命周期"
    t["Label Scheduling"] = "标签调度"
    t["Rolling Upgrades"] = "滚动升级"
    t["Auto-Recovery"] = "自动恢复"
    t["Systemd Integration"] = "Systemd 集成"
    t["Security"] = "安全"
    t["Operational Runbooks"] = "运维手册"

    # =============================================
    # advanced/ros2-bridge.md - Key terms
    # =============================================
    t["Dora provides a declarative YAML-based ROS2 bridge that lets any Dora node communicate with ROS2 topics, services, and actions without modifying application code."] = \
        "Dora 提供了一个基于 YAML 声明式的 ROS2 桥接，让任何 Dora 节点无需修改应用代码即可与 ROS2 的主题、服务和动作通信。"
    t["Topic subscribe"] = "主题订阅"
    t["Topic publish"] = "主题发布"
    t["Service client"] = "服务客户端"
    t["Service server"] = "服务服务端"
    t["Action client"] = "动作客户端"
    t["Action server"] = "动作服务端"
    t["QoS Configuration"] = "QoS 配置"
    t["Type System"] = "类型系统"
    t["Programmatic API"] = "编程 API"
    t["Bridge Architecture"] = "桥接架构"

    # =============================================
    # advanced/ws-control.md - Key terms
    # =============================================
    t["Dora's control plane uses WebSocket connections for all communication between the CLI, coordinator, and daemons. This document describes the protocol."] = \
        "Dora 的控制面使用 WebSocket 连接进行 CLI、协调器和守护进程之间的所有通信。本文档描述了该协议。"
    t["Detail"] = "详情"
    t["Routes"] = "路由"
    t["Wire format"] = "传输格式"
    t["Authentication"] = "认证"

    # =============================================
    # advanced/ws-topic.md - Key terms
    # =============================================
    t["WebSocket Topic Data Channel"] = "WebSocket 主题数据通道"
    t["The topic data channel extends the WebSocket control plane to proxy live dataflow messages from the coordinator to CLI clients."] = \
        "主题数据通道扩展了 WebSocket 控制面，将实时数据流消息从协调器代理到 CLI 客户端。"
    t["Motivation"] = "动机"
    t["Scenario"] = "场景"

    # =============================================
    # development/testing.md - Key terms
    # =============================================
    t["Dora Testing Guide"] = "Dora 测试指南"
    t["This guide covers how to run, write, and troubleshoot tests across the Dora workspace."] = \
        "本指南介绍如何在 Dora 工作空间中运行、编写和排查测试。"
    t["Quick Start (5-minute validation)"] = "快速开始（5 分钟验证）"
    t["Run these three commands to validate that the workspace is healthy:"] = \
        "运行这三个命令来验证工作空间是否健康："
    t["Test Organization"] = "测试组织"
    t["Writing Tests"] = "编写测试"
    t["Unit Tests"] = "单元测试"
    t["Integration Tests"] = "集成测试"
    t["Smoke Tests"] = "冒烟测试"
    t["End-to-End Tests"] = "端到端测试"
    t["Test Infrastructure"] = "测试基础设施"
    t["CI Integration"] = "CI 集成"

    # =============================================
    # languages/rust.md - Key terms
    # =============================================
    t["Rust API Reference"] = "Rust API 参考"
    t["This document covers the two main Rust crates for building Dora dataflow components:"] = \
        "本文档介绍用于构建 Dora 数据流组件的两个主要 Rust crate："
    t["**`dora-node-api`** -- for standalone node executables"] = \
        "**`dora-node-api`** -- 用于独立节点可执行文件"
    t["**`dora-operator-api`** -- for in-process operators managed by the Dora runtime"] = \
        "**`dora-operator-api`** -- 用于由 Dora 运行时管理的进程内算子"
    t["Node API (`dora-node-api`)"] = "节点 API (`dora-node-api`)"
    t["Initialization"] = "初始化"
    t["Event Loop"] = "事件循环"
    t["Sending Output"] = "发送输出"
    t["Service and Action Helpers"] = "服务与动作辅助方法"
    t["Data Allocation"] = "数据分配"
    t["Operator API (`dora-operator-api`)"] = "算子 API (`dora-operator-api`)"
    t["Metadata"] = "元数据"
    t["Error Handling"] = "错误处理"

    # =============================================
    # languages/python.md - Key terms
    # =============================================
    t["Python API Reference"] = "Python API 参考"
    t["Node API"] = "节点 API"
    t["Node class"] = "Node 类"
    t["Event types"] = "事件类型"
    t["Operator API"] = "算子 API"
    t["Operator class"] = "Operator 类"
    t["DataflowBuilder API"] = "DataflowBuilder API"
    t["CUDA Support"] = "CUDA 支持"

    # =============================================
    # languages/c.md - Key terms
    # =============================================
    t["C API Reference"] = "C API 参考"
    t["This document covers the two C APIs provided by the Dora framework: the **Node API** for standalone nodes and the **Operator API** for in-process operators."] = \
        "本文档介绍 Dora 框架提供的两个 C API：用于独立节点的**节点 API** 和用于进程内算子的**算子 API**。"
    t["Node API (dora-node-api-c)"] = "节点 API (dora-node-api-c)"
    t["Operator API (dora-operator-api-c)"] = "算子 API (dora-operator-api-c)"
    t["Memory Management"] = "内存管理"
    t["Thread Safety"] = "线程安全"

    # =============================================
    # languages/cpp.md - Key terms
    # =============================================
    t["C++ API Reference"] = "C++ API 参考"
    t["Service and Action Patterns"] = "服务与动作模式"
    t["ROS2 Integration"] = "ROS2 集成"
    t["Building"] = "构建"
    t["Crate"] = "Crate"
    t["Library"] = "库"

    return t


def load_json_translations(po_dir):
    """Load translations from agent-generated JSON files."""
    import json
    merged = {}
    json_files = [
        'group1_translations.json',
        'group2_translations.json',
        'group3_translations.json',
        'remaining_translations.json',
    ]
    for name in json_files:
        path = po_dir / name
        if path.exists():
            data = json.load(open(path))
            merged.update(data)
            print(f"  Loaded {len(data)} from {name}")
    return merged


def main():
    po_path = Path('po/zh-CN.po')
    po_dir = Path('po')
    entries = parse_po(str(po_path))
    translations = build_translations()

    # Merge agent-generated translations (JSON files override dictionary)
    json_translations = load_json_translations(po_dir)
    translations.update(json_translations)

    print(f"Loaded {len(entries)} .po entries")
    print(f"Translation dictionary has {len(translations)} entries")

    # Count matches
    matched = 0
    for entry in entries:
        if entry['msgid'] in translations:
            matched += 1

    print(f"Matched {matched} entries ({matched*100//max(len(entries),1)}%)")

    write_po(str(po_path), entries, translations)
    print(f"Written to {po_path}")


if __name__ == '__main__':
    main()
