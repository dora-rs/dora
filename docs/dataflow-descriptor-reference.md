# Dataflow 描述符字段参考

本文档列出 Dora 数据流 YAML 中所有节点字段及其适用的节点类型和层级。

## 节点类型

Dora 支持 6 种节点类型，通过以下互斥字段区分：

| YAML 字段 | 类型 | 说明 |
|----------|------|------|
| `path:` | Standard | 直接指定可执行文件路径 |
| `custom:` | Custom | 旧式节点配置（含 source 等） |
| `operators:` | Runtime | 多算子共享进程 |
| `operator:` | Operator | 单算子简写 |
| `ros2:` | ROS2 Bridge | ROS2 话题桥接 |
| `module:` | Module | 可复用子数据流 |

## 字段索引

### 通用字段（所有类型）

这些字段写在 Node 级（与 `id` 同级），对所有类型有效。

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` | NodeId | 唯一标识（必填） |
| `name` | String | 可读名称 |
| `description` | String | 功能描述 |
| `env` | Map | 环境变量（合并 dataflow 级 env） |
| `cpu_affinity` | List\<int\> | CPU 亲和性 |
| `deploy` | Deploy | 部署配置 |

### 输入输出字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `inputs` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `outputs` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `output_types` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `output_framing` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `input_types` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |

> ¹ Custom 节点：Node 级字段会被合并到 `custom:` 内部（子结构优先）。
> 推荐直接写在 `custom:` 内部。

### 通信模式字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `output_metadata` | Node 级 ✅ | **拒绝** ❌ | operator.config | operator.config | 拒绝 |
| `pattern` | Node 级 ✅ | **拒绝** ❌ | operator.config | operator.config | 拒绝 |

> `pattern` 和 `output_metadata` 仅在 Standard 节点 Node 级和 operator config 内合法。

### 重启策略字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `restart_policy` | Node 级 | **custom 内部**¹ | 拒绝² | 拒绝² | Node 级 |
| `max_restarts` | Node 级 | **custom 内部**¹ | 拒绝² | 拒绝² | Node 级 |
| `restart_delay` | Node 级 | **custom 内部**¹ | 拒绝² | 拒绝² | Node 级 |
| `max_restart_delay` | Node 级 | **custom 内部**¹ | 拒绝² | 拒绝² | Node 级 |
| `restart_window` | Node 级 | **custom 内部**¹ | 拒绝² | 拒绝² | Node 级 |

> ¹ Custom 节点：Node 级字段会被合并。
> ² Runtime/Operator 不支持重启策略。

### 运行时字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `health_check_timeout` | Node 级 | **custom 内部**¹ | 拒绝 | 拒绝 | Node 级 |
| `finish_grace_secs` | Node 级 | **custom 内部**¹ | 拒绝 | 拒绝 | Node 级 |
| `shared_memory_pool_size` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |

### 构建/源字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `path` | 必填 | — | — | — | — |
| `git` | 可选 | **拒绝** ❌ | 拒绝 | 拒绝 | — |
| `hub` | 拒绝³ | **拒绝** ❌ | 拒绝 | 拒绝 | — |
| `branch` | 可选 | **拒绝** ❌ | 拒绝 | 拒绝 | — |
| `tag` | 可选 | **拒绝** ❌ | 拒绝 | 拒绝 | — |
| `rev` | 可选 | **拒绝** ❌ | 拒绝 | 拒绝 | — |
| `build` | Node 级 | **custom 内部**¹ | operator.config | operator.config | — |
| `args` | Node 级 | **custom 内部**¹ | 拒绝 | 拒绝 | — |
| `path_sha256` | Node 级 | **custom 内部**¹ | 拒绝 | 拒绝 | 拒绝 |
| `custom.source` | — | 必填 | — | — | — |
| `operator.source` | — | — | — | 必填 | — |

> ¹ 合并到 CustomNode。³ `hub` + `path` 同时设置时报错。

### 日志字段

| 字段 | Standard | Custom | Runtime | Operator | ROS2 |
|------|:---:|:---:|:---:|:---:|:---:|
| `send_stdout_as` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `send_logs_as` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `min_log_level` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `max_log_size` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |
| `max_rotated_files` | Node 级 | **custom 内部**¹ | operator.config | operator.config | Node 级 |

## 完整示例

### Standard 节点

```yaml
nodes:
  - id: my_node
    path: target/release/my_node
    args: --verbose
    build: cargo build --release
    env:
      RUST_LOG: debug
    inputs:
      sensor_data: sensor/output
    outputs:
      - result
    output_metadata:
      response:
        - request_id
    pattern: service-server
    restart_policy: on-failure
    max_restarts: 5
    health_check_timeout: 60.0
    send_stdout_as: stdout
    min_log_level: info
```

### Custom 节点

```yaml
nodes:
  - id: my_custom
    custom:
      path: ./node.py
      source: Local
      # 推荐：所有配置写在 custom 内部
      outputs:
        - data
      restart_policy: always
      health_check_timeout: 30.0
      args: --debug
    # 也可以写在 Node 级（会被合并，但 custom 内部优先）
    env:
      PYTHONUNBUFFERED: "1"
```

### Runtime 节点

```yaml
nodes:
  - id: my_runtime
    operators:
      - id: processor
        shared-library: libproc.so
        inputs:
          data_in: source/out
        outputs:
          - result
        pattern: service-server
        send_stdout_as: op_out
```

### ROS2 Bridge 节点

```yaml
nodes:
  - id: bridge
    ros2:
      topic: /odom
      message_type: nav_msgs/Odometry
      direction: subscribe
      qos:
        reliable: true
    outputs:
      - odom
    restart_policy: on-failure
```
