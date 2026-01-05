# Dora 新配置格式使用指南

## 快速开始

新的配置格式将节点图的连接配置与节点元数据定义分离，使配置更加清晰和可重用。

## 文件结构

### 1. 图配置文件（例如 `dataflow2.yaml`）

定义节点实例和它们之间的连接：

```yaml
graph:
  - id: send_data
    proto: "sender"
    inputs:
      tick: dora/timer/millis/10

  - id: receive_data_with_sleep
    proto: "receiver"
    inputs:
      tick: send_data/data
```

### 2. 节点元数据文件（`dora.yaml` 或 `dora.yml`）

定义可重用的节点原型：

```yaml
nodes:
  - name: receiver
    lang: python
    export: true
    entry: ./receive_data.py
    build: pip install numpy pyarrow

  - name: sender
    lang: python
    export: true
    entry: ./send_data.py
    build: pip install asyncio
    inputs:
      - name: tick
        type: number
        required: true
    outputs:
      - name: data
        type: number
```

## 使用方法

### 安装更新后的 dora-cli

```bash
# 从源代码安装
cd /path/to/dora
cargo install --path binaries/cli --force
```

### 构建和运行

```bash
# 构建 dataflow（会自动加载同目录下的 dora.yaml）
dora build dataflow2.yaml --uv

# 运行 dataflow
dora run dataflow2.yaml --uv
```

## 关键特性

1. **自动加载元数据**：当 dataflow 文件包含 `graph` 字段时，系统会自动从同目录下的 `dora.yaml` 或 `dora.yml` 加载节点元数据。

2. **向后兼容**：旧格式（使用 `nodes` 字段）仍然完全支持，无需修改现有配置。

3. **节点重用**：同一个节点原型可以在多个图中实例化。

4. **关注点分离**：
   - 图文件只关注数据流和连接
   - 元数据文件只关注节点实现细节

## 注意事项

- `dora.yaml` 必须与 dataflow 文件在同一目录下
- 图节点的 `proto` 字段必须匹配元数据文件中的节点 `name`
- 支持 `dora.yaml` 或 `dora.yml` 两种文件名

## 示例

查看 `examples/python-log/` 目录：
- `dataflow.yaml` - 旧格式（仍然支持）
- `dataflow2.yaml` - 新格式的图配置
- `dora.yaml` - 节点元数据定义
