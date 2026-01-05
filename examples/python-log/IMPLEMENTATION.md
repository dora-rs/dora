# 新配置格式实现说明

## 概述

已成功实现了 Dora 的新配置格式，该格式将节点的图连接配置与节点自身的元数据定义分离。

## 文件结构

### 1. 旧格式（已支持，向后兼容）
**文件**: `dataflow.yaml`

```yaml
nodes:
  - id: send_data
    build: pip install asyncio
    path: ./send_data.py
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - data

  - id: receive_data_with_sleep
    build: pip install numpy pyarrow
    path: ./receive_data.py
    inputs:
      tick: send_data/data
```

**特点**:
- 所有信息混合在一起
- 每个节点包含路径、构建命令、输入输出等所有信息
- 不利于节点重用

### 2. 新格式（已实现）

**文件 1**: `dataflow 2.yaml` (或任何名称)
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

**文件 2**: `dora.yaml` (必须在同目录下)
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

**特点**:
- **graph**: 定义节点实例和连接关系
  - `id`: 节点实例的唯一标识符
  - `proto`: 引用 dora.yaml 中定义的节点原型
  - `inputs`: 定义数据来源（连接）
  
- **dora.yaml**: 定义可重用的节点原型
  - `name`: 节点原型名称
  - `entry`: 节点的执行入口
  - `build`: 构建命令
  - `inputs`/`outputs`: 输入输出的类型定义（元数据）

## 代码实现

### 核心文件修改

1. **`libraries/message/src/descriptor.rs`**
   - 添加了 `GraphNode` 结构体：表示图中的节点实例
   - 添加了 `NodeMetadataFile` 结构体：表示 dora.yaml 文件
   - 添加了 `NodeMetadata` 结构体：表示节点原型定义
   - 添加了 `InputDefinition` 和 `OutputDefinition`：输入输出的类型定义
   - 修改了 `Descriptor` 结构体，添加 `graph` 字段

2. **`libraries/core/src/descriptor/mod.rs`**
   - 实现了 `resolve_with_metadata()` 方法：从 graph 和 metadata 解析节点
   - 添加了 `load_descriptor_with_metadata()` 函数：自动加载 dora.yaml
   - 添加了 `resolve_descriptor_from_path()` 函数：完整的解析流程
   - 保留了 `resolve_legacy_format()` 函数：支持旧格式
   - 添加了 `merge_graph_node_with_prototype()` 函数：合并图节点和原型

3. **`libraries/core/src/descriptor/tests.rs`**
   - 添加了测试用例验证新旧格式的解析

## 使用方法

### 在代码中使用

```rust
use dora_core::descriptor::{resolve_descriptor_from_path, load_descriptor_with_metadata};
use std::path::Path;

// 自动检测并解析新旧格式
let resolved_nodes = resolve_descriptor_from_path(Path::new("dataflow.yaml"))?;

// 或者手动加载
let (descriptor, metadata) = load_descriptor_with_metadata(Path::new("dataflow 2.yaml"))?;
if let Some(metadata) = metadata {
    let resolved = descriptor.resolve_with_metadata(Some(&metadata))?;
}
```

### 解析逻辑

1. 检查 `Descriptor` 是否包含 `graph` 字段
2. 如果有 `graph` 字段：
   - 尝试从同目录加载 `dora.yaml`
   - 使用 `proto` 字段查找节点原型
   - 合并图节点配置和原型配置
3. 如果没有 `graph` 字段：
   - 使用旧的 `nodes` 字段
   - 按原来的方式解析

## 向后兼容性

- ✅ 旧格式完全兼容，无需修改现有配置文件
- ✅ 自动检测使用哪种格式
- ✅ 所有现有测试通过
- ✅ 验证逻辑自动支持两种格式

## 新格式的优势

1. **分离关注点**: 图的拓扑结构与节点实现分离
2. **节点重用**: 同一个节点原型可以在多个图中实例化
3. **更清晰的结构**: graph 文件只关注数据流，dora.yaml 只关注节点定义
4. **类型安全**: 输入输出可以定义类型和是否必需
5. **可导出性**: 节点原型可以标记为 export，方便共享

## 测试结果

所有测试通过：
```
running 3 tests
test descriptor::tests::tests::test_legacy_format ... ok
test descriptor::tests::tests::test_new_format ... ok
test descriptor::tests::tests::test_load_with_metadata ... ok

test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured
```

## 待办事项

- [ ] 在 CLI 命令中集成新格式支持（dora build, dora run 等）
- [ ] 添加验证逻辑以检查 proto 引用的有效性
- [ ] 支持从远程仓库导入 node prototypes
- [ ] 添加 schema 验证
- [ ] 更新文档和示例
