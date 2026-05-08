# Dora 页锁内存池开发文档

## 概述

本文档记录了在Dora框架中通过页锁内存池实现跨进程CPU↔CUDA张量高速传输的设计与实现。核心设计为**共享状态（shared-state）API**替代旧的每轮迭代"消息传递"模式：发送端注册一次内存池，之后只需写入新数据；接收端一次读取获得持久化tensor，其内容随发送端写入自动更新。

## 架构概览

```
┌──────────────────────────────────────────────────────────────┐
│                  Sender Node                                 │
│  1. tensor_info = get_tensor_info(tensor)                    │
│  2. memory_pool_id = node.register_memory_pool(tensor_info,  │
│                            receiver_device)                   │
│     → Pool slot assignment (round-robin 3 slots)             │
│     → DORADMA header write + data copy (DMA if CUDA)        │
│     → Register with daemon for lifecycle tracking            │
│     → Returns buffer_id like "pool_0_1"                     │
│  3. node.send_output("data", pa.array([memory_pool_id]))     │
│   ...                                                        │
│  4. tensor_info = get_tensor_info(new_tensor)                │
│  5. node.write_memory_pool(memory_pool_id, tensor_info)      │
│     → Fast path: DORADMA (open shmem, copy data)            │
│     → Slow path: daemon RPC                                 │
└───────────────────────┬──────────────────────────────────────┘
                        │ send_output(memory_pool_id)
┌───────────────────────▼──────────────────────────────────────┐
│              Dora Daemon (MemoryPoolManager)                  │
│  memory_pool: HashMap<MemoryPoolId, MemoryPoolEntry>         │
│  - register_memory_pool: stores metadata keyed by buffer ID  │
│  - free_memory_pool: removes entry from table                │
│  - cleanup_all: lists remaining entries at shutdown          │
│                                                              │
│  Pool shmems (dora_pool_0..2) are persistent ring buffers   │
└───────────────────────┬──────────────────────────────────────┘
                        │
┌───────────────────────▼──────────────────────────────────────┐
│                Receiver Node                                 │
│  1. memory_pool_id = event["value"]                          │
│  2. tensor_info = node.read_memory_pool(memory_pool_id)      │
│     → Fast path (DORADMA, pool_* buffers):                  │
│       1. Extract slot from buffer_id → "dora_pool_{slot}"   │
│       2. Open shmem, validate DORADMA magic header           │
│       3. Read JSON metadata from shmem header (no daemon!)   │
│       4. Resolve pointer: CUDA→GPU VA cache, CPU→shmem ptr  │
│       5. Return tensor_info dict {ptr, size, dtype, shape,   │
│                                    device}                    │
│     → Slow path: query daemon for metadata                   │
│  3. torch_tensor = tensor_from_info(tensor_info)             │
│     → Zero-copy wraps pointer via __cuda_array_interface__   │
│  4. torch_tensor auto-updates as sender calls write_...      │
│  5. node.free_memory_pool(memory_pool_id)                    │
└──────────────────────────────────────────────────────────────┘
```

## DORADMA 共享内存头格式

Pool-mode shmem 在头部嵌入元数据，消除 daemon 往返查询：

```
Offset  Size  Field
0       8     magic: b"DORADMA\x00"
8       8     json_len: u64 LE
16      8     data_offset: u64 LE
24      232   reserved (zeroed)
256     N     JSON metadata (size, dtype, shape, pinned_type)
256+N   M     tensor data
```

接收端打开 shmem 后：
1. 验证 magic → 不匹配则 fallback daemon 查询
2. 读取 json_len → JSON metadata → 获取 dtype/shape/size/pinned_type
3. 根据 pinned_type 决定 CPU/CUDA 指针解析路径

## 文件结构

### Rust 核心文件

| 文件 | 内容 |
|------|------|
| `binaries/daemon/src/memory_manager.rs` | MemoryPoolManager：register_memory_pool / read_memory_pool / free_memory_pool；cleanup_all 统计清理；pool shmem 持久化管理 |
| `binaries/daemon/src/lib.rs` | 处理 RegisterPinnedMemory / ReadPinnedMemory / FreePinnedMemory 事件；daemon 退出时调用 cleanup_all |
| `apis/rust/node/src/node/mod.rs` | DoraNode 的 register/read/free_pinned_memory 方法（底层 Rust API 保持原名） |
| `apis/rust/node/src/node/control_channel.rs` | ControlChannel 的页锁内存通信方法 |
| `libraries/message/src/node_to_daemon.rs` | DaemonRequest 枚举变体 |
| `libraries/message/src/daemon_to_node.rs` | DaemonReply 枚举变体 |

### Python API 文件

| 文件 | 内容 |
|------|------|
| `apis/python/node/src/lib.rs` | Python 绑定：register_memory_pool, write_memory_pool, read_memory_pool, free_memory_pool, try_doradma_read；预编译 CUDA helper 模块，持久化 GPU buffer pool |
| `apis/python/node/dora/cuda.py` | 辅助函数：get_tensor_info, tensor_from_info |

## API 参考

### dora.cuda 模块

#### `get_tensor_info(tensor) -> dict`
将 PyTorch 张量解析为字典，包含 ptr, size, dtype, shape, device。仅负责解析，不涉及内存注册。

#### `tensor_from_info(tensor_info) -> dict`
根据 tensor_info 字典创建 PyTorch 张量（零拷贝）。自动判断指针位置：CPU 指针→CPU 张量，CUDA 指针→CUDA 张量（通过 `__cuda_array_interface__`）。

### Node 类方法

#### `node.register_memory_pool(tensor_info, device) -> memory_pool_id`
1. 从 tensor_info 提取 ptr/size/dtype/shape/device
2. 轮询选择 pool slot（3 槽），创建/复用共享内存
3. 写入 DORADMA header（magic + json_len + data_offset + JSON metadata）
4. 拷贝数据：CUDA 源→cudaMemcpy，CPU 源→ptr::copy_nonoverlapping
5. cudaHostRegister 锁定 pool 内存（发送端预 pin）
6. 向 daemon 注册元数据
7. 返回 buffer_id（格式 `pool_{slot}_{counter}`，pyarrow StringArray）

#### `node.write_memory_pool(memory_pool_id, tensor_info)`
快速路径（DORADMA）：
1. 从 buffer_id 提取 slot 号，打开对应 shmem
2. 验证 DORADMA magic → 读取 data_offset
3. 拷贝数据到 data_offset 位置（cudaMemcpy 或 ptr::copy）
4. 失败时 fallback daemon 查询路径

#### `node.read_memory_pool(memory_pool_id) -> tensor_info`
快速路径（DORADMA）：
1. 从 buffer_id 提取 slot 号，打开对应 shmem
2. 验证 DORADMA magic → 读取 JSON 元数据
3. 根据 pinned_type 决定 CUDA/CPU 指针：
   - CUDA：RECV_GPU_VA cache → GPU VA + data_offset
   - CPU：RECV_CPU_SHMEM cache → shmem CPU ptr + data_offset
4. 返回 tensor_info dict {ptr, size, dtype, shape, device}

失败时 fallback daemon 查询路径。

#### `node.free_memory_pool(memory_pool_id)`
通知 daemon 从 memory_pool 表删除对应条目。daemon 返回未找到时输出警告。

## 数据传输流程

### 发送端
1. 首帧：`get_tensor_info` → `register_memory_pool` → `send_output`
2. 后续帧：`get_tensor_info` → `write_memory_pool` → `send_output`

### 接收端
1. 首帧：收到 memory_pool_id → `read_memory_pool` → `tensor_from_info`
2. 后续帧：tensor 自动更新（同一指针，数据已覆盖），无需重新读取
3. 结束：`free_memory_pool`

## 内存管理

### 共享内存（Pool 模式）
- 预分配 3 个持久化 shmem（`dora_pool_0`、`dora_pool_1`、`dora_pool_2`）
- 首次使用创建，后续迭代直接复用
- `shmem.set_owner(false)` — Drop 执行 munmap+close 但跳过 shm_unlink
- Pool shmem 在 daemon shutdown 时通过 `cleanup_all` 清理

### Daemon 表项
- `memory_pool` 跟踪所有注册的 buffer
- `free_memory_pool` 删除条目；再次 free 输出"未找到"警告
- `cleanup_all` 统计剩余条目，输出 INFO 日志

### 接收端缓存
- `RECV_GPU_VA[3]`：GPU VA cache，防止 munmap 导致 GPU VA 失效
- `RECV_CPU_SHMEM[3]`：CPU Shmem cache，防止 munmap 导致 CPU 指针悬空
- 首次访问时注册+缓存，后续复用

### GPU 内存
- 预编译 CUDA helper 模块维护 `_gpu_bufs: dict[slot→(d_ptr, size)]`
- 3 个 GPU buffer 对应 3 个 shmem slot，按 slot 复用
- `_register_host` 幂等 pinning（`_pinned` dict 去重）
- 无需 per-iteration 的 cudaMalloc/cudaFree

## 快速/慢速路径

| 操作 | 快速路径 | 慢速路径 |
|------|----------|----------|
| write_memory_pool | DORADMA header → 直接写 shmem data | daemon read_pinned_memory RPC |
| read_memory_pool | DORADMA header → 直接读 shmem metadata | daemon read_pinned_memory RPC |
| free_memory_pool | — | daemon free_pinned_memory RPC |
| register_memory_pool | — | daemon register_pinned_memory RPC |

## 开发注意事项

1. Pool 模式 free_memory_pool 通知 daemon 删除条目；再次 free 输出警告而非崩溃
2. try_doradma_read 中检测到无效 head 时返回 None 触发 fallback，而非 panic
3. read_memory_pool 的 daemon fallback 路径返回 tensor_info dict（与快速路径格式一致）
4. Python CUDA helper 模块一旦编译就缓存到 `CUDA_HELPERS` 静态变量
5. `cudaHostRegister` 去重：`_pinned` dict 防止同一 shmem 被重复 register
6. Pool shmem 的 `create` 在已有 shmem 时失败 → fallback `open`
7. RECV_GPU_VA 和 RECV_CPU_SHMEM cache 防止 Shmem 被 Drop 导致地址失效
8. Daemon 退出时自动 `cleanup_all`，输出 INFO 日志记录未释放条目数
9. `PINNED_POOL` 使用 `unsafe impl Send + Sync` 因为 `Shmem` 内部包含裸指针
