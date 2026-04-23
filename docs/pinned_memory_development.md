# Dora 页锁内存功能开发文档

## 概述

本文档记录了在Dora框架中实现CPU到CUDA跨进程张量高速传输功能的开发过程。通过页锁内存（Pinned Memory）和DMA传输机制，从共享内存中读取数据后经由DMA传输至GPU。

当前传输速率约1500MB/s（受限于PCIe带宽和共享内存复制开销），目标为30000MB/s。

## 功能架构

```
┌───────────────────────────────────────────────────────────────┐
│                     Python Node (Sender)                      │
│  1. torch_to_ptr(tensor) → (ptr_array, metadata)              │
│  2. node.register_pinned_memory(ptr_array, metadata)          │
│     → pinned_buffer (shared memory identifier)                │
│     Create shmem, copy data, register with daemon             │
└──────────────────────────┬────────────────────────────────────┘
                           │
┌──────────────────────────▼────────────────────────────────────┐
│              Dora Daemon (Memory Manager)                     │
│  pinned_memory_table: HashMap<Id, PinnedMemoryMetadata>       │
│  - register: stores metadata keyed by buffer ID               │
│  - read: returns metadata for a given buffer ID               │
│  - free: removes entry from table                             │
└──────────────────────────┬────────────────────────────────────┘
                           │
┌──────────────────────────▼────────────────────────────────────┐
│                     Python Node (Receiver)                     │
│  node.read_pinned_memory(pinned_buffer, free=True)             │
│    → (ptr_array, metadata)                                    │
│    1. Query daemon for metadata (free=false to avoid race)    │
│    2. Map shared memory from /dev/shm                         │
│    3. cudaHostRegister + cudaMalloc + cudaMemcpy (DMA)        │
│    4. If free=True, notify daemon to remove entry             │
│    5. Return device pointer and metadata                      │
│                                                               │
│  ptr_to_torch(ptr_array, metadata) → torch.Tensor             │
│    - Zero-copy wraps pointer as tensor via __cuda_array_iface__│
│    - Supports CPU and CUDA pointers                           │
└───────────────────────────────────────────────────────────────┘
```

## 修改的文件列表

### Rust 核心文件

| 文件 | 内容 |
|------|------|
| `dora/binaries/daemon/src/memory_manager.rs` | 页锁内存管理器：pinned_memory_table 注册/读取/释放，freed:bool double-free 检测 |
| `dora/binaries/daemon/src/lib.rs` | 处理 RegisterPinnedMemory / ReadPinnedMemory / FreePinnedMemory 事件 |
| `dora/apis/rust/node/src/node/mod.rs` | DoraNode 的 register/read/free_pinned_memory 方法 |
| `dora/apis/rust/node/src/node/control_channel.rs` | ControlChannel 的页锁内存通信方法 |
| `dora/libraries/message/src/node_to_daemon.rs` | DaemonRequest 枚举变体 |
| `dora/libraries/message/src/daemon_to_node.rs` | DaemonReply::PinnedMemoryMetadata |

### Python API 文件

| 文件 | 内容 |
|------|------|
| `dora/apis/python/node/src/lib.rs` | Python 绑定：register_pinned_memory_internal, read_pinned_memory, free_pinned_memory, pin_and_dma_to_gpu；全局静态：GPU_ALLOCATIONS, LAST_HOST_PTR, PINNED_COUNTER |
| `dora/apis/python/node/dora/cuda.py` | 辅助函数：torch_to_ptr, ptr_to_torch |

## API 参考

### dora.cuda 模块

#### `torch_to_ptr(tensor) -> (pa.array, dict)`
将 PyTorch 张量解析为指针数组和元数据字典。仅负责解析，不涉及内存注册。

#### `ptr_to_torch(ptr_array, metadata) -> torch.Tensor`
根据指针和元数据创建 PyTorch 张量（零拷贝）。自动判断指针位置：CPU 指针生成 CPU 张量，CUDA 指针生成 CUDA 张量。

### Node 类方法

#### `node.register_pinned_memory(ptr_array, metadata) -> pinned_buffer`
1. 从 ptr_array 提取数据指针
2. 创建共享内存（/dev/shm），通过 ptr::copy_nonoverlapping 复制数据
3. 向 daemon 注册内存元数据（存入 pinned_memory_table）
4. 返回共享内存标识符（StringArray）

#### `node.read_pinned_memory(pinned_buffer, free=True) -> (ptr_array, metadata)`
1. 向 daemon 查询元数据（固定传 free=false 避免竞态）
2. 映射共享内存获取 host 指针
3. 执行 cudaHostRegister + cudaMalloc + cudaMemcpy（DMA 传输至 GPU）
4. 若 free=True，通知 daemon 删除表项
5. 返回 device 指针和元数据

#### `node.free_pinned_memory(pinned_buffer)`
通知 daemon 从 pinned_memory_table 删除对应表项。

## 传输机制

### 数据流
1. **发送端**: 创建共享内存 → 复制数据 → 向 daemon 注册元数据 → 发送 buffer_id 给接收端
2. **接收端**: 通过 buffer_id 查询 daemon → 映射共享内存 → DMA 传输至 GPU → 返回 device 指针

### DMA 传输（pin_and_dma_to_gpu）
通过 Python ctypes 内联执行 CUDA 操作：
1. `free_old_gpu_allocations` — 释放前一次迭代的 GPU 缓冲区（防止内存泄漏）
2. `cudaHostRegister` — 将 host 内存注册为页锁内存（最大化 DMA 带宽）
3. `cudaMalloc` — 在 GPU 上分配设备内存
4. `cudaMemcpy` — 从页锁内存 DMA 传输至 GPU（HostToDevice）
5. `cudaDeviceSynchronize` — 等待传输完成

结果通过 Python 模块变量（`result_ptr`）返回，不再使用 stdout 捕获。

### 内存管理
### 内存管理
- **共享内存（发送端）**: 使用 shared_memory_extended 库在 /dev/shm 上创建。通过 `shmem.set_owner(false)` 允许 Drop 执行 munmap+close(fd) 但跳过 shm_unlink，确保共享内存名存活供接收端打开。不使用 `mem::forget`，否则每轮迭代泄漏约 61MB mmap 内存，100 轮后导致 SIGBUS 崩溃。
- **共享内存（接收端）**: 在 `read_pinned_memory` 中通过 `ShmemConf::new().os_id()` 打开共享内存，DMA 传输完成后不再 `mem::forget`，让 Drop 自然执行 munmap+close（该进程为打开者，owner 默认 false，不会 shm_unlink）。
- **Daemon 表项**: pinned_memory_table 跟踪所有注册。释放时条目标记为 `freed=true` 而非删除，以便检测重复释放和保留 size 信息用于警告日志。daemon 退出时自动清理所有未释放条目。`free_pinned_memory` 在 daemon 中同时清除 /dev/shm 文件。
- **GPU 内存**: `pin_and_dma_to_gpu` 中通过 cudaMalloc 分配设备内存。分配前调用 `free_old_gpu_allocations` 释放前一次迭代的所有 GPU 指针（全局列表 `GPU_ALLOCATIONS` 跟踪），确保同时最多仅有一个 GPU 缓冲区存活，防止 GPU 内存泄漏导致的性能衰减（此前导致 8.82→1.18 it/s 速度衰减的根因）。
- **cudaHostRegister 累积**: 每轮迭代的 `pin_and_dma_to_gpu` 调用 `cudaHostRegister` 将共享内存注册为页锁内存。全局 `LAST_HOST_PTR` 跟踪上一轮 host 指针，在新注册前调用 `cudaHostUnregister` 释放旧注册，防止页锁内存累积。
- **Buffer ID 唯一性**: 全局 `PINNED_COUNTER` 生成自增计数，buffer ID 格式为 `pinned_{ptr}_{counter}`。PyTorch 可能在 GC 后复用同一数据指针，若仅用 `pinned_{ptr}` 会导致 daemon 拒绝重复注册。

## 测试

### 集成测试（target_test）
- `test1.yml`: sender1 + receiver1（显式 free=False + 显式 free_pinned_memory），100 轮迭代，大型数据速度测试
- `test2.yml`: sender1 + receiver2（默认 free=True 自动清理 daemon 表项），10 轮迭代
- `test3.yml`: sender1 + receiver3 — 先 free_pinned_memory 释放，再 read_pinned_memory（free=True），测试重复释放警告
- `test4.yml`: sender1 + receiver4 — 同 receiver3，测试读取不存在的页锁内存警告

### 扩展模块测试
- `examples/pinned_memory/test_1/test_cuda_ext.py`: 测试 torch_to_ptr / ptr_to_torch 正确性

## 性能

当前传输速率约 1500MB/s，瓶颈分析：
- 共享内存读/写各一次复制
- PCIe Gen3 x16 理论带宽约 16GB/s
- 页锁内存 DMA 可达接近 PCIe 理论带宽

要达到 30000MB/s 需要根本性架构变更：
- GPU IPC（cudaIpcGetMemHandle / cudaIpcOpenMemHandle）跨进程直接 GPU 访问
- 或利用同一 GPU 内存分配跨进程共享

## 开发注意事项

1. `read_pinned_memory` 向 daemon 查询时始终传 free=false，随后通过独立调用 free_pinned_memory 清理，避免共享内存在接收方尚未 mmap 时被删除的竞态
2. GPU 内存在每次新分配前通过 `free_old_gpu_allocations` 释放前一批缓冲区。全局列表 `GPU_ALLOCATIONS` 跟踪所有未释放的设备指针，确保同时最多只有一个 GPU 缓冲区存活。进程退出时 CUDA 驱动自动回收剩余内存。
3. 发送端可能无 CUDA 设备，因此 cudaHostRegister 在接收端执行
4. 跨进程共享内存在不同进程中物理地址不同，cudaHostGetDevicePointer 不适用
5. 重复释放（double-free）检测：daemon 的 `PinnedMemoryEntry` 有 `freed: bool` 字段，标记释放而非删除条目，使 double-free 和 read-after-free 都能正确返回错误信息而不崩溃
