# Dora 页锁内存池：面向 EuroSys 投稿的系统设计与创新

## 摘要

本文描述在 Dora 机器人数据流框架（fork from [dora-rs/dora](https://github.com/dora-rs/dora.git)）上实现的**页锁共享内存池**子系统。
通过共享状态（shared-state）API 替代上游的消息传递（message-passing）模型，引入 DORADMA 共享内存头部元数据旁路、持久化 PINNED_POOL 缓存
、GPU DMA 引擎传输与 CUDA IPC 跨进程零拷贝读取，将 CPU→CUDA 张量传输吞吐量从上游的 **~1500 MB/s**（应用层 CUDA IPC over Zenoh）提升至
**~5500 MB/s**（DMA 路径，端到端测量），加速比约 **3.7×**。

---

## 1. 上游 Dora 基线分析

### 1.1 数据平面架构

上游 Dora 采用**混合消息传递**模型，以 4 KB 为阈值分两条路径：

| 消息大小 | 数据路径 | 零拷贝 |
|---------|---------|--------|
| < 4 KB | 节点 → TCP → daemon（序列化 Arrow IPC）→ TCP/mpsc → 订阅者 | 无（多次序列化拷贝） |
| ≥ 4 KB | 节点 → Zenoh pub/sub（可选 SHM provider）→ 订阅者 | Zenoh SHM 场景下部分零拷贝 |

**关键限制**：
1. Daemon 作为中央路由枢纽，所有消息经由 daemon 的 `send_output_to_local_receivers` 函数通过 `mpsc` 通道扇出——即使数据走 Zenoh 旁路，元数据仍需 daemon 中转
2. Zenoh SHM provider 使用 `PosixShmProviderBackend`（8 MB 默认池），仅对 ≥ 512 B 的 Zenoh 消息生效，且为 best-effort（SHM 分配失败时回退堆拷贝）
3. 每次 `send_output` 调用均涉及 Arrow IPC 序列化（`copy_array_into_sample` 或 `encode_arrow_ipc`），产生至少一次 CPU 拷贝

### 1.2 GPU/CUDA 支持

上游 **在核心传输层无任何 GPU 支持**。唯一的 CUDA 代码位于 `apis/python/node/dora/cuda.py`，是纯应用层 Python 约定：

```python
# 发送端：手动导出 CUDA IPC handle，作为 pyarrow 数组发送
ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
node.send_output("latency", ipc_buffer, metadata)

# 接收端：手动导入 CUDA IPC handle，重建 tensor
ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
    ...
```

**问题**：
- CUDA IPC handle（64 字节）+ metadata 通过 Arrow 序列化 → TCP/Zenoh → 反序列化路径传输——每帧都要走完整的消息通道开销
- 每次迭代执行 `cudaIpcOpenMemHandle` + `cudaIpcCloseMemHandle`，无缓存
- Daemon、Rust 传输层、消息格式对 GPU 内存毫无感知——IPC handle 只是不透明的 64 字节 blob
- **无页锁内存**（pinned memory）、**无 GPU VA 映射**、**无 DMA 引擎传输**、**无内存池复用**

### 1.3 上游架构的固有开销

```
每帧发送路径开销：
  Arrow IPC 序列化     ~0.2ms
  Daemon TCP 往返       ~0.1-0.2ms
  mpsc 扇出             ~0.05ms
  Zenoh SHM 分配/发布    ~0.1-0.3ms（≥4KB时）
  cudaIpcOpenMemHandle  ~0.05ms
  cudaIpcCloseMemHandle ~0.05ms
  ─────────────────────────────
  每帧总开销             ~0.5-1.0ms（不含数据传输）
```

---

## 2. 本系统架构改进

### 2.1 总体架构

```
┌──────────────────────────────────────────────────────────────────┐
│                     Sender Node (CPU)                             │
│                                                                    │
│  i==0 (注册帧):                                                    │
│    tensor_info = get_tensor_info(tensor)                          │
│    memory_pool_id = node.register_memory_pool(tensor_info,        │
│                                              "cuda")              │
│      → 生成唯一 pool counter（单调递增，无上限）                     │
│      → 创建/复用 /dev/shm/dora_pool_{counter}                         │
│      → 写入 DORADMA header (256B): magic + metadata + IPC handle  │
│      → cudaHostRegister(src) + cudaMemcpyHtoD → GPU HBM          │
│      → cudaIpcGetMemHandle → 写入 header[32:96]                   │
│      → ipc_present=1（标记 DMA 路径）                               │
│      → PINNED_POOL.insert(counter, ...) 缓存 shmem（防 munmap）     │
│      → Daemon 注册元数据（生命周期追踪，非数据路径）                  │
│      → 返回 buffer_id "pool_{counter}"                      │
│    node.send_output("data", pa.array([memory_pool_id]))           │
│                                                                    │
│  i>=1 (写入帧):                                                    │
│    tensor_info = get_tensor_info(new_tensor)                      │
│    node.write_memory_pool(memory_pool_id, tensor_info)            │
│      → 解析 buffer_id → counter                                      │
│      → PINNED_POOL.remove(&counter) → 缓存命中（无 mmap！）          │
│      → DORADMA magic 验证 → 确认 ipc_present=1                    │
│      → cudaHostRegister(src) → cudaMemcpyHtoD → GPU HBM pool     │
│      → cudaHostUnregister(src)                                    │
│      → PINNED_POOL.insert(counter, ...)（无 munmap！）                │
│      → 无 daemon TCP 往返                                          │
│    node.send_output("data", pa.array([]), metadata)               │
└────────────────────┬─────────────────────────────────────────────┘
                     │ send_output (仅 pool id 或空, <100B)
                     │ 无 Arrow IPC 序列化大数据
┌────────────────────▼─────────────────────────────────────────────┐
│                  Dora Daemon                                       │
│  MemoryPoolManager: HashMap<MemoryPoolId, MemoryPoolEntry>        │
│  - register_memory_pool: 元数据登记（仅首次）                       │
│  - read_memory_pool: 慢路径 fallback（DORADMA 失效时）              │
│  - free_memory_pool: 删除表项，跳过 pool_* shm_unlink              │
│  - cleanup_all: daemon shutdown 时清理残留 shmem                   │
│                                                                    │
│  ★ Daemon 不参与数据路径 — 数据不经过 daemon 内存空间             │
└────────────────────┬─────────────────────────────────────────────┘
                     │
┌────────────────────▼─────────────────────────────────────────────┐
│                   Receiver Node (CUDA)                             │
│  i==0:                                                             │
│    event = node.next()                                             │
│    memory_pool_id = event["value"]                                 │
│    tensor_info = node.read_memory_pool(memory_pool_id)             │
│      → 提取 counter → open /dev/shm/dora_pool_{counter}                  │
│      → DORADMA magic 验证 → 读 JSON metadata                      │
│      → ipc_present=1: 读 header[32:96] → cudaIpcOpenMemHandle     │
│      → GPU DRAM ptr 缓存到 RECV_GPU_VA[counter].gpu_buf              │
│      → 返回 tensor_info {ptr=GPU_DRAM, "device"="cuda", ...}      │
│      → 无 daemon 查询                                              │
│    torch_tensor = tensor_from_info(tensor_info)                    │
│      → __cuda_array_interface__ 零拷贝包装 GPU HBM 指针            │
│                                                                    │
│  i>=1:                                                             │
│    event = node.next()                                             │
│    # torch_tensor 自动反映新数据 — 同一 GPU HBM 指针               │
│    # sender 的 DMA 已覆写 GPU buffer，无需任何接收端操作            │
│                                                                    │
│  Cleanup:                                                          │
│    node.free_memory_pool(memory_pool_id)                           │
│    cudaIpcCloseMemHandle → cudaFree                                │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 DORADMA 共享内存头部格式

```
Offset  Size  Field
0       8     magic: b"DORADMA\x00"            — 快速路径识别
8       8     json_len: u64 LE                  — JSON 元数据长度
16      8     data_offset: u64 LE               — tensor 数据起始偏移
24      8     ipc_present: u64                  — 0=GPU VA, 1=GPU DMA+IPC
32      64    ipc_handle: [u8; 64]             — CUDA IPC opaque handle
96      160   reserved                          — 预留扩展
256     N     JSON: {size, dtype, shape, pinned_type}
256+N   M     tensor data (仅 GPU VA 路径使用；DMA 路径数据在 GPU HBM)
```

### 2.3 数据路径：上游 vs 本系统

| 阶段 | 上游 Dora | 本系统 |
|------|----------|--------|
| **发送端序列化** | Arrow IPC encode → heap buffer | 无（指针直接传递） |
| **daemon 路由** | TCP + mpsc 扇出（每帧） | 仅首帧注册 metadata |
| **数据搬运** | Zenoh SHM（best-effort）或 TCP | cudaMemcpyHtoD DMA 引擎 |
| **接收端反序列化** | Arrow IPC decode → Array | 零拷贝 `__cuda_array_interface__` |
| **GPU 内存管理** | 应用层 cudaIpcOpen/Close 每帧 | 一次 open，缓存复用 |
| **shmem 管理** | Zenoh ShmProvider 自动管理 | PINNED_POOL 持久化缓存 |
| **每帧 daemon 交互** | 有（SendMessage + fanout） | 无（DORADMA fast path） |

---

## 3. 核心创新点（EuroSys 投稿）

### 创新 1：Shared-State 内存池 API — 从消息传递到共享状态

**上游**：每次 `node.send_output()` 触发完整消息序列化→路由→反序列化流水线。

**本系统**：引入 `register_memory_pool` / `write_memory_pool` / `read_memory_pool` / `free_memory_pool` 四元 API，实现共享状态语义：
- Sender 注册一次内存池，获得 buffer_id
- 后续写入直接覆写共享内存，无需序列化/反序列化
- Receiver 在首帧创建 tensor，后续帧自动反映新数据（同一 GPU 指针）

**创新性**：将数据流框架从"消息传递"范式转换为"共享状态"范式，适合机器人/自动驾驶等固定数据流图场景。API 设计保持与上游 `send_output`/`next` 的兼容性——首帧用 `register`，后续用 `write`，接收端仅需在 i==0 调用 `read_memory_pool`。

### 创新 2：DORADMA — 共享内存头部元数据旁路

**上游**：所有元数据（shape、dtype、size）嵌入 Arrow IPC 格式，随数据一起序列化。

**本系统**：在 pool shmem 头部嵌入 256 字节 DORADMA header：
- `write_memory_pool` 无需查询 daemon 即可定位数据偏移——验证 magic 后直接 `ptr::copy_nonoverlapping`
- `read_memory_pool` 从 header 的 JSON 区域直接解析 metadata——无需 daemon TCP 往返
- Daemon 退化为"生命周期追踪"角色，仅记录注册/释放，不参与数据路径

**性能收益**：每帧节省 ~0.1-0.2ms daemon TCP 往返 + ~0.2ms Arrow IPC 序列化。

### 创新 3：GPU DMA 引擎传输 + CUDA IPC 跨进程零拷贝

**上游**：CUDA IPC handle 作为不透明 blob 通过消息通道发送，每帧 `cudaIpcOpenMemHandle` + `cudaIpcCloseMemHandle`。

**本系统**：
1. Sender 端通过 `cudaMemcpyHtoD` 调用 GPU DMA 引擎，将 CPU 页锁数据经 PCIe 总线批量传输至 GPU HBM
2. GPU HBM buffer 由 `cudaMalloc` 预分配，按 pool counter 缓存复用
3. CUDA IPC handle 首次写入 DORADMA header（byte 32-95），Receiver 在首帧导入并缓存 GPU DRAM 指针
4. 后续帧 Receiver 的 tensor 直接指向 GPU HBM——GPU kernel 以 500+ GB/s 的 HBM 带宽访问数据

**与上游的 CUDA IPC 本质区别**：

| | 上游 CUDA IPC | 本系统 GPU DMA + IPC |
|---|---|---|
| IPC handle 传输 | 每帧 Arrow 序列化→消息通道 | 首次写入 shmem header，后续复用 |
| 数据搬运 | GPU kernel 直接写入 | DMA 引擎硬件搬运（不占 CUDA core） |
| GPU 内存管理 | 每帧 open/close | 一次 open，持久缓存 |
| 接收端 tensor 创建 | 每帧 cudaIpcOpenMemHandle + as_tensor | 首帧创建，永久复用 |

### 创新 4：PINNED_POOL 持久化 shmem 缓存

**上游**：Zenoh ShmProvider 管理 shmem 生命周期，每次 publish 都涉及 SHM 段分配/释放。

**本系统**：`PINNED_POOL` HashMap（key=counter）缓存每个 pool 的 Shmem 句柄，采用 `remove()`/`insert()` 所有权转移模式：
- `write_memory_pool` 先 `remove(&counter)` 取出（Rust 所有权转移，防止 Drop → munmap）
- 使用后 `insert(counter, ...)` 存回（保持 mmap 存活）
- 仅当 size 不足以容纳新数据时才执行 `ShmemConf::create()`（fallback mmap）

**性能收益**：消除 per-iteration mmap/munmap 开销（60MB 映射 ~1.5-4ms），是 4000→5500 MB/s 恢复的关键修复。

### 创新 5：双路径共存（ipc_present 标志）

DORADMA header byte 24 的 `ipc_present` 标志控制两种 GPU 读取路径的动态切换：

| ipc_present | 路径 | 数据位置 | GPU 访问方式 |
|-------------|------|---------|-------------|
| 0 | GPU VA | CPU 页锁 shmem | `cudaHostGetDevicePointer` → PCIe BAR Read |
| 1 | GPU DMA | GPU DRAM (HBM) | CUDA IPC → 直接 HBM 指针 |

`register_memory_pool` 根据 `receiver_device` 参数自动选择路径。向后兼容——旧 receiver 忽略 ipc_present，退化为 GPU VA 路径。

### 创新 6：接收端多级缓存体系

| 缓存层 | 存储内容 | 避免的开销 |
|--------|---------|-----------|
| `RECV_GPU_VA` (HashMap) | Shmem + GPU VA + GPU DMA buffer ptr | 每次 OpenShmem + cudaHostRegister + cudaHostGetDevicePointer + cudaIpcOpenMemHandle |
| `RECV_CPU_SHMEM` (HashMap) | Shmem (CPU 接收端) | 每次 OpenShmem + munmap (指针悬空) |
| `_gpu_bufs[counter]` (Python CUDA helper) | GPU HBM buffer ptr + size | 每次 cudaMalloc/cudaFree |
| `CUDA_HELPERS` (LazyLock 静态) | 预编译 CUDA Python 模块 | 每次 PyModule::from_code 编译 |

### 创新 7：Daemon 旁路数据架构

**上游**：Daemon 是数据路径的必经节点——`send_output_to_local_receivers` 是数据扇出的关键函数。

**本系统**：Daemon 完全退出数据路径：
- Sender 通过 `write_memory_pool` 直接写入 shmem，不通知 daemon
- Receiver 通过 `read_memory_pool` 直接从 shmem header 读取元数据
- Daemon 仅维护 `MemoryPoolManager` 注册表，用于异常时 fallback 查询和 shutdown 清理
- `send_output` 传递的 payload 从"完整 tensor 数据"变为"buffer_id 字符串（< 50 B）"

---

## 4. 性能评估

### 4.1 测试配置

| 参数 | 值 |
|------|-----|
| Tensor 尺寸 | 15000 × 512 int64 = 60 MB |
| 迭代次数 | 100 |
| 硬件 | PCIe Gen3/Gen4, NVIDIA GPU |
| CPU→CUDA 场景 | CPU 生成随机 pageable 数据，传输至 GPU 接收端 |

### 4.2 端到端吞吐量对比

| 系统 | 场景 | 吞吐量 (MB/s) | 加速比 |
|------|------|-------------|--------|
| 上游 Dora (Zenoh + 应用层 IPC) | CPU→CUDA | ~1500 | 1.0× |
| 上游 Dora (Zenoh SHM + 应用层 IPC) | CPU→CUDA | ~1800 | 1.2× |
| **本系统 GPU VA 路径** | CPU→CUDA | **~7400** | **4.9×** |
| **本系统 GPU DMA + IPC 路径** | CPU→CUDA | **~5500** | **3.7×** |
| 本系统 GPU DMA (源已 pin) | CPU→CUDA | **~12000+** (预估) | **8.0×** |

### 4.3 耗时分解（本系统 DMA 路径，60MB/帧）

| 阶段 | 耗时 (ms) | 占比 |
|------|-----------|------|
| `cudaHostRegister`（~15,000 4K 页） | ~4.0 | 36% |
| `cudaMemcpyHtoD`（PCIe DMA 引擎） | ~5.0 | 45% |
| `cudaHostUnregister` | ~0.5 | 5% |
| DORADMA header 写入 + magic 验证 | <0.01 | <1% |
| 其他（PINNED_POOL take/store、send_output） | ~1.5 | 13% |
| **总计** | **~11.0** | **100%** |

### 4.4 GPU VA 路径 vs DMA 路径分析

**GPU VA 路径 (7400 MB/s) 更快的原因**：
- 页锁 shmem 一次注册永久复用（cudaHostRegister 开销仅首次，分摊至 100+ 迭代）
- CPU memcpy 纯用户态操作，走 DDR 内存总线（~8-10 GB/s），无需内核页锁定
- GPU 通过 PCIe BAR Read 按需访问，与 CPU memcpy 流水线重叠

**DMA 路径 (5500 MB/s) 的权衡**：
- `cudaHostRegister` 是主要瓶颈——每帧锁定 ~15,000 个 4K 页需 ~4ms 内核态开销
- 但数据最终在 GPU HBM——GPU kernel 以 500+ GB/s 访问，对推理场景更优
- DMA 引擎传输不占 CUDA core/SM，CPU 和 GPU 计算可并行

**DMA 路径的优势场景**：
- 源数据已页锁（传感器 DMA 输出、GPU kernel 输出、预注册 buffer）→ 无需 register，纯 DMA ~12 GB/s
- 接收端需反复随机访问 tensor（推理时多次读取）→ 数据在 HBM 避免了 PCIe BAR Read 延迟

---

## 5. 与相关工作的对比

| 系统 | 传输模型 | GPU 支持 | 零拷贝 | 内存池 | 元数据旁路 |
|------|---------|---------|--------|--------|-----------|
| **上游 Dora** | 消息传递 (TCP+Zenoh) | 应用层 IPC | Zenoh SHM (部分) | Zenoh ShmProvider (8MB) | 无 |
| **ROS 2** | DDS (FastDDS/CycloneDDS) | 无内置 | loaned messages (部分) | 无 | 无 |
| **Iceoryx2** | 发布-订阅 SHM | 无 | 全路径 | 固定段池 | 无 |
| **BaM (ASPLOS'23)** | 细粒度 GPU 缺页 | 原生 (GPU storage) | 系统级 | 无 (按需迁移) | N/A |
| **Poseidon (ATC'17)** | GPU-GPU RDMA | 原生 | GPU Direct RDMA | 无 | N/A |
| **本系统** | Shared-State SHM | 传输层集成 (DMA+IPC+VA) | 全路径 | counter-keyed dynamic pool | DORADMA header |

**本系统的独特位置**：在数据流框架层面实现 GPU 传输的一等公民支持——不是应用层约定（如上游 dora 的 `cuda.py`），而是传输层、内存管理、元数据格式的集成设计。与 BaM/Poseidon 等系统级 GPU 内存方案互补——本系统解决的是框架层的数据传递效率，而非 GPU 内存管理策略。

---

## 6. 文件结构

| 文件 | 内容 |
|------|------|
| `apis/python/node/src/lib.rs` | Python 绑定：register/write/read/free_memory_pool, try_doradma_read, PINNED_POOL, RECV_GPU_VA, RECV_CPU_SHMEM, 预编译 CUDA helper |
| `binaries/daemon/src/memory_manager.rs` | MemoryPoolManager：注册表、pool shmem 生命周期、cleanup_all |
| `binaries/daemon/src/lib.rs` | RegisterPinnedMemory/ReadPinnedMemory/FreePinnedMemory 事件处理 |
| `apis/rust/node/src/node/mod.rs` | DoraNode register/read/free_pinned_memory 方法 |
| `apis/rust/node/src/node/control_channel.rs` | ControlChannel 页锁内存 TCP 通信 |
| `libraries/message/src/node_to_daemon.rs` | DaemonRequest 枚举 |
| `libraries/message/src/daemon_to_node.rs` | DaemonReply 枚举 |
| `apis/python/node/dora/cuda.py` | Python API：get_tensor_info, tensor_from_info, CUDA IPC 工具 |

---

## 7. API 参考

### Python API

```python
# 发送端
tensor_info = dora.cuda.get_tensor_info(torch_tensor)     # tensor → {ptr, size, dtype, shape, device}
pool_id = node.register_memory_pool(tensor_info, "cuda")  # 注册内存池（首帧）
node.write_memory_pool(pool_id, tensor_info)               # 写入数据（后续帧）
node.send_output("data", pool_id, metadata)                # 发送 pool id（< 50B）

# 接收端
event = node.next()
tensor_info = node.read_memory_pool(event["value"])        # DORADMA 读取（无 daemon）
torch_tensor = dora.cuda.tensor_from_info(tensor_info)     # 零拷贝包装 GPU 指针
node.free_memory_pool(pool_id)                             # 释放
```

---

## 8. 关键实现细节

1. **Rust Shmem 生命周期管理**：`PINNED_POOL` 采用 `remove(&counter)`/`insert(counter, ...)` 所有权转移模式——`remove()` 移出 Shmem 所有权防止 Drop→munmap，使用后 `insert()` 存回保持 mmap 存活。这是 4000→5500 MB/s 性能恢复的关键。

2. **CUDA IPC handle ABI**：`cudaIpcOpenMemHandle` 第二个参数是 64 字节结构体**值传递**（栈上），`ctypes` 必须声明为 `_CudaIpcMemHandle(ctypes.Structure)` 而非指针类型——否则仅传递 8 字节指针值，其余 56 字节为栈上垃圾，导致 `CUDA_ERROR_INVALID_VALUE(1)`。

3. **cudaHostRegister 去重**：`_register_host` 幂等处理 CUDA error 712（`HOST_MEMORY_ALREADY_REGISTERED`），防止重复 pin 同一块内存。

4. **Pool shmem 持久化**：`shmem.set_owner(false)` 使 Drop 跳过 `shm_unlink`——shmem 文件由 daemon 在 shutdown 时统一清理。

5. **FREED_POOL_IDS**：跟踪已释放 pool ID 防止 read-after-free 读取过期数据。

6. **`unsafe impl Send + Sync`**：PoolSlot/RecvGpuSlot/RecvCpuSlot 包含 Shmem 裸指针，手动标记 Send+Sync 并由 Mutex 保护。
![img.png](img.png)