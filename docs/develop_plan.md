
# 开发规划：DMA-only 跨进程传输 30000MB/s 技术路径

## 现状诊断

当前 pinned memory 路径实测 ~1500-1800 MB/s，距离目标 30000 MB/s 差距约 20x。

三条传输路径对比：

| 路径 | 发送方操作 | 接收方操作 | 拷贝次数 | 实测速度 |
|------|-----------|-----------|---------|---------|
| torch | tensor→numpy→Arrow→dora shmem | Arrow→torch.tensor(device="cuda") | 2次 | ~2200 MB/s |
| ipc | tensor→GPU(DMA)→cudaIpcGetMem | cudaIpcOpenMem→zero-copy tensor | 1次 | ~18000 MB/s |
| pinned | tensor→/dev/shm(copy)→cudaHostReg | /dev/shm→DMA→GPU+cudaHostUnreg | 2次 | ~1500 MB/s |

### pinned 模式每轮迭代开销分析（61MB tensor）

| 操作 | 耗时 | 可消除? |
|------|------|--------|
| `shm_open` + `ftruncate` + `mmap`（发送端） | ~2-3ms | 是（持久化 buffer pool） |
| `ptr::copy_nonoverlapping` 61MB→/dev/shm | ~3-5ms | 是（数据直达 pinned 内存） |
| `mmap`（接收端） | ~1ms | 是（持久映射） |
| `cudaHostRegister` 61MB | ~3-5ms | 是（预 pin / cudaHostAlloc） |
| `cudaMalloc` 61MB GPU | ~1-3ms | 是（GPU buffer pool） |
| `cudaMemcpy H2D` 61MB（PCIe DMA） | ~3-4ms | 不可消除（这是 DMA 本身） |
| `cudaDeviceSynchronize` | ~0.1ms | 可优化（异步 + 流水线） |
| `cudaHostUnregister` + `cudaFree` | ~2-4ms | 是（pool 复用） |
| **合计 per-iteration** | **~15-25ms** | |

### 理论天花板

2 拷贝路径（CPU→shmem→GPU）的理论带宽上限：

```
effective_bw = 1 / (1/RAM_bw + 1/PCIe_bw)
             = 1 / (1/100GB/s + 1/30GB/s)  [PCIe Gen4 x16]
             ≈ 23 GB/s = 23000 MB/s
```

**结论：即使消除所有 syscall 开销，2 拷贝设计无法突破 23000 MB/s。要达到 30000 MB/s，热路径最多只能有 1 次拷贝。**

## 三阶段技术路径

### 第一阶段：持久化 Buffer Pool（目标 5000-8000 MB/s）

消除所有 per-iteration 系统调用开销。

**方案：预分配环形缓冲区**

```
初始化时（一次性）:
  ring_buffers = [Shmem::create() for _ in 0..N]    // N 个预创建 shmem
  gpu_ring = [cudaMalloc() for _ in 0..N]           // N 个预分配 GPU buffer
  cudaHostRegister(每个 ring buffer)                  // 一次性 pin
  streams = [cudaStreamCreate() for _ in 0..N]       // 独立 CUDA stream per slot

发送端运行时:
  slot = ring.next()
  ptr::copy_nonoverlapping(tensor.data, slot.shmem)  // 拷贝 A（保留，无 syscall 开销）
  register_daemon(slot.id, metadata)                  // 仅注册元数据
  send_output(slot.id)

接收端运行时:
  slot = gpu_ring.next()
  cudaMemcpyAsync(slot.gpu, shmem_ptr, size, H2D, slot.stream)  // 纯 DMA
  // 无需 cudaDeviceSynchronize（使用 slot 前再 sync）
```

**改动文件：**
- `apis/python/node/src/lib.rs`：`register_pinned_memory_internal` 支持预分配模式
- `apis/python/node/dora/cuda.py`：新增 `PinnedBufferPool` 类
- `binaries/daemon/src/memory_manager.rs`：支持持久化注册

**预期提升：** 消除 ~10-15ms syscall 开销后，对 61MB 数据可达 ~5000-8000 MB/s。

---

### 第二阶段：消除 CPU→shmem 拷贝（目标 15000-20000 MB/s）

**方案 A（推荐）：cudaHostAlloc 池 + fd 共享**

使用 `cudaHostAlloc` 让数据出生在 pinned 内存中，无需 `cudaHostRegister`。

```
发送端（初始化）:
  ptr = cudaHostAlloc(size, cudaHostAllocPortable | cudaHostAllocWriteCombined)
  // 此内存既 CPU 可写、又 GPU 可 DMA，且已 pinned
  shmem_fd = export_to_shmem(ptr)  // 通过 /proc/pid/fd 导出文件描述符

发送端（运行时）:
  slot = buffer_pool.acquire()
  torch_tensor.numpy().copy_to(slot.ptr)               // 直接写 pinned 内存
  register_pinned_memory(slot.fd, metadata)             // 零拷贝注册（fd 传递）
  send_output(fd_id)

接收端（运行时）:
  host_ptr = mmap_import(shmem_fd)                      // 映射同一物理页
  cudaMemcpyAsync(dst_gpu, host_ptr, size, H2D, stream)  // 纯 DMA
```

关键：`cudaHostAllocWriteCombined` 对 PCIe 写出带宽提升显著（~40%），但 CPU 读取性能差。发送端只写不读，完美匹配。

**方案 B：DMA 前置 + GPU IPC**

如果发送端有 GPU，将 DMA 放在发送端，接收端通过 GPU IPC 零拷贝获取：

```
发送端:
  cudaMemcpyAsync(gpu_buf, pinned_staging, size, H2D)  // DMA 在此完成（不计入传输窗口）
  cudaIpcGetMemHandle(handle, gpu_buf)                  // 导出 GPU 内存句柄
  send_output(handle)                                   // 发送 64 字节句柄

接收端:
  cudaIpcOpenMemHandle(&d_ptr, handle)                  // 零拷贝映射同块 GPU 内存
  tensor = wrap_gpu_tensor(d_ptr)                       // 零拷贝

跨进程传输本身是 0 拷贝！
```

**改动文件：**
- `apis/python/node/src/lib.rs`：重构 `register_pinned_memory_internal`，支持 fd 传递
- `apis/python/node/dora/cuda.py`：新增 `allocate_pinned_pool`、`cudaHostAlloc` 绑定
- `binaries/daemon/src/memory_manager.rs`：支持注册预分配 pinned 内存

**预期提升：** 热路径仅剩 1 次 DMA 拷贝，PCIe Gen4 下达 ~15000-20000 MB/s。

---

### 第三阶段：多流 DMA 并发聚合（目标 30000+ MB/s）

单条 PCIe DMA 通道带宽有限。通过多 CUDA stream 并发 DMA，聚合 PCIe 带宽。

**方案：多 Stream 并行 DMA + 流水线**

```
接收端初始化:
  num_streams = 4  // 或根据 GPU/PCIe 拓扑调优
  for i in 0..num_streams:
    streams[i] = cudaStreamCreate()
    gpu_buffers[i] = cudaMalloc(slot_size)

接收端运行时（流水线）:
  // DMA N 与 Process N-1 重叠
  cudaMemcpyAsync(gpu_buf[next], shmem[next], size, H2D, stream[next])
  cudaStreamSynchronize(stream[prev])  // 确保上一个 DMA 完成
  process(gpu_buf[prev])               // 使用数据
```

**PCIe 带宽对照：**

| 硬件 | 理论带宽 | 实测 | 达标? |
|------|---------|------|------|
| PCIe Gen4 x16 | ~31.5 GB/s | ~25-28 GB/s | 单流不够，多流可达 |
| PCIe Gen5 x16 | ~63 GB/s | ~51-55 GB/s | 单流即可达标 |
| Ascend 910B HCCS | ~56 GB/s | ~50+ GB/s | 单链路即可达标 |
| NVIDIA H100 NVLink | ~900 GB/s | ~600+ GB/s | 远超 |

**Ascend NPU 适配（昇腾种子计划目标硬件）：**

Ascend 910B 使用 HCCS 互联 + PCIe 5.0，对标 CUDA API：

| CUDA API | Ascend CANN API | 用途 |
|----------|----------------|------|
| `cudaHostAlloc` | `aclrtMallocHost` | 分配 pinned memory |
| `cudaHostRegister` | `aclrtMemRegister` | 注册已有内存为 pinned |
| `cudaMalloc` | `aclrtMalloc` | 分配 device 内存 |
| `cudaMemcpyAsync` | `aclrtMemcpyAsync` | 异步 DMA 传输 |
| `cudaStreamCreate` | `aclrtCreateStream` | 创建异步流 |
| `cudaIpcGetMemHandle` | HCCS 自动共享 | 跨芯片内存访问 |

```
Ascend 下的数据流:
发送端:
  host_ptr = aclrtMallocHost(size, ACL_MEM_MALLOC_HUGE_FIRST)
  tensor.data().copy_to(host_ptr)                        // 直接写入 pinned 区
  register_with_daemon(host_ptr, size, fd)

接收端:
  dev_ptr = aclrtMalloc(size, ACL_MEM_MALLOC_HUGE_FIRST)
  aclrtMemcpyAsync(dev_ptr, host_ptr, size, HOST_TO_DEVICE, stream)  // HCCS DMA
  tensor = wrap_npu_tensor(dev_ptr)
```

**改动文件：**
- `apis/python/node/src/lib.rs`：重构 `pin_and_dma_to_gpu` 为多 stream 异步版本
- `apis/python/node/dora/cuda.py`：扩展 buffer pool 支持 CUDA stream
- 新增 Ascend backend（条件编译或 feature flag）

## 实施优先级

| 优先级 | 阶段 | 改动量 | 风险 | 收益 |
|--------|------|--------|------|------|
| P0 | 第一阶段：Buffer Pool | 中 | 低 | 5-8x |
| P0 | 第二阶段方案A：cudaHostAlloc | 中 | 中 | 10-15x |
| P1 | 第三阶段：多 Stream 并发 | 小 | 低 | 1.3-2x |
| P2 | Ascend CANN 适配 | 大 | 中 | 硬件达标 |
| 备选 | 第二阶段方案B：DMA前置+IPC | 小（已有基础） | 低 | 需发送端有 GPU |

## 关键文件汇总

| 文件 | 当前职责 | 需改动 |
|------|---------|--------|
| `apis/python/node/src/lib.rs` | `register/read_pinned_memory`, `pin_and_dma_to_gpu` | 重构为 buffer pool + async DMA |
| `apis/python/node/dora/cuda.py` | `torch_to_ptr`, `ptr_to_torch`, GPU IPC | 新增 `PinnedBufferPool`, `cudaHostAlloc` 绑定 |
| `binaries/daemon/src/memory_manager.rs` | `pinned_memory_table`, register/read/free | 支持持久化注册、fd 传递 |
| `binaries/daemon/src/lib.rs` | DaemonRequest/DaemonReply 处理 | 扩展消息协议 |
| `apis/rust/node/src/node/mod.rs` | DoraNode 方法 | 新增 pool 管理方法 |
| `apis/rust/node/src/node/control_channel.rs` | 控制通道 | 扩展协议 |
| `libraries/message/src/node_to_daemon.rs` | DaemonRequest 枚举 | 新增变体 |
| `libraries/message/src/daemon_to_node.rs` | DaemonReply 枚举 | 新增变体 |
| `docs/pinned_memory_development.md` | 开发文档 | 同步更新 |

## 验证

1. 每个阶段完成后运行 `cargo build --release --package dora-cli`
2. 运行 test1.yml 对比三种模式速度（torch / ipc / pinned）
3. 确保 test1-test5 全部通过，无内存泄漏
4. 使用 `nvidia-smi dmon -s pucv` 监控 PCIe 带宽利用率
5. 对 61MB tensor 的 pinned 模式目标速度：
   - 第一阶段后：5000-8000 MB/s
   - 第二阶段后：15000-20000 MB/s
   - 第三阶段后：25000-30000+ MB/s
