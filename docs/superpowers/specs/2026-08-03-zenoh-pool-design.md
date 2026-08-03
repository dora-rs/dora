# 跨机 Memory Pool 池化复用设计（zenoh 直写路径）

日期：2026-08-03
状态：已批准（用户确认范围与方案）

## 1. 背景与动机

### 现状问题

当前跨机 memory pool 路径（代理池架构）存在 11× 性能差距：

| 环节 | 现状 | 实测 |
|---|---|---|
| 纯 zenoh 传输（61.44MB/帧） | put 414ms | **~148 MB/s** |
| 端到端（含代理池 + hex 往返） | 12.94 MB/s | 瓶颈在 hex 编码（61.44MB → 117MB 字符串） |

现状每帧拷贝链：sender `to_vec()` → daemon `bincode::serialize` → zenoh → B daemon 存 PROXY_POOL_DATA → **hex 编码回复（117MB）** → receiver hex 解码 → from_address。共 ~5 次百 MB 级操作。

### 目标

双端真实 DORADMA 池：`register` 让接收机也建同名 shmem → `write` 本地写 + zenoh 转发原始数据由接收机 daemon **直写**其本地池 → `read` 走原有零拷贝快路径（与单机完全同构）→ 代理路径整个移除。

### 范围（v1）

- **仅支持 cpu2cpu_cross**：发送方 CPU、接收方 CPU，接收侧池固定为 CPU DORADMA shmem
- GPU 发送/接收路径、zenoh SHM 零拷贝：后续迭代（见 §8）

## 2. 架构总览

```
A 机（发送）                          B 机（接收）
┌─────────────┐                     ┌─────────────┐
│ sender node │                     │ receiver node│
│  本地池 shmem│◄──零拷贝读───────────│  本地池 shmem │
└──────┬──────┘                     └──────▲──────┘
       │ 注册/写/释放事件                  │ B daemon 直写
       ▼                                 │（seqlock 协议）
┌─────────────┐     zenoh topic     ┌─────┴─────┐
│ daemon A    │◄────────────────────►│ daemon B  │
│ CROSS_POOLS │     全量数据          │ CROSS_POOLS│
└──────┬──────┘                     └───────────┘
       │ ResolveMachine
       ▼
┌─────────────┐
│ coordinator │（daemon 注册表：machine_id → daemon）
└─────────────┘
```

## 3. 组件与接口

### 3.1 node API：`register_memory_pool`

新增默认参数 `machine: str | None = None`：

- `machine=None`（默认）：现有本地路径，零改动
- `machine="B"`：跨机路径，machine_id 字符串格式（与 YAML `_unstable_deploy: machine` 和 daemon `--machine-id` 一致）

### 3.2 coordinator：`ResolveMachine` 请求

- 新请求类型：daemon → coordinator `ResolveMachine { machine_id }`
- coordinator 在 daemon 注册表中查询，返回 `{ found, daemon_id }`
- 失败语义：未找到 / 无 coordinator → **仅 warn，无实际操作，程序不崩溃**

### 3.3 daemon：跨机池注册表

双端各维护 `CROSS_POOLS: HashMap<pool_id, CrossPoolInfo>`：

```rust
struct CrossPoolInfo {
    peer_machine_id: String,   // 对端 daemon（write/free 追踪用）
    size: usize,
    // v1: 固定 CPU DORADMA
}
```

### 3.4 新事件（zenoh memory-pool topic，沿用 dataflow 作用域）

- `RegisterPool { dataflow_id, machine_id, pool_id, size, dtype, shape, device }`
- `FreePool { dataflow_id, machine_id, pool_id }`
- 写入沿用现有 `MemoryPoolWrite`（其字段已含 size/dtype/shape）

## 4. 数据流

### 4.1 注册（同步确认）

```
node.register_memory_pool(tensor_info, "cpu", machine="B")
  → daemon A（请求带 machine="B"）
  → coordinator.ResolveMachine{"B"}
     ├─ 未找到/无 coordinator → warn + 整个 register 无操作（本地也不建池），
     │    register 返回 None，调用方检查处理，不崩溃
     └─ 找到 →
         daemon A 发布 RegisterPool 事件（zenoh，带目标 machine_id + 池元数据）
         → daemon B（machine_id 匹配者执行）→ 按元数据创建 CPU DORADMA 池
           （shmem：header[magic+json+data_offset+seqlock] + 数据区；偶数代）
         → B 回执 → daemon A → node 的 register 返回（同步确认）
  双端记录 CROSS_POOLS：A 记 {pool_id → "B"}，B 记 {pool_id → "A"}
```

同步确认的理由：B 侧池的存在先于任何 write/read，竞态从根上消除（不依赖读路径的缺失重试行为）。

### 4.2 写入（全量直写）

```
write_memory_pool:
  本地快路径写（不变）
  + 全量数据 → zenoh MemoryPoolWrite → B daemon：
     池存在（同步注册保证，write 前池必已存在）→ seqlock：奇数代开始
     → memcpy 直写数据区 → 偶数代
     池缺失（不应发生；防御性）→ warn + 丢弃该帧，不建池（避免泄漏——
     惰性建出的池不在 free 跟踪里，无人释放）
  A 侧：bincode 序列化缓冲复用（预分配 Vec，避免每帧新建）
  B 侧：单次 memcpy，无中间缓冲、无 hex、无代理池
```

**不设惰性建池**：同步注册已保证池先于任何 write 存在；write 建池是多余的旁路，
且 write 建的池游离于 free 跟踪之外（free 事件只引用已注册池），构成内存泄漏源。

### 4.3 读取（零改动）

```
read_memory_pool: 纯本地快路径（try_doradma_read），与单机完全同构
锁分析：
  - B daemon 写（奇数代）vs receiver 读（偶数代才读）：现有 seqlock 协议协调
  - 无新增锁；PROXY_POOL_DATA 整个移除（读路径不再触碰任何跨机锁）
  - 数据未到（偶数代零填充）：receiver 帧校验重试（现有 tensor[0]==i 循环）
```

### 4.4 释放（异步，双端跟踪）

```
node.free_memory_pool → daemon A：释放本地池 + 清 CROSS_POOLS 记录
  → 转发 FreePool 事件 → daemon B：释放 B 池 + 清记录
  → 双端进程缓存清理（FREED_POOL_IDS / PINNED_POOL / RECV_CPU_SHMEM）
异步（free 发生在数据流尾声，安全性由调用方保证）
```

## 5. 错误处理

| 场景 | 行为 | 警告内容 |
|---|---|---|
| coordinator 找不到 machine / 无 coordinator | 仅 warn，**不创建任何池**（本地也不建），register 返回 `None`，调用方检查处理，不崩溃 | `machine "B" 无法解析：coordinator 无此机器或无 coordinator，未创建跨机内存池` |
| B 不可达 / B 建池失败 | 仅 warn，**不创建任何池**（若本地池已创建则回滚，最终无池存在），register 返回 `None`，不崩溃 | `machine "B" 已解析但远端建池失败：<原因>，未创建跨机内存池` |

两种失败的**行为完全一致**（警告 + 无池 + 不崩溃），仅**警告内容不同**——前者指"解析不到"，后者指"解析到了但创建失败"，便于诊断区分。
| write 时池缺失（不应发生——同步注册保证） | warn + 丢弃该帧，不建池（防泄漏） |
| read 时池缺失（不应发生） | 现有 3600s 窗口重试 |
| register 后立即 write | 安全（同步注册保证池先存在） |

## 6. 测试

- **本地双 daemon 复现台**（本会话已就绪：venv PATH + working_dir + --store memory）
- 正路径：cpu2cpu_cross 全流程（register → write×3 → read×3 → free），preview 匹配 + 帧校验通过
- 负路径：
  - `machine="不存在的机器"` → warn + 不崩 + register 返回未创建
  - 无 coordinator 场景 → warn + 不崩
  - register 后立即 write（时序竞态）→ 数据正确
- 性能：端到端吞吐对比现状（预期从 12.94 MB/s 提升至接近纯 zenoh ~148 MB/s）

## 7. 涉及文件（预估）

- `apis/python/node/src/lib.rs`：register_memory_pool 加 machine 参数；跨机注册路径
- `libraries/message/src/*`：新事件类型（RegisterPool/FreePool）+ ResolveMachine 请求
- `binaries/daemon/src/lib.rs`：CROSS_POOLS 注册表、RegisterPool/FreePool 处理、B 侧建池/直写
- `binaries/coordinator/src/`：ResolveMachine 处理
- `examples/memory-pool/sender.py`：register 传 machine 参数（跨机 YAML 场景）

## 实现状态（2026-08-03）

v1（cpu2cpu_cross）已实现并本地验证：
- 双端真实 DORADMA 池 + machine 参数 + coordinator 解析 + 同步确认
- write 全量直写（seqlock）+ read 零拷贝快路径 + free 双端
- E2E 通过：preview 匹配、3 帧、负路径 warn+优雅退出
- 实现中发现的补充设计（E2E 暴露）：镜像 shmem 命名 machine 限定
  （dora_pool_{machine}_{df}_{node}_{counter}，DORA_MACHINE_ID env 注入）；
  memory-pool 发布 Locality::Remote（回声切断）
- 遗留：release 构建的 WAN 端到端吞吐未实测（本地为 debug 构建）
- 已知设计边界：镜像写入未串行化（multi-thread runtime 下并发帧可能
  字节级撕裂，被 zenoh put 延迟与示例 20s pacing 掩盖）；多写者场景
  需 per-pool 写互斥（后续迭代）

## 8. 后续迭代（不在 v1）

- GPU receiver 池（cpu2cuda_cross：B 侧建 GPU buffer + 数据拷贝）
- cuda2cpu / cuda2cuda 跨机
- A 侧序列化缓冲走 zenoh SHM provider（零拷贝）
- 代理池路径（PROXY_POOL_DATA + hex）的完全移除与清理
