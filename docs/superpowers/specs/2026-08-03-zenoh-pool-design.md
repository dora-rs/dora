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
     ├─ 未找到/无 coordinator → warn + no-op（node 得到"未创建"结果，不崩溃）
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
     池存在（同步注册保证）→ seqlock：奇数代开始 → memcpy 直写数据区 → 偶数代
     池缺失（安全网）→ 按事件 size/dtype/shape 惰性建池再写
  A 侧：bincode 序列化缓冲复用（预分配 Vec，避免每帧新建）
  B 侧：单次 memcpy，无中间缓冲、无 hex、无代理池
```

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

| 场景 | 行为 |
|---|---|
| coordinator 找不到 machine / 无 coordinator | **本地池照常创建**（sender 本机可用），仅跳过远端创建并 warn，不崩溃 |
| B 不可达 / B 建池失败 | **fail loud**（register 返回错误，不静默），**本地池回滚**（不留孤儿） |
| write 时池缺失（乱序防御） | 按事件元数据惰性建池 |
| read 时池缺失（不应发生） | 现有 3600s 窗口重试 |
| register 后立即 write | 安全（同步注册保证池先存在；惰性建池兜底） |

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

## 8. 后续迭代（不在 v1）

- GPU receiver 池（cpu2cuda_cross：B 侧建 GPU buffer + 数据拷贝）
- cuda2cpu / cuda2cuda 跨机
- A 侧序列化缓冲走 zenoh SHM provider（零拷贝）
- 代理池路径（PROXY_POOL_DATA + hex）的完全移除与清理
