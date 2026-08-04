# 跨机 Memory Pool zenoh 直写路径 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现双端真实 DORADMA 池的跨机路径：register 加 `machine` 参数经 coordinator 解析后同步建池于两端，write 全量直写接收端池，read 保持零拷贝快路径，free 双端释放（v1 仅 cpu2cpu_cross）。

**Architecture:** 发送节点 register 时本地建池 + 请求 daemon A 跨机注册；daemon A 经 coordinator 解析目标 machine 后发布 `RegisterPool` 事件（zenoh memory-pool topic），daemon B 收到后按元数据镜像建池并回执；write 事件由 B daemon 按 seqlock 协议 memcpy 直写 B 池数据区；read 走原有零拷贝快路径；free 经 `FreePool` 事件双端清理。同步确认的 ack 等待必须在 spawned task 中执行（ack 事件经同一事件循环送达，阻塞循环会死锁）。

**Tech Stack:** Rust (tokio/flume/zenoh/bincode/shm-rs)，Python (pyo3)，本会话已就绪的本地双 daemon 测试台。

**参考:** 设计文档 `docs/superpowers/specs/2026-08-03-zenoh-pool-design.md`

---

### Task 1: 消息类型扩展（message crate）

**Files:**
- Modify: `libraries/message/src/node_to_daemon.rs`
- Modify: `libraries/message/src/daemon_to_daemon.rs`
- Modify: `libraries/message/src/daemon_to_coordinator.rs`
- Modify: `libraries/message/src/coordinator_to_daemon.rs`

- [ ] **Step 1: 在 `node_to_daemon.rs` 的 `DaemonRequest` 枚举加跨机注册请求**

在 `WritePinnedMemory` 变体之后（约 42 行附近）加：

```rust
    /// Cross-machine memory pool registration: the daemon resolves the
    /// target machine via the coordinator and mirrors the pool there.
    RegisterCrossMachinePool {
        shared_memory_id: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
        machine_id: String,
    },
```

并把它加入 `expects_tcp_bincode_reply()` 的匹配（68 行附近的 `| DaemonRequest::WritePinnedMemory { .. } => true,` 前加 `| DaemonRequest::RegisterCrossMachinePool { .. }`）。

- [ ] **Step 2: 在 `daemon_to_node.rs` 加回复变体**

在 `DaemonReply` 枚举加：

```rust
    /// Result of a cross-machine pool registration. `Err` carries the
    /// warning message (resolution failure or remote creation failure) —
    /// the register is a warn-and-no-op in both cases.
    CrossMachinePoolRegistered(Result<(), String>),
```

- [ ] **Step 3: 在 `daemon_to_daemon.rs` 的 `InterDaemonEvent` 加三个事件**

```rust
    /// Cross-machine pool registration — the matching machine's daemon
    /// mirrors the pool locally and replies with `RegisterPoolAck`.
    RegisterPool {
        dataflow_id: DataflowId,
        machine_id: String,
        shared_memory_id: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
    },
    /// Acknowledge a cross-machine pool registration (sync register).
    RegisterPoolAck {
        dataflow_id: DataflowId,
        shared_memory_id: String,
        ok: bool,
        error: Option<String>,
    },
    /// Release a cross-machine pool on the remote machine.
    FreePool {
        dataflow_id: DataflowId,
        shared_memory_id: String,
    },
```

- [ ] **Step 4: 在 `daemon_to_coordinator.rs` 加解析请求**

```rust
    /// Resolve a machine id to a registered daemon (cross-machine pools).
    ResolveMachine { machine_id: String },
```

- [ ] **Step 5: 在 `coordinator_to_daemon.rs` 加解析回复**

在 `RegisterResult` 附近加（无 reply 字段——回复经 daemon WS 请求-应答通道返回）：

```rust
    /// Reply to `CoordinatorRequest::ResolveMachine`.
    ResolveMachineResult { found: bool },
```

- [ ] **Step 6: 编译检查 + 提交**

```bash
cargo check -j 2 -p dora-message
git add libraries/message/src/
git commit -m "feat(message): cross-machine pool register/ack/free events + ResolveMachine"
```

---

### Task 2: coordinator 处理 ResolveMachine

**Files:**
- Modify: `binaries/coordinator/src/lib.rs`（coordinator 请求处理处）

- [ ] **Step 1: 找到 coordinator 的 daemon 请求处理**

`grep -n "CoordinatorRequest::Register\|CoordinatorRequest::Event" binaries/coordinator/src/lib.rs` —— 在 `Register` 分支附近加：

```rust
            CoordinatorRequest::ResolveMachine { machine_id } => {
                let found = self
                    .store
                    .get_daemon_by_machine(&machine_id)
                    .map(|d| d.is_some())
                    .unwrap_or(false);
                // 回复经现有 WS 请求-应答通道返回（与 Register 的回复方式一致）
                reply_sender.send(CoordinatorToDaemon::ResolveMachineResult { found });
            }
```

（回复发送方式以 Register 分支的现有实现为准——保持同一通道/同一模式。）

- [ ] **Step 2: 编译 + 提交**

```bash
cargo check -j 2 -p dora-coordinator
git add binaries/coordinator/src/lib.rs
git commit -m "feat(coordinator): resolve machine id to registered daemon"
```

---

### Task 3: daemon A 侧——跨机注册请求处理（含同步 ack 等待）

**Files:**
- Modify: `binaries/daemon/src/lib.rs`
- Modify: `binaries/daemon/src/node_communication/mod.rs`

- [ ] **Step 1: 加跨机池注册表与待确认表（daemon 结构体附近）**

```rust
/// Cross-machine pools this daemon participates in:
/// pool id -> peer machine id (write/free tracking).
static CROSS_POOLS: std::sync::LazyLock<
    std::sync::Mutex<std::collections::HashMap<String, String>>,
> = std::sync::LazyLock::new(|| std::sync::Mutex::new(std::collections::HashMap::new()));

/// Pending synchronous register confirmations: pool id -> reply channel.
/// The ack arrives via the event loop's own dispatch, so the waiting
/// task MUST be spawned (not awaited on the loop) or it deadlocks.
static CROSS_REGISTER_PENDING: std::sync::LazyLock<
    std::sync::Mutex<
        std::collections::HashMap<String, tokio::sync::oneshot::Sender<bool>>,
    >,
> = std::sync::LazyLock::new(|| std::sync::Mutex::new(std::collections::HashMap::new()));
```

- [ ] **Step 2: 在 `node_communication/mod.rs` 的请求匹配加新分支**

在 `DaemonRequest::WritePinnedMemory` 分支附近加（沿用 `process_daemon_event` + `reply` 的现有模式，把请求转成 `DaemonNodeEvent`）：

```rust
            DaemonRequest::RegisterCrossMachinePool {
                shared_memory_id,
                size,
                dtype,
                shape,
                device,
                machine_id,
            } => {
                let (reply_sender, reply) = oneshot::channel();
                self.process_daemon_event(
                    DaemonNodeEvent::RegisterCrossMachinePool {
                        shared_memory_id,
                        size,
                        dtype,
                        shape,
                        device,
                        machine_id,
                        reply_sender,
                    },
                    Some(reply),
                    connection,
                )
                .await?;
            }
```

- [ ] **Step 3: 在 `event_types.rs` 的 `DaemonNodeEvent` 加变体**

```rust
    RegisterCrossMachinePool {
        shared_memory_id: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
        machine_id: String,
        reply_sender: oneshot::Sender<DaemonReply>,
    },
```

- [ ] **Step 4: daemon lib.rs 处理 `RegisterCrossMachinePool`（spawn 等待任务，不阻塞事件循环）**

在 `WriteMemoryPool` 处理器附近加：

```rust
            DaemonNodeEvent::RegisterCrossMachinePool {
                shared_memory_id,
                size,
                dtype,
                shape,
                device,
                machine_id,
                reply_sender,
            } => {
                let dataflow_id = dataflow_id;
                let session = self.zenoh_session.clone();
                let clock = self.clock.clone();
                // 整个解析+发布+等待 ack 在 spawn 任务中：ack 经事件循环
                // 送达，若在循环上等待会死锁。
                tokio::spawn(async move {
                    // 1. coordinator 解析
                    let resolved = resolve_machine(&machine_id).await;
                    if !resolved {
                        tracing::warn!(
                            "machine \"{machine_id}\" 无法解析：coordinator 无此机器或无 coordinator，未创建跨机内存池"
                        );
                        let _ = reply_sender.send(DaemonReply::CrossMachinePoolRegistered(Err(
                            format!("machine \"{machine_id}\" 无法解析"),
                        )));
                        return;
                    }
                    // 2. 发布 RegisterPool 并等待 ack（超时 5s）
                    let topic = dataflow_memory_pool_topic(&dataflow_id);
                    let (ack_tx, ack_rx) = tokio::sync::oneshot::channel();
                    CROSS_REGISTER_PENDING
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .insert(shared_memory_id.clone(), ack_tx);
                    let event = InterDaemonEvent::RegisterPool {
                        dataflow_id,
                        machine_id,
                        shared_memory_id: shared_memory_id.clone(),
                        size,
                        dtype,
                        shape,
                        device,
                    };
                    let payload = match bincode::serialize(&Timestamped {
                        inner: event,
                        timestamp: clock.new_timestamp(),
                    }) {
                        Ok(p) => p,
                        Err(e) => {
                            tracing::error!("memory pool register serialize failed: {e}");
                            let _ = reply_sender.send(
                                DaemonReply::CrossMachinePoolRegistered(Err(e.to_string())),
                            );
                            return;
                        }
                    };
                    let declare = match session
                        .declare_publisher(&topic)
                        .congestion_control(CongestionControl::Block)
                        .await
                    {
                        Ok(p) => p,
                        Err(e) => {
                            tracing::warn!(
                                "machine \"{machine_id}\" 已解析但远端建池失败：{e}，未创建跨机内存池"
                            );
                            let _ = reply_sender.send(
                                DaemonReply::CrossMachinePoolRegistered(Err(e.to_string())),
                            );
                            return;
                        }
                    };
                    if let Err(e) = declare.put(payload).await {
                        tracing::warn!(
                            "machine \"{machine_id}\" 已解析但远端建池失败：{e}，未创建跨机内存池"
                        );
                        let _ = reply_sender.send(
                            DaemonReply::CrossMachinePoolRegistered(Err(e.to_string())),
                        );
                        return;
                    }
                    match tokio::time::timeout(
                        std::time::Duration::from_secs(5),
                        ack_rx,
                    )
                    .await
                    {
                        Ok(Ok(true)) => {
                            CROSS_POOLS
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .insert(shared_memory_id.clone(), machine_id.clone());
                            let _ = reply_sender.send(
                                DaemonReply::CrossMachinePoolRegistered(Ok(())),
                            );
                        }
                        Ok(Ok(false)) => {
                            tracing::warn!(
                                "machine \"{machine_id}\" 已解析但远端建池失败，未创建跨机内存池"
                            );
                            let _ = reply_sender.send(
                                DaemonReply::CrossMachinePoolRegistered(Err(
                                    "remote pool creation failed".into(),
                                )),
                            );
                        }
                        Ok(Err(_)) | Err(_) => {
                            tracing::warn!(
                                "machine \"{machine_id}\" 已解析但远端建池失败（超时/通道断开），未创建跨机内存池"
                            );
                            let _ = reply_sender.send(
                                DaemonReply::CrossMachinePoolRegistered(Err(
                                    "remote pool creation timeout".into(),
                                )),
                            );
                        }
                    }
                });
                Ok(())
            }
```

- [ ] **Step 5: 加 daemon→coordinator 请求-应答机制（coordinator.rs）**

daemon 运行时目前只有单向事件（`send_event`）——需要 pending-reply 机制（注册请求的回复是连接建立时内联处理的，不适用于运行时）：

```rust
/// Pending daemon→coordinator request replies: request id -> reply value.
static COORDINATOR_PENDING: std::sync::LazyLock<
    std::sync::Mutex<std::collections::HashMap<Uuid, tokio::sync::oneshot::Sender<serde_json::Value>>>,
> = std::sync::LazyLock::new(|| std::sync::Mutex::new(std::collections::HashMap::new()));

/// Resolve a machine id through the coordinator. Returns false when the
/// machine is unknown or no coordinator is reachable (warn-and-skip).
async fn resolve_machine(
    coordinator_sender: &CoordinatorSender,
    clock: &Arc<uhlc::HLC>,
    machine_id: &str,
) -> bool {
    let request_id = Uuid::new_v4();
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    COORDINATOR_PENDING
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .insert(request_id, reply_tx);
    let params = match serde_json::to_string(&Timestamped {
        inner: CoordinatorRequest::ResolveMachine {
            machine_id: machine_id.to_string(),
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(p) => p,
        Err(_) => return false,
    };
    let json = format!(
        r#"{{"id":"{request_id}","method":"daemon_event","params":{params}}}"#
    );
    if coordinator_sender.send_event(json.as_bytes()).await.is_err() {
        return false;
    }
    match tokio::time::timeout(std::time::Duration::from_secs(5), reply_rx).await {
        Ok(Ok(value)) => value
            .get("found")
            .and_then(|v| v.as_bool())
            .unwrap_or(false),
        _ => false,
    }
}
```

- [ ] **Step 5b: WS 收包循环识别回复（在 `CoordinatorCommandRaw` 解析之前）**

在 coordinator.rs 的收包循环（`let raw: CoordinatorCommandRaw = ...` 之前）插入：

```rust
                    // Replies to our own requests (e.g. ResolveMachine)
                    // arrive as WsResponse { id, result, error } — no
                    // "method" field. Resolve them before command parsing.
                    if let Ok(raw) = serde_json::from_str::<WsResponseRaw>(&text) {
                        if let Some(tx) = COORDINATOR_PENDING
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .remove(&raw.id)
                        {
                            let _ = tx.send(raw.result);
                            continue;
                        }
                    }
```

（`WsResponseRaw`：局部反序列化结构 `{ id: Uuid, result: serde_json::Value, error: Option<Value> }`；coordinator 对 daemon_event 的回复就是 `WsResponse::ok(id, Timestamped<ResolveMachineResult>)` 的 JSON——与 Register 回复同一格式。）

- [ ] **Step 6: 在 `handle_inter_daemon_event` 加 `RegisterPoolAck` 处理**

在 `InterDaemonEvent::MemoryPoolWrite` 分支附近加：

```rust
            InterDaemonEvent::RegisterPoolAck {
                shared_memory_id,
                ok,
                ..
            } => {
                if let Some(tx) = CROSS_REGISTER_PENDING
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&shared_memory_id)
                {
                    let _ = tx.send(ok);
                }
                Ok(())
            }
```

- [ ] **Step 7: 编译 + 提交**

```bash
cargo check -j 2 -p dora-daemon
git add binaries/daemon/src/
git commit -m "feat(daemon): cross-machine register with sync ack (spawned wait)"
```

---

### Task 4: daemon B 侧——RegisterPool 建池 / FreePool 释放 / MemoryPoolWrite 直写

**Files:**
- Modify: `binaries/daemon/src/lib.rs`

- [ ] **Step 1: 加 DORADMA 建池辅助（镜像节点侧 register 的 shmem 创建）**

```rust
/// Create a CPU DORADMA pool mirror on this machine. Mirrors the node
/// API's register_memory_pool shmem layout: header[magic+json_len+
/// data_offset+seqlock] + data region, even generation.
fn create_cross_pool_shmem(
    dataflow_id: &Uuid,
    shared_memory_id: &str,
    size: usize,
    dtype: &str,
    shape: &[i64],
) -> eyre::Result<()> {
    // parse "pool_{node_id}_{counter}" like the node API does
    let (node_id, counter) = shared_memory_id
        .strip_prefix("pool_")
        .and_then(|s| s.rsplit_once('_'))
        .ok_or_else(|| eyre::eyre!("invalid pool id: {shared_memory_id}"))?;
    let shmem_name = format!("dora_pool_{}_{}_{}", dataflow_id, node_id, counter);
    let json = format!(
        "{{\"size\":{size},\"dtype\":\"{dtype}\",\"shape\":{:?},\"pinned_type\":\"cpu\"}}",
        shape
    );
    let data_offset = DORADMA_HEADER_SIZE + json.len();
    let conf = ShmemConf::new().os_id(&shmem_name).size(size + data_offset);
    let shmem = conf.create().map_err(|e| eyre::eyre!("create shmem: {e}"))?;
    unsafe {
        let ptr = shmem.as_ptr();
        std::ptr::copy_nonoverlapping(DORADMA_MAGIC.as_ptr(), ptr, 8);
        write_header_u64(ptr.add(8), json.len() as u64);
        write_header_u64(ptr.add(16), data_offset as u64);
        std::ptr::copy_nonoverlapping(json.as_ptr(), ptr.add(DORADMA_HEADER_SIZE), json.len());
        // seqlock gen starts even
        write_header_u64(ptr.add(96), 0);
    }
    Ok(())
}
```

（`DORADMA_MAGIC`/`DORADMA_HEADER_SIZE`/`read_header_u64`/`write_header_u64` 常量与辅助：从节点 API 的布局复制到 daemon，保证两端布局一致；`ShmemConf` 来自 shm-rs crate，daemon 已有依赖。）

- [ ] **Step 2: 在 `handle_inter_daemon_event` 加 `RegisterPool` 处理（建池 + 回执）**

```rust
            InterDaemonEvent::RegisterPool {
                dataflow_id,
                shared_memory_id,
                size,
                dtype,
                shape,
                device,
                ..
            } => {
                let result = create_cross_pool_shmem(
                    &dataflow_id, &shared_memory_id, size, &dtype, &shape,
                );
                match result {
                    Ok(()) => {
                        CROSS_POOLS
                            .lock()
                            .unwrap_or_else(|e| e.into_inner())
                            .insert(shared_memory_id.clone(), String::new());
                        tracing::info!(
                            "memory pool: mirrored cross-machine pool {shared_memory_id} (size {size})"
                        );
                        publish_memory_pool_event(
                            &self.zenoh_session,
                            &dataflow_id,
                            &InterDaemonEvent::RegisterPoolAck {
                                dataflow_id,
                                shared_memory_id,
                                ok: true,
                                error: None,
                            },
                            &self.clock,
                        )
                        .await;
                    }
                    Err(e) => {
                        tracing::warn!(
                            "memory pool: failed to mirror pool {shared_memory_id}: {e}"
                        );
                        publish_memory_pool_event(
                            &self.zenoh_session,
                            &dataflow_id,
                            &InterDaemonEvent::RegisterPoolAck {
                                dataflow_id,
                                shared_memory_id,
                                ok: false,
                                error: Some(e.to_string()),
                            },
                            &self.clock,
                        )
                        .await;
                    }
                }
                Ok(())
            }
```

（`publish_memory_pool_event`：把 4159-4230 的发布逻辑（serialize + spawn + declare + put）提取成可复用辅助，供 RegisterPoolAck/FreePool/MemoryPoolWrite 共用。**缓冲复用**：序列化输出写入一个 daemon 级共享的可复用缓冲——`static SERIALIZE_BUF: LazyLock<Mutex<Vec<u8>>>`，每次 `buf.clear(); bincode::serialize_into(&mut *buf, ...)` 后 `buf.clone()` 给 spawn 任务（clone 是必需的——任务间共享；省的是每次 61.44MB 的新分配与增长，而非拷贝本身）。）

- [ ] **Step 3: 改 `MemoryPoolWrite` 处理：CROSS_POOLS 命中 → seqlock 直写；否则保持旧代理插入**

在现有 `MemoryPoolWrite` 分支（2958 附近）顶部加：

```rust
            InterDaemonEvent::MemoryPoolWrite {
                dataflow_id,
                shared_memory_id,
                tensor_data,
                size,
                device,
                dtype,
                shape,
                ..
            } => {
                // New cross-machine path: pool mirrored here — write the
                // data straight into the DORADMA data region under the
                // seqlock protocol (receiver reads its local pool
                // zero-copy). Missing pool = should-not-happen defensive
                // case: warn + drop (no creation, avoids leaks).
                let is_cross = CROSS_POOLS
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .contains_key(&shared_memory_id);
                if is_cross {
                    write_cross_pool_data(
                        &dataflow_id, &shared_memory_id, &tensor_data, size,
                    )
                    .await;
                    return Ok(());
                }
                // 代理路径已整体移除（2026-08-04，spec §8 完成）：
                // 非 CROSS_POOLS 的写帧在 daemon 侧 debug 级 drop，
                // 不再有 PROXY_POOL_DATA 缓存
```

并加直写辅助（seqlock 协议复制节点 API 的实现）：

```rust
/// Write tensor bytes into a mirrored cross-machine pool under the
/// DORADMA seqlock protocol (odd gen during write, even after).
async fn write_cross_pool_data(
    dataflow_id: &Uuid,
    shared_memory_id: &str,
    tensor_data: &[u8],
    size: usize,
) {
    let (node_id, counter) = match shared_memory_id
        .strip_prefix("pool_")
        .and_then(|s| s.rsplit_once('_'))
    {
        Some(v) => v,
        None => {
            tracing::warn!("memory pool: invalid pool id {shared_memory_id}, dropping frame");
            return;
        }
    };
    let shmem_name = format!("dora_pool_{}_{}_{}", dataflow_id, node_id, counter);
    let Ok(shmem) = ShmemConf::new().os_id(&shmem_name).open() else {
        tracing::warn!(
            "memory pool: pool {shared_memory_id} missing at write (sync register should have prevented this), dropping frame"
        );
        return;
    };
    let shmem_ptr = shmem.as_ptr();
    // seqlock: odd gen marks an in-progress write
    unsafe {
        let gen_ptr = shmem_ptr.add(96) as *mut u64;
        let pre = seqlock_begin_if_even(gen_ptr);
        let data_offset = read_header_u64(shmem_ptr.add(16)) as usize;
        std::ptr::copy_nonoverlapping(
            tensor_data.as_ptr(),
            shmem_ptr.add(data_offset),
            tensor_data.len().min(size),
        );
        seqlock_end(gen_ptr, pre, true);
    }
}
```

（daemon 内复制 `seqlock_begin_if_even`/`seqlock_end` 与节点 API 同构的 unsafe 实现——两端读写协议必须一致。）

- [ ] **Step 4: 加 `FreePool` 处理**

```rust
            InterDaemonEvent::FreePool {
                shared_memory_id,
                ..
            } => {
                CROSS_POOLS
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .remove(&shared_memory_id);
                // unlink the mirrored shmem (best-effort)
                if let Some((node_id, counter)) = shared_memory_id
                    .strip_prefix("pool_")
                    .and_then(|s| s.rsplit_once('_'))
                {
                    let shmem_name = format!(
                        "dora_pool_{}_{}_{}", dataflow_id, node_id, counter
                    );
                    let _ = ShmemConf::new().os_id(&shmem_name).remove_shmem();
                }
                tracing::info!("memory pool: freed cross-machine pool {shared_memory_id}");
                Ok(())
            }
```

- [ ] **Step 5: 编译 + 提交**

```bash
cargo check -j 2 -p dora-daemon
git add binaries/daemon/src/
git commit -m "feat(daemon): mirror cross-machine pools, direct seqlock writes, dual-end free"
```

---

### Task 5: python API——register_memory_pool 加 machine 参数

**Files:**
- Modify: `apis/python/node/src/lib.rs`

- [ ] **Step 0: rust 节点 API 加 `register_cross_machine_pool` 方法**

`apis/rust/node/src/node/control_channel.rs` 的 `write_pinned_memory` 附近加：

```rust
    /// Register a pool on a remote machine via the daemon (the daemon
    /// resolves the machine through the coordinator and mirrors the
    /// pool there with a synchronous confirmation).
    pub fn register_cross_machine_pool(
        &mut self,
        shared_memory_id: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
        machine_id: String,
    ) -> eyre::Result<Result<(), String>> {
        let request = DaemonRequest::RegisterCrossMachinePool {
            shared_memory_id,
            size,
            dtype,
            shape,
            device,
            machine_id,
        };
        let reply = self
            .channel
            .request(&Timestamped {
                inner: request,
                timestamp: self.clock.new_timestamp(),
            })
            .wrap_err("failed to send RegisterCrossMachinePool request to dora-daemon")?;
        match reply {
            DaemonReply::CrossMachinePoolRegistered(result) => Ok(result),
            other => bail!("unexpected RegisterCrossMachinePool reply: {other:?}"),
        }
    }
```

`apis/rust/node/src/node/mod.rs` 的 `write_pinned_memory` 附近暴露同名公开方法（转发到 `control_channel`）。

- [ ] **Step 1: 改签名（machine 默认 None）**

```rust
    #[pyo3(signature = (tensor_info, device, machine = None))]
    pub fn register_memory_pool(
        &self,
        tensor_info: &Bound<'_, PyDict>,
        device: String,
        machine: Option<String>,
        py: Python,
    ) -> eyre::Result<Py<PyAny>> {
```

- [ ] **Step 2: 本地建池完成后，machine 指定时走跨机注册（现有"代理推送"之前插入）**

在本地 shmem 创建与 header 初始化完成之后、`receiver_is_cuda` 推送之前插入：

```rust
        // Cross-machine: mirror the pool on the target machine via the
        // daemon (coordinator resolves the machine; sync confirm).
        if let Some(target_machine) = machine {
            let reply = self
                .node
                .get_mut()
                .register_cross_machine_pool(
                    buffer_id.clone(),
                    size,
                    dtype.clone(),
                    shape_list.clone(),
                    tensor_device.clone(),
                    target_machine.clone(),
                )
                .map_err(|e| eyre::eyre!("register cross-machine pool: {e}"))?;
            match reply {
                Ok(()) => {
                    // local pool stays; daemon A recorded CROSS_POOLS
                }
                Err(msg) => {
                    // warn-and-no-op: roll back the local pool, return None
                    tracing::warn!("{msg}");
                    self.free_local_pool_resources(&buffer_id);
                    return Ok(py.None());
                }
            }
        }
```

（`register_cross_machine_pool`：rust 节点 API 新增方法——经 control channel 发 `DaemonRequest::RegisterCrossMachinePool` 并解析 `DaemonReply::CrossMachinePoolRegistered`；`free_local_pool_resources`：unlink 本地 shmem + 清 PINNED_POOL/FREED_POOL_IDS 等缓存，复用 free 路径的清理逻辑。）

- [ ] **Step 3: 编译 + 提交**

```bash
cargo check -j 2 -p dora-node-api  # rust node API
# python crate 在服务器 maturin 构建验证（本机环境受限，见 Task 7 部署说明）
git add apis/python/node/src/lib.rs apis/rust/node/src/
git commit -m "feat(api-python): register_memory_pool machine param with sync cross-machine mirror"
```

---

### Task 6: free 双端释放

**Files:**
- Modify: `binaries/daemon/src/lib.rs`

- [ ] **Step 1: 现有 free 处理（4104 附近 `NodeEvent::FreeMemoryPool` 分发处）加跨机转发**

在 daemon 处理节点 free 请求成功之后（`Ok((_meta, touched))` 分支内）加：

```rust
                            // Cross-machine: forward the free to the peer
                            // daemon so it releases the mirrored pool.
                            if let Some(peer) = CROSS_POOLS
                                .lock()
                                .unwrap_or_else(|e| e.into_inner())
                                .remove(&shared_memory_id)
                            {
                                tracing::info!(
                                    "memory pool: forwarding free of {shared_memory_id} to peer {peer}"
                                );
                                publish_memory_pool_event(
                                    &self.zenoh_session,
                                    &dataflow_id,
                                    &InterDaemonEvent::FreePool {
                                        dataflow_id,
                                        shared_memory_id: shared_memory_id.clone(),
                                    },
                                    &self.clock,
                                )
                                .await;
                            }
```

- [ ] **Step 2: 编译 + 提交**

```bash
cargo check -j 2 -p dora-daemon
git add binaries/daemon/src/
git commit -m "feat(daemon): forward cross-machine free to peer daemon"
```

---

### Task 7: 示例与端到端测试（本地双 daemon 台）

**Files:**
- Modify: `examples/memory-pool/sender.py`
- Modify: `examples/memory-pool/cpu2cpu_cross.yml`

- [ ] **Step 1: sender.py 的 register 传 machine 参数 + None 处理**

```python
        memory_pool_id = node.register_memory_pool(
            tensor_info, RECEIVER_DEVICE, machine=os.getenv("cross_machine")
        )
        if memory_pool_id is None:
            print("Cross-machine register failed (warned, no pool created) — exiting", flush=True)
            sys.exit(1)
```

（`cross_machine` 环境变量：默认 None 走本地路径；跨机 YAML 设 `cross_machine: "B"`。）

- [ ] **Step 2: cpu2cpu_cross.yml 加 env**

```yaml
env:
  sender_device: cpu
  receiver_device: cpu
  message_num: 3
  memory_pool_scenario: throughput
  cross_machine: "B"
```

- [ ] **Step 3: 本地双 daemon 台跑正路径**

```bash
# 本地栈（venv PATH + --store memory + 双 daemon A/B，本会话已验证的配置）
cd examples/memory-pool
script -qec "timeout 120 /home/tcr/PyCharmMiscProject/dora/target/debug/dora start --coordinator-addr 127.0.0.1 --coordinator-port 6015 cpu2cpu_cross.yml" /dev/null
```

预期：`Sender preview` 与 `Receiver preview` 匹配；3 帧完整跑完（`Average transfer throughput` 输出）；`B 侧 daemon 日志` 出现 `memory pool: mirrored cross-machine pool pool_sender_node_1`。

- [ ] **Step 4: 负路径 1——machine 不存在**

设 `cross_machine: "NO_SUCH"` 重跑。预期：daemon A 日志出现 `machine "NO_SUCH" 无法解析...` 警告；sender 打印 `Cross-machine register failed` 并退出（exit 1，不崩溃/不挂起）。

- [ ] **Step 5: 负路径 2——无 coordinator**

停掉 coordinator 重跑。预期：同样 warn + 优雅退出。

- [ ] **Step 6: 提交**

```bash
git add examples/memory-pool/
git commit -m "test(memory-pool): cross-machine register via machine param + negatives"
```

---

### Task 8: 性能验证与收尾

**Files:**
- 无（验证）

- [ ] **Step 1: 对比吞吐**

正路径跑完后读 `Average transfer throughput`：预期显著高于旧的 12.94 MB/s（read 走零拷贝快路径，无 hex 往返；理论上限接近纯 zenoh ~148 MB/s 减去两侧处理）。

- [ ] **Step 2: 更新设计文档状态**

在 spec 末尾加"实现状态"小节，标注 v1 完成、后续迭代项（GPU 池、代理路径移除、zenoh SHM 零拷贝）。

- [ ] **Step 3: 提交 + 推送**

```bash
git add docs/superpowers/specs/
git commit -m "docs: mark cross-machine pool v1 implemented"
git push origin main
```
