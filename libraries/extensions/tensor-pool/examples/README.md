# Memory Pool Example

## Overview

This example exercises Dora's pinned memory-pool transport for repeated tensor transfer between a sender node and a receiver node. The positive scenarios keep the existing throughput-oriented behavior, and the negative scenarios verify that lifecycle errors are surfaced as warnings instead of crashing the nodes.

Beyond the single-daemon scenarios, the `*_cross*.yml` dataflows exercise the **same-host multi-daemon** and **cross-machine** topologies: `register_tensor_pool(machine=...)` mirrors the pool on another daemon (same host or another machine), with zero-copy direct reads when the daemons share a host and a reliable zenoh/TCP data plane when they do not.

## Install

```bash
pip install dora-rs-cli  # if not already present
```

When running with `--uv`, Dora provisions torch into the per-node managed environments automatically via the `build:` steps in each YAML file. If you run outside `--uv`, install the dependencies manually:

```bash
pip install torch numpy pyarrow tqdm
```

For CUDA receiver scenarios (`cpu2cuda.yml`, `cuda2cpu.yml`), also verify CUDA is available:

```bash
python -c "import torch; assert torch.cuda.is_available()"
```

## Files

- `sender.py` — registers and updates a memory pool from the sender side. Reads `cross_machine` (target machine id) from the environment to switch to cross-machine registration.
- `receiver.py` — reads from the memory pool, measures throughput, and triggers lifecycle scenarios.
- `cpu2cpu.yml` — positive throughput test for CPU sender → CPU receiver (GPU-less CI safe).
- `cpu2cuda.yml` — positive throughput test for CPU sender → CUDA receiver.
- `cuda2cpu.yml` — positive throughput test for CUDA sender → CPU receiver.
- `duplicate_free.yml` — receiver frees the same memory pool twice (CPU receiver).
- `read_after_free.yml` — receiver frees, then reads the same memory pool again (CPU receiver).
- `write_after_free.yml` — sender frees, then writes the same memory pool again (CPU receiver).
- `auto_cleanup.yml` — receiver does not free; daemon cleanup is expected on shutdown (CPU receiver).
- `cpu2cpu_cross.yml` / `cpu2cpu_cross_local.yml` — CPU→CPU cross-machine / same-host cross-daemon throughput test.
- `cpu2cuda_cross.yml` / `cuda2cpu_cross.yml` / `cuda2cuda_cross.yml` — GPU-involved cross-machine throughput tests (CPU staging pools are created automatically on the GPU side).

## Run

### Positive throughput scenarios (single daemon)

```bash
dora run libraries/extensions/tensor-pool/examples/cpu2cpu.yml
dora run libraries/extensions/tensor-pool/examples/cpu2cuda.yml
dora run libraries/extensions/tensor-pool/examples/cuda2cpu.yml
```

Expected behavior:
- the dataflow runs to completion
- the sender and receiver print preview tensors
- the receiver prints average throughput
- no crash or obvious memory error occurs

### Same-host multi-daemon (`cpu2cpu_cross_local.yml`)

Two daemons on one machine, sender under daemon A, receiver under daemon B. Without any extra configuration the pool **auto-detects** that the daemons share `/dev/shm` (the register ack reports `direct=true`) and the receiver reads the sender's segment in place — zero-copy, bypassing the daemon relay entirely. This is the "防呆" design: a same-host multi-daemon deployment never silently falls back to the relay (89–113 MB/s); it always takes the direct read (~5.8 GB/s).

```bash
# coordinator + two daemons (each needs its own --local-listen-port on one host)
dora coordinator --port 6025 --store memory
dora daemon --machine-id A --coordinator-addr 127.0.0.1 --coordinator-port 6025 \
  --zenoh-peer tcp/127.0.0.1:5463 --local-listen-port 0
dora daemon --machine-id B --coordinator-addr 127.0.0.1 --coordinator-port 6025 \
  --zenoh-peer tcp/127.0.0.1:5463 --local-listen-port 0

# build through the coordinator (deploy sections require it), then start attached
dora build --coordinator-port 6025 libraries/extensions/tensor-pool/examples/cpu2cpu_cross_local.yml
dora start --coordinator-port 6025 libraries/extensions/tensor-pool/examples/cpu2cpu_cross_local.yml --attach
```

Expected: `Average transfer throughput` ≈ **5800 MB/s** (same-host direct read; the relay baseline for the same topology is 89–113 MB/s, a 50–65× gap). This scenario is also covered by the torch-gated smoke test `smoke_local_memory_pool_cpu2cpu_cross_local`.

### Cross-machine (`cpu2cpu_cross.yml` and the GPU `*_cross.yml` variants)

Sender on machine A, receiver on machine B, data over zenoh TCP between the two daemons.

**Cluster bring-up** (one daemon per machine, the coordinator can run on either):

```bash
# machine that publishes to the public network / serves as the zenoh rendezvous
dora coordinator --interface 0.0.0.0 --port 6025 --store memory
dora daemon --machine-id B --coordinator-addr 127.0.0.1 --coordinator-port 6025 \
  --zenoh-peer tcp/0.0.0.0:5463          # listen on 5463, published to the peer machine

# the other machine
dora daemon --machine-id A --coordinator-addr <rendezvous-ip> --coordinator-port 6025 \
  --zenoh-peer tcp/<rendezvous-ip>:5463  # dials the rendezvous
```

**The YAML needs three things** (all present in the `*_cross*.yml` files):

1. `env: cross_machine: "B"` — the sender registers with `register_tensor_pool(machine="B")`; without it the pool stays local and the receiver never sees a mirror.
2. `_unstable_deploy: machine: A|B` per node — which daemon spawns which node.
3. `_unstable_deploy: working_dir: .` (relative to the yml's directory) — relative to the daemon's cwd (repo root), per the multiple-daemons convention. Absolute paths are NOT portable.

**True-WAN zenoh config** (`ZENOH_CONFIG` env on the dialing daemon) — three points, all required on a real WAN link (default multicast discovery does not work across routed networks):

```json5
// zenoh_wan.json5
{
  connect: { endpoints: ["tcp/<rendezvous-ip>:5463"] },   // explicit connect, no multicast
  scouting: { multicast: { enabled: false } },             // else "Scouting delay elapsed"
  transport: { link: { tx: { queue: { congestion_control: {
    block: { wait_before_close: 60000000 }                 // 60s; the 5s default kills the
  } } } } },                                               // session on slow-link bursts
}
```

```bash
ZENOH_CONFIG=/path/to/zenoh_wan.json5 dora daemon --machine-id A --coordinator-addr ... --zenoh-peer tcp/<ip>:5463
```

**Run** (a YAML without `build:` steps can be started directly — `dora build` races a fast build and may report "no running build", which is harmless):

```bash
dora build --coordinator-addr <rendezvous-ip> --coordinator-port 6025 libraries/extensions/tensor-pool/examples/cpu2cpu_cross.yml
dora start --coordinator-addr <rendezvous-ip> --coordinator-port 6025 libraries/extensions/tensor-pool/examples/cpu2cpu_cross.yml --attach
```

**Known behavior** (measured numbers live in `design.md` §5):

- Cross-machine pools accept tensors up to the 1 GiB registration cap. The per-frame write sends only metadata to the daemon (shared-memory reference); the daemon reads the sender's segment and forwards it, so the 64 MiB node→daemon request limit does not apply.
- Writes are **commit-acknowledged**: `write_tensor_pool` returns only after the mirror daemon confirms the segment write, so the `send_output` notification that follows can never overtake the data (the receiver can never return a stale frame). A failed mirror write or a 120 s ack timeout fails the write loudly.
- GPU-involved cross-machine paths stage through CPU pools automatically (GPU_A → DtoH → CPU_A → zenoh TCP → CPU_B → HtoD → GPU_B).
- Native dora's cross-machine relay carries only small frames and hangs beyond that (Drop + express silently drops fragments when the 16-batch TX queue backs up); ROS 2 network DDS is RTT-paced on high-latency links. The pool is the only path that moves large frames over the WAN.
- Same-host cross-daemon reads bypass the write/ack machinery entirely (`direct=true`).

**Debugging checklist** for cross-machine runs:

- Set `WALL_CLOCK: 1` in the YAML env (or `WALL_CLOCK=1`): cross-machine timing must use wall clock — `perf_counter`'s epoch is each machine's boot time, so deltas are dominated by boot-time differences; the hosts are NTP-synced, making wall-clock deltas the true transfer time.
- Verify the connections: `ss -tn | grep 5463` shows the dialing daemon's ESTAB to the rendezvous; the coordinator WS (6025) must be reachable from every daemon.
- The mirror/peer daemon's repeated "Unable to connect to any locator of scouted peer" WARNs are cosmetic when the dialing side is behind NAT — the data path is established by the dialing daemon's outbound connection.
- `receiver preview == sender preview` (byte-identical tensors) is the integrity check; a stale mirror would show mismatched first elements.
- Test scripts (native-dora control harness, sweep scripts, session logs) live in `/home/tcr/dora_test/` — reuse them for re-measurement.

### Negative-path scenarios

```bash
dora run libraries/extensions/tensor-pool/examples/duplicate_free.yml
dora run libraries/extensions/tensor-pool/examples/read_after_free.yml
dora run libraries/extensions/tensor-pool/examples/write_after_free.yml
dora run libraries/extensions/tensor-pool/examples/auto_cleanup.yml
```

Expected warnings/info:
- duplicate free:
  - `Attempt to release memory pool [memory_pool_id] failed - reason: pool does not exist. Operation aborted.`
- read after free:
  - `Attempt to read memory pool [memory_pool_id] failed - reason: pool does not exist. Operation aborted.`
- write after free:
  - `Attempt to write memory pool [memory_pool_id] failed - reason: pool does not exist. Operation aborted.`
- auto cleanup:
  - `Detected xx unreleased memory pool, releasing...`
  - `Successfully released xx unreleased memory pools!`

## Notes

- The scenario is controlled through the `memory_pool_scenario` environment variable in each YAML file.
- `cpu2cpu.yml` and the four negative-lifecycle YAMLs use CPU-only receiver (`receiver_device: cpu`) and are safe for GPU-less CI runners.
- The CUDA receiver scenarios (`cpu2cuda.yml`, `cuda2cpu.yml`) require a working CUDA runtime.
- The negative scenarios use a reduced message count to keep lifecycle validation short and focused.
- When running with `--uv`, each YAML's `build:` step provisions torch (CPU-only from `download.pytorch.org/whl/cpu`) into per-node managed environments, so no pre-installed torch is needed.
- The cross-machine YAMLs (`*_cross*.yml`) need two daemons and therefore cannot run on standard CI; the same-host variant (`cpu2cpu_cross_local.yml`) is covered by the torch-gated `memory-pool-smoke` nightly job.
