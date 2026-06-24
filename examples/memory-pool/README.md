# Memory Pool Example

## Overview

This example exercises Dora's pinned memory-pool transport for repeated tensor transfer between a sender node and a receiver node. The positive scenarios keep the existing throughput-oriented behavior, and the negative scenarios verify that lifecycle errors are surfaced as warnings instead of crashing the nodes.

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

- `sender.py` — registers and updates a memory pool from the sender side.
- `receiver.py` — reads from the memory pool, measures throughput, and triggers lifecycle scenarios.
- `cpu2cpu.yml` — positive throughput test for CPU sender → CPU receiver (GPU-less CI safe).
- `cpu2cuda.yml` — positive throughput test for CPU sender → CUDA receiver.
- `cuda2cpu.yml` — positive throughput test for CUDA sender → CPU receiver.
- `duplicate_free.yml` — receiver frees the same memory pool twice (CPU receiver).
- `read_after_free.yml` — receiver frees, then reads the same memory pool again (CPU receiver).
- `write_after_free.yml` — sender frees, then writes the same memory pool again (CPU receiver).
- `auto_cleanup.yml` — receiver does not free; daemon cleanup is expected on shutdown (CPU receiver).

## Run

### Positive throughput scenarios

```bash
dora run examples/memory-pool/cpu2cpu.yml
dora run examples/memory-pool/cpu2cuda.yml
dora run examples/memory-pool/cuda2cpu.yml
```

Expected behavior:
- the dataflow runs to completion
- the sender and receiver print preview tensors
- the receiver prints average throughput
- no crash or obvious memory error occurs

### Negative-path scenarios

```bash
dora run examples/memory-pool/duplicate_free.yml
dora run examples/memory-pool/read_after_free.yml
dora run examples/memory-pool/write_after_free.yml
dora run examples/memory-pool/auto_cleanup.yml
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
