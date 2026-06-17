# Memory Pool Example

## Overview

This example exercises Dora's pinned memory-pool transport for repeated tensor transfer between a sender node and a receiver node. The positive scenarios keep the existing throughput-oriented behavior, and the negative scenarios verify that lifecycle errors are surfaced as warnings instead of crashing the nodes.

## Install

```bash
pip install dora-rs-cli  # if not already present

# Install pyarrow with GPU support
conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge
python -c "import pyarrow.cuda"

# Install numba for CUDA helper interop
pip install numba
python -c "import numba.cuda"

# Install torch if it is not already present
pip install torch
python -c "import torch"
```

If you run a CUDA receiver scenario, also verify CUDA is available:

```bash
python -c "import torch; assert torch.cuda.is_available()"
```

## Files

- `sender.py` — registers and updates a memory pool from the sender side.
- `receiver.py` — reads from the memory pool, measures throughput, and triggers lifecycle scenarios.
- `cpu2cuda.yml` — positive throughput test for CPU sender → CUDA receiver.
- `cuda2cpu.yml` — positive throughput test for CUDA sender → CPU receiver.
- `duplicate_free.yml` — receiver frees the same memory pool twice.
- `read_after_free.yml` — receiver frees, then reads the same memory pool again.
- `write_after_free.yml` — sender frees, then writes the same memory pool again.
- `auto_cleanup.yml` — receiver does not free; daemon cleanup is expected on shutdown.

## Run

### Positive throughput scenarios

```bash
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
- The CUDA receiver scenarios require a working CUDA runtime.
- The negative scenarios use a reduced message count to keep lifecycle validation short and focused.
