# Example of latency using CUDA Zero Copy instead of regular Shared Memory over CPU

## Install

```bash
cargo install dora-cli  # if not already present

# Install pyarrow with gpu support
conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge

## Test installation with
python -c "import pyarrow.cuda"

# Install torch if it's not already present (dora.cuda uses ctypes + libcudart,
# no numba dependency needed).
pip install torch

## Test installation with
python -c "import torch; assert torch.cuda.is_available()"
```

## Run

```bash
dora run cpu_bench.yml

dora run cuda_bench.yml

cat benchmark_data.csv
```

## To run the demo code

```bash
dora up
dora start demo_bench.yml --detach
python demo_receiver.py
dora down
```
