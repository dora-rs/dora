# Example of latency using CUDA Zero Copy instead of regular Shared Memory over CPU

## Install

```bash
pip install dora-rs-cli # if not already present

# Install pyarrow with gpu support
conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge

## Test installation with
python -c "import pyarrow.cuda"

# Install numba for translation from arrow to torch
pip install numba

## Test installation with
python -c "import numba.cuda"

# Install torch if it's not already present
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
