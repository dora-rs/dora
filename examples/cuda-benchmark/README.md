# Example of latency using CUDA Zero Copy instead of regular Shared Memory over CPU

## Install

```bash
pip install adora-rs-cli # if not already present

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
adora run cpu_bench.yml

adora run cuda_bench.yml

cat benchmark_data.csv
```

## To run the demo code

```bash
adora up
adora start demo_bench.yml --detach
python demo_receiver.py
adora destroy
```
