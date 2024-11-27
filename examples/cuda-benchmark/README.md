# Example of latency using CUDA Zero Copy instead of regular Shared Memory over CPU

## Install

```
pip install dora-rs-cli # if not already present

# Install pyarrow with gpu support
conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge

# Install numba for translation from arrow to torch
pip install numba

# Install torch if it's not already present
pip install torch
```

## Run

```bash
dora run cpu_bench.yml

dora run cuda_bench.yml

cat benchmark_data.csv
```
