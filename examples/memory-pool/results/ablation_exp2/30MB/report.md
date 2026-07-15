# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 07:18:40 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7604.2 | 7338.3 | 7792.8 | 135.4 |
| auto | cpu2cuda | 10 | 4589.8 | 4454.0 | 4816.3 | 115.3 |
| auto | cuda2cpu | 10 | 8127.5 | 8003.7 | 8215.2 | 61.6 |
| nofastpath | cpu2cpu | 10 | 7446.3 | 7183.9 | 7556.3 | 119.4 |
| nofastpath | cpu2cuda | 10 | 4523.1 | 4194.2 | 4742.6 | 159.8 |
| nofastpath | cuda2cpu | 10 | 7860.7 | 7016.1 | 8033.3 | 292.5 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
