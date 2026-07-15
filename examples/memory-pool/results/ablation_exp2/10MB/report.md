# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 06:55:40 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 5250.3 | 4633.3 | 5437.5 | 282.4 |
| auto | cpu2cuda | 10 | 4297.3 | 3915.3 | 4568.2 | 212.1 |
| auto | cuda2cpu | 10 | 6893.5 | 5995.9 | 7426.6 | 521.4 |
| nofastpath | cpu2cpu | 10 | 4441.0 | 3681.4 | 4971.1 | 429.4 |
| nofastpath | cpu2cuda | 10 | 3774.6 | 3425.6 | 4252.4 | 242.7 |
| nofastpath | cuda2cpu | 10 | 6752.9 | 5494.1 | 7227.2 | 597.3 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
