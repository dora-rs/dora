# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 08:02:55 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7255.4 | 7154.2 | 7301.0 | 44.6 |
| auto | cpu2cuda | 10 | 6259.1 | 6036.0 | 6454.1 | 128.2 |
| auto | cuda2cpu | 10 | 8626.6 | 8509.4 | 8729.7 | 73.0 |
| nofastpath | cpu2cpu | 10 | 6913.8 | 6852.1 | 6941.9 | 27.1 |
| nofastpath | cpu2cuda | 10 | 6192.4 | 5305.7 | 6389.7 | 312.8 |
| nofastpath | cuda2cpu | 10 | 8739.2 | 8647.8 | 8833.9 | 57.4 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
