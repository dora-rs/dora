# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 07:41:45 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7199.9 | 7114.6 | 7248.5 | 45.5 |
| auto | cpu2cuda | 10 | 5978.2 | 5835.1 | 6111.4 | 99.9 |
| auto | cuda2cpu | 10 | 8331.2 | 8187.8 | 8399.7 | 72.7 |
| nofastpath | cpu2cpu | 10 | 6755.6 | 6546.9 | 6849.6 | 81.5 |
| nofastpath | cpu2cuda | 10 | 5789.7 | 4880.4 | 5913.0 | 310.7 |
| nofastpath | cuda2cpu | 10 | 8304.5 | 8165.1 | 8440.3 | 72.6 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
