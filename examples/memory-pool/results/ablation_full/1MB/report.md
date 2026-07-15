# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 11:52:47 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7032.3 | 5995.4 | 7465.2 | 478.3 |
| auto | cpu2cuda | 10 | 3057.7 | 2018.0 | 3297.7 | 409.1 |
| auto | cuda2cpu | 10 | 3912.1 | 3551.6 | 4145.3 | 176.8 |
| nofastpath | cpu2cpu | 10 | 4925.1 | 4135.8 | 6476.2 | 641.8 |
| nofastpath | cpu2cuda | 10 | 2961.5 | 2580.8 | 3126.3 | 163.1 |
| nofastpath | cuda2cpu | 10 | 3478.9 | 3065.6 | 3752.8 | 245.2 |
| noreuse | cpu2cpu | 10 | 1252.5 | 1105.1 | 1291.0 | 65.8 |
| noreuse | cpu2cuda | 10 | 698.1 | 635.9 | 762.4 | 33.1 |
| noreuse | cuda2cpu | 10 | 1115.0 | 973.7 | 1162.3 | 54.2 |
| pageable | cpu2cuda | 10 | 3181.2 | 2656.8 | 3276.5 | 186.4 |
| pinned | cpu2cuda | 10 | 1556.3 | 1418.2 | 1680.8 | 83.3 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
