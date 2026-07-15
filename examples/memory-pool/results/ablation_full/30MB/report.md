# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 12:49:45 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7213.8 | 6225.8 | 7568.6 | 532.7 |
| auto | cpu2cuda | 10 | 4035.5 | 3830.2 | 4295.1 | 166.7 |
| auto | cuda2cpu | 10 | 7652.8 | 6673.0 | 8091.6 | 500.4 |
| nofastpath | cpu2cpu | 10 | 7219.5 | 6989.1 | 7367.7 | 110.7 |
| nofastpath | cpu2cuda | 10 | 4036.7 | 3850.7 | 4192.1 | 99.8 |
| nofastpath | cuda2cpu | 10 | 7769.0 | 7043.9 | 7904.4 | 317.3 |
| noreuse | cpu2cpu | 10 | 1509.7 | 1378.9 | 1580.9 | 68.2 |
| noreuse | cpu2cuda | 10 | 1124.1 | 1076.6 | 1164.3 | 26.5 |
| noreuse | cuda2cpu | 10 | 1717.9 | 1614.4 | 1765.5 | 51.6 |
| pageable | cpu2cuda | 10 | 3993.4 | 3442.9 | 4241.9 | 292.7 |
| pinned | cpu2cuda | 10 | 4186.4 | 3955.7 | 4406.9 | 131.1 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
