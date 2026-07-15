# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 13:08:52 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7242.5 | 6893.7 | 7405.7 | 164.8 |
| auto | cpu2cuda | 10 | 4861.5 | 4539.7 | 4986.7 | 139.9 |
| auto | cuda2cpu | 10 | 7817.9 | 6798.4 | 8215.7 | 406.9 |
| nofastpath | cpu2cpu | 10 | 7095.0 | 6442.8 | 7258.7 | 274.4 |
| nofastpath | cpu2cuda | 10 | 4686.4 | 4212.6 | 4890.7 | 185.8 |
| nofastpath | cuda2cpu | 10 | 7638.1 | 6353.5 | 8014.3 | 553.2 |
| noreuse | cpu2cpu | 10 | 1560.7 | 1453.3 | 1584.3 | 44.6 |
| noreuse | cpu2cuda | 10 | 1115.2 | 1015.1 | 1149.5 | 51.7 |
| noreuse | cuda2cpu | 10 | 1739.1 | 1717.9 | 1769.8 | 18.1 |
| pageable | cpu2cuda | 10 | 4136.9 | 3680.5 | 4286.1 | 188.0 |
| pinned | cpu2cuda | 10 | 4897.6 | 4546.4 | 5062.5 | 152.3 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
