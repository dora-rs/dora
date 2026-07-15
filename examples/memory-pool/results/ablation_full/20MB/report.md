# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 12:19:15 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7702.8 | 7306.8 | 7954.9 | 202.1 |
| auto | cpu2cuda | 10 | 4270.4 | 4107.2 | 4340.1 | 64.2 |
| auto | cuda2cpu | 10 | 7690.7 | 5929.3 | 7917.9 | 608.7 |
| nofastpath | cpu2cpu | 10 | 6988.4 | 5856.6 | 7509.7 | 582.7 |
| nofastpath | cpu2cuda | 10 | 4102.5 | 3480.3 | 4243.5 | 224.6 |
| nofastpath | cuda2cpu | 10 | 7653.1 | 5794.7 | 7726.7 | 608.7 |
| noreuse | cpu2cpu | 10 | 1414.3 | 1225.5 | 1466.5 | 88.5 |
| noreuse | cpu2cuda | 10 | 1004.4 | 891.0 | 1030.2 | 49.7 |
| noreuse | cuda2cpu | 10 | 1568.4 | 1465.1 | 1624.2 | 66.0 |
| pageable | cpu2cuda | 10 | 4249.7 | 4021.2 | 4367.1 | 110.2 |
| pinned | cpu2cuda | 10 | 4017.4 | 3669.4 | 4352.8 | 181.2 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
