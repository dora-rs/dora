# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 06:43:07 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 5637.7 | 4911.7 | 6539.7 | 567.1 |
| auto | cpu2cuda | 10 | 3142.7 | 2692.6 | 3413.1 | 230.8 |
| auto | cuda2cpu | 10 | 3450.8 | 3230.9 | 3678.6 | 170.1 |
| nofastpath | cpu2cpu | 10 | 3214.3 | 2601.9 | 4011.6 | 477.7 |
| nofastpath | cpu2cuda | 10 | 2037.8 | 1881.5 | 2330.7 | 160.7 |
| nofastpath | cuda2cpu | 10 | 2570.0 | 2331.4 | 2798.5 | 154.8 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
