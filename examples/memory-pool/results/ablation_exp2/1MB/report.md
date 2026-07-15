# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 06:49:00 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7883.7 | 6555.5 | 8818.5 | 669.8 |
| auto | cpu2cuda | 10 | 3656.6 | 3288.4 | 4008.5 | 247.5 |
| auto | cuda2cpu | 10 | 4577.6 | 4337.0 | 4786.3 | 147.0 |
| nofastpath | cpu2cpu | 10 | 6179.2 | 5468.0 | 6806.7 | 369.3 |
| nofastpath | cpu2cuda | 10 | 3144.2 | 2848.1 | 3376.0 | 152.3 |
| nofastpath | cuda2cpu | 10 | 3708.7 | 3491.6 | 4178.2 | 216.6 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
