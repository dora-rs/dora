# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 07:03:00 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7334.8 | 6247.4 | 8052.2 | 499.6 |
| auto | cpu2cuda | 10 | 4129.7 | 3545.8 | 4410.4 | 309.1 |
| auto | cuda2cpu | 10 | 7833.0 | 7657.6 | 8054.7 | 122.0 |
| nofastpath | cpu2cpu | 10 | 7315.2 | 6586.7 | 7574.8 | 271.0 |
| nofastpath | cpu2cuda | 10 | 4168.7 | 4022.6 | 4262.6 | 70.3 |
| nofastpath | cuda2cpu | 10 | 7611.3 | 7007.2 | 7786.8 | 210.6 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
