# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 07:10:39 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7707.9 | 7325.7 | 7893.1 | 208.5 |
| auto | cpu2cuda | 10 | 4304.5 | 4234.3 | 4471.8 | 84.5 |
| auto | cuda2cpu | 10 | 7949.2 | 7679.6 | 8098.5 | 116.1 |
| nofastpath | cpu2cpu | 10 | 7400.7 | 6931.0 | 7608.0 | 184.0 |
| nofastpath | cpu2cuda | 10 | 4231.6 | 4050.4 | 4355.1 | 82.3 |
| nofastpath | cuda2cpu | 10 | 7826.7 | 7618.0 | 7980.9 | 136.9 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
