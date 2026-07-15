# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 14:18:50 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7509.0 | 7463.6 | 7605.2 | 43.2 |
| auto | cpu2cuda | 10 | 5963.1 | 5700.4 | 6483.9 | 255.8 |
| auto | cuda2cpu | 10 | 8684.5 | 8602.5 | 8735.5 | 41.2 |
| nofastpath | cpu2cpu | 10 | 7436.0 | 6687.3 | 7515.6 | 245.0 |
| nofastpath | cpu2cuda | 10 | 6326.1 | 6103.4 | 6548.0 | 133.9 |
| nofastpath | cuda2cpu | 10 | 8581.9 | 8133.9 | 8686.7 | 159.7 |
| noreuse | cpu2cpu | 10 | 1713.1 | 1683.9 | 1730.3 | 12.9 |
| noreuse | cpu2cuda | 10 | 1329.5 | 1315.0 | 1348.1 | 11.5 |
| noreuse | cuda2cpu | 10 | 1812.1 | 1800.1 | 1830.4 | 10.5 |
| pageable | cpu2cuda | 10 | 4491.9 | 4444.9 | 4505.5 | 23.4 |
| pinned | cpu2cuda | 10 | 6495.5 | 6421.8 | 6547.6 | 46.3 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
