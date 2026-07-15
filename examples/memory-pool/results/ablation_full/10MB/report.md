# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 12:05:12 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 5025.7 | 4164.6 | 5318.4 | 416.7 |
| auto | cpu2cuda | 10 | 4018.6 | 3587.6 | 4392.5 | 286.6 |
| auto | cuda2cpu | 10 | 7026.3 | 5575.8 | 7248.7 | 538.3 |
| nofastpath | cpu2cpu | 10 | 4504.9 | 3707.8 | 4823.8 | 419.9 |
| nofastpath | cpu2cuda | 10 | 3791.0 | 3441.3 | 4237.0 | 231.9 |
| nofastpath | cuda2cpu | 10 | 6813.6 | 6628.1 | 7021.9 | 124.1 |
| noreuse | cpu2cpu | 10 | 1206.2 | 983.5 | 1355.3 | 120.9 |
| noreuse | cpu2cuda | 10 | 895.2 | 721.8 | 922.4 | 63.6 |
| noreuse | cuda2cpu | 10 | 1568.1 | 1512.2 | 1643.7 | 40.8 |
| pageable | cpu2cuda | 10 | 3977.2 | 3501.8 | 4409.6 | 254.7 |
| pinned | cpu2cuda | 10 | 3053.0 | 2886.2 | 3262.1 | 118.5 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
