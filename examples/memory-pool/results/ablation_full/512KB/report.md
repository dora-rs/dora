# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 11:41:56 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 4618.2 | 4036.2 | 5872.4 | 590.2 |
| auto | cpu2cuda | 10 | 2665.6 | 2414.3 | 2953.0 | 187.5 |
| auto | cuda2cpu | 10 | 3169.2 | 2963.2 | 3288.0 | 114.6 |
| nofastpath | cpu2cpu | 10 | 3276.7 | 2668.1 | 3806.5 | 346.1 |
| nofastpath | cpu2cuda | 10 | 1983.5 | 1857.0 | 2149.7 | 118.0 |
| nofastpath | cuda2cpu | 10 | 2458.4 | 2109.0 | 2850.2 | 225.7 |
| noreuse | cpu2cpu | 10 | 762.4 | 522.9 | 890.4 | 108.1 |
| noreuse | cpu2cuda | 10 | 459.1 | 434.2 | 493.4 | 22.6 |
| noreuse | cuda2cpu | 10 | 707.6 | 607.9 | 781.8 | 50.6 |
| pageable | cpu2cuda | 10 | 2638.6 | 2478.2 | 2887.5 | 168.1 |
| pinned | cpu2cuda | 10 | 1143.9 | 1051.4 | 1212.6 | 56.3 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
