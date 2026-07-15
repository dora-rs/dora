# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 14:33:42 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7026.6 | 6889.1 | 7154.3 | 85.2 |
| full | cpu2cuda | 10 | 5095.7 | 4885.9 | 5184.1 | 96.4 |
| full | cuda2cpu | 10 | 7971.8 | 7789.6 | 8046.6 | 84.4 |
| nofastpath | cpu2cpu | 10 | 6667.0 | 6565.0 | 6724.4 | 60.2 |
| nofastpath | cpu2cuda | 10 | 4884.4 | 4783.0 | 5032.6 | 75.2 |
| nofastpath | cuda2cpu | 10 | 7736.2 | 7555.8 | 7853.9 | 99.5 |
| noreuse | cpu2cpu | 10 | 1632.5 | 1617.4 | 1677.4 | 21.5 |
| noreuse | cpu2cuda | 10 | 1234.8 | 1204.3 | 1256.6 | 17.5 |
| noreuse | cuda2cpu | 10 | 1850.3 | 1831.4 | 1884.9 | 17.5 |
| pageable | cpu2cpu | 10 | 7092.2 | 7006.8 | 7208.2 | 69.3 |
| pageable | cpu2cuda | 10 | 4102.2 | 4024.3 | 4143.0 | 35.7 |
| pageable | cuda2cpu | 10 | 7950.9 | 7788.0 | 8067.6 | 103.8 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
