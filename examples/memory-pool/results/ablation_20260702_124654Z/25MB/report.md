# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 13:55:04 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7171.7 | 6653.8 | 7499.1 | 259.4 |
| full | cpu2cuda | 10 | 4284.7 | 4198.4 | 4559.9 | 111.1 |
| full | cuda2cpu | 10 | 7765.0 | 7537.1 | 7936.9 | 140.0 |
| nofastpath | cpu2cpu | 10 | 6822.9 | 6601.8 | 7056.2 | 174.8 |
| nofastpath | cpu2cuda | 10 | 3833.5 | 3760.7 | 3999.7 | 69.1 |
| nofastpath | cuda2cpu | 10 | 7439.4 | 7296.2 | 7626.3 | 114.4 |
| noreuse | cpu2cpu | 10 | 1634.7 | 1556.2 | 1676.1 | 43.1 |
| noreuse | cpu2cuda | 10 | 1202.0 | 1150.1 | 1256.9 | 29.0 |
| noreuse | cuda2cpu | 10 | 1901.3 | 1872.0 | 1935.3 | 19.3 |
| pageable | cpu2cpu | 10 | 7169.7 | 7010.0 | 7377.4 | 124.5 |
| pageable | cpu2cuda | 10 | 4079.7 | 3920.1 | 4156.6 | 66.6 |
| pageable | cuda2cpu | 10 | 7859.4 | 7763.8 | 7963.2 | 59.2 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
