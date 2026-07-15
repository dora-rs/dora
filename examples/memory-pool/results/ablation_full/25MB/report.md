# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-04 12:34:06 UTC

**Total runs:** 110

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7384.2 | 5604.1 | 7704.4 | 726.7 |
| auto | cpu2cuda | 10 | 4075.6 | 3538.4 | 4312.1 | 237.9 |
| auto | cuda2cpu | 10 | 7892.4 | 7034.6 | 8113.8 | 303.5 |
| nofastpath | cpu2cpu | 10 | 6825.5 | 6185.2 | 7249.6 | 356.7 |
| nofastpath | cpu2cuda | 10 | 4099.6 | 3608.6 | 4244.6 | 199.7 |
| nofastpath | cuda2cpu | 10 | 7703.5 | 6709.3 | 7858.5 | 410.8 |
| noreuse | cpu2cpu | 10 | 1517.1 | 1455.5 | 1580.9 | 39.2 |
| noreuse | cpu2cuda | 10 | 1021.8 | 938.7 | 1051.1 | 40.1 |
| noreuse | cuda2cpu | 10 | 1694.3 | 1603.1 | 1746.2 | 44.5 |
| pageable | cpu2cuda | 10 | 4133.6 | 3593.3 | 4262.4 | 231.6 |
| pinned | cpu2cuda | 10 | 4286.1 | 3862.2 | 4581.0 | 245.7 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
