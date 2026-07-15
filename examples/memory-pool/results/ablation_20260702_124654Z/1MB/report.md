# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 13:10:23 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7740.9 | 6876.2 | 8695.9 | 564.0 |
| full | cpu2cuda | 10 | 1697.1 | 1351.1 | 1852.6 | 133.7 |
| full | cuda2cpu | 10 | 4246.2 | 3427.8 | 4563.8 | 324.2 |
| nofastpath | cpu2cpu | 10 | 5296.7 | 4563.4 | 5739.4 | 354.4 |
| nofastpath | cpu2cuda | 10 | 1448.7 | 1342.8 | 1507.1 | 61.1 |
| nofastpath | cuda2cpu | 10 | 3401.8 | 2920.1 | 3622.1 | 233.4 |
| noreuse | cpu2cpu | 10 | 1294.6 | 1081.8 | 1406.6 | 99.2 |
| noreuse | cpu2cuda | 10 | 561.5 | 505.3 | 640.8 | 41.2 |
| noreuse | cuda2cpu | 10 | 1185.6 | 984.1 | 1267.0 | 77.8 |
| pageable | cpu2cpu | 10 | 7354.4 | 6037.0 | 8942.1 | 798.7 |
| pageable | cpu2cuda | 10 | 3431.3 | 3101.4 | 3915.1 | 227.8 |
| pageable | cuda2cpu | 10 | 4254.3 | 3729.6 | 4657.5 | 289.4 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
