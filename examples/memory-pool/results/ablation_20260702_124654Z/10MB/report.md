# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 13:23:55 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 4749.9 | 4028.4 | 5322.9 | 346.2 |
| full | cpu2cuda | 10 | 3021.7 | 2900.3 | 3216.1 | 96.6 |
| full | cuda2cpu | 10 | 6658.0 | 6245.9 | 7352.5 | 403.7 |
| nofastpath | cpu2cpu | 10 | 4630.9 | 4306.0 | 4909.0 | 184.0 |
| nofastpath | cpu2cuda | 10 | 2846.9 | 2368.3 | 3106.2 | 194.8 |
| nofastpath | cuda2cpu | 10 | 6598.1 | 6354.2 | 6736.7 | 134.2 |
| noreuse | cpu2cpu | 10 | 1510.2 | 1324.3 | 1593.5 | 86.4 |
| noreuse | cpu2cuda | 10 | 985.9 | 945.2 | 1009.1 | 21.5 |
| noreuse | cuda2cpu | 10 | 1793.1 | 1736.4 | 1815.2 | 22.6 |
| pageable | cpu2cpu | 10 | 4710.2 | 4324.0 | 5169.3 | 318.1 |
| pageable | cpu2cuda | 10 | 3927.3 | 3497.8 | 4251.4 | 242.5 |
| pageable | cuda2cpu | 10 | 6778.9 | 6079.9 | 7259.1 | 348.1 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
