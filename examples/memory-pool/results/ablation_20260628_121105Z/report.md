# HeteroPool Ablation Experiment Report

**Generated:** 2026-06-28 12:15:11 UTC

**Total runs:** 24

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 2 | 7053.3 | 7000.7 | 7105.9 | 74.4 |
| full | cpu2cuda | 2 | 5621.9 | 5580.2 | 5663.6 | 59.0 |
| full | cuda2cpu | 2 | 8179.5 | 8156.0 | 8202.9 | 33.2 |
| nofastpath | cpu2cpu | 2 | 6894.0 | 6875.7 | 6912.3 | 25.9 |
| nofastpath | cpu2cuda | 2 | 5015.6 | 5015.2 | 5016.0 | 0.6 |
| nofastpath | cuda2cpu | 2 | 7409.0 | 7065.6 | 7752.4 | 485.7 |
| noreuse | cpu2cpu | 2 | 1655.7 | 1557.7 | 1753.7 | 138.6 |
| noreuse | cpu2cuda | 2 | 1259.2 | 1249.5 | 1268.9 | 13.7 |
| noreuse | cuda2cpu | 2 | 1914.6 | 1909.9 | 1919.3 | 6.7 |
| pageable | cpu2cpu | 2 | 7185.5 | 7093.7 | 7277.3 | 129.8 |
| pageable | cpu2cuda | 2 | 4113.5 | 4094.4 | 4132.6 | 27.0 |
| pageable | cuda2cpu | 2 | 8163.9 | 8111.9 | 8215.8 | 73.5 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
