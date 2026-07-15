# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-03 09:37:23 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7275.6 | 6965.2 | 7440.6 | 136.6 |
| full | cpu2cuda | 10 | 6075.2 | 5908.5 | 6193.4 | 82.8 |
| full | cuda2cpu | 10 | 8519.6 | 8367.9 | 8597.9 | 67.9 |
| nofastpath | cpu2cpu | 10 | 7332.9 | 7146.9 | 7445.7 | 98.0 |
| nofastpath | cpu2cuda | 10 | 6038.3 | 5673.1 | 6076.5 | 124.7 |
| nofastpath | cuda2cpu | 10 | 8607.1 | 8543.5 | 8663.4 | 36.0 |
| noreuse | cpu2cpu | 10 | 1680.3 | 1664.4 | 1696.1 | 9.3 |
| noreuse | cpu2cuda | 10 | 1289.4 | 1274.8 | 1294.1 | 7.0 |
| noreuse | cuda2cpu | 10 | 1774.6 | 1720.6 | 1804.5 | 27.3 |
| pageable | cpu2cpu | 10 | 7431.9 | 6527.4 | 7592.4 | 349.5 |
| pageable | cpu2cuda | 10 | 4512.8 | 4284.4 | 4530.4 | 80.3 |
| pageable | cuda2cpu | 10 | 8667.2 | 8612.0 | 8736.8 | 42.5 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
