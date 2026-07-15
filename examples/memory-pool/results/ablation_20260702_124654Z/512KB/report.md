# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 12:58:37 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 4604.0 | 4113.2 | 5810.4 | 558.0 |
| full | cpu2cuda | 10 | 1124.5 | 879.4 | 1254.6 | 106.2 |
| full | cuda2cpu | 10 | 3171.5 | 2884.3 | 3624.4 | 225.1 |
| nofastpath | cpu2cpu | 10 | 3235.2 | 2697.4 | 3910.7 | 332.8 |
| nofastpath | cpu2cuda | 10 | 924.6 | 710.6 | 950.4 | 70.0 |
| nofastpath | cuda2cpu | 10 | 2269.7 | 1966.7 | 2673.1 | 218.7 |
| noreuse | cpu2cpu | 10 | 978.8 | 770.4 | 1053.3 | 92.2 |
| noreuse | cpu2cuda | 10 | 383.0 | 351.9 | 409.6 | 23.6 |
| noreuse | cuda2cpu | 10 | 860.4 | 729.6 | 891.1 | 61.1 |
| pageable | cpu2cpu | 10 | 5717.9 | 4529.8 | 6223.2 | 519.5 |
| pageable | cpu2cuda | 10 | 2671.2 | 2402.7 | 3084.0 | 225.4 |
| pageable | cuda2cpu | 10 | 3182.8 | 2549.6 | 3470.4 | 246.7 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
