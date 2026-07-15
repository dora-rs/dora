# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 14:11:56 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7064.2 | 6407.4 | 7319.8 | 263.1 |
| full | cpu2cuda | 10 | 4562.8 | 4258.1 | 4831.0 | 167.2 |
| full | cuda2cpu | 10 | 7910.9 | 7874.2 | 8055.6 | 60.4 |
| nofastpath | cpu2cpu | 10 | 6794.7 | 6443.1 | 7010.8 | 150.3 |
| nofastpath | cpu2cuda | 10 | 4194.2 | 4056.4 | 4522.7 | 149.3 |
| nofastpath | cuda2cpu | 10 | 7576.9 | 7258.6 | 7781.6 | 146.9 |
| noreuse | cpu2cpu | 10 | 1633.1 | 1601.0 | 1693.5 | 31.5 |
| noreuse | cpu2cuda | 10 | 1222.0 | 1114.8 | 1250.3 | 38.5 |
| noreuse | cuda2cpu | 10 | 1903.8 | 1856.1 | 1929.6 | 20.4 |
| pageable | cpu2cpu | 10 | 7107.7 | 6877.2 | 7369.7 | 148.7 |
| pageable | cpu2cuda | 10 | 4063.1 | 3937.5 | 4185.4 | 76.3 |
| pageable | cuda2cpu | 10 | 7810.0 | 7470.9 | 7979.0 | 152.0 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
