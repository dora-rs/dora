# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-03 08:51:11 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7266.1 | 6620.5 | 7372.6 | 246.6 |
| full | cpu2cuda | 10 | 5792.6 | 5658.2 | 5878.6 | 63.0 |
| full | cuda2cpu | 10 | 8489.6 | 8357.3 | 8556.8 | 62.9 |
| nofastpath | cpu2cpu | 10 | 6743.1 | 6626.0 | 6810.8 | 59.9 |
| nofastpath | cpu2cuda | 10 | 5467.5 | 5421.4 | 5590.0 | 62.5 |
| nofastpath | cuda2cpu | 10 | 7832.8 | 7696.1 | 7914.6 | 61.5 |
| noreuse | cpu2cpu | 10 | 1567.5 | 1457.5 | 1621.9 | 56.0 |
| noreuse | cpu2cuda | 10 | 1237.3 | 1208.4 | 1250.2 | 14.2 |
| noreuse | cuda2cpu | 10 | 1719.3 | 1507.0 | 1741.8 | 88.0 |
| pageable | cpu2cpu | 10 | 7152.1 | 7066.2 | 7202.1 | 48.3 |
| pageable | cpu2cuda | 10 | 4222.6 | 4148.8 | 4253.5 | 31.0 |
| pageable | cuda2cpu | 10 | 8302.9 | 8206.9 | 8355.3 | 43.0 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
