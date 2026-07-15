# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-05 07:28:17 UTC

**Total runs:** 60

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| auto | cpu2cpu | 10 | 7447.3 | 7365.6 | 7517.7 | 53.3 |
| auto | cpu2cuda | 10 | 5356.3 | 5199.7 | 5478.7 | 75.5 |
| auto | cuda2cpu | 10 | 8218.5 | 8011.8 | 8281.8 | 78.2 |
| nofastpath | cpu2cpu | 10 | 6987.9 | 6870.9 | 7217.5 | 108.7 |
| nofastpath | cpu2cuda | 10 | 5165.0 | 5005.2 | 5289.4 | 91.2 |
| nofastpath | cuda2cpu | 10 | 7855.9 | 7751.7 | 7921.4 | 56.0 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
