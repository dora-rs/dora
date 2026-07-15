# HeteroPool Ablation Experiment Report

**Generated:** 2026-07-02 13:39:04 UTC

**Total runs:** 120

## Results Summary

| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |
|------|----------|---------|--------------|-----|-----|--------|
| full | cpu2cpu | 10 | 7387.8 | 6570.8 | 7845.5 | 368.8 |
| full | cpu2cuda | 10 | 3935.4 | 3569.0 | 4179.8 | 213.2 |
| full | cuda2cpu | 10 | 7557.8 | 7231.7 | 7717.1 | 133.5 |
| nofastpath | cpu2cpu | 10 | 6599.8 | 6019.4 | 6883.8 | 258.6 |
| nofastpath | cpu2cuda | 10 | 3711.7 | 3602.2 | 3999.7 | 157.0 |
| nofastpath | cuda2cpu | 10 | 7277.3 | 7034.5 | 7435.3 | 126.1 |
| noreuse | cpu2cpu | 10 | 1579.7 | 1461.8 | 1611.0 | 45.6 |
| noreuse | cpu2cuda | 10 | 1156.5 | 1069.3 | 1181.1 | 36.9 |
| noreuse | cuda2cpu | 10 | 1882.1 | 1856.1 | 1895.4 | 11.1 |
| pageable | cpu2cpu | 10 | 7183.6 | 6735.5 | 7429.0 | 214.5 |
| pageable | cpu2cuda | 10 | 4059.8 | 3835.3 | 4221.7 | 97.6 |
| pageable | cuda2cpu | 10 | 7584.5 | 7506.1 | 7699.5 | 63.9 |

## Data Files

- [raw_data.csv](raw_data.csv) — per-run throughput values
- [summary.csv](summary.csv) — per-group statistics
- [logs/](logs/) — full stdout/stderr for each run
