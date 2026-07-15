#!/usr/bin/env python3
"""Merge Experiment 2 (auto+nofastpath) into ablation_full, keeping other modes."""
import csv, sys
from pathlib import Path

EXP2 = Path("/home/tcr/PyCharmMiscProject/dora/examples/memory-pool/results/ablation_exp2")
FULL = Path("/home/tcr/PyCharmMiscProject/dora/examples/memory-pool/results/ablation_full")

MODES_TO_REPLACE = {"auto", "nofastpath"}

for size_dir in sorted(EXP2.iterdir()):
    if not size_dir.is_dir():
        continue
    size = size_dir.name
    old_csv = FULL / size / "raw_data.csv"
    new_csv = size_dir / "raw_data.csv"

    if not old_csv.exists():
        print(f"  {size}: no old data, skipping")
        continue
    if not new_csv.exists():
        print(f"  {size}: no new data, skipping")
        continue

    # Read old data, filter out modes to replace
    old_rows = []
    with open(old_csv, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["mode"] not in MODES_TO_REPLACE:
                old_rows.append(row)

    # Read new data
    new_rows = []
    with open(new_csv, newline="") as f:
        new_rows = list(csv.DictReader(f))

    # Write merged CSV
    merged_rows = old_rows + new_rows
    fieldnames = ["mode", "scenario", "run", "throughput_mbs", "status", "duration_s", "notes"]
    with open(old_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged_rows)

    old_count = len(old_rows)
    new_count = len(new_rows)
    print(f"  {size}: kept {old_count} old rows, added {new_count} new rows (total {old_count + new_count})")

print("Merge complete. Regenerate report with: python3 regen_report.py")
