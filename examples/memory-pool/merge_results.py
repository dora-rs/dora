#!/usr/bin/env python3
"""Merge old ablation results (512KB–40MB) with the new rerun (80MB, 160MB).

Usage:
  python3 merge_results.py <old_dir> <new_dir> <merged_dir>
"""

import csv
import shutil
import sys
from pathlib import Path
from statistics import median, stdev
from collections import defaultdict


def merge(old_dir: Path, new_dir: Path, merged_dir: Path) -> None:
    merged_dir.mkdir(parents=True, exist_ok=True)

    # Collect size dirs from both sources
    size_dirs: dict[str, Path] = {}  # label → source path

    for src in (old_dir, new_dir):
        for d in src.iterdir():
            if d.is_dir() and (d / "raw_data.csv").exists():
                label = d.name
                if label not in size_dirs:
                    size_dirs[label] = d
                    print(f"  Using {label} from {src.name}")

    # Copy each size's results to merged dir
    for label, src_path in sorted(size_dirs.items()):
        dst_path = merged_dir / label
        if dst_path.exists():
            shutil.rmtree(dst_path)
        shutil.copytree(src_path, dst_path)
        print(f"  Copied {label}")

    # ------------------------------------------------------------------
    # Build aggregate summary_all.csv
    # ------------------------------------------------------------------
    all_results: dict[str, list[dict]] = defaultdict(list)

    for label in sorted(size_dirs.keys()):
        raw_csv = merged_dir / label / "raw_data.csv"
        if not raw_csv.exists():
            continue
        with open(raw_csv, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if row["status"] == "SUCCESS" and row["throughput_mbs"]:
                    all_results[label].append({
                        "mode": row["mode"],
                        "scenario": row["scenario"],
                        "throughput_mbs": float(row["throughput_mbs"]),
                    })

    summary_all = merged_dir / "summary_all.csv"
    with open(summary_all, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "size", "mode", "scenario", "success", "failed",
            "median_mbs", "min_mbs", "max_mbs", "stddev_mbs",
        ])
        for size_label in sorted(all_results.keys()):
            results = all_results[size_label]
            groups: dict[tuple[str, str], list[float]] = defaultdict(list)
            for r in results:
                groups[(r["mode"], r["scenario"])].append(r["throughput_mbs"])
            for (mode, scenario), values in sorted(groups.items()):
                writer.writerow([
                    size_label, mode, scenario, len(values), 0,
                    f"{median(values):.1f}", f"{min(values):.1f}",
                    f"{max(values):.1f}",
                    f"{stdev(values):.1f}" if len(values) >= 2 else "N/A",
                ])
    print(f"\n  Aggregate summary: {summary_all}")

    # ------------------------------------------------------------------
    # Build pivot report_all.md
    # ------------------------------------------------------------------
    report_all = merged_dir / "report_all.md"
    with open(report_all, "w") as f:
        from datetime import datetime, timezone
        f.write("# HeteroPool Ablation Experiment Report (All 9 Sizes)\n\n")
        f.write(f"**Generated:** {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')}\n\n")
        f.write(f"**Sizes:** {', '.join(sorted(all_results.keys()))}\n\n")

        scenarios = sorted({s for lst in all_results.values() for r in lst for s in [r["scenario"]]})
        modes = sorted({r["mode"] for lst in all_results.values() for r in lst})

        for scenario in scenarios:
            f.write(f"## Scenario: {scenario}\n\n")
            f.write("| Size |" + "".join(f" {m} |" for m in modes) + "\n")
            f.write("|------|" + "|".join(["------" for _ in modes]) + "|\n")

            for size_label in sorted(all_results.keys()):
                results = all_results[size_label]
                groups: dict[str, list[float]] = defaultdict(list)
                for r in results:
                    if r["scenario"] == scenario:
                        groups[r["mode"]].append(r["throughput_mbs"])
                f.write(f"| {size_label} |")
                for m in modes:
                    values = groups.get(m, [])
                    if values:
                        f.write(f" {median(values):.1f} |")
                    else:
                        f.write(" N/A |")
                f.write("\n")
            f.write("\n")
    print(f"  Pivot report:    {report_all}")

    # ------------------------------------------------------------------
    # Print final pivot table to stdout
    # ------------------------------------------------------------------
    print()
    for scenario in scenarios:
        print(f"## {scenario}")
        header = f"{'Size':<10}" + "".join(f"{m:>14}" for m in modes)
        print(header)
        print("-" * len(header))
        for size_label in sorted(all_results.keys(), key=_size_sort_key):
            results = all_results[size_label]
            groups: dict[str, list[float]] = defaultdict(list)
            for r in results:
                if r["scenario"] == scenario:
                    groups[r["mode"]].append(r["throughput_mbs"])
            row = f"{size_label:<10}"
            for m in modes:
                values = groups.get(m, [])
                if values:
                    row += f"{median(values):>14.1f}"
                else:
                    row += f"{'N/A':>14}"
            print(row)
        print()


def _size_sort_key(label: str) -> float:
    """Extract numeric size for sorting: '512KB' → 0.5, '80MB' → 80."""
    label = label.upper()
    if label.endswith("KB"):
        return float(label[:-2]) / 1024
    elif label.endswith("MB"):
        return float(label[:-2])
    elif label.endswith("GB"):
        return float(label[:-2]) * 1024
    return float(label)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <old_dir> <new_dir> <merged_dir>")
        sys.exit(1)
    merge(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
