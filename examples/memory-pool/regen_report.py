#!/usr/bin/env python3
"""Regenerate report_all.md organised by ablation experiment (3 experiments).

Experiment 1 — 页锁内存消融 (cpu2cuda only):
  auto vs pinned vs pageable (write_memory_pool mode parameter)

Experiment 2 — 快速读取消融 (all scenarios):
  fast-path (if_fast=true) vs no-fastpath (if_fast=false)

Experiment 3 — 池化复用消融 (all scenarios):
  pool-reuse vs no-reuse (fresh pool per frame)
"""

import csv
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from statistics import median

RESULTS = Path("/home/tcr/PyCharmMiscProject/dora/examples/memory-pool/results/ablation_full")

# Old→new mode name mapping (historical data uses "full" for "auto")
MODE_RENAME = {"full": "auto"}


def size_sort_key(label: str) -> float:
    label = label.upper()
    if label.endswith("KB"):
        return float(label[:-2]) / 1024
    elif label.endswith("MB"):
        return float(label[:-2])
    elif label.endswith("GB"):
        return float(label[:-2]) * 1024
    return float(label)


# Collect all data
# data[mode][scenario][size] = [throughput values]
data: dict[str, dict[str, dict[str, list[float]]]] = defaultdict(
    lambda: defaultdict(lambda: defaultdict(list))
)

for size_dir in sorted(RESULTS.iterdir()):
    if not size_dir.is_dir():
        continue
    size_label = size_dir.name
    raw_csv = size_dir / "raw_data.csv"
    if not raw_csv.exists():
        continue
    with open(raw_csv, newline="") as f:
        for row in csv.DictReader(f):
            if row["status"] == "SUCCESS" and row["throughput_mbs"]:
                mode = MODE_RENAME.get(row["mode"], row["mode"])
                data[mode][row["scenario"]][size_label].append(
                    float(row["throughput_mbs"])
                )

sizes_sorted = sorted(
    {s for m in data.values() for sc in m.values() for s in sc},
    key=size_sort_key,
)


def _cell(mode: str, scenario: str, size: str) -> str:
    values = data.get(mode, {}).get(scenario, {}).get(size, [])
    if values:
        return f"{median(values):.1f}"
    return "N/A"


def _delta_str(baseline_mode: str, exp_mode: str, scenario: str, size: str) -> str:
    """Return overhead percentage like '+53%' or '−23%' or 'N/A'."""
    b_vals = data.get(baseline_mode, {}).get(scenario, {}).get(size, [])
    e_vals = data.get(exp_mode, {}).get(scenario, {}).get(size, [])
    if not b_vals or not e_vals:
        return "N/A"
    b = median(b_vals)
    e = median(e_vals)
    if b == 0:
        return "N/A"
    pct = (b - e) / b * 100
    sign = "+" if pct >= 0 else ""
    return f"{sign}{pct:.0f}%"


# ---------------------------------------------------------------------------
# Build report_all.md
# ---------------------------------------------------------------------------
report = RESULTS / "report_all.md"
with open(report, "w") as f:
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    f.write("# HeteroPool Memory-Pool Ablation Experiment Report\n\n")
    f.write(f"**Generated:** {ts}\n\n")
    f.write(f"**Sizes:** {', '.join(sizes_sorted)}\n")
    f.write(f"**Repetitions:** 10 per cell, median MB/s reported\n\n")
    f.write("Three independent ablation dimensions — each disables one ")
    f.write("optimisation while the other two remain at optimal.\n\n")

    # ------------------------------------------------------------------
    # Experiment 1: 页锁内存消融 (cpu2cuda only)
    # ------------------------------------------------------------------
    sc = "cpu2cuda"
    f.write("---\n\n")
    f.write("## 实验一：页锁内存消融\n\n")
    f.write(
        "消融维度：`write_memory_pool(memory_pool_id, tensor_info, *, mode=...)`\n\n"
    )
    f.write("- **auto** — 根据 25 MiB 阈值自动选择 pinned / pageable（生产默认）\n")
    f.write("- **pinned** — 始终 `cudaHostRegister` + pinned DMA\n")
    f.write("- **pageable** — 跳过 `cudaHostRegister`，使用 pageable `cudaMemcpy`\n\n")
    f.write("*仅测试 `cpu2cuda` 场景（pinned vs pageable 在 cpu2cpu / cuda2cpu 上无 DMA 路径差异）。*\n\n")
    f.write("| Size | auto | pinned | pageable | auto→pageable overhead |\n")
    f.write("|------|------|--------|----------|------------------------|\n")
    for size in sizes_sorted:
        a = _cell("auto", sc, size)
        pi = _cell("pinned", sc, size)
        pa = _cell("pageable", sc, size)
        d = _delta_str("auto", "pageable", sc, size)
        f.write(f"| {size} | {a} | {pi} | {pa} | {d} |\n")
    f.write("\n")

    # ------------------------------------------------------------------
    # Experiment 2: 快速读取消融 (all scenarios)
    # ------------------------------------------------------------------
    f.write("---\n\n")
    f.write("## 实验二：快速读取消融\n\n")
    f.write(
        "消融维度：`read_memory_pool(memory_pool_id, *, if_fast=...)`\n\n"
    )
    f.write("- **fast-path** (`if_fast=true`) — 仅首帧读取元数据 + 复用 tensor 视图\n")
    f.write("- **no-fastpath** (`if_fast=false`) — 每帧查询 daemon + 重建 tensor 视图\n\n")
    for sc in ("cpu2cpu", "cpu2cuda", "cuda2cpu"):
        f.write(f"### {sc}\n\n")
        f.write("| Size | fast-path (auto) | no-fastpath | overhead |\n")
        f.write("|------|-------------------|-------------|----------|\n")
        for size in sizes_sorted:
            fast = _cell("auto", sc, size)
            slow = _cell("nofastpath", sc, size)
            d = _delta_str("auto", "nofastpath", sc, size)
            f.write(f"| {size} | {fast} | {slow} | {d} |\n")
        f.write("\n")

    # ------------------------------------------------------------------
    # Experiment 3: 池化复用消融 (all scenarios)
    # ------------------------------------------------------------------
    f.write("---\n\n")
    f.write("## 实验三：池化复用消融\n\n")
    f.write("消融维度：pool 复用 vs 每帧注册新池\n\n")
    f.write("- **pool-reuse** — 首帧 `register`，后续 `write` 复用；末帧 `free`\n")
    f.write("- **no-reuse** — 每帧 `register` → `write` → `read` → `free`\n\n")
    for sc in ("cpu2cpu", "cpu2cuda", "cuda2cpu"):
        f.write(f"### {sc}\n\n")
        f.write("| Size | pool-reuse (auto) | no-reuse | overhead |\n")
        f.write("|------|-------------------|----------|----------|\n")
        for size in sizes_sorted:
            reuse = _cell("auto", sc, size)
            noreuse = _cell("noreuse", sc, size)
            d = _delta_str("auto", "noreuse", sc, size)
            f.write(f"| {size} | {reuse} | {noreuse} | {d} |\n")
        f.write("\n")

print(f"Regenerated: {report}")

# ---------------------------------------------------------------------------
# Print to stdout
# ---------------------------------------------------------------------------
print()
print("=" * 70)
print("实验一：页锁内存消融 (cpu2cuda only)")
print("=" * 70)
header = f"{'Size':<10}{'auto':>12}{'pinned':>12}{'pageable':>12}{'overhead':>12}"
print(header)
print("-" * len(header))
for size in sizes_sorted:
    a = _cell("auto", "cpu2cuda", size)
    pi = _cell("pinned", "cpu2cuda", size)
    pa = _cell("pageable", "cpu2cuda", size)
    d = _delta_str("auto", "pageable", "cpu2cuda", size)
    print(f"{size:<10}{a:>12}{pi:>12}{pa:>12}{d:>12}")

for exp_name, scenarios, baseline, treatment, bl_label, tx_label in [
    ("实验二：快速读取消融", ("cpu2cpu", "cpu2cuda", "cuda2cpu"),
     "auto", "nofastpath", "fast-path", "no-fastpath"),
    ("实验三：池化复用消融", ("cpu2cpu", "cpu2cuda", "cuda2cpu"),
     "auto", "noreuse", "pool-reuse", "no-reuse"),
]:
    print()
    print("=" * 70)
    print(exp_name)
    print("=" * 70)
    for sc in scenarios:
        print(f"\n  {sc}")
        header = f"  {'Size':<10}{bl_label:>14}{tx_label:>14}{'overhead':>14}"
        print(header)
        print("  " + "-" * (len(header) - 2))
        for size in sizes_sorted:
            bl = _cell(baseline, sc, size)
            tx = _cell(treatment, sc, size)
            d = _delta_str(baseline, treatment, sc, size)
            print(f"  {size:<10}{bl:>14}{tx:>14}{d:>14}")
