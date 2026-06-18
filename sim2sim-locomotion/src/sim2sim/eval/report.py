"""Cross-simulator comparison report — the deliverable of an eval run.

Produces a markdown table (mean +/- std per metric per simulator) and, if
matplotlib is installed, grouped bar charts. The whole point of sim-to-sim is
seeing these columns side by side: a robust policy yields similar numbers across
backends; large divergence flags overfitting to one simulator's quirks.
"""

from __future__ import annotations

from pathlib import Path

from .runner import SimResult

# Metrics to show, with display labels and whether lower is better.
_METRICS = [
    ("survived", "Survival rate", True),
    ("survival_time", "Survival time (s)", True),
    ("distance", "Distance (m)", True),
    ("lin_vel_tracking_err", "Lin-vel err", False),
    ("ang_vel_tracking_err", "Ang-vel err", False),
    ("mean_torque", "Mean |torque|", False),
    ("cost_of_transport", "Cost of transport", False),
    ("action_rate", "Action rate", False),
]


def to_markdown(results: list[SimResult]) -> str:
    """Render a comparison table. Rows = metrics, columns = simulators."""
    sims = [r.sim for r in results]
    summaries = {r.sim: r.summary for r in results}

    header = "| Metric | " + " | ".join(sims) + " |"
    sep = "|" + "---|" * (len(sims) + 1)
    lines = [header, sep]
    for key, label, higher_better in _METRICS:
        cells = []
        for s in sims:
            mean, std = summaries[s].get(key, (float("nan"), 0.0))
            cells.append(f"{mean:.3f} ± {std:.3f}")
        arrow = "↑" if higher_better else "↓"
        lines.append(f"| {label} {arrow} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def write_report(results: list[SimResult], out_dir: str, make_plots: bool = True) -> dict:
    """Write comparison.md (+ plots if matplotlib available). Returns paths written."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    written: dict[str, str] = {}

    md = "# Sim-to-sim locomotion comparison\n\n" + to_markdown(results) + "\n"
    md_path = out / "comparison.md"
    md_path.write_text(md)
    written["markdown"] = str(md_path)

    if make_plots:
        plot_path = _try_plot(results, out)
        if plot_path:
            written["plot"] = plot_path
    return written


def _try_plot(results: list[SimResult], out: Path) -> str | None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except Exception:
        return None

    sims = [r.sim for r in results]
    summaries = {r.sim: r.summary for r in results}
    keys = [m[0] for m in _METRICS]
    labels = [m[1] for m in _METRICS]

    fig, axes = plt.subplots(2, 4, figsize=(18, 8))
    for ax, key, label in zip(axes.ravel(), keys, labels, strict=True):
        means = [summaries[s].get(key, (np.nan, 0))[0] for s in sims]
        stds = [summaries[s].get(key, (np.nan, 0))[1] for s in sims]
        ax.bar(sims, means, yerr=stds, capsize=4)
        ax.set_title(label, fontsize=10)
        ax.tick_params(axis="x", labelsize=8)
    fig.suptitle("Locomotion policy across simulators (mean ± std)", fontsize=13)
    fig.tight_layout()
    path = out / "comparison.png"
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return str(path)
