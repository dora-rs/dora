#!/usr/bin/env python3
"""HeteroPool memory-pool ablation experiment runner.

Drives 4 experiment modes × 3 device-pair scenarios × N repetitions,
collecting throughput metrics and producing statistical summaries.

Usage:
  python3 run_ablation.py                  # full matrix, 10 reps
  python3 run_ablation.py -n 2             # quick smoke test
  python3 run_ablation.py --skip-gpu       # CPU-only scenarios
  python3 run_ablation.py --dry-run        # print plan, don't run
  python3 run_ablation.py --rebuild        # rebuild before running
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from statistics import median, stdev
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MEMORY_POOL_DIR = Path(__file__).resolve().parent
DORA_BIN = shutil.which("dora") or str(PROJECT_ROOT / "target" / "release" / "dora")

# Throughput regex — matches receiver.py output line
THROUGHPUT_RE = re.compile(
    r"Average transfer throughput:\s+([0-9]+(?:\.[0-9]+)?)\s*MB/s"
)

# Experiment modes: label → env vars dict.
# Three independent ablation dimensions, each with its own baseline:
#   实验一 页锁内存 (only cpu2cuda):
#     auto     — mode="auto"  auto-select pinned/pageable (production baseline)
#     pinned   — mode="pinned"  always cudaHostRegister + DMA
#     pageable — mode="pageable"  skip cudaHostRegister, pageable cudaMemcpy
#   实验二 快速读取 (all scenarios):
#     auto / nofastpath — if_fast=true vs if_fast=false on read_memory_pool
#   实验三 池化复用 (all scenarios):
#     auto / noreuse — pool reuse vs fresh pool per frame
MODES: dict[str, dict[str, str]] = {
    "auto":       {},                                   # production default
    "pinned":     {"HETEROPOOL_MODE": "pinned"},         # 实验一
    "pageable":   {"HETEROPOOL_MODE": "pageable"},       # 实验一
    "nofastpath": {"HETEROPOOL_NO_FASTPATH": "1"},       # 实验二
    "noreuse":    {"HETEROPOOL_NO_REUSE": "1"},          # 实验三
}

# Per-mode scenario filter: None = all scenarios, list = only those scenarios.
# 实验一 (页锁内存) 仅在 cpu2cuda 上有差异, cpu2cpu/cuda2cpu 无 DMA 路径.
MODE_SCENARIOS: dict[str, list[str] | None] = {
    "pinned":   ["cpu2cuda"],
    "pageable": ["cpu2cuda"],
}

# Scenarios: label → (yaml_filename, needs_gpu)
SCENARIOS: dict[str, tuple[str, bool]] = {
    "cpu2cpu": ("cpu2cpu.yml", False),
    "cpu2cuda": ("cpu2cuda.yml", True),
    "cuda2cpu": ("cuda2cpu.yml", True),
}

DEFAULT_REPETITIONS = 10
DEFAULT_TIMEOUT = 120  # seconds per run

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class RunResult:
    mode: str
    scenario: str
    run: int
    throughput_mbs: Optional[float] = None
    status: str = "SUCCESS"  # SUCCESS | FAILED | TIMEOUT | SKIPPED
    duration_s: float = 0.0
    notes: str = ""


@dataclass
class ExperimentPlan:
    modes: list[str] = field(default_factory=list)
    scenarios: list[str] = field(default_factory=list)
    repetitions: int = DEFAULT_REPETITIONS
    total_runs: int = 0
    gpu_available: bool = False
    build_fresh: bool = True


# ---------------------------------------------------------------------------
# AblationRunner
# ---------------------------------------------------------------------------


class AblationRunner:
    def __init__(
        self,
        repetitions: int = DEFAULT_REPETITIONS,
        timeout: int = DEFAULT_TIMEOUT,
        skip_gpu: bool = False,
        skip_build_check: bool = False,
        rebuild: bool = False,
        dry_run: bool = False,
        output_dir: Optional[str] = None,
        verbose: bool = False,
        tensor_bytes: int = 0,
        tensor_sizes: Optional[list[int]] = None,
    ):
        self.repetitions = repetitions
        self.timeout = timeout
        self.skip_gpu = skip_gpu
        self.skip_build_check = skip_build_check
        self.rebuild = rebuild
        self.dry_run = dry_run
        self.verbose = verbose
        self.tensor_bytes = tensor_bytes
        # Multi-size support: a list of byte sizes to iterate over.
        # When set, each size gets its own sub-directory and results.
        self.tensor_sizes = tensor_sizes
        self.results: list[RunResult] = []
        self.start_time: float = 0.0
        self.output_dir: Optional[Path] = None
        self._user_output_dir = output_dir
        self._interrupted = False

        # Install SIGINT handler for graceful shutdown
        signal.signal(signal.SIGINT, self._on_sigint)

    # ------------------------------------------------------------------
    # Pre-flight
    # ------------------------------------------------------------------

    def detect_gpu(self) -> bool:
        """Check whether CUDA is available via PyTorch."""
        try:
            import torch  # noqa: F401

            return torch.cuda.is_available()
        except ImportError:
            return False

    def check_build(self) -> bool:
        """Return True if the Python extension .so is newer than lib.rs."""
        lib_rs = PROJECT_ROOT / "apis" / "python" / "node" / "src" / "lib.rs"
        so_files = list(
            (PROJECT_ROOT / "target" / "release").glob("libdora_node_api_python*.so")
        )
        if not so_files:
            return False
        newest_so = max(f.stat().st_mtime for f in so_files)
        return newest_so >= lib_rs.stat().st_mtime

    def rebuild_project(self) -> bool:
        """Run cargo build for the Python extension and CLI."""
        print("=== Rebuilding dora (Python extension + CLI) ===")
        rc = subprocess.run(
            ["cargo", "build", "--release", "-p", "dora-node-api-python", "-p", "dora-cli"],
            cwd=PROJECT_ROOT,
        ).returncode
        if rc != 0:
            print("FATAL: cargo build failed")
            return False
        print("  Build: OK\n")
        return True

    def cleanup_stale(self) -> None:
        """Kill orphaned dora processes and clean stale state.

        Stale lock files in out/ and .dora/python-envs/ survive crashes and
        can cause the next ``dora run`` to block waiting for a lock held by a
        process that no longer exists.  Removing them between runs prevents
        the experiment from hanging.

        Uses SIGKILL (-9) so processes die immediately; the following
        2 s sleep gives the kernel time to release ports and file locks
        before the next ``dora run`` starts.
        """
        # Kill the full process tree: daemon, coordinator, and Python node
        # processes that the daemon spawned (sender.py, receiver.py).
        # When the daemon is killed with SIGKILL its children become orphans
        # adopted by init — they keep running and hold mmap refs to /dev/shm
        # pools, so the kernel won't free the space until they exit.  Kill
        # the nodes first so their shmem mappings are released before we
        # unlink the files.
        for pattern in ("sender.py", "receiver.py"):
            subprocess.run(
                ["pkill", "-9", "-f", f"memory-pool/{pattern}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        for name in ("dora-daemon", "dora-coordinator"):
            subprocess.run(
                ["pkill", "-9", "-f", name],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        time.sleep(2)  # let OS release ports, shmem mappings, and file locks
        # Clean leftover shared-memory files
        for pattern in ("dora_pool_*", "dora_shm_*"):
            for f in Path("/dev/shm").glob(pattern):
                try:
                    f.unlink()
                except OSError:
                    pass
        # Clean stale dora session lock files (survive crashes / SIGKILL)
        for lock in MEMORY_POOL_DIR.glob("out/*.lock"):
            try:
                lock.unlink()
            except OSError:
                pass
        # Clean stale per-node Python-env locks
        for lock in MEMORY_POOL_DIR.glob(".dora/python-envs/*/.lock"):
            try:
                lock.unlink()
            except OSError:
                pass
        time.sleep(0.5)

    def gpu_memory_used(self) -> Optional[int]:
        """Return GPU memory used in MiB, or None if nvidia-smi unavailable."""
        try:
            out = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.used",
                    "--format=csv,noheader,nounits",
                ],
                text=True,
                timeout=5,
            )
            return int(out.strip().split("\n")[0])
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Single-run execution
    # ------------------------------------------------------------------

    def run_single(
        self, mode: str, scenario: str, run_num: int, log_file: Path,
        tensor_bytes: int = 0,
    ) -> RunResult:
        """Execute one dora run and return the result."""
        scenario_yaml, needs_gpu = SCENARIOS[scenario]
        yaml_path = MEMORY_POOL_DIR / scenario_yaml
        env_vars = MODES[mode]

        result = RunResult(mode=mode, scenario=scenario, run=run_num)
        t0 = time.perf_counter()

        # Build environment
        run_env = os.environ.copy()
        run_env.update(env_vars)
        tb = tensor_bytes or self.tensor_bytes
        if tb:
            run_env["TENSOR_BYTES"] = str(tb)

        stop_after_s = max(15, self.timeout - 10)
        cmd = [DORA_BIN, "run", str(yaml_path), "--stop-after", f"{stop_after_s}s"]

        if self.dry_run:
            env_str = " ".join(f"{k}={v}" for k, v in env_vars.items()) if env_vars else "(none)"
            print(
                f"  [{mode}] {scenario} run {run_num}/{self.repetitions}  "
                f"env=({env_str})  cmd={' '.join(cmd)}"
            )
            result.status = "SKIPPED"
            result.notes = "dry-run"
            return result

        try:
            proc = subprocess.run(
                cmd,
                cwd=MEMORY_POOL_DIR,
                env=run_env,
                capture_output=True,
                text=True,
                timeout=self.timeout,
            )

            result.duration_s = time.perf_counter() - t0

            # Write logs
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with open(log_file, "w") as f:
                f.write(f"# command: {' '.join(cmd)}\n")
                f.write(f"# env: {env_vars}\n")
                f.write(f"# exit_code: {proc.returncode}\n")
                f.write(f"# duration_s: {result.duration_s:.1f}\n")
                f.write("# stdout:\n")
                f.write(proc.stdout)
                if proc.stderr:
                    f.write("\n# stderr:\n")
                    f.write(proc.stderr)

            # Check for Python traceback in output (indicates node crash)
            if "Traceback (most recent call last)" in proc.stdout or \
               "Traceback (most recent call last)" in proc.stderr:
                result.status = "FAILED"
                result.notes = "Python traceback detected in output"
                return result

            if proc.returncode != 0:
                result.status = "FAILED"
                result.notes = f"exit code {proc.returncode}"
                return result

            # Parse throughput
            match = THROUGHPUT_RE.search(proc.stdout)
            if match:
                result.throughput_mbs = float(match.group(1))
            else:
                result.status = "FAILED"
                result.notes = "throughput line not found in output"

        except subprocess.TimeoutExpired:
            result.duration_s = time.perf_counter() - t0
            result.status = "TIMEOUT"
            result.notes = f"timed out after {self.timeout}s"
            # Clean up fully before returning — the subprocess timeout only
            # kills the dora CLI parent; daemon, coordinator, and Python
            # node children become orphans and leak /dev/shm space.
            # cleanup_stale kills the full process tree and removes stale
            # shmem and lock files so the next run starts from a clean state.
            self.cleanup_stale()
            # Write whatever we captured
            log_file.parent.mkdir(parents=True, exist_ok=True)
            with open(log_file, "w") as f:
                f.write(f"# TIMEOUT after {self.timeout}s\n")

        return result

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run_all(self, plan: ExperimentPlan) -> bool:
        """Execute the full experiment matrix. Returns True if all runs completed."""
        if plan.total_runs == 0:
            print("No experiments to run (all scenarios skipped?).")
            return False

        # Determine the list of byte-sizes to iterate.
        # When --tensor-sizes is given, each size runs the full mode×scenario matrix
        # in its own sub-directory under the timestamped output dir.
        byte_sizes: list[tuple[str, int]]  # (label, bytes)
        if self.tensor_sizes:
            def _fmt_mib(b: int) -> str:
                if b >= 1024 * 1024:
                    return f"{b // (1024*1024)}MB"
                elif b >= 1024:
                    return f"{b // 1024}KB"
                return f"{b}B"
            byte_sizes = [(_fmt_mib(sz), sz) for sz in self.tensor_sizes]
        elif self.tensor_bytes:
            byte_sizes = [(str(self.tensor_bytes), self.tensor_bytes)]
        else:
            byte_sizes = [("default", 0)]

        # Set up top-level output directory
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%SZ")
        self.output_dir = (
            Path(self._user_output_dir)
            if self._user_output_dir
            else MEMORY_POOL_DIR / "results" / f"ablation_{ts}"
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        print(f"Results directory: {self.output_dir}")

        n_sizes = len(byte_sizes)
        if n_sizes > 1:
            print(f"Sizes: {', '.join(lbl for lbl, _ in byte_sizes)}")
        print()

        if self.dry_run:
            print("=== DRY RUN — no experiments will execute ===\n")
            for size_label, size_bytes in byte_sizes:
                if n_sizes > 1:
                    print(f"--- Size: {size_label} ---")
                for mode in plan.modes:
                    allowed = MODE_SCENARIOS.get(mode, None)
                    mode_scenarios = (
                        [s for s in plan.scenarios if s in allowed]
                        if allowed is not None
                        else plan.scenarios
                    )
                    for scenario in mode_scenarios:
                        for r in range(1, self.repetitions + 1):
                            log_name = f"{mode}__{scenario}__run{r:02d}.log"
                            log_path = self.output_dir / "logs" / log_name
                            env_vars = MODES[mode]
                            scenario_yaml, _ = SCENARIOS[scenario]
                            yaml_path = MEMORY_POOL_DIR / scenario_yaml
                            env_str = " ".join(f"{k}={v}" for k, v in env_vars.items()) or "(none)"
                            tb_str = f" TENSOR_BYTES={size_bytes}" if size_bytes else ""
                            cmd = [DORA_BIN, "run", str(yaml_path), "--stop-after", f"{max(15, self.timeout - 10)}s"]
                            print(
                                f"  [{size_label}] [{mode}] {scenario} run {r}/{self.repetitions}  "
                                f"env=({env_str}{tb_str})  cmd={' '.join(cmd)}"
                            )
            print()
            return True

        self.start_time = time.perf_counter()
        all_results: dict[str, list[RunResult]] = {}  # size_label → results
        total_runs = plan.total_runs * n_sizes
        completed = 0
        run_index = 0

        # Keep a stable reference to the top-level output directory so each
        # size's subdirectory is created directly under it (not nested inside
        # the previous size).
        top_output_dir = self.output_dir

        for size_label, size_bytes in byte_sizes:
            size_dir = top_output_dir / size_label
            logs_dir = size_dir / "logs"
            logs_dir.mkdir(parents=True, exist_ok=True)

            if n_sizes > 1:
                print(f"\n{'='*60}")
                print(f"  Size: {size_label}  ({size_bytes} bytes)")
                print(f"{'='*60}")

            size_results: list[RunResult] = []

            for mode in plan.modes:
                # Filter scenarios per mode (e.g. pinned/pageable only cpu2cuda)
                allowed = MODE_SCENARIOS.get(mode, None)
                mode_scenarios = (
                    [s for s in plan.scenarios if s in allowed]
                    if allowed is not None
                    else plan.scenarios
                )
                for scenario in mode_scenarios:
                    _, needs_gpu = SCENARIOS[scenario]
                    for r in range(1, self.repetitions + 1):
                        run_index += 1

                        if self._interrupted:
                            print("\nInterrupted — saving partial results...")
                            all_results[size_label] = size_results
                            self._save_multi_results(all_results, plan)
                            return False

                        # Cleanup stale state before each run (skip very first)
                        if run_index > 1:
                            self.cleanup_stale()

                        # GPU memory check before GPU runs
                        gpu_before = None
                        if needs_gpu and not self.dry_run:
                            gpu_before = self.gpu_memory_used()

                        # Execute
                        log_name = f"{mode}__{scenario}__run{r:02d}.log"
                        log_path = logs_dir / log_name
                        result = self.run_single(mode, scenario, r, log_path,
                                                 tensor_bytes=size_bytes)
                        size_results.append(result)
                        self.results.append(result)  # aggregate for backward compat

                        # GPU memory check after
                        if needs_gpu and not self.dry_run and result.status == "SUCCESS":
                            gpu_after = self.gpu_memory_used()
                            if gpu_before is not None and gpu_after is not None:
                                delta = gpu_after - gpu_before
                                if delta > 100:
                                    print(
                                        f"  WARNING: GPU memory leak {delta} MiB "
                                        f"({gpu_before} → {gpu_after})"
                                    )

                        # Progress
                        completed += 1
                        elapsed = time.perf_counter() - self.start_time
                        eta = (elapsed / completed) * (total_runs - completed) if completed > 0 else 0

                        status_mark = {
                            "SUCCESS": "OK",
                            "FAILED": "FAIL",
                            "TIMEOUT": "TIME",
                            "SKIPPED": "SKIP",
                        }.get(result.status, result.status)

                        detail = ""
                        if result.throughput_mbs is not None:
                            detail = f"  {result.throughput_mbs:.1f} MB/s  {result.duration_s:.1f}s"
                        elif result.notes:
                            detail = f"  {result.notes}"

                        prefix = f"[{size_label}] " if n_sizes > 1 else ""
                        print(
                            f"  [{completed:3d}/{total_runs}] [{status_mark:4s}] "
                            f"{prefix}[{mode}] {scenario} run {r}/{self.repetitions}{detail}"
                        )

            all_results[size_label] = size_results
            # Save per-size results after each size completes
            self.output_dir = size_dir
            self.results = size_results
            self._save_results()
            self._print_summary_table(size_results, f"  [{size_label}]", plan)
            # Restore top-level output dir for next size (or final save)
            self.output_dir = top_output_dir

        # Save aggregate across all sizes
        self.results = [r for lst in all_results.values() for r in lst]
        self._save_multi_results(all_results, plan)

        # Final cleanup
        self.cleanup_stale()

        elapsed_total = time.perf_counter() - self.start_time
        print(f"\n{'='*60}")
        print(f"All {total_runs} runs completed in {elapsed_total:.0f}s ({elapsed_total/3600:.1f}h)")
        return True

    # ------------------------------------------------------------------
    # Results & reporting
    # ------------------------------------------------------------------

    def _save_results(self) -> None:
        """Write raw_data.csv, summary.csv, and report.md."""
        if not self.output_dir or not self.results:
            return

        # raw_data.csv
        raw_csv = self.output_dir / "raw_data.csv"
        with open(raw_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["mode", "scenario", "run", "throughput_mbs", "status", "duration_s", "notes"]
            )
            for r in self.results:
                writer.writerow(
                    [
                        r.mode,
                        r.scenario,
                        r.run,
                        f"{r.throughput_mbs:.1f}" if r.throughput_mbs is not None else "",
                        r.status,
                        f"{r.duration_s:.1f}",
                        r.notes,
                    ]
                )

        # Group results
        groups: dict[tuple[str, str], list[float]] = defaultdict(list)
        for r in self.results:
            if r.status == "SUCCESS" and r.throughput_mbs is not None:
                groups[(r.mode, r.scenario)].append(r.throughput_mbs)

        # summary.csv
        summary_csv = self.output_dir / "summary.csv"
        with open(summary_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["mode", "scenario", "success", "failed", "median_mbs",
                 "min_mbs", "max_mbs", "stddev_mbs"]
            )
            for (mode, scenario), values in sorted(groups.items()):
                n_failed = sum(
                    1
                    for r in self.results
                    if r.mode == mode and r.scenario == scenario and r.status != "SUCCESS"
                )
                if len(values) >= 2:
                    writer.writerow(
                        [
                            mode,
                            scenario,
                            len(values),
                            n_failed,
                            f"{median(values):.1f}",
                            f"{min(values):.1f}",
                            f"{max(values):.1f}",
                            f"{stdev(values):.1f}",
                        ]
                    )
                elif len(values) == 1:
                    writer.writerow(
                        [
                            mode,
                            scenario,
                            1,
                            n_failed,
                            f"{values[0]:.1f}",
                            f"{values[0]:.1f}",
                            f"{values[0]:.1f}",
                            "N/A",
                        ]
                    )
                else:
                    writer.writerow(
                        [mode, scenario, 0, n_failed, "N/A", "N/A", "N/A", "N/A"]
                    )

        # report.md
        report_md = self.output_dir / "report.md"
        with open(report_md, "w") as f:
            f.write("# HeteroPool Ablation Experiment Report\n\n")
            f.write(
                f"**Generated:** {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')}\n\n"
            )
            f.write(f"**Total runs:** {len(self.results)}\n\n")

            f.write("## Results Summary\n\n")
            f.write(
                "| Mode | Scenario | Success | Median (MB/s) | Min | Max | StdDev |\n"
            )
            f.write(
                "|------|----------|---------|--------------|-----|-----|--------|\n"
            )
            for (mode, scenario), values in sorted(groups.items()):
                n_success = len(values)
                n_failed = sum(
                    1
                    for r in self.results
                    if r.mode == mode
                    and r.scenario == scenario
                    and r.status != "SUCCESS"
                )
                if n_success > 0:
                    std_str = f"{stdev(values):.1f}" if n_success >= 2 else "N/A"
                    f.write(
                        f"| {mode} | {scenario} | {n_success} | "
                        f"{median(values):.1f} | {min(values):.1f} | "
                        f"{max(values):.1f} | "
                        f"{std_str} |\n"
                    )
                else:
                    f.write(
                        f"| {mode} | {scenario} | 0/{n_success + n_failed} | "
                        f"N/A | N/A | N/A | N/A |\n"
                    )
            f.write("\n")

            # Failed runs
            failed = [r for r in self.results if r.status not in ("SUCCESS", "SKIPPED")]
            if failed:
                f.write("## Failed Runs\n\n")
                f.write("| Mode | Scenario | Run | Status | Notes |\n")
                f.write("|------|----------|-----|--------|-------|\n")
                for r in failed:
                    f.write(
                        f"| {r.mode} | {r.scenario} | {r.run} | "
                        f"{r.status} | {r.notes} |\n"
                    )
                f.write("\n")

            f.write("## Data Files\n\n")
            f.write("- [raw_data.csv](raw_data.csv) — per-run throughput values\n")
            f.write("- [summary.csv](summary.csv) — per-group statistics\n")
            f.write("- [logs/](logs/) — full stdout/stderr for each run\n")

        print(f"  Results saved to {self.output_dir}")

    def _print_summary(self) -> None:
        """Print a compact summary table to stdout."""
        groups: dict[tuple[str, str], list[float]] = defaultdict(list)
        for r in self.results:
            if r.status == "SUCCESS" and r.throughput_mbs is not None:
                groups[(r.mode, r.scenario)].append(r.throughput_mbs)

        if not groups:
            print("\nNo successful runs — no summary available.")
            return

        print("\n=== Ablation Summary (median MB/s) ===\n")
        scenarios = sorted({s for _, s in groups})
        modes = sorted({m for m, _ in groups})

        # Header
        header = f"{'Mode':<12}" + "".join(f"{s:>14}" for s in scenarios)
        print(header)
        print("-" * len(header))

        for mode in modes:
            row = f"{mode:<12}"
            for scenario in scenarios:
                values = groups.get((mode, scenario), [])
                if values:
                    row += f"{median(values):>14.1f}"
                else:
                    row += f"{'N/A':>14}"
            print(row)

        print()

    # ------------------------------------------------------------------
    # Multi-size report helpers
    # ------------------------------------------------------------------

    def _print_summary_table(self, results: list[RunResult], prefix: str,
                             plan: ExperimentPlan) -> None:
        """Print a compact summary table for one size slice."""
        groups: dict[tuple[str, str], list[float]] = defaultdict(list)
        for r in results:
            if r.status == "SUCCESS" and r.throughput_mbs is not None:
                groups[(r.mode, r.scenario)].append(r.throughput_mbs)

        if not groups:
            return

        print(f"\n{prefix} Ablation Summary (median MB/s)\n")
        scenarios = sorted({s for _, s in groups})
        modes = sorted({m for m, _ in groups})

        header = f"  {'Mode':<12}" + "".join(f"{s:>14}" for s in scenarios)
        print(header)
        print("  " + "-" * (len(header) - 2))

        for mode in modes:
            row = f"  {mode:<12}"
            for scenario in scenarios:
                values = groups.get((mode, scenario), [])
                if values:
                    row += f"{median(values):>14.1f}"
                else:
                    row += f"{'N/A':>14}"
            print(row)
        print()

    def _save_multi_results(self, all_results: dict[str, list[RunResult]],
                            plan: ExperimentPlan) -> None:
        """Save per-size CSVs and an aggregate summary_all.csv."""
        # Per-size CSVs were already saved during run_all.
        # Build aggregate summary across all sizes.
        top = self.output_dir

        # summary_all.csv — median per (size, mode, scenario)
        summary_all = top / "summary_all.csv"
        with open(summary_all, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["size", "mode", "scenario", "success", "failed",
                 "median_mbs", "min_mbs", "max_mbs", "stddev_mbs"]
            )
            for size_label in sorted(all_results.keys()):
                results = all_results[size_label]
                groups: dict[tuple[str, str], list[float]] = defaultdict(list)
                for r in results:
                    if r.status == "SUCCESS" and r.throughput_mbs is not None:
                        groups[(r.mode, r.scenario)].append(r.throughput_mbs)
                for (mode, scenario), values in sorted(groups.items()):
                    n_failed = sum(
                        1 for r in results
                        if r.mode == mode and r.scenario == scenario
                        and r.status != "SUCCESS"
                    )
                    if len(values) >= 2:
                        writer.writerow([
                            size_label, mode, scenario, len(values), n_failed,
                            f"{median(values):.1f}", f"{min(values):.1f}",
                            f"{max(values):.1f}", f"{stdev(values):.1f}",
                        ])
                    elif len(values) == 1:
                        writer.writerow([
                            size_label, mode, scenario, 1, n_failed,
                            f"{values[0]:.1f}", f"{values[0]:.1f}",
                            f"{values[0]:.1f}", "N/A",
                        ])
        print(f"  Aggregate summary: {summary_all}")

        # report_all.md — pivot table
        report_all = top / "report_all.md"
        with open(report_all, "w") as f:
            f.write("# HeteroPool Ablation Experiment Report (Multi-Size)\n\n")
            f.write(f"**Generated:** {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S UTC')}\n\n")
            f.write(f"**Sizes:** {', '.join(sorted(all_results.keys()))}\n\n")

            for scenario in sorted({s for s, _ in SCENARIOS.values()}):
                f.write(f"## Scenario: {scenario}\n\n")
                f.write("| Size |")
                modes_sorted = sorted({m for lst in all_results.values() for r in lst if r.scenario == scenario for m in [r.mode]})
                for m in modes_sorted:
                    f.write(f" {m} |")
                f.write("\n|------|" + "|".join(["-" * (len(m) + 2) for m in modes_sorted]) + "|\n")

                for size_label in sorted(all_results.keys()):
                    results = all_results[size_label]
                    groups: dict[str, list[float]] = defaultdict(list)
                    for r in results:
                        if r.status == "SUCCESS" and r.throughput_mbs is not None and r.scenario == scenario:
                            groups[r.mode].append(r.throughput_mbs)
                    f.write(f"| {size_label} |")
                    for m in modes_sorted:
                        values = groups.get(m, [])
                        if values:
                            f.write(f" {median(values):.1f} |")
                        else:
                            f.write(" N/A |")
                    f.write("\n")
                f.write("\n")

        print(f"  Pivot report:    {report_all}")

    # ------------------------------------------------------------------
    # Signal handling
    # ------------------------------------------------------------------

    def _on_sigint(self, signum, frame):
        self._interrupted = True
        print("\nInterrupt received — finishing current run, then saving...")


# ---------------------------------------------------------------------------
# Plan printer (used by --dry-run and startup)
# ---------------------------------------------------------------------------


def print_plan(plan: ExperimentPlan) -> None:
    print("=== HeteroPool Ablation Experiment Plan ===\n")
    print(f"  Modes:       {', '.join(plan.modes)}")
    print(f"  Scenarios:   {', '.join(plan.scenarios)}")
    print(f"  Repetitions: {plan.repetitions}")
    print(f"  Total runs:  {plan.total_runs}")
    print(f"  GPU:         {'available' if plan.gpu_available else 'NOT available'}")
    print(f"  Build:       {'fresh' if plan.build_fresh else 'STALE — consider --rebuild'}")
    print()

    est_seconds = plan.total_runs * 15  # rough estimate: 15s per run average
    print(f"  Estimated wall time: ~{est_seconds // 60}m {est_seconds % 60}s")
    print()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main() -> None:
    parser = argparse.ArgumentParser(
        description="HeteroPool memory-pool ablation experiment runner",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                      full matrix (4 modes × 3 scenarios × 10 reps)
  %(prog)s -n 2                 quick smoke test (2 reps)
  %(prog)s --skip-gpu           CPU-only scenarios
  %(prog)s --dry-run            print plan without executing
  %(prog)s --rebuild            rebuild before running
  %(prog)s -o results/my_run    custom output directory
        """,
    )
    parser.add_argument(
        "-n", "--repetitions", type=int, default=DEFAULT_REPETITIONS,
        help=f"Repetitions per experiment (default: {DEFAULT_REPETITIONS})",
    )
    parser.add_argument(
        "-t", "--timeout", type=int, default=DEFAULT_TIMEOUT,
        help=f"Per-run timeout in seconds (default: {DEFAULT_TIMEOUT})",
    )
    parser.add_argument(
        "--skip-gpu", action="store_true",
        help="Skip scenarios that require CUDA GPU",
    )
    parser.add_argument(
        "--no-build", action="store_true",
        help="Skip build freshness check",
    )
    parser.add_argument(
        "--rebuild", action="store_true",
        help="Force rebuild (cargo build --release) before running",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print experiment plan without executing",
    )
    parser.add_argument(
        "-o", "--output-dir", type=str, default=None,
        help="Override default results directory",
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true",
        help="Show dora stdout/stderr in real time",
    )
    parser.add_argument(
        "-s", "--tensor-bytes", type=int, default=0,
        help="Tensor size in bytes (default: 15000*512*8 = 61.44 MB). "
             "Use small sizes (64K–1M) to make ablation overheads visible; "
             "at large sizes (>10 MB) data-copy bandwidth dominates.",
    )
    parser.add_argument(
        "--tensor-sizes", type=str, default=None,
        help="Comma-separated list of tensor sizes in MiB (e.g. 1,10,25,61,160). "
             "When set, runs the full experiment matrix for EACH size, producing "
             "per-size subdirectories and an aggregate summary.",
    )
    args = parser.parse_args()

    # Parse --tensor-sizes into byte list
    tensor_sizes_list: Optional[list[int]] = None
    if args.tensor_sizes:
        tensor_sizes_list = []
        for val in args.tensor_sizes.split(","):
            val = val.strip()
            if not val:
                continue
            mib = float(val)
            tensor_sizes_list.append(int(mib * 1024 * 1024))

    # ------------------------------------------------------------------
    # Build the experiment plan
    # ------------------------------------------------------------------
    runner = AblationRunner(
        repetitions=args.repetitions,
        timeout=args.timeout,
        skip_gpu=args.skip_gpu,
        skip_build_check=args.no_build,
        rebuild=args.rebuild,
        dry_run=args.dry_run,
        output_dir=args.output_dir,
        verbose=args.verbose,
        tensor_bytes=args.tensor_bytes,
        tensor_sizes=tensor_sizes_list,
    )

    gpu_available = runner.detect_gpu()
    build_fresh = runner.check_build() if not args.no_build else True

    active_modes = list(MODES.keys())
    active_scenarios = [
        s for s, (_, needs_gpu) in SCENARIOS.items()
        if not needs_gpu or (gpu_available and not args.skip_gpu)
    ]

    # Calculate total runs accounting for per-mode scenario filters
    total_runs = 0
    for mode in active_modes:
        allowed = MODE_SCENARIOS.get(mode, None)
        n_scenarios = (
            len([s for s in active_scenarios if s in allowed])
            if allowed is not None
            else len(active_scenarios)
        )
        total_runs += n_scenarios * args.repetitions

    plan = ExperimentPlan(
        modes=active_modes,
        scenarios=active_scenarios,
        repetitions=args.repetitions,
        total_runs=total_runs,
        gpu_available=gpu_available,
        build_fresh=build_fresh,
    )

    print_plan(plan)

    # Pre-flight checks
    if not args.dry_run:
        if not build_fresh:
            print(
                "WARNING: Python extension .so is older than lib.rs.\n"
                "  Rust changes may not take effect. Use --rebuild to rebuild.\n"
            )
        if args.rebuild:
            if not runner.rebuild_project():
                sys.exit(1)
        if not args.no_build and not args.rebuild:
            # Re-check after potential warning
            pass

        if not Path(DORA_BIN).exists():
            print(f"FATAL: dora binary not found at {DORA_BIN}")
            sys.exit(1)

        # Pre-cleanup
        runner.cleanup_stale()

    # Run
    success = runner.run_all(plan)
    if not success and not args.dry_run:
        print("\nExperiment suite did not complete — partial results saved.")


if __name__ == "__main__":
    main()
