#!/usr/bin/env bash
# scripts/qa/pgo.sh — Profile-Guided Optimization measurement
#
# Builds dora-cli with cargo-pgo (instrument -> train on examples/benchmark
# -> optimize), then runs the benchmark twice (baseline vs PGO) and prints
# a side-by-side comparison.
#
# Origin: dora-rs/dora#331. On macOS-arm64 (the platform where this was
# first measured) PGO gave roughly +25% throughput geomean across 10 payload
# sizes with latency unchanged. Run this on your target platform to find out
# if the same holds — PGO results don't transfer across OS/arch.
#
# Install:
#   cargo install cargo-pgo
#   rustup component add llvm-tools-preview
#
# Runtime: ~30-40 min wall time. Disk: ~5-10 GB extra target/ artifacts.
# Run on a quiet machine — benchmarks are sensitive to other CPU load.
#
# Decision rule: throughput geomean >= +5% on your platform -> PGO is worth
# adding to that platform's release-pipeline. Below that, the build-time cost
# (~17 min extra per release) outweighs the win.

set -euo pipefail

cd "$(dirname "$0")/../.."
REPO_ROOT="$PWD"

# ---- Preflight ----
if ! command -v cargo-pgo >/dev/null; then
  echo "ERROR: cargo-pgo not installed. Run:"
  echo "  cargo install cargo-pgo"
  echo "  rustup component add llvm-tools-preview"
  exit 1
fi

if ! cargo pgo info 2>&1 | grep -q "llvm-profdata.*found at"; then
  echo "ERROR: llvm-profdata not found (needed for PGO profile merging)."
  echo "Install the rustup component:"
  echo "  rustup component add llvm-tools-preview"
  exit 1
fi

if ! command -v python3 >/dev/null; then
  echo "ERROR: python3 not found (needed for the comparison summary)."
  exit 1
fi

HOST=$(rustc -vV | awk '/host:/ {print $2}')
OUT_DIR="$REPO_ROOT/target/pgo-bench"
BASELINE_DORA="$REPO_ROOT/target/dist/dora"
PGO_DORA="$REPO_ROOT/target/$HOST/dist/dora"

mkdir -p "$OUT_DIR"

# ---- 1/5: baseline build ----
echo "=== [1/5] Building baseline (--profile dist) ==="
cargo build --profile dist \
  -p dora-cli \
  -p benchmark-example-node \
  -p benchmark-example-sink

if [ ! -x "$BASELINE_DORA" ]; then
  echo "ERROR: baseline dora not built at $BASELINE_DORA"
  exit 1
fi

# ---- 2/5: baseline benchmark ----
echo ""
echo "=== [2/5] Running baseline benchmark (~2 min) ==="
(
  cd examples/benchmark
  BENCH_CSV="$OUT_DIR/baseline.csv" "$BASELINE_DORA" run dataflow.yml \
    > "$OUT_DIR/baseline.txt" 2>&1
)
echo "Baseline CSV: $OUT_DIR/baseline.csv"

# ---- 3/5: PGO instrumented build ----
echo ""
echo "=== [3/5] PGO instrumented build (~10 min) ==="
cargo pgo instrument build -- \
  -p dora-cli \
  --profile dist

if [ ! -x "$PGO_DORA" ]; then
  echo "ERROR: instrumented dora not at $PGO_DORA"
  exit 1
fi

# ---- 4/5: training run + optimize ----
echo ""
echo "=== [4/5] Training run + PGO optimize build (~7 min) ==="
(
  cd examples/benchmark
  LLVM_PROFILE_FILE="$REPO_ROOT/target/pgo-profiles/dora_%m_%p.profraw" \
    "$PGO_DORA" run dataflow.yml \
    > "$OUT_DIR/training.txt" 2>&1
)
cargo pgo optimize build -- \
  -p dora-cli \
  --profile dist

# ---- 5/5: PGO benchmark ----
echo ""
echo "=== [5/5] Running PGO benchmark (~2 min) ==="
(
  cd examples/benchmark
  BENCH_CSV="$OUT_DIR/pgo.csv" "$PGO_DORA" run dataflow.yml \
    > "$OUT_DIR/pgo.txt" 2>&1
)
echo "PGO CSV:      $OUT_DIR/pgo.csv"

# ---- Analysis ----
echo ""
echo "=== Comparison ==="
python3 - "$OUT_DIR/baseline.csv" "$OUT_DIR/pgo.csv" <<'PYEOF'
import csv, math, sys

baseline_csv, pgo_csv = sys.argv[1], sys.argv[2]

def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.reader(f):
            mode, bytes_, label, n, avg, p50, p95, p99, p999, _mn, _mx = r
            rows[(mode, label)] = {
                'bytes': int(bytes_), 'n': int(n),
                'avg': int(avg), 'p50': int(p50), 'p95': int(p95),
                'p99': int(p99), 'p999': int(p999),
            }
    return rows

base = load(baseline_csv)
pgo = load(pgo_csv)

def pct(b, p):
    return 0 if b == 0 else (p - b) / b * 100

def gmean(xs):
    return math.exp(sum(math.log(x) for x in xs) / len(xs)) if xs else 1.0

# Discover sizes from baseline keys, preserving insertion order
sizes = []
for (mode, label) in base.keys():
    if mode == 'latency' and label not in sizes:
        sizes.append(label)

# --- Throughput table ---
print()
print("Throughput (msg/s, higher = better)")
print(f"  {'size':>6}  {'baseline':>12}  {'pgo':>12}  {'delta':>10}  verdict")
print(f"  {'-'*6}  {'-'*12}  {'-'*12}  {'-'*10}  -------")
throughput_ratios = []
for size in sizes:
    bt, pt = base.get(('throughput', size)), pgo.get(('throughput', size))
    if not bt or not pt:
        continue
    d = pct(bt['avg'], pt['avg'])
    verdict = 'better' if d >= 5 else ('worse' if d <= -5 else 'noise')
    print(f"  {size:>6}  {bt['avg']:>10} m/s  {pt['avg']:>10} m/s  {d:>+8.1f}%  {verdict}")
    throughput_ratios.append(pt['avg'] / bt['avg'])

# --- Latency p50/p99 aggregate ---
p50_ratios, p99_ratios = [], []
for size in sizes:
    bl, pl = base.get(('latency', size)), pgo.get(('latency', size))
    if not bl or not pl:
        continue
    p50_ratios.append(pl['p50'] / bl['p50'])
    p99_ratios.append(pl['p99'] / bl['p99'])

print()
print("Aggregate (geomean across all sizes)")
print(f"  p50 latency  pgo/baseline = {gmean(p50_ratios):.3f}  ({(gmean(p50_ratios)-1)*100:+.1f}%)")
print(f"  p99 latency  pgo/baseline = {gmean(p99_ratios):.3f}  ({(gmean(p99_ratios)-1)*100:+.1f}%)")
print(f"  throughput   pgo/baseline = {gmean(throughput_ratios):.3f}  ({(gmean(throughput_ratios)-1)*100:+.1f}%)")

# --- Decision ---
print()
throughput_delta = (gmean(throughput_ratios) - 1) * 100
if throughput_delta >= 5:
    print(f"VERDICT: PGO improves throughput by {throughput_delta:+.1f}% geomean on this platform.")
    print("        Worth integrating into the release pipeline if the +5% rule applies.")
elif throughput_delta <= -5:
    print(f"VERDICT: PGO regresses throughput by {throughput_delta:+.1f}% geomean on this platform.")
    print("        Do NOT ship PGO for this platform.")
else:
    print(f"VERDICT: PGO throughput change is {throughput_delta:+.1f}% geomean — within noise floor.")
    print("        Inconclusive — re-run with multiple iterations for stable medians.")

print()
print(f"Raw CSVs: {baseline_csv}  {pgo_csv}")
print(f"Logs:     {baseline_csv.replace('.csv', '.txt')}  {pgo_csv.replace('.csv', '.txt')}")
PYEOF
