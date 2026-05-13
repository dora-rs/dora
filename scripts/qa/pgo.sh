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

# cargo-pgo's `info` exits non-zero when *optional* BOLT tools are missing,
# even if llvm-profdata is found. We only need llvm-profdata for the PGO
# (no-BOLT) path, so capture the output without letting pipefail abort us
# on a BOLT-only failure, then grep for what we actually need.
PGO_INFO=$(cargo pgo info 2>&1 || true)
if ! grep -q "llvm-profdata.*found at" <<<"$PGO_INFO"; then
  echo "ERROR: llvm-profdata not found (needed for PGO profile merging)."
  echo "Install the rustup component:"
  echo "  rustup component add llvm-tools-preview"
  echo ""
  echo "cargo pgo info output was:"
  echo "$PGO_INFO"
  exit 1
fi

if ! command -v python3 >/dev/null; then
  echo "ERROR: python3 not found (needed for the comparison summary)."
  exit 1
fi

HOST=$(rustc -vV | awk '/host:/ {print $2}')
OUT_DIR="$REPO_ROOT/target/pgo-bench"
# Force --target $HOST for both baseline and PGO builds so paths are
# predictable regardless of user's cargo config. Step 3 (instrument) will
# overwrite the binary at target/$HOST/dist/dora, but step 2 has already
# captured the baseline CSV by then, so we also copy the binary to OUT_DIR
# so it survives the overwrite and can be re-inspected.
HOST_DIST_DORA="$REPO_ROOT/target/$HOST/dist/dora"
BASELINE_DORA="$OUT_DIR/baseline-dora"
PGO_DORA="$HOST_DIST_DORA"

mkdir -p "$OUT_DIR"

# ---- 1/5: baseline build ----
echo "=== [1/5] Building baseline (--profile dist) ==="
cargo build --target "$HOST" --profile dist \
  -p dora-cli \
  -p benchmark-example-node \
  -p benchmark-example-sink

if [ ! -x "$HOST_DIST_DORA" ]; then
  echo "ERROR: baseline dora not built at $HOST_DIST_DORA"
  exit 1
fi

# Preserve the baseline binary so step 3's instrument build doesn't clobber
# the only copy we have for re-inspection / re-running step 2.
cp "$HOST_DIST_DORA" "$BASELINE_DORA"

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
# Force --target $HOST so the instrumented binary always overwrites the
# baseline at target/$HOST/dist/dora. Without it, if cargo-pgo's behavior
# or the user's cargo config sends output elsewhere, the existence check
# below would pass against the stale baseline binary from step 1, and
# steps 4-5 would silently train+benchmark the baseline instead of the
# instrumented build — producing a meaningless +0% comparison.
echo ""
echo "=== [3/5] PGO instrumented build (~10 min) ==="
cargo pgo instrument build -- \
  --target "$HOST" \
  -p dora-cli \
  --profile dist

if [ ! -x "$PGO_DORA" ]; then
  echo "ERROR: instrumented dora not at $PGO_DORA"
  exit 1
fi

# Sanity check: the instrumented binary must differ from the baseline copy
# we made in step 1, otherwise step 3 silently produced the wrong artifact
# (e.g., empty profiles, no relink, etc.) and the comparison would be a no-op.
if cmp -s "$BASELINE_DORA" "$PGO_DORA"; then
  echo "ERROR: instrumented binary at $PGO_DORA is byte-identical to baseline"
  echo "       at $BASELINE_DORA — instrumentation did not take effect."
  echo "       Check 'cargo pgo instrument build' output for issues."
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
  --target "$HOST" \
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
single_run_caveat = (
    "        Single run with n=100/size — the geomean itself has ~5% noise floor."
    "\n        Re-run 3-5 times and take the median to confirm the signal is real."
)
if throughput_delta >= 5:
    print(f"VERDICT: PGO improves throughput by {throughput_delta:+.1f}% geomean on this platform.")
    print("        Worth integrating into the release pipeline if the +5% rule applies.")
    print(single_run_caveat)
elif throughput_delta <= -5:
    print(f"VERDICT: PGO regresses throughput by {throughput_delta:+.1f}% geomean on this platform.")
    print("        Do NOT ship PGO for this platform.")
    print(single_run_caveat)
else:
    print(f"VERDICT: PGO throughput change is {throughput_delta:+.1f}% geomean — within noise floor.")
    print("        Inconclusive — re-run with multiple iterations for stable medians.")

print()
print(f"Raw CSVs: {baseline_csv}  {pgo_csv}")
print(f"Logs:     {baseline_csv.replace('.csv', '.txt')}  {pgo_csv.replace('.csv', '.txt')}")
PYEOF
