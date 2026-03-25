#!/usr/bin/env bash
#
# Run the dora benchmark with multiple DORA_ZERO_COPY_THRESHOLD values.
# Outputs results to benchmark_results/ directory.
#
# Prerequisites:
#   cargo build -p benchmark-example-node -p benchmark-example-sink --release
#   cargo install --path binaries/cli --locked
#
# Usage:
#   ./run_benchmark.sh                    # Run with default threshold list
#   ./run_benchmark.sh 0 256 4096 65536   # Run with custom thresholds
#
# Environment:
#   BENCH_RUNS=3  Number of runs per threshold (default: 3)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="$SCRIPT_DIR/benchmark_results"
DATAFLOW="$SCRIPT_DIR/examples/benchmark/dataflow.yml"
TIMEOUT=180  # seconds per benchmark run
RUNS="${BENCH_RUNS:-3}"

# Default thresholds to test (bytes)
if [ $# -gt 0 ]; then
    THRESHOLDS=("$@")
else
    THRESHOLDS=(0 256 512 1024 2048 4096 8192 16384 65536)
fi

mkdir -p "$RESULTS_DIR"

# Clean up any leftover dora state
cleanup() {
    dora destroy 2>/dev/null || true
    # Clean leftover SHM files
    rm -f /dev/shm/zenoh_shm_* /dev/shm/dora_* 2>/dev/null || true
    sleep 1
}

echo "=== Dora Benchmark: Zero-Copy Threshold Comparison ==="
echo "Thresholds: ${THRESHOLDS[*]}"
echo "Runs per threshold: $RUNS"
echo "Results dir: $RESULTS_DIR"
echo ""

for threshold in "${THRESHOLDS[@]}"; do
    echo "--- Threshold: $threshold bytes ---"
    threshold_dir="$RESULTS_DIR/threshold_${threshold}"
    mkdir -p "$threshold_dir"

    for run in $(seq 1 "$RUNS"); do
        echo "  Run $run/$RUNS..."
        outfile="$threshold_dir/run_${run}.txt"

        cleanup

        # Start coordinator+daemon with the threshold env var
        export DORA_ZERO_COPY_THRESHOLD="$threshold"
        dora up &>/dev/null
        sleep 2

        # Run with --attach and capture all output (includes node stdout)
        # Use timeout to handle the known hang issue with zenoh subscribers
        if timeout "$TIMEOUT" dora start "$DATAFLOW" --name bench --attach \
            > "$outfile" 2>&1; then
            echo "    Completed."
        elif [ $? -eq 124 ]; then
            echo "    Timed out after ${TIMEOUT}s"
        else
            echo "    Completed (non-zero exit)."
        fi

        # Verify the threshold was actually applied by checking node's output
        if grep -q "DORA_ZERO_COPY_THRESHOLD=$threshold" "$outfile"; then
            echo "    Threshold verified: $threshold"
        else
            echo "    WARNING: threshold $threshold NOT confirmed in output!"
            grep "DORA_ZERO_COPY_THRESHOLD" "$outfile" || echo "    (no threshold line found)"
        fi

        # Show results inline
        if [ -s "$outfile" ]; then
            grep -E "(Latency|Throughput|size 0x|msg/s)" "$outfile" | sed 's/^/      /'
        else
            echo "      (no output captured)"
        fi

        cleanup
    done
    echo ""
done

# Print summary
echo "=== Summary ==="
echo ""
for threshold in "${THRESHOLDS[@]}"; do
    threshold_dir="$RESULTS_DIR/threshold_${threshold}"
    echo "--- Threshold: $threshold ---"
    for run in $(seq 1 "$RUNS"); do
        outfile="$threshold_dir/run_${run}.txt"
        if [ -s "$outfile" ]; then
            echo "  [Run $run]"
            grep -E "(DORA_ZERO_COPY|Latency|Throughput|size 0x|msg/s)" "$outfile" | sed 's/^/    /'
        fi
    done
    echo ""
done
