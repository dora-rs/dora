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

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESULTS_DIR="$SCRIPT_DIR/benchmark_results"
DATAFLOW="$SCRIPT_DIR/examples/benchmark/dataflow.yml"
TIMEOUT=120  # seconds per benchmark run

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
echo "Results dir: $RESULTS_DIR"
echo ""

for threshold in "${THRESHOLDS[@]}"; do
    echo "--- Threshold: $threshold bytes ---"
    outfile="$RESULTS_DIR/threshold_${threshold}.txt"

    cleanup

    # Start coordinator+daemon
    export DORA_ZERO_COPY_THRESHOLD="$threshold"
    dora up &>/dev/null

    # Give coordinator time to start
    sleep 2

    # Start the dataflow and capture sink output
    echo "  Starting benchmark..."
    dora start "$DATAFLOW" --name bench 2>/dev/null

    # Wait for results by tailing the sink log
    # The benchmark prints to stdout which dora captures in logs
    start_time=$SECONDS
    finished=false

    while [ $(( SECONDS - start_time )) -lt $TIMEOUT ]; do
        # Check if the dataflow has finished by looking at dora list
        status=$(dora list 2>/dev/null || echo "error")
        if echo "$status" | grep -q "Finished\|no running"; then
            finished=true
            break
        fi
        sleep 2
    done

    # Grab the sink logs
    dora logs bench rust-sink 2>/dev/null > "$outfile" || true

    if [ "$finished" = true ]; then
        echo "  Completed. Results saved to $outfile"
    else
        echo "  Timed out after ${TIMEOUT}s (partial results in $outfile)"
        dora stop --name bench 2>/dev/null || true
        sleep 2
    fi

    # Print results inline
    if [ -s "$outfile" ]; then
        sed 's/^/    /' "$outfile"
    else
        echo "    (no output captured)"
    fi
    echo ""

    cleanup
done

# Print summary
echo "=== Summary ==="
echo ""
for threshold in "${THRESHOLDS[@]}"; do
    outfile="$RESULTS_DIR/threshold_${threshold}.txt"
    if [ -s "$outfile" ]; then
        echo "--- Threshold: $threshold ---"
        cat "$outfile"
        echo ""
    fi
done
