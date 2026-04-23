#!/usr/bin/env bash
# Run Dora and ROS2 benchmarks side-by-side and compare results.
#
# Prerequisites:
#   - dora CLI installed
#   - Python dora package: pip install dora-rs
#   - numpy and pyarrow: pip install numpy pyarrow
#   - ROS2 (Humble+) with rclpy and std_msgs
#
# Usage:
#   cd examples/ros2-comparison
#   ./run_comparison.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

RESULTS_DIR="results"
mkdir -p "$RESULTS_DIR"

# --- Dora benchmark ---
echo "=== Running Dora benchmark ==="
BENCH_CSV="$RESULTS_DIR/dora.csv" dora run dataflow.yml --uv | tee "$RESULTS_DIR/dora.txt"

echo ""

# --- ROS2 benchmark ---
echo "=== Running ROS2 benchmark ==="
echo "Starting ROS2 receiver in background..."
BENCH_CSV="$RESULTS_DIR/ros2.csv" python3 ros2_receiver.py > "$RESULTS_DIR/ros2.txt" 2>&1 &
RECEIVER_PID=$!
sleep 2

echo "Starting ROS2 sender..."
python3 ros2_sender.py
sleep 3

echo "Stopping ROS2 receiver..."
kill $RECEIVER_PID 2>/dev/null || true
wait $RECEIVER_PID 2>/dev/null || true

echo ""
echo "=== Comparison ==="
if [ -f "$RESULTS_DIR/dora.csv" ] && [ -f "$RESULTS_DIR/ros2.csv" ]; then
    python3 analyze.py "$RESULTS_DIR/dora.csv" "$RESULTS_DIR/ros2.csv"
else
    echo "CSV files missing. Check benchmark output above for errors."
fi

echo ""
echo "Raw results:"
echo "  Dora: $RESULTS_DIR/dora.txt"
echo "  ROS2:  $RESULTS_DIR/ros2.txt"
echo "  CSVs:  $RESULTS_DIR/dora.csv, $RESULTS_DIR/ros2.csv"
