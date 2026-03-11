#!/usr/bin/env bash
# Run Adora and ROS2 benchmarks side-by-side and compare results.
#
# Prerequisites:
#   - adora CLI installed
#   - Python adora package: pip install adora-rs
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

# --- Adora benchmark ---
echo "=== Running Adora benchmark ==="
BENCH_CSV="$RESULTS_DIR/adora.csv" adora run dataflow.yml --uv | tee "$RESULTS_DIR/adora.txt"

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
if [ -f "$RESULTS_DIR/adora.csv" ] && [ -f "$RESULTS_DIR/ros2.csv" ]; then
    python3 analyze.py "$RESULTS_DIR/adora.csv" "$RESULTS_DIR/ros2.csv"
else
    echo "CSV files missing. Check benchmark output above for errors."
fi

echo ""
echo "Raw results:"
echo "  Adora: $RESULTS_DIR/adora.txt"
echo "  ROS2:  $RESULTS_DIR/ros2.txt"
echo "  CSVs:  $RESULTS_DIR/adora.csv, $RESULTS_DIR/ros2.csv"
