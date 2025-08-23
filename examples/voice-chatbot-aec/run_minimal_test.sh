#!/bin/bash

echo "=========================================="
echo "MINIMAL TEST: AEC + Speech Monitor"
echo "=========================================="
echo ""
echo "This test verifies:"
echo "1. AEC captures audio from hardware"
echo "2. AEC sends audio to Speech Monitor"
echo "3. Speech Monitor receives and processes audio"
echo "4. Speech Monitor outputs audio segments"
echo ""
echo "=========================================="

# Clean up any existing instances
echo "Cleaning up..."
pkill -f dora 2>/dev/null
sleep 2

# Start daemon
echo "Starting Dora daemon..."
dora daemon --quiet &
DAEMON_PID=$!
sleep 2

# Start coordinator
echo "Starting Dora coordinator..."
dora coordinator --quiet &
COORD_PID=$!
sleep 2

# Start the dataflow
echo "Starting minimal test dataflow..."
dora start minimal_test.yml --name minimal-test --detach

# Wait for nodes to initialize
echo "Waiting for nodes to initialize..."
sleep 3

# Start the logger
echo ""
echo "Starting logger..."
python minimal_logger.py

# Cleanup
echo ""
echo "Cleaning up..."
dora stop --name minimal-test 2>/dev/null
kill $DAEMON_PID $COORD_PID 2>/dev/null
pkill -f dora 2>/dev/null

echo ""
echo "Test complete!"