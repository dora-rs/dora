#!/bin/bash

echo "=========================================="
echo "MINIMAL TEST: AEC + Speech Monitor + ASR"
echo "=========================================="
echo ""
echo "This test verifies the complete pipeline:"
echo "1. AEC captures audio with echo cancellation"
echo "2. Speech Monitor detects speech segments"
echo "3. ASR transcribes speech to text"
echo ""
echo "INSTRUCTIONS:"
echo "- Speak clearly in Chinese or English"
echo "- Try saying: '你好，今天天气怎么样？'"
echo "- Or in English: 'Hello, how are you today?'"
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
echo "Starting test dataflow..."
dora start minimal_test_with_asr.yml --name minimal-asr-test --detach

# Wait for nodes to initialize
echo "Waiting for nodes to initialize..."
echo "(ASR model will be downloaded on first run, this may take a minute)"
sleep 5

# Start the logger
echo ""
echo "Starting logger..."
echo "=========================================="
python minimal_logger_asr.py

# Cleanup
echo ""
echo "Cleaning up..."
dora stop --name minimal-asr-test 2>/dev/null
kill $DAEMON_PID $COORD_PID 2>/dev/null
pkill -f dora 2>/dev/null

echo ""
echo "Test complete!"