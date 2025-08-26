#!/bin/bash

echo "========================================="
echo "AEC Recording Test"
echo "========================================="
echo
echo "This will record 30 seconds of audio from MAC-AEC"
echo "Recordings will be saved to ./aec_recordings"
echo

# Create recordings directory
mkdir -p aec_recordings

# Clean up any previous recordings
echo "Cleaning up previous recordings..."
rm -f aec_recordings/*.wav

echo
echo "Starting Dora dataflow..."
dora start voice-chat-with-aec.yml

echo
echo "Starting nodes..."
echo

# Start MAC-AEC in background
echo "1. Starting MAC-AEC node..."
python mac_aec_simple_segmentation.py &
MAC_AEC_PID=$!

sleep 2

# Start audio recorder
echo "2. Starting audio recorder..."
python audio_segment_recorder.py &
RECORDER_PID=$!

# Start viewer (optional)
echo "3. Starting viewer (optional)..."
python viewer.py &
VIEWER_PID=$!

echo
echo "Recording for 30 seconds..."
echo "Speak into the microphone and play some audio to test echo cancellation"
echo

# Wait for recording to complete (30 seconds + some buffer)
sleep 35

echo
echo "Stopping nodes..."

# Stop all nodes
kill $MAC_AEC_PID 2>/dev/null
kill $RECORDER_PID 2>/dev/null
kill $VIEWER_PID 2>/dev/null

# Stop dataflow
dora stop

echo
echo "Recording complete!"
echo
echo "Check the recordings in ./aec_recordings:"
ls -la aec_recordings/*.wav

echo
echo "You can play the recordings with:"
echo "  afplay aec_recordings/continuous_*.wav  # Continuous audio"
echo "  afplay aec_recordings/combined_*.wav    # Combined segments"
echo
echo "To check if AEC is working:"
echo "1. Play some audio through speakers while recording"
echo "2. The recorded audio should have minimal echo"
echo "3. Compare with recordings made without AEC"