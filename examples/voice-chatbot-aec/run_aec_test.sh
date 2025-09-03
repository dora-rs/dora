#!/bin/bash

echo "======================================"
echo "AEC Microphone Input Test"
echo "======================================"
echo ""
echo "This test will:"
echo "1. Start the AEC processor to capture audio from your microphone"
echo "2. Record audio for 20 seconds"
echo "3. Save the recording as a WAV file"
echo ""
echo "Make sure your microphone is connected and permissions are granted."
echo ""
read -p "Press Enter to start the test..."

# Make sure Dora is up
echo "Starting Dora daemon..."
dora up

# Start the test configuration
echo ""
echo "Starting AEC microphone test..."
echo "Recording will begin automatically and last for 20 seconds."
echo ""

# Run the test with the dynamic nodes
dora start test_aec_microphone.yml --attach \
    --audio-recorder audio_recorder.py

echo ""
echo "Test completed!"
echo "Check the current directory for the recorded WAV file (aec_recording_*.wav)"
echo ""
echo "You can play the recording with:"
echo "  afplay aec_recording_*.wav  (on macOS)"
echo "  or"
echo "  open aec_recording_*.wav"