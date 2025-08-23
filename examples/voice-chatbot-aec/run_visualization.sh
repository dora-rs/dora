#!/bin/bash

echo "======================================"
echo "AEC Real-time Sound Visualization"
echo "======================================"
echo ""
echo "This will display:"
echo "• Real-time audio levels (RMS and Peak)"
echo "• Voice Activity Detection (VAD) status"
echo "• Frequency spectrum analysis"
echo "• Audio quality indicators"
echo ""
echo "Make sure your microphone is connected and permissions are granted."
echo ""
read -p "Press Enter to start visualization..."

# Make sure Dora is up
echo "Starting Dora daemon..."
dora up

# Start the visualization
echo ""
echo "Starting AEC sound visualization..."
echo "Speak into your microphone to see the levels change!"
echo ""

# Run with the sound visualizer dynamic node
dora start aec_visualization.yml --attach \
    --sound-visualizer sound_visualizer.py

echo ""
echo "Visualization stopped."