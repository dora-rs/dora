#!/bin/bash

echo "======================================================"
echo "    VOICE CHATBOT WITH AEC (Echo Cancellation)"
echo "======================================================"
echo ""
echo "Features:"
echo "✓ Acoustic Echo Cancellation - Full-duplex conversation"
echo "✓ Real-time speech recognition (English & Chinese)"
echo "✓ Natural conversation with Qwen3 LLM (MLX-optimized)"
echo "✓ High-quality TTS with PrimeSpeech"
echo "✓ No pause/resume needed - can interrupt anytime"
echo "✓ Hardware-accelerated VAD and echo cancellation"
echo "======================================================"
echo ""

# Install required packages first
echo "Installing required packages..."
pip install -e ../../node-hub/dora-aec > /dev/null 2>&1
pip install -e ../../node-hub/dora-chat-controller-aec > /dev/null 2>&1
echo "Packages installed."

# Clean up any existing dataflows
echo "Cleaning up existing dataflows..."
dora stop --name voice-chatbot-aec 2>/dev/null
sleep 1

# Start the dataflow first (detached)
echo ""
echo "Starting AEC voice chatbot dataflow..."
echo "======================================================"
dora start voice_chatbot_aec.yml --name voice-chatbot-aec --detach

# Wait for dataflow to initialize
echo "Waiting for dataflow to initialize..."
sleep 3

# Start dynamic nodes AFTER dataflow is running
# Note: No separate microphone needed - AEC node captures audio directly
echo "Starting audio player..."
python audio_player_aec.py &
AUDIO_PID=$!
sleep 1

echo "Starting log display..."
python log_display.py &
LOG_PID=$!
sleep 1

# Monitor the dataflow
echo "Monitoring dataflow..."
# Just keep the script running to show the architecture
sleep infinity

echo ""
echo "System Architecture with AEC:"
echo ""
echo "                ┌─────────────┐"
echo "                │AEC Processor│◀──── PrimeSpeech (reference)"
echo "                │ (captures   │      TTS"
echo "                │  directly)  │"
echo "                └──────┬──────┘"
echo "                       │ (clean audio)"
echo "                       ▼"
echo "              ┌──────────────┐     ┌─────────┐"
echo "              │Speech Monitor│────▶│   ASR   │"
echo "              └──────────────┘     └────┬────┘"
echo "                                        │"
echo "                                        ▼"
echo "              ┌──────────────┐    ┌──────────────┐"
echo "              │Chat Controller│◀───│   Qwen3 LLM  │"
echo "              └───────┬──────┘    └──────────────┘"
echo "                      │ question"
echo "                      ▼"
echo "┌─────────────┐     ┌──────────────┐     ┌─────────┐"
echo "│Audio Player │◀────│  PrimeSpeech │◀────│  Text   │"
echo "└─────────────┘     │     TTS      │     │Segmenter│"
echo "                    └──────────────┘     └─────────┘"
echo ""
echo "Key Differences from Pause/Resume Version:"
echo "• Full-duplex: Can speak while TTS is playing"
echo "• Zero latency: No switching delays"
echo "• Natural interruption: Interrupt at any time"
echo "• Hardware AEC: Uses macOS VoiceProcessingIO"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================================"

# Cleanup on exit
trap "dora stop --name voice-chatbot-aec; kill $AUDIO_PID $LOG_PID 2>/dev/null; exit" INT

# Keep running
while true; do
    sleep 1
done