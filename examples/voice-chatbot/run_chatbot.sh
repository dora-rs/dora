#!/bin/bash

echo "======================================================"
echo "           VOICE CHATBOT SYSTEM"
echo "======================================================"
echo ""
echo "Features:"
echo "✓ Real-time speech recognition (English & Chinese)"
echo "✓ Natural conversation with Qwen3 LLM (MLX-optimized)"
echo "✓ High-quality TTS with PrimeSpeech"
echo "✓ Automatic pause during answer playback"
echo "✓ Conversation history and context"
echo "======================================================"
echo ""

# Clean up any existing dataflows
echo "Cleaning up existing dataflows..."
dora stop --name voice-chatbot 2>/dev/null
sleep 1

# Start dynamic nodes only (chat controller is now static)
echo "Starting microphone input..."
python microphone_input.py &
MIC_PID=$!
sleep 1

echo "Starting audio player..."
python audio_player.py &
AUDIO_PID=$!
sleep 1

echo "Starting log display..."
python log_display.py &
LOG_PID=$!
sleep 1

# Start the dataflow
echo ""
echo "Starting voice chatbot dataflow..."
echo "======================================================"
dora start voice_chatbot.yml --name voice-chatbot

echo ""
echo "System Architecture:"
echo "┌─────────────┐     ┌──────────────┐     ┌─────────┐"
echo "│  Microphone │────▶│Speech Monitor│────▶│   ASR   │"
echo "└─────────────┘     └──────────────┘     └────┬────┘"
echo "                      ▲ pause/resume           │"
echo "                      │                        ▼"
echo "              ┌───────┴──────────┐    ┌──────────────┐"
echo "              │ Chat Controller  │◀───│   Qwen3 LLM  │"
echo "              └───────┬──────────┘    └──────────────┘"
echo "                      │ answer"
echo "                      ▼"
echo "┌─────────────┐     ┌──────────────┐     ┌─────────┐"
echo "│Audio Player │◀────│  PrimeSpeech │◀────│  Text   │"
echo "└─────────────┘     │     TTS      │     │Segmenter│"
echo "                    └──────────────┘     └─────────┘"
echo ""
echo "Configuration (from voice_chatbot.yml):"
echo "• Silence threshold: 1500ms"
echo "• Min question length: 5 chars"
echo "• Conversation history: 10 exchanges"
echo "• LLM Model: Qwen3-8B (MLX 4-bit on Apple Silicon)"
echo "• Response temperature: 0.7"
echo ""
echo "Press Ctrl+C to stop"
echo "======================================================"

# Cleanup on exit
trap "dora stop --name voice-chatbot; kill $MIC_PID $AUDIO_PID $LOG_PID 2>/dev/null; exit" INT

# Keep running
while true; do
    sleep 1
done