# How to Run the AEC Voice Chatbot

## Quick Start (Easiest)

Just run this single command:

```bash
cd /Users/yuechen/home/conversation/dora/examples/voice-chatbot-aec
chmod +x run_chatbot_aec.sh
./run_chatbot_aec.sh
```

Then **speak naturally** - the system will:
- Listen to your voice
- Transcribe what you say
- Generate a response with Qwen3 LLM
- Speak the response with TTS
- You can interrupt at any time!

## What You'll See

When running, you'll see a log display showing:
- 🎤 When speech is detected
- 📝 Your transcribed text
- 🤖 LLM responses
- 🔊 TTS audio generation
- ✅ System status messages

## To Stop

Press `Ctrl+C` to stop the chatbot.

## Test If AEC Is Working

Run the minimal test first:

```bash
chmod +x run_minimal_test.sh
./run_minimal_test.sh
```

This will show if:
- Microphone is capturing audio
- Speech detection is working
- Audio segments are being created

## Manual Steps (if script fails)

### Step 1: Clean up
```bash
pkill -f dora
```

### Step 2: Start Dora services
```bash
# Terminal 1
dora daemon --quiet

# Terminal 2  
dora coordinator --quiet
```

### Step 3: Start the chatbot
```bash
# Terminal 3
cd /Users/yuechen/home/conversation/dora/examples/voice-chatbot-aec
dora start voice_chatbot_aec.yml --name voice-chatbot-aec --detach
sleep 5  # Wait for initialization

# Terminal 4
python audio_player_aec.py

# Terminal 5
python log_display.py
```

## Tips

1. **Speak clearly** - The ASR works best with clear speech
2. **Wait for initialization** - First startup takes ~10-15 seconds
3. **Check the logs** - The log display shows what's happening
4. **Natural conversation** - You can interrupt the TTS at any time
5. **No echo** - The AEC removes echo from the speakers automatically

## Common Issues

**"Connection refused"** → Start dora daemon first
**"No audio"** → Check microphone permissions in System Settings
**"Node not found"** → Wait for dataflow to initialize before starting dynamic nodes