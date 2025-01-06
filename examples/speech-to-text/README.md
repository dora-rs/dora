# Dora Speech to Text example

Make sure to have, dora, pip and cargo installed.

```bash
dora build https://raw.githubusercontent.com/dora-rs/dora/main/examples/speech-to-text/whisper.yml
dora run https://raw.githubusercontent.com/dora-rs/dora/main/examples/speech-to-text/whisper.yml

# Wait for the whisper model to download which can takes a bit of time.
```

## Graph Visualization

```mermaid

flowchart TB
  dora-microphone
  dora-vad
  dora-distil-whisper
  dora-rerun[/dora-rerun\]
subgraph ___dora___ [dora]
  subgraph ___timer_timer___ [timer]
    dora/timer/secs/2[\secs/2/]
  end
end
  dora/timer/secs/2 -- tick --> dora-microphone
  dora-microphone -- audio --> dora-vad
  dora-vad -- audio as input --> dora-distil-whisper
  dora-distil-whisper -- text as original_text --> dora-rerun

```
