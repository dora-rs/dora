# Quick example on using a VLM with dora-rs

Make sure to have, dora, pip and cargo installed.

```bash
dora build vision_only.yml
dora run vision_only.yml

# Wait for the qwenvl model to download which can takes a bit of time.

dora build dataflow.yml
dora run dataflow.yml

# Wait for the qwenvl, whisper model to download which can takes a bit of time.
```
