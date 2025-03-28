# dora-cotracker

Experimental node for tracking points in a video using the CoTracker model.

## Getting Started

- Install the dora-cotracker package:

```bash
pip install dora-cotracker
```
-Add the dora-cotracker node to your existing graph

Example:

- id: dora-cotracker
  custom:
    source: dora-cotracker
    inputs:
      video: camera/video
      coordinates: user/coordinates
      # You can add any input, and it is going to be logged and processed.

- Run the main script to start the Gradio interface:

```bash
    python -m dora_cotracker.main
```