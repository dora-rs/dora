# dora-dataset-record

Node for recording robot datasets in LeRobot format. You can captures synchronized camera feeds and robot poses to create high-quality datasets for imitation learning and robot training.

- **Robot pose recording** - Capture both state and action data
- **Multi-camera support** - Record from multiple cameras simultaneously
- **LeRobot dataset format (v2.1)** - Direct integration with HuggingFace LeRobot datasets
- **Episode management** - Automatic episode segmentation with reset phases

## Quick Start

### 1. Installation

```bash
# Source your venv 
cd dora/node-hub/dora-dataset-record
uv pip install -e .
```

### 2. Usage Guide

Create a dataflow file, see `examples/lerobot-dataset-record/dataflow.yml`:

```yaml
nodes:
  # Dataset recorder
  - id: dataset_recorder
    build: pip install -e ../../node-hub/dora-dataset-record
    path: dora-dataset-record
    inputs:
      laptop: laptop_cam/image
      front: front_cam/image
      robot_state: robot_follower/pose
      robot_action: leader_interface/pose
    outputs:
      - text
    env:
      # Required settings
      REPO_ID: "your_username/your_dataset_name"
      SINGLE_TASK: "Pick up the cube and place it in the box"
      ROBOT_TYPE: "your_robot_type"
      
      # Recording settings
      FPS: "30"
      TOTAL_EPISODES: "50" 
      EPISODE_DURATION_S: "60"
      RESET_DURATION_S: "15"
      
      # Camera configuration
      CAMERA_NAMES: "laptop,front"
      CAMERA_LAPTOP_RESOLUTION: "480,640,3"
      CAMERA_FRONT_RESOLUTION: "480,640,3"
      
      # Robot configuration
      ROBOT_JOINTS: "joint1,joint2,joint3,joint4,joint5,gripper"
      
      # Optional settings
      USE_VIDEOS: "true"
      PUSH_TO_HUB: "false" 
      PRIVATE: "false"
      TAGS: "robotics,manipulation,imitation_learning"

  # Visualization with rerun
  - id: plot
    build: pip install dora-rerun
    path: dora-rerun
    inputs:
      text: dataset_recorder/text
```

### 3. Start Recording the dataset

```bash
dora build dataflow.yml
dora run dataflow.yml
```

The node will send instructions on dora-rerun, about episode starting, reset time, Saving episodes etc.

## Configuration

### Required Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `REPO_ID` | HuggingFace dataset repo | `"username/dataset_name"` |
| `SINGLE_TASK` | Task description | `"Pick and place objects"` |
| `CAMERA_NAMES` | Comma-separated camera names | `"laptop,front,top"` |
| `CAMERA_*_RESOLUTION` | Resolution for each camera | `"480,640,3"` |
| `ROBOT_JOINTS` | Comma-separated joint names | `"joint1,joint2,gripper"` |

### Optional Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `FPS` | `30` | Recording frame rate (match camera fps) |
| `TOTAL_EPISODES` | `10` | Number of episodes to record |
| `EPISODE_DURATION_S` | `60` | Episode length in seconds |
| `RESET_DURATION_S` | `15` | Break between episodes to reset the environment |
| `USE_VIDEOS` | `true` | Encode as MP4 videos, else saves images |
| `PUSH_TO_HUB` | `false` | Upload to HuggingFace Hub |
| `PRIVATE` | `false` | Make dataset private |
| `ROOT_PATH` | `~/.cache/huggingface/lerobot/your_repo_id` | Local storage path where you want to save the dataset |

## License

This project is released under the MIT License.