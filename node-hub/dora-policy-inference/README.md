# dora-policy-inference

Load pre-trained policies and execute them in real-time to control robots based on camera observations and current robot state.

## Quick Start

### 1. Installation

```bash
# Source your venv 
cd dora/node-hub/dora-policy-inference
uv pip install -e .
```

### 2. Usage Guide

Create a dataflow file, see `examples/lerobot-dataset/policy_inference.yml`:

```yaml
nodes:
  # Policy inference
  - id: policy_inference
    build: pip install -e ../../node-hub/dora-policy-inference
    path: dora-policy-inference
    inputs:
    # your Cameras should be same as the dataset trained policy is on.
      laptop: laptop_cam/image
      front: front_cam/image
      robot_state: robot/pose
    outputs:
      - robot_action
      - status
    env:
      # Required settings
      MODEL_PATH: "/path/to/your/lerobot/model"
      TASK_DESCRIPTION: "pick up the cup"
      INFERENCE_FPS: "30"
      
      # Camera configuration
      CAMERA_NAMES: "laptop,front"
      CAMERA_LAPTOP_RESOLUTION: "480,640,3"
      CAMERA_FRONT_RESOLUTION: "480,640,3"      


  # Robot controller
  - id: robot_controller
    path: your-robot-controller
    inputs:
      action: policy_inference/robot_action # predicted joint state(rad)
    outputs:
      - pose
```

### 3. Start Policy Inference

```bash
dora build policy_inference.yml
dora run policy_inference.yml
```

The node will process camera inputs and robot state to generate actions for robot control.

## Configuration

### Required Environment Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `MODEL_PATH` | Path to trained LeRobot policy model directory | `"outputs/train/your_policy/checkpoints/last/pretrained_model"` |
| `CAMERA_NAMES` | Comma-separated camera names | `"laptop,front,top"` |
| `CAMERA_*_RESOLUTION` | Resolution for each camera (height,width,channels) | `"480,640,3"` |
| `INFERENCE_FPS` | Inference frequency | `"30"` |
| `TASK_DESCRIPTION` | Task description for task-conditioned policies | `"Grab the red cube and and drop in the box."` |

## Camera Configuration

For each camera defined in `CAMERA_NAMES`, you must set the resolution:

```bash
export CAMERA_NAMES="laptop,front,top"
export CAMERA_LAPTOP_RESOLUTION="1080,1920,3"
export CAMERA_FRONT_RESOLUTION="480,640,3" 
export CAMERA_TOP_RESOLUTION="480,640,3"
```

## Model Requirements

The node expects LeRobot trained models with:
- A `config.json` file in the model directory
- Model weights compatible with LeRobot's `from_pretrained()` method

#### Device Selection

The node automatically selects the best available device:
- CUDA GPU (if available)
- MPS (Apple Silicon)
- CPU (fallback)

Device selection is handled by LeRobot's `get_safe_torch_device()` function.

>See the `examples/lerobot-dataset/policy_inference.yml` for complete dataflow configurations.

## License

This project is released under the MIT License.