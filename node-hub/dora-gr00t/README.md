# dora-gr00t

A Dora node for integrating NVIDIA's Isaac-GR00T model. This node enables communication between the Dora and the Isaac-GR00T model.

### Installation

- Install the node with uv:
  ```bash
  uv venv -p 3.11 --seed
  uv pip install -e .
  ```

## Input and Output Specification

### Inputs

| ID | Description | Data Type | Shape |
|---|---|---|---|
| `video.ego_view` | Robot's egocentric camera view | Image (uint8) | (H, W, 3) |
| `state.left_arm` | Left arm joint positions | Numeric (float) | (7,) |
| `state.right_arm` | Right arm joint positions | Numeric (float) | (7,) |
| `state.left_hand` | Left hand joint positions | Numeric (float) | (6,) |
| `state.right_hand` | Right hand joint positions | Numeric (float) | (6,) |
| `state.waist` | Waist joint positions | Numeric (float) | (3,) |
| `annotation.human.action.task_description` | Text description of the task | String | - |

### Outputs

| ID | Description | Data Type | Shape |
|---|---|---|---|
| `action.left_arm` | Predicted left arm joint positions | Numeric (float) | (16, 7) |
| `action.right_arm` | Predicted right arm joint positions | Numeric (float) | (16, 7) |
| `action.left_hand` | Predicted left hand joint positions | Numeric (float) | (16, 6) |
| `action.right_hand` | Predicted right hand joint positions | Numeric (float) | (16, 6) |
| `action.waist` | Predicted waist joint positions | Numeric (float) | (16, 3) |


## YAML Specification Example

```yaml
nodes:
  - id: dora-gr00t
    build: pip install -e ../../node-hub/dora-gr00t
    path: dora-gr00t
    inputs:
      video.ego_view: camera/image
      state.left_arm: robot_left/pose
      state.right_arm: robot_right/pose
      state.left_hand: robot_left_hand/pose
      state.right_hand: robot_right_hand/pose
      state.waist: waist/pose
      annotation.human.action.task_description: whisper/text
    outputs:
      - action.left_arm
      - action.right_arm
      - action.left_hand
      - action.right_hand
      - action.waist
```
