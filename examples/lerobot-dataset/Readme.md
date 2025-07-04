# Dataset Recording with SO101 Robot Arm

Guide for recording imitation learning datasets with the SO101 arm using Dora and train policies with LeRobot.

## Overview

To begin recording, edit the dataflow.yml file. You must update camera settings, robot ports, and dataset configuration based on your setup.

### Required Changes

Update the camera device id, you can also add or remove cameras based on your setup:

```yaml
# Laptop camera
CAPTURE_PATH: "0"  # Laptop camera is defaults to 0
# External camera
CAPTURE_PATH: "1"  # Change this to match your external camera device
```
### SO101 Arms

Identify and set the correct USB ports for both the leader and follower SO101 arms
```yaml
PORT: "/dev/ttyACM0" # change this
```
### Dataset Recorder Configuration

>Edit the following fields:

```yaml
REPO_ID: "your_username/so101_dataset"  # HuggingFace dataset path
SINGLE_TASK: "Pick up the red cube and place it in the blue box"  # Your task description
CAMERA_NAMES: "laptop, front"  # Name your camera sources depending on your setup
CAMERA_LAPTOP_RESOLUTION: "480,640,3"
CAMERA_FRONT_RESOLUTION: "480,640,3"
```

You can adjust the following parameters as needed:

```yaml
TOTAL_EPISODES: "2"         # Number of episodes 
#you may want to try with 2-3 episodes to test, then atleast 50 episodes for training is recommended
EPISODE_DURATION_S: "60"     # Duration of each episode (in seconds) - depends on complexity of task
RESET_DURATION_S: "15"       # Time to reset the environment between episodes
FPS: "30"                    # Should match camera fps
PUSH_TO_HUB: "false"         # Set to "true" to auto-upload dataset to HuggingFace
ROOT_PATH: "full path where you want to save the dataset" 
# if not defined then it will be stored at ~/.cache/huggingface/lerobot/repo_id
```

Once everything is updated in `dataflow.yml`, you are ready to record your dataset.

## Start Recording
Build and Run

```bash
dora build dataflow.yml
dora run dataflow.yml
```

## Recording Process

In the rerun window you can see the the info regarding Start of episodes, start of reset phase and saving of episodes.

#### During Recording

1. **Episode Active**:
   - Use the **leader robot** to demonstrate/perform the task
   - Move smoothly and naturally
   - Complete the full task within the time limit

2. **Reset Phase**:
   - move the leader arm to initial position
   - Reset objects to starting positions
   - Prepare workspace for next demonstration

3. **Repeat** until all episodes are complete

**Recording Tips**

- **Practice First**: Move the leader robot around to get comfortable before recording
- **Smooth Movements**: Avoid jerky or sudden movements
- **Complete Tasks**: Try to finish the task within each episode
- **Consistent Setup**: Keep object positions and lighting consistent
- **Monitor Rerun**: Watch the visualization to ensure cameras and robot data are flowing

## After Recording

Your dataset will be saved locally. Check the recording was successful:
>It will be stored in ~/.cache/huggingface/lerobot/repo_id


# Training and Testing Policies using the recorded dataset

After successfully recording your dataset, we will be training imitation learning policies and deploying them on your SO101 robot arm.

## Training Your Policy

#### Install LeRobot Training Dependencies

Easiest way to train your policy is to use lerobots training scripts
```bash
# Install training requirements and lerobot repo
git clone https://github.com/huggingface/lerobot.git
pip install lerobot[training]
pip install tensorboard wandb  # For monitoring (Optional)
```

### Choose Your Policy

**ACT (Recommended for SO101)**
- Good for manipulation tasks
- Handles multi-modal data well
- Faster training (for me it took 7hrs 🥺 to train on 50 episodes for pick and place, 3050 laptop gpu)

**Diffusion Policy**
- Better for complex behaviors
- More robust to distribution shift
- Longer training


### Start Training

```bash
cd lerobot

python lerobot/scripts/train.py \
  --dataset.repo_id=${HF_USER}/your_repo_id \ # provide full path of your dataset
  --policy.type=act \
  --output_dir=outputs/train/act_so101_test \
  --job_name=act_so101_test \
  --policy.device=cuda \
  --wandb.enable=true \
  --policy.repo_id=${HF_USER}/my_policy
```

You can monitor your training progress on wandb
> For more details regarding training check [lerobot](https://huggingface.co/docs/lerobot/en/il_robots#train-a-policy) guide on Imitation learning for SO101

## Policy Inference and Testing

After training your policy, you can test it on your SO101 arm using the `policy_inference.yml`.

Edit the `policy_inference.yml` file and update the following settings based on your hardware setup:

#### 1. Camera Configuration

Update the camera device IDs to match your setup (same as recording):

```yaml
# Laptop camera
CAPTURE_PATH: "0"  # Usually 0 for built-in laptop camera

# External camera  
CAPTURE_PATH: "1"  # Change this to your external camera device ID
```

#### 2. SO101 Robot Port

Set the correct USB port for your follower SO101 arm:

```yaml
PORT: "/dev/ttyACM1"  # Update this to match your follower robot port
```

#### 3. Model Configuration

Update the path to your trained model and task description:

```yaml
MODEL_PATH: "./outputs/train/act_so101_test/checkpoints/last/pretrained_model"  # Path to your trained model
TASK_DESCRIPTION: "Pick up the red cube and place it in the blue box"
```

#### 4. Camera Settings

Ensure camera settings match your recording configuration:

```yaml
CAMERA_NAMES: "laptop, front"  # Must match training setup
CAMERA_LAPTOP_RESOLUTION: "480,640,3"  # Must match training
CAMERA_FRONT_RESOLUTION: "480,640,3"   # Must match training
```

### Start Policy Inference

Once you've updated the configuration:

```bash
dora build policy_inference.yml
dora run policy_inference.yml
```