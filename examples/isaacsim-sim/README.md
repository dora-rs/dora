# dora-rs & isaacsim

![dora_isaacsim_act](./docs/dora_isaacsim_act.png)

## Preparation

1. Clone this repository.
2. Add https://huggingface.co/berrylvz/policy_act/tree/main/assets/ckpt into `node-hub/dora-act/dora_act/assets/ckpt/`.

3. Download [isaacsim](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html).
4. Add scenary to `node-hub/dora-isaacsim/dora_isaacsim/assets/`.

```shell
assets/
├── franka/
├── gripper/
├── Owl/
├── Simple_Room/
└── stack_cube_franka.usd
```

5. Change `ISAAC_PYTHON_PATH` in `./dataflow.yml` to `"<path of isaacsim>/python.sh"`.
6. Create conda env.

```shell
conda create -n dora_isaacsim python=3.10.4
conda activate dora_isaacsim
pip install -r requirements.txt
```

7. Install necessary packages into isaacsim.

```shell
<path of isaacsim>/python.sh -m pip install -r requirements_isaacsim.txt
```

## Getting Started

1. Activate Conda environment.

```shell
conda activate dora_isaacsim
```

2. Spawn coordinator and daemon.

```shell
dora up
```

3. Start the dataflow.

```shell
dora build dataflow.yml
dora start dataflow.yml
```

4. Execution example (Refer [Youtube video](https://youtu.be/oK5c1U3C87g)).

Click on the Stage > /World/franka and press `F`.

5. Close the dora-rs.

```shell
dora destroy
```

## YAML Specification

```yaml
nodes:
  - id: dora-isaacsim
    build: pip install -e ../../node-hub/dora-isaacsim
    path: ../../node-hub/dora-isaacsim/dora_isaacsim/main.py
    env:
      # substitute to your own "<path of isaacsim>/python.sh"
      ISAAC_PYTHON_PATH: "/home/lv/isaacsim/python.sh"
      CONFIG_NAME: "stack_cube_act"
    inputs:
      request_camera: dora-act/request_camera
      request_joint_pos: dora-act/request_joint_pos
      action: dora-act/action
    outputs:
      - camera
      - joint_pos
  
  - id: dora-act
    build: pip install -e ../../node-hub/dora-act
    path: ../../node-hub/dora-act/dora_act/main.py
    args: 
      --task_name stack_cube
      # path of model parameters (relative to the current dataflow file)
      --ckpt_dir ../../dora-act/dora_act/assets/ckpt/
      --policy_class ACT
      --kl_weight 10
      --chunk_size 20
      --hidden_dim 512
      --batch_size 8
      --dim_feedforward 3200
      --num_epochs 2000
      --lr 1e-5
      --seed 0
      --temporal_agg
      # --eval
    inputs: 
      camera: dora-isaacsim/camera
      joint_pos: dora-isaacsim/joint_pos
    outputs:
      - request_camera
      - request_joint_pos
      - action
    env:
      SCENARIO: sim
```

## Development

Both of `request_camera` and `request_joint_pos` are requests and have no requirements for data format, and can also be modified to DORA's built-in timer, like `request_*: dora/timer/millis/50`.

`camera` is transmitted as RGB data that is flattened into a one-dimensional array.

Both of `joint_pos` and `action` are pose data of the robotic arm, which is a floating-point list.

