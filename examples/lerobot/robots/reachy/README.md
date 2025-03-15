## Reachy 2

### Installation

#### Installation SDK

```bash
### Install the sdk
git clone https://github.com/pollen-robotics/reachy2-sdk
cd reachy2-sdk
pip install -e .
cd ..

### Connect Camera USB
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

### Install Polen vision
git clone https://github.com/pollen-robotics/pollen-vision.git
cd pollen-vision
git checkout lerobot_only_left_camera
pip install ".[depthai_wrapper]"
cd ..

### Teleoperation Collector
git clone https://github.com/pollen-robotics/reachy2_hdf5_recorder/
```

#### Installation dora-lerobot

```bash
## Create new python environment

git clone git@github.com:huggingface/dora_lerobot.git
pip install -e dora_lerobot
git clone git@github.com:dora-rs/dora-dora_lerobot.git --branch WORKING-REACHY
pip install -e dora-dora_lerobot/gym_dora

cargo install dora-rs --locked
pip install dora-rs
```

### AI Pipeline

### Data Collection

```bash
cd reachy2_hdf5_recorder
python3 record_episodes_hdf5.py -n <recording_session_name>_raw -l <epiodes_duration in s> -r <framerate> --robot_ip <robot_ip>
```

```bash
huggingface-cli upload \
                <hf-organisation>/<dataset_name> \
                data/<recording_session_name>_raw/ \
                --repo-type dataset (--private)
```

> ### 06/07/2021
>
> As of today, we need to use several branches:
>
> - mobile_base : branch 21 # server side, install manually
> - reachy-sdk-api : branch 116 # server and client side, install manually
> - mobile-base-sdk : branch 25 # client side, install manually
> - reachy2-sdk-server : branch 135 # server side, install mannually
>   Then push to HF hub!

### Training

```bash
python dora_lerobot/scripts/train.py \
    policy=act_real \
    env=aloha_real \
    env.task=Reachy-v0 \
    dataset_repo_id=<org-id>/<data-id< \
```

### Evaluation

```bash
dora start reachy/graphs/eval.yml --attach
```

### Reachy Initialization

```bash
ssh bedrock@192.168.1.51
```

```bash
cd dev_docker
sudo service stop


docker compose -f mode/dev.yaml up -d core

docker exec -it core bash

# In the docker container

ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true
```
