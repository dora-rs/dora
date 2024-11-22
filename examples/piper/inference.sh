#!/bin/bash
if ! ifconfig | grep -q "can_left"; then
    source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/can_config.sh 
    sleep 5
fi
export PYTHONPATH=$PYTHONPATH:/home/agilex/1ms.ai/pyorbbecsdk/install/lib/:/home/agilex/1ms.ai/ugv_sdk
source /home/agilex/miniconda3/etc/profile.d/conda.sh  
pkill dora
conda activate dora
dora run /home/agilex/1ms.ai/dora/examples/piper/rdt_1b.yml
