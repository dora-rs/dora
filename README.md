# dora-rs
Dataflow Oriented Robotic Architecture

## Python API Design

The Python API is probably going to look as follows:
```python
@register
async def function_name(state: DoraState, message: DoraMessage):
    return outputs
``` 

The philosophy is to use async function as primary instance to:
- Mitigate the risk of running unsafe data mutations.
- Managing several run at the same time with timeout / deadline capabilities
- Using Tokio Spawn to avoid thread locks on CPU bound runs.

## Getting started
### Enabling autonomous driving through Dora.

Project largely inspired by [Pylot](https://github.com/erdos-project/pylot).

Thank you the Pylot team for your contribution to open autonomous driving.


I have made a simple example that can be run with:
```bash
nvidia-docker build --tag dora .
nvidia-docker run -itd --name dora -p 20022:22  dora /bin/bash
nvidia-docker exec -itd dora /home/erdos/workspace/pylot/scripts/run_simulator.sh
nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo chown erdos /home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo service ssh start
ssh -p 20022 -X erdos@localhost
```

Within the docker container, those script run the following scenario:

- simulator + visualisation:

```bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/examples/pylot
cargo run start-python carla_source_operator send &
SOURCE=image cargo run start-python sink_eval_plot plot image pose
```


- simulator + object detector + visualisation:
```bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/examples/pylot
cargo run start-python carla_source_operator send &
cargo run start-python efficient_detection run image &
cargo run start-python sink_eval_plot plot destination
```


- simulator + object detection + planning + visualisation:
```bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/examples/pylot
cargo run start-python carla_source_operator send &
cargo run start-python efficient_detection run image &
cargo run start-python planning_operator run pose open_drive &
SOURCE=image cargo run start-python sink_eval_plot plot image waypoints obstacles pose
```

- simulator + perfect object detection + object location + planning + visualisation:
```bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/examples/pylot
cargo run start-python carla_source_operator send &
cargo run start-python perfect_detection_operator run pose depth_frame segmented_frame &
cargo run start-python obstacle_location_operator run pose depth_frame obstacles_without_location &
cargo run start-python planning_operator run pose open_drive &
SOURCE=image cargo run start-python sink_eval_plot plot image waypoints obstacles pose
```