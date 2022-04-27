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
./launch.sh
```

Within the docker container, those script run the following scenario:

- simulator + perfect object detection + object location + planning + visualisation:
```bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/examples/pylot
./target/release/dora-rs start-python carla_source_operator send &
./target/release/dora-rs start-python perfect_detection_operator run pose depth_frame segmented_frame &
./target/release/dora-rs start-python obstacle_location_operator run pose depth_frame obstacles_without_location &
./target/release/dora-rs start-python planning_operator run pose obstacles &
./target/release/dora-rs start-python pid_control_operator run pose waypoints &
./target/release/dora-rs start-python control_operator run control vehicle_id &
./target/release/dora-rs start-python summary_operator run control_status pose obstacles &
./target/release/dora-rs start-python sink_eval_plot plot image waypoints obstacles pose &
```