# SO100 and SO101 Remote Example

## Hardware requirements

- Realsense Camera
- so101 robotic arm

## Download the 3D model of the SO100

```bash
[ -f "$HOME/Downloads/so100_urdf.zip" ] || (wget -O "$HOME/Downloads/so100_urdf.zip" https://huggingface.co/datasets/haixuantao/urdfs/resolve/main/so100/so100_urdf.zip && unzip -o "$HOME/Downloads/so100_urdf.zip" -d "$HOME/Downloads/so100_urdf")
```

## To get started

```bash
uv venv --seed
dora build no_torque.yml --uv
```

## Make sure that both realsense and robotic arm connected

On linux, for the arm you can check connection with:

```bash
ls /dev/ttyACM*
```

This should show something like:

```bash
/dev/ttyACM0
```

Make sure to enable read with:

```bash
sudo chmod 777 /dev/ttyACM0
```

On linux, For the camera, make sure to have it well connected and check with:

```bash
ls /dev/video**
```

Result should be as follows:

```bash
/dev/video0  /dev/video2  /dev/video4  /dev/video6  /dev/video8
/dev/video1  /dev/video3  /dev/video5  /dev/video7  /dev/video9
```

## To run the no torque demo:

```bash
dora run no_torque.yml --uv
```

If the placement of the virtual robot arm is wrong, you can move it using the so100_transform environment configuration.

## To run the qwenvl demo:

```bash
dora run qwenvl.yml --uv
```

## To run the qwenvl remote demo:

On a remote machine:

```bash
dora coordinator &
dora daemon --machine-id gpu
```

```bash
dora daemon --coordinator-addr <IP_COORDINATOR_ADDR>
dora start qwenvl-remote.yml --uv --coordinator-addr <IP_COORDINATOR_ADDR>
```

## To run the qwenvl compression demo:

On a remote machine:

```bash
dora coordinator &
dora daemon --machine-id gpu
```

```bash
dora daemon --coordinator-addr <IP_COORDINATOR_ADDR>
dora start qwenvl-compression.yml --uv --coordinator-addr <IP_COORDINATOR_ADDR>
```
