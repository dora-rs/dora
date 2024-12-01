# Chongyou Car Control

## Introduce

Control of the movement of the trolley by receiving texts

## Usage

Accepts an array of six f64's

- six f64 array [x, y, z, rx, ry, rz] only used x, rz

see [https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

## Environment

The default serial port is `/dev/ttyUSB0`

Added definition of default serial port number. Can additionally define the environment variable `SERIAL_PORT`
