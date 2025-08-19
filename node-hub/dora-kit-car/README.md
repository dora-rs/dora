# dora-kit-car control

## Introduce

Dora Kit Car is a DORA node for controlling a differential-drive mobile robot to move forward/backward and turn left/right. Developed in Rust with Python API support.

## Highlights

- Compatible with the ROS geometry_msgs/Twist.msg format, utilizing only:
- linear.x (positive: forward movement, negative: backward movement)
- angular.z (positive: left turn, negative: right turn)

## Raw Message Definition

Accepts an array of six f64's

- six f64 array [x, y, z, rx, ry, rz] only used x, rz

see [https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)

## Environment

Adds an environment variable `SERIAL_PORT` to specify the serial port for the car device, with `/dev/ttyUSB0` as the default value

## Demo Video

[![Dora Kit Car Video](https://yt3.ggpht.com/92FGXQL59VsiXim13EJQek4IB7CRI-9SjmW3LhH8PFY16oBXqKUvkKhg5UdzLiGCOmoSuTvdpQxIuw=s640-rw-nd-v1)](https://youtu.be/B7zGHtRUZSo)

## Getting Started

```yaml
nodes:
  - id: keyboard-listener # Run on car
    build: pip install dora-keyboard
    path: dora-keyboard
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - twist # for example [2.0,0.0,0.0,0.0,0.0,1.0]

  - id: car
    build: pip install dora-kit-car
    path: dora-kit-car
    inputs:
      keyboard: keyboard-listener/twist
    env:
      SERIAL_PORT: /dev/ttyUSB0

```

## License

The MIT License (MIT)

Copyright (c) 2024-present, Leon