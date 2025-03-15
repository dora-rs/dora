# Dora pipeline Robots

Aloha is a bi manual robot that can be teleoperated using a similar arm. This repository contains
the Dora pipeline to manipulate arms, cameras, and record/replay episodes with LeRobot.

- To check if the robot is connected, install dynamixel
  wizard [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
- Dynamixel wizard is a very helpful debugging tool that connects to individual motors of the robot. It allows
  things such as rebooting the motor (very useful!), torque on/off, and sending commands.
  However, it has no knowledge about the kinematics of the robot, so be careful about collisions.
  The robot _will_ collapse if motors are torque off i.e. there is no automatically engaged brakes in joints.
- Open Dynamixel wizard, go into `options` and select:
    - Protocal 2.0
    - All ports
    - 1000000 bps
    - ID range from 0-10
- Note: repeat above everytime before you scan.
- Then hit `Scan`. There should be 4 devices showing up, each with 9 motors.
- One issue that arises is the port each robot binds to can change over time, e.g. a robot that
  is initially `ttyUSB0` might suddenly become `ttyUSB5`. To resolve this, we bind each robot to a fixed symlink
  port with the following mapping:
    - `ttyDXL_master_right`: right master robot (master: the robot that the operator would be holding)
    - `ttyDXL_puppet_right`: right puppet robot (puppet: the robot that performs the task)
    - `ttyDXL_master_left`: left master robot
    - `ttyDXL_puppet_left`: left puppet robot
- Take `ttyDXL_master_right`: right master robot as an example:

    1. Find the port that the right master robot is currently binding to, e.g. `ttyUSB0`
    2. run `udevadm info --name=/dev/ttyUSB0 --attribute-walk | grep serial` to obtain the serial number. Use the first
       one that shows up, the format should look similar to `FT6S4DSP`.
    3. `sudo vim /etc/udev/rules.d/99-fixed-interbotix-udev.rules` and add the following line:

       SUBSYSTEM=="tty", ATTRS{serial}=="<serial number here>", ENV{ID_MM_DEVICE_IGNORE}="1",
       ATTR{device/latency_timer}="1", SYMLINK+="ttyDXL_master_right"

    4. This will make sure the right master robot is _always_ binding to `ttyDXL_master_right`
    5. Repeat with the rest of 3 arms.

> You have an example of the given rules in `hardware_config.yml`.

```bash
cd dora-lerobot

sudo cp robots/aloha/hardware_config/99-interbotix-udev.rules /etc/udev/rules.d
sudo cp robots/aloha/hardware_config/99-fixed-interbotix-udev.rules /etc/udev/rules.d
```

- To apply the changes, run `sudo udevadm control --reload && sudo udevadm trigger`
- If successful, you should be able to find `ttyDXL*` in your `/dev`

## Documentation

https://github.com/Interbotix/interbotix_ros_toolboxes/blob/humble/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/core.py

https://github.com/Interbotix/interbotix_ros_toolboxes/blob/c187bcea89b60391244bb19943ebd78f770aa975/interbotix_xs_toolbox/interbotix_xs_modules/interbotix_xs_modules/xs_robot/core.py#L380-L398

## Acknowledgement

This work is inspired from [tonyzhaozh/aloha](https://github.com/tonyzhaozh/aloha) and we're trying to bring perfornance
improvement.

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).