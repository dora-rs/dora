# Dora pipeline Robots

Aloha is a bi manual robot that can be teleoperated using a similar arm. This repository contains
the Dora pipeline to manipulate arms, cameras, and record/replay episodes with LeRobot.

## Configuring

Once you have assembled the robot, and installed the required software, you can configure the robot. It's essential to
configure it
correctly for the robot to work as expected. Here are the reasons why you need to configure the robot:

- You may have done some 'mistakes' during the assembly, like inverting the motors, or changing the offsets by rotating
  the motors before assembling the robot. So your configuration will be different from the one we used to record the
  data set.
- The recording pipeline needs to know the position of the motors to record the data set correctly. If the motors are
  calibrated differently, the data set will be incorrect.

**Please read the instructions carefully before configuring the robot.**

The first thing to do is to configure the Servo BUS:

- Setting all the servos to the same baud rate (1M).
- Setting the ID of the servos from the base (1) to the gripper (9) for the Follower and Leader arms.

Those steps can be done using the official wizard provided by the
manufacturer [ROBOTICS](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

After that, you need to configure the homing offsets and drive mode to have the same behavior for every user. We
recommend using our on-board tool to set all of that automatically:

- Connect the Follower arm to your computer.
- Retrieve the device port from the official wizard.
- Run the configuration tool with the following command and follow the instructions:

```bash
cd dora/

# If you are using a custom environment, you will have to activate it before running the command
source [your_custom_env_bin]/activate

# If you followed the installation instructions, you can run the following command
source venv/bin/activate # On Linux
source venv/Scripts/activate # On Windows bash
venv\Scripts\activate.bat # On Windows cmd
venv\Scripts\activate.ps1 # On Windows PowerShell

python ./examples/aloha/configure.py --port /dev/ttyUSB0 --follower --left # (or right)
```

**Note:** change `/dev/ttyUSB0` to the device port you retrieved from the official wizard (like `COM3` on Windows).
**Note:** The configuration tool will disable all torque so you can move the arm freely to the Position 1.
**Note:** You will be asked to set the arm in two different positions. The two positions are:

TODO: image for aloha

**Node:** You will be asked the path of the configuration file, you can press enter to use the default one.

- Repeat the same steps for the Leader arm:

```bash
python ./examples/aloha/configure.py --port /dev/ttyUSB1 --leader --left # (or right)
```

**Note:** change `/dev/ttyUSB1` to the device port you retrieved from the official wizard (like `COM4` on Windows).
**Note:** The configuration tool will disable all torque so you can move the arm freely to the Position 1.
**Node:** You will be asked the path of the configuration file, you can press enter to use the default one.

After following the guide, you should have the following configuration:

TODO: image for aloha

This configuration has to be exported into environment variables inside the graph file. Here is an example of the
configuration:

```YAML
nodes:
  - id: aloha-follower
    env:
      PORT: /dev/ttyUSB0
      CONFIG: ../configs/follower.left.json # relative path to `./examples/aloha/configs/follower.json`

  - id: aloha-to-aloha
    env:
      LEADER_CONTROL: ../configs/leader.left.json
      FOLLOWER_CONTROL: ../configs/follower.left.json
```

## Acknowledgement

This work is inspired from [tonyzhaozh/aloha](https://github.com/tonyzhaozh/aloha) and we're trying to bring perfornance
improvement.

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).