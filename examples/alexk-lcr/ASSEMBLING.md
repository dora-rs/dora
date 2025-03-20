# Dora pipeline Robots

AlexK Low Cost Robot is a low-cost robotic arm that can be teleoperated using a similar arm. This repository contains
the Dora pipeline to manipulate the arms, the camera, and record/replay episodes with LeRobot.

## Assembling

**Please read the instructions carefully before buying or printing the parts.**

You will need to get the parts for a Follower arm and a Leader:

- [AlexK Follower Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot/?tab=readme-ov-file#follower-arm)
- [AlexK Leader Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot/?tab=readme-ov-file#follower-arm)

You **must** assemble the arm **with the extension** to be able to do some of the tasks.

You then need to print the Follower arm and the Leader arm. The STL files are:

- [AlexK Follower Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot/tree/main/hardware/follower/stl)
- [AlexK Leader Low Cost Robot](https://github.com/AlexanderKoch-Koch/low_cost_robot/tree/main/hardware/leader/stl)

Some parts **must** be replaced by the ones in this repository:

- [Dora-LeRobot Base Leader Low Cost Robot](assets/stl/LEADER_Base.stl)

If you struggle buying XL330 Frame or XL330/XL430 Idler Wheel, here are STL files that can be printed instead:

- [XL330 Frame](assets/stl/XL330_Frame.stl)
- [XL330 Idler Wheel](assets/stl/XL330_Idler_Wheel.stl)
- [XL430 Idler Wheel](assets/stl/XL430_Idler_Wheel.stl)

Please then follow the [YouTube Tutorial by Alexander Koch](https://youtu.be/RckrXOEoWrk?si=ZXDnnlF6BQd_o7v8) to
assemble the arm correctly.
Note that the tutorial is for the arm without the extension, so you will have to adapt the assembly.

Then you can place the two cameras on your desk, following this [image]()

## License

This library is licensed under the [Apache License 2.0](../../LICENSE).