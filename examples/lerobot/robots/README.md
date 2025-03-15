# `dora-rs` powered robots!

This repo will contain all application related to dora-rs powered robots.

## [Tau Robotics - Low Cost Robot](alexk-lcr/README.md)

## [Trossen Robotics - Aloha v2](aloha/README.md)

## [Pollen Robotics - Reachy1](reachy1/README.md)

## [Pollen Robotics - Reachy2](reachy2/README.md)

## [TheRobotStudio - SO-ARM100](so100/README.md)

## Add your own robot!

If you want to add your own robot, please follow the instructions below:

1. Create a new folder in the `robots` directory with the name of your robot.
2. Add a `README.md` file with the following structure:

```markdown

# Dora pipeline Robots

Your robot description here.

## Assembling

Your robot assembling instructions here.

## Installations

Your robot installation instructions here.

## Configuring

Your robot configuration instructions here.

## Recording

Your robot recording instructions here.

## Examples

Your robot examples here.

```

3. Create a `ASSEMBLING.md`, `INSTALLATION.md`, `CONFIGURING.md`, `RECORDING.md`, and `EXAMPLES.md` files with the
   instructions for assembling, installing, configuring, recording, and examples for your robot.
4. Add a `graphs` folder with the graph files for your robot.
5. Add a `requirements.txt` file with the required Python packages for your robot (link requirements of all nodes that you use from node-hub).
6. Add a `nodes` folder with the specific nodes for your robot.
7. Add a `configure.py` file to configure your robot.

**Note:** You can use the `alexk-lcr` robot as a reference.

## License

This library is licensed under the [Apache License 2.0](../LICENSE).
