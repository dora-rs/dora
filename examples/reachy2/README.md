# Reachy Demo

This example uses Reachy in parallel of Qwenvl2.5 in order to grasp object.

## To run

- Place Reachy near a table looking at the right. (He's going to turn left)
- Make sure that the height between the table and the torso camera is around 0.32m. You can modify this constraint within the state machine code.
  `       z = np.clip(z, -0.32, -0.22)`
- Clone and go to this folder:

```bash
git clone https://github.com/dora-rs/dora.git
cd dora/examples/reachy2
```

- Run the demo with:

```bash
# Make sure to not be in a virtual environment
uv venv --seed -p 3.11
dora build demo-dev.yml --uv

# Make sure to change ROBOT_IP!
dora run demo-dev.yml --uv

# First run might take a little bit more time as model need to be downloaded

# When you see:
# 2025-02-18T17:18:57.937123Z  INFO run_inner: dora_daemon: all nodes are ready, starting dataflow `01951a11-cfb9-7a70-9198-e12799f812a2` self.machine_id=
# Start speaking and saying something in the likes of: Grab the orange
# The robot should start moving
# The keywords are ACTIVATION_WORDS: grab pick give output take catch grabs picks gives output takes catches have
```

## Requirements

- This run on both:
  - MacOS with ~ 20G of RAM
  - Linux with ~ 10G of VRAM
  - I have not tried Windows.

TODO:

- [ ] Publish a remote dataflow to run this example without having to clone this repo.
