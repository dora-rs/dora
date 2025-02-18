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
dora run demo-dev.yml --uv

# First run might take a little bit more time as model need to be downloaded
```

## Requirements

- This run on both:
  - MacOS with ~ 20G of RAM
  - Linux with ~ 10G of VRAM
  - I have not tried Windows.

TODO:

- [ ] Publish a remote dataflow to run this example without having to clone this repo.
