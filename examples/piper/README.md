# Getting Started with Tracer + Piper

## Installation (To do once)

Make sure to:

```bash
dora build rdt_1b.yaml

# Make sure to install from source pyorbbecksdk

git clone https://github.com/orbbec/pyorbbecsdk
cd pyorbbecsdk
pip3 install -r requirements.txt
mkdir build
cd build
cmake -Dpybind11_DIR=`pybind11-config --cmakedir` ..
make -j4
make install
cd ..
pip3 install wheel
python3 setup.py bdist_wheel
pip3 install dist/*.whl

export PYTHONPATH=$PYTHONPATH:$(pwd)/install/lib/ # Make sure to save this in your .bashrc


# Install ugv_sdk_py from source
git clone https://github.com/westonrobot/ugv_sdk
cd ugv_sdk
python setup.py build_ext --inplace

export PYTHONPATH=$PYTHONPATH:$(pwd) # Make sure to save this in your .bashrc
```

### Your bashrc should contain something like this

```bash
export PYTHONPATH=$PYTHONPATH:/home/agilex/pyorbbecsdk/install/lib/:/home/agilex/ugv_sdk
```

## Setup ( Every boot of the computer )

```bash
# Run on Agilex provided computer
source /home/agilex/cobot_magic/Piper_ros_private-ros-noetic/can_config.sh
```

## Run

### For recording episode

```bash
dora run record.yml
```

## For inference

```bash
dora run rdt_1b.yml
```
