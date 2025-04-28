## Get started

```bash

## Deactivate any virtual environment


uv venv --seed

# This is because the current urdf fix is not yet merged in main
uv pip install git+https://github.com/dora-rs/rerun-loader-python-urdf.git@fix-urdf

# Git Clone somewhere:
git clone https://github.com/haixuanTao/reachy_description --depth 1

dora build realsense-dev.yml --uv
dora run realsense-dev.yml --uv
```

For macos version

```
dora build ios-dev.yml --uv
dora run ios-dev.yml --uv
```
