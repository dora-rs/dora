# ROS2-bridge development image for macOS (issue #1170, Phase 0).
#
# Reproduces the nightly `ros2-bridge` CI job (.github/workflows/nightly.yml)
# locally on a Mac, where ROS2 cannot run natively. EVERYTHING — the dora
# process AND the ROS2 nodes — runs inside THIS one container, so DDS discovery
# happens within a single Linux network namespace exactly as on the CI runner.
# Never split dora and ROS2 across containers: Docker Desktop for Mac has no
# host networking to macOS and drops multicast SPDP, so discovery silently fails.
#
# Pins match CI: ROS2 Humble, Rust 1.92.0, Python 3.12.
FROM ros:humble

ARG RUST_VERSION=1.92.0

ENV DEBIAN_FRONTEND=noninteractive \
    RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    QT_QPA_PLATFORM=offscreen \
    PATH=/usr/local/cargo/bin:/root/.local/bin:/opt/venv/bin:$PATH

# System + ROS2 packages. turtlesim and examples_rclcpp_minimal_service are
# spawned by the example harnesses via `ros2 run`; clang/cmake build the C++
# example node; libssl/pkg-config build dora's Rust deps.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential clang cmake pkg-config libssl-dev \
        curl git ca-certificates \
        ros-humble-turtlesim \
        ros-humble-examples-rclcpp-minimal-service \
    && rm -rf /var/lib/apt/lists/*

# Rust toolchain pinned to CI's version.
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain ${RUST_VERSION} --profile minimal \
    && rustup component add clippy rustfmt

# uv + a Python 3.12 interpreter (pyo3 in the workspace pins abi3-py311, so the
# build-time interpreter must be >= 3.11). pyarrow + numpy are needed by the
# embedded fixture that `cargo test -p dora-ros2-bridge-python` loads.
RUN curl -LsSf https://astral.sh/uv/install.sh | sh \
    && /root/.local/bin/uv python install 3.12 \
    && /root/.local/bin/uv venv --seed -p 3.12 /opt/venv \
    && /root/.local/bin/uv pip install --python /opt/venv/bin/python pyarrow numpy

# pyo3 build-config introspects this interpreter; the venv ships libpython +
# headers and already has pyarrow/numpy for the test fixture.
ENV VIRTUAL_ENV=/opt/venv \
    PYO3_PYTHON=/opt/venv/bin/python

# The embedded-interpreter test (`cargo test -p dora-ros2-bridge-python`) links
# libpython3.12.so.1.0, which the uv standalone interpreter keeps in its own lib
# dir. Register that dir with the dynamic loader so the test binary finds it.
RUN PYLIB="$(/opt/venv/bin/python -c 'import sysconfig; print(sysconfig.get_config_var("LIBDIR"))')" \
    && echo "$PYLIB" > /etc/ld.so.conf.d/uv-python.conf \
    && ldconfig

# Source ROS2 in every interactive/login shell so AMENT_PREFIX_PATH (runtime
# message discovery) and `ros2` are available.
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc

WORKDIR /work
CMD ["bash"]
