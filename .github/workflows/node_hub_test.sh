#!/bin/bash
set -euo

# Check if we are running in a GitHub Actions environment
CI=${GITHUB_ACTIONS:-false}

# List of ignored modules 
ignored_folders=("dora-parler" "dora-opus" "dora-internvl" "dora-magma")

# Skip test
skip_test_folders=("dora-internvl" "dora-parler" "dora-keyboard" "dora-microphone" "terminal-input" "dora-magma")

# Get current working directory
dir=$(pwd)

# Get the base name of the directory (without the path)
base_dir=$(basename "$dir")

export GIT_LFS_SKIP_SMUDGE=1 
# Large node list requiring space cleanup
large_node=("dora-phi4")

export PYTEST_ADDOPTS="-x"

# Check if the current directory is in the large node list and if we're in the CI environment
if [[ " ${large_node[@]} " =~ " ${base_dir} " ]] && [[ "$CI" == "true" ]]; then
    echo "Running cleanup for $base_dir..."
    sudo rm -rf /opt/hostedtoolcache/CodeQL || :
    # 1.4GB
    sudo rm -rf /opt/hostedtoolcache/go || :
    # 489MB
    sudo rm -rf /opt/hostedtoolcache/PyPy || :
    # 376MB
    sudo rm -rf /opt/hostedtoolcache/node || :
    # Remove Web browser packages
    sudo apt purge -y \
    firefox \
    google-chrome-stable \
    microsoft-edge-stable
    sudo rm -rf /usr/local/lib/android/ 
    sudo rm -rf /usr/share/dotnet/
    sudo rm -rf /opt/ghc/
fi

# Check if the directory name is in the ignored list
if [[ " ${ignored_folders[@]} " =~ " ${base_dir} " ]]; then
    echo "Skipping $base_dir as we cannot test it on the CI..."
else
    # IF job is mixed rust-python job and is on linux 
    if [[ -f "Cargo.toml" && -f "pyproject.toml" &&  "$(uname)" = "Linux" ]]; then
        echo "Running build and tests for Rust project in $dir..."

        cargo check
        cargo clippy
        cargo build
        cargo test

        pip install "maturin[zig, patchelf]"
        maturin build --release --compatibility manylinux_2_28 --zig
        # If GITHUB_EVENT_NAME is release or workflow_dispatch, publish the wheel on multiple platforms
        if [ "$GITHUB_EVENT_NAME" == "release" ] || [ "$GITHUB_EVENT_NAME" == "workflow_dispatch" ]; then
            # Free up ubuntu space
            sudo apt-get clean
            sudo rm -rf /usr/local/lib/android/ 
            sudo rm -rf /usr/share/dotnet/
            sudo rm -rf /opt/ghc/

            maturin publish --skip-existing --compatibility manylinux_2_28 --zig
            # aarch64-unknown-linux-gnu
            rustup target add aarch64-unknown-linux-gnu
            maturin publish --target aarch64-unknown-linux-gnu --skip-existing --zig  --compatibility manylinux_2_28
                
            # armv7-unknown-linux-musleabihf
            rustup target add armv7-unknown-linux-musleabihf
            # If GITHUB_EVENT_NAME is release or workflow_dispatch, publish the wheel
            maturin publish --target armv7-unknown-linux-musleabihf --skip-existing --zig

            # x86_64-pc-windows-gnu
            rustup target add x86_64-pc-windows-gnu
            # If GITHUB_EVENT_NAME is release or workflow_dispatch, publish the wheel
            maturin publish --target x86_64-pc-windows-gnu --skip-existing 
        fi

    elif [[ -f "Cargo.toml" && -f "pyproject.toml" &&  "$(uname)" = "Darwin" ]]; then
        pip install "maturin[zig, patchelf]"
        # aarch64-apple-darwin
        maturin build  --release
        # If GITHUB_EVENT_NAME is release or workflow_dispatch, publish the wheel
        if [ "$GITHUB_EVENT_NAME" == "release" ] || [ "$GITHUB_EVENT_NAME" == "workflow_dispatch" ]; then
            maturin publish --skip-existing
        fi

    elif [[ "$(uname)" = "Linux" ]] || [[ "$CI" == "false" ]]; then
        if [ -f "$dir/Cargo.toml" ]; then
            echo "Running build and tests for Rust project in $dir..."
            cargo check
            cargo clippy
            cargo build
            cargo test

            if [ "$GITHUB_EVENT_NAME" == "release" ] || [ "$GITHUB_EVENT_NAME" == "workflow_dispatch" ]; then
                cargo publish
            fi
        else
            if [ -f "$dir/pyproject.toml" ]; then
            echo "CI: Installing in $dir..."
            uv venv --seed -p 3.11
            uv pip install .
            echo "CI: Running Linting in $dir..."
            uv run ruff check .
            echo "CI: Running Pytest in $dir..."
            # Skip test for some folders
            if [[ " ${skip_test_folders[@]} " =~ " ${base_dir} " ]]; then
                echo "Skipping tests for $base_dir..."
            else
                uv run pytest
            fi
            if [ "${GITHUB_EVENT_NAME:-false}" == "release" ] || [ "${GITHUB_EVENT_NAME:-false}" == "workflow_dispatch" ]; then
                uv build
                uv publish --check-url https://pypi.org/simple
            fi
            fi
        fi 
    fi 
fi