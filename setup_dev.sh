#!/bin/bash
set -e

# Install Rust
if ! command -v cargo &> /dev/null; then
    echo "Installing Rust..."
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
    source "$HOME/.cargo/env"
else
    echo "Rust is already installed."
fi

# Install uv
if ! command -v uv &> /dev/null; then
    echo "Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
else
    echo "uv is already installed."
fi

# Install dora-cli
if ! command -v dora &> /dev/null; then
    echo "Installing dora-cli..."
    cargo install dora-cli
else
    echo "dora-cli is already installed."
fi

echo "Setup complete! Please restart your shell or run 'source \$HOME/.cargo/env' to use the tools."
