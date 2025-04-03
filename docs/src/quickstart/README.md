# Quick Start Guide

## Prerequisites
- Rust toolchain (latest stable version)
- Python 3.8 or higher
- Git

## System Requirements
- Operating System: Windows, macOS, or Linux
- Memory: Minimum 4GB RAM (8GB recommended)
- Storage: At least 1GB free space
- Network: Internet connection for initial setup

## Quick Installation

### Windows
```powershell
# Install Rust
winget install Rust.Rustup

# Clone and setup Dora
git clone https://github.com/dora-rs/dora.git
cd dora
./install.ps1
```

### macOS/Linux
```bash
# Install Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# Clone and setup Dora
git clone https://github.com/dora-rs/dora.git
cd dora
./install.sh
```

## Your First Dora Application

1. Create a new project:
```bash
dora new my-first-app
cd my-first-app
```

2. Run the example:
```bash
dora run
```

## Next Steps
- [Basic Examples](../examples/basic/README.md)
- [Python API Guide](../api/python/README.md)
- [Rust API Guide](../api/rust/README.md)
- [Architecture Overview](../architecture/README.md) 