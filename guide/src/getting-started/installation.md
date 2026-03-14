# Installation

## From crates.io (recommended)

```bash
cargo install adora-cli           # CLI (adora command)
pip install adora-rs              # Python node/operator API
```

## From source

```bash
git clone https://github.com/dora-rs/adora.git
cd adora
cargo build --release -p adora-cli
PATH=$PATH:$(pwd)/target/release

# Python API (requires maturin >= 1.8: pip install maturin)
# Must run from the package directory for dependency resolution
cd apis/python/node && maturin develop --uv && cd ../../..
```

## Platform installers

**macOS / Linux:**

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/dora-rs/adora/releases/latest/download/adora-cli-installer.sh | sh
```

**Windows:**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/adora/releases/latest/download/adora-cli-installer.ps1 | iex"
```

## Build features

| Feature | Description | Default |
|---------|-------------|---------|
| `tracing` | OpenTelemetry tracing support | Yes |
| `metrics` | OpenTelemetry metrics collection | No |
| `python` | Python operator support (PyO3) | No |
| `redb-backend` | Persistent coordinator state (redb) | No |
| `prometheus` | Prometheus `/metrics` endpoint on coordinator | No |

```bash
cargo install adora-cli --features redb-backend
```

## Verify

```bash
adora --version
adora status
```
