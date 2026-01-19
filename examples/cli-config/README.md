# Dora CLI Configuration Guide

This guide describes how to use the `dora config` command to manage your Dora configuration.

## Overview

The `dora config` command allows you to manage configuration settings for the Dora coordinator and other options. It supports a layered configuration system where project-specific settings override global settings.

**Key Features:**
- **Type-safe configuration**: Values are validated when set (e.g., IP addresses must be valid, ports must be 1-65535)
- **Unknown field detection**: Typos in configuration keys are caught immediately
- **Automatic usage**: All CLI commands automatically use config values as defaults when command-line arguments aren't provided
- **Layered configuration**: Project settings override global settings

## Configuration Layers

Configuration values are resolved in the following priority order (highest to lowest):

1.  **Command-line arguments**: Highest priority (e.g., `--coordinator-addr 192.168.1.100`)
2.  **Project Configuration**: Stored in `./dora.toml`
3.  **Global Configuration**: Stored in `~/.dora/config.toml`
4.  **Hardcoded defaults**: Lowest priority (e.g., `127.0.0.1:53290`)

## Usage

### List Configuration
List all active configuration values.

```bash
dora config list
```

Example output:
```
coordinator.addr = "192.168.1.100"
coordinator.port = 53290
```

### Get Configuration
Get a specific configuration value.

```bash
dora config get coordinator.addr
```

### Set Configuration
Set a value in the global configuration (`~/.dora/config.toml`).

```bash
dora config set coordinator.addr 192.168.1.100
dora config set coordinator.port 53290
```

Set a value in the project configuration (`./dora.toml`).

```bash
dora config set --local coordinator.addr 127.0.0.1
```

### Unset Configuration
Remove a value from the global configuration.

```bash
dora config unset coordinator.addr
```

Remove a value from the project configuration.

```bash
dora config unset --local coordinator.addr
```

## Supported Configuration Keys

Currently supported keys:
- `coordinator.addr`: IP address of the Dora coordinator (validated as valid IP)
- `coordinator.port`: Port number of the Dora coordinator (validated as 1-65535)

## Validation

The config system validates all values when they are set:

- **IP addresses** must be valid IPv4 or IPv6 addresses
- **Port numbers** must be between 1 and 65535
- **Unknown keys** are rejected with a helpful error message listing valid keys
- **Typos** in configuration files are detected when loading (e.g., `potr` instead of `port`)

## Integration with CLI Commands

All commands that connect to a coordinator automatically use config values:

- `dora list` - List running dataflows
- `dora logs` - View dataflow logs
- `dora start` - Start a dataflow
- `dora stop` - Stop a dataflow
- `dora destroy` - Destroy coordinator and daemon
- `dora daemon` - Run a daemon
- `dora inspect top` - Monitor node resources
- `dora topic echo/hz/info` - Topic inspection commands
- And more...

Example: If you set `coordinator.addr` to `192.168.1.100`, you can run:
```bash
dora list  # Automatically connects to 192.168.1.100:53290
```

Instead of:
```bash
dora list --coordinator-addr 192.168.1.100  # No longer needed!
```

## Example Workflow

1. Set global defaults for your usual coordinator:
```bash
dora config set coordinator.addr 192.168.1.100
dora config set coordinator.port 53290
```

2. For a specific project that uses a local coordinator:
```bash
cd my-project
dora config set --local coordinator.addr 127.0.0.1
dora config set --local coordinator.port 8080
```

3. Now all commands in `my-project/` will use `127.0.0.1:8080`, while commands elsewhere use `192.168.1.100:53290`.

## Configuration File Format

Global config (`~/.dora/config.toml`):
```toml
[coordinator]
addr = "192.168.1.100"
port = 53290
```

Project config (`./dora.toml`):
```toml
[coordinator]
addr = "127.0.0.1"
port = 8080
```

