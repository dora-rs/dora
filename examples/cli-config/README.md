# Dora CLI Configuration Guide

This guide describes how to use the `dora config` command to manage your Dora configuration.

## Overview

The `dora config` command allows you to manage configuration settings for the Dora coordinator, logging, and other options. It supports a layered configuration system where project-specific settings override global settings, and environment variables override everything.

## Configuration Layers

1.  **Environment Variables**: Highest priority. (e.g., `DORA_COORDINATOR__ADDR`)
2.  **Project Configuration**: Priority over global. Stored in `./dora.toml`.
3.  **Global Configuration**: Lowest priority. Stored in `~/.dora/config.toml`.

## Usage

### List Configuration
List all active configuration values (merged from all sources).

```bash
dora config list
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

Common keys include:
- `coordinator.addr`: IP address of the Dora coordinator.
- `coordinator.port`: Port number of the Dora coordinator.
- `log.level`: Logging level (e.g., `info`, `debug`, `error`).
- `ui.theme`: UI theme setting.
- `ui.color`: Enable/disable colored output.

## Environment Variables
Any configuration key can be set via environment variables using the `DORA_` prefix and double underscores `__` for nested keys.

Example:
`coordinator.addr` -> `DORA_COORDINATOR__ADDR`
