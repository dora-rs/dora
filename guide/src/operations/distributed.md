# Distributed Deployment Guide

Dora supports deploying dataflows across multiple machines for multi-robot fleets, edge AI pipelines, and distributed robotics systems. This guide covers cluster management, node scheduling, binary distribution, auto-recovery, and operational best practices.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Features at a Glance](#features-at-a-glance)
- [Cluster Configuration Reference](#cluster-configuration-reference)
- [Cluster Commands Reference](#cluster-commands-reference)
  - [dora cluster up](#dora-cluster-up)
  - [dora cluster status](#dora-cluster-status)
  - [dora cluster down](#dora-cluster-down)
  - [dora cluster install](#dora-cluster-install)
  - [dora cluster uninstall](#dora-cluster-uninstall)
  - [dora cluster upgrade](#dora-cluster-upgrade)
  - [dora cluster restart](#dora-cluster-restart)
- [Node Scheduling](#node-scheduling)
  - [Machine-based scheduling](#machine-based-scheduling)
  - [Label-based scheduling](#label-based-scheduling)
  - [How resolve_daemon() works internally](#how-resolve_daemon-works-internally)
- [Binary Distribution](#binary-distribution)
- [systemd Service Management](#systemd-service-management)
- [Auto-Recovery](#auto-recovery)
- [Rolling Upgrade](#rolling-upgrade)
- [Use Cases](#use-cases)
- [Operations Runbook](#operations-runbook)
- [Deployment YAML Reference](#deployment-yaml-reference)
- [Best Practices](#best-practices)

---

## Overview

Dora's distributed architecture has three tiers:

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
              (one)          (per machine)     (user code)
```

- **CLI** sends control commands (build, start, stop) to the coordinator.
- **Coordinator** orchestrates daemons, resolves node placement, and manages dataflow lifecycle.
- **Daemons** run on each machine, spawning and supervising node processes.
- **Nodes** communicate via shared memory (same machine) or Zenoh pub-sub (cross-machine).

There are two paths to distributed deployment:

**Ad-hoc** -- manually start `dora daemon` on each machine, then use the coordinator for control. Good for development and testing. See [Distributed Deployments in the CLI reference](cli.md#distributed-deployments).

**Managed (cluster.yml)** -- define your cluster topology in a YAML file, then use `dora cluster` commands for SSH-based lifecycle management. This guide focuses on the managed path.

---

## Quick Start

1. Create a `cluster.yml`:

```yaml
coordinator:
  addr: 10.0.0.1
machines:
  - id: robot
    host: 10.0.0.2
    user: ubuntu
  - id: gpu-server
    host: 10.0.0.3
    user: ubuntu
```

2. Bring up the cluster:

```bash
dora cluster up cluster.yml
```

3. Start a dataflow:

```bash
dora start dataflow.yml --name my-app --attach
```

4. Check cluster health:

```bash
dora cluster status
```

5. Tear down:

```bash
dora cluster down
```

---

## Features at a Glance

| Feature | Command / Config | Description |
|---------|-----------------|-------------|
| Cluster lifecycle | `dora cluster up/status/down` | SSH-based daemon management from a single machine |
| Label scheduling | `_unstable_deploy.labels` | Route nodes to daemons by key-value labels |
| Binary distribution | `_unstable_deploy.distribute` | local, scp, or http strategies |
| systemd services | `dora cluster install/uninstall` | Persistent daemon services that survive reboots |
| Auto-recovery | Automatic | Re-spawn nodes when a daemon reconnects |
| Rolling upgrade | `dora cluster upgrade` | SCP binary + restart per-machine sequentially |
| Dataflow restart | `dora cluster restart` | Restart a running dataflow by name or UUID |

---

## Cluster Configuration Reference

A `cluster.yml` file defines the coordinator address and the set of machines in the cluster.

### Full Schema

```yaml
coordinator:
  addr: 10.0.0.1            # IP address the coordinator binds to (required)
  port: 6013                 # WebSocket port (default: 6013)

machines:
  - id: edge-01              # Unique machine identifier (required)
    host: 10.0.0.2           # SSH-reachable hostname or IP (required)
    user: ubuntu              # SSH user (optional, defaults to current user)
    labels:                   # Key-value labels for scheduling (optional)
      gpu: "true"
      arch: arm64

  - id: edge-02
    host: 10.0.0.3
    labels:
      arch: arm64
```

### Fields

**coordinator**

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `addr` | IP address | (required) | Address the coordinator binds to |
| `port` | u16 | `6013` | WebSocket port |

**machines[]**

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `id` | string | (required) | Unique machine identifier, used in `_unstable_deploy.machine` |
| `host` | string | (required) | SSH-reachable hostname or IP address |
| `user` | string | current user | SSH username |
| `labels` | map | empty | Key-value pairs for label-based scheduling |

### Validation Rules

- At least one machine must be defined.
- Machine IDs must be non-empty and unique.
- Machine hosts must be non-empty.
- Unknown fields are rejected (`deny_unknown_fields`).

### Example: 3-Machine GPU Cluster

```yaml
coordinator:
  addr: 192.168.1.1

machines:
  - id: coordinator-host
    host: 192.168.1.1
    labels:
      role: control

  - id: gpu-a100
    host: 192.168.1.10
    user: ml
    labels:
      gpu: a100
      arch: x86_64

  - id: jetson-01
    host: 192.168.1.20
    user: nvidia
    labels:
      gpu: jetson
      arch: arm64
```

---

## Cluster Commands Reference

All `dora cluster` commands operate on a `cluster.yml` file and use SSH to manage remote machines.

SSH options used: `BatchMode=yes`, `ConnectTimeout=10`, `StrictHostKeyChecking=accept-new`.

### dora cluster up

Bring up a multi-machine cluster from a cluster.yml file. Starts the coordinator locally, then SSH-es into each machine to start a daemon.

```
dora cluster up <PATH>
```

**Arguments:**

| Argument | Description |
|----------|-------------|
| `PATH` | Path to the cluster configuration file |

**Behavior:**

1. Loads and validates the cluster config.
2. Starts the coordinator locally on `addr:port`.
3. For each machine, SSH-es in and runs `nohup dora daemon --machine-id <id> --coordinator-addr <addr> --coordinator-port <port> [--labels k1=v1,k2=v2] --quiet`.
4. Polls until all expected daemons register with the coordinator (30s timeout).

**Example:**

```bash
$ dora cluster up cluster.yml
Starting coordinator on 10.0.0.1:6013...
Starting daemon on robot (ubuntu@10.0.0.2)... OK
Starting daemon on gpu-server (ubuntu@10.0.0.3)... OK
All 2 daemons connected.
```

### dora cluster status

Show the current status of the cluster. Displays connected daemons and active dataflow count.

```
dora cluster status [--coordinator-addr ADDR] [--coordinator-port PORT]
```

**Flags:**

| Flag | Default | Description |
|------|---------|-------------|
| `--coordinator-addr` | `localhost` | Coordinator hostname or IP |
| `--coordinator-port` | `6013` | Coordinator WebSocket port |

**Example:**

```bash
$ dora cluster status
DAEMON ID      LAST HEARTBEAT
robot          2s ago
gpu-server     1s ago

Active dataflows: 1
```

### dora cluster down

Tear down the cluster (coordinator and all daemons).

```
dora cluster down [--coordinator-addr ADDR] [--coordinator-port PORT]
```

Terminates all daemons and the coordinator process.

### dora cluster install

Install `dora-daemon` as a systemd service on each machine. SSH-es into each machine, writes a systemd unit file, and enables the service.

```
dora cluster install <PATH>
```

**Arguments:**

| Argument | Description |
|----------|-------------|
| `PATH` | Path to the cluster configuration file |

**Behavior:**

For each machine, creates and enables a systemd service named `dora-daemon-<id>`. The unit file:

```ini
[Unit]
Description=Dora Daemon (<id>)
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=dora daemon --machine-id <id> --coordinator-addr <addr> --coordinator-port <port> --labels k1=v1,k2=v2 --quiet
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**Example:**

```bash
$ dora cluster install cluster.yml
Installing dora-daemon-robot on ubuntu@10.0.0.2... OK
Installing dora-daemon-gpu-server on ubuntu@10.0.0.3... OK
2/2 succeeded.
```

### dora cluster uninstall

Uninstall dora-daemon systemd services from each machine. Stops, disables, and removes the systemd unit.

```
dora cluster uninstall <PATH>
```

**Behavior:**

For each machine, runs:

```bash
sudo systemctl stop dora-daemon-<id>
sudo systemctl disable dora-daemon-<id>
sudo rm -f /etc/systemd/system/dora-daemon-<id>.service
sudo systemctl daemon-reload
```

### dora cluster upgrade

Rolling upgrade: SCP the local `dora` binary to each machine and restart daemons. Processes machines sequentially to maintain availability.

```
dora cluster upgrade <PATH>
```

**Behavior:**

For each machine sequentially:

1. SCP the local `dora` binary to `/usr/local/bin/dora` on the target machine.
2. Restart the systemd service via `sudo systemctl restart dora-daemon-<id>`.
3. Poll the coordinator until the daemon reconnects (30s timeout, 500ms intervals).

Nodes on other machines continue running while each machine is being upgraded.

**Example:**

```bash
$ dora cluster upgrade cluster.yml
Upgrading robot (ubuntu@10.0.0.2)...
  SCP binary... OK
  Restart service... OK
  Waiting for reconnect... OK (3.2s)
Upgrading gpu-server (ubuntu@10.0.0.3)...
  SCP binary... OK
  Restart service... OK
  Waiting for reconnect... OK (2.8s)
2/2 succeeded.
```

### dora cluster restart

Restart a running dataflow by name or UUID. Stops the dataflow and immediately re-starts it using the stored descriptor (no YAML path needed).

```
dora cluster restart <DATAFLOW>
```

**Arguments:**

| Argument | Description |
|----------|-------------|
| `DATAFLOW` | Name or UUID of the dataflow to restart |

**Example:**

```bash
$ dora cluster restart my-app
Restarting dataflow `my-app`
dataflow restarted: a1b2c3d4-... -> e5f6a7b8-...
```

---

## Node Scheduling

When the coordinator receives a dataflow, it decides which daemon runs each node based on the `_unstable_deploy` section in the dataflow YAML. Resolution priority: **machine > labels > unnamed**.

### Machine-based scheduling

Assign a node to a specific machine by its `id` from `cluster.yml`:

```yaml
nodes:
  - id: camera
    _unstable_deploy:
      machine: robot
    path: ./camera-driver
    outputs:
      - frames
```

The coordinator looks up the daemon whose `machine-id` matches. If no matching daemon is connected, the deployment fails with: `no matching daemon for machine id "robot"`.

### Label-based scheduling

Assign a node by requiring specific labels on the target daemon:

```yaml
nodes:
  - id: inference
    _unstable_deploy:
      labels:
        gpu: "true"
    path: ./ml-model
    inputs:
      frames: camera/frames
    outputs:
      - predictions
```

The coordinator finds the first connected daemon whose labels are a **superset** of the required labels. All required key-value pairs must match exactly. If no daemon satisfies the requirements, deployment fails with: `no daemon matches labels {"gpu": "true"}`.

### Unassigned nodes

Nodes without an `_unstable_deploy` section (or with an empty one) are assigned to the first unnamed daemon -- one that connected without a `--machine-id` flag.

### How resolve_daemon() works internally

The coordinator resolves node placement in `coordinator/run/mod.rs`:

```
resolve_daemon(connections, deploy) -> DaemonId
  1. If deploy.machine is Some(id):
       -> look up daemon by machine-id
  2. Else if deploy.labels is non-empty:
       -> find first daemon where all required labels match
  3. Else:
       -> pick first unnamed daemon
```

The label matching function iterates over all connected daemons and checks that every required key-value pair exists in the daemon's label set (`conn.labels.get(k) == Some(v)`). This is a superset check: a daemon with `{gpu: "true", arch: "arm64", role: "edge"}` satisfies the requirement `{gpu: "true"}`.

---

## Binary Distribution

Control how node binaries are delivered to remote daemons via the `distribute` field.

### Local (default)

Each daemon builds from source on its own machine. This is the current default behavior.

```yaml
nodes:
  - id: my-node
    _unstable_deploy:
      machine: edge-01
      distribute: local
    path: ./my-node
```

### SCP mode

The CLI pushes the locally-built binary to the target machine via SSH/SCP before spawning.

```yaml
nodes:
  - id: my-node
    _unstable_deploy:
      machine: edge-01
      distribute: scp
    path: ./my-node
```

### HTTP mode

The coordinator runs an artifact store. Daemons pull binaries from the coordinator via HTTP before spawning.

```yaml
nodes:
  - id: my-node
    _unstable_deploy:
      machine: edge-01
      distribute: http
    path: ./my-node
```

Artifacts are served from `GET /api/artifacts/{build_id}/{node_id}` on the coordinator's WebSocket port. The endpoint requires authentication (Bearer token) and sanitizes node IDs to prevent path traversal.

### When to use each strategy

| Strategy | Best for | Tradeoffs |
|----------|----------|-----------|
| `local` | Homogeneous clusters, CI builds | Requires build toolchain on every machine |
| `scp` | Heterogeneous clusters, cross-compiled binaries | Requires SSH access from CLI to all machines |
| `http` | Air-gapped daemons, firewalled networks | Requires coordinator reachability from all daemons |

---

## systemd Service Management

For production deployments, install daemons as systemd services so they survive reboots and auto-restart on failure.

### Install

```bash
dora cluster install cluster.yml
```

Creates a systemd unit file on each machine (see [dora cluster install](#dora-cluster-install) for the full unit template). Key properties:

- **Restart=on-failure** with **RestartSec=5**: daemon auto-restarts if it crashes.
- **After=network-online.target**: waits for network before starting.
- **WantedBy=multi-user.target**: starts on boot.

### Uninstall

```bash
dora cluster uninstall cluster.yml
```

Stops, disables, and removes the unit file from each machine, then reloads the systemd daemon.

### Verifying service status

After install, check services directly:

```bash
ssh ubuntu@10.0.0.2 sudo systemctl status dora-daemon-robot
```

---

## Auto-Recovery

When a daemon disconnects and reconnects (e.g., after a network blip, machine reboot, or service restart), the coordinator automatically re-spawns any missing dataflows on that daemon.

### How it works

1. Daemon reconnects and sends a `StatusReport` listing its currently running dataflows.
2. Coordinator compares the report against its expected state (dataflows that should have nodes on this daemon).
3. For each running dataflow with nodes assigned to this daemon that the daemon did not report, the coordinator sends a `SpawnDataflowNodes` command to re-spawn the missing nodes.

### 30-second backoff

To prevent crash loops (e.g., a node that immediately crashes on spawn), recovery uses a per-daemon, per-dataflow backoff:

- After a recovery attempt, the coordinator records the timestamp.
- Subsequent recovery for the same daemon/dataflow pair is skipped until 30 seconds have elapsed.
- The backoff clears when the daemon reports the dataflow as running again.

This means a node that crashes immediately will only be re-spawned once every 30 seconds, not in a tight loop.

### Limitations

- Auto-recovery only applies to dataflows started via `dora start` (coordinator-managed). Local `dora run` dataflows are not tracked by the coordinator.
- Recovery re-spawns all nodes assigned to the reconnecting daemon, not individual nodes. For per-node restart on crash, use [restart policies](fault-tolerance.md#restart-policies).
- **Known issue ([#260](https://github.com/dora-rs/adora/issues/260)):** when the daemon's WebSocket connection to the coordinator drops, the daemon currently kills all running node processes before reconnecting. This means the coordinator's auto-recovery path re-spawns the nodes from scratch rather than reclaiming still-running processes. The net effect is a brief disruption (nodes restart) rather than seamless continuity. A fix to preserve running processes across reconnect cycles is planned.

---

## Rolling Upgrade

Upgrade the `dora` binary on all cluster machines with zero downtime using sequential per-machine upgrades.

### Process

```bash
dora cluster upgrade cluster.yml
```

For each machine, sequentially:

1. **SCP** the local `dora` binary to `/usr/local/bin/dora` on the target.
2. **Restart** the systemd service (`systemctl restart dora-daemon-<id>`).
3. **Poll** the coordinator until the daemon reconnects (30s timeout).

Because machines are upgraded one at a time, nodes on other machines continue running. After the daemon reconnects, auto-recovery re-spawns any dataflow nodes that were running on that machine.

### Prerequisites

- Daemons must be installed as systemd services (`dora cluster install`).
- The local `dora` binary must be compatible with the cluster's coordinator version.
- SSH access with `sudo` permissions on all target machines.

---

## Use Cases

### 1. Edge AI Pipeline (Robot + GPU Server)

A camera node runs on the robot, sends frames to a GPU server for inference, and results flow back to an actuator on the robot.

**cluster.yml:**

```yaml
coordinator:
  addr: 192.168.1.1

machines:
  - id: robot
    host: 192.168.1.10
    user: ubuntu
    labels:
      role: edge
  - id: gpu-server
    host: 192.168.1.20
    user: ml
    labels:
      gpu: "true"
```

**dataflow.yml:**

```yaml
nodes:
  - id: camera
    _unstable_deploy:
      machine: robot
    path: ./camera-driver
    outputs:
      - frames

  - id: inference
    _unstable_deploy:
      labels:
        gpu: "true"
    path: ./ml-model
    inputs:
      frames: camera/frames
    outputs:
      - predictions

  - id: actuator
    _unstable_deploy:
      machine: robot
    path: ./actuator-driver
    inputs:
      commands: inference/predictions
```

### 2. Multi-Robot Fleet

A central coordinator manages N robots with heterogeneous hardware. Label scheduling routes nodes to the right machines without hardcoding machine IDs.

**cluster.yml:**

```yaml
coordinator:
  addr: 10.0.0.1

machines:
  - id: bot-01
    host: 10.0.0.11
    user: robot
    labels:
      fleet: warehouse
      lidar: "true"

  - id: bot-02
    host: 10.0.0.12
    user: robot
    labels:
      fleet: warehouse
      camera: rgbd

  - id: bot-03
    host: 10.0.0.13
    user: robot
    labels:
      fleet: warehouse
      lidar: "true"
      camera: rgbd
```

**dataflow.yml:**

```yaml
nodes:
  - id: lidar-driver
    _unstable_deploy:
      labels:
        lidar: "true"
    path: ./lidar-driver
    outputs:
      - scans

  - id: camera-driver
    _unstable_deploy:
      labels:
        camera: rgbd
    path: ./camera-driver
    outputs:
      - frames
```

With this configuration, `lidar-driver` runs on bot-01 or bot-03, and `camera-driver` runs on bot-02 or bot-03.

### 3. CI/CD Pipeline for Robotics

Automate cluster management in CI:

```bash
# Setup
dora cluster install cluster.yml

# Deploy new version
dora cluster upgrade cluster.yml

# Run integration tests
dora start test-dataflow.yml --name integration-test --attach

# Monitor
dora cluster status
dora top

# Cleanup
dora stop integration-test
```

### 4. Development to Production

| Stage | Approach | Command |
|-------|----------|---------|
| **Local dev** | Single-process, no coordinator | `dora run dataflow.yml` |
| **Staging** | Ad-hoc daemons, manual setup | `dora up` + `dora daemon` on each machine |
| **Production** | Managed cluster, systemd services | `dora cluster install cluster.yml` |

---

## Operations Runbook

### Initial Setup Checklist

1. **SSH keys**: Distribute SSH keys so the CLI machine can reach all cluster machines without a password (`BatchMode=yes`).
2. **Dora binary**: Install the `dora` binary on all machines (same version).
3. **Network**: Ensure coordinator port (default 6013) is reachable from all machines. Ensure Zenoh ports are open between daemons for cross-machine node communication.
4. **cluster.yml**: Create the cluster configuration with correct IPs, users, and labels.

### Day-to-Day Operations

```bash
# Start a dataflow
dora start dataflow.yml --name my-app --attach

# List running dataflows
dora list

# Monitor resource usage
dora top

# View node logs
dora logs my-app <node-id> --follow

# Stop a dataflow
dora stop my-app

# Check cluster health
dora cluster status
```

### Upgrading

1. Build or download the new `dora` binary locally.
2. Run `dora cluster upgrade cluster.yml`.
3. Verify with `dora cluster status` that all daemons reconnected.
4. Running dataflows are automatically re-spawned via auto-recovery.

### Troubleshooting

**Daemon not connecting**

- Verify the coordinator is running and reachable: `curl http://<addr>:6013/api/health` (or check coordinator logs).
- Check daemon logs: `journalctl -u dora-daemon-<id> -f` (systemd) or the daemon's stderr output (ad-hoc).
- Confirm the `--coordinator-addr` and `--coordinator-port` match the coordinator's actual bind address.

**SSH failures during cluster commands**

- Ensure `ssh -o BatchMode=yes <user>@<host> echo ok` works from the CLI machine.
- Check that `StrictHostKeyChecking=accept-new` is acceptable for your environment (first connection auto-accepts the host key).
- Verify the `user` field in `cluster.yml` matches a valid SSH user on the target.

**Label mismatch errors**

- Error: `no daemon matches labels {"gpu": "true"}`.
- Check that the daemon was started with the correct `--labels` flag.
- Run `dora cluster status` to see connected daemons. Labels are set at daemon startup from `cluster.yml` and cannot be changed at runtime.

**Auto-recovery not triggering**

- Auto-recovery only applies to coordinator-managed dataflows (`dora start`), not `dora run`.
- Check coordinator logs for `auto-recovery: re-spawning` messages.
- If the node crashes immediately, recovery is throttled to once every 30 seconds per daemon per dataflow.

---

## Deployment YAML Reference

The `_unstable_deploy` section on each node controls placement and distribution. All fields are optional.

```yaml
nodes:
  - id: my-node
    _unstable_deploy:
      machine: edge-01                # Target machine ID from cluster.yml
      labels:                          # Label requirements (superset match)
        gpu: "true"
        arch: arm64
      distribute: local                # local | scp | http
      working_dir: /opt/my-app         # Working directory on the target machine
    path: ./my-node
```

### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `machine` | string | none | Target machine ID. Takes priority over labels. |
| `labels` | map | empty | Required daemon labels. All key-value pairs must match. |
| `distribute` | string | `local` | Binary distribution strategy: `local`, `scp`, or `http`. |
| `working_dir` | path | none | Working directory on the target machine. |

### Resolution priority

1. **machine** -- if set, the node is assigned to the daemon with that machine ID.
2. **labels** -- if set (and machine is not), the node is assigned to the first daemon whose labels are a superset of the required labels.
3. **Fallback** -- if neither is set, the node is assigned to the first unnamed (no machine-id) daemon.

---

## Best Practices

- **Use labels over machine IDs** for flexibility. Labels decouple your dataflow from specific machines, making it easier to add, remove, or replace hardware.
- **Use systemd install for production**. Daemon services survive reboots and auto-restart on failure with `Restart=on-failure`.
- **Use coordinator persistence** (`dora coordinator --store redb`) with clusters so the coordinator survives restarts. See [Coordinator State Persistence](fault-tolerance.md#coordinator-state-persistence).
- **Set restart policies on nodes** for per-node resilience. Combine with auto-recovery for defense in depth. See [Restart Policies](fault-tolerance.md#restart-policies).
- **Monitor with multiple tools**: `dora cluster status` for daemon health, `dora top` for resource usage, `dora logs` for node output.
- **Test locally first**. Develop with `dora run dataflow.yml`, then deploy to a cluster. The same dataflow YAML works in both modes -- `_unstable_deploy` fields are ignored in local mode.
- **Use rolling upgrades** instead of stopping the entire cluster. `dora cluster upgrade` processes one machine at a time to maintain availability.
- **Keep cluster.yml in version control** alongside your dataflow definitions.