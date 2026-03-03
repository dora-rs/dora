# Dataflow YAML

Dataflows are defined in YAML files. Each node declares its binary/script, inputs, and outputs.

## Minimal Example

```yaml
nodes:
  - id: sender
    path: sender.py
    outputs:
      - message

  - id: receiver
    path: receiver.py
    inputs:
      message: sender/message
```

## Full Schema

```yaml
# Dataflow-level settings
health_check_interval: 5.0    # health check sweep interval in seconds (default: 5.0)

nodes:
  - id: my-node                 # unique identifier (required)
    name: "My Node"             # human-readable name (optional)
    description: "..."          # description (optional)

    # --- Source (pick one) ---
    path: ./target/debug/my-node          # local executable
    # path: https://example.com/node.zip  # download from URL
    # git: https://github.com/org/repo.git  # build from git
    #   branch: main            # git branch (mutually exclusive with tag/rev)
    #   tag: v1.0               # git tag
    #   rev: abc123             # git commit hash

    # --- Build ---
    build: cargo build -p my-node   # shell command to build (optional)

    # --- Inputs ---
    inputs:
      # Short form: source_node/output_id
      tick: adora/timer/millis/100
      data: other-node/output

      # Long form with options
      sensor_data:
        source: sensor/frames
        queue_size: 10            # input buffer size (default: 10)
        input_timeout: 5.0        # circuit breaker timeout in seconds

    # --- Outputs ---
    outputs:
      - processed
      - status

    # --- Environment ---
    env:
      MY_VAR: "value"
      FROM_ENV:
        __adora_env: HOST_VAR     # read from host environment
    args: "--verbose"             # command-line arguments

    # --- Fault tolerance ---
    restart_policy: on-failure    # never (default) | on-failure | always
    max_restarts: 5               # 0 = unlimited
    restart_delay: 1.0            # initial backoff in seconds
    max_restart_delay: 30.0       # backoff cap in seconds
    restart_window: 300.0         # reset counter after N seconds
    health_check_timeout: 30.0    # kill if no activity for N seconds

    # --- Logging ---
    min_log_level: info           # source-level filter (daemon-side)
    send_stdout_as: raw_output    # route raw stdout as data output
    send_logs_as: log_entries     # route structured logs as data output
    max_log_size: "50MB"          # rotate log files at this size
    max_rotated_files: 5          # number of rotated files to keep (1-100)

    # --- Deployment ---
    _unstable_deploy:
      machine: A                  # target machine/daemon ID

# Debug settings
_unstable_debug:
  publish_all_messages_to_zenoh: true   # required for topic echo/hz/info
```

## Built-in Timer Nodes

Timers are virtual nodes that emit ticks at fixed intervals:

```yaml
inputs:
  tick: adora/timer/millis/100   # every 100ms
  slow: adora/timer/millis/1000  # every 1s
  fast: adora/timer/hz/30        # 30 Hz (~33ms)
```

## Input Format

Inputs subscribe to another node's output using the format `<node-id>/<output-name>`:

```yaml
inputs:
  image: camera/frames           # subscribes to "frames" output of "camera" node
```

## Operator Nodes

Operators run in-process inside a shared runtime (no separate process):

```yaml
nodes:
  # Single operator (shorthand)
  - id: detector
    operator:
      python: detect.py
      build: pip install -r requirements.txt
      inputs:
        image: camera/frames
      outputs:
        - bbox

  # Multiple operators sharing a runtime
  - id: runtime-node
    operators:
      - id: preprocessor
        shared-library: ../../target/debug/libpreprocess
        inputs:
          raw: sensor/data
        outputs:
          - processed
      - id: analyzer
        shared-library: ../../target/debug/libanalyze
        inputs:
          data: runtime-node/preprocessor/processed
        outputs:
          - result
```

## Distributed Deployment

Assign nodes to specific machines using `_unstable_deploy`:

```yaml
nodes:
  - id: camera-driver
    _unstable_deploy:
      machine: robot-arm
    path: ./target/debug/camera
    outputs:
      - frames

  - id: ml-inference
    _unstable_deploy:
      machine: gpu-server
    path: ./target/debug/inference
    inputs:
      frames: camera-driver/frames
    outputs:
      - predictions
```

When nodes are on different machines, communication automatically switches from shared memory to Zenoh pub/sub.
