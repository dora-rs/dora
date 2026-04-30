# MAVLink 2 bridge — example dataflow

Three demos of the `dora-mavlink2-bridge-node` running alongside an
in-process MAVLink 2 simulator. The bridge + sim + emitter are shared
across all three; only the consumer that reads `bridge/heartbeat`
changes per language. No autopilot, SITL, or MAVProxy install is
required.

```
heartbeat-emitter ──heartbeat_cmd──▶ bridge ──udp:14550──▶ mavlink-sim
                                       │
                                       └─heartbeat──▶ telemetry-printer-{rust,python,cxx}
```

## Variants

| File | Consumer | Build / dependencies |
|------|----------|----------------------|
| `dataflow-rust.yml`   | Rust crate `telemetry-printer-rust`  | cargo only            |
| `dataflow-python.yml` | `telemetry_printer.py`               | cargo + uv (pyarrow)  |
| `dataflow-cxx.yml`    | C++ binary `telemetry-printer-cxx`   | cargo + clang + Arrow C++ (`pkg-config arrow`) |

The shared nodes:

* dora → MAVLink: `heartbeat-emitter` (Rust) produces an Arrow
  `StructArray` matching `HEARTBEAT_DATA::schema`. The bridge serialises
  it as a real MAVLink 2 frame and writes it on the UDP socket.
* MAVLink → dora: `mavlink-sim` (Rust) blasts a HEARTBEAT every 500 ms
  on the same UDP endpoint. The bridge decodes each frame and publishes
  it on the `heartbeat` output. Each variant's printer consumes that.

## Run it

### Rust variant (no extra deps)

```bash
dora run examples/mavlink2-bridge/dataflow-rust.yml --stop-after 5s
```

### Python variant (needs `--uv` for `pyarrow`)

```bash
dora run examples/mavlink2-bridge/dataflow-python.yml --uv --stop-after 5s
```

### C++ variant (needs Arrow C++ via `pkg-config`)

```bash
# One-shot: compiles the cxx printer + nodes, then dora-runs the dataflow.
cargo run --example mavlink2-bridge-cxx
```

The `cargo run --example` runner mirrors `cxx-arrow-dataflow`: it
discovers Arrow via `pkg-config arrow`, builds the cxx bridge headers,
compiles `telemetry-printer-cxx/main.cc` with `clang++`, then invokes
`dora_cli::run` against `dataflow-cxx.yml`.

### Networked / multi-daemon mode

Same as above, but with the explicit `up`/`build`/`start`/`stop`/`down`
lifecycle. Substitute the right YAML:

```bash
dora up
dora build examples/mavlink2-bridge/dataflow-rust.yml
dora start examples/mavlink2-bridge/dataflow-rust.yml --detach
# ... watch the logs ...
dora stop --all
dora down
```

## Talk to a real MAVLink endpoint

Stop the `mavlink-sim` node (delete it from the YAML) and point the
bridge at your endpoint:

```yaml
- id: bridge
  env:
    MAVLINK_BRIDGE_CONFIG: |
      endpoint: tcp://192.168.1.10:5760    # Pixhawk over telem-radio
      # or: udp://0.0.0.0:14550            # listen for any GCS
      # or: serial:///dev/ttyACM0?baud=115200
```

The `transport::connect` helper supports `tcp://`, `udp://`, and
`serial://` URLs (see `libraries/extensions/mavlink2-bridge/src/transport.rs`).

## Manual smoke (Tier 2): QGroundControl

The CI smoke tests prove the bridge talks to itself. To prove it
talks to a real-world MAVLink consumer, point [QGroundControl](https://qgroundcontrol.com/)
at the bridge once per release. QGC is a GUI app and intentionally
*not* automated — the value is human-eyeball confirmation that the
wire format is right, not a CI gate.

### Setup

1. **Drop the simulator** from any of the variant YAMLs (delete the
   `mavlink-sim` node block). The bridge will listen on UDP 14550;
   QGC will become the peer that sends/receives MAVLink frames.
2. **Switch the bridge to listen for QGC**:
   ```yaml
   - id: bridge
     env:
       MAVLINK_BRIDGE_CONFIG: |
         endpoint: udp://0.0.0.0:14550
   ```
   `0.0.0.0` lets QGC connect from another machine on the LAN; use
   `127.0.0.1` for purely local.
3. **Run the dataflow**:
   ```bash
   dora run examples/mavlink2-bridge/dataflow-rust.yml --stop-after 60s
   ```
4. **Connect QGC**: *Application Settings → Comm Links → Add → UDP*,
   port 14550, *Add Server Address* → `127.0.0.1:14550` (or the bridge
   host's LAN IP), *OK → Connect*.

### What you should see

| Observation | What it proves |
|-------------|----------------|
| QGC banner: "Vehicle 1 connected" within ~2 s | HEARTBEAT round-trip works (bridge → wire → QGC parse) |
| Custom-mode counter ticks up in QGC's vehicle info | `heartbeat-emitter` is producing valid Arrow that the bridge serialises correctly |
| QGC's "Click to Arm" button → no protocol error | The bridge's `command_long_cmd` input + COMMAND_ACK output round-trip a real GCS command |
| Telemetry HUD shows ATTITUDE / GPS values when you wire those into the dataflow | Per-message Arrow encoding matches MAVLink wire format byte-for-byte |

If QGC says "Vehicle has lost data link" the bridge isn't sending
HEARTBEATs at the rate QGC expects (~1 Hz). Check that the
`heartbeat-emitter` tick is firing and the bridge's writer loop
isn't blocked.

### What QGC does *not* prove

QGC validates the **wire layer** only — it has no view into the dora
side. It will not catch Arrow schema drift, dora-side queueing bugs,
or smoke-test regressions; those stay the job of the Tier 1 smoke
suite.
