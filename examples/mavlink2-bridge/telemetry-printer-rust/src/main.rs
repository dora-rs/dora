//! Rust consumer of the `mavlink2-bridge` example.
//!
//! Subscribes to `bridge/heartbeat` (a 1-row Arrow `StructArray` matching
//! `HEARTBEAT_DATA::schema`) and prints each decoded frame. Demonstrates
//! the MAVLink -> dora -> Rust path; pair with `mavlink-sim` and the
//! bridge node to round-trip without an autopilot.

use arrow::array::{AsArray, RecordBatch};
use dora_mavlink2_bridge::{MavlinkArrow, mavlink::common::HEARTBEAT_DATA};
use dora_node_api::{DoraNode, Event, arrow::array::ArrayRef};
use eyre::{Result, bail, eyre};
use std::time::{Duration, Instant};

/// How long the sink waits for the first HEARTBEAT before declaring the
/// bridge silent. The example's `mavlink-sim` emits a HEARTBEAT every
/// 500 ms, so anything past a few seconds means the bridge connected
/// but published nothing — the regression the smoke tests need to
/// catch. The networked smoke harness has a 30 s budget and only
/// signals STOP at cleanup, so the sink must fail fast on its own
/// rather than wait for STOP to detect a 0-heartbeat run.
const FIRST_HEARTBEAT_DEADLINE: Duration = Duration::from_secs(8);
/// Polling cadence for `recv_timeout`; small enough that the warmup
/// deadline above is honoured tightly.
const RECV_POLL: Duration = Duration::from_millis(200);

fn decode(value: ArrayRef) -> Result<HEARTBEAT_DATA> {
    let struct_array = value
        .as_struct_opt()
        .ok_or_else(|| eyre!("expected StructArray, got {:?}", value.data_type()))?;
    let batch = RecordBatch::from(struct_array);
    HEARTBEAT_DATA::from_record_batch(&batch).map_err(|e| eyre!("decoding HEARTBEAT: {e}"))
}

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let started = Instant::now();
    let mut received: u64 = 0;
    loop {
        if received == 0 && started.elapsed() > FIRST_HEARTBEAT_DEADLINE {
            bail!(
                "telemetry-printer-rust: no HEARTBEAT received within {:?}; \
                 bridge appears silent — smoke assertion failed",
                FIRST_HEARTBEAT_DEADLINE
            );
        }
        let Some(event) = events.recv_timeout(RECV_POLL) else {
            continue;
        };
        match event {
            Event::Input { id, data, .. } if id.as_str() == "heartbeat" => {
                let arr: ArrayRef = data.into();
                let hb = decode(arr)?;
                received += 1;
                println!(
                    "telemetry-printer-rust: heartbeat #{received} \
                     custom_mode={} mavtype={:?} autopilot={:?} \
                     base_mode={:?} system_status={:?} mavlink_version={}",
                    hb.custom_mode,
                    hb.mavtype,
                    hb.autopilot,
                    hb.base_mode,
                    hb.system_status,
                    hb.mavlink_version,
                );
            }
            Event::Input { .. } => {}
            Event::Stop(_) => {
                println!("telemetry-printer-rust: STOP after {received} heartbeats");
                break;
            }
            _ => {}
        }
    }
    // Belt-and-suspenders for the local-mode smoke test, which uses
    // `dora run --stop-after`: STOP can arrive before the warmup
    // deadline above, so guard the post-loop case too.
    if received == 0 {
        bail!(
            "telemetry-printer-rust: bridge produced 0 heartbeats before STOP — \
             smoke assertion failed (expected ≥1 decoded HEARTBEAT)"
        );
    }
    Ok(())
}
