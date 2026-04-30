//! Rust consumer of the `mavlink2-bridge` example.
//!
//! Subscribes to `bridge/heartbeat` (a 1-row Arrow `StructArray` matching
//! `HEARTBEAT_DATA::schema`) and prints each decoded frame. Demonstrates
//! the MAVLink -> dora -> Rust path; pair with `mavlink-sim` and the
//! bridge node to round-trip without an autopilot.

use arrow::array::{AsArray, RecordBatch};
use dora_mavlink2_bridge::{MavlinkArrow, mavlink::common::HEARTBEAT_DATA};
use dora_node_api::{DoraNode, Event, arrow::array::ArrayRef};
use eyre::{Result, eyre};

fn decode(value: ArrayRef) -> Result<HEARTBEAT_DATA> {
    let struct_array = value
        .as_struct_opt()
        .ok_or_else(|| eyre!("expected StructArray, got {:?}", value.data_type()))?;
    let batch = RecordBatch::from(struct_array);
    HEARTBEAT_DATA::from_record_batch(&batch).map_err(|e| eyre!("decoding HEARTBEAT: {e}"))
}

fn main() -> Result<()> {
    let (_node, mut events) = DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let mut received: u64 = 0;
    while let Some(event) = events.recv() {
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
    Ok(())
}
