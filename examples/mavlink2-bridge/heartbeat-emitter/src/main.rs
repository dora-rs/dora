//! Source node for the `mavlink2-bridge` example dataflow.
//!
//! On every tick the node builds a `HEARTBEAT_DATA` struct, encodes it
//! into the bridge's Arrow schema via `MavlinkArrow::to_record_batch`,
//! and publishes it on output `heartbeat_cmd`. The bridge is wired to
//! consume that input and forward it as a real MAVLink frame.
//!
//! This demonstrates the dora → MAVLink direction of the bridge.

use std::sync::Arc;

use arrow::array::StructArray;
use dora_mavlink2_bridge::{
    MavlinkArrow,
    mavlink::common::{HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavState, MavType},
};
use dora_node_api::{
    DoraNode, Event, MetadataParameters, arrow::array::ArrayRef, dora_core::config::DataId,
};
use eyre::{Result, eyre};

fn main() -> Result<()> {
    let (mut node, mut events) =
        DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let output = DataId::from("heartbeat_cmd".to_owned());
    let mut tick: u32 = 0;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, .. } if id.as_str() == "tick" => {
                let heartbeat = HEARTBEAT_DATA {
                    custom_mode: tick,
                    mavtype: MavType::MAV_TYPE_QUADROTOR,
                    autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
                    base_mode: MavModeFlag::empty(),
                    system_status: MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                };
                let batch = MavlinkArrow::to_record_batch(&heartbeat)
                    .map_err(|e| eyre!("encoding HEARTBEAT to Arrow: {e}"))?;
                let arr: ArrayRef = Arc::new(StructArray::from(batch));
                node.send_output(output.clone(), MetadataParameters::default(), arr)
                    .map_err(|e| eyre!("send_output heartbeat_cmd: {e}"))?;
                tracing::info!(tick, "emitted heartbeat_cmd");
                tick = tick.wrapping_add(1);
            }
            Event::Input { .. } => {}
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
