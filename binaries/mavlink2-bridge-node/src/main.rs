//! MAVLink 2 ↔ dora bridge node.
//!
//! This binary is spawned by the dora daemon as a normal node. It opens
//! a MAVLink 2 connection (TCP/UDP/serial), decodes incoming common-
//! dialect frames into Arrow `StructArray` rows, and publishes them on
//! per-message dora outputs. Inputs named `<message>_cmd` are decoded
//! back into MAVLink frames and sent on the same connection.
//!
//! Configuration comes from the `MAVLINK_BRIDGE_CONFIG` env var as YAML:
//!
//! ```yaml
//! endpoint: tcp://127.0.0.1:5760
//! system_id: 255
//! component_id: 1
//! ```
//!
//! ## Outputs (telemetry, MAVLink → dora)
//!
//! | dora output            | MAVLink message       |
//! |------------------------|-----------------------|
//! | `heartbeat`            | HEARTBEAT             |
//! | `sys_status`           | SYS_STATUS            |
//! | `system_time`          | SYSTEM_TIME           |
//! | `attitude`             | ATTITUDE              |
//! | `attitude_quaternion`  | ATTITUDE_QUATERNION   |
//! | `local_position_ned`   | LOCAL_POSITION_NED    |
//! | `global_position_int`  | GLOBAL_POSITION_INT   |
//! | `gps_raw_int`          | GPS_RAW_INT           |
//! | `rc_channels`          | RC_CHANNELS           |
//! | `servo_output_raw`     | SERVO_OUTPUT_RAW      |
//! | `command_long`         | COMMAND_LONG          |
//! | `command_ack`          | COMMAND_ACK           |
//! | `mission_current`      | MISSION_CURRENT       |
//!
//! ## Inputs (commands, dora → MAVLink)
//!
//! | dora input             | MAVLink message       |
//! |------------------------|-----------------------|
//! | `heartbeat_cmd`        | HEARTBEAT             |
//! | `command_long_cmd`     | COMMAND_LONG          |
//!
//! Other message types are read-only telemetry; sending them inbound is
//! out of scope for this PR.

use dora_mavlink2_bridge::{
    MavlinkArrow,
    mavlink::common::{COMMAND_LONG_DATA, HEARTBEAT_DATA},
    transport,
};
use dora_node_api::{
    DoraNode, Event, MetadataParameters,
    arrow::array::{ArrayRef, AsArray, RecordBatch, StructArray},
    dora_core::config::DataId,
    flume,
};
use eyre::{Context, Result, bail, eyre};
use mavlink::{MavConnection, MavHeader, Message, common::MavMessage};
use serde::Deserialize;
use std::{sync::Arc, time::Duration};
use url::Url;

const POLL_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Debug, Deserialize)]
struct Config {
    endpoint: String,
    #[serde(default = "default_system_id")]
    system_id: u8,
    #[serde(default = "default_component_id")]
    component_id: u8,
}

fn default_system_id() -> u8 {
    255
}

fn default_component_id() -> u8 {
    1
}

impl Config {
    fn from_env() -> Result<Self> {
        let yaml = std::env::var("MAVLINK_BRIDGE_CONFIG")
            .context("MAVLINK_BRIDGE_CONFIG env var must be set")?;
        let cfg: Self = serde_yaml::from_str(&yaml).context("parsing MAVLINK_BRIDGE_CONFIG")?;
        Ok(cfg)
    }

    fn parsed_endpoint(&self) -> Result<Url> {
        Url::parse(&self.endpoint)
            .with_context(|| format!("parsing endpoint URL '{}'", self.endpoint))
    }
}

/// Reader thread: blocks on `conn.recv()`, converts each supported
/// message into a 1-row Arrow `StructArray`, and forwards it on `tx`.
/// Returns when `tx` is dropped (main thread exited).
fn run_reader(
    conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>,
    tx: flume::Sender<(DataId, ArrayRef)>,
) -> Result<()> {
    /// Convert + queue a single decoded message; on a closed channel,
    /// short-circuit the reader by `return`-ing from the surrounding
    /// `run_reader` call.
    macro_rules! emit {
        ($data:expr, $output_name:literal) => {{
            let batch = MavlinkArrow::to_record_batch(&$data)?;
            let arr: ArrayRef = Arc::new(StructArray::from(batch));
            if tx
                .send((DataId::from($output_name.to_string()), arr))
                .is_err()
            {
                return Ok(()); // main thread gone, drain & exit
            }
        }};
    }

    loop {
        match conn.recv() {
            Ok((_header, msg)) => match msg {
                MavMessage::HEARTBEAT(d) => emit!(d, "heartbeat"),
                MavMessage::SYS_STATUS(d) => emit!(d, "sys_status"),
                MavMessage::SYSTEM_TIME(d) => emit!(d, "system_time"),
                MavMessage::ATTITUDE(d) => emit!(d, "attitude"),
                MavMessage::ATTITUDE_QUATERNION(d) => emit!(d, "attitude_quaternion"),
                MavMessage::LOCAL_POSITION_NED(d) => emit!(d, "local_position_ned"),
                MavMessage::GLOBAL_POSITION_INT(d) => emit!(d, "global_position_int"),
                MavMessage::GPS_RAW_INT(d) => emit!(d, "gps_raw_int"),
                MavMessage::RC_CHANNELS(d) => emit!(d, "rc_channels"),
                MavMessage::SERVO_OUTPUT_RAW(d) => emit!(d, "servo_output_raw"),
                MavMessage::COMMAND_LONG(d) => emit!(d, "command_long"),
                MavMessage::COMMAND_ACK(d) => emit!(d, "command_ack"),
                MavMessage::MISSION_CURRENT(d) => emit!(d, "mission_current"),
                other => {
                    tracing::trace!(msg_id = other.message_id(), "skipping unhandled message");
                }
            },
            Err(e) => {
                tracing::warn!("mavlink recv error: {e}");
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    }
}

/// Decode an incoming Arrow `StructArray` for type `T` from a dora
/// `Event::Input` payload.
fn decode_input<T: MavlinkArrow>(data: &ArrayRef, input: &str) -> Result<T> {
    let struct_array = data.as_struct_opt().ok_or_else(|| {
        eyre!(
            "expected StructArray for input '{input}', got {:?}",
            data.data_type()
        )
    })?;
    let batch = RecordBatch::from(struct_array);
    T::from_record_batch(&batch).with_context(|| format!("decoding {input} from incoming Arrow"))
}

/// Forward one incoming dora `Event::Input` as a MAVLink frame on
/// `conn`. Unknown input ids are silently ignored — the bridge does
/// not abort the whole node on misrouted inputs.
fn handle_input(
    conn: &(dyn MavConnection<MavMessage> + Send + Sync),
    header: &MavHeader,
    id: &DataId,
    data: &ArrayRef,
) -> Result<()> {
    let outgoing = match id.as_str() {
        "heartbeat_cmd" => {
            let d: HEARTBEAT_DATA = decode_input(data, id.as_str())?;
            MavMessage::HEARTBEAT(d)
        }
        "command_long_cmd" => {
            let d: COMMAND_LONG_DATA = decode_input(data, id.as_str())?;
            MavMessage::COMMAND_LONG(d)
        }
        _ => return Ok(()), // unknown input id, ignore
    };
    conn.send(header, &outgoing)
        .map_err(|e| eyre!("mavlink send: {e}"))?;
    Ok(())
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::try_init().ok();

    let cfg = Config::from_env()?;
    let url = cfg.parsed_endpoint()?;
    tracing::info!(endpoint = %url, "opening MAVLink 2 transport");

    let conn_box = transport::connect(&url)?;
    let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::from(conn_box);

    let header = MavHeader {
        system_id: cfg.system_id,
        component_id: cfg.component_id,
        sequence: 0,
    };

    let (mut node, mut events) =
        DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let (tx, rx) = flume::bounded::<(DataId, ArrayRef)>(64);

    let reader_handle = {
        let conn = Arc::clone(&conn);
        std::thread::Builder::new()
            .name("mavlink-reader".into())
            .spawn(move || run_reader(conn, tx))
            .context("spawning mavlink reader thread")?
    };

    loop {
        while let Ok((id, arr)) = rx.try_recv() {
            node.send_output(id, MetadataParameters::default(), arr)
                .map_err(|e| eyre!("send_output: {e}"))?;
        }

        match events.recv_timeout(POLL_INTERVAL) {
            Some(Event::Input { id, data, .. }) => {
                let array_ref: ArrayRef = data.into();
                if let Err(e) = handle_input(conn.as_ref(), &header, &id, &array_ref) {
                    tracing::warn!("writer error on input '{id}': {e:#}");
                }
            }
            Some(Event::Stop(_)) => break,
            Some(_) => {}
            None => {}
        }
    }

    drop(node);
    drop(events);
    match reader_handle.join() {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => bail!("reader thread error: {e:#}"),
        Err(panic) => bail!("reader thread panicked: {panic:?}"),
    }
}
