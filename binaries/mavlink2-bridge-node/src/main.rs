//! MAVLink 2 ↔ dora bridge node.
//!
//! This binary is spawned by the dora daemon as a normal node. It opens
//! a MAVLink 2 connection (TCP in this PR; UDP/serial in #1786 follow-up),
//! decodes incoming frames into Arrow `StructArray` rows, and publishes
//! them on dora outputs. Inputs are encoded back to MAVLink frames and
//! sent on the same connection.
//!
//! Configuration comes from the `MAVLINK_BRIDGE_CONFIG` env var as YAML:
//!
//! ```yaml
//! endpoint: tcp://127.0.0.1:5760
//! system_id: 255
//! component_id: 1
//! ```

use dora_mavlink2_bridge::{MavlinkArrow, mavlink::common::HEARTBEAT_DATA, transport};
use dora_node_api::{
    DoraNode, Event, MetadataParameters,
    arrow::array::{ArrayRef, AsArray, RecordBatch, StructArray},
    dora_core::config::DataId,
    flume,
};
use eyre::{Context, Result, bail, eyre};
use mavlink::{MavConnection, MavHeader, common::MavMessage};
use serde::Deserialize;
use std::{sync::Arc, time::Duration};
use url::Url;

const HEARTBEAT_OUTPUT: &str = "heartbeat";
const HEARTBEAT_INPUT: &str = "heartbeat_cmd";
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

/// Reader thread: blocks on `conn.recv()` and forwards converted Arrow
/// arrays to the main thread via `tx`. Returns when `tx` is dropped
/// (main thread exited) or the connection errors continuously.
fn run_reader(
    conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>,
    tx: flume::Sender<(DataId, ArrayRef)>,
) -> Result<()> {
    loop {
        match conn.recv() {
            Ok((_header, MavMessage::HEARTBEAT(d))) => {
                let batch = d.to_record_batch()?;
                let arr: ArrayRef = Arc::new(StructArray::from(batch));
                if tx
                    .send((DataId::from(HEARTBEAT_OUTPUT.to_string()), arr))
                    .is_err()
                {
                    return Ok(()); // main thread gone
                }
            }
            Ok(_) => {
                // PR #3 only handles HEARTBEAT; other message types are
                // ignored here and will be wired up by PR #5 (common
                // dialect coverage). #1786.
            }
            Err(e) => {
                tracing::warn!("mavlink recv error: {e}");
                std::thread::sleep(Duration::from_millis(50));
            }
        }
    }
}

/// Decode one incoming dora `Event::Input` and forward it as a MAVLink
/// frame on `conn`. Returns Ok even for unknown input ids — the bridge
/// silently ignores them rather than failing the whole node.
fn handle_input(
    conn: &(dyn MavConnection<MavMessage> + Send + Sync),
    header: &MavHeader,
    id: &DataId,
    data: &ArrayRef,
) -> Result<()> {
    if id.as_str() != HEARTBEAT_INPUT {
        return Ok(()); // unknown input id, ignore
    }
    let struct_array = data.as_struct_opt().ok_or_else(|| {
        eyre!(
            "expected StructArray for input '{HEARTBEAT_INPUT}', got {:?}",
            data.data_type()
        )
    })?;
    let batch = RecordBatch::from(struct_array);
    let cmd = HEARTBEAT_DATA::from_record_batch(&batch)
        .context("decoding HEARTBEAT from incoming Arrow")?;
    conn.send(header, &MavMessage::HEARTBEAT(cmd))
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
            Some(_) => {} // ignore other event variants
            None => {}    // timeout, revisit rx
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
