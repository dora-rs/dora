//! In-process MAVLink 2 simulator for the `mavlink2-bridge` example.
//!
//! Connects via UDP `udpout:` to the bridge (which listens with
//! `udpin:`), and emits a HEARTBEAT every 500 ms. UDP is used instead
//! of TCP so successive smoke-test runs are not blocked by `TIME_WAIT`,
//! and so the bridge's reverse-direction `_cmd` path has a known peer
//! address (mavlink's `udpin:` replies to the latest sender).
//!
//! The node participates in the dora lifecycle (init + Stop event) so
//! the daemon can shut it down cleanly; the actual MAVLink work runs
//! on a background thread.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::{Duration, Instant};

use dora_node_api::{DoraNode, Event};
use eyre::{Result, bail, eyre};
use mavlink::{
    MavConnection, MavHeader, MavlinkVersion,
    dialects::common::{HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType},
};

const DEFAULT_PEER: &str = "127.0.0.1:14550";
const SEND_INTERVAL: Duration = Duration::from_millis(500);
const PEER_RETRY_BUDGET: Duration = Duration::from_secs(5);
const PEER_RETRY_INTERVAL: Duration = Duration::from_millis(100);

fn open_sender(peer: &str) -> Result<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let deadline = Instant::now() + PEER_RETRY_BUDGET;
    loop {
        match mavlink::connect::<MavMessage>(&format!("udpout:{peer}")) {
            Ok(mut conn) => {
                conn.set_protocol_version(MavlinkVersion::V2);
                return Ok(Box::new(conn));
            }
            Err(e) => {
                if Instant::now() >= deadline {
                    return Err(eyre!(
                        "udpout:{peer} unreachable after {:?}: {e}",
                        PEER_RETRY_BUDGET
                    ));
                }
                tracing::debug!("waiting for MAVLink peer at {peer}: {e}");
                std::thread::sleep(PEER_RETRY_INTERVAL);
            }
        }
    }
}

fn run_simulator(peer: String, stop: Arc<AtomicBool>) -> Result<()> {
    let conn = open_sender(&peer)?;
    let header = MavHeader {
        system_id: 1,
        component_id: 1,
        sequence: 0,
    };
    let mut tick: u32 = 0;
    while !stop.load(Ordering::Relaxed) {
        let frame = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            custom_mode: tick,
            mavtype: MavType::MAV_TYPE_QUADROTOR,
            autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
            base_mode: MavModeFlag::empty(),
            system_status: MavState::MAV_STATE_ACTIVE,
            mavlink_version: 3,
        });
        if let Err(e) = conn.send(&header, &frame) {
            tracing::warn!("simulator send failed: {e}");
        } else {
            tracing::trace!(tick, "sent simulated heartbeat");
        }
        tick = tick.wrapping_add(1);
        std::thread::sleep(SEND_INTERVAL);
    }
    Ok(())
}

fn main() -> Result<()> {
    tracing_subscriber_init();

    let peer = std::env::var("MAVLINK_SIM_PEER").unwrap_or_else(|_| DEFAULT_PEER.to_string());
    tracing::info!(%peer, "MAVLink simulator starting");

    let (_node, mut events) = DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let stop = Arc::new(AtomicBool::new(false));
    let sim_handle = {
        let stop = Arc::clone(&stop);
        std::thread::Builder::new()
            .name("mavlink-sim".into())
            .spawn(move || run_simulator(peer, stop))
            .map_err(|e| eyre!("spawn simulator thread: {e}"))?
    };

    while let Some(event) = events.recv() {
        if matches!(event, Event::Stop(_)) {
            break;
        }
    }

    stop.store(true, Ordering::Relaxed);
    match sim_handle.join() {
        Ok(Ok(())) => Ok(()),
        Ok(Err(e)) => bail!("simulator thread error: {e:#}"),
        Err(panic) => bail!("simulator thread panicked: {panic:?}"),
    }
}

fn tracing_subscriber_init() {
    // Best-effort init; ignore errors if a subscriber is already set.
    let _ = tracing_subscriber::fmt::try_init();
}
