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
//! | dora input                          | MAVLink message                  |
//! |-------------------------------------|----------------------------------|
//! | `heartbeat_cmd`                     | HEARTBEAT                        |
//! | `command_long_cmd`                  | COMMAND_LONG                     |
//! | `set_mode_cmd`                      | SET_MODE                         |
//! | `rc_channels_override_cmd`          | RC_CHANNELS_OVERRIDE             |
//! | `set_position_target_global_int_cmd`| SET_POSITION_TARGET_GLOBAL_INT   |
//! | `set_position_target_local_ned_cmd` | SET_POSITION_TARGET_LOCAL_NED    |
//!
//! Other message types are read-only telemetry; sending them inbound is
//! out of scope for this PR.

use dora_mavlink2_bridge::{
    MavlinkArrow,
    mavlink::common::{
        COMMAND_LONG_DATA, HEARTBEAT_DATA, RC_CHANNELS_OVERRIDE_DATA, SET_MODE_DATA,
        SET_POSITION_TARGET_GLOBAL_INT_DATA, SET_POSITION_TARGET_LOCAL_NED_DATA,
    },
    transport,
};
use dora_node_api::{
    DoraNode, Event, MetadataParameters,
    arrow::array::{ArrayRef, AsArray, RecordBatch, StructArray},
    dora_core::config::DataId,
    flume,
};
use eyre::{Context, Result, bail, eyre};
use mavlink::{MavConnection, MavHeader, MavlinkVersion, Message, common::MavMessage};
use serde::Deserialize;
use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};
use url::Url;

const POLL_INTERVAL: Duration = Duration::from_millis(100);
/// Belt-and-suspenders fallback for shutdown after the interrupt
/// signals below have already fired. Covers the only transport mavlink
/// 0.13 doesn't let us interrupt cleanly (serial: `SerialConnection`'s
/// `SystemPort` is a private field, so we can't close it from another
/// thread). After this budget elapses we drop the `JoinHandle` and let
/// the OS reap the reader on process exit, so `dora stop` /
/// `--stop-after` don't hang until SIGKILL at the daemon's grace
/// deadline.
const READER_SHUTDOWN_GRACE: Duration = Duration::from_millis(500);
const READER_SHUTDOWN_POLL: Duration = Duration::from_millis(20);
/// Total time to wait for a MAVLink endpoint to become reachable on
/// initial bridge startup. A real autopilot can be slower to come up
/// than the dora daemon, and the example dataflow spawns the bridge
/// in parallel with an in-process simulator — both cases need a
/// budgeted retry rather than a one-shot connect.
const CONNECT_RETRY_BUDGET: Duration = Duration::from_secs(5);
const CONNECT_RETRY_INTERVAL: Duration = Duration::from_millis(100);

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
///
/// Exits when:
///   * `tx` is dropped (main thread exited normally), OR
///   * `shutdown` is set AND `conn.recv()` returns. The flag check
///     fires at the top of every loop iteration. mavlink-core 0.13's
///     **TCP** transport sets a 100 ms socket read_timeout, so each
///     `conn.recv()` returns `Err(WouldBlock)` within ~100 ms even on a
///     silent peer, giving the flag a window to fire. **UDP** and
///     **Serial** loop internally on errors (see `connection/udp.rs`
///     and `connection/direct_serial.rs`); main wakes UDP via a
///     self-loopback HEARTBEAT so the inner `recv_from` returns Ok
///     and we recheck the flag. Serial has no externally accessible
///     wake path in mavlink 0.13, so it falls back to the OS-reap-on-
///     exit grace window in `main`.
fn run_reader(
    conn: Arc<dyn MavConnection<MavMessage> + Send + Sync>,
    tx: flume::Sender<(DataId, ArrayRef)>,
    shutdown: Arc<AtomicBool>,
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
        if shutdown.load(Ordering::Relaxed) {
            return Ok(());
        }
        match conn.recv() {
            Ok((_header, msg)) => {
                if shutdown.load(Ordering::Relaxed) {
                    return Ok(());
                }
                match msg {
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
                }
            }
            Err(e) => {
                // Suppress the warn during shutdown: TCP's 100 ms read
                // timeout will have produced a `WouldBlock` here, which
                // is the *intended* path back to the shutdown flag check.
                if !shutdown.load(Ordering::Relaxed) {
                    tracing::warn!("mavlink recv error: {e}");
                    std::thread::sleep(Duration::from_millis(50));
                }
            }
        }
    }
}

/// Send a self-loopback HEARTBEAT to the bridge's own bound UDP port
/// so mavlink-core 0.13's `UdpConnection::recv` (which loops internally
/// on errors and never returns Err) sees a valid frame and returns
/// `Ok`, letting `run_reader` re-check its shutdown flag and exit.
///
/// For `udp://0.0.0.0:N` we redirect to `127.0.0.1:N` because sending
/// to 0.0.0.0 isn't routable. Any send error is non-fatal — the grace
/// window in `main` is the safety net.
fn wake_udp_recv(url: &Url, system_id: u8, component_id: u8) -> std::io::Result<()> {
    use mavlink::common::{
        HEARTBEAT_DATA, MavAutopilot, MavMessage, MavModeFlag, MavState, MavType,
    };
    use std::net::UdpSocket;

    let host = url.host_str().unwrap_or("127.0.0.1");
    let target_host = if host == "0.0.0.0" || host.is_empty() {
        "127.0.0.1"
    } else {
        host
    };
    let port = url.port().unwrap_or(transport::DEFAULT_UDP_PORT);
    let dest = format!("{target_host}:{port}");

    let socket = UdpSocket::bind("0.0.0.0:0")?;
    let header = MavHeader {
        system_id,
        component_id,
        sequence: 0,
    };
    let hb = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_GENERIC,
        base_mode: MavModeFlag::empty(),
        system_status: MavState::MAV_STATE_UNINIT,
        mavlink_version: 3,
    });
    let mut buf = Vec::new();
    mavlink::write_versioned_msg(&mut buf, MavlinkVersion::V2, header, &hb)
        .map_err(|e| std::io::Error::other(format!("encode wake heartbeat: {e}")))?;
    socket.send_to(&buf, dest)?;
    Ok(())
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
        "set_mode_cmd" => {
            let d: SET_MODE_DATA = decode_input(data, id.as_str())?;
            MavMessage::SET_MODE(d)
        }
        "rc_channels_override_cmd" => {
            let d: RC_CHANNELS_OVERRIDE_DATA = decode_input(data, id.as_str())?;
            MavMessage::RC_CHANNELS_OVERRIDE(d)
        }
        "set_position_target_global_int_cmd" => {
            let d: SET_POSITION_TARGET_GLOBAL_INT_DATA = decode_input(data, id.as_str())?;
            MavMessage::SET_POSITION_TARGET_GLOBAL_INT(d)
        }
        "set_position_target_local_ned_cmd" => {
            let d: SET_POSITION_TARGET_LOCAL_NED_DATA = decode_input(data, id.as_str())?;
            MavMessage::SET_POSITION_TARGET_LOCAL_NED(d)
        }
        _ => return Ok(()), // unknown input id, ignore
    };
    conn.send(header, &outgoing)
        .map_err(|e| eyre!("mavlink send: {e}"))?;
    Ok(())
}

/// Open the MAVLink transport, retrying briefly if the peer is not yet
/// listening. Returns once a connection succeeds or the budget elapses.
fn connect_with_retry(url: &Url) -> Result<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let deadline = std::time::Instant::now() + CONNECT_RETRY_BUDGET;
    let mut last_err: Option<eyre::Report> = None;
    loop {
        match transport::connect(url) {
            Ok(conn) => return Ok(conn),
            Err(e) => {
                let now = std::time::Instant::now();
                if now >= deadline {
                    return Err(eyre!(
                        "failed to open MAVLink transport at {url} within {:?}: {} (last attempt: {})",
                        CONNECT_RETRY_BUDGET,
                        last_err.unwrap_or_else(|| eyre!("(none)")),
                        e
                    ));
                }
                tracing::debug!(%url, "waiting for MAVLink endpoint: {e}");
                last_err = Some(eyre!("{e}"));
                std::thread::sleep(CONNECT_RETRY_INTERVAL);
            }
        }
    }
}

fn main() -> Result<()> {
    tracing_subscriber::fmt::try_init().ok();

    let cfg = Config::from_env()?;
    let url = cfg.parsed_endpoint()?;
    tracing::info!(endpoint = %url, "opening MAVLink 2 transport");

    let conn_box = connect_with_retry(&url)?;
    let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::from(conn_box);

    let header = MavHeader {
        system_id: cfg.system_id,
        component_id: cfg.component_id,
        sequence: 0,
    };

    // Request all data streams from the autopilot at 5 Hz so HEARTBEAT,
    // GLOBAL_POSITION_INT, GPS_RAW_INT, COMMAND_ACK, etc. start flowing
    // on this link. Without this, ArduCopter 3.x sends only HEARTBEAT
    // until a GCS asks. Failure is non-fatal (some autopilots ignore the
    // legacy REQUEST_DATA_STREAM and prefer SET_MESSAGE_INTERVAL).
    let req = mavlink::common::REQUEST_DATA_STREAM_DATA {
        req_message_rate: 5,
        target_system: 0, // 0 = broadcast to whichever autopilot answers
        target_component: 0,
        req_stream_id: 0, // MAV_DATA_STREAM_ALL
        start_stop: 1,
    };
    if let Err(e) = conn.send(&header, &MavMessage::REQUEST_DATA_STREAM(req)) {
        tracing::warn!("REQUEST_DATA_STREAM send failed (non-fatal): {e}");
    } else {
        tracing::info!("requested all data streams at 5 Hz");
    }

    let (mut node, mut events) =
        DoraNode::init_from_env().map_err(|e| eyre!("DoraNode init: {e}"))?;

    let (tx, rx) = flume::bounded::<(DataId, ArrayRef)>(64);

    // Shared shutdown signal: main flips this after `Event::Stop` so
    // `run_reader` can break out of its receive loop on the next
    // iteration boundary instead of hanging on `conn.recv()`.
    let shutdown = Arc::new(AtomicBool::new(false));

    let reader_handle = {
        let conn = Arc::clone(&conn);
        let shutdown = Arc::clone(&shutdown);
        std::thread::Builder::new()
            .name("mavlink-reader".into())
            .spawn(move || run_reader(conn, tx, shutdown))
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
                    // Writer errors mean the dora node's command did NOT reach the
                    // autopilot. Surface at error level so users debugging missions see
                    // it in default log filters rather than treating it as routine noise.
                    tracing::error!("writer error on input '{id}': {e:#}");
                }
            }
            Some(Event::Stop(_)) => break,
            Some(_) => {}
            None => {}
        }
    }

    // Three-layer shutdown sequence — each layer covers a transport
    // mavlink-core 0.13 leaves un-interruptible at the previous layer:
    //
    //   1. Set the shutdown flag. `run_reader` checks it at the top of
    //      every loop iteration; **TCP** wakes within ~100 ms because
    //      mavlink's TCP socket has a 100 ms `set_read_timeout` so each
    //      `recv()` returns `Err(WouldBlock)` and we revisit the flag.
    //
    //   2. Send a self-loopback HEARTBEAT for **UDP**, since
    //      `UdpConnection::recv` loops internally on errors and never
    //      returns Err — only a successfully parsed frame gets us back
    //      to the flag check. The wake packet is a valid HEARTBEAT to
    //      our own bound port; the reader receives it, hits the flag,
    //      exits. Send errors are non-fatal (the grace window below
    //      catches us).
    //
    //   3. Bounded grace + detach. **Serial** has no externally usable
    //      wake path in mavlink 0.13 (`SerialConnection`'s `SystemPort`
    //      is private and `recv` loops internally), so on serial we
    //      fall through to here, drop the JoinHandle, and let the OS
    //      reap the reader thread on process exit. `dora stop` /
    //      `--stop-after` get control back within
    //      `READER_SHUTDOWN_GRACE` regardless.
    shutdown.store(true, Ordering::Relaxed);
    if url.scheme() == "udp"
        && let Err(e) = wake_udp_recv(&url, cfg.system_id, cfg.component_id)
    {
        tracing::debug!("UDP wake send failed (non-fatal): {e}");
    }
    drop(rx);
    drop(node);
    drop(events);

    let shutdown_deadline = Instant::now() + READER_SHUTDOWN_GRACE;
    while !reader_handle.is_finished() && Instant::now() < shutdown_deadline {
        std::thread::sleep(READER_SHUTDOWN_POLL);
    }

    if reader_handle.is_finished() {
        match reader_handle.join() {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => bail!("reader thread error: {e:#}"),
            Err(panic) => bail!("reader thread panicked: {panic:?}"),
        }
    } else {
        // Reader is still blocked in `conn.recv()` with no inbound
        // traffic to wake it (this is the serial-transport path, or
        // any UDP setup where the wake packet was dropped by a
        // firewall on a non-loopback bind). Detaching is safe: the
        // thread holds no shared resources beyond `conn` (an `Arc`
        // cleaned up at exit) and a closed channel sender, and the
        // OS terminates it when this process exits a moment later.
        tracing::debug!(
            "mavlink reader still blocked after {READER_SHUTDOWN_GRACE:?} grace; detaching"
        );
        drop(reader_handle);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    //! Shutdown-path regression tests.
    //!
    //! These cover the failure mode the original review flagged: a
    //! reader thread blocked in `conn.recv()` against a silent
    //! MAVLink peer must be wakeable from `main`, otherwise the node
    //! hangs until the daemon's SIGKILL at the grace deadline.
    //!
    //! We don't spawn the full bridge process (that's the integration
    //! tests' job); we drive `run_reader` + `wake_udp_recv` directly
    //! against an isolated UDP loopback to assert the contract.

    use super::*;
    use std::net::UdpSocket;

    /// Reserve a UDP port and immediately drop it so mavlink can
    /// rebind. Mirrors the pattern in `tests/udp_loopback.rs`.
    fn pick_free_udp_port() -> u16 {
        let probe = UdpSocket::bind("127.0.0.1:0").expect("bind probe");
        let port = probe.local_addr().expect("local_addr").port();
        drop(probe);
        port
    }

    /// Silent UDP peer: no inbound traffic at all. Without the wake
    /// path, mavlink-core 0.13's `UdpConnection::recv` loops in
    /// `recv_from` forever. Setting the shutdown flag alone does
    /// nothing because the flag is only checked between iterations —
    /// the wake packet is what causes `recv_from` to return so
    /// `run_reader` can revisit the flag and exit.
    #[test]
    fn run_reader_exits_promptly_when_udp_peer_is_silent() {
        let port = pick_free_udp_port();
        let url = Url::parse(&format!("udp://127.0.0.1:{port}")).unwrap();
        let conn_box = transport::connect(&url).expect("transport::connect");
        let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::from(conn_box);

        let (tx, _rx_keep_alive) = flume::bounded::<(DataId, ArrayRef)>(64);
        let shutdown = Arc::new(AtomicBool::new(false));

        let handle = std::thread::spawn({
            let conn = Arc::clone(&conn);
            let shutdown = Arc::clone(&shutdown);
            move || run_reader(conn, tx, shutdown)
        });

        // Let the reader thread settle into `recv_from`.
        std::thread::sleep(Duration::from_millis(150));
        assert!(
            !handle.is_finished(),
            "reader exited unexpectedly before shutdown was signalled"
        );

        let started = Instant::now();
        shutdown.store(true, Ordering::Relaxed);
        wake_udp_recv(&url, 0xFE, 1).expect("wake send");

        // The wake frame should round-trip and the reader should
        // notice the flag within well under a second. Cap at 2 s to
        // give CI runners headroom; on a developer laptop this
        // typically returns in <50 ms.
        let join_deadline = Instant::now() + Duration::from_secs(2);
        while !handle.is_finished() && Instant::now() < join_deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(
            handle.is_finished(),
            "reader did not exit within 2s of shutdown wake; elapsed={:?}",
            started.elapsed()
        );
        let result = handle.join().expect("reader thread panicked");
        assert!(result.is_ok(), "reader returned Err: {result:?}");
    }

    /// `wake_udp_recv` should rewrite an `udp://0.0.0.0:port` URL to
    /// `127.0.0.1:port` for the send target, since `0.0.0.0` is not a
    /// routable destination. We assert behavior, not the rewrite, by
    /// confirming the wake actually delivers to a listener bound on
    /// `0.0.0.0`.
    #[test]
    fn wake_udp_recv_handles_zero_address_bind() {
        let port = pick_free_udp_port();
        let url = Url::parse(&format!("udp://0.0.0.0:{port}")).unwrap();
        let conn_box = transport::connect(&url).expect("transport::connect");
        let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::from(conn_box);

        let (tx, _rx_keep_alive) = flume::bounded::<(DataId, ArrayRef)>(64);
        let shutdown = Arc::new(AtomicBool::new(false));

        let handle = std::thread::spawn({
            let conn = Arc::clone(&conn);
            let shutdown = Arc::clone(&shutdown);
            move || run_reader(conn, tx, shutdown)
        });
        std::thread::sleep(Duration::from_millis(150));

        shutdown.store(true, Ordering::Relaxed);
        wake_udp_recv(&url, 0xFE, 1).expect("wake send");

        let join_deadline = Instant::now() + Duration::from_secs(2);
        while !handle.is_finished() && Instant::now() < join_deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(handle.is_finished(), "0.0.0.0-bind wake did not deliver");
        handle.join().expect("panic").expect("err");
    }
}
