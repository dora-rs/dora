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
    DoraNode, Event, MetadataParameters, TryRecvError,
    arrow::array::{Array, ArrayRef, AsArray, RecordBatch, StructArray},
    dora_core::config::DataId,
    flume,
};
use eyre::{Context, Result, bail, eyre};
use mavlink::{
    MavConnection, MavHeader, MavlinkVersion, Message, common::MavMessage, error::MessageReadError,
};
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
/// Minimum gap between `recv`-error warnings. mavlink-core 0.13's TCP
/// transport returns `Err(WouldBlock)` every ~100 ms on a silent peer,
/// so without rate-limiting a transient (or even a fatal, pre-fix) link
/// would emit ~10 warns/second. We log at most one warning per this
/// interval so a flaky or dead link doesn't flood the logs.
const RECV_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// How `run_reader` should react to an error returned by `conn.recv()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RecvDisposition {
    /// "No data yet" — a socket read timeout (`WouldBlock`/`TimedOut`),
    /// an interrupted syscall, or a single corrupt frame on an
    /// otherwise-live link. The reader keeps looping; the connection is
    /// still usable.
    Transient,
    /// "Connection dead" — a fatal I/O error such as `UnexpectedEof`
    /// (peer closed the socket), `ConnectionReset`, or `BrokenPipe`.
    /// The reader must stop spinning and surface this so the node
    /// terminates/restarts per the daemon's policy.
    Fatal,
}

/// Classify a `conn.recv()` error so the reader can tell a transient
/// "no data yet" timeout apart from a fatal "connection dead" error.
///
/// This is the core of the #2034 fix: the previous `Err` arm treated
/// every error the same way — sleeping 50 ms and warning forever —
/// which turned a dead link into a logging zombie and could never
/// distinguish a 100 ms read timeout from an EOF.
fn classify_recv_error(err: &MessageReadError) -> RecvDisposition {
    match err {
        // A parse failure means a frame arrived but was garbled (CRC
        // mismatch, unknown id, bad enum). The link is still alive, so
        // skip the bad frame and keep reading.
        MessageReadError::Parse(_) => RecvDisposition::Transient,
        MessageReadError::Io(io_err) => match io_err.kind() {
            std::io::ErrorKind::WouldBlock
            | std::io::ErrorKind::TimedOut
            | std::io::ErrorKind::Interrupted => RecvDisposition::Transient,
            _ => RecvDisposition::Fatal,
        },
    }
}

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
///     exit grace window in `main`, OR
///   * `conn.recv()` returns a **fatal** error (a dead link, e.g.
///     `UnexpectedEof`/`ConnectionReset`). Then it returns `Err` so
///     `main` tears the node down instead of spinning forever. Transient
///     errors (read timeouts, single corrupt frames) keep the loop
///     running quietly; see `classify_recv_error`.
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

    // Rate-limit the recv-error warning: only the first error and then
    // at most one per `RECV_WARN_INTERVAL` reaches the logs.
    let mut last_warn: Option<Instant> = None;
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
                // During shutdown, swallow the error: TCP's 100 ms read
                // timeout produces a `WouldBlock` here, which is the
                // *intended* path back to the shutdown flag check.
                if shutdown.load(Ordering::Relaxed) {
                    return Ok(());
                }
                match classify_recv_error(&e) {
                    RecvDisposition::Fatal => {
                        // Connection is dead. Surface the error so the
                        // node terminates and the daemon restarts it per
                        // policy, instead of spinning as a logging zombie
                        // with a link that will never recover.
                        return Err(eyre!("mavlink connection lost: {e}"));
                    }
                    RecvDisposition::Transient => {
                        // "No data yet" (read timeout) or a single
                        // corrupt frame. Keep looping quietly; rate-limit
                        // the warn so a chatty-but-live link can't flood.
                        let now = Instant::now();
                        let should_warn = last_warn
                            .is_none_or(|prev| now.duration_since(prev) >= RECV_WARN_INTERVAL);
                        if should_warn {
                            tracing::warn!("mavlink recv error (transient): {e}");
                            last_warn = Some(now);
                        }
                    }
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
    // RecordBatch::from(&StructArray) asserts null_count == 0; check first
    // so malformed inputs surface as BridgeError rather than a process panic.
    if struct_array.null_count() > 0 {
        bail!(
            "input '{input}' is a nullable StructArray with {} null row(s); \
             expected exactly one non-null MAVLink message",
            struct_array.null_count()
        );
    }
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
    let sent = conn
        .send(header, &outgoing)
        .map_err(|e| eyre!("mavlink send: {e}"))?;
    if sent == 0 {
        // UDP server mode (`udpin:`) reports a successful 0-byte send until a
        // client has connected, so the command was silently dropped rather
        // than transmitted. Surface it instead of logging a false success
        // (dora-rs/dora#2027).
        tracing::warn!("command '{id}' not transmitted: no MAVLink peer connected yet");
    }
    Ok(())
}

/// Try to send the `REQUEST_DATA_STREAM` setup message. Returns `true` once it
/// no longer needs retrying — either it was actually transmitted (`>0` bytes)
/// or it failed with a hard error (treated as non-fatal, like the autopilots
/// that ignore the legacy request). Returns `false` only for the UDP
/// server-mode case where `send` succeeds with 0 bytes because no client has
/// connected yet; the caller retries until a peer appears (dora-rs/dora#2027).
fn attempt_data_stream_request(
    conn: &(dyn MavConnection<MavMessage> + Send + Sync),
    header: &MavHeader,
) -> bool {
    let req = mavlink::common::REQUEST_DATA_STREAM_DATA {
        req_message_rate: 5,
        target_system: 0, // 0 = broadcast to whichever autopilot answers
        target_component: 0,
        req_stream_id: 0, // MAV_DATA_STREAM_ALL
        start_stop: 1,
    };
    match conn.send(header, &MavMessage::REQUEST_DATA_STREAM(req)) {
        Ok(0) => false, // no UDP peer yet — retry once one connects
        Ok(_) => {
            tracing::info!("requested all data streams at 5 Hz");
            true
        }
        Err(e) => {
            tracing::warn!("REQUEST_DATA_STREAM send failed (non-fatal): {e}");
            true
        }
    }
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
    // until a GCS asks. In UDP server mode this first attempt is dropped
    // (no client yet), so it is retried in the event loop below until a peer
    // connects (dora-rs/dora#2027).
    let mut data_stream_requested = attempt_data_stream_request(conn.as_ref(), &header);

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

        // If the reader thread exited on its own (a fatal transport
        // disconnect, see #2034), stop the event loop and let the
        // join below surface its error so this node terminates rather
        // than running blind without a live MAVLink link.
        if reader_handle.is_finished() {
            break;
        }

        // Retry the data-stream request until it is actually transmitted. In
        // UDP server mode the initial attempt is dropped because no client has
        // connected; once the reader receives a packet the shared connection
        // learns the peer and this send goes out (dora-rs/dora#2027).
        if !data_stream_requested {
            data_stream_requested = attempt_data_stream_request(conn.as_ref(), &header);
        }

        let next = match events.recv_timeout(POLL_INTERVAL) {
            Some(event) => Some(event),
            // `recv_timeout` returns `None` for BOTH a timeout and a closed
            // stream. Disambiguate with `try_recv`: a closed stream means the
            // daemon went away without a `Stop` event, so exit instead of
            // looping forever (dora-rs/dora#2027). `try_recv` also returns any
            // event that raced in after the timeout, so none is lost.
            None => match events.try_recv() {
                Ok(event) => Some(event),
                Err(TryRecvError::Closed) => break,
                Err(TryRecvError::Empty) => None,
            },
        };

        if let Some(event) = next {
            match event {
                Event::Input { id, data, .. } => {
                    let array_ref: ArrayRef = data.into();
                    if let Err(e) = handle_input(conn.as_ref(), &header, &id, &array_ref) {
                        // Writer errors mean the dora node's command did NOT reach the
                        // autopilot. Surface at error level so users debugging missions see
                        // it in default log filters rather than treating it as routine noise.
                        tracing::error!("writer error on input '{id}': {e:#}");
                    }
                }
                Event::Stop(_) => break,
                _ => {}
            }
        }
    }

    // Final drain: forward any telemetry the reader already decoded into `rx`
    // but the loop exited before emitting (a `Stop` arrives while frames are
    // queued; the daemon is still alive on that path). Best-effort — on a
    // closed stream `send_output` errors and is ignored (dora-rs/dora#2027).
    while let Ok((id, arr)) = rx.try_recv() {
        let _ = node.send_output(id, MetadataParameters::default(), arr);
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
    //! Shutdown-path and input-decode regression tests.
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

    // ---- #2034: recv-error classification ----------------------------
    //
    // The reader must tell a transient "no data yet" read timeout apart
    // from a fatal "connection dead" error. The pre-fix `Err` arm
    // conflated both and spun forever as a logging zombie.

    fn io(kind: std::io::ErrorKind) -> MessageReadError {
        MessageReadError::Io(std::io::Error::new(kind, "test"))
    }

    #[test]
    fn would_block_is_transient() {
        // TCP's 100 ms socket read timeout surfaces as WouldBlock; this
        // is the normal "silent peer, no data yet" path and must NOT be
        // treated as a dead link.
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::WouldBlock)),
            RecvDisposition::Transient
        );
    }

    #[test]
    fn timed_out_is_transient() {
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::TimedOut)),
            RecvDisposition::Transient
        );
    }

    #[test]
    fn interrupted_is_transient() {
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::Interrupted)),
            RecvDisposition::Transient
        );
    }

    #[test]
    fn parse_error_is_transient() {
        // A garbled frame on an otherwise-live link: skip it, keep reading.
        let err = MessageReadError::Parse(mavlink::error::ParserError::UnknownMessage { id: 9999 });
        assert_eq!(classify_recv_error(&err), RecvDisposition::Transient);
    }

    #[test]
    fn unexpected_eof_is_fatal() {
        // Peer closed the TCP socket: mavlink's `read_exact` returns
        // UnexpectedEof. This is the disconnect that previously turned
        // the reader into a logging zombie.
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::UnexpectedEof)),
            RecvDisposition::Fatal
        );
    }

    #[test]
    fn connection_reset_is_fatal() {
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::ConnectionReset)),
            RecvDisposition::Fatal
        );
    }

    #[test]
    fn broken_pipe_is_fatal() {
        assert_eq!(
            classify_recv_error(&io(std::io::ErrorKind::BrokenPipe)),
            RecvDisposition::Fatal
        );
    }

    /// End-to-end of the fix: a real TCP peer that accepts then closes
    /// its socket makes `run_reader` return `Err` (fatal disconnect)
    /// instead of spinning forever. Pre-fix, the reader looped on the
    /// EOF error and never returned.
    #[test]
    fn run_reader_exits_on_fatal_tcp_disconnect() {
        use std::io::Read;
        use std::net::TcpListener;

        let listener = TcpListener::bind("127.0.0.1:0").expect("bind listener");
        let port = listener.local_addr().expect("local_addr").port();

        // Accept the bridge's outbound TCP connection, hold it briefly so
        // the reader settles into `recv`, then drop it to signal EOF.
        let server = std::thread::spawn(move || {
            let (stream, _addr) = listener.accept().expect("accept");
            // Give the reader thread time to block in `recv`.
            std::thread::sleep(Duration::from_millis(150));
            // Read any REQUEST_DATA_STREAM the bridge might send, then
            // close the socket to deliver EOF to the reader.
            let mut stream = stream;
            let _ = stream.set_read_timeout(Some(Duration::from_millis(50)));
            let mut scratch = [0u8; 64];
            let _ = stream.read(&mut scratch);
            drop(stream);
        });

        let url = Url::parse(&format!("tcp://127.0.0.1:{port}")).unwrap();
        let conn_box = transport::connect(&url).expect("transport::connect");
        let conn: Arc<dyn MavConnection<MavMessage> + Send + Sync> = Arc::from(conn_box);

        let (tx, _rx_keep_alive) = flume::bounded::<(DataId, ArrayRef)>(64);
        let shutdown = Arc::new(AtomicBool::new(false));

        let handle = std::thread::spawn({
            let conn = Arc::clone(&conn);
            let shutdown = Arc::clone(&shutdown);
            move || run_reader(conn, tx, shutdown)
        });

        // The reader must return promptly once the peer closes. The
        // shutdown flag stays `false`, so the only way out is the fatal
        // path. Cap at 3 s for CI headroom; pre-fix this never finishes.
        let join_deadline = Instant::now() + Duration::from_secs(3);
        while !handle.is_finished() && Instant::now() < join_deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(
            handle.is_finished(),
            "reader did not exit after fatal TCP disconnect (logging-zombie regression #2034)"
        );
        let result = handle.join().expect("reader thread panicked");
        assert!(
            result.is_err(),
            "reader should surface a fatal error on disconnect, got Ok"
        );
        server.join().expect("server thread panicked");
    }

    /// #2027: in UDP server mode (`udp://` => `udpin:`) the data-stream
    /// request cannot be transmitted until a client connects — `conn.send`
    /// reports a 0-byte success. `attempt_data_stream_request` must report
    /// that as "not yet sent" (false) so the event loop retries, rather than
    /// logging a false success and giving up.
    #[test]
    fn data_stream_request_deferred_without_udp_peer() {
        let port = pick_free_udp_port();
        let url = Url::parse(&format!("udp://127.0.0.1:{port}")).unwrap();
        let conn = transport::connect(&url).expect("transport::connect");
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };

        assert!(
            !attempt_data_stream_request(conn.as_ref(), &header),
            "with no UDP peer connected the request is dropped (0 bytes) and must be retried"
        );
    }

    /// Regression test for #2298: a nullable top-level `StructArray` (i.e. one
    /// with a validity/null buffer and `null_count > 0`) must be rejected by
    /// `decode_input` as a `BridgeError`, not panic inside
    /// `RecordBatch::from(&StructArray)`.
    #[test]
    fn decode_input_rejects_nullable_struct_array() {
        use arrow::array::{
            Float32Array, StructArray as ArrowStructArray, UInt8Array, UInt32Array,
        };
        use arrow::buffer::NullBuffer;
        use arrow::datatypes::{DataType, Field};
        use dora_mavlink2_bridge::mavlink::common::COMMAND_LONG_DATA;
        use std::sync::Arc;

        let fields = vec![
            Arc::new(Field::new("param1", DataType::Float32, false)),
            Arc::new(Field::new("param2", DataType::Float32, false)),
            Arc::new(Field::new("param3", DataType::Float32, false)),
            Arc::new(Field::new("param4", DataType::Float32, false)),
            Arc::new(Field::new("param5", DataType::Float32, false)),
            Arc::new(Field::new("param6", DataType::Float32, false)),
            Arc::new(Field::new("param7", DataType::Float32, false)),
            Arc::new(Field::new("command", DataType::UInt32, false)),
            Arc::new(Field::new("target_system", DataType::UInt8, false)),
            Arc::new(Field::new("target_component", DataType::UInt8, false)),
            Arc::new(Field::new("confirmation", DataType::UInt8, false)),
        ];
        let arrays: Vec<arrow::array::ArrayRef> = vec![
            Arc::new(Float32Array::from(vec![1.0f32])),
            Arc::new(Float32Array::from(vec![2.0f32])),
            Arc::new(Float32Array::from(vec![3.0f32])),
            Arc::new(Float32Array::from(vec![4.0f32])),
            Arc::new(Float32Array::from(vec![5.0f32])),
            Arc::new(Float32Array::from(vec![6.0f32])),
            Arc::new(Float32Array::from(vec![7.0f32])),
            Arc::new(UInt32Array::from(vec![22u32])),
            Arc::new(UInt8Array::from(vec![1u8])),
            Arc::new(UInt8Array::from(vec![1u8])),
            Arc::new(UInt8Array::from(vec![0u8])),
        ];
        // Build a StructArray with a top-level null buffer marking row 0 as null.
        let nulls = NullBuffer::from(vec![false]); // row 0 is null
        let struct_array = ArrowStructArray::new(
            fields.into_iter().collect::<arrow::datatypes::Fields>(),
            arrays,
            Some(nulls),
        );
        let array_ref: ArrayRef = Arc::new(struct_array);

        let err = decode_input::<COMMAND_LONG_DATA>(&array_ref, "command_long_cmd")
            .expect_err("nullable StructArray must be rejected, not panic");
        let msg = format!("{err:?}");
        assert!(
            msg.contains("nullable") || msg.contains("null"),
            "error must mention null rows, got: {msg}"
        );
    }
}
