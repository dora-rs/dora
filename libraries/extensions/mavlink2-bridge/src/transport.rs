//! Open MAVLink 2 transports from a `url::Url`.
//!
//! Supported URL schemes (this PR's scope, **not** a fully general
//! "open MAVLink transport from URL" surface):
//!
//! | URL form                              | mavlink address              | Mode      |
//! |---------------------------------------|------------------------------|-----------|
//! | `tcp://host:port`                     | `tcpout:host:port`           | client    |
//! | `udp://host:port`                     | `udpin:host:port`            | server    |
//! | `serial:///dev/path?baud=N` (Unix)    | `serial:/dev/path:N`         | n/a       |
//! | `serial://COM1?baud=N` (Windows)      | `serial:COM1:N`              | n/a       |
//!
//! ## Known limitations (intentional, not bugs)
//!
//! * **`tcp://` is always `tcpout:` (client mode).** There is no way
//!   to bind a TCP server with this builder; the autopilot must be
//!   the listener. Callers needing `tcpin:` (acting as a TCP server)
//!   must drop down to `mavlink::connect` directly.
//! * **`udp://` is always `udpin:` (server mode — bind + listen).**
//!   The typical use case is receiving Pixhawk telemetry on a fixed
//!   local port, so we hard-wire that. `udpout:` (client) and
//!   `udpbcast:` (broadcast) are out of scope; callers needing them
//!   must drop down to `mavlink::connect` directly. The shutdown
//!   wake-up logic in the bridge node assumes server mode (it sends
//!   a self-loopback HEARTBEAT to the bound port), so flipping this
//!   would require revisiting that path too.
//! * **Serial** baud defaults to `115_200` when `?baud=` is omitted.

use crate::{BridgeError, BridgeResult};
use mavlink::{MavConnection, MavlinkVersion, dialects::common::MavMessage};
use url::Url;

/// Default MAVLink TCP port used when the URL omits one.
pub const DEFAULT_TCP_PORT: u16 = 5760;
/// Default MAVLink UDP port used when the URL omits one.
pub const DEFAULT_UDP_PORT: u16 = 14550;
/// Default serial baud rate used when the `?baud=` query is missing.
pub const DEFAULT_SERIAL_BAUD: u32 = 115_200;

/// Open a MAVLink 2 transport described by `url`.
///
/// The returned connection is pre-configured for MAVLink V2 and is
/// `Send + Sync`, so reader and writer threads may share an `Arc<_>`
/// without an extra mutex.
pub fn connect(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    match url.scheme() {
        "tcp" => connect_tcp(url),
        "udp" => connect_udp(url),
        "serial" => connect_serial(url),
        scheme => Err(BridgeError::Config(format!(
            "unsupported transport scheme '{scheme}' (supported: tcp, udp, serial)"
        ))),
    }
}

fn connect_tcp(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let host = url
        .host_str()
        .ok_or_else(|| BridgeError::Config(format!("missing host in '{url}'")))?;
    let port = url.port().unwrap_or(DEFAULT_TCP_PORT);
    let version = parse_proto_query(url)?;
    open_mavlink_versioned(&format!("tcpout:{host}:{port}"), version)
}

fn connect_udp(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let host = url
        .host_str()
        .ok_or_else(|| BridgeError::Config(format!("missing host in '{url}'")))?;
    let port = url.port().unwrap_or(DEFAULT_UDP_PORT);
    let version = parse_proto_query(url)?;
    open_mavlink_versioned(&format!("udpin:{host}:{port}"), version)
}

fn connect_serial(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    // Windows form `serial://COM1?...` puts the device in `host_str()`;
    // Unix form `serial:///dev/tty.usbmodem1?...` puts it in `path()`
    // (with leading slash, which mavlink's parser keeps).
    let device = match url.host_str() {
        Some(host) if !host.is_empty() => host.to_string(),
        _ => url.path().to_string(),
    };
    if device.is_empty() {
        return Err(BridgeError::Config(format!(
            "missing device path in '{url}'"
        )));
    }
    let baud = parse_baud(url).unwrap_or(DEFAULT_SERIAL_BAUD);
    open_mavlink(&format!("serial:{device}:{baud}"))
}

fn parse_baud(url: &Url) -> Option<u32> {
    url.query_pairs()
        .find(|(k, _)| k == "baud")
        .and_then(|(_, v)| v.parse::<u32>().ok())
}

fn open_mavlink(addr: &str) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    open_mavlink_versioned(addr, MavlinkVersion::V2)
}

fn open_mavlink_versioned(
    addr: &str,
    version: MavlinkVersion,
) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let mut conn = mavlink::connect::<MavMessage>(addr)
        .map_err(|e| BridgeError::Config(format!("failed to connect mavlink to '{addr}': {e}")))?;
    conn.set_protocol_version(version);
    Ok(Box::new(conn))
}

/// Resolve the MAVLink protocol version from an optional `?proto=` query
/// parameter.
///
/// - absent → the [`MavlinkVersion::V2`] default;
/// - `v1`/`1`/`1.0` or `v2`/`2`/`2.0` (case-insensitive) → that version;
/// - anything else → a [`BridgeError::Config`].
///
/// The explicit error on an unrecognized value is deliberate: a typo such as
/// `?proto=v3` previously fell back to V2 silently, so a misconfigured
/// endpoint connected on the wrong protocol with no diagnostic. An absent
/// parameter is distinct from an invalid one and still defaults quietly.
fn parse_proto_query(url: &Url) -> BridgeResult<MavlinkVersion> {
    match url.query_pairs().find(|(k, _)| k == "proto") {
        None => Ok(MavlinkVersion::V2),
        Some((_, v)) => match v.to_ascii_lowercase().as_str() {
            "v1" | "1" | "1.0" => Ok(MavlinkVersion::V1),
            "v2" | "2" | "2.0" => Ok(MavlinkVersion::V2),
            other => Err(BridgeError::Config(format!(
                "unsupported mavlink protocol '{other}' in '{url}' (supported: v1, v2)"
            ))),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_baud_query() {
        let url = Url::parse("serial:///dev/tty.usbmodem1?baud=57600").unwrap();
        assert_eq!(parse_baud(&url), Some(57600));
    }

    #[test]
    fn parses_baud_missing() {
        let url = Url::parse("serial:///dev/tty.usbmodem1").unwrap();
        assert_eq!(parse_baud(&url), None);
    }

    #[test]
    fn parses_baud_garbage() {
        let url = Url::parse("serial:///dev/tty.usbmodem1?baud=fast").unwrap();
        assert_eq!(parse_baud(&url), None);
    }

    #[test]
    fn proto_absent_defaults_to_v2() {
        let url = Url::parse("tcp://127.0.0.1:5760").unwrap();
        assert!(matches!(parse_proto_query(&url), Ok(MavlinkVersion::V2)));
    }

    #[test]
    fn proto_recognized_values() {
        for (q, expected) in [
            ("v1", MavlinkVersion::V1),
            ("1", MavlinkVersion::V1),
            ("1.0", MavlinkVersion::V1),
            ("v2", MavlinkVersion::V2),
            ("2", MavlinkVersion::V2),
            ("2.0", MavlinkVersion::V2),
            ("V2", MavlinkVersion::V2), // case-insensitive
        ] {
            let url = Url::parse(&format!("tcp://host:5760?proto={q}")).unwrap();
            let got = parse_proto_query(&url).unwrap();
            assert_eq!(
                std::mem::discriminant(&got),
                std::mem::discriminant(&expected),
                "proto={q} should parse to {expected:?}"
            );
        }
    }

    #[test]
    fn proto_unrecognized_is_rejected() {
        // Regression: a typo like `?proto=v3` must surface a config error
        // instead of silently falling back to V2.
        for q in ["v3", "3", "typo", ""] {
            let url = Url::parse(&format!("udp://host:14550?proto={q}")).unwrap();
            let err = parse_proto_query(&url).unwrap_err();
            assert!(
                matches!(err, BridgeError::Config(_)),
                "proto={q:?} should be a Config error, got {err:?}"
            );
        }
    }
}
