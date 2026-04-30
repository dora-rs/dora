//! Open MAVLink 2 transports from a `url::Url`.
//!
//! Supported URL schemes:
//!
//! | URL form                              | mavlink address              | Mode      |
//! |---------------------------------------|------------------------------|-----------|
//! | `tcp://host:port`                     | `tcpout:host:port`           | client    |
//! | `udp://host:port`                     | `udpin:host:port`            | server    |
//! | `serial:///dev/path?baud=N` (Unix)    | `serial:/dev/path:N`         | n/a       |
//! | `serial://COM1?baud=N` (Windows)      | `serial:COM1:N`              | n/a       |
//!
//! UDP defaults to *server* mode (listening) because the typical
//! use case is receiving Pixhawk telemetry on a fixed port. Override by
//! constructing the URL with a query that future PRs may add. For now,
//! callers needing UDP client mode can drop down to `mavlink::connect`
//! directly.
//!
//! Serial baud defaults to 115200 if the `?baud=` query is omitted.

use crate::{BridgeError, BridgeResult};
use mavlink::{MavConnection, MavlinkVersion, common::MavMessage};
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
    open_mavlink(&format!("tcpout:{host}:{port}"))
}

fn connect_udp(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let host = url
        .host_str()
        .ok_or_else(|| BridgeError::Config(format!("missing host in '{url}'")))?;
    let port = url.port().unwrap_or(DEFAULT_UDP_PORT);
    open_mavlink(&format!("udpin:{host}:{port}"))
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
    let mut conn = mavlink::connect::<MavMessage>(addr)
        .map_err(|e| BridgeError::Config(format!("failed to connect mavlink to '{addr}': {e}")))?;
    conn.set_protocol_version(MavlinkVersion::V2);
    Ok(conn)
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
}
