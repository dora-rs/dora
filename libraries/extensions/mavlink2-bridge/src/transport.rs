//! Open MAVLink 2 transports from a `url::Url`.
//!
//! URL scheme map (this PR adds `tcp` only; `udp`/`serial` land in the
//! follow-up):
//!
//! | URL                                | mavlink address    | Mode   |
//! |------------------------------------|--------------------|--------|
//! | `tcp://host:port`                  | `tcpout:host:port` | client |

use crate::{BridgeError, BridgeResult};
use mavlink::{MavConnection, MavlinkVersion, common::MavMessage};
use url::Url;

/// Default MAVLink TCP port used when the URL omits one.
pub const DEFAULT_TCP_PORT: u16 = 5760;

/// Open a MAVLink 2 transport described by `url`.
///
/// The returned connection is pre-configured for MAVLink V2 and is
/// `Send + Sync`, so reader and writer threads may share an `Arc<_>`
/// without an extra mutex (see `MavConnection::recv/send` taking
/// `&self`).
pub fn connect(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    match url.scheme() {
        "tcp" => connect_tcp(url),
        scheme => Err(BridgeError::Config(format!(
            "unsupported transport scheme '{scheme}' (this build supports: tcp)"
        ))),
    }
}

fn connect_tcp(url: &Url) -> BridgeResult<Box<dyn MavConnection<MavMessage> + Send + Sync>> {
    let host = url
        .host_str()
        .ok_or_else(|| BridgeError::Config(format!("missing host in '{url}'")))?;
    let port = url.port().unwrap_or(DEFAULT_TCP_PORT);
    let address = format!("tcpout:{host}:{port}");
    let mut conn = mavlink::connect::<MavMessage>(&address).map_err(|e| {
        BridgeError::Config(format!("failed to connect mavlink to '{address}': {e}"))
    })?;
    conn.set_protocol_version(MavlinkVersion::V2);
    Ok(conn)
}
