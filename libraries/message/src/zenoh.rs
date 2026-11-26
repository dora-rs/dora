//! Zenoh endpoint classification shared by advertisement senders and receivers.

use std::net::IpAddr;

/// Check that `endpoint` has zenoh's endpoint shape:
/// `<protocol>/<address>[?<metadata>][#<config>]`.
///
/// This is zenoh's own parser, and it does not depend on the protocol: it
/// accepts any protocol name, including ones dora does not use. It is the
/// check zenoh runs on every `connect/endpoints` entry when the config is
/// built, so a value it rejects could never have been dialed.
///
/// ```
/// use dora_message::zenoh::check_zenoh_endpoint_syntax;
///
/// assert!(check_zenoh_endpoint_syntax("tcp/10.0.0.5:5456").is_ok());
/// assert!(check_zenoh_endpoint_syntax("quic/[fd7a:1::2]:7447?prio=high").is_ok());
/// // No protocol: zenoh rejects it.
/// assert!(check_zenoh_endpoint_syntax("10.0.0.5:5456").is_err());
/// ```
pub fn check_zenoh_endpoint_syntax(endpoint: &str) -> Result<(), String> {
    endpoint
        .parse::<zenoh_protocol::core::EndPoint>()
        .map(drop)
        .map_err(|err| err.to_string())
}

/// Whether a zenoh endpoint (`tcp/127.0.0.1:5456`, `tcp/[::1]:5456`,
/// `tcp/localhost:5456`, optionally with `?metadata` or `#config` suffixes)
/// names a loopback address, which only reaches the host it was bound on.
///
/// An endpoint that does not parse counts as not loopback.
pub fn zenoh_endpoint_is_loopback(endpoint: &str) -> bool {
    // zenoh's own parser strips the protocol and the `?metadata` / `#config`
    // suffixes; what is left is `host:port`, bracketed for IPv6.
    let Ok(endpoint) = endpoint.parse::<zenoh_protocol::core::EndPoint>() else {
        return false;
    };
    let address = endpoint.address().as_str();
    // `[v6]:port`, `[v6]`, `v4:port`, or a bare host.
    let host = match address.strip_prefix('[') {
        Some(rest) => rest.split(']').next().unwrap_or_default(),
        None => address
            .rsplit_once(':')
            .map_or(address, |(host, _port)| host),
    };
    // Host names are case-insensitive (RFC 4343), and a trailing dot only
    // marks the name as fully qualified, so `LOCALHOST` and `localhost.` name
    // loopback just like `localhost`.
    let is_localhost = |host: &str| {
        host.strip_suffix('.')
            .unwrap_or(host)
            .eq_ignore_ascii_case("localhost")
    };
    host.parse::<IpAddr>()
        .map_or_else(|_| is_localhost(host), |ip| ip.to_canonical().is_loopback())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loopback_endpoints_are_recognized_in_every_spelling() {
        for endpoint in [
            "tcp/127.0.0.1:5456",
            "tcp/127.0.0.1:5456?prio=high#iface=lo",
            "tcp/127.0.0.1:5456#iface=lo",
            "tcp/[::1]:5456",
            "tcp/[::1]",
            "tcp/[::1]:5456?prio=high#iface=lo",
            "tcp/[::ffff:127.0.0.1]:5456",
            "tcp/localhost:5456",
            "tcp/LOCALHOST:5456",
            "tcp/Localhost:5456",
            "tcp/localhost.:5456",
            "udp/127.1.2.3:1",
        ] {
            assert!(zenoh_endpoint_is_loopback(endpoint), "{endpoint}");
        }
        for endpoint in [
            "tcp/10.0.2.100:5456",
            "tcp/[fd7a:1::2]:5456",
            "tcp/robot-01.local:5456",
            "tcp/0.0.0.0:5456",
            "not-an-endpoint",
            "",
        ] {
            assert!(!zenoh_endpoint_is_loopback(endpoint), "{endpoint}");
        }
    }
}
