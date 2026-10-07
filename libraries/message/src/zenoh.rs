//! Zenoh endpoint classification shared by advertisement senders and receivers.

use std::net::IpAddr;

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
    host.parse::<IpAddr>()
        .map_or(host == "localhost", |ip| ip.to_canonical().is_loopback())
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
