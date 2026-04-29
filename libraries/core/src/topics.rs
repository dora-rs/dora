use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
/// Env var to override the daemon's local listener port for dynamic nodes.
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_ENV: &str = "DORA_DAEMON_LOCAL_LISTEN_PORT";
pub const DORA_COORDINATOR_PORT_WS_DEFAULT: u16 = 6013;

/// Env var injected by the daemon into spawned nodes that points at the
/// daemon's loopback zenoh listener. Lets nodes bootstrap zenoh peer discovery
/// without multicast (dev containers, locked-down hosts, many CI runners).
pub const DORA_ZENOH_CONNECT_ENV: &str = "DORA_ZENOH_CONNECT";

pub const MANUAL_STOP: &str = "dora/stop";

#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session(coordinator_addr: Option<IpAddr>) -> eyre::Result<zenoh::Session> {
    open_zenoh_session_with_listen(coordinator_addr, None).await
}

/// Like [`open_zenoh_session`], but also configures the session to listen on
/// the given loopback endpoint (e.g. `tcp/127.0.0.1:43217`). The daemon uses
/// this so spawned nodes can connect via `DORA_ZENOH_CONNECT` without
/// multicast scouting.
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session_with_listen(
    coordinator_addr: Option<IpAddr>,
    listen_endpoint: Option<&str>,
) -> eyre::Result<zenoh::Session> {
    use eyre::{Context, eyre};
    use tracing::warn;

    let zenoh_session = match std::env::var(zenoh::Config::DEFAULT_CONFIG_PATH_ENV) {
        Ok(path) => {
            let zenoh_config = zenoh::Config::from_file(&path)
                .map_err(|e| eyre!(e))
                .wrap_err_with(|| format!("failed to read zenoh config from {path}"))?;
            zenoh::open(zenoh_config)
                .await
                .map_err(|e| eyre!(e))
                .context("failed to open zenoh session")?
        }
        Err(std::env::VarError::NotPresent) => {
            let mut zenoh_config = zenoh::Config::default();
            // Linkstate make it possible to connect two daemons on different network through a public daemon
            // TODO: There is currently a CI/CD Error in windows linkstate.
            if cfg!(not(target_os = "windows")) {
                zenoh_config
                    .insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
                    .unwrap();
            }

            // Latency note: each data-plane publisher in
            // `apis/rust/node/src/node/mod.rs::zenoh_publish` is declared with
            // `express(true)`, which bypasses zenoh's adaptive batch timer per
            // publication. We deliberately do NOT enable Zenoh's inter-peer
            // SHM transport (`transport/shared_memory/enabled`) — it is
            // page-aligned and its setup overhead does not pay off below
            // 4 KiB, where the existing daemon-relay path is used instead.
            // Zenoh's TCP link layer already sets TCP_NODELAY unconditionally
            // (see `zenoh-link-tcp::unicast::set_nodelay(true)`), so no
            // tcp_nodelay knob is needed here.

            // Daemon-bootstrapped local discovery: when DORA_ZENOH_CONNECT is
            // set in the process env (the daemon injects it into spawned
            // nodes), seed connect/endpoints from it and disable multicast
            // scouting. This makes the >=4 KiB zenoh data path work in
            // environments without working multicast (#1778).
            if let Ok(ep) = std::env::var(DORA_ZENOH_CONNECT_ENV) {
                zenoh_config
                    .insert_json5("connect/endpoints", &format!(r#"["{ep}"]"#))
                    .unwrap();
                zenoh_config
                    .insert_json5("scouting/multicast/enabled", "false")
                    .unwrap();
            }

            if let Some(ep) = listen_endpoint {
                zenoh_config
                    .insert_json5("listen/endpoints", &format!(r#"["{ep}"]"#))
                    .unwrap();
                // Tolerate a race between OS port reservation and zenoh's own
                // bind on the same port: the connect side still works, and
                // child nodes get a clear error rather than the daemon
                // exiting.
                zenoh_config
                    .insert_json5("listen/exit_on_failure", "false")
                    .unwrap();
            }

            if let Some(addr) = coordinator_addr {
                zenoh_config
                    .insert_json5(
                        "connect/endpoints",
                        &format!(
                            r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                            addr
                        ),
                    )
                    .unwrap();
            }
            if let Ok(zenoh_session) = zenoh::open(zenoh_config).await {
                zenoh_session
            } else {
                warn!("failed to open zenoh session, retrying with default config");
                let zenoh_config = zenoh::Config::default();
                zenoh::open(zenoh_config)
                    .await
                    .map_err(|e| eyre!(e))
                    .context("failed to open zenoh session")?
            }
        }
        Err(std::env::VarError::NotUnicode(_)) => eyre::bail!(
            "{} env variable is not valid unicode",
            zenoh::Config::DEFAULT_CONFIG_PATH_ENV
        ),
    };
    Ok(zenoh_session)
}

/// Reserve an unused TCP port on `127.0.0.1` for use as a zenoh listen
/// endpoint. Returns a string suitable for the zenoh `listen/endpoints`
/// config (e.g. `tcp/127.0.0.1:43217`).
///
/// There is a small race window between dropping the reservation socket and
/// zenoh's own bind. `open_zenoh_session_with_listen` sets
/// `listen/exit_on_failure: false` so the daemon survives that race; in
/// practice the OS keeps allocating new ports so collisions are vanishingly
/// rare.
pub fn reserve_loopback_zenoh_endpoint() -> std::io::Result<String> {
    let listener = std::net::TcpListener::bind((Ipv4Addr::LOCALHOST, 0))?;
    let port = listener.local_addr()?.port();
    drop(listener);
    Ok(format!("tcp/127.0.0.1:{port}"))
}

#[cfg(feature = "zenoh")]
pub fn zenoh_output_publish_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    format!("dora/{network_id}/{dataflow_id}/output/{node_id}/{output_id}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reserve_loopback_endpoint_returns_loopback_tcp() {
        let endpoint = reserve_loopback_zenoh_endpoint().expect("reservation succeeds");
        assert!(
            endpoint.starts_with("tcp/127.0.0.1:"),
            "expected loopback tcp endpoint, got {endpoint}"
        );
        let port: u16 = endpoint
            .rsplit(':')
            .next()
            .and_then(|p| p.parse().ok())
            .expect("endpoint has a numeric port");
        assert!(port > 0, "kernel must hand out a non-zero ephemeral port");
    }
}
