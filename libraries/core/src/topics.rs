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
    let (session, _) = open_zenoh_session_with_listen(coordinator_addr, None).await?;
    Ok(session)
}

/// Like [`open_zenoh_session`], but also configures the session to listen on
/// the given loopback endpoint (e.g. `tcp/127.0.0.1:43217`). The daemon uses
/// this so spawned nodes can connect via `DORA_ZENOH_CONNECT` without
/// multicast scouting.
///
/// Returns `(session, effective_listen_endpoint)`. The second element is
/// `Some(ep)` only when `listen_endpoint` was requested, zenoh accepted the
/// `listen/endpoints` insert, and `session.info().locators()` confirms that
/// it actually bound. It is `None` if `listen_endpoint` was `None`, the insert
/// failed, the open path used the `ZENOH_CONFIG_PATH`-from-file branch, or the
/// configured listener did not bind. Callers must inject the returned endpoint
/// into peers (e.g. via `DORA_ZENOH_CONNECT`) instead of the value they passed
/// in, so peers never receive a stale endpoint the listener did not actually
/// bind (#1856, #1858).
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session_with_listen(
    coordinator_addr: Option<IpAddr>,
    listen_endpoint: Option<&str>,
) -> eyre::Result<(zenoh::Session, Option<String>)> {
    use eyre::{Context, eyre};
    use tracing::warn;

    // Source-of-truth for the listener: stays `None` unless we actually
    // accepted `listen/endpoints` into the config below. Callers use this
    // (not their requested endpoint) to advertise the listener to peers.
    let mut effective_listen_endpoint: Option<String> = None;

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
            if cfg!(not(target_os = "windows"))
                && let Err(err) =
                    zenoh_config.insert_json5("routing/peer", r#"{ mode: "linkstate" }"#)
            {
                warn!("failed to set zenoh routing/peer to linkstate: {err}");
            }

            // Dora's data plane favors per-message latency over Zenoh's
            // default adaptive batching path. Publishers also set
            // `express(true)`, but low-latency unicast lets peer transports
            // skip the universal batching/priority queues entirely when both
            // ends use Dora's default config. Zenoh's low-latency transport is
            // negotiated without QoS, so disable QoS together with it instead
            // of falling back during session open. We deliberately do NOT enable
            // Zenoh's inter-peer SHM transport (`transport/shared_memory/enabled`):
            // Dora nodes select Zenoh SHM per payload, while small payloads
            // stay heap-buffered because page-aligned SHM setup does not pay
            // off below the zero-copy threshold.
            zenoh_config
                .insert_json5("transport/unicast/lowlatency", "true")
                .unwrap();
            zenoh_config
                .insert_json5("transport/unicast/qos/enabled", "false")
                .unwrap();

            // Daemon-bootstrapped local discovery: when DORA_ZENOH_CONNECT is
            // set in the process env (the daemon injects it into spawned
            // nodes), seed connect/endpoints from it and disable multicast
            // scouting. This makes the >=4 KiB zenoh data path work in
            // environments without working multicast (#1778).
            if let Ok(ep) = std::env::var(DORA_ZENOH_CONNECT_ENV) {
                // Only disable multicast scouting if we successfully replaced
                // it with an explicit connect endpoint — otherwise the node
                // would have neither and open an isolated session (#1856).
                let connect_inserted = match zenoh_config
                    .insert_json5("connect/endpoints", &format!(r#"["{ep}"]"#))
                {
                    Ok(()) => true,
                    Err(err) => {
                        warn!(
                            "failed to set zenoh connect/endpoints from DORA_ZENOH_CONNECT ({err}); leaving multicast scouting enabled as fallback"
                        );
                        false
                    }
                };
                if connect_inserted
                    && let Err(err) =
                        zenoh_config.insert_json5("scouting/multicast/enabled", "false")
                {
                    warn!("failed to disable zenoh scouting/multicast: {err}");
                }
            }

            // Track whether listen/endpoints was accepted into THIS config.
            // We don't promote it to `effective_listen_endpoint` until the
            // configured open succeeds — the fallback default-config path
            // below has no listener and must not advertise one (#1856).
            let mut listen_inserted_into_configured: Option<String> = None;

            if let Some(ep) = listen_endpoint {
                // `listen/exit_on_failure: false` is a tuning knob for the
                // listener; only set it if the listener itself got
                // configured (#1856).
                let listen_inserted =
                    match zenoh_config.insert_json5("listen/endpoints", &format!(r#"["{ep}"]"#)) {
                        Ok(()) => {
                            listen_inserted_into_configured = Some(ep.to_string());
                            true
                        }
                        Err(err) => {
                            warn!("failed to set zenoh listen/endpoints to `{ep}`: {err}");
                            false
                        }
                    };
                // Tolerate a race between OS port reservation and zenoh's own
                // bind on the same port: the connect side still works, and
                // child nodes get a clear error rather than the daemon
                // exiting.
                if listen_inserted
                    && let Err(err) = zenoh_config.insert_json5("listen/exit_on_failure", "false")
                {
                    warn!("failed to set zenoh listen/exit_on_failure: {err}");
                }
            }

            if let Some(addr) = coordinator_addr
                && let Err(err) = zenoh_config.insert_json5(
                    "connect/endpoints",
                    &format!(
                        r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{}:5456"] }}"#,
                        addr
                    ),
                )
            {
                warn!("failed to set zenoh connect/endpoints for coordinator {addr}: {err}");
            }
            match zenoh::open(zenoh_config).await {
                Ok(zenoh_session) => {
                    // Verify the listener actually bound. `zenoh::open` returning
                    // Ok is necessary but not sufficient — with
                    // `listen/exit_on_failure: false` (set above), zenoh tolerates
                    // a silently-failed listen bind. The most plausible cause is
                    // the race between `reserve_loopback_zenoh_endpoint` dropping
                    // its reservation socket and zenoh's own bind, during which
                    // some other process could grab the port. Trusting `Ok` here
                    // would advertise an endpoint nothing is listening on, and
                    // spawned nodes would fail their `DORA_ZENOH_CONNECT` connect
                    // attempts (#1858).
                    //
                    // `info().locators()` is the zenoh-canonical "what actually
                    // bound" query (unstable API gated behind the workspace
                    // `unstable` feature, already enabled in the root Cargo.toml
                    // zenoh dependency).
                    if let Some(requested) = listen_inserted_into_configured {
                        let bound_locators: Vec<String> = zenoh_session
                            .info()
                            .locators()
                            .await
                            .into_iter()
                            .map(|l| l.as_str().to_string())
                            .collect();
                        // Strip zenoh's endpoint-string separators before
                        // comparing. Per `zenoh-protocol::core::endpoint`,
                        // `?` separates metadata and `#` separates config
                        // (e.g. `tcp/127.0.0.1:43217?prio=high#iface=lo0`).
                        // `Locator::from(EndPoint)` already truncates `#`,
                        // but a `?`-metadata suffix would survive into
                        // `info().locators()`'s output. Strip both, then
                        // exact-match — substring `contains` would
                        // false-positive on port-prefix collisions (e.g.
                        // requested `:5000` matching bound `:50000`), which
                        // is exactly the mismatch this check exists to
                        // catch. NOTE: the comparison is by canonical
                        // `tcp/127.0.0.1:<port>` form; callers passing
                        // non-canonical loopback forms (`localhost`, IPv6
                        // mapped) won't match — `reserve_loopback_zenoh_
                        // endpoint` only emits the canonical form, so this
                        // is fine for the daemon's only call site.
                        let bound = bound_locators
                            .iter()
                            .any(|l| l.split(['?', '#']).next() == Some(requested.as_str()));
                        if bound {
                            effective_listen_endpoint = Some(requested);
                        } else {
                            warn!(
                                "zenoh session opened but listener for `{requested}` \
                                 did not bind (actually bound: {bound_locators:?}); \
                                 spawned nodes will use multicast scouting only"
                            );
                        }
                    }
                    zenoh_session
                }
                Err(err) => {
                    warn!(
                        "failed to open tuned zenoh session ({err}), retrying with default config"
                    );
                    // Default fallback has no listener; `effective_listen_endpoint`
                    // stays `None` so peers don't try to reach a bind that isn't
                    // there (#1856).
                    let zenoh_config = zenoh::Config::default();
                    zenoh::open(zenoh_config)
                        .await
                        .map_err(|e| eyre!(e))
                        .context("failed to open zenoh session")?
                }
            }
        }
        Err(std::env::VarError::NotUnicode(_)) => eyre::bail!(
            "{} env variable is not valid unicode",
            zenoh::Config::DEFAULT_CONFIG_PATH_ENV
        ),
    };
    Ok((zenoh_session, effective_listen_endpoint))
}

/// Reserve an unused TCP port on `127.0.0.1` for use as a zenoh listen
/// endpoint. Returns a string suitable for the zenoh `listen/endpoints`
/// config (e.g. `tcp/127.0.0.1:43217`).
///
/// There is a small race window between dropping the reservation socket
/// and zenoh's own bind. `open_zenoh_session_with_listen` defends against
/// it on two layers:
///
/// 1. `listen/exit_on_failure: false` keeps the daemon alive if zenoh's
///    bind silently fails inside the race window.
/// 2. After `zenoh::open`, the helper queries `session.info().locators()`
///    and only advertises the returned endpoint when our requested
///    `addr:port` is actually in the bound-locator list. If the port was
///    grabbed by another process, the returned `effective_listen_endpoint`
///    is `None` and callers fall back to multicast scouting instead of
///    advertising a phantom endpoint (#1858).
///
/// In practice the OS keeps allocating fresh ephemeral ports each call,
/// so collisions remain vanishingly rare.
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
