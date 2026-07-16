use std::net::{IpAddr, Ipv4Addr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
/// Env var to override the daemon's local listener port for dynamic nodes.
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_ENV: &str = "DORA_DAEMON_LOCAL_LISTEN_PORT";
pub const DORA_COORDINATOR_PORT_WS_DEFAULT: u16 = 6013;

/// Comma-separated zenoh endpoints a spawned node should connect to, injected by
/// the daemon: the daemon's own loopback listener plus the listeners of the nodes
/// this one consumes from (see [`DORA_ZENOH_LISTEN_ENV`]).
///
/// Lets nodes bootstrap zenoh peer discovery without multicast (dev containers,
/// locked-down hosts, many CI runners), and — since zenoh 1.9 removed peer
/// relaying — establishes the node↔node links the dataflow needs *explicitly*
/// rather than leaving them to gossip's best-effort autoconnect.
pub const DORA_ZENOH_CONNECT_ENV: &str = "DORA_ZENOH_CONNECT";

/// Loopback zenoh endpoint a spawned node should listen on, injected by the
/// daemon so this node's consumers can dial it directly (they receive it via
/// their [`DORA_ZENOH_CONNECT_ENV`]).
///
/// Peers in zenoh 1.9 do not relay for each other, so a producer and consumer
/// that never form a direct link simply cannot exchange data — no amount of
/// waiting fixes it. Assigning each node a known listener makes those links
/// deterministic instead of racy.
pub const DORA_ZENOH_LISTEN_ENV: &str = "DORA_ZENOH_LISTEN";

/// Split a comma-separated endpoint list env var, ignoring empty entries.
#[cfg(feature = "zenoh")]
fn split_endpoints(value: &str) -> impl Iterator<Item = String> + '_ {
    value
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(String::from)
}

#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session(coordinator_addr: Option<IpAddr>) -> eyre::Result<zenoh::Session> {
    let (session, _) = open_zenoh_session_with_listen(coordinator_addr, None, None).await?;
    Ok(session)
}

/// Like [`open_zenoh_session`], but also configures the session to listen on
/// the given loopback endpoint (e.g. `tcp/127.0.0.1:43217`). The daemon uses
/// this so spawned nodes can connect via `DORA_ZENOH_CONNECT` without
/// multicast scouting.
///
/// `inter_daemon_peer` is an optional shared endpoint used as the
/// rendezvous for daemon-to-daemon discovery when multicast isn't
/// available. When set, it is added to both `listen/endpoints` and
/// `connect/endpoints`: the first daemon to bind it serves as the
/// gossip hub, and the rest fall through to connect-only via
/// `listen/exit_on_failure: false`. Multicast scouting is disabled in
/// that mode since we have explicit endpoints. This complements the
/// per-spawned-node `DORA_ZENOH_CONNECT` fallback from #1778, which
/// only covers daemon↔node, leaving daemon↔daemon dependent on
/// multicast — broken in dev containers and many CI environments.
///
/// Returns `(session, effective_listen_endpoint)`. The second element is
/// `Some(ep)` only when `listen_endpoint` was requested, zenoh accepted the
/// `listen/endpoints` insert, and `session.info().locators()` confirms that
/// it actually bound. It is `None` if `listen_endpoint` was `None`, the insert
/// failed, the open path used the `ZENOH_CONFIG_PATH`-from-file branch, or the
/// configured listener did not bind. Callers must inject the returned endpoint
/// into peers (e.g. via `DORA_ZENOH_CONNECT`) instead of the value they passed
/// in, so peers never receive a stale endpoint the listener did not actually
/// bind (#1856, #1858). The `inter_daemon_peer` is intentionally NOT
/// part of the returned endpoint — it is cluster-wide configuration
/// shared by the caller (e.g. `dora cluster up`), not per-daemon
/// state to advertise back to nodes.
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session_with_listen(
    coordinator_addr: Option<IpAddr>,
    listen_endpoint: Option<&str>,
    inter_daemon_peer: Option<&str>,
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
            // NOTE: we used to set `routing/peer: { mode: "linkstate" }` here so
            // that peers would relay for each other (e.g. two daemons on separate
            // networks reaching each other through a public one). Zenoh 1.9
            // *removed* peer routing modes — "routing.peer is removed, all peers
            // now operate in peer-to-peer mode" — so the setting became a silent
            // no-op (`insert_json5` still returns `Ok`, and only zenoh logs a
            // deprecation warning). It is deleted rather than ported because
            // there is no 1.9 equivalent for peers: peer regions must now form a
            // clique, and `peers_failover_brokering` was removed too. Only
            // *routers* still do linkstate routing.
            //
            // Consequences, and why this is not a regression here:
            //   * Same-machine nodes are all loopback-addressable, so the links
            //     the dataflow needs are established explicitly via
            //     `connect/endpoints` below (see `DORA_ZENOH_CONNECT`) instead of
            //     being left to gossip's best-effort autoconnect.
            //   * Multi-machine/NAT setups, which is what linkstate was meant to
            //     serve, supply their own config via `ZENOH_CONFIG_PATH` (handled
            //     in the branch above) and can put a real router in the path.
            //
            // Dora's data plane favors per-message latency over Zenoh's
            // default adaptive batching path. Publishers also set
            // `express(true)`, but low-latency unicast lets peer transports
            // skip the universal batching/priority queues entirely when both
            // ends use Dora's default config. Zenoh's low-latency transport is
            // negotiated without QoS, so disable QoS together with it instead
            // of falling back during session open.
            //
            // We rely on zenoh's SHM transport (`transport/shared_memory/enabled`)
            // being enabled, which is its default — do NOT set it to `false`: the
            // API keeps working, but SHM buffers silently get serialized as plain
            // bytes onto the wire (i.e. copied) instead of sent as a ~16-byte
            // descriptor. Worse, `lowlatency` cannot fragment (message must stay
            // under `batch_size`, 64 KiB), so without SHM a payload above that has
            // no working path at all — zenoh's "fallback to network mode" is not
            // transparent under this config.
            zenoh_config
                .insert_json5("transport/unicast/lowlatency", "true")
                .unwrap();
            zenoh_config
                .insert_json5("transport/unicast/qos/enabled", "false")
                .unwrap();

            // Build the connect-endpoint list from two sources:
            //   1. DORA_ZENOH_CONNECT env var — daemon-bootstrapped local
            //      discovery for spawned nodes (#1778).
            //   2. `inter_daemon_peer` — shared rendezvous for daemon-to-
            //      daemon discovery (extends #1778 to the daemon↔daemon
            //      hop). One daemon binds it as a listener, others connect
            //      and gossip-discover their peers via it.
            // Both are explicit endpoints; if we set any of them we
            // disable multicast scouting so we don't end up with mixed
            // discovery modes.
            let mut connect_eps: Vec<String> = Vec::new();
            if let Ok(eps) = std::env::var(DORA_ZENOH_CONNECT_ENV) {
                connect_eps.extend(split_endpoints(&eps));
            }
            if let Some(peer) = inter_daemon_peer {
                connect_eps.push(peer.to_string());
            }
            let mut connect_inserted = false;
            if !connect_eps.is_empty() {
                let json = format!(
                    "[{}]",
                    connect_eps
                        .iter()
                        .map(|s| format!(r#""{s}""#))
                        .collect::<Vec<_>>()
                        .join(",")
                );
                match zenoh_config.insert_json5("connect/endpoints", &json) {
                    Ok(()) => connect_inserted = true,
                    Err(err) => {
                        warn!(
                            "failed to set zenoh connect/endpoints to {json} ({err}); leaving multicast scouting enabled as fallback"
                        );
                    }
                }
            }
            // Only disable multicast scouting if we successfully replaced
            // it with explicit connect endpoints — otherwise we'd end up
            // with no discovery at all (#1856).
            if connect_inserted
                && let Err(err) = zenoh_config.insert_json5("scouting/multicast/enabled", "false")
            {
                warn!("failed to disable zenoh scouting/multicast: {err}");
            }

            // Track whether listen/endpoints was accepted into THIS config.
            // We don't promote it to `effective_listen_endpoint` until the
            // configured open succeeds — the fallback default-config path
            // below has no listener and must not advertise one (#1856).
            // We only track `listen_endpoint` (the per-daemon loopback that
            // gets advertised to spawned nodes), NOT `inter_daemon_peer`
            // which is cluster-wide config — daemons that bind it act as
            // the rendezvous, but advertising it back to nodes would be
            // wrong (nodes would try to reach it through what may be a
            // remote address, defeating the loopback shortcut).
            let mut listen_inserted_into_configured: Option<String> = None;

            // Build the listen-endpoint list (loopback for spawned nodes +
            // optional inter-daemon rendezvous). With multiple entries,
            // zenoh binds whichever ones it can; `listen/exit_on_failure:
            // false` (set below when any listener is configured) lets the
            // daemon proceed even if some don't bind — e.g. the second
            // daemon to start on the same host with the same rendezvous
            // port falls through to connect-only.
            // A spawned node gets its listener from the daemon via
            // `DORA_ZENOH_LISTEN` (the daemon itself passes `listen_endpoint`
            // directly). Without a known listener a node cannot be dialled, and
            // since zenoh 1.9 peers do not relay, a consumer that cannot dial its
            // producer never receives its data at all.
            let env_listen_endpoint = std::env::var(DORA_ZENOH_LISTEN_ENV).ok();
            let listen_endpoint = listen_endpoint.or(env_listen_endpoint.as_deref());

            let mut listen_eps: Vec<String> = Vec::new();
            if let Some(ep) = listen_endpoint {
                listen_eps.push(ep.to_string());
            }
            if let Some(peer) = inter_daemon_peer {
                listen_eps.push(peer.to_string());
            }
            if !listen_eps.is_empty() {
                let json = format!(
                    "[{}]",
                    listen_eps
                        .iter()
                        .map(|s| format!(r#""{s}""#))
                        .collect::<Vec<_>>()
                        .join(",")
                );
                let listen_inserted = match zenoh_config.insert_json5("listen/endpoints", &json) {
                    Ok(()) => {
                        listen_inserted_into_configured = listen_endpoint.map(String::from);
                        true
                    }
                    Err(err) => {
                        warn!("failed to set zenoh listen/endpoints to {json}: {err}");
                        false
                    }
                };
                // Tolerate a race between OS port reservation and zenoh's
                // own bind, AND the multi-daemon-same-rendezvous case where
                // only one daemon wins the bind. The connect side still
                // works, and child nodes get a clear error rather than the
                // daemon exiting.
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

/// Zenoh key for node output data.
///
/// Payload format: raw Arrow bytes with bincode `Metadata` in the Zenoh
/// attachment. This topic is published by nodes and consumed directly by
/// downstream nodes (plus debug-inspection subscribers). Daemon control frames
/// must not be published here; use [`zenoh_daemon_control_topic`] instead.
#[cfg(feature = "zenoh")]
pub fn zenoh_output_publish_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    format!("dora/{network_id}/{dataflow_id}/output/{node_id}/{output_id}")
}

/// Zenoh key carrying the Arrow IPC **schema** for an output's data topic, as a
/// `/@schema` sub-key of [`zenoh_output_publish_topic`]. The producer publishes
/// the schema here (on change) through a zenoh-ext `AdvancedPublisher` whose
/// cache retains the last sample; a subscriber's `AdvancedSubscriber` history
/// query fetches it on join, so the data topic only ever carries schema-less
/// record batches. The `@`-prefixed final chunk keeps it from matching the
/// concrete data key (no cross-delivery to the data subscriber).
#[cfg(feature = "zenoh")]
pub fn zenoh_output_schema_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    format!(
        "{}/@schema",
        zenoh_output_publish_topic(dataflow_id, node_id, output_id)
    )
}

/// Zenoh key for control frames associated with a node output.
///
/// Payload format: bincode `Timestamped<InterDaemonEvent>` with no Zenoh
/// attachment. Published by daemons for inter-daemon control (for example
/// `OutputClosed`) and by the coordinator for explicit topic injection. Keeping
/// this separate from [`zenoh_output_publish_topic`] avoids mixing control frames
/// with raw node output payloads on the same key.
#[cfg(feature = "zenoh")]
pub fn zenoh_daemon_control_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    format!("dora/{network_id}/{dataflow_id}/control/{node_id}/{output_id}")
}

/// Hex-encode a `DataId` so it occupies exactly one zenoh key chunk.
///
/// A `DataId` may legally contain `/` (unlike a `NodeId`), so embedding one
/// verbatim as a key segment would spill into extra chunks and let a producer's
/// wildcard readiness subscription collide across outputs whose names nest —
/// e.g. output `cmd` would over-count tokens belonging to output `cmd/vel`,
/// which in the startup barrier could switch `cmd` to the direct path before all
/// of *its* subscribers are wired and drop messages. Hex is unambiguous
/// (`[0-9a-f]`, never `/`), collision-free, and computed identically by the
/// subscriber (declaring the token) and the producer (matching it).
#[cfg(feature = "zenoh")]
fn hex_key_segment(id: &dora_message::id::DataId) -> String {
    use std::fmt::Write;
    let s: &str = id.as_ref();
    let mut out = String::with_capacity(s.len() * 2);
    for b in s.bytes() {
        let _ = write!(out, "{b:02x}");
    }
    out
}

/// Zenoh **liveliness** key a subscriber declares once it has wired up its
/// data-plane subscriber for `source_node`'s `source_output`.
///
/// The data plane is direct node-to-node zenoh pub/sub, so a producer that
/// starts publishing before a consumer's subscription has propagated would drop
/// those early samples (zenoh does not buffer for not-yet-declared subscribers).
/// Each subscriber therefore declares a liveliness token here right after
/// declaring its data subscriber; the producer counts these tokens (via
/// [`zenoh_output_ready_liveliness_prefix`]) and keeps delivering over the
/// reliable daemon path until every expected subscriber is present, only then
/// switching to the fast zenoh path. This gives startup a lossless barrier that
/// the daemon control-plane "all nodes ready" gate cannot (it does not observe
/// the zenoh data plane). The `<subscriber_node>/<subscriber_input>` suffix makes
/// each link's token unique so producers can count distinct subscribers.
///
/// `source_output` is [hex-encoded](hex_key_segment) into a single chunk so it
/// cannot collide with a differently-named output whose key would nest under the
/// producer's wildcard subscription.
#[cfg(feature = "zenoh")]
pub fn zenoh_input_ready_liveliness_topic(
    dataflow_id: uuid::Uuid,
    source_node: &dora_message::id::NodeId,
    source_output: &dora_message::id::DataId,
    subscriber_node: &dora_message::id::NodeId,
    subscriber_input: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    let output = hex_key_segment(source_output);
    let input = hex_key_segment(subscriber_input);
    format!(
        "dora/{network_id}/{dataflow_id}/ready/{source_node}/{output}/{subscriber_node}/{input}"
    )
}

/// Wildcard liveliness key matching every subscriber-ready token for
/// `source_node`'s `source_output` (see [`zenoh_input_ready_liveliness_topic`]).
/// The producer subscribes to / queries this to count how many subscribers have
/// wired up their data-plane subscription.
#[cfg(feature = "zenoh")]
pub fn zenoh_output_ready_liveliness_prefix(
    dataflow_id: uuid::Uuid,
    source_node: &dora_message::id::NodeId,
    source_output: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    let output = hex_key_segment(source_output);
    // `source_output` is hex-encoded to a single chunk (see
    // `zenoh_input_ready_liveliness_topic`), so `**` matches exactly the
    // `<subscriber_node>/<subscriber_input>` suffix of tokens for *this* output
    // and nothing from a nested-named output.
    format!("dora/{network_id}/{dataflow_id}/ready/{source_node}/{output}/**")
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

    // Node raw output and daemon control frames MUST live on distinct Zenoh
    // keys: they share neither format nor consumer, and merging them caused the
    // #1992 crossover (daemon bincode-decoding node output). Guard the split.
    #[cfg(feature = "zenoh")]
    #[test]
    fn output_and_control_topics_are_distinct() {
        use dora_message::id::{DataId, NodeId};

        let dataflow_id = uuid::Uuid::nil();
        let node = NodeId::from("node".to_string());
        let output = DataId::from("out".to_string());

        let output_topic = zenoh_output_publish_topic(dataflow_id, &node, &output);
        let control_topic = zenoh_daemon_control_topic(dataflow_id, &node, &output);

        assert!(
            output_topic.contains("/output/"),
            "node output key must contain `/output/`, got {output_topic}"
        );
        assert!(
            control_topic.contains("/control/"),
            "daemon control key must contain `/control/`, got {control_topic}"
        );
        assert_ne!(
            output_topic, control_topic,
            "node output and daemon control must not share a Zenoh key (dora #1992/#2008)"
        );
    }

    // The readiness barrier counts a producer's subscriber tokens via a wildcard
    // key. Because a `DataId` may contain `/`, a naive `.../{output}/**` prefix
    // would also match tokens of a nested-named output (`cmd` matching
    // `cmd/vel`), over-counting and letting the producer switch `cmd` to the
    // direct path before all of *its* subscribers are wired — dropping startup
    // messages. Hex-encoding the output segment must prevent that collision while
    // still matching the output's own tokens (incl. slash-containing inputs).
    #[cfg(feature = "zenoh")]
    #[test]
    fn ready_prefix_isolates_nested_output_names() {
        use dora_message::id::{DataId, NodeId};
        use zenoh::key_expr::KeyExpr;

        let df = uuid::Uuid::nil();
        let source = NodeId::from("source".to_string());
        let sub_node = NodeId::from("sink".to_string());
        let cmd = DataId::from("cmd".to_string());
        let cmd_vel = DataId::from("cmd/vel".to_string());
        // A namespaced runtime-node input (`operator/input`) exercises a
        // slash-containing subscriber input on the token side.
        let sub_input = DataId::from("op/in".to_string());

        let cmd_prefix =
            KeyExpr::new(zenoh_output_ready_liveliness_prefix(df, &source, &cmd)).unwrap();
        let cmd_token = KeyExpr::new(zenoh_input_ready_liveliness_topic(
            df, &source, &cmd, &sub_node, &sub_input,
        ))
        .unwrap();
        let cmd_vel_token = KeyExpr::new(zenoh_input_ready_liveliness_topic(
            df, &source, &cmd_vel, &sub_node, &sub_input,
        ))
        .unwrap();

        assert!(
            cmd_prefix.intersects(&cmd_token),
            "producer of `cmd` must count its own subscriber's token (even with a `/`-containing input)"
        );
        assert!(
            !cmd_prefix.intersects(&cmd_vel_token),
            "producer of `cmd` must NOT count a token of the nested output `cmd/vel`"
        );
    }
}
