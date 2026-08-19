use std::net::{IpAddr, Ipv4Addr, SocketAddr};

pub const LOCALHOST: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_DEFAULT: u16 = 53291;
/// Env var to override the daemon's local listener port for dynamic nodes.
pub const DORA_DAEMON_LOCAL_LISTEN_PORT_ENV: &str = "DORA_DAEMON_LOCAL_LISTEN_PORT";
pub const DORA_COORDINATOR_PORT_WS_DEFAULT: u16 = 6013;

/// Comma-separated zenoh endpoints a spawned node should connect to, injected by
/// the daemon: the daemon's own listener plus the listeners of the nodes this one
/// consumes from (see [`DORA_ZENOH_LISTEN_ENV`]).
///
/// The daemon's listener is loopback for a single-machine deployment, but may be
/// a routable address when the daemon is part of a cluster (see
/// [`zenoh_bind_address_for`]). Either way it is on this node's own host, so the
/// node can always reach it.
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

/// Opt out of zenoh multicast scouting for this process, regardless of whether
/// explicit connect endpoints replaced it.
///
/// Set to `off`, `0`, `false`, or `no` to disable. Any other value (including
/// unset) leaves the default behaviour, where multicast is dropped only once
/// [`DORA_ZENOH_CONNECT_ENV`] gives the session something to dial instead.
///
/// Exists for networks where the scouting socket itself is the problem: a busy
/// DDS/ROS2 multicast graph can keep zenoh from binding its scouting group,
/// which fails `zenoh::open` outright. Disabling scouting sidesteps that bind
/// entirely — but it removes a discovery mechanism, so a session that has no
/// connect endpoints *and* no multicast can reach nobody. Set it only where
/// every link is established explicitly (the daemon injects
/// [`DORA_ZENOH_CONNECT_ENV`] into the nodes it spawns, so those are covered).
///
/// The daemon sets this on the nodes it spawns when started with
/// `--zenoh-no-multicast`, so a single flag covers the whole process tree.
pub const DORA_ZENOH_MULTICAST_ENV: &str = "DORA_ZENOH_MULTICAST";

/// Pid of the process whose death must end this node, injected **only** by a
/// daemon that runs in-process with whoever started it: `Daemon::run_dataflow`
/// and its callers — `dora run`, `dora daemon --run-dataflow`, and embedders
/// that drive one dataflow to completion.
///
/// There, the one process is coordinator, daemon and node-parent at once, so
/// its death is the end of the dataflow by definition. Every teardown path
/// dora has is cooperative and so cannot survive `SIGKILL`, which is neither
/// catchable nor blockable: no CLI- or daemon-side code runs after it. Nodes
/// are deliberately spawned as process-group leaders (so a terminal `Ctrl-C`
/// cannot kill them out from under the daemon), which also means an orphan
/// keeps running with `ppid 1` in a group of its own — unreachable by both
/// inherited signal delivery and a group-kill of the parent. Handing the node
/// the pid lets it notice on its own (dora-rs/dora#2856).
///
/// Deliberately NOT set on the `dora up` + `dora start` path: there the parent
/// is a long-lived daemon whose lifetime is decoupled from its nodes on
/// purpose — a node survives a coordinator drop, a reconnect, and a watchdog
/// disconnect while keeping its pid (dora-rs/dora#2029). Tying node lifetime
/// to that parent would break exactly the property `daemon-reconnect-e2e`
/// asserts.
pub const DORA_RUN_PARENT_PID_ENV: &str = "DORA_RUN_PARENT_PID";

/// Zenoh's own config-file override, honored by
/// [`open_zenoh_session_with_listen`].
///
/// Takes precedence over every `DORA_ZENOH_*` variable: when it is set the
/// session is built entirely from the named file, so the connect/listen plan
/// and the multicast decision are never read. That makes it a full bypass of
/// the daemon's node wiring, which is why the daemon refuses it from a
/// descriptor's `env:` (#2944) while still honoring it from its own
/// environment — the documented way to point a whole deployment at a custom
/// zenoh config.
#[cfg(feature = "zenoh")]
pub const ZENOH_CONFIG_PATH_ENV: &str = zenoh::Config::DEFAULT_CONFIG_PATH_ENV;

/// Whether a session may discover peers by multicast scouting.
///
/// Spelled as an enum rather than a bool because the concept flips polarity at
/// every hop it crosses — `--zenoh-no-multicast`, `DORA_ZENOH_MULTICAST=off`,
/// `scouting/multicast/enabled=false` — and an inverted bool would not fail a
/// test, it would silently partition the dataflow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MulticastScouting {
    /// Scout unless explicit connect endpoints replace it. The default
    /// everywhere; [`DORA_ZENOH_MULTICAST_ENV`] can still turn it off.
    #[default]
    Allowed,
    /// The caller establishes every link explicitly and wants no scouting.
    Disabled,
}

/// Whether [`DORA_ZENOH_MULTICAST_ENV`] asks for multicast scouting to be off.
#[cfg(feature = "zenoh")]
fn multicast_disabled_by_env() -> bool {
    multicast_disabled_by_value(std::env::var(DORA_ZENOH_MULTICAST_ENV).ok().as_deref())
}

/// The effective decision for a process that also has its own request.
///
/// [`open_zenoh_session_with_listen`] ORs the caller's request with
/// [`DORA_ZENOH_MULTICAST_ENV`], so a process that has to *forward* its
/// decision — the daemon, to the nodes it spawns — must OR them the same way.
/// Forwarding only its own flag drops the environment half, leaving nodes
/// scouting by multicast in exactly the environments where the variable was
/// set to stop them.
#[cfg(feature = "zenoh")]
pub fn multicast_disabled(requested_off: bool) -> bool {
    requested_off || multicast_disabled_by_env()
}

/// Parse a [`DORA_ZENOH_MULTICAST_ENV`] value (`None` when the var is unset).
///
/// Split from [`multicast_disabled_by_env`] so it is testable without mutating
/// the process environment, which is `unsafe` in edition 2024 and racy against
/// other tests in the same binary.
#[cfg(feature = "zenoh")]
fn multicast_disabled_by_value(value: Option<&str>) -> bool {
    matches!(
        value.map(|v| v.trim().to_ascii_lowercase()).as_deref(),
        Some("off" | "0" | "false" | "no")
    )
}

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
    // Nodes and the coordinator have no in-process way to know, so
    // [`DORA_ZENOH_MULTICAST_ENV`] (honored inside) is their only channel.
    let (session, _) = open_zenoh_session_with_listen(ZenohSessionParams {
        coordinator_addr,
        ..Default::default()
    })
    .await?;
    Ok(session)
}

/// How a dora process wants its zenoh session wired.
///
/// A struct rather than a parameter list because every field is optional and
/// most callers set one of them: the positional form made
/// `open_zenoh_session_with_listen(None, None, None, Allowed)` a common sight,
/// where a misplaced `None` is a silent partition rather than a type error.
/// [`Default`] gives "a plain peer with whatever the environment says", which
/// is what nodes and the coordinator want.
#[cfg(feature = "zenoh")]
#[derive(Debug, Default, Clone, Copy)]
pub struct ZenohSessionParams<'a> {
    /// Coordinator to reach through a zenoh router/peer pair. Unused by every
    /// in-tree caller today; see the `coordinator_addr` branch below.
    pub coordinator_addr: Option<IpAddr>,
    /// Endpoint this session listens on and advertises to its peers, e.g.
    /// `tcp/127.0.0.1:43217` for a single-machine daemon or `tcp/10.0.2.100:5456`
    /// for one in a cluster. Verified against `info().locators()` after open;
    /// see the return value.
    pub listen_endpoint: Option<&'a str>,
    /// Shared rendezvous endpoint for daemon-to-daemon discovery when multicast
    /// isn't available: added to *both* listen and connect endpoints, so the
    /// first daemon to bind it becomes the gossip hub and the rest fall through
    /// to connect-only.
    pub inter_daemon_peer: Option<&'a str>,
    /// Peers this session dials, in addition to whatever
    /// [`DORA_ZENOH_CONNECT_ENV`] carries. This is how a deployment wires its
    /// daemons into an explicit mesh instead of relying on gossip through a
    /// single rendezvous: every daemon dials every other one, which is the
    /// clique zenoh 1.9 requires of a peer region.
    pub connect_endpoints: &'a [String],
    /// Whether this session may scout by multicast. A request, not a command —
    /// see the `#1856` guard below.
    pub multicast: MulticastScouting,
}

/// Builds the zenoh `connect/endpoints` JSON5 for a coordinator peer.
///
/// The peer address is formatted through a [`SocketAddr`] so that IPv6
/// addresses are bracketed (`tcp/[::1]:5456`), matching zenoh's TCP locator
/// grammar. Interpolating a bare [`IpAddr`] instead would emit `tcp/::1:5456`
/// for IPv6 — a malformed locator where the port colon is indistinguishable
/// from the address colons, which `insert_json5` rejects (#3041). This is the
/// same bracketing [`reserve_zenoh_endpoint`] already relies on.
#[cfg(feature = "zenoh")]
fn coordinator_connect_endpoints(addr: IpAddr) -> String {
    let peer = SocketAddr::new(addr, 5456);
    format!(r#"{{ router: ["tcp/[::]:7447"], peer: ["tcp/{peer}"] }}"#)
}

/// Like [`open_zenoh_session`], but takes the full [`ZenohSessionParams`]: a
/// listen endpoint to bind and advertise (e.g. `tcp/127.0.0.1:43217`, or a
/// routable address such as `tcp/10.0.2.100:5456` for a daemon in a cluster),
/// extra peers to dial, and the multicast request. The daemon uses this so
/// spawned nodes can connect via `DORA_ZENOH_CONNECT` without multicast
/// scouting, and so that other daemons can dial it.
///
/// `connect_endpoints` are dialed in addition to whatever
/// [`DORA_ZENOH_CONNECT_ENV`] carries, deduplicated against it. They are
/// dial-only, which is what distinguishes an explicit mesh (each daemon
/// listens on its own endpoint and dials its peers') from the rendezvous
/// below (every daemon both listens on and dials the *same* endpoint).
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
///
/// `multicast` lets a caller that establishes every link explicitly opt out of
/// scouting — see [`DORA_ZENOH_MULTICAST_ENV`], honored in addition to this
/// argument. It is a request, not a command: it is ignored unless this session
/// ends up reachable some other way (see the `#1856` guard below).
#[cfg(feature = "zenoh")]
pub async fn open_zenoh_session_with_listen(
    params: ZenohSessionParams<'_>,
) -> eyre::Result<(zenoh::Session, Option<String>)> {
    use eyre::{Context, eyre};
    use tracing::warn;

    let ZenohSessionParams {
        coordinator_addr,
        listen_endpoint,
        inter_daemon_peer,
        connect_endpoints,
        multicast,
    } = params;

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
            // networks reaching each other through a public one). In zenoh 1.8 that
            // worked: its `linkstate_peer` hat derived
            // `peer_full_linkstate = routing.peer.mode == "linkstate"`. Zenoh 1.9
            // dropped that hat; its `peer` hat hardcodes `full_linkstate: false`
            // (release notes, under Bug fixes: "Disable `full_linkstate` in
            // `peer::Hat::Network`"), so peers no longer relay. The setting became a
            // silent no-op — `insert_json5` still returns `Ok`, so our own error
            // branch never fired, and only zenoh's deprecation log hinted at it.
            // Deleted rather than ported: there is no peer-side equivalent in 1.9.
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
            // NOTE: we used to set `transport/unicast/lowlatency: true` here (and
            // `qos/enabled: false` with it, since the low-latency transport is
            // negotiated without QoS) to skip zenoh's batching/priority queues.
            // Both are gone, because low-latency cannot fragment: a message has to
            // fit one batch, and `batch_size` is capped at 64 KiB
            // (`pub type BatchSize = u16`, so 65535 is the max, not just the
            // default). Shared memory hid that — an SHM payload travels as a
            // ~16-byte descriptor and never fragments — but SHM is per-host, so it
            // cannot negotiate between machines. A >64 KiB message to another host
            // therefore had *no* working path: the sender writes it with a 4-byte
            // length prefix and no size check, `put()` returns `Ok`, and the peer
            // rejects the frame ("Batch len is invalid") — silent loss, with the
            // publisher believing it succeeded.
            //
            // Dropping both is not a latency regression — measured against the old
            // config (release, `examples/benchmark`), p50 is neutral-to-better
            // (64 B 65->55 µs, 512 B 66->58 µs, 16 KB 73->63 µs; only 8 B is ~7 µs
            // worse) and throughput is up (4 KB +51%, 16 KB +41%), since bypassing
            // batching cost a syscall per message.
            //
            // The two settings must be removed *together*: dropping `lowlatency`
            // while leaving `qos/enabled: false` did cost ~25 µs p50 on small
            // messages. Restoring QoS recovers it, because the publishers'
            // `Priority::RealTime` finally takes effect — it was silently inert
            // while QoS was off. Publishers also still set `express(true)`, which is
            // what actually carries small-message latency here.
            //
            // We rely on zenoh's SHM transport (`transport/shared_memory/enabled`)
            // being enabled, which is its default — do NOT set it to `false`: the
            // API keeps working, but SHM buffers silently get serialized as plain
            // bytes onto the wire (i.e. copied) instead of sent as a ~16-byte
            // descriptor.

            // Build the connect-endpoint list from three sources:
            //   1. DORA_ZENOH_CONNECT env var — daemon-bootstrapped local
            //      discovery for spawned nodes (#1778).
            //   2. `connect_endpoints` — peers this process was told to dial,
            //      i.e. the explicit daemon↔daemon mesh (`--zenoh-connect`).
            //      Unlike (3) this is dial-only: a mesh member listens on its
            //      own advertised endpoint, not on its peers'.
            //   3. `inter_daemon_peer` — shared rendezvous for daemon-to-
            //      daemon discovery (extends #1778 to the daemon↔daemon
            //      hop). One daemon binds it as a listener, others connect
            //      and gossip-discover their peers via it.
            // All are explicit endpoints; if we set any of them we
            // disable multicast scouting so we don't end up with mixed
            // discovery modes.
            let mut connect_eps: Vec<String> = Vec::new();
            if let Ok(eps) = std::env::var(DORA_ZENOH_CONNECT_ENV) {
                connect_eps.extend(split_endpoints(&eps));
            }
            connect_eps.extend(connect_endpoints.iter().cloned());
            if let Some(peer) = inter_daemon_peer {
                connect_eps.push(peer.to_string());
            }
            // A duplicate dial is not fatal, but it is a wasted connection
            // attempt per duplicate and, when the peer is unreachable, a
            // second retry loop against it — which is exactly what keeps the
            // net runtime busy (#2776). Callers can legitimately overlap:
            // `--zenoh-connect` may name the same endpoint the environment
            // already carries. `retain` over a seen-set rather than `dedup`,
            // which only collapses *adjacent* equals and would leave the
            // env/param/rendezvous interleaving untouched.
            let mut seen_connect = std::collections::HashSet::new();
            connect_eps.retain(|ep| seen_connect.insert(ep.clone()));
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
            // Track whether listen/endpoints was accepted into THIS config.
            // We don't promote it to `effective_listen_endpoint` until the
            // configured open succeeds — the fallback default-config path
            // below has no listener and must not advertise one (#1856).
            // We only track the caller's own `listen_endpoint` (the per-daemon
            // listener that gets advertised to spawned nodes), NOT
            // `inter_daemon_peer` which is cluster-wide config — daemons that
            // bind it act as the rendezvous, but advertising it back to nodes
            // would be wrong (nodes would try to reach it through what may be a
            // remote address, defeating the loopback shortcut) — and not the
            // env-supplied node listeners either, which the daemon planned and
            // already knows.
            let mut listen_inserted_into_configured: Option<String> = None;
            // Any accepted listener, including the cluster-wide rendezvous that
            // is deliberately absent from `listen_inserted_into_configured`.
            // Reachability, not advertisability, is what the #1856 guard needs.
            let mut listen_configured = false;

            // Build the listen-endpoint list (loopback for spawned nodes +
            // optional inter-daemon rendezvous). With multiple entries,
            // zenoh binds whichever ones it can; `listen/exit_on_failure:
            // false` (set below when any listener is configured) lets the
            // daemon proceed even if some don't bind — e.g. the second
            // daemon to start on the same host with the same rendezvous
            // port falls through to connect-only.
            // A spawned node gets its listeners from the daemon via
            // `DORA_ZENOH_LISTEN` (the daemon itself passes `listen_endpoint`
            // directly). Without a known listener a node cannot be dialled, and
            // since zenoh 1.9 peers do not relay, a consumer that cannot dial its
            // producer never receives its data at all.
            //
            // The variable carries a *list*, because a node with a consumer on
            // another machine listens both on loopback (for its same-machine
            // consumers, whose transport can then carry shared memory) and on a
            // routable address (for the remote one).
            let env_listen = std::env::var(DORA_ZENOH_LISTEN_ENV).ok();
            let env_listen_endpoints: Vec<String> = env_listen
                .as_deref()
                .map(|value| split_endpoints(value).collect())
                .unwrap_or_default();

            let mut listen_eps: Vec<String> = Vec::new();
            if let Some(ep) = listen_endpoint {
                listen_eps.push(ep.to_string());
            }
            listen_eps.extend(env_listen_endpoints.iter().cloned());
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
                        listen_configured = true;
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

            // Drop multicast scouting only once this session is reachable some
            // other way — otherwise it has no endpoints to dial and no way to
            // be found, which is the silent partition #1856 exists to prevent.
            //
            // Two things make it reachable. `connect_inserted`: explicit
            // endpoints replaced scouting (the pre-existing rule). Or an
            // accepted listener plus a caller that asked to stop scouting —
            // `dora run` without dynamic nodes, or `--zenoh-no-multicast` on a
            // network where the scouting bind itself fails; a listener means
            // peers that hold the endpoint can dial in.
            //
            // Deliberately *not* honoring the request when neither holds: the
            // reservation-failure paths in `build_daemon` log "falling back to
            // multicast scouting only" and mean it. Treating the request as
            // absolute would disarm that recovery and strand the daemon.
            let requested_off =
                multicast_disabled(matches!(multicast, MulticastScouting::Disabled));
            if (connect_inserted || (requested_off && listen_configured))
                && let Err(err) = zenoh_config.insert_json5("scouting/multicast/enabled", "false")
            {
                warn!("failed to disable zenoh scouting/multicast: {err}");
            }

            if let Some(addr) = coordinator_addr
                && let Err(err) = zenoh_config
                    .insert_json5("connect/endpoints", &coordinator_connect_endpoints(addr))
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
                    // Verify every endpoint we asked for, but only ever
                    // *return* the caller's own: the env-supplied ones belong
                    // to a node whose daemon planned them and already knows
                    // them, so there they are a diagnostic, not a value to
                    // propagate.
                    let mut verify: Vec<&str> = listen_inserted_into_configured
                        .iter()
                        .map(String::as_str)
                        .collect();
                    if listen_configured {
                        verify.extend(env_listen_endpoints.iter().map(String::as_str));
                    }
                    if !verify.is_empty() {
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
                        // catch. NOTE: the comparison is against the
                        // requested string verbatim, so a caller must request
                        // the same canonical `tcp/<addr>:<port>` form that
                        // zenoh reports back. `reserve_zenoh_endpoint` emits
                        // that form, but only for a *concrete* address: a
                        // wildcard request (`tcp/0.0.0.0:<port>`) can never
                        // match, because zenoh binds every interface and
                        // reports the concrete one. Callers must therefore
                        // reject wildcards up front (the daemon does) rather
                        // than reach this check, which would read the mismatch
                        // as "the listener did not bind" and silently fall back
                        // to multicast scouting.
                        for requested in verify {
                            let bound = bound_locators
                                .iter()
                                .any(|l| l.split(['?', '#']).next() == Some(requested));
                            if bound {
                                if listen_inserted_into_configured.as_deref() == Some(requested) {
                                    effective_listen_endpoint = Some(requested.to_string());
                                }
                            } else if connect_inserted {
                                // We set explicit `connect/endpoints` above, so
                                // multicast scouting was disabled for this session
                                // (#1856). There is therefore NO discovery fallback:
                                // peers already told to dial `{requested}` (e.g. via
                                // the per-node `DORA_ZENOH_CONNECT` plan from #2716)
                                // cannot reach this now-listener-less session, and it
                                // cannot be scouted either — that edge is silently
                                // partitioned. Do not claim "multicast scouting only"
                                // here; that fallback does not exist in this mode and
                                // the old message pointed debuggers the wrong way
                                // (#2762).
                                warn!(
                                    "zenoh session opened but listener for `{requested}` \
                                     did not bind (actually bound: {bound_locators:?}); \
                                     multicast scouting is disabled for this session \
                                     (explicit connect endpoints are set), so peers told \
                                     to dial `{requested}` have no fallback path to reach \
                                     it (#2762)"
                                );
                            } else {
                                warn!(
                                    "zenoh session opened but listener for `{requested}` \
                                     did not bind (actually bound: {bound_locators:?}); \
                                     falling back to multicast scouting for discovery"
                                );
                            }
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

/// Default TCP port for a daemon's inter-daemon zenoh listener.
///
/// Only used when a deployment *names* the port — `--zenoh-listen <IP>` alone
/// still reserves an ephemeral one. An explicit mesh needs a port its peers can
/// predict, since they must dial the endpoint before the daemon has told anyone
/// what it bound. 5456 is the port dora already uses for a zenoh peer in
/// [`coordinator_connect_endpoints`] and in every deployment doc example.
pub const DORA_ZENOH_LISTEN_PORT_DEFAULT: u16 = 5456;

/// Format `addr:port` as a zenoh TCP endpoint string.
///
/// The address goes through [`SocketAddr`], whose `Display` brackets IPv6 —
/// which is also zenoh's locator grammar (`tcp/[::1]:7447`). Interpolating a
/// bare [`IpAddr`] instead would emit `tcp/::1:7447`, where the port colon is
/// indistinguishable from the address colons and `insert_json5` rejects the
/// result (#3041).
///
/// Unlike [`reserve_zenoh_endpoint`] this binds nothing, so there is no
/// reserve→bind window for another process to slip into: a named port is
/// either free when zenoh binds it or it is not, and the
/// `info().locators()` check in [`open_zenoh_session_with_listen`] tells the
/// caller which.
pub fn zenoh_endpoint(addr: IpAddr, port: u16) -> String {
    format!("tcp/{}", SocketAddr::new(addr, port))
}

/// Reserve an unused TCP port on `bind` for use as a zenoh listen endpoint.
/// Returns a string suitable for the zenoh `listen/endpoints` config
/// (e.g. `tcp/127.0.0.1:43217`, or `tcp/[::1]:43217` for IPv6).
///
/// The bind address matters beyond which interface accepts connections:
/// zenoh advertises the address it bound as its locator, and remote peers
/// dial exactly that. A daemon that binds loopback is therefore not merely
/// unreachable from another machine — it actively tells remote daemons to
/// dial `127.0.0.1`, i.e. their own loopback, where they find nothing (or an
/// unrelated local process). Since zenoh 1.9 peers do not relay for each
/// other, such a pair has no fallback path and is silently dead. Multi-machine
/// deployments must therefore reserve on an address the other machines can
/// actually reach; see [`reserve_loopback_zenoh_endpoint`] for the
/// single-machine case.
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
///
/// Note that the locator check in (2) compares the requested `addr:port`
/// literally, so reserving on the unspecified address (`0.0.0.0`) will not
/// match the concrete interface address zenoh reports back. Pass the address
/// you want advertised, not a wildcard.
pub fn reserve_zenoh_endpoint(bind: IpAddr) -> std::io::Result<String> {
    let listener = std::net::TcpListener::bind((bind, 0))?;
    let port = listener.local_addr()?.port();
    drop(listener);
    Ok(zenoh_endpoint(bind, port))
}

/// Loopback case of [`reserve_zenoh_endpoint`] — the right choice when every
/// zenoh peer that needs this listener is on the same host, which is what a
/// single-machine deployment looks like.
pub fn reserve_loopback_zenoh_endpoint() -> std::io::Result<String> {
    reserve_zenoh_endpoint(LOCALHOST)
}

/// Pick the address a daemon's zenoh listener should bind so that the other
/// daemons in the deployment can dial it.
///
/// Returns [`LOCALHOST`] when the coordinator is itself local: everything that
/// needs this listener is then on this host, and binding loopback keeps the
/// daemon's zenoh unreachable from the network — which is the status quo for
/// single-machine users, and worth preserving.
///
/// Otherwise the daemon is part of a multi-machine deployment, and we return
/// the local address the kernel would use to reach `coordinator_addr`. That is
/// the correct choice without anyone configuring anything: on a LAN it is the
/// LAN address, and on a mesh VPN (Tailscale/WireGuard) — where the
/// coordinator is reached over the tunnel — it is the tunnel address, which is
/// exactly the one remote daemons can dial. A multi-homed host that picks the
/// wrong interface can be overridden explicitly by the caller.
///
/// The lookup `connect()`s a UDP socket, which sends no packets: it only asks
/// the routing table which source address applies. If there is no route (or
/// the lookup otherwise fails) we fall back to [`LOCALHOST`] rather than
/// guessing, since a wrong address is advertised to peers and fails silently,
/// whereas loopback at least keeps same-host behavior working.
pub fn zenoh_bind_address_for(coordinator_addr: SocketAddr) -> IpAddr {
    if coordinator_addr.ip().is_loopback() {
        return LOCALHOST;
    }
    local_address_toward(coordinator_addr).unwrap_or(LOCALHOST)
}

/// Where a daemon's zenoh listener binds: an address, and optionally the port.
///
/// Both forms are accepted from `--zenoh-listen`:
///
/// * `10.0.2.100` — the port is left to the OS. Peers learn the resulting
///   endpoint by gossip, so this suits a deployment with a rendezvous
///   (`--zenoh-peer`) or working multicast.
/// * `10.0.2.100:5456`, `[fd7a:1::2]:5456` — a named port, which peers can dial
///   without having discovered it first. This is what an explicit mesh needs:
///   every daemon's endpoint has to be known before any of them has announced
///   anything.
///
/// IPv6 must be bracketed when naming a port, because `fd7a:1::2:5456` is
/// itself a valid IPv6 address and is read as one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZenohListen {
    /// Address to bind, which is also the address advertised to peers.
    pub addr: IpAddr,
    /// Port to bind, or `None` to let the OS pick one.
    pub port: Option<u16>,
}

impl ZenohListen {
    /// The endpoint to request, reserving an ephemeral port if none was named.
    ///
    /// A named port skips the reservation entirely: it has no reserve→bind
    /// window for another process to slip into, and zenoh's own bind is the
    /// only claim on it.
    pub fn endpoint(&self) -> std::io::Result<String> {
        match self.port {
            Some(port) => Ok(zenoh_endpoint(self.addr, port)),
            None => reserve_zenoh_endpoint(self.addr),
        }
    }
}

impl std::fmt::Display for ZenohListen {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.port {
            Some(port) => write!(f, "{}", SocketAddr::new(self.addr, port)),
            None => write!(f, "{}", self.addr),
        }
    }
}

impl std::str::FromStr for ZenohListen {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // `SocketAddr` first: `10.0.2.100:5456` is not an `IpAddr`, while a
        // bare `::1` is not a `SocketAddr`, so neither form is stolen by the
        // other. The one genuine ambiguity — unbracketed IPv6 with a port —
        // resolves to "address", which is why the docs above insist on
        // brackets.
        let listen = if let Ok(socket) = s.parse::<SocketAddr>() {
            Self {
                addr: socket.ip(),
                port: Some(socket.port()),
            }
        } else if let Ok(addr) = s.parse::<IpAddr>() {
            Self { addr, port: None }
        } else {
            return Err(format!(
                "`{s}` is neither an IP address (`10.0.2.100`) nor an address \
                 with a port (`10.0.2.100:5456`, `[fd7a:1::2]:5456`)"
            ));
        };
        if listen.port == Some(0) {
            // Port 0 binds an ephemeral port, which is fine, but the endpoint
            // we advertise would still say `:0` — no peer could dial it, and
            // the `info().locators()` check would read the mismatch as "the
            // listener did not bind". Omitting the port asks for the same thing
            // and reserves a concrete one to advertise.
            return Err(format!(
                "`{s}` names port 0; omit the port to let the OS pick one \
                 (dora then advertises the concrete port it reserved)"
            ));
        }
        validate_zenoh_listen(listen.addr).map_err(|err| format!("{err}"))?;
        Ok(listen)
    }
}

/// Reject a zenoh listen address that cannot be advertised to peers.
///
/// Only the wildcard is rejected here, and only because it is *structurally*
/// unadvertisable: zenoh would bind every interface but report a concrete
/// locator, so the listener verification in [`open_zenoh_session_with_listen`]
/// could never match it, and the daemon would silently stop advertising an
/// endpoint to its nodes. Whether a concrete address actually exists on this
/// host is not knowable here — that surfaces as a bind error from
/// [`reserve_zenoh_endpoint`], which callers must treat as fatal when the
/// operator named the address explicitly.
pub fn validate_zenoh_listen(bind: IpAddr) -> eyre::Result<()> {
    if bind.is_unspecified() {
        eyre::bail!(
            "zenoh listen address must be concrete, not the wildcard `{bind}`. \
             Zenoh advertises the address it binds and remote daemons dial exactly \
             that, so a wildcard has nothing to advertise. Pass the address other \
             daemons should use to reach this host (e.g. its LAN or VPN address)."
        );
    }
    Ok(())
}

/// Whether a source address the routing table handed back can be advertised to
/// peers as a locator.
///
/// The unspecified address is not a real source, and a loopback source for a
/// *remote* target means the routing table told us nothing usable (there is no
/// route, or the target resolved back to this host). Advertising either would
/// point peers at nothing, so both mean "we learned nothing" and the caller
/// should fall back rather than guess.
fn usable_source(local: IpAddr) -> Option<IpAddr> {
    if local.is_unspecified() || local.is_loopback() {
        return None;
    }
    Some(local)
}

fn local_address_toward(target: SocketAddr) -> Option<IpAddr> {
    // Bind the wildcard of the same family as the target, then `connect` to
    // consult the routing table. UDP `connect` is local-only: no traffic.
    let bind: SocketAddr = if target.is_ipv4() {
        (Ipv4Addr::UNSPECIFIED, 0).into()
    } else {
        (std::net::Ipv6Addr::UNSPECIFIED, 0).into()
    };
    let socket = std::net::UdpSocket::bind(bind).ok()?;
    socket.connect(target).ok()?;
    usable_source(socket.local_addr().ok()?.ip())
}

/// Zenoh key for node output data.
///
/// Payload format: raw Arrow bytes with postcard `Metadata` in the Zenoh
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

/// Hex-encode a `DataId` so it occupies exactly one zenoh key chunk.
///
/// A `DataId` may legally contain `/` (unlike a `NodeId`), so embedding one
/// verbatim as a key segment would spill into extra chunks. Hex is unambiguous
/// (`[0-9a-f]`, never `/`) and collision-free. Same helper previously used for
/// readiness liveliness keys (#2666).
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

/// Zenoh key carrying the Arrow IPC **schema** for an output's data topic.
///
/// Layout: `dora/{network}/{dataflow}/schema/{node}/{hex(output_id)}`.
///
/// This lives under a dedicated `schema/` plane — **not** under
/// [`zenoh_output_publish_topic`] — so it cannot collide with a nested DataId
/// such as `cmd/_schema` (whose data topic would otherwise share a key with
/// the schema side-channel for `cmd`), and so wildcard subscribers on the
/// data-topic namespace never see schema traffic.
///
/// The output id is [hex-encoded](hex_key_segment) into a single chunk because
/// DataIds may contain `/`. The key has no `@…` verbatim chunks: zenoh-ext
/// liveliness tokens are `${remaining:**}/@adv/${entity}/${zid}/${eid}/${meta}`
/// and Zenoh verbatim chunks are hermetic — `**` cannot cross them — so a key
/// that introduced `@schema` before `/@adv/…` failed `ke_liveliness::parse`
/// and flooded WARN logs (#2923). The producer still publishes here through a
/// zenoh-ext `AdvancedPublisher` (cache + `publisher_detection`); subscribers
/// recover via `AdvancedSubscriber` history.
#[cfg(feature = "zenoh")]
pub fn zenoh_output_schema_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    let network_id = "default";
    let output = hex_key_segment(output_id);
    format!("dora/{network_id}/{dataflow_id}/schema/{node_id}/{output}")
}

/// Zenoh key on which consumers acknowledge a producer's startup route-probe
/// markers, as a `/@ack` sub-key of [`zenoh_output_publish_topic`].
///
/// The producer declares one **exact-key** subscriber here per output and the
/// consumers of that output publish their acks to the same exact key, with the
/// acking consumer's identity in the attachment (never in the key). Exact-key
/// matching means `.../cmd/@ack` and `.../cmd/vel/@ack` can never
/// cross-deliver even though `cmd` is a chunk-prefix of `cmd/vel` — the
/// collision that forced hex-encoded wildcard keys in the earlier
/// liveliness-counting design (#2666) cannot arise without wildcards. The
/// `@`-prefixed final chunk additionally keeps the key from matching any
/// wildcard subscription on the data-topic namespace.
#[cfg(feature = "zenoh")]
pub fn zenoh_output_ack_topic(
    dataflow_id: uuid::Uuid,
    node_id: &dora_message::id::NodeId,
    output_id: &dora_message::id::DataId,
) -> String {
    format!(
        "{}/@ack",
        zenoh_output_publish_topic(dataflow_id, node_id, output_id)
    )
}

/// Zenoh key for control frames associated with a node output.
///
/// Payload format: postcard `Timestamped<InterDaemonEvent>` with no Zenoh
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

/// Zenoh topic carrying [`InterDaemonEvent::ExtensionMessage`][msg] for one
/// extension within one dataflow. Every daemon in the dataflow subscribes.
///
/// Per-namespace rather than one shared topic, so two extensions in the same
/// dataflow never see each other's traffic.
///
/// [msg]: dora_message::daemon_to_daemon::InterDaemonEvent::ExtensionMessage
pub fn dataflow_extension_topic(dataflow_id: &uuid::Uuid, namespace: &str) -> String {
    let network_id = "default";
    format!("dora/{network_id}/{dataflow_id}/ext/{namespace}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "zenoh")]
    #[test]
    fn multicast_disable_spellings_are_recognized() {
        for value in ["off", "0", "false", "no", "OFF", "False", "  off  "] {
            assert!(
                multicast_disabled_by_value(Some(value)),
                "{value:?} should disable multicast scouting"
            );
        }
    }

    #[cfg(feature = "zenoh")]
    #[test]
    fn unset_or_unrecognized_multicast_value_keeps_default() {
        // Anything that is not an explicit disable spelling must leave the
        // default behaviour: silently dropping discovery because of a typo
        // would be a partition with no error to point at (#1856).
        assert!(!multicast_disabled_by_value(None));
        for value in ["on", "1", "true", "yes", "", "maybe"] {
            assert!(
                !multicast_disabled_by_value(Some(value)),
                "{value:?} must not disable multicast scouting"
            );
        }
    }

    /// A caller's own request must survive the fold, whatever the environment
    /// says. The environment half is covered by the value tests above; this
    /// pins that [`multicast_disabled`] never *weakens* an explicit request —
    /// the daemon forwards its result to every node it spawns.
    #[cfg(feature = "zenoh")]
    #[test]
    fn an_explicit_multicast_disable_is_never_lost() {
        assert!(multicast_disabled(true));
    }

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

    // A local coordinator means every zenoh peer is on this host, so we must
    // keep binding loopback — a single-machine daemon should not start
    // listening on the network just because this code path exists.
    #[test]
    fn local_coordinator_keeps_the_listener_on_loopback() {
        for addr in ["127.0.0.1:6013", "[::1]:6013"] {
            let addr: SocketAddr = addr.parse().unwrap();
            assert_eq!(
                zenoh_bind_address_for(addr),
                LOCALHOST,
                "a loopback coordinator at {addr} must not move the listener off loopback"
            );
        }
    }

    // IPv6 endpoints must be bracketed or zenoh cannot parse the port back off
    // (`tcp/::1:7447` is ambiguous).
    #[test]
    fn ipv6_endpoints_are_bracketed() {
        match reserve_zenoh_endpoint(IpAddr::V6(std::net::Ipv6Addr::LOCALHOST)) {
            Ok(endpoint) => assert!(
                endpoint.starts_with("tcp/[::1]:"),
                "expected a bracketed IPv6 endpoint, got {endpoint}"
            ),
            // Many CI runners, Docker containers, and locked-down hosts have no
            // IPv6 address on `lo`, so binding `::1` fails with EAFNOSUPPORT
            // (`Unsupported`) or `AddrNotAvailable`. That is a property of the
            // host, not of the endpoint formatting under test, so skip rather
            // than fail — the production path never binds `::1` on such hosts.
            Err(e)
                if matches!(
                    e.kind(),
                    std::io::ErrorKind::AddrNotAvailable | std::io::ErrorKind::Unsupported
                ) || e.raw_os_error() == Some(97) => {}
            Err(e) => panic!("unexpected error reserving ::1 endpoint: {e}"),
        }
    }

    // The coordinator peer endpoint must bracket IPv6 too, or `insert_json5`
    // rejects the malformed locator and the peer connect-endpoint is silently
    // dropped (#3041). Pure string formatting — no session is opened.
    #[cfg(feature = "zenoh")]
    #[test]
    fn coordinator_connect_endpoints_bracket_ipv6() {
        let v6 = coordinator_connect_endpoints(IpAddr::V6(std::net::Ipv6Addr::LOCALHOST));
        assert!(
            v6.contains(r#"peer: ["tcp/[::1]:5456"]"#),
            "IPv6 coordinator peer must be bracketed, got {v6}"
        );
        assert!(
            !v6.contains("tcp/::1:5456"),
            "unbracketed IPv6 peer locator is malformed, got {v6}"
        );

        let v4 = coordinator_connect_endpoints(IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1)));
        assert!(
            v4.contains(r#"peer: ["tcp/127.0.0.1:5456"]"#),
            "IPv4 coordinator peer must be unbracketed, got {v4}"
        );
    }

    // The filter behind the routing lookup, tested directly rather than through
    // the runner's routing table (which would make the assertion depend on the
    // machine and, for a remote target, be satisfiable by either branch).
    #[test]
    fn only_concrete_routable_sources_are_advertisable() {
        // Real interface addresses are what we want to advertise.
        for ok in ["10.0.2.100", "192.168.1.7", "100.64.0.3"] {
            let addr: IpAddr = ok.parse().unwrap();
            assert_eq!(
                usable_source(addr),
                Some(addr),
                "{ok} is a concrete routable address and must be advertisable"
            );
        }
        // A wildcard is not a source at all, and a loopback source for a remote
        // target means the routing table told us nothing — advertising either
        // points peers at nothing.
        for rejected in ["0.0.0.0", "127.0.0.1", "::", "::1"] {
            let addr: IpAddr = rejected.parse().unwrap();
            assert_eq!(
                usable_source(addr),
                None,
                "{rejected} must never be advertised to peers as a locator"
            );
        }
    }

    // The wildcard cannot be advertised: zenoh binds every interface but reports
    // a concrete locator, so the listener check can never match it and the
    // daemon would silently stop advertising an endpoint to its nodes —
    // reintroducing the gossip race #2716 removed. Reject it up front instead.
    #[test]
    fn wildcard_zenoh_listen_is_rejected() {
        for wildcard in ["0.0.0.0", "::"] {
            let addr: IpAddr = wildcard.parse().unwrap();
            let err = validate_zenoh_listen(addr)
                .expect_err("wildcard listen address must be rejected, not accepted");
            assert!(
                err.to_string().contains("concrete"),
                "error should tell the operator to pass a concrete address, got: {err}"
            );
        }
    }

    // An IPv6 endpoint must bracket its address, or the port colon is
    // indistinguishable from the address colons and `insert_json5` rejects the
    // locator outright (#3041). A named-port endpoint has to match the shape
    // `reserve_zenoh_endpoint` produces, because both end up in the same
    // `listen/endpoints` list and are compared verbatim against
    // `info().locators()` after open.
    #[test]
    fn zenoh_endpoint_brackets_ipv6_and_matches_the_reserved_shape() {
        assert_eq!(
            zenoh_endpoint("10.0.2.100".parse().unwrap(), 5456),
            "tcp/10.0.2.100:5456"
        );
        assert_eq!(
            zenoh_endpoint("::1".parse().unwrap(), 5456),
            "tcp/[::1]:5456"
        );

        let reserved = reserve_zenoh_endpoint(LOCALHOST).expect("loopback reservation");
        let port = reserved
            .rsplit_once(':')
            .expect("reserved endpoint carries a port")
            .1
            .parse()
            .expect("reserved port is numeric");
        assert_eq!(zenoh_endpoint(LOCALHOST, port), reserved);
    }

    // `--zenoh-listen` takes both forms, and which one was given decides
    // whether peers can dial this daemon before it has announced anything.
    #[test]
    fn zenoh_listen_parses_address_with_and_without_port() {
        let addr_only: ZenohListen = "10.0.2.100".parse().unwrap();
        assert_eq!(addr_only.addr, "10.0.2.100".parse::<IpAddr>().unwrap());
        assert_eq!(addr_only.port, None);

        let with_port: ZenohListen = "10.0.2.100:5456".parse().unwrap();
        assert_eq!(with_port.port, Some(5456));
        assert_eq!(with_port.endpoint().unwrap(), "tcp/10.0.2.100:5456");

        // IPv6 needs brackets to carry a port; unbracketed, the trailing group
        // is part of the address. Both parse — they just mean different things,
        // which is why the flag docs insist on brackets.
        let v6_port: ZenohListen = "[fd7a:1::2]:5456".parse().unwrap();
        assert_eq!(v6_port.port, Some(5456));
        assert_eq!(v6_port.endpoint().unwrap(), "tcp/[fd7a:1::2]:5456");
        let v6_bare: ZenohListen = "fd7a:1::2:5456".parse().unwrap();
        assert_eq!(v6_bare.port, None);
    }

    // A wildcard has nothing to advertise, and port 0 would advertise `:0`.
    // Both are rejected at parse time so the operator hears about it at the
    // command line rather than as a silent partition later.
    #[test]
    fn zenoh_listen_rejects_unadvertisable_forms() {
        for bad in ["0.0.0.0", "0.0.0.0:5456", "::"] {
            let err = bad
                .parse::<ZenohListen>()
                .expect_err("wildcard must be rejected");
            assert!(
                err.contains("concrete"),
                "unexpected error for {bad}: {err}"
            );
        }
        let err = "10.0.2.100:0"
            .parse::<ZenohListen>()
            .expect_err("port 0 must be rejected");
        assert!(err.contains("port 0"), "unexpected error: {err}");

        let err = "not-an-address"
            .parse::<ZenohListen>()
            .expect_err("garbage must be rejected");
        assert!(err.contains("neither"), "unexpected error: {err}");
    }

    // Concrete addresses pass validation whether or not they exist on this host:
    // existence is not knowable here and surfaces as a bind error instead.
    #[test]
    fn concrete_zenoh_listen_addresses_are_accepted() {
        for ok in ["127.0.0.1", "10.0.2.100", "::1"] {
            let addr: IpAddr = ok.parse().unwrap();
            assert!(
                validate_zenoh_listen(addr).is_ok(),
                "{ok} is concrete and must pass validation"
            );
        }
    }

    // Node raw output and daemon control frames MUST live on distinct Zenoh
    // keys: they share neither format nor consumer, and merging them caused the
    // #1992 crossover (daemon postcard-decoding node output). Guard the split.
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

    // Data, schema, ack, and control keys for the same (node, output) must all
    // be distinct: each carries a different payload format for a different
    // consumer, and any overlap would cross-deliver frames to a decoder that
    // cannot parse them.
    #[cfg(feature = "zenoh")]
    #[test]
    fn per_output_topics_are_distinct() {
        use dora_message::id::{DataId, NodeId};

        let dataflow_id = uuid::Uuid::nil();
        let node = NodeId::from("node".to_string());
        let output = DataId::from("out".to_string());

        let topics = [
            zenoh_output_publish_topic(dataflow_id, &node, &output),
            zenoh_output_schema_topic(dataflow_id, &node, &output),
            zenoh_output_ack_topic(dataflow_id, &node, &output),
            zenoh_daemon_control_topic(dataflow_id, &node, &output),
        ];
        for (i, a) in topics.iter().enumerate() {
            for b in &topics[i + 1..] {
                assert_ne!(a, b, "per-output zenoh keys must not overlap");
            }
        }
    }

    // zenoh-ext publisher-detection liveliness uses
    // `${remaining:**}/@adv/...`. Verbatim (`@…`) chunks are hermetic, so a
    // schema key that itself introduced `/@schema` before `/@adv/` made tokens
    // unparseable and flooded WARN logs (#2923). Keep the schema key free of
    // `@` chunks (AdvancedPublisher + publisher_detection still used).
    #[cfg(feature = "zenoh")]
    #[test]
    fn schema_topic_has_no_verbatim_chunks() {
        use dora_message::id::{DataId, NodeId};

        let dataflow_id = uuid::Uuid::nil();
        let node = NodeId::from("node".to_string());
        let output = DataId::from("out".to_string());
        let topic = zenoh_output_schema_topic(dataflow_id, &node, &output);

        assert!(
            topic.contains("/schema/"),
            "schema side-channel must live under the dedicated `/schema/` plane, got {topic}"
        );
        assert!(
            !topic.contains("/output/"),
            "schema side-channel must not nest under the data-topic `/output/` path, got {topic}"
        );
        for chunk in topic.split('/') {
            assert!(
                !chunk.starts_with('@'),
                "schema topic chunk `{chunk}` must not be verbatim (`@…`); \
                 otherwise zenoh_ext liveliness tokens fail to parse (#2923)"
            );
        }
    }

    // `/_schema` nested under the output path collided with a valid DataId
    // `cmd/_schema`. The schema plane must stay outside `output/{node}/{data_id}`.
    #[cfg(feature = "zenoh")]
    #[test]
    fn schema_topic_does_not_collide_with_nested_data_id() {
        use dora_message::id::{DataId, NodeId};

        let dataflow_id = uuid::Uuid::nil();
        let node = NodeId::from("node".to_string());
        let parent = DataId::from("cmd");
        let nested = DataId::from("cmd/_schema");
        assert_ne!(
            zenoh_output_schema_topic(dataflow_id, &node, &parent),
            zenoh_output_publish_topic(dataflow_id, &node, &nested),
        );
    }

    // The ack design relies on exact-key matching instead of wildcards, so an
    // output id that is a chunk-prefix of another (`cmd` vs `cmd/vel` — the
    // collision that forced hex-encoded keys in the #2666 liveliness design)
    // must yield distinct ack keys with no subsumption possible.
    #[cfg(feature = "zenoh")]
    #[test]
    fn ack_topics_of_prefix_outputs_are_distinct() {
        use dora_message::id::{DataId, NodeId};

        let dataflow_id = uuid::Uuid::nil();
        let node = NodeId::from("node".to_string());
        let cmd = DataId::from("cmd".to_string());
        let cmd_vel = DataId::from("cmd/vel".to_string());

        let cmd_ack = zenoh_output_ack_topic(dataflow_id, &node, &cmd);
        let cmd_vel_ack = zenoh_output_ack_topic(dataflow_id, &node, &cmd_vel);

        assert_ne!(cmd_ack, cmd_vel_ack);
        // `cmd`'s ack key ends in `cmd/@ack`; the nested output's key contains
        // `cmd/vel/@ack`. Neither is a prefix of the other, so exact-key
        // subscribers can never receive the other output's acks.
        assert!(cmd_ack.ends_with("/cmd/@ack"));
        assert!(cmd_vel_ack.ends_with("/cmd/vel/@ack"));
        assert!(!cmd_vel_ack.starts_with(&cmd_ack));
        assert!(!cmd_ack.starts_with(&cmd_vel_ack));
    }
}
