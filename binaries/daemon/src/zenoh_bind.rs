//! How a daemon wires its zenoh session: which endpoint it listens on, how it
//! reserves and announces that endpoint, and what it hands to spawned nodes.

use dora_core::topics::{ZenohListen, reserve_zenoh_endpoint};
use eyre::Context;
use std::net::{IpAddr, SocketAddr};

/// How a daemon wires its zenoh session: who it listens as, and who it dials.
///
/// Grouped rather than passed as four more positionals, because every field is
/// optional and they are only meaningful together — a `listen` port with no
/// peer `connect`ing to it, or the reverse, is a half-built mesh.
#[derive(Debug, Default, Clone)]
pub struct ZenohOptions {
    /// Shared rendezvous endpoint every daemon both binds and dials
    /// (`--zenoh-peer`). The first to bind it becomes the gossip hub; the rest
    /// fall through to connect-only. Discovery still goes through gossip, so
    /// the daemon-to-daemon links themselves are best-effort.
    pub inter_daemon_peer: Option<String>,
    /// Address, and optionally port, this daemon's listener binds
    /// (`--zenoh-listen`). `None` derives it from the coordinator address,
    /// which is right whenever daemons reach each other over the network they
    /// reach the coordinator over — a LAN, or a mesh VPN. An address named here
    /// must bind: unlike the derived one, failing to bind it is fatal.
    pub listen: Option<ZenohListen>,
    /// Peers this daemon dials directly (`--zenoh-connect`) — the explicit
    /// mesh. Dial-only, unlike `inter_daemon_peer`, and it establishes the
    /// clique zenoh 1.9 requires by construction rather than by gossip.
    pub connect: Vec<String>,
    /// Open sessions without multicast scouting, for this daemon and the nodes
    /// it spawns (`--zenoh-no-multicast`).
    pub disable_multicast: bool,
}

/// Where the daemon's zenoh listen address came from, which decides what a bind
/// failure means.
///
/// A derived address is a best guess, so failing to bind it may fall back to
/// multicast scouting. An address the operator *named* must bind or the daemon
/// must exit: they are on a network where they had to say the address out loud,
/// which is precisely a network where multicast is not going to rescue us — and
/// a daemon with no listener is undialable and silently dead, which is the
/// failure this whole mechanism exists to prevent.
#[derive(Debug, Clone, Copy)]
pub(crate) enum ZenohBind {
    /// Derived from the coordinator address; a bind failure can fall back.
    /// Always an ephemeral port — nobody named one, so nobody is waiting to
    /// dial a specific one either.
    Derived(IpAddr),
    /// Named via `--zenoh-listen`; a bind failure is fatal. Carries a port when
    /// the operator named one, which is what lets peers dial this daemon
    /// without discovering it first.
    Explicit(ZenohListen),
}

impl ZenohBind {
    pub(crate) fn addr(self) -> IpAddr {
        match self {
            Self::Derived(addr) => addr,
            Self::Explicit(listen) => listen.addr,
        }
    }

    pub(crate) fn is_explicit(self) -> bool {
        matches!(self, Self::Explicit(_))
    }

    /// The endpoint to request from zenoh: a named port verbatim, an ephemeral
    /// one reserved from the OS otherwise.
    fn endpoint(self) -> std::io::Result<String> {
        match self {
            Self::Derived(addr) => reserve_zenoh_endpoint(addr),
            Self::Explicit(listen) => listen.endpoint(),
        }
    }
}

/// Report whether `zenoh_bind` can be reached by remote daemons, and reject a
/// configuration that would run silently undialable.
///
/// A loopback listener advertises `127.0.0.1` to peers, who dial their own
/// loopback and reach nothing — and since zenoh 1.9 peers do not relay, that
/// pair is dead with no fallback. When the coordinator is remote (so other
/// daemons are expected):
///
/// * an *explicit* loopback address is a hard error — the operator named it, and
///   the [`ZenohBind::Explicit`] contract is "bind a routable address or exit",
///   the same "nothing to advertise" reason [`validate_zenoh_listen`] already
///   rejects the wildcard for;
/// * a *derived* loopback only warns — it is a best-effort fallback the operator
///   can override with `--zenoh-listen`.
///
/// A routable bind is announced at info level; a single-machine (loopback
/// coordinator) setup is silent.
pub(crate) fn announce_zenoh_bind(
    zenoh_bind: ZenohBind,
    coordinator_ws_addr: SocketAddr,
) -> eyre::Result<()> {
    if zenoh_bind.addr().is_loopback() && !coordinator_ws_addr.ip().is_loopback() {
        match zenoh_bind {
            ZenohBind::Explicit(listen) => {
                let addr = listen.addr;
                eyre::bail!(
                    "--zenoh-listen {listen} is a loopback address, but the coordinator \
                     at {coordinator_ws_addr} is remote, so other daemons must be able \
                     to reach this one. A loopback listener advertises {addr} to peers, \
                     who would dial their own loopback and reach nothing. Pass the \
                     address other daemons should use to reach this host (e.g. its LAN \
                     or VPN address)."
                );
            }
            ZenohBind::Derived(_) => {
                tracing::warn!(
                    "coordinator at {coordinator_ws_addr} is remote, but no routable local \
                     address toward it was found; zenoh will bind loopback and other daemons \
                     will not be able to reach this one. Pass --zenoh-listen <IP> explicitly."
                );
            }
        }
    } else if !zenoh_bind.addr().is_loopback() {
        tracing::info!(
            "zenoh listener binding {} ({}); this port accepts connections from \
             other hosts",
            zenoh_bind.addr(),
            match zenoh_bind {
                ZenohBind::Explicit(_) => "given via --zenoh-listen".to_string(),
                ZenohBind::Derived(_) =>
                    format!("derived from coordinator address {coordinator_ws_addr}"),
            }
        );
    }
    Ok(())
}

/// What `register` needs to reserve and advertise this daemon's zenoh listener.
///
/// Grouped because the three travel together through every hop between the run
/// loop and the register frame, and separately they read as three unrelated
/// booleans-and-options at each call site.
pub(crate) struct ZenohRegistration {
    /// Where the listener binds, and whether a bind failure is fatal.
    pub bind: ZenohBind,
    /// What to tell the coordinator this daemon is reachable at.
    pub advertise: AdvertiseListener,
    /// The endpoint already reserved, reused so a reconnect does not reserve a
    /// port the open session never bound. Pre-filled for an explicit bind,
    /// whose reservation happens before the connect loop.
    pub reserved: Option<String>,
}

/// Which endpoint a registration advertises to the coordinator.
///
/// Spelled out rather than left as a bool plus the reserved value, because the
/// reserved endpoint and the *bound* one diverge exactly when it matters: a
/// listener that lost its port between reservation and `zenoh::open` leaves the
/// daemon holding a reserved string nothing is listening on, and re-advertising
/// it on the next reconnect would hand that dead port to every daemon that
/// registers afterwards.
pub(crate) enum AdvertiseListener {
    /// Nothing — a loopback bind, which would point a remote daemon at its own
    /// host.
    Never,
    /// Whatever `register` reserves. The first connect, where no session exists
    /// yet and the reserved endpoint is the only candidate.
    Reserved,
    /// What the open session actually bound, verified against
    /// `info().locators()`. `None` withdraws a previously advertised endpoint.
    Bound(Option<String>),
}

/// Reserve the endpoint this daemon's zenoh listener will bind.
///
/// Split out of `build_daemon` so it can run *before* the daemon registers:
/// the endpoint is advertised in the registration, which is what lets the
/// coordinator hand it to daemons that register afterwards without a window in
/// which two of them learn nothing about each other (see
/// `DaemonRegisterRequest::zenoh_listen_endpoint`).
///
/// A reservation binds an ephemeral port and drops the socket, so there is a
/// window before zenoh's own bind in which another process could take it. That
/// window already existed; registering first widens it by one round-trip. It
/// stays tolerable because the listener is verified after `zenoh::open` — a
/// port lost in the window is caught there, and the advertised endpoint is
/// withdrawn rather than left to mislead peers.
///
/// Failing to reserve is fatal for an address the operator *named*: continuing
/// would leave the daemon with no listener at all — undialable and silently
/// dead — and someone who had to say the address out loud is, almost by
/// definition, on a network where multicast will not rescue them. A derived
/// address falls back instead: loopback always exists, so the error was
/// effectively unreachable anyway.
pub(crate) fn reserve_zenoh_listen_endpoint(zenoh_bind: ZenohBind) -> eyre::Result<Option<String>> {
    match zenoh_bind.endpoint() {
        Ok(ep) => Ok(Some(ep)),
        Err(err) if zenoh_bind.is_explicit() => Err(err).wrap_err_with(|| {
            format!(
                "failed to bind the zenoh listen address {} given via \
                 --zenoh-listen; other daemons would have no way to reach this \
                 one. Check that this address exists on this host",
                zenoh_bind.addr()
            )
        }),
        Err(err) => {
            tracing::warn!(
                "failed to reserve zenoh listen endpoint on {}: {err}; \
                 falling back to multicast scouting only",
                zenoh_bind.addr()
            );
            Ok(None)
        }
    }
}

#[cfg(test)]
mod announce_zenoh_bind_tests {
    use super::*;
    use std::net::Ipv4Addr;

    const LOOPBACK: IpAddr = IpAddr::V4(Ipv4Addr::LOCALHOST);
    const REMOTE_COORDINATOR: IpAddr = IpAddr::V4(Ipv4Addr::new(203, 0, 113, 5));
    const ROUTABLE_LOCAL: IpAddr = IpAddr::V4(Ipv4Addr::new(192, 168, 1, 20));

    fn sock(ip: IpAddr) -> SocketAddr {
        SocketAddr::new(ip, 6012)
    }

    /// An `--zenoh-listen <IP>` with no port named, which is what these cases
    /// are about: the port never changes whether the *address* is advertisable.
    fn explicit(addr: IpAddr) -> ZenohBind {
        ZenohBind::Explicit(ZenohListen { addr, port: None })
    }

    #[test]
    fn explicit_loopback_under_remote_coordinator_is_rejected() {
        // #2770: an operator who names a loopback address while the coordinator
        // is remote gets a silently-undialable daemon. The `Explicit` contract
        // is "bind a routable address or exit", so this must be a hard error.
        let err = announce_zenoh_bind(explicit(LOOPBACK), sock(REMOTE_COORDINATOR)).unwrap_err();
        let msg = err.to_string();
        assert!(msg.contains("loopback"), "unexpected error: {msg}");
        assert!(msg.contains("--zenoh-listen"), "unexpected error: {msg}");
    }

    #[test]
    fn derived_loopback_under_remote_coordinator_only_warns() {
        // A derived loopback is a best-effort fallback (no routable address was
        // found), so the daemon still starts — the operator can override it.
        announce_zenoh_bind(ZenohBind::Derived(LOOPBACK), sock(REMOTE_COORDINATOR)).unwrap();
    }

    #[test]
    fn explicit_loopback_under_local_coordinator_is_allowed() {
        // Single-machine `dora run`: coordinator and nodes share loopback, so a
        // loopback listener is both sufficient and correct.
        announce_zenoh_bind(explicit(LOOPBACK), sock(LOOPBACK)).unwrap();
    }

    #[test]
    fn explicit_routable_address_is_allowed() {
        announce_zenoh_bind(explicit(ROUTABLE_LOCAL), sock(REMOTE_COORDINATOR)).unwrap();
    }
}
