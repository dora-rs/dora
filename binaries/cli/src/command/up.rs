use super::system::status::daemon_running;
use super::{Executable, default_tracing};
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, connect_with_retry, send_control_request},
};
use dora_core::topics::DORA_COORDINATOR_PORT_WS_DEFAULT;
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::{ControlRequestReply, DataflowIdAndName},
};
use eyre::{Context, ContextCompat, bail};
use std::io::{Read, Seek};
use std::path::PathBuf;
use std::process::Stdio;
use std::{fs, net::SocketAddr, path::Path, process::Command, time::Duration};

#[derive(Debug, clap::Args)]
/// Spawn coordinator and daemon in local mode (with default config)
pub struct Up {
    /// Use a custom configuration
    #[clap(long, hide = true, value_name = "PATH", value_hint = clap::ValueHint::FilePath)]
    config: Option<PathBuf>,
    /// Enable token authentication for the coordinator.
    ///
    /// When enabled, the coordinator generates a random token on startup
    /// and writes it to ~/.config/dora/.dora-token. Clients must present
    /// this token to connect.
    #[clap(long)]
    auth: bool,
    /// Network interface the coordinator binds to.
    ///
    /// Defaults to loopback, which keeps a development coordinator off the
    /// network. Pass a routable address (or `0.0.0.0`) when daemons on other
    /// machines have to reach it — a loopback coordinator refuses their
    /// connections, and the daemon only reports a connect timeout.
    ///
    /// This is the *bind* address. `DORA_COORDINATOR_ADDR` stays the address
    /// this and every other lifecycle command connects to.
    #[clap(long, value_name = "IP", env = "DORA_COORDINATOR_INTERFACE")]
    interface: Option<std::net::IpAddr>,
    /// Archive the default coordinator store before starting a fresh coordinator.
    ///
    /// Use this after an upgrade when the on-disk schema is incompatible.
    /// The previous store is preserved next to the new one as a `.backup` file.
    #[clap(long)]
    recreate_store: bool,
}

impl Executable for Up {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        up(
            self.config.as_deref(),
            self.auth,
            self.recreate_store,
            self.interface,
        )
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct UpConfig {}

pub(crate) fn up(
    config_path: Option<&Path>,
    auth: bool,
    recreate_store: bool,
    interface: Option<std::net::IpAddr>,
) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
    // Surface a malformed value instead of silently falling back to the
    // default: every other lifecycle command reads these env vars through
    // clap's typed `env=` on `CoordinatorOptions`, which errors on an
    // unparseable value. If `dora up` quietly ignored a typo it would bind a
    // different port than `dora stop`/`list`/`down` then look for.
    // A coordinator bound to a concrete non-loopback interface does *not*
    // accept connections on loopback, so that interface is also the address
    // this command — and the readiness poll below, which would otherwise time
    // out and kill the coordinator it just started — has to connect to. The
    // wildcard is the exception: it covers loopback too, so the default stands.
    //
    // An explicit `DORA_COORDINATOR_ADDR` still wins: it is how a user points
    // these commands at a coordinator reachable under a different address than
    // the one it binds (a NAT, a container port mapping).
    let env_addr: Option<std::net::IpAddr> = coordinator_env_opt("DORA_COORDINATOR_ADDR")?;
    let addr = connect_addr_for(env_addr, interface)?;
    let port: u16 =
        coordinator_env_value("DORA_COORDINATOR_PORT", DORA_COORDINATOR_PORT_WS_DEFAULT)?;
    let coordinator_addr = (addr, port).into();
    // The port is passed explicitly so the spawned coordinator binds the same
    // one every other lifecycle command connects to; without it a
    // `DORA_COORDINATOR_PORT` set only in this process's environment would be
    // read by the child too, but a `--port` on the command line would not.
    let spawn = CoordinatorSpawn {
        interface,
        port: Some(port),
        auth,
    };
    // Keyed off the *explicit* env var rather than `addr`: a `--interface`
    // pointing at one of this machine's own addresses still means a local
    // store, and refusing it would be wrong. Only a user-supplied address can
    // mean "some other machine's coordinator".
    if recreate_store
        && let Some(env_addr) = env_addr
        && !env_addr.is_loopback()
    {
        bail!(
            "--recreate-store only applies to the local default coordinator store, \
             but DORA_COORDINATOR_ADDR is set to the non-loopback address {env_addr}\n\n  \
             hint: unset DORA_COORDINATOR_ADDR, or run this command on the machine \
             that hosts the coordinator store"
        );
    }
    // Keep the lock through daemon readiness so overlapping recreation commands
    // cannot race either the fresh coordinator or its daemon startup.
    let mut _recreation_lock = None;
    let session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => attach_to_running_coordinator(session, coordinator_addr, recreate_store),
        Err(_) if recreate_store => {
            _recreation_lock = Some(lock_default_coordinator_store_recreation()?);
            match connect_to_coordinator(coordinator_addr) {
                Ok(session) => {
                    attach_to_running_coordinator(session, coordinator_addr, recreate_store)
                }
                Err(err) => {
                    ensure_no_listener_before_archive(coordinator_addr, err)?;
                    archive_default_coordinator_store()?;
                    start_and_wait_for_coordinator(coordinator_addr, port, spawn)?
                }
            }
        }
        Err(_) => {
            ensure_no_coordinator_elsewhere_on_port(coordinator_addr, interface)?;
            start_and_wait_for_coordinator(coordinator_addr, port, spawn)?
        }
    };

    if !daemon_running(&session)? {
        // A wildcard bind has no single address to hand the daemon, so it stays
        // on loopback: its nodes are then reachable only through the
        // daemon-forwarded path from other machines. Naming a concrete
        // interface instead gives remote daemons something to dial.
        // `addr` — not `interface` — because that is the address this daemon
        // must actually reach the coordinator at, and the one it derives its
        // zenoh bind from.
        let daemon_coordinator_addr =
            Some(addr).filter(|i| !i.is_loopback() && !i.is_unspecified());
        if interface.is_some_and(|i| i.is_unspecified()) {
            println!(
                "note: the local daemon's zenoh listener stays on loopback with a wildcard \
                 --interface; pass this machine's address (e.g. --interface 192.168.1.10) \
                 so daemons on other machines can dial its nodes directly"
            );
        }
        start_daemon(daemon_coordinator_addr, port).wrap_err("failed to start dora-daemon")?;

        // wait a bit until daemon is connected
        let mut i = 0;
        const WAIT_S: f32 = 0.1;
        loop {
            if daemon_running(&session)? {
                break;
            }
            i += 1;
            if i > 20 {
                eyre::bail!("daemon not connected after {}s", WAIT_S * i as f32);
            }
            std::thread::sleep(Duration::from_secs_f32(WAIT_S));
        }
    } else {
        println!("daemon already running");
    }

    Ok(())
}

/// Resolve a coordinator connection env var: unset yields `default`, but a
/// value that is present yet unparseable is a hard error rather than a silent
/// fallback. This mirrors the strictness of clap's typed `env=` on
/// `CoordinatorOptions`, so `dora up` and the other lifecycle commands agree
/// on where the coordinator lives.
fn coordinator_env_value<T>(var_name: &str, default: T) -> eyre::Result<T>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    parse_coordinator_env(var_name, std::env::var(var_name).ok().as_deref(), default)
}

/// The address `dora up` — and its readiness poll — connects to.
///
/// `--interface` is the *bind* address; this is the one to reach it on. They
/// are not interchangeable: a coordinator bound to a concrete non-loopback
/// address does not answer on loopback, so polling loopback would time out and
/// kill the coordinator we just started, reporting "is port N already in use?"
/// — which names the wrong cause entirely.
///
/// A wildcard bind covers loopback, so the default stands there. An explicit
/// `DORA_COORDINATOR_ADDR` wins, since it is how a user points these commands
/// at a coordinator reachable under a different address than the one it binds
/// (a NAT, a container port mapping) — except when it contradicts a concrete
/// `--interface`, which is a mistake worth naming rather than silently
/// resolving into a startup timeout.
fn connect_addr_for(
    env_addr: Option<std::net::IpAddr>,
    interface: Option<std::net::IpAddr>,
) -> eyre::Result<std::net::IpAddr> {
    let bound_concretely = interface.filter(|i| !i.is_unspecified());
    match (env_addr, bound_concretely) {
        (Some(env), Some(iface)) if env != iface => bail!(
            "DORA_COORDINATOR_ADDR is {env} but --interface is {iface}, so the \
             coordinator would bind {iface} and answer nowhere else while every \
             command looked for it on {env}\n\n  \
             hint: drop one of them, or set DORA_COORDINATOR_ADDR={iface}"
        ),
        (Some(env), _) => Ok(env),
        (None, Some(iface)) => Ok(iface),
        (None, None) => Ok(LOCALHOST),
    }
}

/// Like [`coordinator_env_value`], but reports whether the variable was set at
/// all, for callers whose default depends on other flags. A present-but-
/// unparseable value is still a hard error.
fn coordinator_env_opt<T>(var_name: &str) -> eyre::Result<Option<T>>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    parse_coordinator_env_opt(var_name, std::env::var(var_name).ok().as_deref())
}

/// Testable core of [`coordinator_env_value`], split out so tests can supply a
/// raw value without touching the process-global environment.
fn parse_coordinator_env<T>(var_name: &str, raw: Option<&str>, default: T) -> eyre::Result<T>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    Ok(parse_coordinator_env_opt(var_name, raw)?.unwrap_or(default))
}

/// Parsing core shared by both accessors: an absent variable is `None`, a
/// present but unparseable one is an error.
fn parse_coordinator_env_opt<T>(var_name: &str, raw: Option<&str>) -> eyre::Result<Option<T>>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    match raw {
        Some(s) => s
            .parse()
            .map(Some)
            .map_err(|e| eyre::eyre!("invalid {var_name}: {s:?} ({e})")),
        None => Ok(None),
    }
}

fn attach_to_running_coordinator(
    session: crate::ws_client::WsSession,
    coordinator_addr: SocketAddr,
    recreate_store: bool,
) -> crate::ws_client::WsSession {
    println!("coordinator already running on {coordinator_addr}");
    if recreate_store {
        println!(
            "--recreate-store skipped: the running coordinator holds the store; \
             run `dora down` first, then rerun `dora up --recreate-store`"
        );
    }
    session
}

/// A failed WebSocket connect only proves the coordinator is absent when
/// nothing is listening on the port at all. If a process is listening but the
/// handshake failed (stale auth token, protocol mismatch), archiving would
/// rename the store out from under a live coordinator, silently diverting all
/// further writes into the backup file.
fn ensure_no_listener_before_archive(
    coordinator_addr: SocketAddr,
    connect_err: eyre::Report,
) -> eyre::Result<()> {
    // Probe loopback as well as the address we tried to reach. The store being
    // archived is this machine's *default* one, and the coordinator holding it
    // may well be bound somewhere we were not looking: a plain `dora up` binds
    // loopback, so a later `dora up --recreate-store --interface <routable IP>`
    // would find nothing at the routable address and rename the redb file out
    // from under that live coordinator. Any listener on this port locally is
    // reason enough to refuse.
    let loopback = SocketAddr::from((LOCALHOST, coordinator_addr.port()));
    // Deduplicated: on the default path both entries are loopback, and probing
    // it twice doubles the wait wherever the port is DROP-filtered rather than
    // refused.
    let mut probes = vec![coordinator_addr];
    if coordinator_addr != loopback {
        probes.push(loopback);
    }
    for addr in probes {
        if std::net::TcpStream::connect_timeout(&addr, Duration::from_secs(1)).is_ok() {
            return Err(connect_err.wrap_err(format!(
                "refusing to archive the coordinator store: a process is listening on \
                 {addr} but the connection failed\n\n  \
                 hint: resolve the connection error below (e.g. an auth token mismatch), \
                 or stop the coordinator with `dora down` before recreating the store"
            )));
        }
    }
    Ok(())
}

/// Refuse to start a second coordinator when one already holds this port at a
/// different local address.
///
/// `--interface` moved the address we probe, so a coordinator started by a
/// plain `dora up` (loopback) is invisible to the probe above. Spawning anyway
/// produces a child that dies on the redb lock and surfaces as "coordinator
/// exited before it became ready", which says nothing about the real cause.
///
/// Only the loopback/derived pair is checked, which is what distinguishes the
/// two `dora up` invocations from each other; a coordinator on some third
/// interface still falls through to the port-in-use error.
fn ensure_no_coordinator_elsewhere_on_port(
    coordinator_addr: SocketAddr,
    interface: Option<std::net::IpAddr>,
) -> eyre::Result<()> {
    // Only when `--interface` is what moved us off loopback. A plain
    // `DORA_COORDINATOR_ADDR` pointing elsewhere is a deliberate "reach the
    // coordinator over there", and the hint below ("drop --interface") would
    // name a flag the user never passed.
    if interface.is_none() {
        return Ok(());
    }
    let loopback = SocketAddr::from((LOCALHOST, coordinator_addr.port()));
    if coordinator_addr == loopback {
        return Ok(());
    }
    if std::net::TcpStream::connect_timeout(&loopback, Duration::from_secs(1)).is_ok() {
        bail!(
            "a coordinator is already running on port {} bound to loopback, so it \
             cannot be reached at {coordinator_addr}\n\n  \
             hint: stop it with `dora down` and re-run, or drop --interface to \
             attach to the loopback coordinator",
            coordinator_addr.port()
        );
    }
    Ok(())
}

pub(crate) struct CoordinatorStartup {
    child: std::process::Child,
    /// `None` when the capture file could not be created; startup proceeds
    /// without stderr in error reports rather than failing.
    stderr: Option<fs::File>,
}

fn start_and_wait_for_coordinator(
    coordinator_addr: SocketAddr,
    port: u16,
    spawn: CoordinatorSpawn,
) -> eyre::Result<crate::ws_client::WsSession> {
    let startup = spawn_coordinator(spawn).wrap_err("failed to start dora-coordinator")?;
    wait_for_coordinator_start(coordinator_addr, port, startup)
}

pub(crate) fn wait_for_coordinator_start(
    coordinator_addr: SocketAddr,
    port: u16,
    mut startup: CoordinatorStartup,
) -> eyre::Result<crate::ws_client::WsSession> {
    let deadline = std::time::Instant::now() + Duration::from_secs(10);
    loop {
        if let Some(status) = startup
            .child
            .try_wait()
            .wrap_err("failed to poll dora-coordinator")?
        {
            // A concurrent `dora up` may have won the port race, in which case
            // our child exits with a bind error while a healthy coordinator is
            // accepting connections. Attach to the winner instead of failing,
            // keeping `dora up` idempotent under concurrent invocation.
            if let Ok(session) = connect_to_coordinator(coordinator_addr) {
                println!("coordinator already running on {coordinator_addr}");
                return Ok(session);
            }
            // Degrades to an empty capture on read failure: the exit status is
            // the primary diagnostic and must not be lost to capture plumbing.
            let stderr = captured_stderr(startup.stderr.as_mut());
            let mut message = format!("coordinator exited before it became ready: {status}");
            if !stderr.is_empty() {
                message.push('\n');
                message.push_str(&stderr);
            }
            if stderr.contains(dora_coordinator::dora_coordinator_store::SCHEMA_MISMATCH_MARKER) {
                message.push_str(
                    "\n\n  hint: run `dora up --recreate-store` to archive the incompatible \
                     store and create a fresh one",
                );
            }
            bail!(message);
        }

        match connect_to_coordinator(coordinator_addr) {
            Ok(session) => return Ok(session),
            Err(_) if std::time::Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(err) => {
                // Don't leave a detached child racing the failure we are about
                // to report: it could bind the port seconds after the user was
                // told startup failed (and, with --recreate-store, after the
                // recreation lock has been released).
                let _ = startup.child.kill();
                let _ = startup.child.wait();
                bail!(
                    "timed out waiting for coordinator to start at {coordinator_addr}: {err}\n\n  \
                     hint: is port {port} already in use? Check with `dora status`\n  \
                     or stop the existing coordinator with `dora down`"
                );
            }
        }
    }
}

/// Best-effort read of the coordinator stderr capture; empty on any failure.
fn captured_stderr(file: Option<&mut fs::File>) -> String {
    let Some(file) = file else {
        return String::new();
    };
    let mut bytes = Vec::new();
    if file.rewind().is_err() || file.read_to_end(&mut bytes).is_err() {
        return String::new();
    }
    String::from_utf8_lossy(&bytes).trim().to_owned()
}

/// Open `~/.dora/coordinator-stderr.log` (truncated on each start) to capture
/// the spawned coordinator's stderr. A named file keeps post-startup panics
/// inspectable — an unlinked tempfile would be held invisibly by the detached
/// coordinator for its entire lifetime. Returns `None` (capture disabled)
/// instead of failing startup when the file cannot be created.
fn open_coordinator_stderr_capture() -> Option<fs::File> {
    let dir = super::coordinator::dora_dir();
    fs::create_dir_all(&dir).ok()?;
    fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(true)
        .open(dir.join("coordinator-stderr.log"))
        .ok()
}

#[cfg(not(feature = "redb-backend"))]
const RECREATE_STORE_REQUIRES_REDB: &str = "--recreate-store requires the `redb-backend` feature";

fn archive_default_coordinator_store() -> eyre::Result<()> {
    #[cfg(feature = "redb-backend")]
    {
        let path = super::coordinator::default_redb_path()?;
        archive_coordinator_store(&path)
    }
    #[cfg(not(feature = "redb-backend"))]
    {
        bail!(RECREATE_STORE_REQUIRES_REDB)
    }
}

#[cfg(feature = "redb-backend")]
fn archive_coordinator_store(path: &Path) -> eyre::Result<()> {
    use fs2::FileExt as _;

    if !path.exists() {
        println!(
            "no coordinator store at `{}`; nothing to archive",
            path.display()
        );
        return Ok(());
    }

    // redb holds an exclusive OS lock on every open database. Trying that
    // lock checks the actual invariant — "no live process has this store
    // open" — which the TCP probe cannot: a coordinator on a non-default
    // port, or one still in its pre-listen window, holds the file without
    // answering the default address. Hold the lock across the rename so no
    // opener can slip in between check and archive.
    let store_file = fs::File::open(path)
        .with_context(|| format!("failed to open coordinator store `{}`", path.display()))?;
    if store_file.try_lock_exclusive().is_err() {
        bail!(
            "refusing to archive coordinator store `{}`: another process still \
             has it open\n\n  \
             hint: stop the coordinator with `dora down`, then rerun \
             `dora up --recreate-store`",
            path.display()
        );
    }

    let backup = available_backup_path(path)?;
    fs::rename(path, &backup).with_context(|| {
        format!(
            "failed to archive coordinator store `{}` as `{}`",
            path.display(),
            backup.display()
        )
    })?;
    println!(
        "archived coordinator store `{}` as `{}`",
        path.display(),
        backup.display()
    );
    Ok(())
}

fn lock_default_coordinator_store_recreation() -> eyre::Result<fs::File> {
    #[cfg(feature = "redb-backend")]
    {
        let path = super::coordinator::default_redb_path()?;
        lock_coordinator_store_recreation(&path)
    }
    #[cfg(not(feature = "redb-backend"))]
    {
        bail!(RECREATE_STORE_REQUIRES_REDB)
    }
}

#[cfg(feature = "redb-backend")]
fn lock_coordinator_store_recreation(path: &Path) -> eyre::Result<fs::File> {
    use fs2::FileExt as _;

    let file_name = path
        .file_name()
        .context("coordinator store path has no file name")?
        .to_string_lossy();
    let lock_path = path.with_file_name(format!("{file_name}.recreate.lock"));
    let lock = fs::OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(&lock_path)
        .with_context(|| {
            format!(
                "failed to open coordinator store recreation lock `{}`",
                lock_path.display()
            )
        })?;
    // Announce contention before blocking: the holder keeps the lock through
    // coordinator + daemon startup, so the wait is seconds in the normal case
    // and would otherwise look like a silent hang.
    if lock.try_lock_exclusive().is_err() {
        println!("waiting for another `dora up --recreate-store` to finish...");
        lock.lock_exclusive().with_context(|| {
            format!(
                "failed to lock coordinator store recreation at `{}`",
                lock_path.display()
            )
        })?;
    }
    Ok(lock)
}

#[cfg(feature = "redb-backend")]
fn available_backup_path(path: &Path) -> eyre::Result<PathBuf> {
    let file_name = path
        .file_name()
        .context("coordinator store path has no file name")?
        .to_string_lossy();
    let parent = path
        .parent()
        .context("coordinator store path has no parent directory")?;

    for suffix in 0..u32::MAX {
        let backup_name = if suffix == 0 {
            format!("{file_name}.backup")
        } else {
            format!("{file_name}.backup.{suffix}")
        };
        let candidate = parent.join(backup_name);
        if !candidate.exists() {
            return Ok(candidate);
        }
    }
    bail!("could not find an available coordinator store backup path")
}

/// Whether `dora down` may destroy the coordinator it just connected to.
///
/// Separate from the command so the rule is testable: the CLI reaches
/// whatever coordinator owns the port, which may belong to an unrelated
/// project on the same machine (dora-rs/dora#2924).
// No `PartialEq`: it would require widening `DataflowIdAndName` in
// dora-message purely for test convenience. Tests destructure instead.
#[derive(Debug)]
pub(crate) enum DownDecision {
    Proceed,
    /// Running dataflows would be killed; the operator has not said so.
    Refuse(Vec<DataflowIdAndName>),
}

pub(crate) fn down_decision(running: Vec<DataflowIdAndName>, force: bool) -> DownDecision {
    if running.is_empty() || force {
        DownDecision::Proceed
    } else {
        DownDecision::Refuse(running)
    }
}

pub(crate) fn down(
    config_path: Option<&Path>,
    coordinator_addr: SocketAddr,
    force: bool,
) -> Result<(), eyre::ErrReport> {
    let UpConfig {} = parse_dora_config(config_path)?;
    // Retry connection briefly — the coordinator may still be initializing after `dora up`.
    let session = connect_with_retry(coordinator_addr, Duration::from_secs(5)).map_err(|_| {
        eyre::eyre!(
            "could not connect to coordinator at {coordinator_addr}\n\n  \
             hint: is it running? Start it with `dora up`"
        )
    })?;

    // Look before destroying. Every lifecycle command targets whatever
    // coordinator owns the port, so this may be an unrelated project's
    // instance on the same machine (#2924).
    let running = match send_control_request(&session, &ControlRequest::List) {
        Ok(ControlRequestReply::DataflowList(list)) => list.get_active(),
        // An unexpected reply or a transport error must not make `down`
        // unusable: report what we could not determine and carry on, which
        // is the pre-#2924 behavior.
        Ok(other) => {
            eprintln!(
                "warning: could not determine running dataflows before destroying \
                 (unexpected reply {other:?}); proceeding"
            );
            Vec::new()
        }
        Err(err) => {
            eprintln!(
                "warning: could not determine running dataflows before destroying \
                 ({err}); proceeding"
            );
            Vec::new()
        }
    };

    // `--force` means "clean up anyway", so it has to actually clean up.
    // `Destroy` alone does not: the coordinator sends each dataflow a stop
    // and then tears the daemons down immediately, without waiting for the
    // stop ladder (grace -> SIGTERM -> SIGKILL) to run. A node that exits
    // promptly on `Stop` is reaped; a wedged one outlives its daemon and is
    // orphaned. Stopping first goes through the path that does wait, which
    // is also the one the refusal message recommends.
    if force && !running.is_empty() {
        println!(
            "Stopping {} running dataflow(s) before teardown...",
            running.len()
        );
        for entry in &running {
            if let Err(err) = crate::command::stop::stop_dataflow(entry.uuid, None, true, &session)
            {
                // Report and continue: a dataflow that cannot be stopped
                // must not block teardown, which is what `--force` was
                // asked for.
                eprintln!("warning: failed to stop {entry}: {err}");
            }
        }
    }

    if let DownDecision::Refuse(running) = down_decision(running, force) {
        let list = running
            .iter()
            .map(|d| format!("  - {d}"))
            .collect::<Vec<_>>()
            .join("\n");
        eyre::bail!(
            "refusing to destroy the coordinator at {coordinator_addr}: \
             {} dataflow(s) still running\n{list}\n\n  \
             This tears down the coordinator, its daemons and every dataflow \
             above. Lifecycle commands target whatever coordinator owns the \
             port, so this may not be the instance you meant — a different \
             checkout or another project on this machine shares \
             {coordinator_addr} by default.\n\n  \
             hint: stop the dataflows first (`dora stop --all`), or re-run \
             with `--force` if you do mean to kill them\n  \
             hint: to keep instances isolated, give each one its own port via \
             `DORA_COORDINATOR_PORT` (or `--coordinator-port`)",
            running.len(),
        );
    }

    println!("Destroying coordinator at {coordinator_addr}");

    // send destroy command to dora-coordinator
    let reply = send_control_request(&session, &ControlRequest::Destroy)?;
    match reply {
        ControlRequestReply::DestroyOk => {
            println!("Coordinator and daemons destroyed successfully");
        }
        other => {
            bail!("unexpected reply to Destroy: {other:?}");
        }
    }

    Ok(())
}

fn parse_dora_config(config_path: Option<&Path>) -> Result<UpConfig, eyre::ErrReport> {
    let path = config_path.or_else(|| Some(Path::new("dora-config.yml")).filter(|p| p.exists()));
    let config = match path {
        Some(path) => {
            let raw = fs::read_to_string(path)
                .with_context(|| format!("failed to read `{}`", path.display()))?;
            serde_yaml::from_str(&raw)
                .with_context(|| format!("failed to parse `{}`", path.display()))?
        }
        None => Default::default(),
    };
    Ok(config)
}

/// Path to the `dora` console script recorded by the `dora-rs-cli` wheel's
/// `py_main`. See [`set_python_executable_path`].
static PYTHON_EXECUTABLE_PATH: std::sync::OnceLock<std::ffi::OsString> = std::sync::OnceLock::new();

/// Record the console-script/executable path (`sys.argv[0]`) that the wheel's
/// `py_main` was launched with, so [`dora_executable_path`] can re-spawn the
/// real `dora` binary (`dora coordinator`, `dora daemon`, …).
///
/// `sys.argv[0]` is the robust source on **both** platforms: pip's launcher
/// puts the console-script path there on Unix and the `dora.exe` path there on
/// Windows. This is called from [`crate::lib_main_from_argv`], the single entry
/// point both the wheel and the standalone binary go through.
pub(crate) fn set_python_executable_path(path: std::ffi::OsString) {
    // First writer wins; a second call (there is none in practice) is ignored.
    let _ = PYTHON_EXECUTABLE_PATH.set(path);
}

/// Decide which executable to re-spawn as `dora`.
///
/// Split out from [`dora_executable_path`] so the platform-independent decision
/// is unit-testable without the `python` feature compiled in.
///
/// - From the Python wrapper (`dora-rs-cli` wheel): use the recorded
///   `sys.argv[0]`. `current_exe()` is wrong there — inside the embedded
///   interpreter it returns the Python interpreter on Unix, so spawning it would
///   run `python coordinator`. The previous `args_os().nth(1)` was also wrong on
///   Windows, where the process argv has no interpreter entry and `nth(1)` is
///   the subcommand (e.g. `"up"`) rather than the dora path (#3327).
/// - Standalone binary: use `current_exe()`.
fn choose_executable_path(
    from_python_wrapper: bool,
    recorded: Option<std::ffi::OsString>,
    current_exe: impl FnOnce() -> std::io::Result<PathBuf>,
) -> eyre::Result<std::ffi::OsString> {
    if from_python_wrapper {
        recorded.context(
            "could not determine the dora executable path from the Python wrapper \
             (sys.argv[0] was not recorded)",
        )
    } else {
        current_exe()
            .map(Into::into)
            .wrap_err("could not determine dora executable path")
    }
}

pub(crate) fn dora_executable_path() -> eyre::Result<std::ffi::OsString> {
    choose_executable_path(
        cfg!(feature = "python"),
        PYTHON_EXECUTABLE_PATH.get().cloned(),
        std::env::current_exe,
    )
}

#[cfg(test)]
mod executable_path_tests {
    use super::choose_executable_path;
    use std::ffi::OsString;
    use std::path::PathBuf;

    #[test]
    fn python_wrapper_uses_recorded_argv0() {
        // In the wheel, the recorded `sys.argv[0]` (the console-script path) must
        // be used verbatim -- not `current_exe()`, which would be the embedded
        // Python interpreter on Unix.
        let path = choose_executable_path(true, Some(OsString::from("/opt/venv/bin/dora")), || {
            panic!("current_exe must not be consulted for the Python wrapper")
        })
        .expect("recorded path should resolve");
        assert_eq!(path, OsString::from("/opt/venv/bin/dora"));
    }

    #[test]
    fn python_wrapper_without_recorded_path_errors() {
        let err = choose_executable_path(true, None, || Ok(PathBuf::from("/should/not/be/used")))
            .expect_err("missing recorded path must be an error, not a wrong fallback");
        assert!(
            format!("{err:#}").contains("Python wrapper"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn standalone_binary_uses_current_exe() {
        let path = choose_executable_path(false, None, || Ok(PathBuf::from("/usr/bin/dora")))
            .expect("current_exe should resolve");
        assert_eq!(path, OsString::from("/usr/bin/dora"));
    }

    #[test]
    fn standalone_binary_ignores_any_recorded_path() {
        // Even if a path was recorded, the standalone binary trusts `current_exe`.
        let path =
            choose_executable_path(false, Some(OsString::from("/opt/venv/bin/dora")), || {
                Ok(PathBuf::from("/usr/bin/dora"))
            })
            .expect("current_exe should resolve");
        assert_eq!(path, OsString::from("/usr/bin/dora"));
    }
}

/// Detach a child process so it survives after the parent exits:
/// - null stdio prevents broken-pipe crashes when the parent's terminal closes
/// - new process group prevents terminal signals (SIGHUP/SIGINT) from propagating
pub(crate) fn detach_process(cmd: &mut Command) {
    detach_process_with_stderr(cmd, Stdio::null());
}

fn detach_process_with_stderr(cmd: &mut Command, stderr: impl Into<Stdio>) {
    cmd.stdin(Stdio::null());
    cmd.stdout(Stdio::null());
    cmd.stderr(stderr);
    #[cfg(unix)]
    {
        use std::os::unix::process::CommandExt;
        cmd.process_group(0);
    }
}

/// How a spawned `dora coordinator` child should be configured.
///
/// Both `dora up` and `dora cluster up` start a coordinator, and they used to
/// do it with two independent implementations that disagreed on the most
/// important default: `dora up` passed no `--interface` at all (so the child
/// inherited loopback and no remote daemon could reach it) while
/// `dora cluster up` passed `0.0.0.0`. The teardown half of the pair has been
/// shared since `dora cluster down` started delegating to [`down`]; this is
/// the same consolidation for startup.
///
/// `None` means "do not pass the flag", leaving the coordinator's own default.
pub(crate) struct CoordinatorSpawn {
    pub interface: Option<std::net::IpAddr>,
    pub port: Option<u16>,
    pub auth: bool,
}

pub(crate) fn spawn_coordinator(spawn: CoordinatorSpawn) -> eyre::Result<CoordinatorStartup> {
    let CoordinatorSpawn {
        interface,
        port,
        auth,
    } = spawn;
    let path = dora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("coordinator");
    cmd.arg("--quiet");
    if let Some(interface) = interface {
        cmd.args(["--interface".to_string(), interface.to_string()]);
    }
    if let Some(port) = port {
        cmd.args(["--port".to_string(), port.to_string()]);
    }
    if auth {
        cmd.arg("--auth");
    }
    let stderr = open_coordinator_stderr_capture();
    let child_stderr = stderr
        .as_ref()
        .and_then(|file| file.try_clone().ok())
        .map(Into::into)
        .unwrap_or_else(Stdio::null);
    detach_process_with_stderr(&mut cmd, child_stderr);
    let child = cmd.spawn().wrap_err(
        "failed to run `dora coordinator`\n\n  \
         hint: ensure the `dora` binary is in your PATH",
    )?;

    match interface {
        Some(interface) if !interface.is_loopback() && !interface.is_unspecified() => {
            // Every other lifecycle command (`list`, `logs`, `stop`, `start`,
            // `down`) defaults `--coordinator-addr` to loopback, where a
            // coordinator bound to a concrete address does not answer. `dora up`
            // derives its own connect address from `--interface`; the rest
            // cannot, so say plainly what they need.
            println!("started dora coordinator on {interface} (reachable from other machines)");
            println!(
                "  other commands default to loopback — export \
                 DORA_COORDINATOR_ADDR={interface} (or pass --coordinator-addr {interface})"
            );
        }
        Some(interface) if !interface.is_loopback() => {
            println!("started dora coordinator on {interface} (reachable from other machines)");
        }
        _ => println!("started dora coordinator"),
    }

    Ok(CoordinatorStartup { child, stderr })
}

/// Start the local daemon that `dora up` manages.
///
/// `coordinator_addr` is the address the daemon should use to reach the
/// coordinator. It is not merely how the daemon connects: the daemon derives
/// the address its *zenoh* listener binds from it (the local address that
/// routes toward the coordinator), and a loopback coordinator therefore yields
/// a loopback listener that no other machine can dial. Passing the routable
/// interface through is what lets a daemon on another machine reach this one
/// directly instead of falling back to multicast — see `zenoh_bind_address_for`.
///
/// `None` leaves the daemon's own default (loopback), which is what a
/// single-machine `dora up` wants.
fn start_daemon(coordinator_addr: Option<std::net::IpAddr>, port: u16) -> eyre::Result<()> {
    let path = dora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("daemon");
    cmd.arg("--quiet");
    // Passed explicitly for the same reason the coordinator's is: relying on
    // environment inheritance means a port resolved here from a flag rather
    // than a variable would not reach the child.
    cmd.args(["--coordinator-port".to_string(), port.to_string()]);
    if let Some(addr) = coordinator_addr {
        cmd.args(["--coordinator-addr".to_string(), addr.to_string()]);
    }
    detach_process(&mut cmd);
    cmd.spawn().wrap_err(
        "failed to run `dora daemon`\n\n  \
         hint: ensure the `dora` binary is in your PATH",
    )?;

    println!("started dora daemon");

    Ok(())
}

#[cfg(test)]
#[cfg(feature = "redb-backend")]
mod tests {
    use super::{
        LOCALHOST, archive_coordinator_store, available_backup_path, connect_addr_for,
        ensure_no_listener_before_archive, lock_coordinator_store_recreation,
    };
    use std::sync::mpsc;
    use std::time::Duration;

    /// The crux of the `--interface` handling: the address we poll for
    /// readiness has to be one the coordinator actually answers on, or `dora up`
    /// kills the coordinator it just started and blames the port.
    #[test]
    fn connect_address_follows_a_concrete_bind_interface() {
        use std::net::IpAddr;
        let iface: IpAddr = "192.168.1.10".parse().unwrap();

        // Nothing configured: loopback, exactly as before this flag existed.
        assert_eq!(connect_addr_for(None, None).unwrap(), LOCALHOST);
        // A concrete bind is also where it answers.
        assert_eq!(connect_addr_for(None, Some(iface)).unwrap(), iface);
        // A wildcard bind covers loopback, so the default stands.
        assert_eq!(
            connect_addr_for(None, Some("0.0.0.0".parse().unwrap())).unwrap(),
            LOCALHOST
        );
        // An explicit env var wins — that is how a NAT or port mapping is
        // expressed — including alongside a wildcard bind.
        let env: IpAddr = "10.9.9.9".parse().unwrap();
        assert_eq!(connect_addr_for(Some(env), None).unwrap(), env);
        assert_eq!(
            connect_addr_for(Some(env), Some("0.0.0.0".parse().unwrap())).unwrap(),
            env
        );
        // Agreeing values are not a conflict.
        assert_eq!(connect_addr_for(Some(iface), Some(iface)).unwrap(), iface);
    }

    /// A contradiction is named rather than resolved into a startup timeout:
    /// `DORA_COORDINATOR_ADDR=127.0.0.1` in a shell profile plus
    /// `--interface <routable>` used to bind one address and poll another.
    #[test]
    fn a_bind_interface_contradicting_the_connect_address_is_rejected() {
        let err = connect_addr_for(Some(LOCALHOST), Some("192.168.1.10".parse().unwrap()))
            .expect_err("contradicting addresses must be refused");
        let msg = format!("{err}");
        assert!(msg.contains("DORA_COORDINATOR_ADDR"), "{msg}");
        assert!(msg.contains("--interface"), "{msg}");
    }

    /// A coordinator bound to loopback must block the archive even when the
    /// address we failed to connect to was a different one.
    ///
    /// `dora up --recreate-store --interface <routable IP>` connects to that
    /// routable address; a coordinator started by a plain `dora up` earlier is
    /// listening on loopback and never sees that probe. Probing only the
    /// address we tried would archive the redb file out from under it.
    #[test]
    fn archive_refused_when_a_coordinator_holds_the_port_on_loopback_only() {
        let listener =
            std::net::TcpListener::bind("127.0.0.1:0").expect("bind loopback coordinator stand-in");
        let port = listener.local_addr().expect("read local addr").port();

        // A routable-looking address on the same port, which nothing is bound
        // to — exactly what `--interface <routable IP>` would hand us.
        let unbound: std::net::SocketAddr = format!("10.255.255.1:{port}").parse().unwrap();
        let result = ensure_no_listener_before_archive(unbound, eyre::eyre!("connection refused"));

        assert!(
            result.is_err(),
            "archive must be refused while a coordinator holds port {port} on loopback"
        );
    }

    #[test]
    fn archive_refused_while_store_file_is_locked() {
        use fs2::FileExt as _;

        let dir = tempfile::tempdir().expect("create temporary store directory");
        let store = dir.path().join("coordinator.redb");
        std::fs::write(&store, [1u8]).expect("create store file");
        let holder = std::fs::File::open(&store).expect("open store file");
        holder
            .try_lock_exclusive()
            .expect("lock store file like an open database would");

        let err = archive_coordinator_store(&store)
            .expect_err("archive must be refused while the store file is locked");
        assert!(
            format!("{err:#}").contains("refusing to archive"),
            "unexpected error: {err:#}"
        );
        assert!(store.exists(), "locked store must not be renamed");

        fs2::FileExt::unlock(&holder).expect("release store lock");
        archive_coordinator_store(&store).expect("archive must succeed once the lock is free");
        assert!(!store.exists(), "store was not renamed");
        assert!(
            dir.path().join("coordinator.redb.backup").exists(),
            "backup was not created"
        );
    }

    #[test]
    fn recreate_store_archive_refused_while_port_is_listening() {
        let listener = std::net::TcpListener::bind(("127.0.0.1", 0)).expect("bind listener");
        let addr = listener.local_addr().expect("listener addr");

        let err = ensure_no_listener_before_archive(addr, eyre::eyre!("handshake failed"))
            .expect_err("archive must be refused while a listener holds the port");
        assert!(
            format!("{err:#}").contains("refusing to archive"),
            "unexpected error: {err:#}"
        );

        drop(listener);
        ensure_no_listener_before_archive(addr, eyre::eyre!("connection refused"))
            .expect("archive must be allowed when nothing is listening");
    }

    #[test]
    fn coordinator_store_backup_path_skips_existing_backups() {
        let dir = tempfile::tempdir().expect("create temporary store directory");
        let store = dir.path().join("coordinator.redb");
        std::fs::write(dir.path().join("coordinator.redb.backup"), []).expect("create backup");
        std::fs::write(dir.path().join("coordinator.redb.backup.1"), [])
            .expect("create numbered backup");

        let backup = available_backup_path(&store).expect("select available backup path");

        assert_eq!(backup, dir.path().join("coordinator.redb.backup.2"));
    }

    #[test]
    fn coordinator_store_recreation_lock_serializes_callers() {
        let dir = tempfile::tempdir().expect("create temporary store directory");
        let store = dir.path().join("coordinator.redb");
        let first = lock_coordinator_store_recreation(&store).expect("acquire first lock");
        let (tx, rx) = mpsc::channel();

        let waiter = std::thread::spawn(move || {
            let second = lock_coordinator_store_recreation(&store).expect("acquire second lock");
            tx.send(second).expect("report acquired lock");
        });

        assert!(
            rx.recv_timeout(Duration::from_millis(100)).is_err(),
            "second recreation acquired the lock before the first released it"
        );
        drop(first);
        let second = rx
            .recv_timeout(Duration::from_secs(2))
            .expect("second recreation did not acquire the released lock");
        drop(second);
        waiter.join().expect("join lock waiter");
    }
}

#[cfg(test)]
mod coordinator_env_tests {
    use super::{DORA_COORDINATOR_PORT_WS_DEFAULT, LOCALHOST, parse_coordinator_env};
    use std::net::IpAddr;

    #[test]
    fn unset_uses_default() {
        let addr: IpAddr = parse_coordinator_env("DORA_COORDINATOR_ADDR", None, LOCALHOST).unwrap();
        assert_eq!(addr, LOCALHOST);
        let port: u16 = parse_coordinator_env(
            "DORA_COORDINATOR_PORT",
            None,
            DORA_COORDINATOR_PORT_WS_DEFAULT,
        )
        .unwrap();
        assert_eq!(port, DORA_COORDINATOR_PORT_WS_DEFAULT);
    }

    #[test]
    fn valid_value_is_parsed() {
        let addr: IpAddr =
            parse_coordinator_env("DORA_COORDINATOR_ADDR", Some("10.0.0.5"), LOCALHOST).unwrap();
        assert_eq!(addr, "10.0.0.5".parse::<IpAddr>().unwrap());
        let port: u16 = parse_coordinator_env(
            "DORA_COORDINATOR_PORT",
            Some("6100"),
            DORA_COORDINATOR_PORT_WS_DEFAULT,
        )
        .unwrap();
        assert_eq!(port, 6100);
    }

    #[test]
    fn malformed_value_errors_instead_of_defaulting() {
        // Regression: `dora up` previously swallowed a parse error via
        // `.and_then(|s| s.parse().ok()).unwrap_or(default)`, silently binding
        // the default port while every other command errored on the same typo.
        let err = parse_coordinator_env::<u16>(
            "DORA_COORDINATOR_PORT",
            Some("not-a-port"),
            DORA_COORDINATOR_PORT_WS_DEFAULT,
        )
        .expect_err("a malformed port must be rejected, not defaulted");
        assert!(format!("{err:#}").contains("DORA_COORDINATOR_PORT"));

        assert!(
            parse_coordinator_env::<IpAddr>("DORA_COORDINATOR_ADDR", Some("999.1.1.1"), LOCALHOST)
                .is_err()
        );
    }
}

#[cfg(test)]
mod destroy_guard_tests {
    use super::{DownDecision, down_decision};
    use dora_message::coordinator_to_cli::DataflowIdAndName;

    // ---- dora-rs/dora#2924: `dora down` must not silently destroy an
    //      unrelated instance's coordinator ----

    fn df(name: &str) -> DataflowIdAndName {
        DataflowIdAndName {
            uuid: uuid::Uuid::nil(),
            name: Some(name.to_string()),
        }
    }

    /// The reported case: a coordinator with live work is not torn down
    /// just because someone ran `dora destroy` in another checkout.
    #[test]
    fn down_refuses_when_dataflows_are_running() {
        match down_decision(vec![df("long-experiment")], false) {
            DownDecision::Refuse(blocked) => {
                let names: Vec<_> = blocked.iter().filter_map(|d| d.name.clone()).collect();
                assert_eq!(
                    names,
                    vec!["long-experiment"],
                    "the refusal must name what it would have killed, so the \
                     operator can tell whose coordinator they just hit"
                );
            }
            other => panic!(
                "a destroy that would kill running dataflows must stop and say \
                 so, got {other:?}"
            ),
        }
    }

    /// An idle coordinator is the normal teardown case and must stay
    /// frictionless — otherwise every `dora up` / `dora down` loop grows a
    /// flag and people learn to pass `--force` reflexively.
    #[test]
    fn down_proceeds_when_nothing_is_running() {
        assert!(
            matches!(down_decision(Vec::new(), false), DownDecision::Proceed),
            "an idle coordinator must tear down without a flag"
        );
    }

    /// `--force` is the documented escape hatch for "yes, kill them".
    #[test]
    fn force_destroys_despite_running_dataflows() {
        assert!(
            matches!(down_decision(vec![df("busy")], true), DownDecision::Proceed),
            "`--force` is the documented way to say \"yes, kill them\""
        );
    }
}
