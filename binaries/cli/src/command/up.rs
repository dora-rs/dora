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
        up(self.config.as_deref(), self.auth, self.recreate_store)
    }
}

#[derive(Debug, Default, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
struct UpConfig {}

pub(crate) fn up(config_path: Option<&Path>, auth: bool, recreate_store: bool) -> eyre::Result<()> {
    let UpConfig {} = parse_dora_config(config_path)?;
    let addr: std::net::IpAddr = std::env::var("DORA_COORDINATOR_ADDR")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(LOCALHOST);
    let port: u16 = std::env::var("DORA_COORDINATOR_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(DORA_COORDINATOR_PORT_WS_DEFAULT);
    let coordinator_addr = (addr, port).into();
    if recreate_store && !addr.is_loopback() {
        bail!(
            "--recreate-store only applies to the local default coordinator store, \
             but DORA_COORDINATOR_ADDR is set to the non-loopback address {addr}\n\n  \
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
                    start_and_wait_for_coordinator(coordinator_addr, port, auth)?
                }
            }
        }
        Err(_) => start_and_wait_for_coordinator(coordinator_addr, port, auth)?,
    };

    if !daemon_running(&session)? {
        start_daemon().wrap_err("failed to start dora-daemon")?;

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
    if std::net::TcpStream::connect_timeout(&coordinator_addr, Duration::from_secs(1)).is_ok() {
        return Err(connect_err.wrap_err(format!(
            "refusing to archive the coordinator store: a process is listening on \
             {coordinator_addr} but the connection failed\n\n  \
             hint: resolve the connection error below (e.g. an auth token mismatch), \
             or stop the coordinator with `dora down` before recreating the store"
        )));
    }
    Ok(())
}

struct CoordinatorStartup {
    child: std::process::Child,
    /// `None` when the capture file could not be created; startup proceeds
    /// without stderr in error reports rather than failing.
    stderr: Option<fs::File>,
}

fn start_and_wait_for_coordinator(
    coordinator_addr: SocketAddr,
    port: u16,
    auth: bool,
) -> eyre::Result<crate::ws_client::WsSession> {
    let startup = start_coordinator(auth).wrap_err("failed to start dora-coordinator")?;
    wait_for_coordinator_start(coordinator_addr, port, startup)
}

fn wait_for_coordinator_start(
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
    bail!("could not find an available coordinator store backup path");
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

pub(crate) fn dora_executable_path() -> eyre::Result<std::ffi::OsString> {
    if cfg!(feature = "python") {
        // When invoked via Python wrapper, argv[1] is the real dora binary path
        std::env::args_os()
            .nth(1)
            .context("could not get dora path from Python wrapper arguments")
    } else {
        std::env::current_exe()
            .map(Into::into)
            .wrap_err("could not determine dora executable path")
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

fn start_coordinator(auth: bool) -> eyre::Result<CoordinatorStartup> {
    let path = dora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("coordinator");
    cmd.arg("--quiet");
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

    println!("started dora coordinator");

    Ok(CoordinatorStartup { child, stderr })
}

fn start_daemon() -> eyre::Result<()> {
    let path = dora_executable_path()?;
    let mut cmd = Command::new(path);
    cmd.arg("daemon");
    cmd.arg("--quiet");
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
        archive_coordinator_store, available_backup_path, ensure_no_listener_before_archive,
        lock_coordinator_store_recreation,
    };
    use std::sync::mpsc;
    use std::time::Duration;

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
