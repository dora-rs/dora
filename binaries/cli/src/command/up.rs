use super::system::status::daemon_running;
use super::{Executable, default_tracing};
use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, connect_with_retry, send_control_request},
};
use dora_core::topics::DORA_COORDINATOR_PORT_WS_DEFAULT;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
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
    // Keep the lock through daemon readiness so overlapping recreation commands
    // cannot race either the fresh coordinator or its daemon startup.
    let mut _recreation_lock = None;
    let session = match connect_to_coordinator(coordinator_addr) {
        Ok(session) => {
            println!("coordinator already running on {coordinator_addr}");
            session
        }
        Err(_) if recreate_store => {
            _recreation_lock = Some(lock_default_coordinator_store_recreation()?);
            match connect_to_coordinator(coordinator_addr) {
                Ok(session) => {
                    println!("coordinator already running on {coordinator_addr}");
                    session
                }
                Err(_) => {
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

struct CoordinatorStartup {
    child: std::process::Child,
    stderr: fs::File,
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
            let stderr = captured_stderr(&mut startup.stderr)?;
            let mut message = format!("coordinator exited before it became ready: {status}");
            if !stderr.is_empty() {
                message.push('\n');
                message.push_str(&stderr);
            }
            if stderr.contains("redb schema version mismatch") {
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
                bail!(
                    "timed out waiting for coordinator to start at {coordinator_addr}: {err}\n\n  \
                     hint: is port {port} already in use? Check with `dora status`\n  \
                     or stop the existing coordinator with `dora down`"
                );
            }
        }
    }
}

fn captured_stderr(file: &mut fs::File) -> eyre::Result<String> {
    file.rewind()
        .wrap_err("failed to rewind coordinator stderr capture")?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes)
        .wrap_err("failed to read coordinator stderr capture")?;
    Ok(String::from_utf8_lossy(&bytes).trim().to_owned())
}

fn archive_default_coordinator_store() -> eyre::Result<()> {
    #[cfg(feature = "redb-backend")]
    {
        let path = super::coordinator::default_redb_path()?;
        if !path.exists() {
            println!(
                "no coordinator store at `{}`; nothing to archive",
                path.display()
            );
            return Ok(());
        }

        let backup = available_backup_path(&path)?;
        fs::rename(&path, &backup).with_context(|| {
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
    #[cfg(not(feature = "redb-backend"))]
    {
        bail!("--recreate-store requires the `redb-backend` feature")
    }
}

fn lock_default_coordinator_store_recreation() -> eyre::Result<fs::File> {
    #[cfg(feature = "redb-backend")]
    {
        let path = super::coordinator::default_redb_path()?;
        lock_coordinator_store_recreation(&path)
    }
    #[cfg(not(feature = "redb-backend"))]
    {
        bail!("--recreate-store requires the `redb-backend` feature")
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
    lock.lock_exclusive().with_context(|| {
        format!(
            "failed to lock coordinator store recreation at `{}`",
            lock_path.display()
        )
    })?;
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

pub(crate) fn down(
    config_path: Option<&Path>,
    coordinator_addr: SocketAddr,
) -> Result<(), eyre::ErrReport> {
    let UpConfig {} = parse_dora_config(config_path)?;
    // Retry connection briefly — the coordinator may still be initializing after `dora up`.
    let session = connect_with_retry(coordinator_addr, Duration::from_secs(5)).map_err(|_| {
        eyre::eyre!(
            "could not connect to coordinator at {coordinator_addr}\n\n  \
             hint: is it running? Start it with `dora up`"
        )
    })?;
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
    let stderr = tempfile::tempfile().wrap_err("failed to create coordinator stderr capture")?;
    let child_stderr = stderr
        .try_clone()
        .wrap_err("failed to clone coordinator stderr capture")?;
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

#[cfg(all(test, feature = "redb-backend"))]
mod tests {
    use super::{available_backup_path, lock_coordinator_store_recreation};
    use std::sync::mpsc;
    use std::time::Duration;

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
