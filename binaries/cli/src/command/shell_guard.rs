use eyre::Context;
use std::time::Duration;

use super::Executable;
use dora_core::topics::DORA_RUN_PARENT_PID_ENV;
use libc::pid_t;

/// Hidden subcommand: a process-group-aware supervisor for `path: shell` nodes.
///
/// The daemon wraps shell node spawns with `dora __shell-guard -- sh -c <args>`
/// on unix, but only on the in-process `dora run` / `Daemon::run_dataflow`
/// spawn path. The guard becomes the direct child (and process-group leader) of
/// the daemon, spawns the shell as its own child, and polls [`DORA_RUN_PARENT_PID`].
/// When the parent is gone, the guard `killpg`s its entire process group —
/// which, because the daemon wrapped it as `ProcessGroup::leader()`, covers the
/// shell and its background forks *while the guard is alive* (dora-rs/dora#3472).
///
/// On the *normal* stop path (a terminal `dora run`, `--stop-after`, …) the
/// daemon SIGTERMs the whole group — the shell gets the signal directly from
/// that group kill, so the guard forwards nothing. The guard only has to
/// survive those signals — it swallows SIGTERM/SIGINT/SIGHUP in armed mode so
/// it stays registered and can reap the shell before exiting — because without
/// the handlers it exits on the first SIGTERM, the node is unregistered, and
/// the daemon skips the group-SIGKILL escalation for a node it believes
/// already stopped: the TERM-ignoring shell and its background forks live on
/// as orphans (dora-rs/dora#3472 review).
///
/// The guard can only contain forks while it is running. A background fork
/// abandoned by a shell that already exited (`sh -c 'cmd &'` — the shell
/// returns immediately, the guard reaps it and exits) survives the guard, so
/// the daemon additionally `killpg`s a node's group the moment the node
/// process exits (`binaries/daemon/src/spawn/prepared.rs`, the node's
/// process-wait task); every node is spawned as its own group leader, so that
/// is the one place a finished node's stragglers are still reachable.
///
/// On the coordinator-attached path (`dora up` + `dora start`) the daemon does
/// not use the guard at all — nodes there are meant to outlive the daemon
/// (#2029) and would never carry [`DORA_RUN_PARENT_PID`]. Rendering the guard
/// directly would add a resident `dora` process per shell node for nothing, so
/// the daemon spawns `sh -c` itself. The passthrough below remains only as a
/// bare fallback for a manually-invoked guard that finds the env var absent.
#[derive(Debug, clap::Args)]
pub struct ShellGuardArgs {
    /// The command to run under the guard, e.g. `sh -c <args>`.
    ///
    /// Everything after `--` on the command line is collected here, so shell
    /// arguments starting with `-` are not mistaken for guard flags.
    #[clap(trailing_var_arg = true, allow_hyphen_values = true)]
    pub command: Vec<String>,
}

impl Executable for ShellGuardArgs {
    fn execute(self) -> eyre::Result<()> {
        let (program, args) = self
            .command
            .split_first()
            .ok_or_else(|| eyre::eyre!("__shell-guard: no command given"))?;
        shell_guard_main(program, args)
    }
}

/// How often the parent is re-checked — same interval as the in-node orphan
/// guard (`apis/rust/node/src/orphan_guard.rs`).
const POLL_INTERVAL: Duration = Duration::from_millis(500);

fn shell_guard_main(program: &str, args: &[String]) -> eyre::Result<()> {
    let parent: Option<u32> = std::env::var(DORA_RUN_PARENT_PID_ENV)
        .ok()
        .and_then(|raw| raw.trim().parse().ok());

    match parent {
        None => passthrough(program, args),
        Some(parent) => armed(program, args, parent),
    }
}

/// Transparent passthrough: spawn the child, wait for it, forward the exit
/// status.  Used on the coordinator-attached path where nodes must outlive
/// daemon restarts.
fn passthrough(program: &str, args: &[String]) -> eyre::Result<()> {
    use std::process::Command;

    let status = Command::new(program)
        .args(args)
        .status()
        .wrap_err_with(|| format!("failed to spawn `{program}`"))?;

    std::process::exit(status.code().unwrap_or(1));
}

/// Armed mode: the guard clears `PR_SET_PDEATHSIG` (set on it by the daemon's
/// pre_exec), spawns the child, and polls the parent.  A dedicated reaper
/// thread blocks on the shell's exit, so nothing delays node teardown; when
/// the parent is gone, the poll thread `killpg`s its own process group —
/// which, because the daemon wraps the guard with `ProcessGroup::leader()`,
/// contains the shell and all of its background forks.
fn armed(program: &str, args: &[String], parent: u32) -> eyre::Result<()> {
    use std::os::unix::process::ExitStatusExt as _;
    use std::process::Command;

    // Whether the guard is the daemon's direct child.  This is the common case
    // on the `dora run` path (no `--uv` wrapper for shell nodes), and it makes
    // `getppid()` an exact liveness test.
    // SAFETY: reads this process's own parent id.
    let direct_child = unsafe { libc::getppid() } == parent as pid_t;
    let parent = parent as pid_t;

    // The daemon's pre_exec sets `PR_SET_PDEATHSIG(SIGKILL)` on us (the direct
    // child).  Once the poll loop is running it is the sole containment
    // mechanism, and the signal is a liability — it fires on the death of the
    // *spawning thread*, not the daemon process.  Clear it now, mirroring the
    // in-node guard (`clear_parent_death_signal`).
    clear_parent_death_signal();

    // Install stop-signal handlers BEFORE spawning the child: the daemon's stop
    // ladder sends SIGTERM to the whole node process group, landing on the
    // guard and the shell at once.  With the default disposition the guard
    // dies first and is unregistered, the daemon then skips the SIGKILL
    // escalation for a node it thinks already stopped, and a shell that
    // ignores SIGTERM survives — reparented to init.  The handler keeps the
    // guard alive to stay registered and reap the shell (dora-rs/dora#3472
    // review).  A real handler rather than `SIG_IGN`: ignored dispositions
    // survive `exec` and would be inherited by the shell.
    install_stop_signal_handlers()?;

    // Check the parent *before* spawning the child — if it is already gone
    // there is nothing to contain and we should exit immediately.  The guard
    // has not spawned anything yet, so its group is (only) itself.
    if parent_is_gone(parent, direct_child) {
        std::process::exit(0);
    }

    let mut child = Command::new(program)
        .args(args)
        .spawn()
        .wrap_err_with(|| format!("failed to spawn `{program}` under guard"))?;

    // Reaper thread: block on the shell's exit and propagate its status, so a
    // node's normal teardown is not delayed by a poll interval.  When the
    // shell dies by a signal, die the same way — the daemon classifies stops
    // by the signal (e.g. `143` for SIGTERM) — and otherwise with its code.
    std::thread::spawn(move || match child.wait() {
        Ok(status) => {
            if let Some(signal) = status.signal() {
                re_raise(signal);
            }
            std::process::exit(status.code().unwrap_or(1));
        }
        Err(e) => {
            eprintln!("shell guard: failed to reap guarded process: {e}");
            std::process::exit(1);
        }
    });

    // Poll the parent for the containment side: when it is gone (e.g. a
    // SIGKILLed `dora run`), take down the whole group — the shell, its
    // background forks, and this guard.  The reaper thread owns the child, so
    // this group `killpg` is what ends a shell that outlives the daemon.  The
    // loop only ever fires here: stop signals need no action (the group kill
    // already reaches the shell), so the handlers above exist solely to keep
    // the guard alive until the reaper thread exits.
    loop {
        // Contain background forks when the parent is gone.
        if parent_is_gone(parent, direct_child) {
            contain();
        }

        std::thread::sleep(POLL_INTERVAL);
    }
}

/// The stop signals the guard must survive so it can reap the shell they were
/// sent to stop.
const STOP_SIGNALS: [libc::c_int; 3] = [libc::SIGTERM, libc::SIGINT, libc::SIGHUP];

/// Signal handler: swallow the signal.  The guard's only job on a stop signal
/// is to stay alive — the daemon group-kills the shell itself, so there is
/// nothing to re-forward, and the reaper thread ends the guard once the shell
/// is gone.  `SIG_IGN` would also work for staying alive but must not be used:
/// ignored dispositions survive `exec` and would be inherited by the shell,
/// changing shell semantics (e.g. rustup/sccache proxies that expect TERM to
/// hurt).
extern "C" fn swallow_stop_signal(_signal: libc::c_int) {}

fn install_stop_signal_handlers() -> eyre::Result<()> {
    for signal in STOP_SIGNALS {
        // SAFETY: `signal` installs a handler that touches no state.  The
        // handler never allocates or calls anything non-async-signal-safe.
        let prev = unsafe {
            libc::signal(
                signal,
                swallow_stop_signal as *const () as libc::sighandler_t,
            )
        };
        if prev == libc::SIG_ERR {
            eyre::bail!("failed to install a handler for signal {signal}");
        }
    }
    Ok(())
}

/// Reset `signal` to its default action and re-raise it, so the process dies
/// from the signal rather than the handler swallowing it.
fn re_raise(signal: i32) -> ! {
    // SAFETY: `signal` and `raise` are async-signal-safe, and resetting to the
    // default first is what makes a second delivery terminal.
    unsafe {
        libc::signal(signal, libc::SIG_DFL);
        libc::raise(signal);
    }
    // Defensive: if the signal was somehow blocked after re-raising, fall back
    // to the shell convention for "killed by signal N".
    std::process::exit(128 + signal);
}

/// Whether the process identified by `parent` has exited.
///
/// Two checks because there are two shapes, mirroring `ContainmentPlan` in
/// `apis/rust/node/src/orphan_guard.rs`:
///
/// - Direct child (the common `dora run` case): the kernel reparents an orphan,
///   so `getppid()` moving away from the recorded pid proves that pid is gone,
///   whether or not the id has since been recycled.
/// - Not a direct child: ask about the pid directly with signal 0.  `EPERM`
///   means the process exists but is not ours to signal, so only an outright
///   lookup failure counts as gone.
fn parent_is_gone(parent: pid_t, direct_child: bool) -> bool {
    if direct_child {
        // SAFETY: reads this process's own parent id.
        return unsafe { libc::getppid() } != parent;
    }
    // SAFETY: signal 0 delivers nothing; it is a pure existence check.
    let probe = unsafe { libc::kill(parent, 0) };
    probe != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH)
}

/// Kill this process's own process group.
///
/// The daemon wraps the guard with `ProcessGroup::leader()`, so the guard's
/// pgid equals its own pid and contains the shell + all of its background
/// forks.  `killpg` targets the group, not just this process, which is the
/// whole point: ending only the guard would leave orphans one level down.
///
/// Reached only when the parent is gone; there is no one to flush outputs to.
fn contain() -> ! {
    // SAFETY: `killpg` signals this process's own group.  The group exists
    // solely for this node (spawned as `ProcessGroup::leader()`), so there is
    // no risk of signalling an unrelated process group.
    unsafe {
        libc::killpg(libc::getpgrp(), libc::SIGKILL);
    }
    // `_exit`, not `std::process::exit`: this runs on the main thread while the
    // child may still be executing — `exit` would run `atexit` / destructors
    // concurrently and could deadlock on locks held by spawned threads.
    // SAFETY: `_exit` ends the process; it touches no state of ours.
    unsafe {
        libc::_exit(1);
    }
}

/// Drop the parent-death signal the daemon set for the pre-`init` window.
///
/// The daemon arms `PR_SET_PDEATHSIG` at spawn so a child killed before the
/// poll loop starts is still contained.  Once the loop is running, the signal
/// is not merely redundant but a liability — it fires on the death of the
/// spawning *thread*, not the daemon process — so the guard drops it.
///
/// On macOS there is no `PDEATHSIG` equivalent; this is a no-op there.
#[cfg(target_os = "linux")]
fn clear_parent_death_signal() {
    // SAFETY: `prctl` is async-signal-safe and this only clears this process's
    // own parent-death setting.
    unsafe {
        libc::prctl(libc::PR_SET_PDEATHSIG, 0 as libc::c_ulong);
    }
}

#[cfg(all(unix, not(target_os = "linux")))]
fn clear_parent_death_signal() {}
