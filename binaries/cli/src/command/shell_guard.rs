use std::{process::Command, time::Duration};

use super::Executable;
use dora_core::topics::DORA_RUN_PARENT_PID_ENV;
use libc::pid_t;

/// Hidden subcommand: keeps a `path: shell` node's process group contained.
///
/// The daemon wraps shell node spawns with `dora __shell-guard -- sh -c <args>`
/// on unix, but only on the in-process `dora run` / `Daemon::run_dataflow`
/// spawn path. The `dora` executable to re-spawn is the CLI host itself —
/// `current_exe` for the standalone binary, the recorded `sys.argv[0]`
/// console-script path for the `dora-rs-cli` wheel, whose interpreter is not a
/// `dora` binary (see `binaries/daemon/src/spawn/command.rs`).
///
/// The guard does two things and then gets out of the way:
///
/// 1. It forks a *watchdog* — detached from the node with a double `fork`, so
///    it is not a child of the shell — that watches the `DORA_RUN_PARENT_PID`
///    process and `killpg`s the node's group once that process is gone. That is
///    the one case no other process can cover: a `SIGKILL`ed `dora run` runs no
///    teardown code, so a shell that backgrounded a fork (`sh -c 'cmd & wait'`)
///    would leave the fork orphaned to `ppid 1` (dora-rs/dora#3472).
/// 2. It `exec`s the shell *in place*, keeping the pid and the process-group
///    leadership the daemon gave the guard. The shell is therefore the node
///    exactly as if the daemon had spawned it directly: `$$` is the shell's own
///    process group, `kill -- -$$` reaches it, and the daemon's TERM-then-KILL
///    stop ladder and its exit-time `killpg` address the node unchanged
///    (dora-rs/dora#3472 review). Exec-in-place is what preserves that; a guard
///    that stayed between the daemon and the shell would make the shell an
///    ordinary child, so `$$` would no longer be its pgid.
///
/// On the *normal* stop path (a terminal `dora run`, `--stop-after`, …) the
/// daemon `killpg`s the node's group; the shell gets the signal directly and
/// the watchdog — which sits outside that group, see [`watchdog`] — exits as
/// soon as the group is gone. On a node that exits on its own the daemon
/// `killpg`s the group the same way (`binaries/daemon/src/spawn/prepared.rs`,
/// `contain_exited_group`), with the watchdog collecting nothing and exiting.
///
/// On the coordinator-attached path (`dora up` + `dora start`) the daemon does
/// not use the guard at all — nodes there are meant to outlive the daemon
/// (#2029) and never carry `DORA_RUN_PARENT_PID`. Rendering the guard directly
/// then just `exec`s the command, since there is nothing to contain.
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

/// How often the watchdog re-checks the parent — same interval as the in-node
/// orphan guard (`apis/rust/node/src/orphan_guard.rs`).
const POLL_INTERVAL: Duration = Duration::from_millis(500);

fn shell_guard_main(program: &str, args: &[String]) -> eyre::Result<()> {
    let parent: Option<u32> = std::env::var(DORA_RUN_PARENT_PID_ENV)
        .ok()
        .and_then(|raw| raw.trim().parse().ok());

    match parent {
        // Nothing to contain: exec the command so it keeps this pid and group,
        // exactly as on the guarded path.
        None => exec_in_place(program, args),
        Some(parent) => armed(program, args, parent),
    }
}

/// Armed mode: leave the watchdog behind, then become the command.
fn armed(program: &str, args: &[String], parent: u32) -> eyre::Result<()> {
    let parent = parent as pid_t;

    // The daemon's pre-exec armed `PR_SET_PDEATHSIG(SIGKILL)` on us. The
    // watchdog owns containment from here, and the signal is a liability: it
    // fires on the death of the *spawning thread*, not the daemon process.
    clear_parent_death_signal();

    // Whether the guard is the daemon's direct child (the common case on the
    // `dora run` path), which makes `getppid()` an exact liveness test.
    // SAFETY: reads this process's own parent id.
    let direct_child = unsafe { libc::getppid() } == parent;

    // Check the parent *before* leaving the watchdog: if it is already gone
    // there is nothing left to contain and nothing worth running the command
    // for. The guard has spawned nothing yet, so its group is only itself.
    if parent_is_gone(parent, direct_child) {
        std::process::exit(0);
    }

    leave_watchdog(parent)?;
    exec_in_place(program, args)
}

/// Fork the containment watchdog and return in the guard, ready to `exec`.
///
/// Two forks on purpose. The intermediate child exits at once and is reaped
/// here, so the watchdog is reparented to `init` rather than becoming a child
/// of the shell this function's caller is about to `exec` into. A direct child
/// would be caught by the node's own `wait` — `sh -c 'cmd & wait'` waits for
/// *every* child — and the node would hang instead of exiting (dora-rs/dora#3472
/// review).
fn leave_watchdog(parent: pid_t) -> eyre::Result<()> {
    // SAFETY: this runs before any thread is spawned, so a plain `fork` has no
    // lock to strand; the forked children below call only async-signal-safe
    // functions (`fork`, `getpgrp`, `setpgid`, `kill`, `nanosleep`, `_exit`).
    match unsafe { libc::fork() } {
        -1 => eyre::bail!(
            "__shell-guard: fork failed: {}",
            std::io::Error::last_os_error()
        ),
        // The intermediate: fork the real watchdog and exit, so the watchdog is
        // reparented away from the shell.
        0 => match unsafe { libc::fork() } {
            -1 => unsafe { libc::_exit(1) },
            0 => watchdog(parent),
            _ => unsafe { libc::_exit(0) },
        },
        intermediate => {
            // Reap the intermediate before continuing, so the shell starts with
            // no leftover child of its own. Best effort: a failure only leaves a
            // zombie, which the group kill collects anyway.
            let mut status = 0;
            // SAFETY: `waitpid` on the pid just forked; `status` is a valid
            // out-pointer for the call.
            unsafe { libc::waitpid(intermediate, &mut status, 0) };
            Ok(())
        }
    }
}

/// The watchdog: a detached process that contains the node's group if the
/// daemon dies while the node is still running.
///
/// It reads the daemon's pid directly rather than through `getppid`, because
/// the double `fork` in [`leave_watchdog`] left it a child of `init`. That
/// probe is one-sided: a recycled pid can leave the node running — today's
/// behavior, never a wrong kill.
fn watchdog(parent: pid_t) -> ! {
    // The group the daemon created for this node. Captured before moving out of
    // it: this is what the watchdog contains if the daemon dies.
    // SAFETY: reads this process's own process-group id.
    let node_group = unsafe { libc::getpgrp() };
    // Move into a group of our own. The daemon watches the node's group to
    // decide when a node is really gone (`contain_exited_group`): a watchdog
    // still sitting in it would read as a live straggler — a spurious
    // "abandoned" warning on every clean exit and a full grace period added to
    // every stop. Out here it is invisible to that scan, dies only by its own
    // hand, and is still in place to contain the node's group.
    //
    // Best effort: `setpgid(0, 0)` on a process that is not a session leader
    // does not fail in practice, and the fallback (staying in the node's group)
    // only costs the spurious scan behavior above, not containment.
    // SAFETY: affects this process's own group id only.
    unsafe { libc::setpgid(0, 0) };

    loop {
        if parent_is_gone(parent, false) {
            // The daemon is gone: contain the node and every background fork it
            // left, as the daemon's own reaper would.
            // SAFETY: signals the node's group, which the daemon created for
            // this node alone, so no unrelated process can be signalled.
            unsafe { libc::killpg(node_group, libc::SIGKILL) };
            // SAFETY: `_exit` ends the watchdog; it touches no shared state.
            unsafe { libc::_exit(0) };
        }
        // The node's group is gone: the node finished or was stopped, and the
        // daemon has already collected its stragglers. Nothing is left to
        // contain, so the watchdog is done. Only a recycled pgid could delay
        // this, which is why the check never kills on its own.
        // SAFETY: signal 0 performs error checking only and sends no signal.
        let probe = unsafe { libc::killpg(node_group, 0) };
        if probe != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH) {
            // SAFETY: `_exit` ends the watchdog; it touches no shared state.
            unsafe { libc::_exit(0) };
        }
        sleep_in_poll();
    }
}

/// Sleep one poll interval using only async-signal-safe calls: the watchdog
/// runs in a forked child and must not touch the allocator or a lock the
/// pre-fork process might have held.
fn sleep_in_poll() {
    let interval = libc::timespec {
        tv_sec: 0,
        tv_nsec: POLL_INTERVAL.as_nanos() as libc::c_long,
    };
    // SAFETY: `nanosleep` is async-signal-safe, affects only this thread, and
    // accepts a null remainder.
    unsafe { libc::nanosleep(&interval, std::ptr::null_mut()) };
}

/// Replace this process with `program`.
///
/// `exec` keeps the pid and the process-group leadership, so the command
/// becomes the node exactly as the daemon spawned it. The node's interpreter
/// env the daemon set aside is handed back here.
fn exec_in_place(program: &str, args: &[String]) -> eyre::Result<()> {
    use std::os::unix::process::CommandExt as _;

    let mut command = Command::new(program);
    command.args(args);
    restore_node_env(&mut command);
    // `exec` returns only on failure; on success this process *is* the command.
    let error = command.exec();
    Err(eyre::eyre!(
        "__shell-guard: failed to exec `{program}`: {error}"
    ))
}

/// Put the node's interpreter env back for the exec, under its own names.
///
/// The daemon keeps the node's `PYTHON*` variables off the guard host — under
/// the `dora-rs-cli` wheel the host is itself a python console script, and any
/// of them can stop that python from starting, taking the guard down with it —
/// by handing them over under `DORA_SHELL_GUARD_`-prefixed names. The exec'd
/// command is the process they were meant for, so restore them and drop the
/// prefixed copies.
fn restore_node_env(command: &mut Command) {
    restore_node_env_from(command, std::env::vars());
}

/// `restore_node_env`, with the input read from `vars` instead of this
/// process's environment — which is the only thing a test can supply without
/// `set_var`, and that is process-global.
fn restore_node_env_from<I>(command: &mut Command, vars: I)
where
    I: IntoIterator<Item = (String, String)>,
{
    for (key, value) in vars {
        if let Some(name) = key.strip_prefix(GUARD_ENV_PREFIX) {
            command.env(name, value);
            command.env_remove(key);
        }
    }
}

/// Prefix the daemon hands the node's interpreter variables under, spelled the
/// same as `GUARD_ENV_PREFIX` in the daemon's `spawn/command.rs`. One string in
/// two crates, so they have to be changed together — if either is edited alone
/// the node silently loses its interpreter.
const GUARD_ENV_PREFIX: &str = "DORA_SHELL_GUARD_";

/// Whether `parent` has exited.
///
/// Two checks because there are two shapes, mirroring `ContainmentPlan` in
/// `apis/rust/node/src/orphan_guard.rs`:
///
/// - Direct child (the guard itself, before the exec): the kernel reparents an
///   orphan, so `getppid()` moving away from the recorded pid proves that pid
///   is gone, whether or not the id has since been recycled.
/// - Not a direct child (the watchdog): ask about the pid directly with signal
///   0.  `EPERM` means the process exists but is not ours to signal, so only an
///   outright lookup failure counts as gone.
fn parent_is_gone(parent: pid_t, direct_child: bool) -> bool {
    if direct_child {
        // SAFETY: reads this process's own parent id.
        return unsafe { libc::getppid() } != parent;
    }
    // SAFETY: signal 0 delivers nothing; it is a pure existence check.
    let probe = unsafe { libc::kill(parent, 0) };
    probe != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH)
}

/// Drop the parent-death signal the daemon set for the pre-`init` window.
///
/// The daemon arms `PR_SET_PDEATHSIG` at spawn so a child killed before the
/// watchdog forks is still contained. Once the watchdog is running the signal
/// is a liability — it fires on the death of the spawning *thread*, not the
/// daemon process — so the guard drops it before exec.
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// The node's interpreter selection is handed back under its own name for
    /// the exec, so a node that needs any `PYTHON*` variable still gets it even
    /// though the guard host ran without it.
    #[test]
    fn the_node_env_is_restored_under_its_own_name() {
        let mut command = Command::new("sh");
        restore_node_env_from(
            &mut command,
            [
                (
                    "DORA_SHELL_GUARD_PYTHONHOME".to_string(),
                    "/opt/py".to_string(),
                ),
                (
                    "DORA_SHELL_GUARD_PYTHONNOUSERSITE".to_string(),
                    "1".to_string(),
                ),
                ("PATH".to_string(), "/bin".to_string()),
            ],
        );

        let envs: BTreeMap<String, Option<String>> = command
            .get_envs()
            .map(|(k, v)| {
                (
                    k.to_string_lossy().into_owned(),
                    v.map(|v| v.to_string_lossy().into_owned()),
                )
            })
            .collect();
        assert_eq!(
            envs.get("PYTHONHOME"),
            Some(&Some("/opt/py".to_string())),
            "every PYTHON* the node set comes back under its own name, got {envs:?}"
        );
        assert_eq!(
            envs.get("PYTHONNOUSERSITE"),
            Some(&Some("1".to_string())),
            "not just PYTHONHOME/PYTHONPATH: {envs:?}"
        );
        assert!(
            !envs.contains_key("PATH"),
            "a variable the daemon did not set aside is left as inherited"
        );
        assert_eq!(
            envs.get("DORA_SHELL_GUARD_PYTHONHOME"),
            Some(&None),
            "the prefixed copy must not reach the node: {envs:?}"
        );
    }

    /// Nothing set aside means nothing overridden: the node keeps whatever it
    /// would otherwise inherit.
    #[test]
    fn without_a_passthrough_the_command_is_left_alone() {
        let mut command = Command::new("sh");
        restore_node_env_from(&mut command, [("PATH".to_string(), "/bin".to_string())]);
        assert_eq!(
            command.get_envs().count(),
            0,
            "nothing handed over must mean nothing overridden"
        );
    }
}
