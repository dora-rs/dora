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
///    would leave the fork orphaned to `ppid 1` (dora-rs/dora#3472). The
///    watchdog releases the node's stdout/stderr so it never holds `dora run`
///    open waiting for the log to end, and on Linux it treats a zombie as gone
///    on both sides: a killed-but-unreaped `dora run` must not read as alive,
///    nor a corpse left in the node's group as a member (dora-rs/dora#3472
///    review).
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

    // The watchdog outlives the node, and it inherited the node's stdout/stderr
    // pipes. The daemon reports a node finished only after the node's log
    // reaches EOF, so while the watchdog held those write ends a shell node's
    // exit would hang `dora run` for as long as the watchdog lived. Point them
    // at `/dev/null`: it prints nothing.
    redirect_stdio_away();

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
        // The node's group is empty: the node finished or was stopped, and the
        // daemon has already collected its stragglers. Nothing is left to
        // contain, so the watchdog is done. Only a recycled pgid could delay
        // this, which is why the check never kills on its own.
        if group_is_empty(node_group) {
            // SAFETY: `_exit` ends the watchdog; it touches no shared state.
            unsafe { libc::_exit(0) };
        }
        // A group that still answers `killpg` can hold nothing but corpses: the
        // node — the group leader, since the daemon spawns it with pgid == pid —
        // is gone, leaving only a reparented fork an init has not collected
        // (`docker run` without `--init`). Treat that as empty too, as the
        // daemon does in `contain_exited_group`: otherwise the watchdog outlives
        // the node holding a pipe shut, and later risks signalling a recycled
        // group (dora-rs/dora#3472 review).
        #[cfg(target_os = "linux")]
        {
            if proc_is_gone(node_group) {
                // SAFETY: `_exit` ends the watchdog; it touches no shared state.
                unsafe { libc::_exit(0) };
            }
        }
        sleep_in_poll();
    }
}

/// Point this process's stdio at `/dev/null`.
///
/// The watchdog is forked from the guard and so holds the node's stdout/stderr
/// pipe write ends. It lives past the node, so leaving them open keeps the
/// daemon from ever seeing the node's log end, and `dora run` never reports the
/// node finished (dora-rs/dora#3472 review). It writes nothing, so dropping them
/// costs nothing.
fn redirect_stdio_away() {
    // SAFETY: opens `/dev/null` and `dup2`s it over this process's own fds 0/1/2,
    // then closes the surplus descriptor. All async-signal-safe.
    unsafe {
        let null = b"/dev/null\0";
        let fd = libc::open(null.as_ptr().cast(), libc::O_RDWR);
        if fd < 0 {
            return;
        }
        for target in 0..=2 {
            libc::dup2(fd, target);
        }
        if fd > 2 {
            libc::close(fd);
        }
    }
}

/// Whether the process group is empty, zombie or not. One syscall, so it is the
/// cheap check that ends the loop on the common clean exit.
fn group_is_empty(pgid: pid_t) -> bool {
    // SAFETY: signal 0 performs error checking only and sends no signal.
    let probe = unsafe { libc::killpg(pgid, 0) };
    probe != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH)
}

/// Whether `pid` is gone: reaped, or a real single-threaded zombie.
///
/// `kill(pid, 0)` cannot tell those apart — a zombie still answers it — so a
/// parent that was killed but not yet reaped reads as alive, and the watchdog
/// never contains the node it was watching (dora-rs/dora#3472 review). The
/// `state` field on its own cannot either: the kernel reports `Z` for a process
/// whose main thread has ended while other threads keep running, and only a
/// single-threaded `Z` is a corpse. This mirrors the daemon's
/// `group_has_live_member` (`binaries/daemon/src/spawn/group_lifetime.rs`).
///
/// Linux only. Everywhere else the one-sided `kill(pid, 0)` probe stands, which
/// is no worse than `main` there (macOS has no `PDEATHSIG` to clear either, so
/// a shell node already outlived a killed `dora run`).
#[cfg(target_os = "linux")]
fn proc_is_gone(pid: pid_t) -> bool {
    match proc_state(pid) {
        None => true, // reaped, or never existed
        Some(parsed) => is_zombie(parsed),
    }
}

/// Whether a parsed `(state, num_threads)` is a real corpse rather than a
/// process still running on other threads.
#[cfg(target_os = "linux")]
fn is_zombie((state, threads): (u8, u32)) -> bool {
    state == b'Z' && threads <= 1
}

/// The `state` byte and `num_threads` from `/proc/<pid>/stat`, or `None` if the
/// entry cannot be read (reaped). Uses only `open`/`read`/`close` and a stack
/// buffer, so it is safe in the forked watchdog.
#[cfg(target_os = "linux")]
fn proc_state(pid: pid_t) -> Option<(u8, u32)> {
    let mut path = [0u8; 32];
    stat_path(&mut path, pid)?;
    // SAFETY: `path` holds a NUL-terminated path within the buffer.
    let fd = unsafe { libc::open(path.as_ptr().cast(), libc::O_RDONLY | libc::O_CLOEXEC) };
    if fd < 0 {
        return None;
    }
    let mut buf = [0u8; 256];
    // SAFETY: `buf` is a valid writable region of `buf.len()` bytes.
    let read = unsafe { libc::read(fd, buf.as_mut_ptr().cast(), buf.len()) };
    // SAFETY: closes the fd just opened.
    unsafe { libc::close(fd) };
    if read <= 0 {
        return None;
    }
    parse_stat_state(&buf[..read as usize])
}

/// Write `b"/proc/<pid>/stat\0"` into `buf`.
#[cfg(target_os = "linux")]
fn stat_path(buf: &mut [u8], pid: pid_t) -> Option<()> {
    let mut digits = [0u8; 10];
    let mut count = 0;
    let mut value = pid.max(0) as u32;
    loop {
        digits[count] = b'0' + (value % 10) as u8;
        count += 1;
        value /= 10;
        if value == 0 {
            break;
        }
    }
    let total = 6 + count + 6; // "/proc/", the pid, "/stat\0"
    if total > buf.len() {
        return None;
    }
    let mut i = 0;
    for &b in b"/proc/" {
        buf[i] = b;
        i += 1;
    }
    for j in 0..count {
        buf[i] = digits[count - 1 - j];
        i += 1;
    }
    for &b in b"/stat\0" {
        buf[i] = b;
        i += 1;
    }
    Some(())
}

/// The `state` (field 3) and `num_threads` (field 20) from a `/proc/<pid>/stat`
/// buffer.
///
/// The format is `pid (comm) state ppid …`, and `comm` may contain spaces and
/// parentheses, so the fields start after the *last* `)`.
#[cfg(target_os = "linux")]
fn parse_stat_state(buf: &[u8]) -> Option<(u8, u32)> {
    let close = buf.iter().rposition(|&b| b == b')')?;
    let rest = &buf[close + 1..];
    let mut state = None;
    let mut threads = 1;
    for (index, field) in rest
        .split(|&b| b == b' ')
        .filter(|f| !f.is_empty())
        .enumerate()
    {
        match index {
            0 => state = field.first().copied(),
            17 => threads = parse_u32(field).unwrap_or(1),
            _ => {}
        }
        if index == 17 {
            break;
        }
    }
    Some((state?, threads))
}

/// Parse an unsigned decimal, `None` on anything else (or overflow).
#[cfg(target_os = "linux")]
fn parse_u32(bytes: &[u8]) -> Option<u32> {
    let mut value = 0u32;
    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add((byte - b'0') as u32)?;
    }
    Some(value)
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
/// - Not a direct child (the watchdog): ask the kernel about the pid directly.
///   On Linux that means `/proc`, because `kill(pid, 0)` — and `getppid`, which
///   the double fork took away — both count a zombie as alive, and a
///   killed-but-unreaped `dora run` is exactly a zombie (dora-rs/dora#3472
///   review). `EPERM` means the process exists but is not ours to signal, so
///   only a real corpse or an outright lookup failure counts as gone.
fn parent_is_gone(parent: pid_t, direct_child: bool) -> bool {
    if direct_child {
        // SAFETY: reads this process's own parent id.
        return unsafe { libc::getppid() } != parent;
    }
    #[cfg(target_os = "linux")]
    if proc_is_gone(parent) {
        return true;
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

    /// `/proc/<pid>/stat` parsing: the fields follow the *comm*, which can hold
    /// spaces and parentheses, so they start after the last `)`. `num_threads`
    /// is field 20 (index 17 after it).
    #[cfg(target_os = "linux")]
    #[test]
    fn stat_fields_are_read_after_a_comm_that_holds_spaces_and_parens() {
        let live = b"1234 (sh) R 1 1234 1234 0 -1 4194560 100 0 0 0 1 1 0 0 20 0 1 0 12345 9999";
        assert_eq!(parse_stat_state(live), Some((b'R', 1)));

        let corpse = b"99 (a )b) Z 1 99 99 0 -1 0 0 0 0 0 0 0 0 0 20 0 2 0 0 0";
        assert_eq!(parse_stat_state(corpse), Some((b'Z', 2)));
    }

    /// A zombie is gone only when its main thread is the last one: a `Z` with
    /// other threads still running is not a corpse.
    #[cfg(target_os = "linux")]
    #[test]
    fn a_threaded_zombie_is_not_gone_but_a_plain_one_is() {
        let plain = b"7 (sh) Z 1 7 7 0 -1 0 0 0 0 0 0 0 0 0 20 0 1 0 0 0";
        let threaded = b"7 (sh) Z 1 7 7 0 -1 0 0 0 0 0 0 0 0 0 20 0 3 0 0 0";
        let running = b"7 (sh) S 1 7 7 0 -1 0 0 0 0 0 0 0 0 0 20 0 2 0 0 0";
        assert_eq!(parse_stat_state(plain), Some((b'Z', 1)));
        assert_eq!(parse_stat_state(threaded), Some((b'Z', 3)));
        assert_eq!(parse_stat_state(running), Some((b'S', 2)));
        assert!(is_zombie(parse_stat_state(plain).unwrap()));
        assert!(!is_zombie(parse_stat_state(threaded).unwrap()));
    }

    /// End to end through `/proc`: this live process is not gone, and a pid
    /// that cannot exist reads as gone.
    #[cfg(target_os = "linux")]
    #[test]
    fn proc_is_gone_matches_a_live_and_a_missing_pid() {
        let me = std::process::id() as pid_t;
        assert!(
            !proc_is_gone(me),
            "this test process is running, so it must not read as gone"
        );
        assert!(
            proc_is_gone(1_000_000_000),
            "a pid with no /proc entry must read as gone"
        );
    }
}
