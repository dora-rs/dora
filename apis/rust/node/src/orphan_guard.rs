//! Ending a node whose `dora run` parent was killed outright.
//!
//! Every cooperative teardown path — the `Stop` event, the daemon's
//! SIGTERM/SIGKILL ladder, the destroy reaper — needs the daemon to still be
//! running. `SIGKILL` to `dora run` leaves none of them: the signal is neither
//! catchable nor blockable, so no CLI- or daemon-side code executes afterwards.
//!
//! A node that polls its event stream still notices, because the daemon
//! connection hits EOF. A node that does not poll — one busy in a long
//! computation, or blocked on a device — notices nothing and runs forever with
//! `ppid 1`, holding whatever it held (dora-rs/dora#2856). Nodes are spawned as
//! process-group *leaders*, on purpose, so that a terminal `Ctrl-C` cannot kill
//! them out from under the daemon; the same property means neither inherited
//! signal delivery nor a group-kill of the CLI can reach the orphan.
//!
//! So the node watches for itself. The daemon hands it
//! [`DORA_RUN_PARENT_PID_ENV`] — and *only* when daemon and node-parent are the
//! same process, which is `dora run` and the other in-process
//! `Daemon::run_dataflow` callers. Once that pid is gone, this guard `SIGKILL`s
//! the node's own process group.
//!
//! # Why the whole group, not just this process
//!
//! The process the daemon tracks is frequently a wrapper: `uv run python
//! node.py`, `sh -c ...`, a console script. The real node is one level down, in
//! the wrapper's process group. Ending only the calling process would leave the
//! interpreter behind — the same orphan, one level down — so the guard signals
//! the group, exactly as the daemon's own reaper does.
//!
//! # Why not `PR_SET_PDEATHSIG`
//!
//! Linux can ask the kernel to signal a child when its parent dies, which
//! needs no thread and covers a node that is killed before it reaches `init`.
//! It is not enough on its own, and it is not free:
//!
//! - It reaches only the *direct* child. Under `--uv` that child is `uv run
//!   python ...` and the node is one level down, so the interpreter is left
//!   exactly as orphaned as before — the shape this issue was reported for.
//! - It fires when the parent **thread** exits, not the parent process. The
//!   daemon spawns from a tokio worker thread, so the guarantee is only as
//!   stable as which thread happened to run the spawn — a spawn moved behind
//!   `spawn_blocking` would start killing nodes early, silently.
//! - It is Linux-only, and the report is from macOS.
//!
//! It remains a reasonable *complement* for the pre-`init` window (see
//! "Coverage" below); it is not the mechanism.
//!
//! # Why `SIGKILL` rather than a `SIGTERM` grace period first
//!
//! By the time this fires the daemon is already gone: there is nothing to flush
//! outputs to, no coordinator to report to, and no one to answer a cooperative
//! stop. A grace period would also be self-defeating here — `SIGTERM` to the
//! group includes *this* process, so if it dies on the first rung nothing is
//! left to deliver the second one to a sibling that ignores `SIGTERM`.
//!
//! # Coverage
//!
//! A node is guarded from [`DoraNode::init`][crate::DoraNode::init] onwards, so
//! two gaps remain. A process killed *before* it gets there — a Python node
//! still in `import torch`, a `uv run` still resolving dependencies — is
//! orphaned as before. And a node that never calls `init` at all (a `path:
//! shell` command) is never guarded, because nothing of dora's runs in it.
//!
//! Unix only; on Windows this module compiles to a no-op and the gap remains.
//! Windows has no process groups in this sense, so a node cannot contain its
//! own tree the way `killpg` does here. The idiomatic equivalent is the job
//! object the daemon already spawns nodes into (`process_wrap`'s `JobObject`),
//! which ties the tree to the parent's handle only once
//! `JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE` is set — a daemon-side change that
//! `process-wrap` couples to its `KillOnDrop` wrapper, and therefore to
//! `Command::kill_on_drop`, altering when every node dies. That is not
//! verifiable from this workspace, so it is left to a follow-up rather than
//! shipped untested (dora-rs/dora#2856).

use dora_core::topics::DORA_RUN_PARENT_PID_ENV;
use std::sync::Once;

/// How often the parent is re-checked.
///
/// The parent is already dead when this matters, so the interval is pure
/// latency between the kill and the node's exit; it buys nothing to poll
/// faster, and one sleeping thread per node costs nothing to poll this often.
#[cfg(unix)]
const POLL_INTERVAL: std::time::Duration = std::time::Duration::from_millis(500);

/// One guard per process, so initializing several nodes (the operator runtime
/// does) does not accumulate threads all watching the same pid.
static ARMED: Once = Once::new();

/// Arm the guard if this node was spawned by `dora run`.
///
/// A no-op when [`DORA_RUN_PARENT_PID_ENV`] is absent, which is every other
/// way a node starts: `dora up` + `dora start` (where the node is *meant* to
/// outlive daemon restarts — dora-rs/dora#2029), a manually launched dynamic
/// node, interactive mode, and the integration-test harness.
pub(crate) fn arm_if_run_child() {
    let Ok(raw) = std::env::var(DORA_RUN_PARENT_PID_ENV) else {
        return;
    };
    let Ok(parent) = raw.trim().parse::<u32>() else {
        warn(&format!(
            "ignoring malformed {DORA_RUN_PARENT_PID_ENV}={raw:?}"
        ));
        return;
    };
    ARMED.call_once(|| arm(parent));
}

#[cfg(not(unix))]
fn arm(_parent: u32) {}

#[cfg(unix)]
fn arm(parent: u32) {
    let plan = ContainmentPlan::new(parent);
    // Checked before the thread starts, because the parent may ALREADY be gone:
    // it can be killed between forking this node and this node reaching
    // `init`, and a first check only after one sleep would be a window in which
    // the very orphan this exists to prevent is created.
    if plan.parent_is_gone() {
        plan.contain();
    }
    let spawned = std::thread::Builder::new()
        .name("dora-orphan-guard".into())
        .spawn(move || {
            loop {
                std::thread::sleep(POLL_INTERVAL);
                if plan.parent_is_gone() {
                    plan.contain();
                }
            }
        });
    if let Err(err) = spawned {
        // Not fatal: the node works, it just loses the guarantee. Say so, since
        // the symptom otherwise only appears much later as a stranded process.
        warn(&format!(
            "failed to start the orphan guard ({err}); this node may outlive a \
             killed `dora run`"
        ));
    }
}

/// What this node will do once `parent` is gone, decided while the parent is
/// still alive and therefore still inspectable.
#[cfg(unix)]
struct ContainmentPlan {
    parent: libc::pid_t,
    /// True when this process is the parent's direct child, which makes
    /// `getppid()` an exact liveness test — see [`Self::parent_is_gone`].
    direct_child: bool,
}

#[cfg(unix)]
impl ContainmentPlan {
    fn new(parent: u32) -> Self {
        let parent = parent as libc::pid_t;
        // SAFETY: reads this process's own parent id.
        let direct_child = unsafe { libc::getppid() } == parent;
        Self {
            parent,
            direct_child,
        }
    }

    /// Whether the process this node may not outlive has exited.
    ///
    /// Two checks because there are two shapes. A direct child can settle it
    /// exactly: the kernel reparents an orphan, so `getppid()` moving away from
    /// the recorded pid proves that pid is gone, whether or not the id has
    /// since been recycled.
    ///
    /// A node under a wrapper (`uv run python ...`) has the wrapper as its
    /// parent, not the CLI, so it has to ask about the pid directly. That check
    /// is one-sided: it can only be fooled by the OS handing the CLI's pid to
    /// an unrelated process within a poll interval, which reports the parent as
    /// alive and leaves the node running — today's behavior, never a wrong kill.
    fn parent_is_gone(&self) -> bool {
        if self.direct_child {
            // SAFETY: reads this process's own parent id.
            return unsafe { libc::getppid() } != self.parent;
        }
        // SAFETY: signal 0 delivers nothing; it is a pure existence check.
        let probe = unsafe { libc::kill(self.parent, 0) };
        // `EPERM` means the process exists but is not ours to signal, so only
        // an outright lookup failure counts as gone.
        probe != 0 && std::io::Error::last_os_error().raw_os_error() == Some(libc::ESRCH)
    }

    /// End this node, and everything it spawned alongside it.
    fn contain(&self) -> ! {
        // Logged before the kill, which lands on this process too.
        warn(&format!(
            "the `dora run` that spawned this node (pid {}) is gone; terminating",
            self.parent
        ));
        // The whole group, not just this process. Signalling only ourselves
        // would leave a wrapper — and anything else the node started — running:
        // the same orphan, one level down.
        //
        // The group is safe to signal without further checks *because* this
        // process was handed `DORA_RUN_PARENT_PID`: only the daemon sets it,
        // only on processes it spawns, and every unix spawn goes through
        // `ProcessGroup::leader()`, so the group exists solely for this node.
        //
        // SAFETY: signals this process's own group. Not a pid lookup, so
        // nothing here can land on a recycled id.
        unsafe { libc::killpg(libc::getpgrp(), libc::SIGKILL) };
        // Reached only if the signal did not land; this process must end anyway.
        //
        // `_exit`, not `std::process::exit`: this runs on a background thread
        // while the node's own threads keep going, and `exit` would run their
        // `atexit`/destructor work concurrently — which can block on a lock one
        // of those threads holds and leave the node alive after all. There is
        // nothing worth flushing; the daemon that owned the other end of this
        // node's pipes is gone.
        //
        // SAFETY: `_exit` ends the process; it touches no state of ours.
        unsafe { libc::_exit(1) }
    }
}

/// Report a guard problem without depending on a `tracing` subscriber being
/// installed (nodes commonly have none) and without `eprintln!`'s panic on a
/// broken stderr pipe.
fn warn(message: &str) {
    use std::io::Write as _;
    let _ = writeln!(std::io::stderr(), "dora: orphan guard: {message}");
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The environment variable is the entire gate: without it — the `dora up`
    /// case, a manually started dynamic node, interactive mode — nothing is
    /// armed, so nothing can kill a node that is supposed to outlive its
    /// daemon (dora-rs/dora#2029).
    #[test]
    fn nothing_is_armed_without_the_variable() {
        assert!(
            std::env::var_os(DORA_RUN_PARENT_PID_ENV).is_none(),
            "test environment must not set {DORA_RUN_PARENT_PID_ENV}"
        );
        arm_if_run_child();
        assert!(
            !ARMED.is_completed(),
            "a node with no `dora run` parent must not be armed"
        );
    }

    /// A live parent must never read as gone — the check that stands between
    /// this module and killing healthy nodes twice a second.
    ///
    /// Asserted on both shapes: this process is its runner's direct child, so
    /// the exact `getppid` branch applies, while a plan built for the runner's
    /// *own* parent exercises the `kill(pid, 0)` branch a wrapped node takes.
    #[cfg(unix)]
    #[test]
    fn a_live_parent_is_never_reported_gone() {
        // SAFETY: reads this process's own parent id.
        let runner = unsafe { libc::getppid() };
        let direct = ContainmentPlan::new(runner as u32);
        assert!(
            direct.direct_child,
            "the test binary is its runner's direct child"
        );
        assert!(
            !direct.parent_is_gone(),
            "the test runner is alive while it waits for this test"
        );

        // SAFETY: reads the runner's parent id; a failure yields -1, handled
        // by the `is_positive` guard below.
        let grandparent = unsafe { libc::getpgid(runner) };
        if grandparent.is_positive() && grandparent != runner {
            let wrapped = ContainmentPlan::new(grandparent as u32);
            assert!(
                !wrapped.direct_child,
                "the runner's own parent is not this process's parent"
            );
            assert!(
                !wrapped.parent_is_gone(),
                "a live process must not read as gone through the probe branch"
            );
        }
    }
}
