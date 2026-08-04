//! Reaping node processes when the daemon is torn down.
//!
//! The daemon's stop path is asynchronous: `stop_all` sends the cooperative
//! `Stop` and schedules the grace → SIGTERM → SIGKILL ladder on per-node
//! tasks. That is fine while the daemon keeps running, but `Destroy` means
//! the process is about to end — and once it does, those tasks are never
//! polled again. A node that ignored the cooperative `Stop` is then never
//! signalled at all and survives with `ppid 1`, holding whatever it held:
//! shared memory, GPU contexts, ports (dora-rs/dora#2980).
//!
//! So a destroy must not reply until the daemon's children are gone. The
//! waiting is deliberately *not* done inline in the event handler: a node
//! shutting down cooperatively finishes by asking the daemon to close its
//! outputs and report them done, and those requests are answered by the very
//! event loop that would be blocked. Waiting there deadlocks every
//! well-behaved node against the reaper until the deadline, turning a clean
//! exit into a signal kill. Instead the destroy is left pending, the loop
//! keeps serving nodes, and [`DestroyWait::poll`] is driven by a tick until
//! the nodes are gone or the deadline forces a kill.

use std::time::Duration;
use sysinfo::{Pid, ProcessRefreshKind, ProcessStatus, ProcessesToUpdate, System};
use tokio::time::Instant;

/// How long a destroy waits for nodes to exit before killing them.
///
/// Sized to outlast the stop ladder that the coordinator's `StopDataflow`
/// already scheduled (`DEFAULT_STOP_GRACE` for SIGTERM, half as much again
/// for SIGKILL), so a node that would have died on either rung dies *there*,
/// through the daemon's normal path, which signals the node's whole process
/// group. Killing earlier would cut short the SIGTERM window that nodes use
/// to flush. This deadline is the backstop for whatever that ladder does not
/// finish in time.
pub(crate) const DESTROY_WAIT: Duration = Duration::from_secs(16);

/// How long to wait for a killed process to disappear. SIGKILL cannot be
/// caught, so this only covers the kernel tearing the process down.
const KILL_SETTLE: Duration = Duration::from_secs(2);

/// How often liveness is re-checked while a destroy is pending.
pub(crate) const POLL_INTERVAL: Duration = Duration::from_millis(100);

/// A destroy that is waiting for this daemon's nodes to exit.
pub(crate) struct DestroyWait {
    /// When to stop waiting and kill what is left.
    deadline: Instant,
    /// Set once the kill has been issued; `deadline` then becomes the settle
    /// deadline, and the next expiry gives up rather than killing again.
    killed: bool,
    /// Pids that had to be killed, for the caller to report.
    pub killed_pids: Vec<u32>,
    system: System,
    own_pid: u32,
}

/// What [`DestroyWait::poll`] concluded.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum DestroyProgress {
    /// Nodes are still running; poll again.
    Waiting,
    /// No node processes are left — the daemon may exit.
    Done,
    /// Gave up on these pids: they survived the kill, or could not be
    /// killed. Exiting now orphans them, which is worth an error.
    Abandoned(Vec<u32>),
}

impl DestroyWait {
    pub fn new() -> Self {
        Self {
            deadline: Instant::now() + DESTROY_WAIT,
            killed: false,
            killed_pids: Vec::new(),
            system: System::new(),
            own_pid: std::process::id(),
        }
    }

    /// Re-check `pids` and act if the deadline has passed.
    ///
    /// `pids` is re-read from the daemon's state on every call rather than
    /// captured up front, so a node that respawns mid-teardown is followed to
    /// its new process instead of leaving the old pid to be chased.
    pub fn poll(&mut self, pids: &[u32]) -> DestroyProgress {
        let alive = self.live_children(pids);
        if alive.is_empty() {
            return DestroyProgress::Done;
        }
        if Instant::now() < self.deadline {
            return DestroyProgress::Waiting;
        }
        if self.killed {
            return DestroyProgress::Abandoned(alive);
        }

        for pid in &alive {
            if kill_process_group(*pid) {
                self.killed_pids.push(*pid);
            }
        }
        self.killed = true;
        self.deadline = Instant::now() + KILL_SETTLE;
        DestroyProgress::Waiting
    }

    /// The subset of `pids` that are still running children of this process.
    ///
    /// Parentage, not mere existence: once a node exits its pid is free for
    /// reuse, and killing blind would eventually hit an unrelated process.
    fn live_children(&mut self, pids: &[u32]) -> Vec<u32> {
        if pids.is_empty() {
            return Vec::new();
        }
        let lookup: Vec<Pid> = pids.iter().map(|pid| Pid::from_u32(*pid)).collect();
        self.system.refresh_processes_specifics(
            ProcessesToUpdate::Some(&lookup),
            true,
            ProcessRefreshKind::nothing(),
        );
        let own_pid = self.own_pid;
        let system = &self.system;
        pids.iter()
            .copied()
            .filter(|pid| {
                system
                    .process(Pid::from_u32(*pid))
                    .is_some_and(|process| is_live_child(process, own_pid))
            })
            .collect()
    }
}

/// Kill the node's whole process group, not just the pid the daemon tracks.
///
/// Nodes are spawned as process-group leaders, which is what lets the stop
/// ladder's signals reach whatever the node spawned. The tracked pid is
/// frequently a wrapper — `uv run python script.py`, `sh -c ...` — so
/// signalling it alone would leave the real node behind: the same orphan, one
/// level down.
fn kill_process_group(pid: u32) -> bool {
    #[cfg(unix)]
    {
        // SAFETY: `killpg`/`kill` on a pid this process spawned as a group
        // leader. A pid the OS recycled is already excluded by the parentage
        // check in `live_children`.
        let killed = unsafe { libc::killpg(pid as i32, libc::SIGKILL) } == 0;
        if killed {
            return true;
        }
        // Not a group leader after all (the node called `setsid`), or the
        // group is already gone: fall back to the process itself.
        unsafe { libc::kill(pid as i32, libc::SIGKILL) == 0 }
    }
    #[cfg(not(unix))]
    {
        // Windows: nodes are spawned into a job object, which terminates
        // their children along with them.
        let mut system = System::new();
        system.refresh_processes_specifics(
            ProcessesToUpdate::Some(&[Pid::from_u32(pid)]),
            true,
            ProcessRefreshKind::nothing(),
        );
        system
            .process(Pid::from_u32(pid))
            .is_some_and(|process| process.kill())
    }
}

/// A zombie has already exited (its supervision task just has not reaped it
/// yet), and a pid the OS recycled belongs to someone else.
fn is_live_child(process: &sysinfo::Process, own_pid: u32) -> bool {
    if process.status() == ProcessStatus::Zombie {
        return false;
    }
    process.parent() == Some(Pid::from_u32(own_pid))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::process::{Command, Stdio};

    fn spawn_sleeper() -> std::process::Child {
        Command::new("sleep")
            .arg("300")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("failed to spawn sleep")
    }

    /// A node that will not exit is killed once the deadline passes — the
    /// case #2980 is about.
    #[tokio::test(start_paused = true)]
    #[cfg_attr(not(unix), ignore = "spawns `sleep`")]
    async fn a_node_that_outlives_the_deadline_is_killed() {
        let mut child = spawn_sleeper();
        let pid = child.id();
        let mut wait = DestroyWait::new();

        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Waiting,
            "a live node before the deadline must be waited for, not killed"
        );

        tokio::time::advance(DESTROY_WAIT + Duration::from_secs(1)).await;
        assert_eq!(wait.poll(&[pid]), DestroyProgress::Waiting);
        assert_eq!(wait.killed_pids, vec![pid], "the straggler must be killed");

        let status = child.wait().expect("failed to wait for killed child");
        assert!(!status.success(), "killed process must not exit cleanly");
    }

    /// The common case must stay free: a node that exits on its own is never
    /// killed, and the destroy completes as soon as it is gone rather than
    /// sitting out the deadline.
    #[tokio::test(start_paused = true)]
    #[cfg_attr(not(unix), ignore = "spawns `sleep`")]
    async fn a_node_that_exits_on_its_own_is_not_killed() {
        let mut child = spawn_sleeper();
        let pid = child.id();
        let mut wait = DestroyWait::new();
        assert_eq!(wait.poll(&[pid]), DestroyProgress::Waiting);

        let _ = child.kill();
        let _ = child.wait();

        assert_eq!(
            wait.poll(&[pid]),
            DestroyProgress::Done,
            "the wait must end when the node does"
        );
        assert!(wait.killed_pids.is_empty(), "nothing to kill");
    }

    /// A pid the OS recycled after the node exited belongs to a stranger.
    /// Killing it would be worse than the leak this module exists to fix.
    #[tokio::test(start_paused = true)]
    #[cfg_attr(not(unix), ignore = "spawns `sleep`")]
    async fn a_process_that_is_not_our_child_is_left_alone() {
        let mut child = spawn_sleeper();
        let pid = child.id();

        let mut wait = DestroyWait::new();
        assert_eq!(wait.live_children(&[pid]), vec![pid]);

        let mut foreign = DestroyWait::new();
        foreign.own_pid = std::process::id() + 424_242;
        assert!(
            foreign.live_children(&[pid]).is_empty(),
            "a process parented elsewhere must not be treated as ours"
        );
        tokio::time::advance(DESTROY_WAIT + Duration::from_secs(1)).await;
        assert_eq!(foreign.poll(&[pid]), DestroyProgress::Done);
        assert!(
            foreign.killed_pids.is_empty(),
            "a stranger's pid must never be signalled"
        );

        let _ = child.kill();
        let _ = child.wait();
    }

    /// Nothing to wait for is not a reason to wait.
    #[tokio::test]
    async fn a_daemon_with_no_nodes_finishes_at_once() {
        assert_eq!(DestroyWait::new().poll(&[]), DestroyProgress::Done);
    }
}
